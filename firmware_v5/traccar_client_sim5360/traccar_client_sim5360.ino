/******************************************************************************
* Traccar client for Freematics ONE+ and Freematics Esprit w/ SIM5360
* Written by Stanley Huang <stanley@freematics.com.au>
* Distributed under BSD license
* Visit https://freematics.com/products for hardware information
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
******************************************************************************/

#include <Arduino.h>
#include <TinyGPS.h>

#define PIN_LED 4
#define PIN_GPS_POWER 15
#define PIN_GPS_UART_RXD 32
#define PIN_GPS_UART_TXD 33
#define GPS_BAUDRATE 115200

#define PIN_SIM_POWER 27
#define PIN_SIM_UART_RXD 16
#define PIN_SIM_UART_TXD 17
#define BEE_BAUDRATE 115200L

#define OPERATOR_APN "mdata.net.au"
#define TRACCAR_HOST "YOUR_TRACCAR_SERVER"
#define TRACCAR_PORT 5055
#define TRACCAR_DEV_ID "YOUR_DEV_ID"
#define CONN_TIMEOUT 5000

HardwareSerial xbSerial(1);
HardwareSerial gpsSerial(2);
TinyGPS gps;

typedef enum {
    CLIENT_DISCONNECTED = 0,
    CLIENT_CONNECTED,
    CLIENT_ERROR,
} NET_STATES;

typedef enum {
  HTTP_GET = 0,
  HTTP_POST,
} HTTP_METHOD;

class SIM5360 {
public:
  bool init()
  {
    xbBegin(BEE_BAUDRATE);
    for (byte n = 0; n < 10; n++) {
      // try turning on module
      xbTogglePower();
      delay(2000);
      xbPurge();
      for (byte m = 0; m < 5; m++) {
        if (sendCommand("AT\r")) {
          return true;
        }
      }
    }
    return false;
  }
  bool setup(const char* apn, bool only3G = false, bool roaming = false)
  {
    uint32_t t = millis();
    bool success = false;
    if (!sendCommand("ATE0\r")) return false;
    if (only3G) sendCommand("AT+CNMP=14\r"); // use WCDMA only
    do {
      do {
        Serial.print('.');
        delay(500);
        success = sendCommand("AT+CPSI?\r", 1000, "Online");
        if (success) {
          if (!strstr(buffer, "NO SERVICE"))
            break;
          success = false;
        }
        if (strstr(buffer, "Off")) break;
      } while (millis() - t < 30000);
      if (!success) break;

      t = millis();
      do {
        success = sendCommand("AT+CREG?\r", 5000, roaming ? "+CREG: 0,5" : "+CREG: 0,1");
      } while (!success && millis() - t < 30000);
      if (!success) break;

      do {
        success = sendCommand("AT+CGREG?\r",1000, roaming ? "+CGREG: 0,5" : "+CGREG: 0,1");
      } while (!success && millis() - t < 30000);
      if (!success) break;

      do {
        sprintf(buffer, "AT+CGSOCKCONT=1,\"IP\",\"%s\"\r", apn);
        success = sendCommand(buffer);
      } while (!success && millis() - t < 30000);
      if (!success) break;

      //sendCommand("AT+CSOCKAUTH=1,1,\"APN_PASSWORD\",\"APN_USERNAME\"\r");

      success = sendCommand("AT+CSOCKSETPN=1\r");
      if (!success) break;

      success = sendCommand("AT+CIPMODE=0\r");
      if (!success) break;

      sendCommand("AT+NETOPEN\r");
    } while(0);
    if (!success) Serial.println(buffer);
    return success;
  }
  bool initGPS()
  {
    for (;;) {
      Serial.println("INIT GPS");
      if (sendCommand("AT+CGPS=1\r")) break;
      Serial.println(buffer);
      sendCommand("AT+CGPS=0\r");
      delay(3000);
    }

    Serial.println(buffer);

    for (;;) {
      sendCommand("AT+CGPSINFO\r");
      Serial.println(buffer);
      delay(3000);
    }
    return true;
  }
  const char* getIP()
  {
    uint32_t t = millis();
    char *ip = 0;
    do {
      if (sendCommand("AT+IPADDR\r", 5000, "\r\nOK\r\n", true)) {
        char *p = strstr(buffer, "+IPADDR:");
        if (p) {
          ip = p + 9;
          if (*ip != '0') {
            break;
          }
        }
      }
      delay(500);
      ip = 0;
    } while (millis() - t < 15000);
    return ip;
  }
  int getSignal()
  {
      if (sendCommand("AT+CSQ\r", 500)) {
          char *p = strchr(buffer, ':');
          if (p) {
            p += 2;
            int db = atoi(p) * 10;
            p = strchr(p, '.');
            if (p) db += *(p + 1) - '0';
            return db;
          }
      }
      return -1;
  }
  bool getOperatorName()
  {
      // display operator name
      if (sendCommand("AT+COPS?\r") == 1) {
          char *p = strstr(buffer, ",\"");
          if (p) {
              p += 2;
              char *s = strchr(p, '\"');
              if (s) *s = 0;
              strcpy(buffer, p);
              return true;
          }
      }
      return false;
  }
  bool udpInit()
  {
    return sendCommand("AT+CIPOPEN=0,\"UDP\",,,8000\r", 3000);
  }
  bool udpSend(const char* ip, uint16_t port, const char* data, unsigned int len)
  {
    sprintf(buffer, "AT+CIPSEND=0,%u,\"%s\",%u\r", len, ip, port);
    if (sendCommand(buffer, 100, ">")) {
      xbWrite((uint8_t*)data, len);
      return sendCommand(0, 1000);
    } else {
      Serial.println(buffer);
    }
    return false;
  }
  char* udpReceive(int* pbytes = 0)
  {
    if (sendCommand(0, 3000, "+IPD")) {
      char *p = strstr(buffer, "+IPD");
      if (!p) return 0;
      int len = atoi(p + 4);
      if (pbytes) *pbytes = len;
      p = strchr(p, '\n');
      if (p) {
        *(++p + len) = 0;
        return p;
      }
    }
    return 0;
  }
  bool httpOpen()
  {
      return sendCommand("AT+CHTTPSSTART\r", 3000);
  }
  void httpClose()
  {
    sendCommand("AT+CHTTPSCLSE\r");
    m_state = CLIENT_DISCONNECTED;
  }
  bool httpConnect(const char* host, unsigned int port)
  {
      sprintf(buffer, "AT+CHTTPSOPSE=\"%s\",%u,1\r", host, port);
      //Serial.println(buffer);
      if (sendCommand(buffer, CONN_TIMEOUT)) {
        m_state = CLIENT_CONNECTED;
        return true;
      } else {
        m_state = CLIENT_ERROR;
        return false;
      }
  }
  unsigned int genHttpHeader(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload, int payloadSize)
  {
      // generate HTTP header
      char *p = buffer;
      p += sprintf(p, "%s %s HTTP/1.1\r\nConnection: %s\r\n",
        method == HTTP_GET ? "GET" : "POST", path, keepAlive ? "keep-alive" : "close");
      if (method == HTTP_POST) {
        p += sprintf(p, "Content-length: %u\r\n", payloadSize);
      }
      p += sprintf(p, "\r\n\r");
      return (unsigned int)(p - buffer);
  }
  bool httpSend(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload = 0, int payloadSize = 0)
  {
    unsigned int headerSize = genHttpHeader(method, path, keepAlive, payload, payloadSize);
    // issue HTTP send command
    sprintf(buffer, "AT+CHTTPSSEND=%u\r", headerSize + payloadSize);
    if (!sendCommand(buffer, 100, ">")) {
      Serial.println(buffer);
      Serial.println("Connection closed");
    }
    // send HTTP header
    genHttpHeader(method, path, keepAlive, payload, payloadSize);
    xbWrite(buffer);
    // send POST payload if any
    if (payload) xbWrite(payload);
    buffer[0] = 0;
    if (sendCommand("AT+CHTTPSSEND\r")) {
      return true;
    } else {
      Serial.println(buffer);
      m_state = CLIENT_ERROR;
      return false;
    }
  }
  bool checkRecvEvent()
  {
    const char* expected = "RECV EVENT";
    if (strstr(buffer, expected)) {
      return true;
    }
    return xbRead(buffer, sizeof(buffer), 0) > 0 && strstr(buffer, expected) != 0;
  }
  int httpReceive(char** payload)
  {
      int received = 0;
      /*
        +CHTTPSRECV:XX\r\n
        [XX bytes from server]\r\n
        \r\n+CHTTPSRECV: 0\r\n
      */
      if (sendCommand("AT+CHTTPSRECV=384\r", CONN_TIMEOUT, "\r\n+CHTTPSRECV: 0", true)) {
        char *p = strstr(buffer, "+CHTTPSRECV:");
        if (p) {
          p = strchr(p, ',');
          if (p) {
            received = atoi(p + 1);
            if (payload) {
              char *q = strchr(p, '\n');
              *payload = q ? (q + 1) : p;
            }
          }
        }
      }
      if (received == 0) {
        m_state = CLIENT_ERROR;
      }
      return received;
  }
  bool sendCommand(const char* cmd, unsigned int timeout = 2000, const char* expected = "\r\nOK\r\n", bool terminated = false)
  {
    if (cmd) {
      xbWrite(cmd);
    }
    buffer[0] = 0;
    byte ret = xbReceive(buffer, sizeof(buffer), timeout, &expected, 1);
    if (ret) {
      if (terminated) {
        char *p = strstr(buffer, expected);
        if (p) *p = 0;
      }
      return true;
    } else {
      return false;
    }
  }
  byte state() { return m_state; }
  char buffer[384] = {0};
private:
  int dumpLine(char* buffer, int len)
  {
    int bytesToDump = len >> 1;
    for (int i = 0; i < len; i++) {
      // find out first line end and discard the first line
      if (buffer[i] == '\r' || buffer[i] == '\n') {
        // go through all following \r or \n if any
        while (++i < len && (buffer[i] == '\r' || buffer[i] == '\n'));
        bytesToDump = i;
        break;
      }
    }
    memmove(buffer, buffer + bytesToDump, len - bytesToDump);
    return bytesToDump;
  }
  void xbTogglePower()
  {
    digitalWrite(PIN_SIM_POWER, HIGH);
    delay(50);
    digitalWrite(PIN_SIM_POWER, LOW);
    delay(2000);
    digitalWrite(PIN_SIM_POWER, HIGH);
    delay(1000);
    digitalWrite(PIN_SIM_POWER, LOW);
  }
  void xbPurge()
  {
    xbSerial.flush();
  }
  bool xbBegin(unsigned long baudrate)
  {
    pinMode(PIN_SIM_POWER, OUTPUT);
    digitalWrite(PIN_SIM_POWER, HIGH);
    xbSerial.begin(BEE_BAUDRATE, SERIAL_8N1, PIN_SIM_UART_RXD, PIN_SIM_UART_TXD);
    return true;
  }
  void xbWrite(const char* cmd)
  {
    xbSerial.print(cmd);
  }
  void xbWrite(uint8_t* data, int bytes)
  {
    xbSerial.write(data, bytes);
  }
  int xbRead(char* buffer, int bufsize, unsigned int timeout)
  {
    int n = 0;
    uint32_t t = millis();
    do {
      while (xbSerial.available() && n < bufsize - 1) {
        buffer[n++] = xbSerial.read();
      }
    } while (millis() - t <= timeout);
    buffer[n] = 0;
    return n;
  }
  int xbReceive(char* buffer, int bufsize, unsigned int timeout, const char** expected, byte expectedCount)
  {
    int bytesRecv = 0;
    uint32_t t = millis();
    do {
      if (bytesRecv >= bufsize - 16) {
        bytesRecv -= dumpLine(buffer, bytesRecv);
      }
      int n = xbRead(buffer + bytesRecv, bufsize - bytesRecv - 1, 100);
      if (n > 0) {
        bytesRecv += n;
        buffer[bytesRecv] = 0;
        for (byte i = 0; i < expectedCount; i++) {
          // match expected string(s)
          if (expected[i] && strstr(buffer, expected[i])) return i + 1;
        }
      } else if (n == -1) {
        // an erroneous reading
        break;
      }
    } while (millis() - t < timeout);
    buffer[bytesRecv] = 0;
    return 0;
  }  
  byte m_state = CLIENT_DISCONNECTED;
};

SIM5360 net;

void initSIM5360()
{
  for (;;) {
    // initialize SIM5360 xBee module (if present)
    Serial.print("Init SIM5360...");
    if (net.init()) {
      Serial.println("OK");
    } else {
      Serial.println("NO");
      for (;;);
    }

    Serial.print("Connecting network (APN:");
    Serial.print(OPERATOR_APN);
    Serial.print(')');
    if (net.setup(OPERATOR_APN, true)) {
      Serial.println("OK");
      break;
    } else {
      Serial.println("NO");
    }
  }

  if (net.getOperatorName()) {
    Serial.print("Operator:");
    Serial.println(net.buffer);
  }

  Serial.print("Obtaining IP address...");
  const char *ip = net.getIP();
  if (ip) {
    Serial.print(ip);
  } else {
    Serial.println("failed");
  }

  int signal = net.getSignal();
  if (signal > 0) {
    Serial.print("CSQ:");
    Serial.print((float)signal / 10, 1);
    Serial.println("dB");
  }

  Serial.print("Init HTTP...");
  if (net.httpOpen()) {
    Serial.println("OK");
  } else {
    Serial.println("NO");
  }
}

void setup()
{
  // turn on indicator LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  // USB serial
  Serial.begin(115200);
  delay(1000);
  Serial.println("TRACCAR CLIENT");

  initSIM5360();

  // turn on GPS power
  pinMode(PIN_GPS_POWER, OUTPUT);
  digitalWrite(PIN_GPS_POWER, HIGH);

  // start serial UART where GPS receiver is connected
  gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, PIN_GPS_UART_RXD, PIN_GPS_UART_TXD);
  Serial.print("Waiting for GPS signal ");

  // turn off indicator LED
  digitalWrite(PIN_LED, LOW);
}

void loop()
{
  static unsigned long lastutc = 0;
  static int retries = 0;

  // check incoming NMEA data from GPS
  if (!gpsSerial.available()) {
    return;
  }

  // read a character of NMEA stream from GPS
  char c = gpsSerial.read();
  if (lastutc == 0) {
    // display progress before GPS has signal
    if (c == '\n') Serial.write('.');
    //Serial.write(c);
  }

  // decode NMEA stream
  if (!gps.encode(c)) {
    return;
  }

  // connect to HTTP server
  if (net.state() != CLIENT_CONNECTED) {
    Serial.println("Connecting...");
    if (!net.httpConnect(TRACCAR_HOST, TRACCAR_PORT)) {
      Serial.println(net.buffer);
      if (++retries >= 3) {
        initSIM5360();
        retries = 0;
      }
      return;
    }
  }

  // check UTC timestamp from GPS
  unsigned long utcdate, utctime;
  gps.get_datetime(&utcdate, &utctime, 0);
  if (utctime == lastutc) {
    return;
  }

  // turn on  indicator LED
  digitalWrite(PIN_LED, HIGH);

  // now that new GPS coordinates are available
  long lat, lng;
  gps.get_position(&lat, &lng, 0);
  long speed = gps.speed();
  long alt = gps.altitude();
  int sats = gps.satellites();
  int heading = gps.course();
  // generate ISO time string
  char isotime[24];
  sprintf(isotime, "%04u-%02u-%02uT%02u:%02u:%02u.%01uZ",
    (unsigned int)(utcdate % 100) + 2000, (unsigned int)(utcdate / 100) % 100, (unsigned int)(utcdate / 10000),
    (unsigned int)(utctime / 1000000), (unsigned int)(utctime % 1000000) / 10000, (unsigned int)(utctime % 10000) / 100, ((unsigned int)utctime % 100) / 10);

  // display GPS UTC time and coordinates
  Serial.print(isotime);
  Serial.print(" LAT:");
  Serial.print((float)lat / 1000000, 6);
  Serial.print(" LNG:");
  Serial.print((float)lng / 1000000, 6);
  Serial.print(" Speed:");
  Serial.print((int)speed * 1852 / 100000);
  Serial.print("km/h Alt:");
  Serial.print(alt / 100);
  Serial.print("m Sats:");
  Serial.println(sats);

  // arrange and send data in OsmAnd protocol
  // refer to https://www.traccar.org/osmand
  char url[256];
  sprintf(url, "/?id=%s&timestamp=%s&lat=%f&lon=%f&altitude=%d&speed=%f&heading=%d",
    TRACCAR_DEV_ID, isotime,
    (float)lat / 1000000, (float)lng / 1000000, (int)(alt / 100), (float)speed / 100, heading / 100);

  if (!net.httpSend(HTTP_GET, url, true)) {
    Serial.println("Error sending data");
    net.httpClose();
  }

  // output server response
  if (net.state() == CLIENT_CONNECTED) {
    // waiting for server response while still decoding NMEA
    for (uint32_t t = millis(); !net.checkRecvEvent() && millis() - t < 3000; ) {
      if (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
      }
    }

    char *response;
    int bytes = net.httpReceive(&response);
    if (bytes > 0) {
      Serial.println(response);
    }
  }

  // keep processed UTC timestamp
  lastutc = utctime;

  // turn off indicator LED
  digitalWrite(PIN_LED, LOW);
}