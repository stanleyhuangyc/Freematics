/******************************************************************************
* Traccar client for Freematics ONE w/ SIM5360 and Trackie
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

#include "FreematicsONE.h"

#define OPERATOR_APN "internet"
#define TRACCAR_HOST "trackie.freematics.com"
#define TRACCAR_PORT 5055
#define CONN_TIMEOUT 5000 /* ms */
#define SIM5360_BAUDRATE 115200L
#define ENABLE_STANDBY 0
#define VOLTAGE_IDLE 13.5f /* volts */
#define VOLTAGE_START 14 /* volts */

COBDSPI sys;

typedef enum {
    DISCONNECTED = 0,
    CONNECTED,
    ERROR,
    STANDBY,
} STATE;

class SIM5360 {
public:
  bool begin()
  {
    unsigned long t = millis();
    for (;;) {
      if (!sendCommand("AT")) sendCommand(0, 5000, "START");
      if (sendCommand("ATE0\r") && sendCommand("ATI\r", 1000, "IMEI:")) {
        char *p = strstr_P(buffer, PSTR("IMEI:"));
        if (p) {
          char *q = strchr(p, '\r');
          if (q) *q = 0;
          strncpy(IMEI, p + 6, sizeof(IMEI) - 1);
        }
        return true;
      }
      if (millis() - t >= 30000) break;
      sys.xbTogglePower();
    }
    return false;
  }
  void end()
  {
    sendCommand("AT+CPOF\r");
  }
  bool setup(const char* apn)
  {
    uint32_t t = millis();
    bool success = false;
    do {
      do {
        Serial.print('.');
        delay(1000);
        success = sendCommand("AT+CPSI?\r", 1000, "Online");
        if (success) {
          if (!strstr_P(buffer, PSTR("NO SERVICE")))
            break;
          success = false;
          if (strstr_P(buffer, PSTR("ERROR"))) break;
        } else {
          if (strstr_P(buffer, PSTR("Off"))) break;
        }
      } while (millis() - t < 30000);
      if (!success) break;

      success = false;
      do {
        if (sendCommand("AT+CREG?\r", 1000, "+CREG: 0,")) {
          char *p = strstr(buffer, "+CREG: 0,");
          success = (p && (*(p + 9) == '1' || *(p + 9) == '5'));
        }
      } while (!success && millis() - t < 30000);
      if (!success) break;

      success = false;
      do {
        if (sendCommand("AT+CGREG?\r",1000, "+CGREG: 0,")) {
          char *p = strstr(buffer, "+CGREG: 0,");
          success = (p && (*(p + 10) == '1' || *(p + 10) == '5'));
        }
      } while (!success && millis() - t < 30000);
      if (!success) break;

      if (*apn) {
        sprintf_P(buffer, PSTR("AT+CGSOCKCONT=1,\"IP\",\"%s\"\r"), apn);
        sendCommand(buffer);
      }

      sendCommand("AT+CSOCKSETPN=1\r");
      //sendCommand("AT+CSOCKAUTH=,,\"password\",\"password\"\r");
      sendCommand("AT+CIPMODE=0\r");
      sendCommand("AT+NETOPEN\r");
    } while(0);
    if (!success) Serial.println(buffer);
    return success;
  }
  bool startGPS()
  {
    // set GPS antenna voltage
    sendCommand("AT+CVAUXV=61\r");
    sendCommand("AT+CVAUXS=1\r");
    if (sendCommand("AT+CGPS?\r", 1000, "+CGPS: 1")) return true;
    delay(2000);
    return sendCommand("AT+CGPS=1\r");
  }
  long parseDegree(const char* s)
  {
    char *p;
    unsigned long left = atol(s);
    unsigned long tenk_minutes = (left % 100UL) * 100000UL;
    if ((p = strchr(s, '.')))
    {
      unsigned long mult = 10000;
      while (isdigit(*++p))
      {
        tenk_minutes += mult * (*p - '0');
        mult /= 10;
      }
    }
    return (left / 100) * 1000000 + tenk_minutes / 6;
  }
  bool loadGPSData(GPS_DATA& gd)
  {
    char *p;
    if (sendCommand("AT+CGPSINFO\r", 200, "+CGPSINFO:")) do {
      if (!(p = strchr(buffer, ':'))) break;
      if (*(++p) == ',') break;
      gd.lat = parseDegree(p);
      if (!(p = strchr(p, ','))) break;
      if (*(++p) == 'S') gd.lat = -gd.lat;
      if (!(p = strchr(p, ','))) break;
      gd.lng = parseDegree(++p);
      if (!(p = strchr(p, ','))) break;
      if (*(++p) == 'W') gd.lng = -gd.lng;
      if (!(p = strchr(p, ','))) break;
      gd.date = atol(++p);
      if (!(p = strchr(p, ','))) break;
      gd.time = atol(++p);
      if (!(p = strchr(p, ','))) break;
      gd.alt = atoi(++p);
      if (!(p = strchr(p, ','))) break;
      gd.speed = atof(++p) * 100;
      if (!(p = strchr(p, ','))) break;
      gd.heading = atoi(++p);
      return true;
    } while (0);
    return false;
  }
  const char* getIP()
  {
    uint32_t t = millis();
    char *ip = 0;
    do {
      if (sendCommand("AT+IPADDR\r", 5000)) {
        char *p = strstr(buffer, "+IPADDR:");
        if (p) {
          ip = p + 9;
          if (*ip != '0') {
            p = strchr(p, '\r');
            if (p) *p = 0;
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
          int csq = atoi(p + 2);
          if (csq != 99) {
            return csq * 2 - 113;
          }
        }
    }
    return 0;
  }
  bool getOperatorName()
  {
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
  bool httpOpen()
  {
    return sendCommand("AT+CHTTPSSTART\r", 3000);
  }
  bool httpClose()
  {
    state = DISCONNECTED;
    return sendCommand("AT+CHTTPSCLSE\r");
  }
  bool httpConnect(const char* host, unsigned int port)
  {
    sprintf(buffer, "AT+CHTTPSOPSE=\"%s\",%u,%c\r", host, port, port == 443 ? '2' : '1');
    //Serial.println(buffer);
    if (sendCommand(buffer, CONN_TIMEOUT)) {
      state = CONNECTED;
      return true;
    } else {
      state = ERROR;
      return false;
    }
  }
  bool httpSend(const char* payload, int payloadSize)
  {
    // issue HTTP send command
    sprintf(buffer, "AT+CHTTPSSEND=%u\r", payloadSize);
    if (!sendCommand(buffer, 100, ">")) {
      Serial.println(buffer);
      return false;
    }
    buffer[0] = 0;
    if (sendCommand(payload, 200)) {
      return true;
    }
    return false;
  }
  bool checkRecvEvent(unsigned int timeout)
  {
    return sendCommand(0, timeout, "+CHTTPS: RECV EVENT");
  }
  int httpReceive(char** payload)
  {
    int received = 0;
    /*
      +CHTTPSRECV:XX\r\n
      [XX bytes from server]\r\n
      \r\n+CHTTPSRECV: 0\r\n
    */
    if (sendCommand("AT+CHTTPSRECV=256\r", CONN_TIMEOUT, "\r\n+CHTTPSRECV: 0")) {
      char *p = strstr(buffer, "\r\n+CHTTPSRECV:");
      if (p) {
        p = strchr(p, ',');
        if (p) {
          received = atoi(p + 1);
          if (payload) {
            char *q = strchr(p, '\n');
            *payload = q ? (q + 1) : p;
            q = strstr(*payload, "\r\n+CHTTPSRECV:");
            if (q) *q = 0;
          }
        }
      }
    }
    if (received == 0) {
      state = ERROR;
    }
    return received;
  }
  bool sendCommand(const char* cmd, unsigned int timeout = 2000, const char* expected = "\r\nOK")
  {
    if (cmd) sys.xbWrite(cmd);
    buffer[0] = 0;
    return sys.xbReceive(buffer, sizeof(buffer), timeout, &expected, 1) != 0;
  }
  char buffer[320] = {0};
  char IMEI[16] = {0};
  STATE state = DISCONNECTED;
};

SIM5360 net;

void initSIM5360()
{
  for (;;) {
    // initialize SIM5360 module
    Serial.print("Init...");
    if (net.begin()) {
      Serial.println("OK");
      Serial.print("IMEI:");
      Serial.println(net.IMEI);
    } else {
      Serial.println("NO");
      continue;
    }

    Serial.print("Searching network...");
    if (net.setup(OPERATOR_APN)) {
      if (net.getOperatorName()) {
        Serial.println(net.buffer);
      } else {
        Serial.println("OK");
      }
      break;
    } else {
      Serial.println("NO");
    }
  }

  Serial.print("Starting GPS...");
  if (net.startGPS()) {
    Serial.println("OK");
  } else {
    Serial.println("NO");
  }

  Serial.print("IP address...");
  const char *ip = net.getIP();
  if (ip) {
    Serial.println(ip);
  } else {
    Serial.println("failed");
  }

  int signal = net.getSignal();
  if (signal > 0) {
    Serial.print("RSSI:");
    Serial.print((float)signal / 10, 1);
    Serial.println("dBm");
  }

  Serial.print("Init HTTP...");
  if (net.httpOpen() || net.httpClose()) {
    Serial.println("OK");
  } else {
    Serial.println("NO");
  }
}

void standby()
{
  Serial.println("Standby...");
  net.end();
  net.state = STANDBY;
}

void setup()
{
  Serial.begin(115200);
  sys.begin();
  sys.xbBegin(SIM5360_BAUDRATE);
  initSIM5360();
}

void loop()
{
  static unsigned long lastutc = 0;
  static long lastlat = 0, lastlng = 0;
  static int long lastspeed = 0;
  static byte retries = 0;

  if (net.state == STANDBY) {
    delay(3000);
    float v = sys.getVoltage();
    Serial.print(v);
    Serial.println('V');
    if (v < VOLTAGE_START) return;
    initSIM5360();
    net.state = DISCONNECTED;
  }

  // connect to HTTP server
  if (net.state != CONNECTED) {
    Serial.print("Connecting server...");
    if (!net.httpConnect(TRACCAR_HOST, TRACCAR_PORT)) {
      Serial.println();
      Serial.println(net.buffer);
      net.httpClose();
      if (++retries >= 10) {
        initSIM5360();
        retries = 0;
      }
      return;
    } else {
      Serial.println("OK");
    }
  }

  // load GPS data and wait for new one
  GPS_DATA gd = {0};
  if (!net.loadGPSData(gd) || gd.date == 0) {
    // possibly no GPS signal
#if ENABLE_STANDBY
    if (sys.getVoltage() < VOLTAGE_IDLE) standby();
#endif
    return;
  }

  if (gd.time == lastutc || (gd.lat == lastlat && gd.lng == lastlng)) {
    // no new GPS data
    delay(200);
    return;
  }

  // reduce data rate when speed is 0
  for (byte n = 0; (gd.speed == 0 && lastspeed == 0) && n < 10; n++) {
    delay(1000);
#if ENABLE_STANDBY
    if (sys.getVoltage() < VOLTAGE_IDLE) {
      standby();
      return;
    }
#endif
    // load new data
    net.loadGPSData(gd);
    lastspeed = gd.speed;
  }

  // generate HTTP request
  // refer to https://www.traccar.org/osmand for URL format
  char request[256];
  int len = sprintf(request, "GET /?id=%s&timestamp=%04u-%02u-%02uT%02u:%02u:%02uZ&lat=%d.%06lu&lon=%d.%06lu&altitude=%d&speed=%u.%02u&heading=%d HTTP/1.1\r\nConnection: keep-alive\r\nContent-length: 0\r\n\r\n",
    net.IMEI,
    (unsigned int)(gd.date % 100) + 2000, (unsigned int)(gd.date / 100) % 100, (unsigned int)(gd.date / 10000),
    (unsigned int)(gd.time / 10000), (unsigned int)(gd.time % 10000) / 100, (unsigned int)(gd.time % 100),
    (int)(gd.lat / 1000000), abs(gd.lat) % 1000000, (int)(gd.lng / 1000000), abs(gd.lng) % 1000000, gd.alt, gd.speed / 100, gd.speed % 100, gd.heading);

  Serial.println("[REQUEST]");
  Serial.print(request);

  // send HTTP request containing data
  if (!net.httpSend(request, len)) {
    Serial.println("Error sending data");                        
    net.httpClose();
  } else if (net.state == CONNECTED && net.checkRecvEvent(CONN_TIMEOUT)) {
    // get and output server response
    char *response;
    int bytes = net.httpReceive(&response);
    if (bytes > 0) {
      Serial.println("[RESPONSE]");
      Serial.print(response);
    }
    // keep processed data
    lastutc = gd.time;
    lastlat = gd.lat;
    lastlng = gd.lng;
    lastspeed = gd.speed;
  }
}