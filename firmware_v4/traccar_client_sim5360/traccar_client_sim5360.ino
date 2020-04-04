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

#include "FreematicsONE.h"

#define BEE_BAUDRATE 115200L

#define OPERATOR_APN "internet"
#define TRACCAR_HOST "trackie.freematics.com"
#define TRACCAR_PORT 5055
#define CONN_TIMEOUT 5000

COBDSPI sys;

typedef enum {
    HTTP_DISCONNECTED = 0,
    HTTP_CONNECTED,
    HTTP_ERROR,
} NET_STATES;

typedef enum {
  HTTP_GET = 0,
  HTTP_POST,
} HTTP_METHOD;

class SIM5360 {
public:
  bool init()
  {
    sys.xbBegin(BEE_BAUDRATE);
    unsigned long t = millis();
    for (;;) {
      sys.xbTogglePower();
      delay(3000);
      for (byte m = 0; m < 3; m++) {
        sendCommand("AT\r");
        if (sendCommand("ATE0\r") && sendCommand("ATI\r", 1000, "IMEI:")) {
          char *p = strstr_P(buffer, PSTR("IMEI:"));
          if (p) {
            char *q = strchr(p, '\r');
            if (q) *q = 0;
            strncpy(IMEI, p + 6, sizeof(IMEI) - 1);
          }
          return true;
        }
      }
      if (millis() - t >= 30000) break;
    }
    return false;
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
    if (sendCommand("AT+CGPS=1\r") || sendCommand("AT+CGPS?\r", 100, "+CGPS: 1")) {
      return true;
    }
    return false;
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
  bool getGPSInfo(GPS_DATA& gd)
  {
    char *p;
    if (sendCommand("AT+CGPSINFO\r", 100) && (p = strstr_P(buffer, PSTR("+CGPSINFO:")))) do {
      if (!(p = strchr(p, ':'))) break;
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
      if (sendCommand("AT+IPADDR\r", 5000, "\r\nOK", true)) {
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
  bool httpOpen()
  {
      return sendCommand("AT+CHTTPSSTART\r", 3000);
  }
  void httpClose()
  {
    sendCommand("AT+CHTTPSCLSE\r");
    m_state = HTTP_DISCONNECTED;
  }
  bool httpConnect(const char* host, unsigned int port)
  {
      sprintf(buffer, "AT+CHTTPSOPSE=\"%s\",%u,%c\r", host, port, port == 443 ? '2' : '1');
      //Serial.println(buffer);
      if (sendCommand(buffer, CONN_TIMEOUT)) {
        m_state = HTTP_CONNECTED;
        return true;
      } else {
        m_state = HTTP_ERROR;
        return false;
      }
  }
  bool httpSend(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload = 0, int payloadSize = 0)
  {
    unsigned int headerSize = genHttpHeader(method, path, keepAlive, payload, payloadSize);
    // issue HTTP send command
    sprintf(buffer, "AT+CHTTPSSEND=%u\r", headerSize + payloadSize);
    if (!sendCommand(buffer, 100, ">")) {
      Serial.println(buffer);
      return false;
    }
    // send HTTP header
    genHttpHeader(method, path, keepAlive, payload, payloadSize);
    sys.xbWrite(buffer);
    // send POST payload if any
    if (payload) sys.xbWrite(payload);
    buffer[0] = 0;
    if (sendCommand(0, 200)) {
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
        m_state = HTTP_ERROR;
      }
      return received;
  }
  bool sendCommand(const char* cmd, unsigned int timeout = 2000, const char* expected = "\r\nOK", bool terminated = false)
  {
    if (cmd) {
      sys.xbWrite(cmd);
    }
    buffer[0] = 0;
    byte ret = sys.xbReceive(buffer, sizeof(buffer), timeout, &expected, 1);
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
  char IMEI[16] = {0};
private:
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
  byte m_state = HTTP_DISCONNECTED;
};

SIM5360 net;

void initSIM5360()
{
  for (;;) {
    // initialize SIM5360 xBee module (if present)
    Serial.print("Init...");
    if (net.init()) {
      Serial.println("OK");
      Serial.print("IMEI:");
      Serial.println(net.IMEI);
    } else {
      Serial.println("NO");
      continue;
    }

    Serial.print("Searching network");
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

  net.startGPS();

  Serial.print("IP address...");
  const char *ip = net.getIP();
  if (ip) {
    Serial.print(ip);
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
  if (net.httpOpen()) {
    Serial.println("OK");
  } else {
    Serial.println("NO");
  }
}

void setup()
{
  Serial.begin(115200);
  sys.begin();
  initSIM5360();
}

void loop()
{
  static unsigned long lastutc = 0;
  static long lastlat = 0, lastlng = 0;
  static byte retries = 0;

  // connect to HTTP server
  if (net.state() != HTTP_CONNECTED) {
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

  GPS_DATA gd = {0};
  if (!net.getGPSInfo(gd) || gd.time == lastutc || gd.date == 0 || (gd.lat == lastlat && gd.lng == lastlng)) {
    delay(200);
    return;
  }

  // arrange and send data in OsmAnd protocol
  // refer to https://www.traccar.org/osmand
  char buf[128];
  sprintf(buf, "/?id=%s&timestamp=%04u-%02u-%02uT%02u:%02u:%02uZ&lat=%d.%06lu&lon=%d.%06lu&altitude=%d&speed=%u.%02u&heading=%d",
    net.IMEI,
    (unsigned int)(gd.date % 100) + 2000, (unsigned int)(gd.date / 100) % 100, (unsigned int)(gd.date / 10000),
    (unsigned int)(gd.time / 10000), (unsigned int)(gd.time % 10000) / 100, (unsigned int)(gd.time % 100),
    (int)(gd.lat / 1000000), abs(gd.lat) % 1000000, (int)(gd.lng / 1000000), abs(gd.lng) % 1000000, gd.alt, gd.speed / 100, gd.speed % 100, gd.heading);
  Serial.println(buf + 2);

  if (!net.httpSend(HTTP_GET, buf, true)) {
    Serial.println("Error sending data");                        
    net.httpClose();
  } else if (net.state() == HTTP_CONNECTED && net.checkRecvEvent(CONN_TIMEOUT)) {
    // get and output server response
    char *response;
    int bytes = net.httpReceive(&response);
    if (bytes > 0) {
      Serial.println(response);
    }
    // keep processed data
    lastutc = gd.time;
    lastlat = gd.lat;
    lastlng = gd.lng;
  }
}