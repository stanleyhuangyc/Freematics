/******************************************************************************
* Sample testing sketch for SIM5360 (WCDMA/GSM) module in Freematics ONE+
* Developed by Stanley Huang https://www.facebook.com/stanleyhuangyc
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one for hardware information
* To obtain your Freematics Hub server key, contact support@freematics.com.au
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
******************************************************************************/

#include <FreematicsPlus.h>

#define APN "connect"
#define HTTP_SERVER_URL "hub.freematics.com"
#define HTTP_SERVER_PORT 80
#define UDP_SERVER_IP "47.74.68.134"
#define UDP_SERVER_PORT 8081
#define MAX_CONN_TIME 5000
#define XBEE_BAUDRATE 115200

typedef enum {
    NET_DISCONNECTED = 0,
    NET_CONNECTED,
    NET_HTTP_ERROR,
} NET_STATES;

typedef enum {
  HTTP_GET = 0,
  HTTP_POST,
} HTTP_METHOD;

FreematicsESP32 sys;

class SIM5360 {
public:
    SIM5360() { buffer[0] = 0; }
    bool init()
    {
      if (sendCommand("ATI\r", 50, "SIM5360")) {
        return true;
      }
      for (byte n = 0; n < 10; n++) {
        // try turning on module
        sys.xbTogglePower();
        delay(2000);
        // discard any stale data
        sys.xbPurge();
        for (byte m = 0; m < 3; m++) {
          if (sendCommand("ATI\r", 500, "SIM5360"))
            return true;
        }
      }
      return false;
    }
    bool setup(const char* apn, bool only3G = false, bool roaming = false)
    {
      uint32_t t = millis();
      bool success = false;
      sendCommand("ATE0\r");
      if (only3G) sendCommand("AT+CNMP=14\r"); // use WCDMA only
      do {
        do {
          Serial.print('.');
          delay(500);
          success = sendCommand("AT+CPSI?\r", 1000, "Online");
          if (success) {
            if (!strstr_P(buffer, PSTR("NO SERVICE")))
              break;
            success = false;
          } else {
            if (strstr_P(buffer, PSTR("Off"))) break;
          }
        } while (millis() - t < 60000);
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
          sprintf_P(buffer, PSTR("AT+CGSOCKCONT=1,\"IP\",\"%s\"\r"), apn);
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
      sprintf_P(buffer, PSTR("AT+CIPSEND=0,%u,\"%s\",%u\r"), len, ip, port);
      if (sendCommand(buffer, 100, ">")) {
        sys.xbWrite(data, len);
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
    }
    bool httpConnect()
    {
        sprintf_P(buffer, PSTR("AT+CHTTPSOPSE=\"%s\",%u,1\r"), HTTP_SERVER_URL, HTTP_SERVER_PORT);
        //Serial.println(buffer);
	      return sendCommand(buffer, MAX_CONN_TIME);
    }
    unsigned int genHttpHeader(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload, int payloadSize)
    {
        // generate HTTP header
        char *p = buffer;
        p += sprintf_P(p, PSTR("%s %s HTTP/1.1\r\nUser-Agent: ONE\r\nHost: %s\r\nConnection: %s\r\n"),
          method == HTTP_GET ? "GET" : "POST", path, HTTP_SERVER_URL, keepAlive ? "keep-alive" : "close");
        if (method == HTTP_POST) {
          p += sprintf_P(p, PSTR("Content-length: %u\r\n"), payloadSize);
        }
        p += sprintf_P(p, PSTR("\r\n\r"));
        return (unsigned int)(p - buffer);
    }
    bool httpSend(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload = 0, int payloadSize = 0)
    {
      unsigned int headerSize = genHttpHeader(method, path, keepAlive, payload, payloadSize);
      // issue HTTP send command
      sprintf_P(buffer, PSTR("AT+CHTTPSSEND=%u\r"), headerSize + payloadSize);
      if (!sendCommand(buffer, 100, ">")) {
        Serial.println(buffer);
        Serial.println("Connection closed");
      }
      // send HTTP header
      genHttpHeader(method, path, keepAlive, payload, payloadSize);
      sys.xbWrite(buffer);
      // send POST payload if any
      if (payload) sys.xbWrite(payload);
      buffer[0] = 0;
      if (sendCommand("AT+CHTTPSSEND\r")) {
        checkTimer = millis();
        return true;
      } else {
        Serial.println(buffer);
        return false;
      }
    }
    int httpReceive(char** payload)
    {
        int received = 0;
        // wait for RECV EVENT
        checkbuffer("RECV EVENT", MAX_CONN_TIME);
        /*
          +CHTTPSRECV:XX\r\n
          [XX bytes from server]\r\n
          \r\n+CHTTPSRECV: 0\r\n
        */
        if (sendCommand("AT+CHTTPSRECV=384\r", MAX_CONN_TIME, "\r\n+CHTTPSRECV: 0", true)) {
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
        return received;
    }
    byte checkbuffer(const char* expected, unsigned int timeout = 2000)
    {
      // check if expected string is in reception buffer
      if (strstr(buffer, expected)) {
        return 1;
      }
      // if not, receive a chunk of data from xBee module and look for expected string
      byte ret = sys.xbReceive(buffer, sizeof(buffer), timeout, &expected, 1) != 0;
      if (ret == 0) {
        // timeout
        return (millis() - checkTimer < timeout) ? 0 : 2;
      } else {
        return ret;
      }
    }
    bool sendCommand(const char* cmd, unsigned int timeout = 2000, const char* expected = "\r\nOK\r\n", bool terminated = false)
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
    char buffer[384];
private:
    uint32_t checkTimer;
};

SIM5360 net;
byte netState = NET_DISCONNECTED;
byte errors = 0;

void setup()
{
    Serial.begin(115200);
    delay(500);
    // this will init SPI communication

    // initialize SIM5360 xBee module (if present)
    Serial.print("Init SIM5360...");
    sys.xbBegin(XBEE_BAUDRATE);
    if (net.init()) {
      Serial.println("OK");
    } else {
      Serial.println("NO");
      for (;;);
    }
    Serial.println(net.buffer);

    Serial.print("Connecting network");
    if (net.setup(APN, true)) {
      Serial.println("OK");
    } else {
      Serial.println("NO");
      for (;;);
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

    Serial.print("Init UDP...");
    Serial.println(net.udpInit() ? "OK" : "NO");

    Serial.print("Init HTTP...");
    if (net.httpOpen()) {
      Serial.println("OK");
    } else {
      Serial.println("NO");
    }
}

void loop()
{
  if (errors > 0) {
    net.httpClose();
    netState = NET_DISCONNECTED;
    if (errors > 3) {
      // re-initialize 3G module
      setup();
      errors = 0;
    }
  }

  // send and receive UDP datagram
  Serial.print("Sending UDP datagram...");
  uint32_t t = millis();
  char buf[16];
  int len = sprintf(buf, "%lu", t);
  if (net.udpSend(UDP_SERVER_IP, UDP_SERVER_PORT, buf, len))
    Serial.println("OK");
  else
    Serial.println("failed");
  char *data = net.udpReceive(&len);
  if (data) {
    Serial.print(len);
    Serial.print(" bytes UDP datagram received [");
    Serial.print(data);
    Serial.print("] in ");
    Serial.print(millis() - t);
    Serial.println("ms");
  }

  // connect to HTTP server
  if (netState != NET_CONNECTED) {
    Serial.println("Connecting...");
    sys.xbPurge();
    if (!net.httpConnect()) {
      Serial.println("Error connecting");
      Serial.println(net.buffer);
      errors++;
      return;
    }
  }

  // send HTTP request
  Serial.print("Sending HTTP request...");
  if (!net.httpSend(HTTP_GET, "/test", true)) {
    Serial.println("failed");
    net.httpClose();
    errors++;
    netState = NET_DISCONNECTED;
    return;
  } else {
    Serial.println("OK");
  }

  Serial.print("Receiving...");
  char *payload;
  if (net.httpReceive(&payload)) {
    Serial.println("OK");
    Serial.println("-----HTTP RESPONSE-----");
    Serial.println(payload);
    Serial.println("-----------------------");
    netState = NET_CONNECTED;
    errors = 0;
  } else {
    Serial.println("failed");
    errors++;
  }

  Serial.println("Waiting 3 seconds...");
  delay(3000);
}
