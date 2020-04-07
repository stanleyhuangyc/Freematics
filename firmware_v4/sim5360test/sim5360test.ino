/******************************************************************************
  Sample testing sketch for SIM5360 (WCDMA/GSM) module in Freematics ONE
  Developed by Stanley Huang, distributed under BSD license
  Visit http://freematics.com/products/freematics-one for hardware information
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
******************************************************************************/

#include <FreematicsONE.h>

// testing URL: https://hub.freematics.com/test
#define HTTP_SERVER_URL "hub.freematics.com"
#define HTTP_SERVER_PORT 443
#define HTTP_SERVER_PATH "/test"
#define APN "hologram"
#define MAX_CONN_TIME 10000
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

class CSIM5360 : public COBDSPI {
  public:
    CSIM5360() {
      buffer[0] = 0;
    }
    bool netInit()
    {
      for (byte n = 0; n < 3; n++) {
        // try turning on module
        xbTogglePower();
        delay(3000);
        // discard any stale data
        xbPurge();
        for (byte m = 0; m < 3; m++) {
          if (netSendCommand("AT\r"))
            return true;
        }
      }
      return false;
    }
    bool netSetup(const char* apn, bool only3G = false)
    {
      uint32_t t = millis();
      bool success = false;
      netSendCommand("ATE0\r");
      if (only3G) netSendCommand("AT+CNMP=14\r"); // use WCDMA only
      do {
        do {
          Serial.print('.');
          success = netSendCommand("AT+CPSI?\r", 1000, "Online");
          if (success) {
            if (!strstr(buffer, "NO SERVICE")) success = false;
            break;
          } else {
            if (strstr(buffer, "Off")) break;
          }
        } while (millis() - t < 30000);
        if (!success) break;

        t = millis();
        do {
          delay(1000);
          Serial.print('.');
          if (netSendCommand("AT+CREG?\r", 1000, "+CREG: 0,")) {
            char *p = strstr(buffer, "+CREG: 0,");
            success = (p && (*(p + 9) == '1' || *(p + 9) == '5'));
          }
        } while (!success && millis() - t < 30000);
        if (!success) break;

        do {
          delay(1000);
          Serial.print('.');
          if (netSendCommand("AT+CGREG?\r", 1000, "+CGREG: 0,")) {
            char *p = strstr(buffer, "+CGREG: 0,");
            success = (p && (*(p + 10) == '1' || *(p + 10) == '5'));
          }
        } while (!success && millis() - t < 30000);
        if (!success) break;

        do {
          sprintf(buffer, "AT+CGSOCKCONT=1,\"IP\",\"%s\"\r", apn);
          success = netSendCommand(buffer);
        } while (!success && millis() - t < 30000);
        if (!success) break;

        success = netSendCommand("AT+CSOCKSETPN=1\r");
        if (!success) break;

        success = netSendCommand("AT+CIPMODE=0\r");
        if (!success) break;

        netSendCommand("AT+NETOPEN\r");
        delay(5000);
      } while (0);
      if (!success) Serial.println(buffer);
      return success;
    }
    const char* getIP()
    {
      uint32_t t = millis();
      char *ip = 0;
      do {
        if (netSendCommand("AT+IPADDR\r", 5000, "+IPADDR:")) {
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
      } while (millis() - t < 60000);
      return ip;
    }
    int getSignal()
    {
      if (netSendCommand("AT+CSQ\r", 500)) {
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
      if (netSendCommand("AT+COPS?\r") == 1) {
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
      return netSendCommand("AT+CHTTPSSTART\r", 3000);
    }
    void httpClose()
    {
      netSendCommand("AT+CHTTPSCLSE\r", 1000, "+CHTTPSCLSE:");
      netSendCommand("AT+CHTTPSSTOP\r");
    }
    bool httpConnect()
    {
      sprintf(buffer, "AT+CHTTPSOPSE=\"%s\",%u,%u\r", HTTP_SERVER_URL, HTTP_SERVER_PORT, HTTP_SERVER_PORT == 443 ? 2 : 1);
      //Serial.println(buffer);
      return netSendCommand(buffer, MAX_CONN_TIME);
    }
    unsigned int genHttpHeader(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload, int payloadSize)
    {
      // generate HTTP header
      char *p = buffer;
      p += sprintf(p, "%s %s HTTP/1.1\r\nUser-Agent: ONE\r\nHost: %s\r\nConnection: %s\r\n",
                   method == HTTP_GET ? "GET" : "POST", path, HTTP_SERVER_URL, keepAlive ? "keep-alive" : "close");
      if (method == HTTP_POST) {
        p += sprintf(p, "Content-length: %u\r\n", payloadSize);
      }
      p += sprintf(p, "\r\n");
      return (unsigned int)(p - buffer);
    }
    bool httpSend(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload = 0, int payloadSize = 0)
    {
      unsigned int headerSize = genHttpHeader(method, path, keepAlive, payload, payloadSize);
      // issue HTTP send command
      sprintf(buffer, "AT+CHTTPSSEND=%u\r", headerSize + payloadSize);
      if (!netSendCommand(buffer, 100, ">")) {
        Serial.println(buffer);
        return false;
      }
      // send HTTP header
      genHttpHeader(method, path, keepAlive, payload, payloadSize);
      xbWrite(buffer);
      // send POST payload if any
      if (payload) xbWrite(payload);
      buffer[0] = 0;
      if (netSendCommand(0, 200)) {
        return true;
      }
      return false;
    }
    char* httpReceive()
    {
      // wait for RECV EVENT
      if (!netSendCommand(0, 5000, "+CHTTPS: RECV EVENT")) {
        return 0;
      }
      /*
        +CHTTPSRECV:XX\r\n
        [XX bytes from server]\r\n
        \r\n+CHTTPSRECV: 0\r\n
      */
      if (netSendCommand("AT+CHTTPSRECV=384\r", MAX_CONN_TIME, "\r\n+CHTTPSRECV: 0", true)) {
        char *p = strstr(buffer, "\r\n\r\n");
        if (p) {
          return p + 4;
        }
      }
      return 0;
    }
    bool netSendCommand(const char* cmd, unsigned int timeout = 2000, const char* expected = "\r\nOK", bool terminated = false)
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
    char buffer[384];
};

CSIM5360 net;
byte netState = NET_DISCONNECTED;
byte errors = 0;

void setup()
{
  Serial.begin(115200);
  // this will init SPI communication
  net.begin();
  net.xbBegin(XBEE_BAUDRATE);

  // initialize SIM5360 xBee module (if present)
  for (;;) {
    Serial.print("Init SIM5360...");
    if (net.netInit()) {
      Serial.println("OK");
      break;
    } else {
      Serial.println("NO");
    }
  }

  Serial.print("Connecting network");
  if (net.netSetup(APN, false)) {
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
}

void printTime()
{
  Serial.print('[');
  Serial.print(millis());
  Serial.print(']');
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

  // connect to HTTP server
  if (netState != NET_CONNECTED) {
    net.httpOpen();
    printTime();
    Serial.print("Connecting...");
    net.xbPurge();
    if (!net.httpConnect()) {
      Serial.println("Error connecting");
      Serial.println(net.buffer);
      net.httpClose();
      errors++;
      return;
    } else {
      Serial.println("OK");
    }
  }

  // send HTTP request
  printTime();
  Serial.print("Sending HTTP request...");
  if (!net.httpSend(HTTP_GET, HTTP_SERVER_PATH, true)) {
    Serial.println("failed");
    net.httpClose();
    errors++;
    netState = NET_DISCONNECTED;
    return;
  } else {
    Serial.println("OK");
    netState = NET_CONNECTED;
  }

  printTime();
  Serial.print("Receiving...");
  char *response = net.httpReceive();
  if (response) {
    Serial.println("OK");
    Serial.println("-----HTTP RESPONSE-----");
    Serial.println(response);
    Serial.println("-----------------------");
    netState = NET_CONNECTED;
    errors = 0;
  } else {
    Serial.println("failed");
    errors++;
  }

  Serial.println("Waiting 5 seconds...");
  delay(5000);
}
