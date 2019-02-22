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

#define HTTP_SERVER_URL "hub.freematics.com"
#define HTTP_SERVER_PORT 80
#define CONN_TIMEOUT 10000
#define XBEE_BAUDRATE 115200

typedef enum {
    NET_DISCONNECTED = 0,
    NET_CONNECTED,
    NET_HTTP_ERROR,
} NET_STATES;

FreematicsESP32 sys;

class SIM5360 {
public:
    SIM5360() { buffer[0] = 0; }
    bool init()
    {
      for (byte n = 0; n < 10; n++) {
        // try turning on module
        sys.xbTogglePower();
        delay(3000);
        // discard any stale data
        sys.xbPurge();
        for (byte m = 0; m < 3; m++) {
          if (sendCommand("ATI\r", 500)) {
            return true;
          }
        }
      }
      return false;
    }
    bool setup(bool roaming = false)
    {
      uint32_t t = millis();
      bool success = false;
      sendCommand("ATE0\r");
      //sendCommand("AT+CNMP=13\r"); // GSM only
      //sendCommand("AT+CNMP=14\r"); // WCDMA only
      //sendCommand("AT+CNMP=38\r"); // LTE only
      do {
        do {
          success = sendCommand("AT+CPSI?\r", 1000, "Online");
          if (strstr(buffer, "Off")) {
            success = false;
            break;
          }
          if (success) {
            if (!strstr(buffer, "NO SERVICE"))
              break;
            success = false;
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

        //sendCommand("AT+CGSOCKCONT=1,\"IP\",\"APN\"\r");
        //sendCommand("AT+CSOCKAUTH=1,1,\"APN_PASSWORD\",\"APN_USERNAME\"\r");
        sendCommand("AT+CSOCKSETPN=1\r");
        sendCommand("AT+CIPMODE=0\r");
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
    String getIP()
    {
      uint32_t t = millis();
      do {
        if (sendCommand("AT+IPADDR\r", 3000, "\r\nOK\r\n")) {
          char *p = strstr(buffer, "+IPADDR:");
          if (p) {
            char *ip = p + 9;
            if (*ip != '0') {
              char *q = strchr(ip, '\r');
              if (q) *q = 0;
              return ip;
            }
          }
        }
        delay(500);
      } while (millis() - t < 15000);
      return "";
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
    bool httpOpen()
    {
        return sendCommand("AT+CHTTPSSTART\r", 3000);
    }
    void httpClose()
    {
      sendCommand("AT+CHTTPSCLSE\r");
    }
    bool httpConnect(const char* host, unsigned int port)
    {
        sprintf(buffer, "AT+CHTTPACT=\"%s\",%u\r", host, port);
	      return sendCommand(buffer, CONN_TIMEOUT, "+CHTTPACT: REQUEST");
    }
    bool httpSend(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload = 0, int payloadSize = 0)
    {
      // generate HTTP header
      char *p = buffer;
      p += sprintf(p, "%s %s HTTP/1.1\r\nUser-Agent: ONE\r\nHost: %s\r\nConnection: %s\r\n",
        method == HTTP_GET ? "GET" : "POST", path, HTTP_SERVER_URL, keepAlive ? "keep-alive" : "close");
      if (method == HTTP_POST) {
        p += sprintf(p, "Content-length: %u\r\n", payloadSize);
      }
      p += sprintf(p, "\r\n");
      sys.xbWrite(buffer);
      // send POST payload if any
      if (payload) sys.xbWrite(payload);
      buffer[0] = 0;
      if (sendCommand("\x1A")) {
        checkTimer = millis();
        return true;
      } else {
        Serial.println(buffer);
        return false;
      }
    }
    String httpReceive()
    {
      String response;
      int receivedBytes = 0;
      if (sendCommand(0, CONN_TIMEOUT, "+CHTTPACT: 0")) {
        char *p = strstr(buffer, "+CHTTPACT: DATA,");
        if (p) {
          p += 16;
          receivedBytes = atoi(p + 1);
          p = strchr(p, '\n');
          if (p) {
            char *q = strstr(p, "+CHTTPACT: 0");
            if (q) *q = 0;
            response = p;
            Serial.print(receivedBytes);
            Serial.print(' ');
            Serial.println(response.length());
          }
        }
      }
      return response;
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
    bool sendCommand(const char* cmd, unsigned int timeout = 2000, const char* expected = "\r\nOK\r\n")
    {
      if (cmd) {
        sys.xbWrite(cmd);
      }
      buffer[0] = 0;
      byte ret = sys.xbReceive(buffer, sizeof(buffer), timeout, &expected, 1);
      if (ret) {
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
    while (!sys.begin());

    // initialize SIM7600 module
    Serial.print("Init SIM7600...");
    sys.xbBegin(XBEE_BAUDRATE);
    if (net.init()) {
      Serial.println("OK");
    } else {
      Serial.println("NO");
      for (;;);
    }
    Serial.println(net.buffer);

    Serial.print("Connecting network...");
    if (net.setup()) {
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
    String ip = net.getIP();
    if (ip.length()) {
      Serial.println(ip);
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

void loop()
{
  if (errors > 0) {
    net.httpClose();
    netState = NET_DISCONNECTED;
    if (errors > 3) {
      // re-initialize cellular module
      setup();
      errors = 0;
    }
  }

  // connect to HTTP server
    if (!net.httpConnect(HTTP_SERVER_URL, HTTP_SERVER_PORT)) {
      Serial.println("Error connecting");
      Serial.println(net.buffer);
      errors++;
      return;
    }

  // send HTTP request
  Serial.print("Sending HTTP request...");
  if (!net.httpSend(HTTP_GET, "/hub/api/test", true)) {
    Serial.println("failed");
    net.httpClose();
    errors++;
    return;
  } else {
    Serial.println("OK");
  }

  Serial.print("Receiving...");
  String response = net.httpReceive();
  if (response.length() > 0) {
    Serial.println("-----HTTP RESPONSE-----");
    Serial.println(response);
    Serial.println("-----------------------");
    errors = 0;
  } else {
    Serial.println("failed");
    errors++;
  }

  Serial.println("Waiting 3 seconds...");
  delay(3000);
}
