/******************************************************************************
* Test sketch for SIM5360E (3G) module in Freematics ONE
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

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <FreematicsONE.h>
#ifdef ESP32
#include <TinyGPS.h>
#endif

#define APN "connect"
#define SERVER_URL "hub.freematics.com"
#define SERVER_PORT 80
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
    CSIM5360() { buffer[0] = 0; }
    bool netInit()
    {
      for (byte n = 0; n < 10; n++) {
        // try turning on module
        xbTogglePower();
        sleep(3000);
        // discard any stale data
        xbPurge();
        for (byte m = 0; m < 3; m++) {
          if (netSendCommand("AT\r"))
            return true;
        }
      }
      return false;
    }
    bool netSetup(const char* apn)
    {
      uint32_t t = millis();
      bool success = false;
      netSendCommand("ATE0\r");
      do {
        do {
          Serial.print('.');
          sleep(500);
          success = netSendCommand("AT+CPSI?\r", 1000, "Online");
          if (success) {
            if (!strstr_P(buffer, PSTR("NO SERVICE")))
              break;
            success = false;
          } else {
            if (strstr_P(buffer, PSTR("Off"))) break;
          }
        } while (millis() - t < 60000);
        Serial.println(buffer);
        if (!success) break;
        
        t = millis();
        do {
          success = netSendCommand("AT+CREG?\r", 5000, "+CREG: 0,1");
        } while (!success && millis() - t < 30000);
        if (!success) break;

        do {
          success = netSendCommand("AT+CGREG?\r",1000, "+CGREG: 0,1");
        } while (!success && millis() - t < 30000);
        if (!success) break;

        do {
          sprintf_P(buffer, PSTR("AT+CGSOCKCONT=1,\"IP\",\"%s\"\r"), apn);
          success = netSendCommand(buffer);
        } while (!success && millis() - t < 30000);
        if (!success) break;

        success = netSendCommand("AT+CSOCKSETPN=1\r");
        if (!success) break;

        success = netSendCommand("AT+CIPMODE=0\r");
        if (!success) break;
        
        netSendCommand("AT+NETOPEN\r");
        sleep(500);
        t = millis();
        do {
          netSendCommand("AT+IPADDR\r", 5000);
          success = !strstr_P(buffer, PSTR("0.0.0.0")) && !strstr_P(buffer, PSTR("ERROR"));
        } while (!success && millis() - t < 15000);
      } while(0);
      Serial.println(buffer);
      return success;
    }
    int getSignal()
    {
        if (netSendCommand("AT+CSQ\r", 500)) {
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
      netSendCommand("AT+CHTTPSCLSE\r");
    }
    bool httpConnect()
    {
        xbPurge();
        sprintf_P(buffer, PSTR("AT+CHTTPSOPSE=\"%s\",%u,1\r"), SERVER_URL, SERVER_PORT);
	      return netSendCommand(buffer, MAX_CONN_TIME);
    }
    unsigned int genHttpHeader(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload, int payloadSize)
    {
        // generate HTTP header
        char *p = buffer;
        p += sprintf_P(p, PSTR("%s %s HTTP/1.1\r\nUser-Agent: ONE\r\nHost: %s\r\nConnection: %s\r\n"),
          method == HTTP_GET ? "GET" : "POST", path, SERVER_URL, keepAlive ? "keep-alive" : "close");
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
      if (!netSendCommand(buffer, 100, ">")) {
        Serial.println("Connection closed");
      }
      // send HTTP header
      genHttpHeader(method, path, keepAlive, payload, payloadSize);
      xbWrite(buffer);
      // send POST payload if any
      if (payload) xbWrite(payload);
      buffer[0] = 0;
      if (netSendCommand("AT+CHTTPSSEND\r")) {
        return true;        
      } else {
        Serial.println(buffer);
        return false;
      }
    }
    bool httpStartRecv()
    {
        if (checkbuffer("RECV EVENT", MAX_CONN_TIME) == 1) {
          xbWrite("AT+CHTTPSRECV=1024\r");
          return true;
        } else {
          return false;
        }
    }
    byte httpReceive()
    {
      byte ret = xbReceive(buffer, sizeof(buffer), MAX_CONN_TIME, "+CHTTPSRECV:");
      if (ret == 1) {
        char *p = strstr(buffer, "+CHTTPSRECV:");
        if (!p) return 0;
        p += 13;
        if (*p == '0') {
          // eos
          return 2;
        } else {
          return 1;
        }
      }
      Serial.println(buffer);
      return 0;
    }
    byte checkbuffer(const char* expected, unsigned int timeout = 2000)
    {
      // check if expected string is in reception buffer
      if (strstr(buffer, expected)) {
        return 1;
      }
      // if not, receive a chunk of data from xBee module and look for expected string
      byte ret = xbReceive(buffer, sizeof(buffer), timeout, expected) != 0;
      if (ret == 0) {
        // timeout
      } else {
        return ret;
      }
    }
    bool netSendCommand(const char* cmd, unsigned int timeout = 2000, const char* expected = "OK")
    {
      xbPurge();
      if (cmd) {
        xbWrite(cmd);
        delay(10);
      }
      buffer[0] = 0;
      return xbReceive(buffer, sizeof(buffer), timeout, expected) != 0;
    }
    char buffer[384];
private:
    void togglePower()
    {
        setTarget(TARGET_OBD);
        sendCommand("ATGSMPWR\r", buffer, sizeof(buffer));
    }
};

CSIM5360 sim;
byte netState = NET_DISCONNECTED;
byte errors = 0;

void setup()
{
    Serial.begin(115200);
    Serial.println("Freematics ONE");

    delay(500);
    // this will init SPI communication
    sim.begin();

    // initialize SIM5360 xBee module (if present)
    Serial.print("Init SIM5360...");
    sim.xbBegin(XBEE_BAUDRATE);
    if (sim.netInit()) {
      Serial.println("OK");
    } else {
      Serial.println("NO");
      for (;;);
    }
    
    Serial.print("Setup network");
    if (sim.netSetup(APN)) {
      Serial.println("OK");
    } else {
      Serial.println("NO");
    }

    int signal = sim.getSignal();
    if (signal > 0) {
      Serial.print("CSQ:");
      Serial.print((float)signal / 10, 1);
      Serial.println("dB");
    }

    if (sim.getOperatorName()) {
      Serial.print("Operator:");
      Serial.println(sim.buffer);
    }
    
    Serial.print("Init HTTP stack...");
    if (sim.httpOpen()) {
      Serial.println("OK");
    } else {
      Serial.println("NO");
    }
}

void loop()
{
  if (netState != NET_CONNECTED) {
    Serial.println("Connecting...");
    sim.xbPurge();
    if (!sim.httpConnect()) {
      Serial.println("Error connecting");
      Serial.println(sim.buffer);
      return false;
    }
  }
  
  // send HTTP request
  Serial.print("Sending HTTP request...");
  if (!sim.httpSend(HTTP_GET, "/test", true)) {
    Serial.println("failed");
    sim.httpClose();
    netState = NET_DISCONNECTED;
    return;
  } else {
   Serial.println("OK"); 
  }

  Serial.print("Wait for response...");
  if (sim.httpStartRecv()) {
    Serial.println("OK");
  } else {
    Serial.println("failed");
    errors++;
    return;
  }

  Serial.print("Receiving...");
  byte ret;
  while ((ret = sim.httpReceive()) == 1);
  if (ret == 2) {
    Serial.println("OK");
    char *payload = strstr(sim.buffer, "\r\n\r\n");
    if (payload) {
      char *p = strstr(payload, "+CHTTPSRECV:");
      if (p) *p = 0;
      Serial.println(payload + 4);
    }
    netState = NET_CONNECTED;
    errors = 0;
  } else {
    Serial.println("failed");
    errors++;
  }

  if (errors > 3) {
    sim.httpClose();
    netState = NET_DISCONNECTED;
  }
  delay(3000);
}

