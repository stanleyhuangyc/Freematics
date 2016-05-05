/******************************************************************************
* Vehicle Telematics Data Logger Sketch for Freematics ONE
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one for more information
* Developed by Stanley Huang <stanleyhuangyc@gmail.com>
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
#include "config.h"
#include "datalogger.h"

// logger states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_SLEEPING 0x20
#define STATE_CONNECTED 0x40

#if !ENABLE_DATA_OUT
#define Serial Serial
#endif

static uint16_t lastUTC = 0;
static uint8_t lastGPSDay = 0;
static uint32_t nextConnTime = 0;
static uint16_t connCount = 0;

#define TIER_NUM1 sizeof(pidTier1)
#define TIER_NUM2 sizeof(pidTier2)

typedef enum {
    WIFI_DISCONNECTED = 0,
    WIFI_READY,
    WIFI_CONNECTING,
    WIFI_SENDING,
    WIFI_RECEIVING,
    WIFI_HTTP_ERROR,
} WIFI_STATES;

typedef enum {
  HTTP_GET = 0,
  HTTP_POST,
} HTTP_METHOD;

class COBDWIFI : public COBDSPI {
public:
    COBDWIFI():wifiState(WIFI_DISCONNECTED),connErrors(0) { buffer[0] = 0; }
    void disconnectWifi()
    {
      sendWifiCommand("AT+CWQAP\r\n");
    }
    bool initWifi()
    {
      xbBegin(9600);
      bool success = false;
      for (byte n = 0; !(success = sendWifiCommand("ATE0\r\n")) && n < 10; n++) {
        delay(100);
      }
      sendWifiCommand("AT+CWMODE=1\r\n", 100);
      sendWifiCommand("AT+CIPMUX=0\r\n");
      return success;
    }
    bool setupWifi()
    {
      sprintf(buffer, "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PASSWORD);
      byte ret = sendWifiCommand(buffer, 10000, "OK");
      if (ret == 1) {
        // get IP address
        if (sendWifiCommand("AT+CIFSR\r\n", 1000, "OK")) {
          char *p = strchr(buffer, '\r');
          if (p) *p = 0;
          Serial.print(buffer);
          return true;
        } else {
          Serial.println(buffer); 
        }
      } else if (ret == 2) {
        Serial.print("timeout"); 
      }
      return false;
    }
    void httpClose()
    {
      sendWifiCommand("AT+CIPCLOSE\r\n", 1000, "Unlink");
      Serial.println("DISCONNECTED");
    }
    void httpConnect()
    {
      Serial.println("CONNECTING");
      sprintf(buffer, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", SERVER_URL, SERVER_PORT);
      xbWrite(buffer);
      buffer[0] = 0;
      bytesRecv = 0;
      checkTimer = millis();
    }
    bool httpIsConnected()
    {
        byte ret = checkbuffer("Linked", MAX_CONN_TIME);
        if (ret == 1) {
          Serial.println("CONNECTED");
          connErrors = 0;
          return true;
        } else if (ret == 2) {
          // timeout
          wifiState = WIFI_HTTP_ERROR;
          connErrors++;
        }
        return false;
    }
    bool httpSend(HTTP_METHOD method, const char* path, const char* payload = 0, int payloadSize = 0)
    {
      char header[128];
      char *p = header;
      // generate HTTP header
      p += sprintf(p, "%s %s HTTP/1.1\r\nUser-Agent: Freematics\r\nConnection: keep-alive\r\n",
      method == HTTP_GET ? "GET" : "POST", path, SERVER_URL);
      if (method == HTTP_POST) {
        p += sprintf(p, "Content-length: %u\r\n", payloadSize);
      }
      p += sprintf(p, "\r\n");
      // start TCP send
      sprintf(buffer, "AT+CIPSEND=%u\r\n", (unsigned int)(p - header) + payloadSize);
      if (sendWifiCommand(buffer, 1000, ">")) {
        // send HTTP header
        xbWrite(header);
        // send POST payload if any
        if (payload) xbWrite(payload);
        buffer[0] = 0;
        bytesRecv = 0;
        checkTimer = millis();
        return true;
      }
      connErrors++;
      return false;
    }
    bool httpIsSent()
    {
      byte ret = checkbuffer("SEND OK", MAX_CONN_TIME);
      if (ret == 1) {
        // success
        connErrors = 0;
        return true;
      } else if (ret == 2) {
        // timeout
        wifiState = WIFI_HTTP_ERROR;
        connErrors++;
      }
      return false;
    }
    bool httpRead()
    {
        byte ret = checkbuffer("+IPD", MAX_CONN_TIME);
        if (ret == 1) {
          // success
          connErrors = 0;
          return true;
        } else if (ret == 2) {
          // timeout
          wifiState = WIFI_HTTP_ERROR;
          connErrors++;
        }
        return false;
    }
    byte checkbuffer(const char* expected, unsigned int timeout = 2000)
    {
      if (strstr(buffer, expected)) {
        return 1;
      }
      byte ret = xbReceive(buffer, sizeof(buffer), 0, expected) != 0;
      if (ret == 0) {
        // timeout
        return (millis() - checkTimer < timeout) ? 0 : 2;
      } else {
        return ret;
      }
    }
    bool sendWifiCommand(const char* cmd, unsigned int timeout = 2000, const char* expected = "OK")
    {
      if (cmd) {
        xbWrite(cmd);
        delay(50);
      }
      buffer[0] = 0;
      return xbReceive(buffer, sizeof(buffer), timeout, expected) != 0;
    }
    char buffer[192];
    byte bytesRecv;
    uint32_t checkTimer;
    byte wifiState;
    byte connErrors;
};

class CTeleLogger : public COBDWIFI, public CDataLogger
{
public:
    CTeleLogger():state(0),channel(0) {}
    void setup()
    {
        delay(500);
        
        Serial.begin(115200);

        // this will init SPI communication
        begin();

        Serial.print("#OBD..");
        setTarget(TARGET_OBD);
        do {
            Serial.print('.');
        } while (!init());
        Serial.print("VER ");
        Serial.println(version);
        state |= STATE_OBD_READY;

#if USE_MPU6050
        Wire.begin();
        Serial.print("#MEMS...");
        if (memsInit()) {
          Serial.println("OK");
        } else {
          Serial.println("NO");
        }
#endif

#if USE_GPS
        if (initGPS(GPS_SERIAL_BAUDRATE)) {
          state |= STATE_GPS_READY;
          Serial.println("#GPS...OK");
        }
#endif

        Serial.print("#ESP8266...");
        if (initWifi()) {
            Serial.println("OK");
        } else {
            Serial.println(buffer);
        }

        for (;;) {
          delay(100);
          Serial.print("#WIFI(SSID:");
          Serial.print(WIFI_SSID);
          Serial.print(")...");
          if (setupWifi()) {
              Serial.print(' ');
              Serial.println("OK");
              break;
          } else {
              Serial.println("NO");
          }
        }
        
        joinChannel(0);
        state |= STATE_CONNECTED;
        
        Serial.println();
        delay(1000);
    }
    void joinChannel(byte action)
    {
#if ENABLE_DATA_CACHE
      char *vin = cache;
#else
      char vin[240];
#endif
      if (action == 0) {
        setTarget(TARGET_OBD);
#if ENABLE_DATA_CACHE
        getVIN(cache, MAX_CACHE_SIZE);
#else
        getVIN(vin, sizeof(vin));
#endif
        Serial.print("#VIN:");
        Serial.println(vin);
      }
      wifiState = WIFI_READY;
      for (;;) {
        httpConnect();
        do {
          Serial.print('.');
          delay(200);
        } while (!httpIsConnected() && wifiState != WIFI_HTTP_ERROR);
        if (wifiState == WIFI_HTTP_ERROR) {
          Serial.println("Unable to connect");
          Serial.println(buffer);
          httpClose();
          delay(1000);
          wifiState = WIFI_READY;
          continue;
        }

        if (action == 0) {
          sprintf(buffer, "/push?VIN=%s", vin);
        } else {
          sprintf(buffer, "/push?id=%d&OFF=1", channel);
        }
        if (!httpSend(HTTP_GET, buffer, 0)) {
          Serial.println("Invalid response"); 
          httpClose();
          continue;
        }

        if (action != 0) return;
        
        if (xbReceive(buffer, sizeof(buffer), MAX_CONN_TIME, "+IPD")) {
          char *p = strstr(buffer, "CH:");
          if (!p) {
            // read one more time
            xbReceive(buffer, sizeof(buffer), MAX_CONN_TIME, "CH:");
            p = strstr(buffer, "CH:");
          }
          if (p) {
            // parse channel number
            int m = atoi(p + 3);
            if (m > 0) {
              channel = m;
              Serial.print("#CHANNEL:");
              Serial.println(m);
              state |= STATE_CONNECTED;
              break;
            }
          }
        }
        Serial.println(buffer);
        httpClose();
        delay(3000);
      }
    }
    void processOBD()
    {
        // poll OBD-II PIDs
        const byte pids[]= {PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_THROTTLE};
        int values[sizeof(pids)] = {0};
        // read multiple OBD-II PIDs
        byte results = read(pids, sizeof(pids), values);
        if (results == sizeof(pids)) {
          for (byte n = 0; n < sizeof(pids); n++) {
            logData(0x100 | pids[n], values[n]);
          }
        }
        static byte index2 = 0;
        const byte pidTier2[] = {PID_INTAKE_TEMP, PID_COOLANT_TEMP};
        byte pid = pgm_read_byte(pidTier2 + index2);
        int value;
        // read a single OBD-II PID
        if (read(pid, value)) {
          logData(0x100 | pid, value);
        }
        index2 = (index2 + 1) % sizeof(pidTier2);
        if (errors > 10) {
            reconnect();
        }
    }
    void loop()
    {
        if (state & STATE_OBD_READY) {
          processOBD();
        }

#if USE_GPS
        if (state & STATE_GPS_READY) {
          processGPS();
          delay(10);
        }
#endif

#if USE_MPU6050
        if (state & STATE_MEMS_READY) {
            processMEMS();  
        }
#endif

        if (millis() > nextConnTime) {
          processWifi();
        } else {
#if ENABLE_DATA_LOG
          flushData();
#endif
        }
        if (connErrors >= MAX_CONN_ERRORS) {
          // reset WIFI
          Serial.println("Reset WIFI...");
          httpClose();
          disconnectWifi();
          delay(1000);
          setupWifi();
          httpConnect();
          connErrors = 0;
          wifiState = WIFI_CONNECTING;          
        }
    }
private:
    void processWifi()
    {
        switch (wifiState) {
        case WIFI_READY:
            if (cacheBytes > 0) {
              // there is data in cache for sending
              sprintf(buffer, "/post?id=%u", channel);
              if (httpSend(HTTP_POST, buffer, cache, cacheBytes)) {
                // success
                Serial.print("Sending ");
                Serial.print(cacheBytes);
                Serial.println(" bytes");
                cacheBytes = 0;
                wifiState = WIFI_SENDING;
              } else {
                Serial.println("Request error");
                wifiState = WIFI_HTTP_ERROR;
              }
            }
            break;
        case WIFI_DISCONNECTED:
            xbPurge();
            httpConnect();
            wifiState = WIFI_CONNECTING;            
            connCount = 0;
            nextConnTime = millis() + 100;
            break;
        case WIFI_CONNECTING:
            if (httpIsConnected()) {
              wifiState = WIFI_READY;
              break; 
            }
            nextConnTime = millis() + 100;
            break;
        case WIFI_SENDING:
            if (httpIsSent() || strstr(buffer, "+IPD")) {
              Serial.println("Sent");
              wifiState = WIFI_RECEIVING;
              break; 
            }
            nextConnTime = millis() + 100;
            break;
        case WIFI_RECEIVING:
            if (httpRead()) {
              // success
              connCount++;
              Serial.print("Success #");
              Serial.println(connCount);
              //Serial.println(buffer);
              if (connCount >= MAX_HTTP_CONNS) {
                // re-establish TCP connection
                httpClose(); 
                nextConnTime = millis() + 500;
                wifiState = WIFI_DISCONNECTED;
              } else {
                wifiState = WIFI_READY;
              }
              break;
            }
            nextConnTime = millis() + 200; 
            break;
        case WIFI_HTTP_ERROR:
            Serial.println(buffer);
            if (connErrors >= 3) {
              // need to tear down TCP connection for reconnection
              httpClose();
              nextConnTime = millis() + 500;
              wifiState = WIFI_DISCONNECTED;
            } else {
              // just ignore some minor response errors
              wifiState = WIFI_READY;
            }
            break;
        }
    }
#if USE_MPU6050
    void processMEMS()
    {
      if (state & STATE_MEMS_READY) {
        MEMS_DATA mems;
        memsRead(&mems);
        logData(PID_ACC, mems.value.x_accel / ACC_DATA_RATIO, mems.value.y_accel / ACC_DATA_RATIO, mems.value.z_accel / ACC_DATA_RATIO);
      }
    }
#endif
    void processGPS()
    {
        GPS_DATA gd = {0};
        if (getGPSData(&gd)) {
            if (lastUTC != (uint16_t)gd.time) {
              dataTime = millis();
              byte day = gd.date / 10000;
              logData(PID_GPS_TIME, gd.time);
              if (lastGPSDay != day) {
                logData(PID_GPS_DATE, gd.date);
                lastGPSDay = day;
              }
              logCoordinate(PID_GPS_LATITUDE, gd.lat);
              logCoordinate(PID_GPS_LONGITUDE, gd.lng);
              logData(PID_GPS_ALTITUDE, gd.alt);
              logData(PID_GPS_SPEED, gd.speed);
              logData(PID_GPS_SAT_COUNT, gd.sat);
              lastUTC = (uint16_t)gd.time;
            }
            //Serial.print("#UTC:"); 
            //Serial.println(gd.time);
        } else {
          Serial.println("GPS error");
          delay(500);
        }
    }
    void reconnect()
    {
        if (init()) return; 
        delay(3000);
        if (init()) return; 
        Serial.print("Sleeping");
        state &= ~STATE_OBD_READY;
        joinChannel(1); // leave channel
        disconnectWifi(); // disconnect from AP
#if USE_GPS
        initGPS(0); // turn off GPS power
#endif
        Serial.println();
        state |= STATE_SLEEPING;
        for (;;) {
            int value;
            if (read(PID_RPM, value))
                break;
            sleep(2);
        }
        // reset device
        void(* resetFunc) (void) = 0; //declare reset function at address 0
        resetFunc();
    }
    void dataIdleLoop()
    {
      delay(10);
    }
    
    byte state;
    byte channel;
};

CTeleLogger logger;

void setup()
{
    logger.initSender();
    logger.setup();
}

void loop()
{
    logger.loop();
}
