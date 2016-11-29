/******************************************************************************
* Reference sketch for a vehicle data feed for Freematics Hub
* Works with Freematics ONE with ESP8266 WIFI module
* Developed by Stanley Huang https://www.facebook.com/stanleyhuangyc
* Distributed under BSD license
* Visit http://freematics.com/hub/api for Freematics Hub API reference
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
#include "config.h"
#include "datalogger.h"

// logger states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_SLEEPING 0x20
#define STATE_CONNECTED 0x40

uint32_t nextConnTime = 0;
uint16_t connCount = 0;
long accSum[3]; // sum of accelerometer x/y/z data
byte accCount = 0; // count of accelerometer readings
int lastSpeed = 0;
uint32_t lastSpeedTime = 0;
uint32_t distance = 0;

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
    void resetWifi()
    {
      sendWifiCommand("AT+RST\r\n");
    }
    void disconnectWifi()
    {
      sendWifiCommand("AT+CWQAP\r\n");
    }
    bool initWifi()
    {
      // set xBee module serial baudrate
      bool success = false;
      // test the module by issuing AT command and confirming response of "OK"
      for (byte n = 0; !(success = sendWifiCommand("ATE0\r\n")) && n < 10; n++) {
        delay(100);
      }
      if (success) {
        // send ESP8266 initialization commands
        sendWifiCommand("AT+CWMODE=1\r\n", 100);
        sendWifiCommand("AT+CIPMUX=0\r\n");
        return true;
      } else {
        return false;
      }
    }
    bool setupWifi()
    {
      // generate and send AT command for joining AP
      sprintf_P(buffer, PSTR("AT+CWJAP=\"%s\",\"%s\"\r\n"), WIFI_SSID, WIFI_PASSWORD);
      byte ret = sendWifiCommand(buffer, 10000, "OK");
      if (ret == 1) {
        // get IP address
        if (sendWifiCommand("AT+CIFSR\r\n", 1000, "OK") && !strstr(buffer, "0.0.0.0")) {
          char *p = strchr(buffer, '\r');
          if (p) *p = 0;
          Serial.println(buffer);
          // output IP address
          return true;
        } else {
          // output error message
          Serial.println(buffer); 
        }
      } else if (ret == 2) {
        Serial.println("Failed to join AP"); 
      }
      return false;
    }
    void httpClose()
    {
      sendWifiCommand("AT+CIPCLOSE\r\n", 1000, "Unlink");
      Serial.println("TCP closed");
    }
    void httpConnect()
    {
      // start TCP connection
      sprintf_P(buffer, PSTR("AT+CIPSTART=\"TCP\",\"%s\",%d\r\n"), SERVER_URL, SERVER_PORT);
      xbWrite(buffer);
      Serial.print(buffer);
      // clear reception buffer
      buffer[0] = 0;
      bytesRecv = 0;
      // reset reception timeout timer
      checkTimer = millis();
    }
    bool httpIsConnected()
    {
        // check if "Linked" is received
        byte ret = checkbuffer("Linked", MAX_CONN_TIME);
        if (ret == 1) {
          // success
          Serial.println("CONNECTED");
          connErrors = 0;
          return true;
        } else if (ret == 2) {
          // timeout
          wifiState = WIFI_HTTP_ERROR;
          connErrors++;
        }
        // not yet
        return false;
    }
    bool httpSend(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload = 0, int payloadSize = 0)
    {
      char header[192];
      char *p = header;
      // generate HTTP header
      p += sprintf_P(p, PSTR("%s %s HTTP/1.1\r\nUser-Agent: ONE\r\nHost: %s\r\nConnection: %s\r\n"),
        method == HTTP_GET ? "GET" : "POST", path, SERVER_URL, keepAlive ? "keep-alive" : "close");
      if (method == HTTP_POST) {
        p += sprintf_P(p, PSTR("Content-length: %u\r\n"), payloadSize);
      }
      p += sprintf_P(p, PSTR("\r\n"));
      // start TCP send
      sprintf_P(buffer, PSTR("AT+CIPSEND=%u\r\n"), (unsigned int)(p - header) + payloadSize);
      if (sendWifiCommand(buffer, 1000, ">")) {
        // send HTTP header
        xbWrite(header);
        delay(50);
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
      // check if "SEND OK" is received
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
      // not yet
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
      // check if expected string is in reception buffer
      if (strstr(buffer, expected)) {
        return 1;
      }
      // if not, receive a chunk of data from xBee module and look for expected string
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
      xbPurge();
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
#if USE_MPU6050
,public CMPU6050
#elif USE_MPU9250
,public CMPU9250
#endif
{
public:
    CTeleLogger():state(0),channel(0) {}
    void setup()
    {
        delay(500);
        
        // initialize hardware serial (for USB or BLE)
        Serial.begin(115200);

        // this will init SPI communication
        begin();

        // initialize OBD communication
        Serial.print("#OBD..");
        for (uint32_t t = millis(); millis() - t < OBD_CONN_TIMEOUT; ) {
            Serial.print('.');
            if (init()) {
              state |= STATE_OBD_READY;
              break;              
            }
        }
        if (state & STATE_OBD_READY) {
          Serial.print("VER ");
          Serial.println(version);
        } else {
          Serial.println("NO"); 
        }

#if USE_MPU6050 || USE_MPU9250
        // start I2C communication 
        Wire.begin();
#if USE_MPU6050
        Serial.print("#MPU6050:");
#else
        Serial.print("#MPU9250:");
#endif
        if (memsInit()) {
          state |= STATE_MEMS_READY;
          Serial.println("OK");
        } else {
          Serial.println("NO");
        }
#endif

#if USE_GPS
        // start serial communication with GPS receive
        Serial.print("#GPS...");
        if (initGPS(GPS_SERIAL_BAUDRATE)) {
          state |= STATE_GPS_READY;
          Serial.println("OK");
        } else {
          Serial.println("NO");
        }
#endif

        // initialize ESP8266 xBee module (if present)
        Serial.print("#ESP8266...");
        xbBegin(XBEE_BAUDRATE);
        if (initWifi()) {
            Serial.println("OK");
        } else {
            Serial.println(buffer);
        }

        // attempt to join AP with pre-defined credential
        // make sure to change WIFI_SSID and WIFI_PASSWORD to your own ones
        for (;;) {
          delay(100);
          Serial.print("#WIFI(SSID:");
          Serial.print(WIFI_SSID);
          Serial.print(")...");
          if (setupWifi()) {
              break;
          } else {
              Serial.println("NO");
          }
        }
        
        // connect with TeleServer (Freematics live data server)
        // will be stuck in the function if unsuccessful
        regDataFeed(0);

        Serial.println();
        delay(1000);
    }
    void regDataFeed(byte action)
    {
      // action == 0 for registering a data feed, action == 1 for de-registering a data feed

      // retrieve VIN
      char vin[20] = {0};
      if (action == 0 && getVIN(buffer, sizeof(buffer))) {
        strncpy(vin, buffer, sizeof(vin) - 1);
        Serial.print("#VIN:");
        Serial.println(vin);
      }

      wifiState = WIFI_READY;
      for (byte n = 0; ;n++) {
        // start a HTTP connection (TCP)
        httpConnect();
        do {
          Serial.print('.');
          delay(200);
        } while (!httpIsConnected() && wifiState != WIFI_HTTP_ERROR);
        if (wifiState == WIFI_HTTP_ERROR) {
          Serial.println(buffer);
          Serial.println("Unable to connect");
          httpClose();
          if (n >= MAX_ERRORS_RESET) {
            Serial.println("Reset WIFI");
            resetWifi();
            delay(5000);
            initWifi();
            setupWifi();
          }
          delay(1000);
          wifiState = WIFI_READY;
          continue;
        }

        // generate HTTP URL
        if (action == 0) {
          sprintf_P(buffer, PSTR("/%s/reg?vin=%s"), SERVER_KEY, vin);
        } else {
          sprintf_P(buffer, PSTR("/%s/reg?id=%d&off=1"), SERVER_KEY, channel);
        }
        
        // send HTTP request
        if (!httpSend(HTTP_GET, buffer, true) || !xbReceive(buffer, sizeof(buffer), MAX_CONN_TIME, "SEND OK")) {
          Serial.println("Error sending");
          httpClose();
          delay(3000);
          continue;
        }

        delay(500);
        // receive and parse response
        xbReceive(buffer, sizeof(buffer), MAX_CONN_TIME, "\"id\"");
        char *p = strstr(buffer, "\"id\"");
        if (!p) {
          if (!xbReceive(buffer, sizeof(buffer), MAX_CONN_TIME, "\"id\"")) {
            httpClose();
            delay(3000);
            continue; 
          }
        }
        if (action == 0) {
          // receive HTTP response
          // grab the data we need from HTTP response payload
          // parse channel number
          p = strstr(buffer, "\"id\"");
          if (p) {
            int m = atoi(p + 5);
            if (m > 0) {
              // keep the channel number needed for data pushing
              channel = m;
              Serial.print("#FEED ID:");
              Serial.println(m);
              state |= STATE_CONNECTED;
              break;
            }
          }
        } else {
          httpClose();
          break; 
        }
        Serial.println(buffer);
        httpClose();
        delay(3000);
      }
    }
    void loop()
    {
        // the main loop
        uint32_t start = millis();

        if (state & STATE_CONNECTED) {
          // process OBD data if connected
          if (state & STATE_OBD_READY) {
            processOBD();
          }

#if USE_MPU6050 || USE_MPU9250
          // process MEMS data if available
          if (state & STATE_MEMS_READY) {
              processMEMS();  
          }
#endif

#if USE_GPS
          // process GPS data if connected
          if (state & STATE_GPS_READY) {
            processGPS();
          }
#endif
  
          // read and log car battery voltage
          int v = getVoltage() * 100;
          logData(PID_BATTERY_VOLTAGE, v);
        }

        do {
          if (millis() > nextConnTime) {
            // process HTTP state machine
            processHttp();
            // continously read speed for calculating trip distance
            if (state & STATE_OBD_READY) {
               readSpeed();
            }
          }
        } while (millis() - start < MIN_LOOP_TIME);
        
        if (wifiState == WIFI_READY && errors > 10) {
          reconnect();
        }
    }
private:
    void processHttp()
    {
        // state machine for HTTP communications
        nextConnTime = millis() + 200;
        switch (wifiState) {
        case WIFI_READY:
            // ready for doing next HTTP request
            if (cacheBytes > 0) {
              // and there is data in cache to send
              sprintf_P(buffer, PSTR("/%s/post?id=%u"), SERVER_KEY, channel);
              // send HTTP POST request with cached data as payload
              if (httpSend(HTTP_POST, buffer, true, cache, cacheBytes)) {
                // success
                Serial.print("Sending ");
                Serial.print(cacheBytes);
                Serial.println(" bytes");
                //Serial.println(cache);
                purgeCache();
                wifiState = WIFI_SENDING;
              } else {
                Serial.println("Request error");
                wifiState = WIFI_HTTP_ERROR;
              }
            }
            break;
        case WIFI_DISCONNECTED:
            // attempt to connect again
            xbPurge();
            httpConnect();
            wifiState = WIFI_CONNECTING;            
            connCount = 0;
            break;
        case WIFI_CONNECTING:
            // in the progress of connecting
            if (httpIsConnected()) {
              wifiState = WIFI_READY;
              state |= STATE_CONNECTED;
              break; 
            }
            break;
        case WIFI_SENDING:
            // in the progress of data sending
            if (httpIsSent() || strstr(buffer, "+IPD")) {
              Serial.println("Sent");
              connErrors = 0;
              wifiState = WIFI_RECEIVING;
              break; 
            }
            break;
        case WIFI_RECEIVING:
            // in the progress of data receiving
            if (httpRead()) {
              // success
              connCount++;
              Serial.print("Success #");
              Serial.println(connCount);
              //Serial.println(buffer);
              if (connCount >= MAX_HTTP_CONNS) {
                // re-establish TCP connection
                httpClose(); 
                wifiState = WIFI_DISCONNECTED;
              } else {
                wifiState = WIFI_READY;
              }
              break;
            }
            break;
        case WIFI_HTTP_ERROR:
            // oops, we got an error
            Serial.println(buffer);
            // check if there are too many connection errors
            if (connErrors >= MAX_ERRORS_RECONNECT || strstr_P(buffer, PSTR("link is not"))) {
              // reset WIFI
              httpClose();
              if (connErrors >= MAX_ERRORS_RESET) {
                state &= ~STATE_CONNECTED;
                Serial.println("Reset WIFI...");
                disconnectWifi();
                resetWifi();
                delay(1000);
                initWifi();
                setupWifi();
                connErrors = 0;
              }
              wifiState = WIFI_DISCONNECTED;
            } else {
              wifiState = WIFI_READY;
            }
            break;
        }
    }
    void processOBD()
    {
        int speed = readSpeed();
        if (speed == -1) {
          return;
        }
        logData(0x100 | PID_SPEED, speed);
        logData(PID_TRIP_DISTANCE, distance);
        // poll more PIDs
        byte pids[]= {0, PID_RPM, PID_ENGINE_LOAD, PID_THROTTLE};
        const byte pidTier2[] = {PID_INTAKE_TEMP, PID_COOLANT_TEMP};
        static byte index2 = 0;
        int values[sizeof(pids)] = {0};
        // read multiple OBD-II PIDs, tier2 PIDs are less frequently read
        pids[0] = pidTier2[index2 = (index2 + 1) % sizeof(pidTier2)];
        byte results = readPID(pids, sizeof(pids), values);
        if (results == sizeof(pids)) {
          for (byte n = 0; n < sizeof(pids); n++) {
            logData(0x100 | pids[n], values[n]);
          }
        }
    }
    int readSpeed()
    {
        int value;
        if (readPID(PID_SPEED, value)) {
           dataTime = millis();
           distance += (value + lastSpeed) * (dataTime - lastSpeedTime) / 3600 / 2;
           lastSpeedTime = dataTime;
           lastSpeed = value;
           return value;
        } else {
          return -1; 
        }
    }
#if USE_MPU6050 || USE_MPU9250
    void processMEMS()
    {
         // log the loaded MEMS data
        if (accCount) {
          logData(PID_ACC, accSum[0] / accCount / ACC_DATA_RATIO, accSum[1] / accCount / ACC_DATA_RATIO, accSum[2] / accCount / ACC_DATA_RATIO);
          accSum[0] = 0;
          accSum[1] = 0;
          accSum[2] = 0;
          accCount = 0;
        }
    }
#endif
    void processGPS()
    {
        static uint16_t lastUTC = 0;
        static uint8_t lastGPSDay = 0;
        GPS_DATA gd = {0};
        // read parsed GPS data
        if (getGPSData(&gd)) {
            if (lastUTC != (uint16_t)gd.time) {
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
          Serial.println("No GPS data");
          xbPurge();
        }
    }
    void reconnect()
    {
        // try to re-connect to OBD
        Serial.println("Reconnecting");
        for (byte n = 0; n < 3; n++) {
          if (init()) {
            // reconnected
            return; 
          }
          delay(1000);
        }
        // seems we can't connect, put the device into sleeping mode
        httpClose();
        regDataFeed(1); // de-register
        disconnectWifi(); // disconnect from AP
        //delay(500);
        //resetWifi();
#if USE_GPS
        initGPS(0); // turn off GPS power
#endif
        state &= ~(STATE_OBD_READY | STATE_GPS_READY | STATE_MEMS_READY);
        state |= STATE_SLEEPING;
        Serial.println("Sleeping");
        // regularly check if we can get any OBD data
        for (;;) {
            if (readSpeed() != -1) {
              // a successful readout
              break;
            }
            // put the device into low power mode for 30 seconds
            sleep(30);
        }
        // we are able to get OBD data again
        // reset the device
        void(* resetFunc) (void) = 0; //declare reset function at address 0
        resetFunc();
    }
#if USE_MPU6050 || USE_MPU9250
    void dataIdleLoop()
    {
      // do something while waiting for data on SPI
      if (state & STATE_MEMS_READY) {
        // load accelerometer and temperature data
        int acc[3] = {0};
        int temp; // device temperature (in 0.1 celcius degree)
        memsRead(acc, 0, 0, &temp);
        if (accCount < 250) {
          accSum[0] += acc[0];
          accSum[1] += acc[1];
          accSum[2] += acc[2];
          accCount++;
        }
      }
    }
#endif
    
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
