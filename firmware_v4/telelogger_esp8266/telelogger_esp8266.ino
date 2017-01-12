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
#define STATE_NET_READY 0x10
#define STATE_CONNECTED 0x40

uint32_t nextConnTime = 0;
byte accCount = 0; // count of accelerometer readings
long accSum[3] = {0}; // sum of accelerometer data
int accCal[3] = {0}; // calibrated reference accelerometer data
byte deviceTemp = 0; // device temperature
int lastSpeed = 0;
uint32_t lastSpeedTime = 0;
uint32_t distance = 0;

typedef enum {
    NET_DISCONNECTED = 0,
    NET_CONNECTED,
    NET_CONNECTING,
    NET_RECEIVING,
    NET_HTTP_ERROR,
} NET_STATES;

typedef enum {
  HTTP_GET = 0,
  HTTP_POST,
} HTTP_METHOD;

class COBDWIFI : public COBDSPI {
public:
    COBDWIFI() { buffer[0] = 0; }
    void netReset()
    {
      netSendCommand("AT+RST\r\n");
    }
    void netDisconnect()
    {
      netSendCommand("AT+CWQAP\r\n");
    }
    bool netInit()
    {
      // set xBee module serial baudrate
      bool success = false;
      // test the module by issuing AT command and confirming response of "OK"
      for (byte n = 0; !(success = netSendCommand("ATE0\r\n")) && n < 10; n++) {
        delay(100);
      }
      if (success) {
        // send ESP8266 initialization commands
        netSendCommand("AT+CWMODE=1\r\n", 100);
        netSendCommand("AT+CIPMUX=0\r\n");
        return true;
      } else {
        return false;
      }
    }
    bool netSetup()
    {
      // generate and send AT command for joining AP
      sprintf_P(buffer, PSTR("AT+CWJAP=\"%s\",\"%s\"\r\n"), WIFI_SSID, WIFI_PASSWORD);
      byte ret = netSendCommand(buffer, 10000, "OK");
      if (ret == 1) {
        // get IP address
        if (netSendCommand("AT+CIFSR\r\n", 1000, "OK") && !strstr(buffer, "0.0.0.0")) {
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
    void tcpClose()
    {
      netSendCommand("AT+CIPCLOSE\r\n", 1000, "Unlink");
      Serial.println("TCP closed");
    }
    bool tcpConnect()
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
      return true;
    }
    byte tcpIsConnected()
    {
        // check if "Linked" is received
        return checkbuffer("Linked", MAX_CONN_TIME);
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
      if (netSendCommand(buffer, 1000, ">")) {
        // send HTTP header
        xbWrite(header);
        delay(50);
        // send POST payload if any
        if (payload) xbWrite(payload);
        if (xbReceive(buffer, sizeof(buffer), MAX_CONN_TIME, "SEND OK")) {
          buffer[0] = 0;
          bytesRecv = 0;
          checkTimer = millis();
        }
        return true;
      }
      return false;
    }
    byte tcpReceive(const char* expected)
    {
      return checkbuffer(expected, MAX_CONN_TIME);
    }
    byte checkbuffer(const char* expected, unsigned int timeout = 2000)
    {
      // check if expected string is in reception buffer
      /*
      if (strstr(buffer, expected)) {
        return 1;
      }
      */
      // if not, receive a chunk of data from xBee module and look for expected string
      byte ret = xbReceive(buffer, sizeof(buffer), 0, expected) != 0;
      if (ret == 0) {
        // timeout
        return (millis() - checkTimer < timeout) ? 0 : 2;
      } else {
        return ret;
      }
    }
    bool netSendCommand(const char* cmd, unsigned int timeout = 2000, const char* expected = "OK")
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
};


class CTeleLogger : public COBDWIFI, public CDataLogger
#if USE_MPU6050
,public CMPU6050
#elif USE_MPU9250
,public CMPU9250
#endif
{
public:
    CTeleLogger():state(0),feedid(0),connErrors(0),connCount(0),netState(NET_DISCONNECTED) {}
    void setup()
    {
        // this will init SPI communication
        begin();
 
#if USE_MPU6050 || USE_MPU9250
        // start I2C communication 
        Wire.begin();
        Serial.print("#MEMS...");
        if (memsInit()) {
          state |= STATE_MEMS_READY;
          Serial.println("OK");
        } else {
          Serial.println("NO");
        }
#endif

        // initialize OBD communication
        Serial.print("#OBD...");
        if (init()) {
          Serial.println("OK");
        } else {
          Serial.println("NO");
          reconnect();
        }
        state |= STATE_OBD_READY;

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
        if (netInit()) {
          state |= STATE_NET_READY;
          Serial.println("OK");
        } else {
          Serial.println(buffer);
          standby();
        }

        // attempt to join AP with pre-defined credential
        // make sure to change WIFI_SSID and WIFI_PASSWORD to your own ones
        for (byte n = 0; ;n++) {
          delay(100);
          Serial.print("#WIFI...");
          if (netSetup()) {
              break;
          } else {
              Serial.println("NO");
              if (n >= MAX_ERRORS_RECONNECT) standby();
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

      for (byte n = 0; ;n++) {
        // make sure OBD is still accessible
        if (readSpeed() == -1) {
          reconnect(); 
        }
        // start a HTTP connection (TCP)
        tcpConnect();
        byte ret;
        for (byte n = 0; n < 10 && (ret = tcpIsConnected()) == 0; n++) {
          Serial.print('.');
          delay(1000);
        }
        Serial.println();
        if (ret == 2) {
          Serial.println(buffer);
          Serial.println("Unable to connect");
          tcpClose();
          if (n >= MAX_ERRORS_RESET) {
            standby();
          }
          delay(5000);
          continue;
        }

        // generate HTTP request path
        if (action == 0) {
          char vin[20] = {0};
          byte n = sprintf_P(buffer, PSTR("/%s/reg?vin="), SERVER_KEY);
          // retrieve VIN and append it to URL
          getVIN(buffer + n, sizeof(buffer) - n - 1);
          Serial.print("#VIN:");
          Serial.println(buffer + n);
        } else {
          sprintf_P(buffer, PSTR("/%s/reg?id=%d&off=1"), SERVER_KEY, feedid);
        }
        
        // send HTTP request
        if (!httpSend(HTTP_GET, buffer, true)) {
          Serial.println("Error sending");
          tcpClose();
          delay(3000);
          continue;
        }

        delay(500);
        // receive and parse response
        if (tcpReceive("\"id\"") != 1) {
            tcpClose();
            delay(3000);
            continue; 
        }
        netState = NET_CONNECTED;

        if (action == 0) {
          // receive HTTP response
          // grab the data we need from HTTP response payload
          // parse feed ID
          char *p = strstr(buffer, "\"id\"");
          if (p) {
            int m = atoi(p + 5);
            if (m > 0) {
              // keep the feed ID needed for data pushing
              feedid = m;
              Serial.print("#FEED ID:");
              Serial.println(feedid);
              state |= STATE_CONNECTED;
              break;
            }
          }
        } else {
          tcpClose();
          break; 
        }
        Serial.println(buffer);
        tcpClose();
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
            // error and exception handling
            if (netState == NET_CONNECTED) {
              if (errors > 10) {
                reconnect();
              } else if (deviceTemp >= COOLING_DOWN_TEMP && deviceTemp < 100) {
                // device too hot, slow down communication a bit
                Serial.print("Cooling (");
                Serial.print(deviceTemp);
                Serial.println("C)");
                delay(5000);
                break;
              }
            }          
          }
        } while (millis() - start < MIN_LOOP_TIME);
    }
private:
    void processHttp()
    {
        // state machine for HTTP communications
        nextConnTime = millis() + 100;
        switch (netState) {
        case NET_CONNECTED:
            // TCP connected, ready for doing next HTTP request
            if (cacheBytes > 0) {
              // and there is data in cache to send
              sprintf_P(buffer, PSTR("/%s/post?id=%u"), SERVER_KEY, feedid);
              // send HTTP POST request with cached data as payload
              if (httpSend(HTTP_POST, buffer, true, cache, cacheBytes)) {
                // success
                Serial.print(cacheBytes);
                Serial.println(" bytes sent");
                //Serial.println(cache);
                purgeCache();
                netState = NET_RECEIVING;
              } else {
                Serial.println("Request error");
                netState = NET_HTTP_ERROR;
              }
            }
            break;
        case NET_CONNECTING: {
            byte ret = tcpIsConnected();
            // in the progress of connecting
            if (ret == 1) {
              // success
              netState = NET_CONNECTED;
              state |= STATE_CONNECTED;
            } else if (ret == 2) {
              // timeout
              netState = NET_HTTP_ERROR;
            }
            } break;
        case NET_RECEIVING: {
            // in the progress of data receiving
            byte ret = tcpReceive("\"result\"");
            if (ret == 1) {
              // success
              connCount++;
              connErrors = 0;
              Serial.print("Success #");
              Serial.println(connCount);
              //Serial.println(buffer);
              if (connCount >= MAX_HTTP_CONNS) {
                // re-establish TCP connection
                tcpClose(); 
                netState = NET_DISCONNECTED;
              } else {
                netState = NET_CONNECTED;
              }
            } else if (ret == 2) {
              // timeout
              Serial.print("Timeout");
              netState = NET_HTTP_ERROR;
            }
            } break;
        case NET_DISCONNECTED:
            // TCP disconnected, attempt to connect again
            if (state & STATE_CONNECTED) {
              xbPurge();
              tcpConnect();
              netState = NET_CONNECTING;            
              connCount = 0;
            }
            break;
        case NET_HTTP_ERROR:
            // oops, we got an error
            Serial.println(buffer);
            // check if there are too many connection errors
            if (++connErrors >= MAX_ERRORS_RECONNECT || strstr_P(buffer, PSTR("link is not"))) {
              // reset WIFI
              tcpClose();
              if (connErrors >= MAX_ERRORS_RESET) {
                state &= ~STATE_CONNECTED;
                Serial.println("Reset WIFI...");
                netDisconnect();
                netReset();
                delay(1000);
                if (netInit() && netSetup()) {
                  state |= STATE_CONNECTED;
                  connErrors = 0;
                }
              }
              netState = NET_DISCONNECTED;
            } else {
              netState = NET_CONNECTED;
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
          Serial.println("No GPS data");
          delay(100);
        }
    }
    void reconnect()
    {
        // try to re-connect to OBD
        if (init()) return;
        delay(1000);
        if (init()) return;
        standby();
    }
    void standby()
    {
        if (state & STATE_NET_READY) {
          tcpClose();
          regDataFeed(1); // de-register
          netDisconnect(); // disconnect from AP
          delay(500);
          netReset();
        }
#if USE_GPS
        initGPS(0); // turn off GPS power
#endif
        state &= ~(STATE_OBD_READY | STATE_GPS_READY | STATE_NET_READY | STATE_CONNECTED);
        Serial.print("Standby");
        // put OBD chips into low power mode
        enterLowPowerMode();
        // sleep for serveral seconds
        for (byte n = 0; n < 30; n++) {
          Serial.print('.');
          readMEMS();
          sleepms(250);
        }
        calibrateMEMS();
        for (;;) {
          accSum[0] = 0;
          accSum[1] = 0;
          accSum[2] = 0;
          for (accCount = 0; accCount < 10; ) {
            readMEMS();
            sleepms(30);
          }
          // calculate relative movement
          unsigned long motion = 0;
          for (byte i = 0; i < 3; i++) {
            long n = accSum[i] / accCount - accCal[i];
            motion += n * n;
          }
          // check movement
          if (motion > START_MOTION_THRESHOLD) {
            Serial.println(motion);
            // try OBD reading
            leaveLowPowerMode();
            if (init()) {
              // OBD is accessible
              break;
            }
            enterLowPowerMode();
            // calibrate MEMS again in case the device posture changed
            calibrateMEMS();
          }
        }
        // now we are able to get OBD data again
        // reset device
        void(* resetFunc) (void) = 0; //declare reset function at address 0
        resetFunc();
    }
    void calibrateMEMS()
    {
        // get accelerometer calibration reference data
        accCal[0] = accSum[0] / accCount;
        accCal[1] = accSum[1] / accCount;
        accCal[2] = accSum[2] / accCount;
    }
    void readMEMS()
    {
        // load accelerometer and temperature data
        int acc[3] = {0};
        int temp; // device temperature (in 0.1 celcius degree)
        memsRead(acc, 0, 0, &temp);
        if (accCount >= 250) {
          accSum[0] >>= 1;
          accSum[1] >>= 1;
          accSum[2] >>= 1;
          accCount >>= 1;
        }
        accSum[0] += acc[0];
        accSum[1] += acc[1];
        accSum[2] += acc[2];
        accCount++;
        deviceTemp = temp / 10;
    }
    void dataIdleLoop()
    {
      // do something while waiting for data on SPI
      if (state & STATE_MEMS_READY) {
        readMEMS();
      }
      delay(20);
    }
private:
    byte state;
    uint16_t feedid;
    uint16_t connCount;
    byte connErrors;
    byte netState;
};

CTeleLogger logger;

void setup()
{
    // initialize hardware serial (for USB and BLE)
    Serial.begin(115200);
    delay(500);
    // perform initializations
    logger.setup();
}

void loop()
{
    logger.loop();
}
