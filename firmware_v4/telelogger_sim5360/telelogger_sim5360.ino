/******************************************************************************
* Reference sketch for a vehicle data feed for Freematics Hub
* Works with Freematics ONE with SIM800 GSM/GPRS module
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

uint16_t lastUTC = 0;
uint8_t lastGPSDay = 0;
uint32_t nextConnTime = 0;
uint16_t connCount = 0;
#if USE_MPU6050 || USE_MPU9250
byte accCount = 0; // count of accelerometer readings
long accSum[3] = {0}; // sum of accelerometer data
int accCal[3] = {0}; // calibrated reference accelerometer data
byte deviceTemp = 0; // device temperature
#endif
int lastSpeed = 0;
uint32_t lastSpeedTime = 0;
uint32_t distance = 0;

typedef enum {
    NET_DISCONNECTED = 0,
    NET_CONNECTED,
    NET_RECEIVING,
    NET_HTTP_ERROR,
} NET_STATES;

typedef enum {
  HTTP_GET = 0,
  HTTP_POST,
} HTTP_METHOD;

class COBD3G : public COBDSPI {
public:
    COBD3G() { buffer[0] = 0; }
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
        sprintf_P(buffer, PSTR("AT+CHTTPSOPSE=\"%s\",%u,1\r"), SERVER_URL, SERVER_PORT);
        //Serial.println(buffer);
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
        checkTimer = millis();
        return true;        
      } else {
        Serial.println(buffer);
        return false;
      }
    }
    bool httpReceive()
    {
        checkbuffer("RECV EVENT", MAX_CONN_TIME);
        if (netSendCommand("AT+CHTTPSRECV=384\r", MAX_CONN_TIME, "+CHTTPSRECV: 0")) {
          return true;
        } else {
          //Serial.println("READ ERROR");
          return false;
        }
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
        return (millis() - checkTimer < timeout) ? 0 : 2;
      } else {
        return ret;
      }
    }
    bool netSendCommand(const char* cmd, unsigned int timeout = 2000, const char* expected = "\r\nOK\r\n")
    {
      if (cmd) {
        xbWrite(cmd);
      }
      buffer[0] = 0;
      return xbReceive(buffer, sizeof(buffer), timeout, expected) != 0;
    }
#ifdef ESP32
    char buffer[512];
#else
    char buffer[192];
#endif
    uint32_t checkTimer;
};

class CTeleLogger : public COBD3G, public CDataLogger
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

        begin();

        for (;;) {
          // initialize OBD communication
          Serial.print("#OBD...");
          if (init()) {
            Serial.println("OK");
          } else {
            Serial.println("NO");
            reconnect();
            continue;
          }
          state |= STATE_OBD_READY;
  
#if USE_GPS
          // start serial communication with GPS receive
          Serial.print("#GPS...");
          if (initGPS(GPS_SERIAL_BAUDRATE)) {
            state |= STATE_GPS_READY;
#ifdef ESP32
            Serial.print("OK(");
            Serial.print(internalGPS() ? "internal" : "external");
            Serial.println(')');
#else
            Serial.println("OK");
#endif    
            waitGPS();  
          } else {
            Serial.println("NO");
          }
#endif

          // initialize SIM5360 xBee module (if present)
          Serial.print("#SIM5360...");
          xbBegin(XBEE_BAUDRATE);
          if (netInit()) {
            Serial.println("OK");
            state |= STATE_NET_READY;
          } else {
            Serial.println("NO");
            standby();
            continue;
          }
          
          Serial.print("#3G(APN:");
          Serial.print(APN);
          Serial.print(")...");
          if (netSetup(APN)) {
            Serial.println("OK");
            state |= STATE_CONNECTED;
          } else {
            Serial.println("NO");
            standby();
            continue;
          }
  
          Serial.print("#HTTP...");
          if (httpOpen()) {
            Serial.println("OK");
          } else {
            Serial.println("NO");
            standby();
            continue;
          }
  
          // sign in server
          if (!regDataFeed(0)) {
            standby();
            continue;
          }
  
  #if USE_MPU6050 || USE_MPU9250
          calibrateMEMS();
  #endif
  
          if (!(state & STATE_CONNECTED)) {
            standby();
            continue;
          }
          break;
        }

#if USE_GPS
        if (state & STATE_GPS_READY) {
          waitGPS();
        }
#endif
    }
    bool regDataFeed(byte action)
    {
      // action == 0 for registering a data feed, action == 1 for de-registering a data feed

      // retrieve VIN
      char vin[20] = {0};
      if (action == 0) {
        // retrieve VIN
        if (getVIN(buffer, sizeof(buffer))) {
          strncpy(vin, buffer, sizeof(vin) - 1);
          Serial.print("#VIN:");
          Serial.println(vin);
        } else {
          strcpy(vin, "DEFAULT_VIN");
        }
      } else {
        if (feedid == 0) return false; 
      }

      Serial.print("#SERVER...");
      for (byte n = 0; n < 3; n++) {
        // make sure OBD is still accessible
        if (readSpeed() == -1) {
          reconnect(); 
        }
        // start a HTTP connection (TCP)
        httpConnect();
        Serial.println(buffer);

        // generate HTTP request path
        char path[64];
        if (action == 0) {
          sprintf_P(path, PSTR("/%s/reg?vin=%s"), SERVER_KEY, vin);
        } else {
          sprintf_P(path, PSTR("/%s/reg?id=%d&off=1"), SERVER_KEY, feedid);
        }
        // send HTTP request
        if (!httpSend(HTTP_GET, path, true)) {
          Serial.println("Error sending");
          httpClose();
          sleep(3000);
          continue;
        }

        // receive and parse response
        if (!httpReceive() || checkbuffer("\"id\"",MAX_CONN_TIME)!=1) {
          httpClose();
          continue;
        }
        Serial.print(buffer);
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
              return true;
            }
          }
        } else {
          httpClose();
          break; 
        }
        Serial.println(buffer);
        httpClose();
        sleep(3000);
      }
      return false;
    }
    void loop()
    {
        logTimestamp();

        if (state & STATE_OBD_READY) {
          processOBD();
        }

#if CACHE_SIZE >= 256
#if USE_MPU6050 || USE_MPU9250
        if (state & STATE_MEMS_READY) {
            processMEMS();  
        }
#endif
#endif

        if (state & STATE_GPS_READY) {
#if USE_GPS
          processGPS();
#endif
        }

#if CACHE_SIZE >= 256
        // read and log car battery voltage , data in 0.01v
        {
          int v = getVoltage() * 100;
          logData(PID_BATTERY_VOLTAGE, v);
        }
#endif

        // process HTTP transaction
        if (state & STATE_CONNECTED) {
          processHTTP();
        }

        if (errors > 10) {
          reconnect();
        } 
#if USE_MPU6050 || USE_MPU9250
        if (deviceTemp >= COOLING_DOWN_TEMP && deviceTemp < 100) {
          // device too hot, cool down
          Serial.print("Cooling (");
          Serial.print(deviceTemp);
          Serial.println("C)");
          sleep(10000);
        }
#endif
    }
private:
    void processHTTP()
    {
        if (netState != NET_CONNECTED) {
            xbPurge();
            if (httpConnect()) {
              Serial.println("Reconnected");              
              netState = NET_CONNECTED;            
            }
            connCount = 0;
        }

        if (cacheBytes == 0) return;

        // and there is data in cache to send
        char path[64];
        sprintf_P(path, PSTR("/%s/post?id=%u"), SERVER_KEY, feedid);
        // send HTTP POST request with cached data as payload
        cache[cacheBytes++]='\r';
        cache[cacheBytes]=0;
        if (httpSend(HTTP_POST, path, true, cache, cacheBytes)) {
          // success
          Serial.println(cache);
          Serial.print(cacheBytes);
          Serial.println(" bytes sent");
          //Serial.println(cache);
          purgeCache();
          Serial.println("Receiving...");  
          if (!httpReceive()) {
            Serial.println(buffer);
            netState = NET_HTTP_ERROR;
          } else {
            netState = NET_CONNECTED;
          }
        } else {
          Serial.println("Request error");
          netState = NET_HTTP_ERROR;
        }

        if (netState != NET_HTTP_ERROR) {
            // in the progress of data receiving
            if (strstr(buffer, "\"result\"") || checkbuffer("\"result\"",MAX_CONN_TIME) == 1) {
              // success
              connCount++;
              connErrors = 0;
              Serial.print("Success #");
              Serial.println(connCount);
              //Serial.println(buffer);
              if (connCount >= MAX_HTTP_CONNS) {
                // re-establish TCP connection
                httpClose(); 
                netState = NET_DISCONNECTED;
              }
            } else {
              Serial.println("Invalid response");
              netState = NET_HTTP_ERROR;
            }
        }
        if (netState == NET_HTTP_ERROR) {
            // oops, we got an error
            httpClose();
            if (++connErrors > MAX_ERRORS_RESET) {
              state &= ~(STATE_CONNECTED | STATE_NET_READY);
              netState = NET_DISCONNECTED;
              Serial.println("Reset 3G...");
              xbTogglePower();
              // check if OBD is still accessible
              if (readSpeed() == -1) {
                standby();
              }
              if (netInit() && netSetup(APN)) {
                state |= STATE_CONNECTED;
                connErrors = 0;
              } else {
                standby();
              }
            }
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
           uint32_t t = millis();
           distance += (value + lastSpeed) * (t - lastSpeedTime) / 3600 / 2;
           lastSpeedTime = t;
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
        } else {
          Serial.println("No GPS data");
          sleep(100);
        }
    }
    void waitGPS()
    {
          int elapsed = 0;
          for (uint32_t t = millis(); millis() - t < 300000;) {
            int t1 = (millis() - t) / 1000;
            if (t1 != elapsed) {
              Serial.print("Waiting for GPS (");
              Serial.print(elapsed);
              Serial.println(")");
              elapsed = t1;
            } else {
              decodeGPSData();
              continue;
            }
            GPS_DATA gd = {0};
            // read parsed GPS data
            if (getGPSData(&gd) && gd.sat != 0 && gd.sat != 255) {
              Serial.print("SAT:");
              Serial.println(gd.sat);
              break;
            }
          }
    }
    void reconnect()
    {
        // try to re-connect to OBD
        if (init()) return;
        sleep(1000);
        if (init()) return;
        standby();
    }
    void standby()
    {
        if (state & STATE_NET_READY) {
          regDataFeed(1); // de-register
          xbTogglePower(); // turn off GSM power
        }
#if USE_GPS
        initGPS(0); // turn off GPS power
#endif
        state &= ~(STATE_OBD_READY | STATE_GPS_READY | STATE_NET_READY | STATE_CONNECTED);
        Serial.println("Standby");
        enterLowPowerMode();
#if USE_MPU6050 || USE_MPU9250
        calibrateMEMS();
        if (state & STATE_MEMS_READY) {
          for (;;) {
            sleep(1000);
            // calculate relative movement
            unsigned long motion = 0;
            for (byte i = 0; i < 3; i++) {
              long n = accSum[i] / accCount - accCal[i];
              motion += n * n;
            }
            // check movement
            if (motion > WAKEUP_MOTION_THRESHOLD) {
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
        } else {
          while (!init()) sleepSec(10);
        }
#else
        while (!init()) sleepSec(10);
#endif
        Serial.println("Wakeup");
        leaveLowPowerMode();
    }
#if USE_MPU6050 || USE_MPU9250
    void calibrateMEMS()
    {
        // store accelerometer reference data
        if ((state & STATE_MEMS_READY) && accCount) {
          accCal[0] = accSum[0] / accCount;
          accCal[1] = accSum[1] / accCount;
          accCal[2] = accSum[2] / accCount;
        }
    }
    void readMEMS()
    {
        // load accelerometer and temperature data
        int16_t acc[3] = {0};
        int16_t temp; // device temperature (in 0.1 celcius degree)
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
#endif
    void dataIdleLoop()
    {
      // do tasks while waiting for data on SPI
#if USE_MPU6050 || USE_MPU9250
      if (state & STATE_MEMS_READY) {
        readMEMS();
      }
#endif
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
    Serial.println("Freematics ONE");
    delay(1000);
    // perform initializations
    logger.setup();
}

void loop()
{
    logger.loop();
}
