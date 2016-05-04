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

static uint16_t lastUTC = 0;
static uint8_t lastGPSDay = 0;
static uint32_t nextConnTime = 0;
static uint16_t connCount = 0;

typedef enum {
    GPRS_DISABLED = 0,
    GPRS_READY,
    GPRS_HTTP_CONNECTING,
    GPRS_HTTP_RECEIVING,
    GPRS_HTTP_ERROR,
} GPRS_STATES;

typedef enum {
  HTTP_GET = 0,
  HTTP_POST,
} HTTP_METHOD;

typedef struct {
  float lat;
  float lon;
  uint8_t year; /* year past 2000, e.g. 15 for 2015 */
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} GSM_LOCATION;

class COBDGSM : public COBDSPI {
public:
    COBDGSM():gprsState(GPRS_DISABLED),connErrors(0) { buffer[0] = 0; }
    void toggleGSM()
    {
        setTarget(TARGET_OBD);
        sendCommand("ATGSMPWR\r", buffer, sizeof(buffer));
    }
    bool initGSM()
    {
      // init xBee module serial communication
      xbBegin();
      // discard any stale data
      xbPurge();
      for (;;) {
        // try turning on GSM
        toggleGSM();
        delay(3000);
        if (sendGSMCommand("ATE0\r") != 0) {
          break;
        }
      }
      //sendGSMCommand("ATE0\r");
    }
    bool setupGPRS(const char* apn)
    {
      while (sendGSMCommand("AT+CREG?\r", 5000, "+CREG: 0,") == 0) {
        Serial.print('.'); 
      }
      sendGSMCommand("AT+CGATT?\r");
      sendGSMCommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r");
      sprintf(buffer, "AT+SAPBR=3,1,\"APN\",\"%s\"\r", apn);
      sendGSMCommand(buffer, 15000);
      do {
        sendGSMCommand("AT+SAPBR=1,1\r", 5000);
        sendGSMCommand("AT+SAPBR=2,1\r", 5000);
      } while (strstr(buffer, "0.0.0.0") || strstr(buffer, "ERROR"));
      //Serial.println(buffer);
      return true;
    }
    int getSignal()
    {
        if (sendGSMCommand("AT+CSQ\r", 500)) {
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
        if (sendGSMCommand("AT+COPS?\r") == 1) {
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
    void httpUninit()
    {
      sendGSMCommand("AT+HTTPTERM\r");
    }
    bool httpInit()
    {
      if (!sendGSMCommand("AT+HTTPINIT\r", 3000) || !sendGSMCommand("AT+HTTPPARA=\"CID\",1\r", 3000)) {
        gprsState = GPRS_DISABLED;
        return false;
      }
      gprsState = GPRS_READY;
      return true;
    }
    void httpConnect(HTTP_METHOD method)
    {
        // 0 for GET, 1 for POST
        char cmd[17];
        sprintf(cmd, "AT+HTTPACTION=%c\r", '0' + method);
        setTarget(TARGET_BEE);
        write(cmd);
        gprsState = GPRS_HTTP_RECEIVING;
        bytesRecv = 0;
        checkTimer = millis();
    }
    bool httpIsConnected()
    {
        // may check for "ACTION: 0" for GET and "ACTION: 1" for POST
        byte ret = checkbuffer("ACTION:", MAX_CONN_TIME);
        if (ret == 1) {
          // success
          connErrors = 0;
          return true;
        } else if (ret == 2) {
          // timeout
          gprsState = GPRS_HTTP_ERROR;
          connErrors++;
        }
        return false;
    }
    bool httpRead()
    {
        if (sendGSMCommand("AT+HTTPREAD\r", MAX_CONN_TIME) && strstr(buffer, "+HTTPREAD:")) {
          gprsState = GPRS_READY;
          return true;
        } else {
          Serial.println("READ ERROR");
          gprsState = GPRS_HTTP_ERROR;
          return false;
        }
    }
    bool getLocation(GSM_LOCATION* loc)
    {
      if (sendGSMCommand("AT+CIPGSMLOC=1,1\r", 1000)) do {
        char *p;
        if (!(p = strchr(buffer, ':'))) break;
        if (!(p = strchr(p, ','))) break;
        loc->lon = atof(++p);
        if (!(p = strchr(p, ','))) break;
        loc->lat = atof(++p);
        if (!(p = strchr(p, ','))) break;
        loc->year = atoi(++p) - 2000;
        if (!(p = strchr(p, '/'))) break;
        loc->month = atoi(++p);
        if (!(p = strchr(p, '/'))) break;
        loc->day = atoi(++p);
        if (!(p = strchr(p, ','))) break;
        loc->hour = atoi(++p);
        if (!(p = strchr(p, ':'))) break;
        loc->minute = atoi(++p);
        if (!(p = strchr(p, ':'))) break;
        loc->second = atoi(++p);
        return true;
      } while(0);
      return false;
    }
    byte checkbuffer(const char* expected, unsigned int timeout = 2000)
    {
      byte ret = xbReceive(buffer, sizeof(buffer), 0, expected) != 0;
      if (ret == 0) {
        // timeout
        return (millis() - checkTimer < timeout) ? 0 : 2;
      } else {
        return ret;
      }
    }
    bool sendGSMCommand(const char* cmd, unsigned int timeout = 2000, const char* expected = "OK")
    {
      xbWrite(cmd);
      delay(10);
      return xbReceive(buffer, sizeof(buffer), timeout, expected) != 0;
    }
    bool setPostPayload(const char* payload, int bytes)
    {
        // set HTTP POST payload data
        char cmd[24];
        sprintf(cmd, "AT+HTTPDATA=%d,1000\r", bytes);
        if (!sendGSMCommand(cmd, 1000, "DOWNLOAD")) {
          return false;
        }
        // send cached data
        return sendGSMCommand(payload, 1000);
    }
    char buffer[128];
    byte bytesRecv;
    uint32_t checkTimer;
    byte gprsState;
    byte connErrors;
};

class CTeleLogger : public COBDGSM, public CDataLogger
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
        Serial.print("#GPS...");
        if (initGPS(GPS_SERIAL_BAUDRATE)) {
          state |= STATE_GPS_READY;
          Serial.println("OK");
        } else {
          Serial.println("NO");
        }
#endif

        Serial.print("#GSM...");
        if (initGSM()) {
            Serial.println("OK");
        } else {
            Serial.println(buffer);
        }

        Serial.print("#GPRS...");
        delay(100);
        if (setupGPRS(APN)) {
            Serial.println("OK");
        } else {
            Serial.println(buffer);
        }
        
        // init HTTP
        Serial.print("#HTTP...");
        while (!httpInit()) {
          Serial.print('.');
          httpUninit();
          delay(1000);
        }
        Serial.println("OK");

        joinChannel(0);
        state |= STATE_CONNECTED;
        
        Serial.println();
        delay(1000);
    }
    void joinChannel(byte action)
    {
      int signal;
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
        Serial.print("#SIGNAL:");
        signal = getSignal();
        Serial.println(signal);
      }
      gprsState = GPRS_READY;
      for (;;) {
        char *p = buffer;
        p += sprintf(buffer, "AT+HTTPPARA=\"URL\",\"%s/push?", HOST_URL);
        if (action == 0) {
          Serial.print("#SERVER:"); 
          sprintf(p, "CSQ=%d&VIN=%s\"\r", signal, vin);
        } else {
          sprintf(p, "id=%d&OFF=1\"\r", channel);
        }
        if (!sendGSMCommand(buffer)) {
          Serial.println(buffer);
          continue;
        }
        httpConnect(HTTP_GET);
        do {
          delay(500);
          Serial.print('.');
        } while (!httpIsConnected() && action == 0);
        if (action != 0) return;
        if (gprsState != GPRS_HTTP_ERROR && httpRead()) {
          char *p = strstr(buffer, "CH:");
          if (p) {
            int m = atoi(p + 3);
            if (m > 0) {
              channel = m;
              Serial.print(m);
              state |= STATE_CONNECTED;
              break;
            }
          }            
        }
        Serial.println("Error");
        Serial.println(buffer);
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
        }
#endif

#if USE_MPU6050
        if (state & STATE_MEMS_READY) {
            processMEMS();  
        }
#endif

        if (millis() > nextConnTime) {
          processGPRS();
        } else {
#if ENABLE_DATA_LOG
          flushData();
#endif
        }
        if (connErrors >= MAX_CONN_ERRORS) {
          // reset GPRS 
          Serial.print(connErrors);
          Serial.println("Reset GPRS...");
          initGSM();
          setupGPRS(APN);
          if (httpInit()) {
            Serial.println("OK"); 
          } else {
            Serial.println(buffer); 
          }
          connErrors = 0;
        }
    }
private:
    void processGPRS()
    {
        switch (gprsState) {
        case GPRS_READY:
            if (state & STATE_CONNECTED) {
                // generate URL
                sprintf(buffer, "AT+HTTPPARA=\"URL\",\"%s/post?id=%u\"\r", HOST_URL, channel);
                if (!sendGSMCommand(buffer)) {
                  break;
                }
                // add required terminating '\r'
                cache[cacheBytes] = '\r';
                cache[cacheBytes + 1] = 0;
                if (setPostPayload(cache, cacheBytes)) {
                  // success
                  Serial.print("POST:");
                  Serial.println(cacheBytes);
                  gprsState = GPRS_HTTP_CONNECTING;
                  cacheBytes = 0;
                } else {
                  Serial.println("POST FAIL");
                  Serial.println(buffer);
                  gprsState = GPRS_HTTP_ERROR;
                  nextConnTime = millis() + 1000; 
                }
            }
            break;        
        case GPRS_HTTP_CONNECTING:
            Serial.print("CONNECT#");
            Serial.println(++connCount);
            httpConnect(HTTP_POST);
            nextConnTime = millis() + 2000;
            break;
        case GPRS_HTTP_RECEIVING:
            if (httpIsConnected()) {
                if (httpRead()) {
                  // success
                  Serial.println("SUCCESS");
                  //Serial.println(buffer);
                  break;
                }
            }
            nextConnTime = millis() + 200; 
            break;
        case GPRS_HTTP_ERROR:
            Serial.println("HTTP ERROR");
            Serial.println(buffer);
            connCount = 0;
            xbPurge();
            httpUninit();
            delay(500);
            httpInit();
            gprsState = GPRS_READY;
            nextConnTime = millis() + 500;
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
              //logData(PID_GPS_SAT_COUNT, gd.sat);
              lastUTC = (uint16_t)gd.time;
            }
            //Serial.print("#UTC:"); 
            //Serial.println(gd.time);
        } else {
          Serial.println("GPS Error");
          delay(1000);
        }
    }
    void reconnect()
    {
        if (init()) {
          // reconnected
          return; 
        }
        Serial.print("Sleeping");
        state &= ~STATE_OBD_READY;
        joinChannel(1); // leave channel
        toggleGSM(); // turn off GSM power
#if USE_GPS
        initGPS(0); // turn off GPS power
#endif
        Serial.println();
        state |= STATE_SLEEPING;
        for (;;) {
            int value;
            if (read(PID_RPM, value))
                break;
            lowPowerMode();
            sleep(4);
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
