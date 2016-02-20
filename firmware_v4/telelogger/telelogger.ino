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
#include <I2Cdev.h>
#include <MPU9150.h>
#include <SPI.h>
#include <Narcoleptic.h>
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
#define SerialRF Serial
#endif

static uint16_t lastUTC = 0;
static uint8_t lastGPSDay = 0;
static uint32_t nextConnTime = 0;
static uint16_t connCount = 0;

const byte PROGMEM pidTier1[]= {PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_THROTTLE};
const byte PROGMEM pidTier2[] = {PID_INTAKE_TEMP, PID_COOLANT_TEMP};

#define TIER_NUM1 sizeof(pidTier1)
#define TIER_NUM2 sizeof(pidTier2)

#if USE_MPU6050
MPU6050 accelgyro;
#endif

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
      // check GSM
      xbBegin();
      for (;;) {
        // try turning on GSM
        //Serial.print("Turn on GSM...");
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
      } while (strstr(buffer, "0.0.0.0"));
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
        byte n = xbRead(buffer + bytesRecv, sizeof(buffer) - bytesRecv, timeout);
        if (n > 0) {
            if (memcmp(buffer + bytesRecv, "$GSMNO DATA", 11)) {
              //Serial.print(buffer + bytesRecv);
              bytesRecv += n;
              if (strstr(buffer, expected)) {
                  return 1;
              }
              if (bytesRecv >= sizeof(buffer) - 1) {
                  // buffer full, discard first half
                  bytesRecv = sizeof(buffer) / 2 - 1;
                  memcpy(buffer, buffer + sizeof(buffer) / 2, bytesRecv);
              }
            }
        }
        return (millis() - checkTimer < timeout) ? 0 : 2;
    }
    byte sendGSMCommand(const char* cmd, unsigned int timeout = 2000, const char* expected = 0)
    {
      if (cmd) {
        xbSend(cmd);
        delay(10);
      }
      uint32_t t = millis();
      do {
        byte n = xbRead(buffer, sizeof(buffer), timeout);
        if (n > 0) {
          if (strstr(buffer, expected ? expected : "OK")) {
            return n;
          }
        }
      } while (millis() - t < timeout);
      return 0;
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
        
        SerialRF.begin(115200);

        // this will init SPI communication
        begin(7, 6);

        SerialRF.print("#OBD..");
        setTarget(TARGET_OBD);
        do {
            SerialRF.print('.');
        } while (!init());
        SerialRF.println("OK");
        state |= STATE_OBD_READY;

        SerialRF.print("#GSM...");
        if (initGSM()) {
            SerialRF.println("OK");
        } else {
            SerialRF.println(buffer);
        }

#if USE_MPU6050
        SerialRF.print("#MEMS...");
        Wire.begin();
        accelgyro.initialize();
        if (accelgyro.testConnection()) {
          state |= STATE_MEMS_READY;
          SerialRF.print("OK");
        }
        SerialRF.println();
#endif

#if USE_GPS
        delay(100);
        if (initGPS(GPS_SERIAL_BAUDRATE)) {
          state |= STATE_GPS_READY;
          SerialRF.println("#GPS...OK");
        }
#endif

        SerialRF.print("#GPRS...");
        delay(100);
        if (setupGPRS(APN)) {
            SerialRF.println("OK");
        } else {
            SerialRF.println(buffer);
        }
        
        // init HTTP
        SerialRF.print("#HTTP...");
        while (!httpInit()) {
          SerialRF.print('.');
          httpUninit();
          delay(1000);
        }
        SerialRF.println("OK");

        joinChannel(0);
        state |= STATE_CONNECTED;
        
        SerialRF.println();
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
        SerialRF.print("#VIN:");
        SerialRF.println(vin);
        SerialRF.print("#SIGNAL:");
        signal = getSignal();
        SerialRF.println(signal);
      }
      gprsState = GPRS_READY;
      for (;;) {
        char *p = buffer;
        p += sprintf(buffer, "AT+HTTPPARA=\"URL\",\"%s/push?", HOST_URL);
        if (action == 0) {
          SerialRF.print("#CHANNEL:"); 
          sprintf(p, "CSQ=%d&VIN=%s\"\r", signal, vin);
        } else {
          sprintf(p, "id=%d&OFF=1\"\r", channel);
        }
        if (!sendGSMCommand(buffer)) {
          SerialRF.println(buffer);
          continue;
        }
        httpConnect(HTTP_GET);
        do {
          delay(500);
          SerialRF.print('.');
        } while (!httpIsConnected());
        if (action != 0) return;
        if (gprsState != GPRS_HTTP_ERROR && httpRead()) {
          char *p = strstr(buffer, "CH:");
          if (p) {
            int m = atoi(p + 3);
            if (m > 0) {
              channel = m;
              SerialRF.print(m);
              state |= STATE_CONNECTED;
              break;
            }
          }            
        }
        SerialRF.println("Error");
        SerialRF.println(buffer);
      }
    }
    void loop()
    {
        // poll OBD-II PIDs
        int value = 0;
        setTarget(TARGET_OBD);
        for (byte index = 0; index < TIER_NUM1; index++) {
          byte pid = pgm_read_byte(pidTier1 + index);
          if (read(pid, value)) {
              dataTime = millis();
              logData(0x100 | pid, value);
          }
        }
        if (errors >= 2) {
            reconnect();
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

        static byte index2 = 0;
        byte pid = pgm_read_byte(pidTier2 + index2);
        if (read(pid, value)) {
          logData(0x100 | pid, value);
        }
        index2 = (index2 + 1) % TIER_NUM2;

        if (millis() > nextConnTime) {
          processGPRS();
        } else {
#if ENABLE_DATA_LOG
          flushData();
#endif
        }
        if (connErrors >= MAX_CONN_ERRORS) {
          // reset GPRS 
          SerialRF.print(connErrors);
          SerialRF.println("Reset GPRS...");
          xbPurge();
          initGSM();
          setupGPRS(APN);
          if (httpInit()) {
            SerialRF.println("OK"); 
          } else {
            SerialRF.println(buffer); 
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
                sprintf(buffer, "AT+HTTPPARA=\"URL\",\"%s/post?id=%d\"\r", HOST_URL, channel);
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
                  delay(100);
                } else {
                  SerialRF.println("POST FAIL");
                  SerialRF.println(buffer);
                  nextConnTime = millis() + 1000; 
                }
            }
            break;        
        case GPRS_HTTP_CONNECTING:
            SerialRF.print("CONNECT#");
            SerialRF.println(++connCount);
            httpConnect(HTTP_POST);
            nextConnTime = millis() + 2000;
            break;
        case GPRS_HTTP_RECEIVING:
            if (httpIsConnected()) {
                if (httpRead()) {
                  // success
                  SerialRF.println("SUCCESS");
                  //SerialRF.println(buffer);
                } else {
                  delay(100);  
                }
            } else {
              nextConnTime = millis() + 200; 
            }
            break;
        case GPRS_HTTP_ERROR:
            SerialRF.println("HTTP ERROR");
            SerialRF.println(buffer);
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
          int16_t ax = 0, ay = 0, az = 0;
          int16_t gx = 0, gy = 0, gz = 0;
          accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
          dataTime = millis();
          // log x/y/z of accelerometer
          logData(PID_ACC, ax / ACC_DATA_RATIO, ay / ACC_DATA_RATIO, az / ACC_DATA_RATIO);
          // log x/y/z of gyro meter
          logData(PID_GYRO, gx / GYRO_DATA_RATIO, gy / GYRO_DATA_RATIO, gz / GYRO_DATA_RATIO);
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
        SerialRF.print("Sleeping");
        state &= ~STATE_OBD_READY;
        joinChannel(1); // leave channel
        toggleGSM(); // turn off GSM power
#if USE_GPS
        initGPS(0); // turn off GPS power
#endif
        SerialRF.println();
        state |= STATE_SLEEPING;
        for (;;) {
            int value;
            if (read(PID_RPM, value))
                break;
            Narcoleptic.delay(3000);
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
