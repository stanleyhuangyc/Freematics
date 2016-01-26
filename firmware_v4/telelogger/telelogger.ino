/******************************************************************************
* Vehicle Telematics Data Logger Sketch for Freematics ONE
* Developed by Stanley Huang <stanleyhuangyc@gmail.com>
* Distributed under GPL v2.0
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
const byte PROGMEM pidTier2[] = {PID_INTAKE_MAP, PID_INTAKE_TEMP, PID_COOLANT_TEMP};

#define TIER_NUM1 sizeof(pidTier1)
#define TIER_NUM2 sizeof(pidTier2)

int pidValue[TIER_NUM1] = {0};
int pidValueTier2[sizeof(pidTier2)] = {0};

#if USE_MPU6050
static int16_t ax = 0, ay = 0, az = 0;
static int16_t gx = 0, gy = 0, gz = 0;
static int temp = 0;
#endif

#if USE_MPU6050
MPU6050 accelgyro;
static uint32_t lastMemsDataTime = 0;
#endif

typedef enum {
    GPRS_DISABLED = 0,
    GPRS_READY,
    GPRS_HTTP_CONNECTING,
    GPRS_HTTP_RECEIVING,
    GPRS_HTTP_ERROR,
} GPRS_STATES;

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
    COBDGSM():gprsState(GPRS_DISABLED) { buffer[0] = 0; }
    void toggleGSM()
    {
        setTarget(TARGET_OBD);
        sendCommand("ATGSMPWR\r", buffer, sizeof(buffer));
    }
    bool initGSM()
    {
      // check GSM
      setTarget(TARGET_OBD);
      sendCommand("ATCLRGSM\r", buffer, sizeof(buffer));
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
    void httpConnect()
    {
        // Starts GET action
        setTarget(TARGET_BEE);
        write("AT+HTTPACTION=1\r");
        gprsState = GPRS_HTTP_RECEIVING;
        bytesRecv = 0;
        checkTimer = millis();
    }
    bool httpIsConnected()
    {
        byte ret = checkbuffer("OK", 10000);
        if (ret == 1) {
          if (strstr(buffer, ": 1,6")) {
            gprsState = GPRS_HTTP_ERROR;
          } else {
            return strstr(buffer, ": 1,") != 0;
          }
        } else if (ret == 2) {
          // timeout
          gprsState = GPRS_HTTP_ERROR;
        }
        return false;
    }
    bool httpRead()
    {
        if (sendGSMCommand("AT+HTTPREAD\r", 5000) && strstr(buffer, "+HTTPREAD:")) {
          gprsState = GPRS_READY;
          return true;
        } else {
          Serial.println("Read error");
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
        setTarget(TARGET_OBD);
        write("ATGRD\r");
        delay(10);
        byte n = receive(buffer + bytesRecv, sizeof(buffer) - bytesRecv, timeout);
        if (n > 0) {
            if (memcmp(buffer + bytesRecv, "$GSMNO DATA", 11)) {
              //Serial.print(buffer + bytesRecv);
              bytesRecv += n;
              if (bytesRecv >= sizeof(buffer) - 1) {
                  // buffer full, discard first half
                  bytesRecv = sizeof(buffer) / 2 - 1;
                  memcpy(buffer, buffer + sizeof(buffer) / 2, bytesRecv);
              }
              if (strstr(buffer, expected)) {
                  return 1;
              }
            }
        }
        return (millis() - checkTimer < timeout) ? 0 : 2;
    }
    byte sendGSMCommand(const char* cmd, unsigned int timeout = 2000, const char* expected = 0)
    {
      if (cmd) {
        setTarget(TARGET_BEE);
        write(cmd);
        delay(10);
      }
      setTarget(TARGET_OBD);
      uint32_t t = millis();
      do {
        write("ATGRD\r");
        delay(50);
        byte n = receive(buffer, sizeof(buffer), timeout);
        if (n > 0) {
          if (strstr(buffer, expected ? expected : "OK")) {
            return n;
          }
        }
      } while (millis() - t < timeout);
      return 0;
    }
    char buffer[64];
    byte bytesRecv;
    uint32_t checkTimer;
    byte gprsState;
};

class CTeleLogger : public COBDGSM, public CDataLogger
{
public:
    CTeleLogger():state(0),connErrors(0),channel(0) {}
    void setup()
    {
        delay(1000);
        
        SerialRF.begin(115200);

/*
//#if ENABLE_DATA_LOG
        uint16_t volsize = initSD();
        if (volsize) {
          SerialRF.print("#SD:");
          SerialRF.print(volsize);
          SerialRF.println("MB");
          //openLogFile();
        }
//#endif
*/
        begin(7, 6);
        setTarget(TARGET_OBD);


        SerialRF.print("#OBD..");
        do {
            SerialRF.print('.');
        } while (!init());
        SerialRF.println("OK");

        SerialRF.print("#GSM...");
        if (initGSM()) {
            SerialRF.println("OK");
        } else {
            SerialRF.println(buffer);
        }

        state |= STATE_OBD_READY;

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

        joinChannel();
        
        SerialRF.println();
        connErrors = 0;
        delay(1000);
    }
    void joinChannel()
    {
      char vin[256];
      getVIN(vin, sizeof(vin));
      SerialRF.print("#VIN:");
      SerialRF.println(vin);

      int signal = getSignal();
      SerialRF.print("#SIGNAL:");
      SerialRF.println(signal);

      for (;;) {
          SerialRF.print("#CHANNEL:"); 
          sprintf(buffer, "AT+HTTPPARA=\"URL\",\"%s/push\"\r", HOST_URL);
          if (!sendGSMCommand(buffer)) {
            SerialRF.println(buffer);
            continue;
          }
          if (!sendGSMCommand("AT+HTTPDATA=60,1000\r", 500, "DOWNLOAD")) {
            SerialRF.println(buffer);
            continue;
          }
          memset(buffer, ' ', 60);
          buffer[60] = '\r';
          buffer[61] = 0;
          byte len = sprintf(buffer, "VIN=%s&CSQ=%d", vin, signal);
          buffer[len] = ' ';
          if (!sendGSMCommand(buffer, 1000)) {
            SerialRF.println(buffer);
            continue;
          }

          httpConnect();
          do {
            delay(500);
            SerialRF.print('.');
          } while (!httpIsConnected());
          if (gprsState != GPRS_HTTP_ERROR && httpRead()) break;
          SerialRF.println("Error");
          SerialRF.println(buffer);
        }
        
        char *p = strstr(buffer, "CH:");
        if (p) {
          int m = atoi(p + 3);
          if (m > 0) {
            channel = m;
            SerialRF.print(m);
            state |= STATE_CONNECTED;
          }
        }
    }
    void loop()
    {
#if USE_GPS
        if (state & STATE_GPS_READY) {
          processGPS();
        }
#endif

        // poll OBD-II PIDs
        static byte index2 = 0;
        int value = 0;
        setTarget(TARGET_OBD);
        for (byte index = 0; index < TIER_NUM1; index++) {
          byte pid = pgm_read_byte(pidTier1 + index);
          if (read(pid, value)) {
              dataTime = millis();
              pidValue[index] = value;
              //SerialRF.println(value);
              logData(0x100 | pid, value);
          } else {
            //SerialRF.println("N/A"); 
          }
        }
        if (index2 == TIER_NUM2) {
            index2 = 0;
        } else {
            byte pid = pgm_read_byte(pidTier2 + index2);
            if (read(pid, value)) {
              pidValueTier2[index2] = value;
            }
            index2++;
        }

#if USE_MPU6050
        if (state & STATE_MEMS_READY) {
            processMEMS();  
        }
#endif

        if (errors >= 2) {
            reconnect();
        }
        
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
          SerialRF.println("#Reset GPRS...");
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
                cache[cacheBytes] = '\r';
                cache[cacheBytes + 1] = 0;
                sprintf(buffer, "AT+HTTPDATA=%d,1000\r", cacheBytes);
                if (!sendGSMCommand(buffer, 1000, "DOWNLOAD")) {
                  nextConnTime = millis() + 500; 
                  break;
                }

#if 0
                char *p = buffer;
                memset(buffer, ' ', 200);
                buffer[200] = '\r';
                buffer[201] = 0;
                
                p += sprintf(p, "id=%u&", channel);
                for (byte n = 0; n < sizeof(pidTier1); n++) {
                    p += sprintf(p, "%x=%d&", pgm_read_byte(pidTier1 + n), pidValue[n]);
                }
                for (byte n = 0; n < sizeof(pidTier2); n++) {
                    p += sprintf(p, "%x=%d&", pgm_read_byte(pidTier2 + n), pidValueTier2[n]);
                }
#if USE_MPU6050
                p += sprintf(p, "A=%d,%d,%d&G=%d,%d,%d&", ax, ay, az, gx, gy, gz);
                //p += sprintf(p, "&T=%d", temp);
#endif
                
                *(p - 1) = '\"';
                SerialRF.println(buffer);
                *p = '\r';
#endif

                if (sendGSMCommand(cache, 1000)) {
                  gprsState = GPRS_HTTP_CONNECTING;
                  cacheBytes = 0;
                } else {
                  SerialRF.println("POST FAIL");
                  SerialRF.println(buffer);
                  nextConnTime = millis() + 1000; 
                }
            }
            break;        
        case GPRS_HTTP_CONNECTING:
            httpConnect();
            nextConnTime = millis() + 2000;
            break;
        case GPRS_HTTP_RECEIVING:
            if (httpIsConnected()) {
                SerialRF.print("#HTTP:");
                SerialRF.println(++connCount);
                if (httpRead()) {
                  // success
                  SerialRF.println(buffer);
                }
            }
            break;
/*
        case GPRS_HTTP_READING:
            if (httpIsRead()) {
                SerialRF.print("#HTTP:");
                SerialRF.println(buffer);
                connErrors = 0;
                // ready for next connection
            }
            break;
*/
        case GPRS_HTTP_ERROR:
            SerialRF.println("#HTTP ERROR");
            SerialRF.println(buffer);
            connErrors++;
            connCount = 0;
            //sendCommand("ATCLRGSM\r", buffer, sizeof(buffer));
            httpUninit();
            delay(500);
            httpInit();
            gprsState = GPRS_READY;
            nextConnTime = millis() + 200;
            break;
        }
    }
#if USE_MPU6050
    void processMEMS()
    {
        if (dataTime - lastMemsDataTime < ACC_DATA_INTERVAL) {
            return;
        }

        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        dataTime = millis();

        temp = accelgyro.getTemperature();

        ax /= ACC_DATA_RATIO;
        ay /= ACC_DATA_RATIO;
        az /= ACC_DATA_RATIO;
        gx /= GYRO_DATA_RATIO;
        gy /= GYRO_DATA_RATIO;
        gz /= GYRO_DATA_RATIO;

        // log x/y/z of accelerometer
        logData(PID_ACC, ax, ay, az);
        // log x/y/z of gyro meter
        logData(PID_GYRO, gx, gy, gz);
        logData(PID_MEMS_TEMP, temp);

        lastMemsDataTime = dataTime;
    }
#endif
    void processGPS()
    {
        GPS_DATA gd = {0};
        if (getGPSData(&gd)) {
            Serial.println("#GPS"); 
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
              logData(PID_GPS_ALTITUDE, gd.alt / 100);
              logData(PID_GPS_SPEED, gd.speed);
              //logData(PID_GPS_SAT_COUNT, gd.sat);
              lastUTC = (uint16_t)gd.time;
            } else {
              delay(10);
            }
        }
    }
    void reconnect()
    {
        SerialRF.println("#Sleeping");
        state &= ~STATE_OBD_READY;
        toggleGSM();
        state |= STATE_SLEEPING;
        for (uint16_t i = 0; ; i++) {
            if (init()) {
                int value;
                if (read(PID_RPM, value) && value > 0)
                    break;
            }
        }
        SerialRF.println("#Resuming");
        state &= ~STATE_SLEEPING;
        setup();
    }
    byte state;
    byte channel;
    byte connErrors;
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
