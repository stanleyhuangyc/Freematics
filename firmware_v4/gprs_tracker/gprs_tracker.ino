/******************************************************************************
* GPRS OBD-II/GPS Tracker based on Freematics ONE with SIM800L module 
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one for more information
* Written by Stanley Huang and partially by Brendan Myers
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
#include <SPI.h>
#include <Wire.h>
#include <FreematicsONE.h>

/**************************************
* GPRS settings
**************************************/
#define APN "connect"
#define HOST_URL "http://f.skygrid.io/DEVICE_KEY"

/**************************************
* GPS settings
**************************************/
#define USE_GPS 1
#define GPS_SERIAL_BAUDRATE 115200L

/**************************************
* Other options
**************************************/
#define OBD_CONN_TIMEOUT 5000 /* ms */
#define MAX_CONN_ERRORS 5
#define MAX_CONN_TIME 10000 /* ms */

// device states
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_SLEEPING 0x20
#define STATE_CONNECTED 0x40

static uint32_t nextConnTime = 0;
static uint16_t connCount = 0;
static GPS_DATA gd = {0};
static const byte pids[]= {PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_COOLANT_TEMP, PID_THROTTLE};
static int pidData[sizeof(pids)] = {0};

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
          Serial.println(buffer);
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
    char buffer[192];
    byte bytesRecv;
    uint32_t checkTimer;
    byte gprsState;
    byte connErrors;
};

class CGPRSTracker : public COBDGSM
{
public:
    void setup()
    {
        state = 0;
        delay(500);
        Serial.begin(115200);

        // this will init SPI communication
        begin();
        
        // connect to OBD port
        Serial.print("#OBD..");
        for (uint32_t t = millis(); millis() - t < OBD_CONN_TIMEOUT; ) {
            Serial.print('.');
            if (init()) {
              state |= STATE_OBD_READY;
              break;              
            }
        }
        
        // display OBD adapter version
        if (state & STATE_OBD_READY) {
          Serial.print("VER ");
          Serial.println(version);
        } else {
          Serial.println("NO"); 
        }

#if USE_GPS
        // initialize GPS
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

        Serial.print("#GPRS(APN:");
        Serial.print(APN);
        Serial.print(")...");
        if (setupGPRS(APN)) {
            Serial.println("OK");
        } else {
            Serial.println(buffer);
        }

        int csq = getSignal();
        Serial.print("#SIGNAL:");
        Serial.println(csq);

        if (getOperatorName()) {
          Serial.print("#OP:");
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

        state |= STATE_CONNECTED;
        
        Serial.println();
        delay(1000);
    }
    void loop()
    {
        if (state & STATE_OBD_READY) {
          if (readPID(pids, sizeof(pids), pidData) != sizeof(pids)) {
            Serial.println("OBD error");
          }
        }

#if USE_GPS
        if (state & STATE_GPS_READY) {
          if (!getGPSData(&gd)) {
            Serial.println("GPS error");
            delay(500);
          }
        }
#endif

        if (millis() > nextConnTime) {
          processGPRS();
        }
        
        if (connErrors >= MAX_CONN_ERRORS) {
          // reset GPRS 
          Serial.print("Reset GPRS...");
          initGSM();
          setupGPRS(APN);
          if (httpInit()) {
            Serial.println("OK"); 
          } else {
            Serial.println(buffer); 
          }
          connErrors = 0;
          connCount = 0;
        }
    }
private:
    void processGPRS()
    {
        // state machine for GPRS/HTTP communication
        switch (gprsState) {
        case GPRS_READY:
            if (state & STATE_CONNECTED) {
                // URL format: http://f.skygrid.io/<DEVICE_KEY>/<RPM>/<SPEED>/<ENGINE_LOAD>/<COOLANT_TEMP>/<INTAKE_PRESSURE>/<THROTTLE_POSITION>/<FUEL_RATE>/<GPS_LAT>/<GPS_LNG>
                // generate URL
                snprintf_P(buffer, sizeof(buffer), PSTR("AT+HTTPPARA=\"URL\",\"%s/%u/%u/%u/%d/0/%u/0/%ld/%ld\"\r"),
                  HOST_URL, pidData[0], pidData[1], pidData[2], pidData[3], pidData[4], gd.lat, gd.lng);
                Serial.println(buffer);
                if (!sendGSMCommand(buffer)) {
                  Serial.println("Request error");
                  break;
                }
                gprsState = GPRS_HTTP_CONNECTING;
            }
            break;        
        case GPRS_HTTP_CONNECTING:
            Serial.print("CONNECT#");
            Serial.println(++connCount);
            httpConnect(HTTP_GET);
            nextConnTime = millis() + 2000;
            break;
        case GPRS_HTTP_RECEIVING:
            if (httpIsConnected()) {
                /* no response to be read
                if (httpRead()) {
                  // success
                  Serial.println("Response:");
                  Serial.println(buffer);
                }
                */
                gprsState = GPRS_READY;
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
    void reconnect()
    {
        if (init()) {
          // reconnected
          return; 
        }
        Serial.println("Sleeping");
        state &= ~STATE_OBD_READY;
        toggleGSM(); // turn off GSM power
#if USE_GPS
        initGPS(0); // turn off GPS power
#endif
        state |= STATE_SLEEPING;
        for (;;) {
            int value;
            if (readPID(PID_RPM, value))
                break;
            lowPowerMode();
            sleep(4);
        }
        // reset device
        void(* resetFunc) (void) = 0; //declare reset function at address 0
        resetFunc();
    }
    byte state;
};

CGPRSTracker tracker;

void setup()
{
    tracker.setup();
}

void loop()
{
    tracker.loop();
}
