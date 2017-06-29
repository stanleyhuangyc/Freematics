/*************************************************************************
* OBD-II/MEMS/GPS Data Logging Sketch for Freematics ONE
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one for more information
* Developed by Stanley Huang <support@freematics.com.au>
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*************************************************************************/

#include <FreematicsONE.h>
#include "config.h"
#include "datalogger.h"

// states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_FOUND 0x4
#define STATE_GPS_READY 0x8
#define STATE_MEMS_READY 0x10
#define STATE_FILE_READY 0x20

static uint8_t lastFileSize = 0;
static uint16_t fileIndex = 0;

uint16_t MMDD = 0;
uint32_t UTC = 0;

#if USE_MPU9250
int16_t acc[3] = {0};
int16_t accCal[3] = {0}; // calibrated reference accelerometer data
byte deviceTemp; // device temperature (celcius degree)
#endif

class ONE : public COBDSPI, public CDataLogger
#if USE_MPU9250
,public CMPU9250
#endif
{
public:
    ONE():state(0) {}
    void setup()
    {
#if USE_MPU9250
        Serial.print("MEMS ");
        if (memsInit()) {
          state |= STATE_MEMS_READY;
          Serial.println("OK");
        } else {
          Serial.println("NO");
        }
#endif

#if ENABLE_OBD
        byte ver = begin();
        Serial.print("Firmware Ver. ");
        Serial.println(ver);

        Serial.print("OBD ");
        if (init()) {
          Serial.println("OK");
        } else {
          Serial.println("NO");
          reconnect();
        }
        state |= STATE_OBD_READY;
#else
        SPI.begin();
        SPI.setFrequency(250000);
#endif

#if 0
        // retrieve VIN
        char buffer[128];
        if ((state & STATE_OBD_READY) && getVIN(buffer, sizeof(buffer))) {
          Serial.print("VIN:");
          Serial.println(buffer);
        }
#endif

#if ENABLE_DATA_LOG
        if (!(state & STATE_SD_READY)) {
          Serial.print("SD ");
          uint16_t volsize = initSD();
          if (volsize) {
            Serial.print(volsize);
            Serial.println("MB");
          } else {
            Serial.println("NO");
          }
        }
#endif

#if USE_GPS
        Serial.print("GPS ");
        if (gpsInit(GPS_SERIAL_BAUDRATE)) {
          state |= STATE_GPS_FOUND;
          Serial.println("OK");
          //waitGPS();
        } else {
          Serial.println("NO");
        }
#endif
    }
#if USE_GPS
    void logGPSData()
    {
#if LOG_GPS_NMEA_DATA
        // issue the command to get NMEA data (one line per request)
        char buf[255];
        byte n = gpsGetRawData(buf, sizeof(buf));
        if (n) {
            dataTime = millis();
            // strip heading $GPS and ending \r\n
            logData(buf + 4, n - 6);
        }
#endif
#if LOG_GPS_PARSED_DATA
        // issue the command to get parsed GPS data
        GPS_DATA gd = {0};
        if (gpsGetData(&gd)) {
            dataTime = millis();
            if (gd.time && gd.time != UTC) {
              byte day = gd.date / 10000;
              if (MMDD % 100 != day) {
                logData(PID_GPS_DATE, gd.date);
              }
              logData(PID_GPS_TIME, gd.time);
              logData(PID_GPS_LATITUDE, gd.lat);
              logData(PID_GPS_LONGITUDE, gd.lng);
              logData(PID_GPS_ALTITUDE, gd.alt);
              logData(PID_GPS_SPEED, gd.speed);
              logData(PID_GPS_SAT_COUNT, gd.sat);
              // save current date in MMDD format
              unsigned int DDMM = gd.date / 100;
              UTC = gd.time;
              MMDD = (DDMM % 100) * 100 + (DDMM / 100);
              // set GPS ready flag
              state |= STATE_GPS_READY;
            }
        }
#endif
    }
    void waitGPS()
    {
        int elapsed = 0;
        GPS_DATA gd = {0};
        for (uint32_t t = millis(); millis() - t < 300000;) {
          int t1 = (millis() - t) / 1000;
          if (t1 != elapsed) {
            Serial.print("Waiting for GPS (");
            Serial.print(elapsed);
            Serial.println(")");
            elapsed = t1;
          }
          // read parsed GPS data
          if (gpsGetData(&gd) && gd.sat != 0 && gd.sat != 255) {
            Serial.print("SAT:");
            Serial.println(gd.sat);
            break;
          }
        }
    }
#endif
#if ENABLE_DATA_LOG
    uint16_t initSD()
    {
        state &= ~STATE_SD_READY;
        pinMode(SD_CS_PIN, OUTPUT);
        if (SD.begin(SD_CS_PIN)) {
          state |= STATE_SD_READY;
          return SD.cardSize();
        } else {
          return 0;
        }
    }
    void flushData()
    {
      // flush SD data every 1KB
        byte dataSizeKB = dataSize >> 10;
        if (dataSizeKB != lastFileSize) {
            flushFile();
            lastFileSize = dataSizeKB;
#if MAX_LOG_FILE_SIZE
            if (dataSize >= 1024L * MAX_LOG_FILE_SIZE) {
              closeFile();
              state &= ~STATE_FILE_READY;
            }
#endif
        }
    }
#endif
    void reconnect()
    {
        Serial.println("Disconnected");
        // try to re-connect to OBD
        if (init()) return;
#if ENABLE_DATA_LOG
        closeFile();
#endif
        // turn off GPS power
#if USE_GPS
        gpsInit(0);
#endif
        state &= ~(STATE_OBD_READY | STATE_GPS_READY);
        Serial.println("Standby");
        // put OBD chips into low power mode
        enterLowPowerMode();
#if USE_MPU9250
        for (;;) {
          unsigned long accSum[3] = {0};
          unsigned int count = 0;
          for (uint32_t t = millis(); millis() - t < 1000; count++) {
            memsRead(acc, 0, 0, 0);
            accSum[0] += acc[0] * acc[0];
            accSum[1] += acc[1] * acc[1];
            accSum[2] += acc[2] * acc[2];
          }
          // calculate relative movement
          unsigned long motion = (accSum[0] + accSum[1] + accSum[2]) / count;
          Serial.print("M:");
          Serial.println(motion);
          // check movement
          if (motion > WAKEUP_MOTION_THRESHOLD) {
            break;
          }
        }
#else
        while (!init()) sleep(10);
#endif
        Serial.println("Wakeup");
        leaveLowPowerMode();
#ifdef ARDUINO_ARCH_AVR
        // reset device
        void(* resetFunc) (void) = 0; //declare reset function at address 0
        resetFunc();
#else
        setup();
#endif
    }
#if USE_MPU9250
    void dataIdleLoop()
    {
      // do something while waiting for data on SPI
      if (state & STATE_MEMS_READY) {
        // load accelerometer and temperature data
        int16_t temp; // device temperature (in 0.1 celcius degree)
        memsRead(acc, 0, 0, &temp);
        deviceTemp = temp / 10;
      }
    }
 #endif
    byte state;
};

static ONE one;

void setup()
{
    Serial.begin(115200);
    Serial.println("Freematics ONE");
    delay(3000);
    one.setup();
}

void loop()
{
#if ENABLE_DATA_LOG
    if (!(one.state & STATE_FILE_READY) && (one.state & STATE_SD_READY)) {
      if (one.state & STATE_GPS_FOUND) {
        // GPS connected
        Serial.print("File ");
        if (one.state & STATE_GPS_READY) {
          uint32_t dateTime = (uint32_t)MMDD * 10000 + UTC / 10000;
          if (one.openFile(dateTime) != 0) {
            Serial.println(dateTime);
            MMDD = 0;
            one.state |= STATE_FILE_READY;
          } else {
            Serial.println("File error");
          }
        }
      } else {
        // no GPS connected
        int index = one.openFile(0);
        if (index != 0) {
          one.state |= STATE_FILE_READY;
          Serial.println(index);
        } else {
          Serial.println("File error");
        }
      }
    }
#endif
#if ENABLE_OBD
    if (one.state & STATE_OBD_READY) {
        byte pids[]= {0, PID_RPM, PID_SPEED, PID_THROTTLE, PID_ENGINE_LOAD};
        byte pids2[] = {PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_DISTANCE};
        int values[sizeof(pids)];
        static byte index2 = 0;
        pids[0] = pids2[index2 = (index2 + 1) % sizeof(pids2)];
        // read multiple OBD-II PIDs
        if (one.readPID(pids, sizeof(pids), values) == sizeof(pids)) {
          one.dataTime = millis();
          for (byte n = 0; n < sizeof(pids); n++) {
            one.logData((uint16_t)pids[n] | 0x100, values[n]);
          }
        }
        if (one.errors >= 10) {
            one.reconnect();
        }
    } else {
      if (!OBD_ATTEMPT_TIME || millis() < OBD_ATTEMPT_TIME * 1000) {
        if (one.init()) {
            one.state |= STATE_OBD_READY;
        }
      }
    }

    // log battery voltage (from voltmeter), data in 0.01v
    int v = one.getVoltage() * 100;
    one.dataTime = millis();
    one.logData(PID_BATTERY_VOLTAGE, v);
#else
    one.dataIdleLoop();
#endif

#if USE_MPU9250
    if (one.state & STATE_MEMS_READY) {
       // log the loaded MEMS data
      one.logData(PID_ACC, acc[0], acc[1], acc[2]);
    }
#endif
#if USE_GPS
    if (one.state & STATE_GPS_FOUND) {
      one.logGPSData();
    }
#endif

#if ENABLE_DATA_LOG
    uint32_t logsize = sdfile.size();
    if (logsize > 0) {
      one.flushData();
      Serial.print(sdfile.size());
      Serial.println(" bytes");
    }
#endif
}
