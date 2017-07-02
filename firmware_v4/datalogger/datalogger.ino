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

uint16_t MMDD = 0;
uint32_t UTC = 0;

#if USE_MEMS
float acc[3] = {0};
#if ENABLE_ORIENTATION
uint32_t nextOriTime = 0;
#endif
int16_t accCal[3] = {0}; // calibrated reference accelerometer data
byte deviceTemp; // device temperature (celcius degree)
#endif

class ONE : public COBDSPI, public CDataLogger
#if USE_MEMS
,public CMPU9250
#endif
{
public:
    ONE():state(0) {}
    void setup()
    {
      byte ver = begin();
      Serial.print("Firmware Ver. ");
      Serial.println(ver);

#if USE_MEMS
      if (!(state & STATE_MEMS_READY)) {
        Serial.print("MEMS ");
        byte ret = memsInit(ENABLE_ORIENTATION);
        if (ret) {
          state |= STATE_MEMS_READY;
          Serial.println(ret == 1 ? "MPU6050" : "MPU9250");
        } else {
          Serial.println("NO");
        }
      }
#endif

#if USE_OBD
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

#if USE_OBD
      // retrieve VIN
      char buffer[128];
      if ((state & STATE_OBD_READY) && getVIN(buffer, sizeof(buffer))) {
        Serial.print("VIN:");
        Serial.println(buffer);
      }
#endif
    }
#if USE_GPS
    void logGPSData()
    {
        // issue the command to get parsed GPS data
        GPS_DATA gd = {0};
        if (gpsGetData(&gd)) {
            dataTime = millis();
            if (gd.time && gd.time != UTC) {
              byte day = gd.date / 10000;
              if (MMDD % 100 != day) {
                log(PID_GPS_DATE, gd.date);
              }
              log(PID_GPS_TIME, gd.time);
              log(PID_GPS_LATITUDE, gd.lat);
              log(PID_GPS_LONGITUDE, gd.lng);
              log(PID_GPS_ALTITUDE, gd.alt);
              log(PID_GPS_SPEED, gd.speed);
              log(PID_GPS_SAT_COUNT, gd.sat);
              // save current date in MMDD format
              unsigned int DDMM = gd.date / 100;
              UTC = gd.time;
              MMDD = (DDMM % 100) * 100 + (DDMM / 100);
              // set GPS ready flag
              state |= STATE_GPS_READY;
            }
        }
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
    void flushData(uint32_t fileSize)
    {
      // flush SD data every 1KB
        static uint8_t lastFileSize = 0;
        byte dataSizeKB = fileSize >> 10;
        if (dataSizeKB != lastFileSize) {
            flushFile();
            lastFileSize = dataSizeKB;
#if MAX_DATA_FILE_SIZE
            if (fileSize >= 1024L * MAX_DATA_FILE_SIZE) {
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
#if USE_MEMS
        for (;;) {
          unsigned long accSum[3] = {0};
          unsigned int count = 0;
          for (uint32_t t = millis(); millis() - t < 1000; count++) {
            memsRead(acc, 0, 0, 0);
            accSum[0] += acc[0] * acc[0];
            accSum[1] += acc[1] * acc[1];
            accSum[2] += acc[2] * acc[2];
            delay(10);
          }
          // calculate relative movement
          unsigned long motion = (accSum[0] + accSum[1] + accSum[2]) / count / count;
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
        setup();
    }
#if USE_MEMS
    void dataIdleLoop()
    {
      // do something while waiting for data on SPI
      if (state & STATE_MEMS_READY) {
        int16_t temp; // device temperature (in 0.1 celcius degree)
#if ENABLE_ORIENTATION
        float gyr[3];
        float mag[3];
        memsRead(acc, gyr, mag, &temp);
#else
        memsRead(acc, 0, 0, &temp);
#endif
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
#ifdef ESP32
    Serial.println("Freematics ONE+");
#else
    Serial.println("Freematics ONE");
#endif
    delay(1000);
    one.setup();
}

void loop()
{
#if ENABLE_DATA_LOG
    if (!(one.state & STATE_FILE_READY) && (one.state & STATE_SD_READY)) {
      if (one.state & STATE_GPS_FOUND) {
        // GPS connected
        if (one.state & STATE_GPS_READY) {
          uint32_t dateTime = (uint32_t)MMDD * 10000 + UTC / 10000;
          if (one.openFile(dateTime)) {
            MMDD = 0;
            one.state |= STATE_FILE_READY;
          }
        }
      } else {
        // no GPS connected
        if (one.openFile(0)) {
          one.state |= STATE_FILE_READY;
        }
      }
      delay(1000);
      return;
    }
#endif
#if USE_OBD
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
            one.log((uint16_t)pids[n] | 0x100, values[n]);
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
    one.log(PID_BATTERY_VOLTAGE, v);
#else
    one.dataIdleLoop();
#endif

#if USE_MEMS
    if (one.state & STATE_MEMS_READY) {
#if ENABLE_ORIENTATION
      uint32_t t = millis();
      if (t > nextOriTime) {
        float yaw, pitch, roll;
        one.memsOrientation(yaw, pitch, roll);
        Serial.print("Orientation: ");
        Serial.print(yaw, 2);
        Serial.print(' ');
        Serial.print(pitch, 2);
        Serial.print(' ');
        Serial.println(roll, 2);
        one.log(PID_ORIENTATION, (int16_t)(yaw * 100), (int16_t)(pitch * 100), (int16_t)(roll * 100));
        nextOriTime = t + ORIENTATION_INTERVAL;
      }
#endif
      // log the loaded acceleration data
      one.log(PID_ACC, (int16_t)(acc[0] * 100), (int16_t)(acc[1] * 100), (int16_t)(acc[2] * 100));
    }
#endif
#if USE_GPS
    if (one.state & STATE_GPS_FOUND) {
      one.logGPSData();
    }
#endif

#if !ENABLE_DATA_OUT && !ENABLE_ORIENTATION
    Serial.print(one.dataCount);
    Serial.print(" samples ");
#if ENABLE_DATA_LOG
    uint32_t fileSize = sdfile.size();
    if (fileSize > 0) {
      one.flushData(fileSize);
      Serial.print(sdfile.size());
      Serial.print(" bytes");
    }
#endif
    Serial.println();
#endif
}
