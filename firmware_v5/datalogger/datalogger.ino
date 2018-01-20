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

#include <FreematicsPlus.h>
#include "config.h"
#include "datalogger.h"

// states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_FOUND 0x4
#define STATE_GPS_READY 0x8
#define STATE_MEMS_READY 0x10
#define STATE_FILE_READY 0x20

#define PIN_LED 4

uint16_t MMDD = 0;
uint32_t UTC = 0;

COBDSPI obd;

#if MEMS_MODE

#if MEMS_MODE == MEMS_ACC
MPU9250_ACC mems;
#elif MEMS_MODE == MEMS_9DOF
MPU9250_9DOF mems;
#elif MEMS_MODE == MEMS_DMP
MPU9250_DMP mems;
#endif

float accBias[3];

void calibrateMEMS()
{
    // MEMS data collected while sleeping
    accBias[0] = 0;
    accBias[1] = 0;
    accBias[2] = 0;
    int n;
    for (n = 0; n < 100; n++) {
      float acc[3] = {0};
      mems.read(acc);
      accBias[0] += acc[0];
      accBias[1] += acc[1];
      accBias[2] += acc[2];
      delay(10);
    }
    accBias[0] /= n;
    accBias[1] /= n;
    accBias[2] /= n;
    Serial.print("ACC Bias:");
    Serial.print(accBias[0]);
    Serial.print('/');
    Serial.print(accBias[1]);
    Serial.print('/');
    Serial.println(accBias[2]);
}
#endif

class CLogger : public virtual CFreematicsESP32, public CDataLogger
{
public:
    void setup()
    {
#if USE_OBD
      Serial.print("OBD ");
      if (obd.init()) {
        Serial.println("OK");
        // retrieve VIN
        char buffer[128];
        if (obd.getVIN(buffer, sizeof(buffer))) {
          Serial.print("VIN:");
          Serial.println(buffer);
        }
      } else {
        Serial.println("NO");
        reconnect();
      }
      setState(STATE_OBD_READY);
#else
      SPI.begin();
#endif

#if USE_GPS
      if (!checkState(STATE_GPS_FOUND)) {
        Serial.print("GPS ");
        if (gpsInit(GPS_SERIAL_BAUDRATE)) {
          setState(STATE_GPS_FOUND);
          Serial.println("OK");
          //waitGPS();
        } else {
          Serial.println("NO");
        }
      }
#endif

#if ENABLE_DATA_LOG
      if (!checkState(STATE_SD_READY)) {
        Serial.print("SD ");
        uint16_t volsize = initSD();
        if (volsize) {
          Serial.print(volsize);
          Serial.println("MB");
          setState(STATE_SD_READY);
        } else {
          Serial.println("NO");
        }
      }
#endif
    }
#if USE_GPS
    void logGPSData()
    {
        // issue the command to get parsed GPS data
        GPS_DATA gd = {0};
        if (checkState(STATE_GPS_FOUND) && gpsGetData(&gd)) {
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
              setState(STATE_GPS_READY);
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
        pinMode(PIN_SD_CS, OUTPUT);
        if (SD.begin(PIN_SD_CS)) {
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
              clearState(STATE_FILE_READY);
            }
#endif
        }
    }
#endif
    void reconnect()
    {
        if (obd.init()) return;
        // try to re-connect to OBD
#if ENABLE_DATA_LOG
        closeFile();
#endif
        // turn off GPS power
#if USE_GPS
        gpsInit(0);
#endif
        clearState(STATE_OBD_READY | STATE_GPS_READY);
        standby();
    }
    void standby()
    {
  #if ENABLE_GPS
        if (checkState(STATE_GPS_READY)) {
          Serial.print("GPS:");
          gpsInit(0); // turn off GPS power
          Serial.println("OFF");
        }
  #endif
        clearState(STATE_OBD_READY | STATE_GPS_READY);
        Serial.println("Standby");
  #if MEMS_MODE
        if (checkState(STATE_MEMS_READY)) {
          calibrateMEMS();
          for (;;) {
            // calculate relative movement
            float motion = 0;
            for (byte n = 0; n < 10; n++) {
              float acc[3];
              mems.read(acc);
              for (byte i = 0; i < 3; i++) {
                float m = (acc[i] - accBias[i]);
                motion += m * m;
              }
#ifdef ESP32
              esp_sleep_enable_timer_wakeup(100000); //100ms
              esp_light_sleep_start();
#else
              delay(100);
#endif
            }
            Serial.println(motion);
            // check movement
            if (motion > WAKEUP_MOTION_THRESHOLD) {
              break;
            }
          }
        }
  #else
        while (!init()) Serial.print('.');
  #endif
        Serial.println("Wakeup");
    }
    bool checkState(byte flags) { return (m_state & flags) == flags; }
    void setState(byte flags) { m_state |= flags; }
    void clearState(byte flags) { m_state &= ~flags; }
private:
    byte m_state = 0;
};

CLogger logger;

void setup()
{
    delay(1000);
    // initialize USB serial
    Serial.begin(115200);
    Serial.print("ESP32 ");
    Serial.print(ESP.getCpuFreqMHz());
    Serial.print("MHz ");
    Serial.print(spi_flash_get_chip_size() >> 20);
    Serial.println("M Flash");

    // init LED pin
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    byte ver = obd.begin();
    Serial.print("Firmware Ver. ");
    Serial.println(ver);

#if MEMS_MODE
    Serial.print("MEMS ");
    byte ret = mems.begin(ENABLE_ORIENTATION);
    if (ret) {
      logger.setState(STATE_MEMS_READY);
      Serial.println("OK");
      calibrateMEMS();
    } else {
      Serial.println("NO");
    }
#endif

    logger.setup();
    digitalWrite(PIN_LED, LOW);
    delay(1000);
}

void loop()
{
#if USE_OBD
    if (!logger.checkState(STATE_OBD_READY)) {
      digitalWrite(PIN_LED, HIGH);
      logger.setup();
      digitalWrite(PIN_LED, LOW);
      return;
    }
#endif
#if ENABLE_DATA_LOG
    if (!logger.checkState(STATE_FILE_READY) && logger.checkState(STATE_SD_READY)) {
      digitalWrite(PIN_LED, HIGH);
#if USE_GPS
      if (logger.checkState(STATE_GPS_FOUND)) {
        // GPS connected
        logger.logGPSData();
        if (logger.checkState(STATE_GPS_READY)) {
          uint32_t dateTime = (uint32_t)MMDD * 10000 + UTC / 10000;
          if (logger.openFile(dateTime)) {
            MMDD = 0;
            logger.setState(STATE_FILE_READY);
          }
        } else {
          Serial.println("Waiting for GPS...");
        }
      }
      else
#endif
      {
        // no GPS connected
        if (logger.openFile(0)) {
          logger.setState(STATE_FILE_READY);
        }
      }
      logger.sleep(1000);
      digitalWrite(PIN_LED, LOW);
      return;
    }
#endif
#if USE_OBD
    byte pids[]= {PID_RPM, PID_SPEED, PID_THROTTLE, PID_ENGINE_LOAD};
    logger.dataTime = millis();
    for (byte i = 0; i < sizeof(pids) / sizeof(pids[0]); i++) {
      int value;
      byte pid = pids[i];
      if (obd.readPID(pid, value)) {
        logger.log((uint16_t)pids[i] | 0x100, value);
      } else {
        Serial.println("PID not read");
        if (obd.errors >= 3) {
          obd.reset();
          logger.reconnect();
        }
      }
#endif

#if MEMS_MODE
      if (logger.checkState(STATE_MEMS_READY)) {
        float acc[3];
        bool updated;
#if ENABLE_ORIENTATION
        ORIENTATION ori;
#if !ENABLE_DMP
        float gyr[3];
        float mag[3];
        updated = mems.read(acc, gyr, mag, 0, &ori);
#else
        updated = mems.read(acc, 0, 0, 0, &ori);
#endif
        if (updated) {
          Serial.print("Orientation: ");
          Serial.print(ori.yaw, 2);
          Serial.print(' ');
          Serial.print(ori.pitch, 2);
          Serial.print(' ');
          Serial.println(ori.roll, 2);
          logger.log(PID_ACC, (int16_t)(acc[0] * 100), (int16_t)(acc[1] * 100), (int16_t)(acc[2] * 100));
          logger.log(PID_ORIENTATION, (int16_t)(ori.yaw * 100), (int16_t)(ori.pitch * 100), (int16_t)(ori.roll * 100));
        }
#else
        updated = mems.read(acc);
        if (updated) {
          logger.log(PID_ACC, (int16_t)(acc[0] * 100), (int16_t)(acc[1] * 100), (int16_t)(acc[2] * 100));
        }
#endif
      }
#endif

#if USE_OBD
    }
    // log battery voltage (from voltmeter), data in 0.01v
    int v = obd.getVoltage() * 100;
    logger.log(PID_BATTERY_VOLTAGE, v);
#endif

#if USE_GPS
    logger.logGPSData();
#endif

#if !ENABLE_DATA_OUT
    uint32_t t = millis();
    Serial.print(logger.dataCount);
    Serial.print(" samples ");
    Serial.print((float)logger.dataCount * 1000 / t, 1);
    Serial.print(" sps ");
#if ENABLE_DATA_LOG
    uint32_t fileSize = sdfile.size();
    if (fileSize > 0) {
      logger.flushData(fileSize);
      Serial.print(fileSize);
      Serial.print(" bytes");
    }
#endif
    Serial.println();
#endif
}
