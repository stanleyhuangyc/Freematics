/*************************************************************************
* OBD/MEMS/GPS Data Logger for Freematics ONE+
* Distributed under BSD license
*
* Visit http://freematics.com/products/freematics-one-plus for more information
* Developed by Stanley Huang <stanley@freematics.com.au>
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*************************************************************************/

#include <SPIFFS.h>
#include <FreematicsPlus.h>
#include "datalogger.h"
#include "config.h"

// states
#define STATE_STORE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_FOUND 0x4
#define STATE_GPS_READY 0x8
#define STATE_MEMS_READY 0x10
#define STATE_FILE_READY 0x20

void serverProcess(int timeout);
bool serverSetup();
void serverCheckup();

uint16_t MMDD = 0;
uint32_t UTC = 0;
uint32_t startTime = 0;
uint32_t pidErrors = 0;
char vin[18] = {0};
float accBias[3];
uint32_t fileid = 0;

COBDSPI obd;
GATTServer ble;
FreematicsESP32 sys;

class DataOutputter : public NullLogger
{
    void write(const char* buf, byte len)
    {
#if ENABLE_SERIAL_OUT
        Serial.println(buf);
#endif
        ble.println(buf);
    }
};

#if STORAGE == STORAGE_SD
SDLogger store(new DataOutputter);
#elif STORAGE == STORAGE_SPIFFS
SPIFFSLogger store(new DataOutputter);
#else
NullLogger store(new DataOutputter);
#endif

#if MEMS_MODE
#if MEMS_MODE == MEMS_ACC
MPU9250_ACC mems;
#elif MEMS_MODE == MEMS_9DOF
MPU9250_9DOF mems;
#elif MEMS_MODE == MEMS_DMP
MPU9250_DMP mems;
#endif

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

void listDir(fs::FS &fs, const char * dirname, uint8_t levels)
{
    Serial.printf("Listing directory: %s\n", dirname);
    fs::File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    fs::File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print(file.name());
            Serial.print(' ');
            Serial.print(file.size());
            Serial.println(" bytes");
        }
        file = root.openNextFile();
    }
}

class DataLogger
{
public:
    void init()
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
          strncpy(vin, buffer, sizeof(vin) - 1);
        }
      } else {
        Serial.println("NO");
        standby();
      }
      setState(STATE_OBD_READY);
#else
      SPI.begin();
#endif

#if USE_GPS
      if (!checkState(STATE_GPS_FOUND)) {
        Serial.print("GPS ");
        if (sys.gpsInit(GPS_SERIAL_BAUDRATE)) {
          setState(STATE_GPS_FOUND);
          Serial.println("OK");
          //waitGPS();
        } else {
          Serial.println("NO");
        }
      }
#endif

      if (!checkState(STATE_STORE_READY)) {
#if STORAGE == STORAGE_SD
        Serial.print("SD ");
        int volsize = store.begin();
        if (volsize > 0) {
          Serial.print(volsize);
          Serial.println("MB");
          setState(STATE_STORE_READY);
        } else {
          Serial.println("NO");
        }
#elif STORAGE == STORAGE_SPIFFS
        Serial.print("SPIFFS ");
        int ret = store.begin();
        if (ret >= 0) {
          Serial.print(ret);
          Serial.println(" bytes free");
          setState(STATE_STORE_READY);
          listDir(SPIFFS, "/", 0);
        } else {
          Serial.println("NO");
        }
#endif
      }
      startTime = millis();
    }
#if USE_GPS
    void logGPSData()
    {
        // issue the command to get parsed GPS data
        GPS_DATA gd = {0};
        if (checkState(STATE_GPS_FOUND) && sys.gpsGetData(&gd)) {
            store.setTimestamp(millis());
            if (gd.time && gd.time != UTC) {
              byte day = gd.date / 10000;
              if (MMDD % 100 != day) {
                store.log(PID_GPS_DATE, gd.date);
              }
              store.log(PID_GPS_TIME, gd.time);
              store.log(PID_GPS_LATITUDE, gd.lat);
              store.log(PID_GPS_LONGITUDE, gd.lng);
              store.log(PID_GPS_ALTITUDE, gd.alt);
              store.log(PID_GPS_SPEED, gd.speed);
              store.log(PID_GPS_SAT_COUNT, gd.sat);
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
          if (sys.gpsGetData(&gd) && gd.sat != 0 && gd.sat != 255) {
            Serial.print("SAT:");
            Serial.println(gd.sat);
            break;
          }
        }
    }
#endif
    void flushData(uint32_t fileSize)
    {
      // flush SD data every 1KB
        static uint8_t lastFileSize = 0;
        byte dataSizeKB = fileSize >> 10;
        if (dataSizeKB != lastFileSize) {
            digitalWrite(PIN_LED, HIGH);
            store.flush();
            lastFileSize = dataSizeKB;
#if MAX_DATA_FILE_SIZE
            if (fileSize >= 1024L * 1024 * MAX_DATA_FILE_SIZE) {
              store.close();
              clearState(STATE_FILE_READY);
            }
#endif
            digitalWrite(PIN_LED, LOW);
        }
    }
    void standby()
    {
  #if USE_GPS
        if (checkState(STATE_GPS_READY)) {
          Serial.print("GPS:");
          sys.gpsInit(0); // turn off GPS power
          Serial.println("OFF");
        }
  #endif
        store.close();
        clearState(STATE_OBD_READY | STATE_GPS_READY | STATE_FILE_READY);
        Serial.println("Standby");
        ble.println("Standby");
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
#if ENABLE_HTTPD
              serverProcess(100);
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
        while (!obd.init()) Serial.print('.');
  #endif
        Serial.println("Wakeup");
        ble.println("Wakeup");
    }
    bool checkState(byte flags) { return (m_state & flags) == flags; }
    void setState(byte flags) { m_state |= flags; }
    void clearState(byte flags) { m_state &= ~flags; }
private:
    byte m_state = 0;
};

DataLogger logger;

void showStats()
{
    uint32_t t = millis() - startTime;
    uint32_t dataCount = store.getDataCount();
    // calculate samples per second
    float sps = (float)dataCount * 1000 / t;
    // output to serial monitor
    char timestr[24];
    sprintf(timestr, "%02u:%02u.%c", t / 60000, (t % 60000) / 1000, (t % 1000) / 100 + '0');
    uint32_t fileSize = store.size();
    Serial.print(timestr);
    Serial.print(" | ");
    Serial.print(dataCount);
    Serial.print(" samples | ");
    Serial.print(sps, 1);
    Serial.print(" sps");
    if (fileSize > 0) {
      digitalWrite(PIN_LED, HIGH);
      logger.flushData(fileSize);
      Serial.print(" | ");
      Serial.print(fileSize);
      Serial.print(" bytes");
      digitalWrite(PIN_LED, LOW);
    }
    Serial.println();
    // output via BLE
    ble.print(timestr);
    ble.print(' ');
    ble.print(dataCount);
    ble.print(' ');
    ble.print(sps, 1);
    if (fileSize > 0) {
      ble.print(' ');
      ble.print(fileSize >> 10);
      ble.print('K');
    }
    ble.println();
}

void setup()
{
    delay(1000);

    // initialize USB serial
    Serial.begin(115200);
    Serial.print("ESP32 ");
    Serial.print(ESP.getCpuFreqMHz());
    Serial.print("MHz ");
    Serial.print(getFlashSize() >> 10);
    Serial.println("MB Flash");

    sys.begin();
    ble.begin("Freematics ONE+");

#if ENABLE_HTTPD
    serverSetup();
#endif

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
      if (ret == 2) Serial.print("9-DOF ");
      Serial.println("OK");
      calibrateMEMS();
    } else {
      Serial.println("NO");
    }
#endif

    logger.init();
    digitalWrite(PIN_LED, LOW);
    delay(1000);
}

void loop()
{
    // if file not opened, create a new file
    if (logger.checkState(STATE_STORE_READY) && !logger.checkState(STATE_FILE_READY)) {
      fileid = store.open();
      if (fileid) {
        logger.setState(STATE_FILE_READY);
      }
    }

    // re-initialize when OBD is disconnected
#if USE_OBD
    if (!logger.checkState(STATE_OBD_READY)) {
      digitalWrite(PIN_LED, HIGH);
      logger.init();
      digitalWrite(PIN_LED, LOW);
      return;
    }
#endif

    // poll and log OBD data
    store.setTimestamp(millis());
#if USE_OBD
    byte pids[]= {PID_RPM, PID_SPEED, PID_THROTTLE, PID_ENGINE_LOAD};
    for (byte i = 0; i < sizeof(pids) / sizeof(pids[0]); i++) {
      int value;
      byte pid = pids[i];
      if (obd.readPID(pid, value)) {
        store.log((uint16_t)pids[i] | 0x100, value);
      } else {
        pidErrors++;
        Serial.print("PID errors: ");
        Serial.println(pidErrors);
        ble.print("PID errors: ");
        ble.println(pidErrors);
        if (obd.errors >= 3 && !obd.init()) {
            logger.standby();
        }
      }
#if ENABLE_HTTPD
      serverProcess(5);
#endif
    }
#endif

#if MEMS_MODE
    if (logger.checkState(STATE_MEMS_READY)) {
      float acc[3];
      float gyr[3];
      float mag[3];
      bool updated;
#if ENABLE_ORIENTATION
      ORIENTATION ori;
      updated = mems.read(acc, gyr, mag, 0, &ori);
      if (updated) {
        Serial.print("Orientation: ");
        Serial.print(ori.yaw, 2);
        Serial.print(' ');
        Serial.print(ori.pitch, 2);
        Serial.print(' ');
        Serial.println(ori.roll, 2);
        store.log(PID_ACC, (int16_t)(acc[0] * 100), (int16_t)(acc[1] * 100), (int16_t)(acc[2] * 100));
        store.log(PID_ORIENTATION, (int16_t)(ori.yaw * 100), (int16_t)(ori.pitch * 100), (int16_t)(ori.roll * 100));
      }
#else
      updated = mems.read(acc, gyr, mag);
      if (updated) {
        store.log(PID_ACC, (int16_t)(acc[0] * 100), (int16_t)(acc[1] * 100), (int16_t)(acc[2] * 100));
        //store.log(PID_GYRO, (int16_t)(gyr[0] * 100), (int16_t)(gyr[1] * 100), (int16_t)(gyr[2] * 100));
      }
#endif
    }
#endif

#if USE_OBD
    // log battery voltage (from voltmeter), data in 0.01v
    int v = obd.getVoltage() * 100;
    store.log(PID_BATTERY_VOLTAGE, v);
#endif

#if USE_GPS
    logger.logGPSData();
#endif

#if ENABLE_HTTPD
    serverProcess(10);
    serverCheckup();
#endif

#if !ENABLE_SERIAL_OUT
    showStats();
#endif
}
