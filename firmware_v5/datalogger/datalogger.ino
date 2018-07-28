/*************************************************************************
* Vehicle Telemetry Data Logger for Freematics ONE+
*
* Developed by Stanley Huang <stanley@freematics.com.au>
* Distributed under BSD license
* Visit https://freematics.com/products/freematics-one-plus for more info
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
#include <httpd.h>
#include "datalogger.h"
#include "config.h"

// states
#define STATE_STORE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_FOUND 0x4
#define STATE_GPS_READY 0x8
#define STATE_MEMS_READY 0x10
#define STATE_FILE_READY 0x20
#define STATE_STANDBY 0x40

void serverProcess(int timeout);
bool serverSetup();
void serverCheckup();
void executeCommand();

uint16_t MMDD = 0;
uint32_t UTC = 0;
uint32_t startTime = 0;
uint32_t pidErrors = 0;
float accBias[3];
uint32_t fileid = 0;
// live data
char vin[18] = {0};
int16_t batteryVoltage = 0;

typedef struct {
  byte pid;
  byte tier;
  int16_t value;
  uint32_t ts;
} PID_POLLING_INFO;

PID_POLLING_INFO obdData[]= {
  {PID_SPEED, 1},
  {PID_RPM, 1},
  {PID_THROTTLE, 1},
  {PID_ENGINE_LOAD, 1},
  {PID_FUEL_PRESSURE, 2},
  {PID_TIMING_ADVANCE, 2},
  {PID_COOLANT_TEMP, 3},
  {PID_INTAKE_TEMP, 3},
};

#if MEMS_MODE
float acc[3] = {0};
float gyr[3] = {0};
float mag[3] = {0};
ORIENTATION ori = {0};
#endif

#if USE_GPS
GPS_DATA gd = {0};
#endif

char command[16] = {0};

COBD* obd = 0;

FreematicsESP32 sys;

WiFiServer nmeaServer(NMEA_TCP_PORT);
WiFiClient nmeaClient;

class DataOutputter : public NullLogger
{
    void write(const char* buf, byte len)
    {
#if ENABLE_SERIAL_OUT
        Serial.println(buf);
#endif
    }
};

#if STORAGE == STORAGE_SD
SDLogger store(new DataOutputter);
#elif STORAGE == STORAGE_SPIFFS
SPIFFSLogger store(new DataOutputter);
#else
DataOutputter store;
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

int handlerLiveData(UrlHandlerParam* param)
{
    char *buf = param->pucBuffer;
    int bufsize = param->bufSize;
    int n = snprintf(buf, bufsize, "{\"obd\":{\"vin\":\"%s\",\"battery\":%d,\"pid\":[", vin, (int)batteryVoltage);
    uint32_t t = millis();
    for (int i = 0; i < sizeof(obdData) / sizeof(obdData[0]); i++) {
        n += snprintf(buf + n, bufsize - n, "{\"pid\":%u,\"value\":%d,\"age\":%u},",
            0x100 | obdData[i].pid, obdData[i].value, t - obdData[i].ts);
    }
    n--;
    n += snprintf(buf + n, bufsize - n, "]}");
#if MEMS_MODE
    n += snprintf(buf + n, bufsize - n, ",\"mems\":{\"acc\":[%d,%d,%d]",
        (int)((acc[0] - accBias[0]) * 100), (int)((acc[1] - accBias[1]) * 100), (int)((acc[2] - accBias[2]) * 100));
#if MEMS_MODE == MEMS_9DOF || MEMS_MODE == MEMS_DMP
    n += snprintf(buf + n, bufsize - n, ",\"gyro\":[%d,%d,%d]",
        (int)(gyr[0] * 100), (int)(gyr[1] * 100), (int)(gyr[2] * 100));
#endif
#if MEMS_MODE == MEMS_9DOF
    n += snprintf(buf + n, bufsize - n, ",\"mag\":[%d,%d,%d]",
        (int)(mag[0] * 10000), (int)(mag[1] * 10000), (int)(mag[2] * 10000));
#endif
#if ENABLE_ORIENTATION
    n += snprintf(buf + n, bufsize - n, ",\"orientation\":{\"pitch\":\"%f\",\"roll\":\"%f\",\"yaw\":\"%f\"}",
        ori.pitch, ori.roll, ori.yaw);
#endif
    buf[n++] = '}';
#endif
#if USE_GPS
    n += snprintf(buf + n, bufsize - n, ",\"gps\":{\"date\":%u,\"time\":%u,\"lat\":%d,\"lng\":%d,\"alt\":%d,\"speed\":%d,\"sat\":%u,\"sentences\":%u,\"errors\":%u}",
        gd.date, gd.time, gd.lat, gd.lng, gd.alt, gd.speed, gd.sat,
        gd.sentences, gd.errors);
#endif
    buf[n++] = '}';
    param->contentLength = n;
    param->contentType=HTTPFILETYPE_JSON;
    return FLAG_DATA_RAW;
}

int handlerControl(UrlHandlerParam* param)
{
    char *cmd = mwGetVarValue(param->pxVars, "cmd", 0);
    if (!cmd) return 0;
    char *buf = param->pucBuffer;
    int bufsize = param->bufSize;
    if (command[0]) {
        param->contentLength = snprintf(buf, bufsize, "{\"result\":\"pending\"}");
    } else {
        strncpy(command, cmd, sizeof(command) - 1);
        param->contentLength = snprintf(buf, bufsize, "{\"result\":\"OK\"}");
    }
    param->contentType=HTTPFILETYPE_JSON;
    return FLAG_DATA_RAW;
}

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
      Serial.print("OBD...");
      if (obd->init()) {
        Serial.println("OK");
        pidErrors = 0;
        // retrieve VIN
        char buffer[128];
        if (obd->getVIN(buffer, sizeof(buffer))) {
          Serial.print("VIN:");
          Serial.println(buffer);
          strncpy(vin, buffer, sizeof(vin) - 1);
        }
      } else {
        Serial.println("NO");
        standby();
        return;
      }
      setState(STATE_OBD_READY);
#endif

#if USE_GPS
      if (!checkState(STATE_GPS_FOUND)) {
        Serial.print("GPS...");
        memset(&gd, 0, sizeof(gd));
        if (sys.gpsInit(GPS_SERIAL_BAUDRATE, true)) {
          setState(STATE_GPS_FOUND);
          Serial.println("OK");
          //waitGPS();
        } else {
          Serial.println("NO");
        }
      }
#endif

      startTime = millis();
    }
#if USE_GPS
    void logGPSData()
    {
        // issue the command to get parsed GPS data
        if (checkState(STATE_GPS_FOUND) && sys.gpsGetData(&gd)) {
            store.setTimestamp(millis());
            if (gd.time != UTC) {
                byte day = gd.date / 10000;
                if (MMDD % 100 != day) {
                    store.log(PID_GPS_DATE, gd.date);
                }
                store.log(PID_GPS_TIME, gd.time);
                store.log(PID_GPS_LATITUDE, gd.lat);
                store.log(PID_GPS_LONGITUDE, gd.lng);
                store.log(PID_GPS_ALTITUDE, gd.alt / 100); /* m */
                float kph = (float)gd.speed * 1852 / 100000;
                store.log(PID_GPS_SPEED, kph);
                store.log(PID_GPS_SAT_COUNT, gd.sat);
                // save current date in MMDD format
                unsigned int DDMM = gd.date / 100;
                UTC = gd.time;
                MMDD = (DDMM % 100) * 100 + (DDMM / 100);
                // set GPS ready flag
                setState(STATE_GPS_READY);
                char buf[48];
                sprintf(buf, "GPS UTC:%02u:%02u:%02u.%01u Sats:%u",
                    gd.time / 1000000, (gd.time % 1000000) / 10000, (gd.time % 10000) / 100, (gd.time % 100) / 10, (unsigned int)gd.sat);
                Serial.println(buf);
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
    void checkFileSize(uint32_t fileSize)
    {
        static uint8_t lastSizeKB = 0;
        static uint8_t flushCount = 0;
        uint8_t sizeKB = fileSize >> 10;
        if (sizeKB != lastSizeKB) {
            lastSizeKB = sizeKB;
            flushCount = 0;
        } else if (++flushCount == 100) {
          // if file size does not increase after many flushes, close file
          store.close();
          clearState(STATE_FILE_READY);
          flushCount = 0;
        }
    }
    void standby()
    {
      store.close();
#if USE_GPS
      if (checkState(STATE_GPS_READY)) {
        Serial.print("GPS:");
        sys.gpsInit(0); // turn off GPS power
        Serial.println("OFF");
      }
#endif
      clearState(STATE_OBD_READY | STATE_GPS_READY | STATE_FILE_READY);
      serverCheckup();
      setState(STATE_STANDBY);
      Serial.println("Standby");
#if MEMS_MODE
      if (checkState(STATE_MEMS_READY)) {
        calibrateMEMS();
        while (checkState(STATE_STANDBY)) {
          // calculate relative movement
          float motion = 0;
          for (byte n = 0; n < 10; n++) {
            mems.read(acc);
            for (byte i = 0; i < 3; i++) {
              float m = (acc[i] - accBias[i]);
              motion += m * m;
            }
#if (ENABLE_WIFI_STATION || ENABLE_WIFI_AP)
            serverProcess(100);
#else
            delay(100);
#endif
          }
          // check movement
          if (motion > WAKEUP_MOTION_THRESHOLD * WAKEUP_MOTION_THRESHOLD) {
            Serial.print("Motion:");
            Serial.println(motion);
            break;
          }
          executeCommand();
        }
      }
#else
      while (!obd->init()) Serial.print('.');
#endif
      Serial.println("Wakeup");
      //ESP.restart();
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
      logger.checkFileSize(fileSize);
      Serial.print(" | ");
      Serial.print(fileSize);
      Serial.print(" bytes");
    }
    Serial.println();
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

    // init LED pin
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_LED, HIGH);

    sys.begin();
#if USE_OBD
    while (!(obd = createOBD()));
#endif

#if MEMS_MODE
    Serial.print("MEMS...");
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

#if STORAGE == STORAGE_SD
    Serial.print("SD...");
    int volsize = store.begin();
    if (volsize > 0) {
      Serial.print(volsize);
      Serial.println("MB");
      logger.setState(STATE_STORE_READY);
    } else {
      Serial.println("NO");
    }
#elif STORAGE == STORAGE_SPIFFS
    Serial.print("SPIFFS...");
    int freebytes = store.begin();
    if (freebytes >= 0) {
      Serial.print(freebytes >> 10);
      Serial.println(" KB free");
      logger.setState(STATE_STORE_READY);
      listDir(SPIFFS, "/", 0);
    } else {
      Serial.println("NO");
    }
#endif

#if ENABLE_WIFI_STATION || ENABLE_WIFI_AP
#if ENABLE_HTTPD
    Serial.print("HTTP Server...");
    if (serverSetup()) {
      Serial.println("OK");
    } else {
      Serial.println("NO");
    }
#endif
    serverCheckup();
    nmeaServer.begin();
#endif

    pinMode(PIN_LED, LOW);

    logger.init();
}

void executeCommand()
{
    if (!command[0]) return;
    if (!strcmp(command, "reset")) {
        store.close();
        ESP.restart();
        // never reach here
    } else if (!strcmp(command, "standby")) {
        command[0] = 0;
        if (!logger.checkState(STATE_STANDBY)) {
            logger.standby();
        }
    } else if (!strcmp(command, "wakeup")) {
        logger.clearState(STATE_STANDBY);
    }
    command[0] = 0;
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

    uint32_t ts = millis();
    // poll and log OBD data
    store.setTimestamp(ts);
#if USE_OBD
    static int idx[2] = {0, 0};
    int tier = 1;
    for (byte i = 0; i < sizeof(obdData) / sizeof(obdData[0]); i++) {
        if (obdData[i].tier > tier) {
            // reset previous tier index
            idx[tier - 2] = 0;
            // keep new tier number
            tier = obdData[i].tier;
            // move up current tier index
            i += idx[tier - 2]++;
            // check if into next tier
            if (obdData[i].tier != tier) {
                idx[tier - 2]= 0;
                i--;
                continue;
            }
        }
        byte pid = obdData[i].pid;
        if (!obd->isValidPID(pid)) continue;
        int value;
        if (obd->readPID(pid, value)) {
            obdData[i].ts = millis();
            store.log((uint16_t)pid | 0x100, value);
        } else {
            pidErrors++;
            Serial.print("PID errors: ");
            Serial.println(pidErrors);
            if (obd->errors >= 3 && !obd->init()) {
                logger.standby();
            }
        }
#if USE_GPS
        logger.logGPSData();
#endif
        if (tier > 1) break;
#if ENABLE_WIFI_STATION || ENABLE_WIFI_AP
        serverProcess(0);
#endif
    }
#else
#if USE_GPS
    logger.logGPSData();
#endif
    delay(50);
#endif

    // re-initialize when OBD is disconnected
#if USE_OBD
    if (!logger.checkState(STATE_OBD_READY)) {
      logger.init();
      return;
    }
#endif

#if MEMS_MODE
    if (logger.checkState(STATE_MEMS_READY)) {
      bool updated;
#if ENABLE_ORIENTATION
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
      }
#endif
    }
#endif

#if USE_OBD
    // log battery voltage (from voltmeter), data in 0.01v
    batteryVoltage = obd->getVoltage() * 100;
    store.log(PID_BATTERY_VOLTAGE, batteryVoltage);
#endif

#if !ENABLE_SERIAL_OUT
    showStats();
#endif

    executeCommand();

#if ENABLE_WIFI_STATION || ENABLE_WIFI_AP
    // NMEA-to-TCP bridge
    serverCheckup();
    if (!nmeaClient || !nmeaClient.connected()) {
        nmeaClient.stop();
        nmeaClient = nmeaServer.available();
    }
    do {
        if (nmeaClient.connected()) {
            char buf[NMEA_BUF_SIZE];
            int bytes = sys.gpsGetNMEA(buf, sizeof(buf));
            if (bytes > 0) nmeaClient.write(buf, bytes);
            bytes = 0;
            while (nmeaClient.available() && bytes < NMEA_BUF_SIZE) {
                buf[bytes++] = nmeaClient.read();
            }
            if (bytes > 0) sys.gpsSendCommand(buf, bytes);
        }
        serverProcess(0);
    } while (millis() - ts < MIN_LOOP_TIME);
#else
    ts = millis() - ts;
    if (ts < MIN_LOOP_TIME) delay(MIN_LOOP_TIME - ts);
#endif

}
