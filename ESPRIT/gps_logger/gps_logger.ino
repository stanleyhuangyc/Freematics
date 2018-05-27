/*************************************************************************
* GPS data logger for Freematics Esprit / ESP32
*
* Developed by Stanley Huang <stanley@freematics.com.au>
* Distributed under BSD license
* Visit https://freematics.com/products/esprit for more info
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
#define STATE_GPS_FOUND 0x4
#define STATE_GPS_READY 0x8
#define STATE_FILE_READY 0x20

void serverProcess(int timeout);
bool serverSetup();
void serverCheckup();
void executeCommand();

#ifdef ESP32
extern "C"
{
uint8_t temprature_sens_read();
uint32_t hall_sens_read();
}
#endif

uint16_t MMDD = 0;
uint32_t UTC = 0;
uint32_t startTime = 0;
uint32_t pidErrors = 0;
uint32_t fileid = 0;

GPS_DATA gd = {0};

char command[16] = {0};

GATTServer ble;
FreematicsESP32 sys;

class DataOutputter : public NullLogger
{
    void write(const char* buf, byte len)
    {
#if ENABLE_SERIAL_OUT
        Serial.println(buf);
        ble.println(buf);
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

int handlerLiveData(UrlHandlerParam* param)
{
    char *buf = param->pucBuffer;
    int bufsize = param->bufSize;
    int n = 0;
    n += snprintf(buf + n, bufsize - n, "{\"gps\":{\"lat\":%d,\"lng\":%d,\"alt\":%d,\"speed\":%d,\"sat\":%d}",
        gd.lat, gd.lng, gd.alt, gd.speed, gd.sat);
    buf[n++] = '}';
    param->contentLength = n;
    param->fileType=HTTPFILETYPE_JSON;
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
    param->fileType=HTTPFILETYPE_JSON;
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
      if (!checkState(STATE_GPS_FOUND)) {
        Serial.print("GPS...");
        if (sys.gpsInit(GPS_SERIAL_BAUDRATE)) {
          setState(STATE_GPS_FOUND);
          Serial.println("OK");
          //waitGPS();
        } else {
          Serial.println("NO");
        }
      }

      startTime = millis();
    }
    void logGPSData()
    {
        // issue the command to get parsed GPS data
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
    void logSensorData()
    {
        int deviceTemp = (int)temprature_sens_read() * 165 / 255 - 40;
        store.log(PID_DEVICE_TEMP, deviceTemp);
        store.log(PID_DEVICE_HALL, hall_sens_read());
        store.log(PID_ANALOG_INPUT1, (int16_t)analogRead(A0));
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
    ble.begin(WIFI_AP_SSID);

    // init LED pin
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_LED, HIGH);

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

#if ENABLE_HTTPD
    Serial.print("HTTP Server...");
    if (serverSetup()) {
      Serial.println("OK");
    } else {
      Serial.println("NO");
    }
    serverCheckup();
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
    
    store.setTimestamp(ts);
    logger.logGPSData();
    logger.logSensorData();

#if ENABLE_HTTPD
    serverProcess(10);
    serverCheckup();
#endif

#if !ENABLE_SERIAL_OUT
    showStats();
#endif

    executeCommand();

    int waitTime = MIN_LOOP_TIME - (millis() - ts);
    if (waitTime > 0) delay(waitTime);
}
