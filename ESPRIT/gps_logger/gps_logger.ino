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

#include <Wire.h>
#include <FS.h>
#include <SD.h>
#include <SPIFFS.h>
#include <FreematicsPlus.h>
#include <FreematicsOLED.h>
#include <httpd.h>
#include "datalogger.h"
#include "config.h"

// states
#define STATE_STORE_READY 0x1
#define STATE_GPS_FOUND 0x4
#define STATE_FILE_READY 0x20

void serverProcess(int timeout);
bool serverSetup(IPAddress& ip);
bool serverCheckup(int wifiJoinPeriod = WIFI_JOIN_TIMEOUT);
void executeCommand();

#ifdef ESP32
extern "C"
{
uint8_t temprature_sens_read();
uint32_t hall_sens_read();
}
#endif

uint32_t lastGPSts = 0;
uint32_t startTime = 0;
uint32_t pidErrors = 0;
uint32_t fileid = 0;
uint32_t stationeryTimer = 0;
float distance = 0;

GPS_DATA* gd = 0;

char command[16] = {0};

FreematicsESP32 sys;

#if ENABLE_DISPLAY
OLED_SH1106 lcd;
#endif

WiFiServer nmeaServer(NMEA_TCP_PORT);
WiFiClient nmeaClient;

#if ENABLE_TRACCAR_CLIENT

#define CLIENT_STATE_IDLE 0
#define CLIENT_STATE_RECEIVING 1

class TraccarClient
{
public:
    bool connect()
    {
        Serial.print("Connecting to ");
        Serial.print(TRACCAR_HOST);
        Serial.print("...");
        if (client.connect(TRACCAR_HOST, TRACCAR_PORT)) {
            Serial.println("OK");
            return true;
        } else {
            Serial.println("failed");
            return false;
        }
    }
    void process(bool updated = true)
    {
        // check and keep the connection with Traccar server
        if (!client.connected()) {
            client.stop();
            if (!connect()) return;
        }
        if (state == CLIENT_STATE_RECEIVING) {
            int ret = receive();
            if (ret) {
                state = CLIENT_STATE_IDLE;
                Serial.print('#');
                Serial.print(sent);
                Serial.print(' ');
                Serial.print(ret);
                Serial.print("ms HTTP Code:");
                Serial.println(code);
#if ENABLE_DISPLAY
                lcd.setFontSize(FONT_SIZE_SMALL);
                lcd.setCursor(0, 7);
                lcd.print('#');
                lcd.print(sent);
                lcd.setFontSize(FONT_SIZE_SMALL);
                lcd.print(' ');
                lcd.print(bytes >> 10);
                lcd.setFontSize(FONT_SIZE_SMALL);
                lcd.print("KB ");
                lcd.print(ret);
                lcd.print("ms  ");
#endif
                if (code == 200)
                    sent++;
                else
                    errors++;
            } else if (millis() - sentTime > TRACCAR_SERVER_TIMEOUT) {
                errors++;
                client.stop();
                Serial.println("Connection teared down");
                state = CLIENT_STATE_IDLE;
            }
        }
        if (state == CLIENT_STATE_IDLE) {
            if (updated) transmit();
        }
    }
    unsigned int bytes = 0;
    unsigned int sent = 0;
    unsigned int errors = 0;
    unsigned short code = 0;
private:
    int receive()
    {
        if (state != CLIENT_STATE_RECEIVING) return 0;
        // waiting for server response while still decoding NMEA
        while (client.available()) {
            String resp = client.readStringUntil('\n');
            if (resp[0] == '\r') {
                state = CLIENT_STATE_IDLE;
                // return time elapsed
                return millis() - sentTime;
            } else {
                char *p = strstr(resp.c_str(), "HTTP/");
                if (p && (p = strchr(p, ' '))) {
                    code = atoi(p + 1);
                }
            }
        }
        return 0;
    }
    void transmit()
    {
        // arrange and send data in OsmAnd protocol
        // refer to https://www.traccar.org/osmand
        char data[128];
        sprintf(data, "&lat=%f&lon=%f&altitude=%.1f&speed=%.1f&heading=%d", gd->lat, gd->lng, gd->alt, gd->speed, gd->heading);
        // generate ISO formatted UTC date/time
        char isotime[24];
        sprintf(isotime, "%04u-%02u-%02uT%02u:%02u:%02u.%01uZ",
            (unsigned int)(gd->date % 100) + 2000, (unsigned int)(gd->date / 100) % 100, (unsigned int)(gd->date / 10000),
            (unsigned int)(gd->time / 1000000), (unsigned int)(gd->time % 1000000) / 10000, (unsigned int)(gd->time % 10000) / 100, ((unsigned int)gd->time % 100) / 10);
        // send data
        int n = client.print(String("GET /?id=") + TRACCAR_DEV_ID + "&timestamp=" + isotime + data + " HTTP/1.1\r\n" +
            //"Host: " + TRACCAR_HOST + "\r\n" +
            "Connection: keep-alive\r\n\r\n");
        if (n > 0) {
            state = CLIENT_STATE_RECEIVING;
            code = 0;
            sentTime = millis();
            bytes += n;
        }
    }
    WiFiClient client;
    byte state = CLIENT_STATE_IDLE;
    unsigned long sentTime = 0;
};

TraccarClient traccar;
#endif

#if STORAGE == STORAGE_SD
SDLogger store;
#elif STORAGE == STORAGE_SPIFFS
SPIFFSLogger store;
#else
NullLogger store;
#endif

int handlerLiveData(UrlHandlerParam* param)
{
    char *buf = param->pucBuffer;
    int bufsize = param->bufSize;
    int n = 0;
    n += snprintf(buf + n, bufsize - n, "{\"gps\":{\"lat\":%f,\"lng\":%f,\"alt\":%f,\"speed\":%f,\"sat\":%d}",
        gd->lat, gd->lng, gd->alt, gd->speed, gd->sat);
    buf[n++] = '}';
    param->contentLength = n;
    param->contentType=HTTPFILETYPE_JSON;
    return FLAG_DATA_RAW;
}

int handlerNMEA(UrlHandlerParam* param)
{
    param->contentLength = sys.gpsGetNMEA(param->pucBuffer, param->bufSize);
    param->contentType=HTTPFILETYPE_TEXT;
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
    bool logGPSData()
    {
        // issue the command to get parsed GPS data
        if (!sys.gpsGetData(&gd) || lastGPSts == gd->ts) {
            return false;
        }
        store.setTimestamp(millis());
        store.log(PID_GPS_DATE, gd->date);
        float kph = (float)gd->speed * 1852 / 1000;
        store.log(PID_GPS_TIME, gd->time);
        store.logFloat(PID_GPS_LATITUDE, gd->lat);
        store.logFloat(PID_GPS_LONGITUDE, gd->lng);
        store.log(PID_GPS_ALTITUDE, gd->alt);
        store.log(PID_GPS_SPEED, kph);
        store.log(PID_GPS_HEADING, gd->heading);
        store.log(PID_GPS_SAT_COUNT, gd->sat);
        // some computations
        if (kph < 1) stationeryTimer = gd->ts;
        lastGPSts = gd->ts;
        return true;
    }
    void logSensorData()
    {
        store.log(PID_EXT_SENSOR1, analogRead(A0));
    }
    void waitGPS()
    {
        uint32_t t = millis();
        Serial.println("Waiting GPS signal...");
#if ENABLE_DISPLAY
        lcd.setCursor(0, 6);
        lcd.print("Waiting GPS Signal...");
#endif
        while (!sys.gpsGetData(&gd)) {
            if (!gd) continue;
            Serial.print((millis() - t) / 1000);
            Serial.print("s NMEA:");
            Serial.print(gd->sentences);
            Serial.print(" ERR:");
            Serial.println(gd->errors);
#if ENABLE_DISPLAY
            lcd.setCursor(0, 7);
            lcd.print((millis() - t) / 1000);
            lcd.print("s NMEA:");
            lcd.print(gd->sentences);
#endif
            delay(1000);
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
    logger.checkFileSize(fileSize);
    Serial.print(" | ");
    Serial.print(fileSize);
    Serial.print(" bytes saved");
#if ENABLE_TRACCAR_CLIENT
    Serial.print(" | ");
    Serial.print(traccar.bytes);
    Serial.print(" bytes sent");
#endif
    Serial.println();
}

#if ENABLE_DISPLAY
void updateDisplay(bool updated)
{
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.setCursor(68, 0);
    lcd.printInt((int)distance, 4);
    lcd.setFontSize(FONT_SIZE_SMALL);
    lcd.print('.');
    lcd.printInt((int)(distance * 10) % 10);
    lcd.print("km");    

    int seconds = (millis() - startTime) / 1000;
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.setCursor(0, 0);
    lcd.setFlags(FLAG_PAD_ZERO);
    lcd.printInt(seconds / 60, 2);
    lcd.print(':');
    lcd.printInt(seconds % 60, 2);
    lcd.setFlags(0);

    if (!updated) return;

    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.setCursor(0, 3);
    lcd.printInt(gd->alt, 3);
    lcd.setFontSize(FONT_SIZE_SMALL);
    lcd.setCursor(0, 5);
    lcd.print("m Alt");

    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.setCursor(105, 3);
    lcd.printInt(gd->sat, 2);
    lcd.setFontSize(FONT_SIZE_SMALL);
    lcd.setCursor(103, 5);
    lcd.print("Sats");

    lcd.setFontSize(FONT_SIZE_XLARGE);
    lcd.setCursor(50, 2);
    lcd.printInt(gd->speed * 1852 / 100000, 2);
    lcd.setFontSize(FONT_SIZE_SMALL);
    lcd.setCursor(58, 5);
    lcd.print("km/h");
}

void initDisplay()
{
    lcd.clear();
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.print("GPS LOGGER");
    lcd.setFontSize(FONT_SIZE_SMALL);
    lcd.setCursor(98, 0);
    lcd.println("ESP32");
    lcd.setCursor(104, 1);
    lcd.println("WiFi");
    lcd.setCursor(0, 3);
    lcd.print("CPU:");
    lcd.print(ESP.getCpuFreqMHz());
    lcd.print("MHz ");
    lcd.print(ESP.getFlashChipSize() >> 20);
    lcd.println("MB FLASH");
}
#endif

void setup()
{
#ifdef ARDUINO_ESP32C3_DEV
    Wire.begin(4, 5);
#else
    Wire.begin();
#endif

#if ENABLE_DISPLAY
    lcd.begin();
    initDisplay();
#endif

    // initialize USB serial
    Serial.begin(115200);
    Serial.print("ESP32 ");
    Serial.print(ESP.getCpuFreqMHz());
    Serial.print("MHz ");
    Serial.print(ESP.getFlashChipSize() >> 20);
    Serial.println("MB Flash");

    sys.begin(false, false);

#if STORAGE == STORAGE_SD
    Serial.print("SD...");
    int volsize = store.begin();
    if (volsize > 0) {
        Serial.print(volsize);
        Serial.println("MB");
        logger.setState(STATE_STORE_READY);
#if ENABLE_DISPLAY
        lcd.print("SD ");
        lcd.print(volsize);
        lcd.println("MB");
#endif
    } else {
      Serial.println("NO");
    }
#elif STORAGE == STORAGE_SPIFFS
    Serial.print("SPIFFS...");
    int freebytes = store.begin();
    if (freebytes >= 0) {
        Serial.print(freebytes >> 10);
        Serial.println("KB Free");
        logger.setState(STATE_STORE_READY);
        //listDir(SPIFFS, "/", 0);
#if ENABLE_DISPLAY
        lcd.print("SPIFFS ");
        lcd.print(freebytes >> 10);
        lcd.println("KB Free");
#endif
    } else {
        Serial.println("NO");
    }
#endif

#if ENABLE_WIFI_STATION
#if ENABLE_DISPLAY
    lcd.println("Connecting hotspot..");
#endif
    while (!serverCheckup()) delay(1000);
#if ENABLE_DISPLAY
    lcd.print("IP:");
    lcd.println(WiFi.localIP());
#endif
#endif

#if ENABLE_NMEA_SERVER
    nmeaServer.begin();
#endif

    Serial.print("GPS...");
    if (sys.gpsBeginExt(GPS_SERIAL_BAUDRATE)) {
        logger.setState(STATE_GPS_FOUND);
        lcd.println("GPS Receiver OK");
        Serial.println("OK");
        logger.waitGPS();
    } else {
        Serial.println("NO");
#if ENABLE_DISPLAY
        lcd.println("GPS not connected");
#endif
        for (;;) delay(1000);
    }

#if ENABLE_DISPLAY
    lcd.clear();
#endif
#if ENABLE_HTTPD
    IPAddress ip;
    Serial.print("HTTP Server...");
    if (serverSetup(ip)) {
      Serial.println("OK");
    } else {
      Serial.println("NO");
    }
    serverCheckup();
    if (ip) {
        Serial.print("WiFi AP IP:");
        Serial.println(ip);
#if ENABLE_DISPLAY
        lcd.println("WiFi AP started");
        lcd.print("SSID:");
        lcd.println(WIFI_AP_SSID);
        lcd.print("IP:");
        lcd.println(ip);
        lcd.println("HTTPd is running");
#endif        
    }
#endif

#if ENABLE_TRACCAR_CLIENT
#if ENABLE_DISPLAY
    lcd.println("Connecting server..");
#endif
    while (!serverCheckup() || !traccar.connect()) delay(1000);
    lcd.println(TRACCAR_HOST);
#endif

    Serial.print("File:");
    fileid = store.open();
    if (fileid) {
        Serial.println(fileid);
        logger.setState(STATE_FILE_READY);
    } else {
        Serial.print("NO");
    }

#if ENABLE_DISPLAY
    delay(3000);
    lcd.clear();
#endif
    startTime = millis();
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
    //logger.logSensorData();
    bool updated = logger.logGPSData();
    Serial.println(gd->ts);
    if (gd && millis() - gd->ts > GPS_SIGNAL_TIMEOUT) {
#ifdef ENABLE_WIFI_STATION
        WiFi.disconnect(false);
#endif
#if ENABLE_DISPLAY
        initDisplay();
#endif
        logger.waitGPS();
#if ENABLE_DISPLAY
        lcd.clear();
#endif        
    }

    showStats();

    executeCommand();

#if ENABLE_DISPLAY
    updateDisplay(updated);
#endif

#if ENABLE_TRACCAR_CLIENT
    traccar.process(updated);
#endif

#if ENABLE_WIFI_AP || ENABLE_WIFI_STATION
    serverCheckup();
#endif

#if ENABLE_NMEA_SERVER
    // NMEA-to-TCP bridge
    if (!nmeaClient || !nmeaClient.connected()) {
        nmeaClient.stop();
        nmeaClient = nmeaServer.available();
    }
    do {
        if (nmeaClient.connected()) {
            char buf[256];
            int bytes = sys.gpsGetNMEA(buf, sizeof(buf));
            if (bytes > 0) nmeaClient.write(buf, bytes);
            bytes = 0;
            while (nmeaClient.available() && bytes < sizeof(buf)) {
                buf[bytes++] = nmeaClient.read();
            }
            if (bytes > 0) sys.gpsSendCommand(buf, bytes);
        }
#if ENABLE_HTTPD
        serverProcess(50);
#else
        delay(50);
#endif
    } while (millis() - ts < MIN_LOOP_TIME);
#else
    ts = millis() - ts;
#if ENABLE_HTTPD
    serverProcess(ts < MIN_LOOP_TIME ? (MIN_LOOP_TIME - ts) : 0);
#else
    //if (ts < MIN_LOOP_TIME) delay(MIN_LOOP_TIME - ts);
#endif
#endif
}
