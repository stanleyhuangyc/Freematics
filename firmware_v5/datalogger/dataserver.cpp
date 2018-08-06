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
*
* Implemented HTTP APIs:
* /api/info - device info
* /api/live - live data (OBD/GPS/MEMS)
* /api/control - issue a control command
* /api/list - list of log files
* /api/log/<file #> - raw CSV format log file
* /api/data/<file #>?pid=<PID in hex> - JSON array of PID data
*************************************************************************/

#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <SPIFFS.h>
#include <httpd.h>
#include <FreematicsPlus.h>
#if defined(ESP32)
#include <WiFi.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <apps/sntp/sntp.h>
#include <esp_spi_flash.h>
#include <esp_err.h>
#else
#error Unsupported board type
#endif
#include "config.h"

#define WIFI_TIMEOUT 5000

extern uint32_t fileid;

extern "C"
{
uint8_t temprature_sens_read();
uint32_t hall_sens_read();
}

#if ENABLE_HTTPD

HttpParam httpParam;

int handlerLiveData(UrlHandlerParam* param);
int handlerControl(UrlHandlerParam* param);

uint16_t hex2uint16(const char *p);

int handlerInfo(UrlHandlerParam* param)
{
    char *buf = param->pucBuffer;
    int bufsize = param->bufSize;
    int bytes = snprintf(buf, bufsize, "{\"httpd\":{\"uptime\":%lu,\"clients\":%d,\"requests\":%u,\"traffic\":%u},\n",
        millis(), httpParam.stats.clientCount, (unsigned int)httpParam.stats.reqCount, (unsigned int)(httpParam.stats.totalSentBytes >> 10));

    time_t now;
    time(&now);
    struct tm timeinfo = { 0 };
    localtime_r(&now, &timeinfo);
    if (timeinfo.tm_year) {
        bytes += snprintf(buf + bytes, bufsize - bytes, "\"rtc\":{\"date\":\"%04u-%02u-%02u\",\"time\":\"%02u:%02u:%02u\"},\n",
        timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
        timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    }

    int deviceTemp = (int)temprature_sens_read() * 165 / 255 - 40;
    bytes += snprintf(buf + bytes, bufsize - bytes, "\"cpu\":{\"temperature\":%d,\"magnetic\":%d},\n",
        deviceTemp, hall_sens_read());

#if STORAGE == STORAGE_SPIFFS
    bytes += snprintf(buf + bytes, bufsize - bytes, "\"spiffs\":{\"total\":%u,\"used\":%u}",
        SPIFFS.totalBytes(), SPIFFS.usedBytes());
#else
    bytes += snprintf(buf + bytes, bufsize - bytes, "\"sd\":{\"total\":%llu,\"used\":%llu}",
        SD.totalBytes(), SD.usedBytes());
#endif

    if (bytes < bufsize - 1) buf[bytes++] = '}';

    param->contentLength = bytes;
    param->contentType=HTTPFILETYPE_JSON;
    return FLAG_DATA_RAW;
}

#if STORAGE != STORAGE_NONE

class LogDataContext {
public:
    File file;
    uint32_t tsStart;
    uint32_t tsEnd;
    uint16_t pid;
};

int handlerLogFile(UrlHandlerParam* param)
{
    LogDataContext* ctx = (LogDataContext*)param->hs->ptr;
    param->contentType = HTTPFILETYPE_TEXT;
    if (ctx) {
		if (!param->pucBuffer) {
			// connection to be closed, final calling, cleanup
			ctx->file.close();
            delete ctx;
			param->hs->ptr = 0;
			return 0;
		}
    } else {
        int id = 0;
        if (param->pucRequest[0] == '/') {
            id = atoi(param->pucRequest + 1);
        }
        sprintf(param->pucBuffer, "/DATA/%u.CSV", id == 0 ? fileid : id);
        ctx = new LogDataContext;
#if STORAGE == STORAGE_SPIFFS
        ctx->file = SPIFFS.open(param->pucBuffer, FILE_READ);
#else
        ctx->file = SD.open(param->pucBuffer, FILE_READ);
#endif
        if (!ctx->file) {
            strcat(param->pucBuffer, " not found");
            param->contentLength = strlen(param->pucBuffer);
            delete ctx;
            return FLAG_DATA_RAW;
        }
        param->hs->ptr = (void*)ctx;
    }

    if (!ctx->file.available()) {
        // EOF
        return 0;
    }
    param->contentLength = ctx->file.readBytes(param->pucBuffer, param->bufSize);
    param->contentType = HTTPFILETYPE_TEXT;
    Serial.print(param->contentLength);
    Serial.println(" bytes read");
    return FLAG_DATA_STREAM;
}

int handlerLogData(UrlHandlerParam* param)
{
    uint32_t duration = 0;
    LogDataContext* ctx = (LogDataContext*)param->hs->ptr;
    param->contentType = HTTPFILETYPE_JSON;
    if (ctx) {
		if (!param->pucBuffer) {
			// connection to be closed, final calling, cleanup
			ctx->file.close();
            delete ctx;
			param->hs->ptr = 0;
			return 0;
		}
    } else {
        int id = 0;
        if (param->pucRequest[0] == '/') {
            id = atoi(param->pucRequest + 1);
        }
        sprintf(param->pucBuffer, "/DATA/%u.CSV", id == 0 ? fileid : id);
        ctx = new LogDataContext;
#if STORAGE == STORAGE_SPIFFS
        ctx->file = SPIFFS.open(param->pucBuffer, FILE_READ);
#else
        ctx->file = SD.open(param->pucBuffer, FILE_READ);
#endif
        if (!ctx->file) {
            param->contentLength = sprintf(param->pucBuffer, "{\"error\":\"Data file not found\"}");
            delete ctx;
            return FLAG_DATA_RAW;
        }
        ctx->pid = mwGetVarValueHex(param->pxVars, "pid", 0);
        ctx->tsStart = mwGetVarValueInt(param->pxVars, "start", 0);
        ctx->tsEnd = 0xffffffff;
        duration = mwGetVarValueInt(param->pxVars, "duration", 0);
        if (ctx->tsStart && duration) {
            ctx->tsEnd = ctx->tsStart + duration;
            duration = 0;
        }
        param->hs->ptr = (void*)ctx;
        // JSON head
        param->contentLength = sprintf(param->pucBuffer, "[");
    }
    
    int len = 0;
    char buf[64];
    uint32_t ts = 0;

    for (;;) {
        int c = ctx->file.read();
        if (c == -1) {
            if (param->contentLength == 0) {
                // EOF
                return 0;
            }
            // JSON tail
            if (param->pucBuffer[param->contentLength - 1] == ',') param->contentLength--;
            param->pucBuffer[param->contentLength++] = ']';
            break;
        }
        if (c == '\n') {
            // line end, process the line
            buf[len] = 0;
            char *value = strchr(buf, ',');
            if (value++) {
                uint16_t pid = hex2uint16(buf);
                if (pid == 0) {
                    // timestamp
                    ts = atoi(value);
                    if (duration) {
                        ctx->tsEnd = ts + duration;
                        duration = 0;
                    }
                } else if (pid == ctx->pid && ts >= ctx->tsStart && ts < ctx->tsEnd) {
                    // generate json array element
                    param->contentLength += snprintf(param->pucBuffer + param->contentLength, param->bufSize - param->contentLength,
                        "[%u,%s],", ts, value);
                }
            }
            len = 0;
            if (param->contentLength + 32 > param->bufSize) break;
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        }
    }
    return FLAG_DATA_STREAM;
}

int handlerLogList(UrlHandlerParam* param)
{
    char *buf = param->pucBuffer;
    int bufsize = param->bufSize;
    File file;
#if STORAGE == STORAGE_SPIFFS
    File root = SPIFFS.open("/");
#elif STORAGE == STORAGE_SD
    File root = SD.open("/DATA");
#endif
    int n = snprintf(buf, bufsize, "[");
    if (root) {
        while(file = root.openNextFile()) {
            const char *fn = file.name();
            if (!strncmp(fn, "/DATA/", 6)) {
                fn += 6;
                unsigned int size = file.size();
                Serial.print(fn);
                Serial.print(' ');
                Serial.print(size);
                Serial.println(" bytes");
                unsigned int id = atoi(fn);
                if (id) {
                    n += snprintf(buf + n, bufsize - n, "{\"id\":%u,\"size\":%u",
                        id, size);
                    if (id == fileid) {
                        n += snprintf(buf + n, bufsize - n, ",\"active\":true");
                    }
                    n += snprintf(buf + n, bufsize - n, "},");
                }
            }
        }
        if (buf[n - 1] == ',') n--;
    }
    n += snprintf(buf + n, bufsize - n, "]");
    param->contentType=HTTPFILETYPE_JSON;
    param->contentLength = n;
    return FLAG_DATA_RAW;
}

#endif

UrlHandler urlHandlerList[]={
    {"api/live", handlerLiveData},
    {"api/info", handlerInfo},
    {"api/control", handlerControl},
#if STORAGE != STORAGE_NONE
    {"api/list", handlerLogList},
    {"api/data", handlerLogData},
    {"api/log", handlerLogFile},
#endif
    {0}
};

#endif

void obtainTime()
{
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, (char*)"pool.ntp.org");
    sntp_init();
}

void serverProcess(int timeout)
{
#if ENABLE_HTTPD
    mwHttpLoop(&httpParam, timeout);
#endif
}

bool serverCheckup(int wifiJoinPeriod)
{
    static uint32_t wifiStartTime = 0;
    if (WiFi.status() != WL_CONNECTED) {
        if (wifiStartTime == 0 || millis() - wifiStartTime > wifiJoinPeriod) {
            WiFi.disconnect(false);
#if ENABLE_WIFI_AP && ENABLE_WIFI_STATION
            WiFi.mode (WIFI_AP_STA);
#elif ENABLE_WIFI_AP
            WiFi.mode (WIFI_AP);
#elif ENABLE_WIFI_STATION
            WiFi.mode (WIFI_STA);
#endif
            Serial.print("Connecting to hotspot (SSID:");
            Serial.print(WIFI_SSID);
            Serial.println(')');
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            wifiStartTime = millis();
        }
    } else {
        if (wifiStartTime) {
            // just connected
            Serial.print("Connected to hotspot. IP:");
            Serial.println(WiFi.localIP());

            // start mDNS responder
            MDNS.begin("datalogger");
            MDNS.addService("http", "tcp", 80);
            MDNS.addService("nmea", "tcp", NMEA_TCP_PORT);

            wifiStartTime = 0;
        }
        return true;
    }
    return false;
}

bool serverSetup()
{
#if ENABLE_WIFI_AP && ENABLE_WIFI_STATION
    WiFi.mode (WIFI_AP_STA);
#elif ENABLE_WIFI_AP
    WiFi.mode (WIFI_AP);
#elif ENABLE_WIFI_STATION
    WiFi.mode (WIFI_STA);
#endif

#if ENABLE_WIFI_AP
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
    Serial.print("AP:");
    Serial.print(WiFi.softAPIP());
    Serial.print(' ');
#endif

#if ENABLE_HTTPD
    mwInitParam(&httpParam, 80, "/spiffs");
    httpParam.pxUrlHandler = urlHandlerList;

    if (mwServerStart(&httpParam)) {
        return false;
    }
#endif

#if ENABLE_WIFI_STATION
    obtainTime();
#endif
    return true;
}
