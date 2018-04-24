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
* http://ip/api/info - device info
* http://ip/api/live - live data (OBD/GPS/MEMS)
* http://ip/api/list - list of log files
* http://ip/api/log - raw CSV format log file
* http://ip/api/data - timestamped PID data in JSON array
*************************************************************************/

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
#include <httpd.h>
#include "config.h"

#define WIFI_TIMEOUT 5000

extern uint32_t fileid;
extern float accBias[];
extern float acc[];
extern char vin[];
extern byte obdPID[];
extern int16_t obdValue[];
extern GPS_DATA gd;

extern "C"
{
uint8_t temprature_sens_read();
uint32_t hall_sens_read();
}

HttpParam httpParam;


int handlerInfo(UrlHandlerParam* param)
{
    char *buf = param->pucBuffer;
    int bufsize = param->bufSize;
    int bytes = snprintf(buf, bufsize, "{\"httpd\":{\"uptime\":%u,\"clients\":%u,\"requests\":%u,\"traffic\":%u},\n",
        millis(), httpParam.stats.clientCount, httpParam.stats.reqCount, (unsigned int)(httpParam.stats.totalSentBytes >> 10));

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

    bytes += snprintf(buf + bytes, bufsize - bytes, "\"spiffs\":{\"total\":%u,\"used\":%u}",
        SPIFFS.totalBytes(), SPIFFS.usedBytes());

    if (bytes < bufsize - 1) buf[bytes++] = '}';

    param->contentLength = bytes;
    param->fileType=HTTPFILETYPE_JSON;
    return FLAG_DATA_RAW;
}

int handlerLogFile(UrlHandlerParam* param)
{
    int id = mwGetVarValueInt(param->pxVars, "id", 0);
    sprintf(param->pucBuffer, "DATA/%u.CSV", id == 0 ? fileid : id);
    param->fileType=HTTPFILETYPE_TEXT;
    return FLAG_DATA_FILE;
}

typedef struct {
    FILE* fp;
    uint32_t tsStart;
    uint32_t tsEnd;
    uint16_t pid;
} DATA_CONTEXT;

int handlerLogData(UrlHandlerParam* param)
{
    uint32_t duration = 0;
    DATA_CONTEXT* ctx = (DATA_CONTEXT*)param->hs->ptr;
    if (ctx) {
		if (!param->pucBuffer) {
			// connection being closed, last calling, cleanup
            Serial.println("CLEAN UP");
			fclose(ctx->fp);
            free(ctx);
			param->hs->ptr = 0;
			return 0;
		}
    } else {
        int id = mwGetVarValueInt(param->pxVars, "id", 0);
        sprintf(param->pucBuffer, "%s/DATA/%u.CSV", httpParam.pchWebPath, id == 0 ? fileid : id);
        FILE *fp = fopen(param->pucBuffer, "r");
        if (!fp) {
            Serial.print("Unable to open ");
            Serial.println(param->pucBuffer);
            return 0;
        }
        param->hs->ptr = calloc(1, sizeof(DATA_CONTEXT));
        ctx = (DATA_CONTEXT*)param->hs->ptr;
        ctx->fp = fp;
        ctx->pid = mwGetVarValueInt(param->pxVars, "pid", 0);
        ctx->tsStart = mwGetVarValueInt(param->pxVars, "start", 0);
        ctx->tsEnd = 0xffffffff;
        duration = mwGetVarValueInt(param->pxVars, "duration", 0);
        if (ctx->tsStart && duration) {
            ctx->tsEnd = ctx->tsStart + duration;
            duration = 0;
        }
        // JSON head
        param->contentLength = sprintf(param->pucBuffer, "[");
    }
    
    int len = 0;
    char buf[64];
    uint32_t ts = 0;

    for (;;) {
        int c = fgetc(ctx->fp);
        if (c < 0) {
            if (param->contentLength == 0) {
                // no more data
                Serial.println("EOF");
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
    param->fileType = HTTPFILETYPE_JSON;
    return FLAG_DATA_STREAM;
}

int handlerLogList(UrlHandlerParam* param)
{
    char *buf = param->pucBuffer;
    int bufsize = param->bufSize;
    fs::File root = SPIFFS.open("/");
    int n = snprintf(buf + n, bufsize - n, "[");
    if (root) {
        fs::File file;
        while(file = root.openNextFile()) {
            if (!strncmp(file.name(), "/DATA/", 6)) {
                Serial.print(file.name());
                Serial.print(' ');
                Serial.print(file.size());
                Serial.println(" bytes");
                unsigned int id = atoi(file.name() + 6);
                if (id) {
                    n += snprintf(buf + n, bufsize - n, "{\"id\":%u,\"size\":%u",
                        id, file.size());
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
    param->fileType=HTTPFILETYPE_JSON;
    param->contentLength = n;
    return FLAG_DATA_RAW;
}

int handlerLiveData(UrlHandlerParam* param)
{
    char *buf = param->pucBuffer;
    int bufsize = param->bufSize;
    int n = snprintf(buf + n, bufsize - n, "{\"obd\":{\"vin\":\"%s\",\"pid\":[", vin);
    for (int i = 0; obdPID[i]; i++) {
        n += snprintf(buf + n, bufsize - n, "[%u,%d],", 0x100 | obdPID[i], obdValue[i]);
    }
    n--;
    n += snprintf(buf + n, bufsize - n, "]}");
    n += snprintf(buf + n, bufsize - n, ",\"mems\":{\"acc\":{\"x\":\"%f\",\"y\":\"%f\",\"z\":\"%f\"}}",
        acc[0] - accBias[0], acc[1] - accBias[1], acc[2] - accBias[2]);
    n += snprintf(buf + n, bufsize - n, ",\"gps\":{\"lat\":%d,\"lng\":%d,\"alt\":%d,\"speed\":%d,\"sat\":%d}}",
        gd.lat, gd.lng, gd.alt, gd.speed, gd.sat);
    param->contentLength = n;
    param->fileType=HTTPFILETYPE_JSON;
    return FLAG_DATA_RAW;
}

UrlHandler urlHandlerList[]={
    {"api/live", handlerLiveData},
    {"api/info", handlerInfo},
    {"api/list", handlerLogList},
#if STORAGE == STORAGE_SPIFFS
    {"api/data", handlerLogData},
    {"api/log", handlerLogFile},
#endif
{0}
};

void obtainTime()
{
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, (char*)"pool.ntp.org");
    sntp_init();
}

void serverProcess(int timeout)
{
    mwHttpLoop(&httpParam, timeout);
}

void serverCheckup()
{
#if ENABLE_WIFI_STATION
    static uint32_t wifiStartTime = 0;
    if (WiFi.status() != WL_CONNECTED) {
        if (wifiStartTime == 0 || millis() - wifiStartTime > WIFI_JOIN_TIMEOUT) {
            WiFi.disconnect(false);
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

            wifiStartTime = 0;
        }

    }
#endif
}

void listAPs()
{
    int n = WiFi.scanNetworks();
    if (n <= 0) {
        Serial.println("No WiFi AP found");
    } else {
        Serial.println("WiFi APs found:");
        for (int i = 0; i < n; ++i) {
            // Print SSID and RSSI for each network found
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(WiFi.SSID(i));
            Serial.print(" (");
            Serial.print(WiFi.RSSI(i));
            Serial.println("dB)");
        }
    }
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
    listAPs();
#endif

#if ENABLE_WIFI_AP
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
#endif

    mwInitParam(&httpParam, 80, "/spiffs");
    httpParam.pxUrlHandler = urlHandlerList;

    if (mwServerStart(&httpParam)) {
        Serial.println("Error starting HTTPd");
        return false;
    }

    obtainTime();

    serverCheckup();
    return true;
}
