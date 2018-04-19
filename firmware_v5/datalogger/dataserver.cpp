/*************************************************************************
* OBD/MEMS/GPS Data Logger for Freematics ONE+
* HTTP server for trip data access via WIFI
*
* Distributed under BSD license
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

#include <Arduino.h>
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

#ifdef ESP32
extern "C"
{
uint8_t temprature_sens_read();
uint32_t hall_sens_read();
}
#endif

HttpParam httpParam;

extern uint32_t fileid;

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

int handlerLogList(UrlHandlerParam* param)
{
    char *buf = param->pucBuffer;
    int bufsize = param->bufSize;
    fs::File root = SPIFFS.open("/");
    int n = snprintf(buf + n, bufsize - n, "{\"trips\":[");
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
    n += snprintf(buf + n, bufsize - n, "]}");
    param->fileType=HTTPFILETYPE_JSON;
    param->contentLength = n;
    return FLAG_DATA_RAW;
}

UrlHandler urlHandlerList[]={
  {"info", handlerInfo},
  {"list", handlerLogList},
#if STORAGE == STORAGE_SPIFFS
  {"log", handlerLogFile},
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
            obtainTime();

            wifiStartTime = 0;
        }

    }
#endif
}

void listAPs()
{
    int n = WiFi.scanNetworks();
    if (n <= 0) {
        Serial.println("No WIFI AP found");
    } else {
        Serial.println("WIFI APs found:");
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

#if ENABLE_WIFI_STATION
    listAPs();
#endif

#if ENABLE_WIFI_AP
    WiFi.softAP(WIFI_AP_SSID);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
#endif

    mwInitParam(&httpParam, 80, "/spiffs");
    httpParam.pxUrlHandler = urlHandlerList;

    if (mwServerStart(&httpParam)) {
        Serial.println("Error starting HTTPd");
        return false;
    }

    serverCheckup();
    return true;
}
