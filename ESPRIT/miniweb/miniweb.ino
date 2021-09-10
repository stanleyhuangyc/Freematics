/******************************************************************************
* MiniWeb example for Freematics Esprit
* Distributed under BSD license
* Developed by Stanley Huang https://facebook.com/stanleyhuangyc
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
******************************************************************************/

#include <Arduino.h>
#if defined(ESP32)
#include <WiFi.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <apps/sntp/sntp.h>
#include <esp_spi_flash.h>
#include <esp_err.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#else
#error Unsupported board type
#endif
#include <httpd.h>

#define ENABLE_WIFI_AP 1
#define ENABLE_WIFI_STATION 0
#define WIFI_SSID "FREEMATICS"
#define WIFI_PASSWORD "PASSWORD"
#define WIFI_AP_SSID "MINIWEB"
#define WIFI_TIMEOUT 5000
#define PIN_LED 4

HttpParam httpParam;

int handlerRoot(UrlHandlerParam* param)
{
  digitalWrite(PIN_LED, HIGH);
#if ENABLE_WIFI_STATION
  IPAddress ip = WiFi.localIP();
#else
  IPAddress ip = WiFi.softAPIP();
#endif
  param->contentLength = snprintf(param->pucBuffer, param->bufSize,
    "<html><head><title>MiniWeb for Arduino</title></head><body><h3>Hello from MiniWeb (%u.%u.%u.%u)</h3><ul><li>Up time: %u seconds</li><li>Connected clients: %u</li><li>Total requests: %u</li></body>",
    ip[0], ip[1], ip[2], ip[3],
    millis() / 1000, httpParam.stats.clientCount, httpParam.stats.reqCount);
  param->contentType = HTTPFILETYPE_HTML;
  digitalWrite(PIN_LED, LOW);
  return FLAG_DATA_RAW;
}

int handlerDigitalPins(UrlHandlerParam* param)
{
  if (!param->pxVars) return 0; // return 404 when no URL arguments
  char *buf = param->pucBuffer;
  int bufsize = param->bufSize;
  int gpio = -1;
  int level = -1;
  int bytes = snprintf(buf, bufsize, "{");
  for (int i = 0; param->pxVars[i].name; i++) {
    if (!strncmp(param->pxVars[i].name, "gpio", 4)) {
      gpio = atoi(param->pxVars[i].name + 4);
      if (*param->pxVars[i].value) level = atoi(param->pxVars[i].value);
      if (gpio == -1 || gpio > 39) {
        bytes += snprintf(buf + bytes, bufsize - bytes, "\"Error\":\"invalid pin number\",");
        break;
      }
      if (level == -1) {
        // reading pin
        pinMode(gpio, INPUT);
        level = digitalRead(gpio);
      } else {
        // writing pin
        pinMode(gpio, OUTPUT);
        digitalWrite(gpio, level);
      }
      bytes += snprintf(buf + bytes, bufsize - bytes, "\"GPIO%u\":%u,", gpio, level);
    }
  }
  if (buf[bytes - 1] == ',') bytes--;
  buf[bytes++] = '}';
  param->contentLength = bytes;
  param->contentType = HTTPFILETYPE_JSON;
  return FLAG_DATA_RAW;
}

int handlerAnalogPins(UrlHandlerParam* param)
{
  if (!param->pxVars) return 0; // return 404 when no URL arguments
  char *buf = param->pucBuffer;
  int bufsize = param->bufSize;
  int gpio = -1;
  int bytes = snprintf(buf, bufsize, "{");
  for (int i = 0; param->pxVars[i].name; i++) {
    if (!strncmp(param->pxVars[i].name, "gpio", 4)) {
      gpio = atoi(param->pxVars[i].name + 4);
      if (gpio == -1 || gpio > 39) {
        bytes += snprintf(buf + bytes, bufsize - bytes, "\"Error\":\"invalid pin number\",");
        break;
      }
      int level = analogRead(gpio);
      bytes += snprintf(buf + bytes, bufsize - bytes, "\"GPIO%u\":%d,", gpio, level);
    }
  }
  if (buf[bytes - 1] == ',') bytes--;
  buf[bytes++] = '}';
  param->contentLength = bytes;
  param->contentType = HTTPFILETYPE_JSON;
  return FLAG_DATA_RAW;
}

extern "C" int handlerSerial(UrlHandlerParam* param)
{
	int timeout = mwGetVarValueInt(param->pxVars, "timeout", 1000);
  param->contentType=HTTPFILETYPE_TEXT;

	if (!strcmp(param->pucRequest, "/read")) {
		if (Serial1) {
      int matched = 0;
			char* eos = mwGetVarValue(param->pxVars, "eos", 0);
      size_t maxlen = mwGetVarValueInt(param->pxVars, "bytes", param->bufSize) - 1;
      size_t bytes = 0;
      uint32_t t = GetTickCount();
      do {
        while (Serial1.available() && bytes < maxlen) {
          param->pucBuffer[bytes++] = Serial1.read();;
        }
        param->pucBuffer[bytes] = 0;
        if (eos) {
          if (strstr(param->pucBuffer, eos)) {
            // matched
            matched = 1;
          }
        }
      } while (bytes < maxlen && !matched && GetTickCount() - t <= timeout);
      if (eos && !matched) {
        param->hs->response.statusCode = 504;
      }
      param->contentLength = bytes;
		}
		else {
			param->contentLength = sprintf(param->pucBuffer, "No port opened");
			param->hs->response.statusCode = 503;
		}
	} else if (!strcmp(param->pucRequest, "/write")) {
		if (Serial1) {
      int written = 0;
      if (param->hs->request.payloadSize > 0) {
        int payloadSize = param->hs->request.payloadSize;
        written = Serial1.write((uint8_t*)param->pucPayload, payloadSize);
      }
      param->contentLength = sprintf(param->pucBuffer, "%d", written);
			if (written <= 0) {
				param->hs->response.statusCode = 504;
      }
		} else {
			param->contentLength = sprintf(param->pucBuffer, "No port opened");
			param->hs->response.statusCode = 503;
		}
  } else if (!strcmp(param->pucRequest, "/command")) {
    char* cmd = mwGetVarValue(param->pxVars, "cmd", "");
    char* eos = mwGetVarValue(param->pxVars, "eos", 0);
    Serial1.write(cmd);
    Serial1.write('\r');
    if (eos) {
      int matched = 0;
      size_t maxlen = param->bufSize - 1;
      size_t bytes = 0;
      uint32_t t = GetTickCount();
      do {
        if (!Serial1.available()) continue;
        param->pucBuffer[bytes++] = Serial1.read();;
        param->pucBuffer[bytes] = 0;
        if (strstr(param->pucBuffer, eos)) {
          // matched
          matched = 1;
        }
      } while (bytes < maxlen && !matched && GetTickCount() - t <= timeout);
      if (!matched) {
        param->hs->response.statusCode = 504;
      }
      param->contentLength = bytes;
    } else {
      param->contentLength = 0;
    }
  } else if (!strcmp(param->pucRequest, "/open")) {
    int pinRx = mwGetVarValueInt(param->pxVars, "rx", 16);
    int pinTx = mwGetVarValueInt(param->pxVars, "tx", 17);
    int baudrate = mwGetVarValueInt(param->pxVars, "baudrate", 115200);
    //char* proto = mwGetVarValue(param->pxVars, "protocol", "8N1");
    Serial1.begin(baudrate, SERIAL_8N1, pinRx, pinTx);
    param->contentLength = sprintf(param->pucBuffer, "OK");
  } else if (!strcmp(param->pucRequest, "/close")) {
    Serial1.end();
    param->contentLength = sprintf(param->pucBuffer, "OK");
  } else {
      return 0;
  }
	return FLAG_DATA_RAW;
}

int handlerInfo(UrlHandlerParam* param)
{
  char *buf = param->pucBuffer;
  int bufsize = param->bufSize;
  int bytes = snprintf(buf, bufsize, "{\"uptime\":%u,\"clients\":%u,\"requests\":%u,\"traffic\":%u,",
    millis(), httpParam.stats.clientCount, httpParam.stats.reqCount, (unsigned int)(httpParam.stats.totalSentBytes >> 10));

#ifdef ESP32
  time_t now;
  time(&now);
  struct tm timeinfo = { 0 };
  localtime_r(&now, &timeinfo);
  if (timeinfo.tm_year) {
    bytes += snprintf(buf + bytes, bufsize - bytes, "\"date\":\"%04u-%02u-%02u\",\"time\":\"%02u:%02u:%02u\",",
      timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
      timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  }
#endif
  if (bytes < bufsize - 1) buf[bytes++] = '}';

  param->contentLength = bytes;
  param->contentType=HTTPFILETYPE_JSON;
  return FLAG_DATA_RAW;
}

UrlHandler urlHandlerList[]={
	{"info", handlerInfo},
  {"digital", handlerDigitalPins},
  {"analog", handlerAnalogPins},
  {"serial", handlerSerial},
  {0}
};

void obtainTime()
{
#ifdef ESP32
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, (char*)"pool.ntp.org");
    sntp_init();
#endif
}

uint16_t getFlashSize()
{
#ifdef ESP32
    char out;
    uint16_t size = 16384;
    do {
        if (spi_flash_read((size_t)size * 1024 - 1, &out, 1) == ESP_OK) return size;
    } while ((size >>= 1));
    return 0;
#else
  return 4096;
#endif
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);
    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
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

void setup()
{
  // light up onboard LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  // initialize serial
  Serial.begin(115200);
  Serial.println();
  Serial.print("ESP32 ");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.print("MHz ");
  Serial.print(getFlashSize() >> 10);
  Serial.println("MB Flash");

#if ENABLE_WIFI_AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_AP_SSID);
  Serial.print("WiFi AP IP address: ");
  Serial.println(WiFi.softAPIP());
#endif
#if ENABLE_WIFI_STATION
  do {
    Serial.print("Connecting to WiFi (");
    Serial.print(WIFI_SSID);
    Serial.print(')');

    // Connect to WiFi network
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    // Wait for connection
    for (unsigned long t = millis(); WiFi.status() != WL_CONNECTED && millis() - t < WIFI_TIMEOUT; ) {
        Serial.print(".");
        delay(500);
    }
    Serial.println();
  } while (WiFi.status() != WL_CONNECTED);

  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (!MDNS.begin("miniweb")) {
    Serial.println("Error setting up MDNS responder!");
  } else {
    Serial.println("mDNS responder started");
  }
  // Add service to MDNS-SD
  MDNS.addService("http", "tcp", 80);

  obtainTime();
#endif

  mwInitParam(&httpParam, 80, "/htdoc");
  httpParam.pxUrlHandler = urlHandlerList;
  httpParam.maxClientsPerIP = 4;
  httpParam.maxClients = 8;

  if (SPIFFS.begin(true, httpParam.pchWebPath)) {
    //Serial.println();
    //listDir(SPIFFS, "/", 0);
    //Serial.println();
  }

  if (mwServerStart(&httpParam)) {
  		Serial.println("Error starting");
      for (;;);
  }
  Serial.println("MiniWeb started");
  // turn off onboard LED
  digitalWrite(PIN_LED, LOW);
}

void loop()
{
  mwHttpLoop(&httpParam, 1000);
}
