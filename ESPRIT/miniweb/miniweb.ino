/******************************************************************************
* MiniWeb example for ESP32
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
#include <apps/sntp/sntp.h>
#else
#error Unsupported board type
#endif
#include <httpd.h>

#define WIFI_SSID "...."
#define WIFI_PASSWORD "...."
#define PIN_LED 4

#ifdef ESP32
extern "C"
{
uint8_t temprature_sens_read();
uint32_t hall_sens_read();
}
#endif

HttpParam httpParam;

int handlerRoot(UrlHandlerParam* param)
{
  digitalWrite(PIN_LED, HIGH);
  IPAddress ip = WiFi.localIP();
  param->contentLength = snprintf(param->pucBuffer, param->bufSize,
    "<html><head><title>MiniWeb for Arduino</title></head><body><h3>Hello from MiniWeb (%u.%u.%u.%u)</h3><ul><li>Up time: %u seconds</li><li>Connected clients: %u</li><li>Total requests: %u</li></body>",
    ip[0], ip[1], ip[2], ip[3],
    millis() / 1000, httpParam.stats.clientCount, httpParam.stats.reqCount);
  param->fileType = HTTPFILETYPE_HTML;
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
  param->fileType = HTTPFILETYPE_JSON;
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
  param->fileType = HTTPFILETYPE_JSON;
  return FLAG_DATA_RAW;
}

int handlerInfo(UrlHandlerParam* param)
{
  char *buf = param->pucBuffer;
  int bufsize = param->bufSize;
  int bytes = snprintf(buf, bufsize, "{\"uptime\":%u,\"clients\":%u,\"requests\":%u,\"traffic\":%u,",
    millis(), httpParam.stats.clientCount, httpParam.stats.reqCount, (unsigned int)(httpParam.stats.fileSentBytes >> 10));

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

  int deviceTemp = (int)temprature_sens_read() * 165 / 255 - 40;
  bytes += snprintf(buf + bytes, bufsize - bytes, "\"temperature\":%d,", deviceTemp);
  bytes += snprintf(buf + bytes, bufsize - bytes, "\"magnetic\":%d", hall_sens_read());
#endif
  if (bytes < bufsize - 1) buf[bytes++] = '}';

  param->contentLength = bytes;
  param->fileType=HTTPFILETYPE_JSON;
  return FLAG_DATA_RAW;
}

UrlHandler urlHandlerList[]={
	{"info", handlerInfo},
  {"digital", handlerDigitalPins},
  {"analog", handlerAnalogPins},
  {"", handlerRoot},
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

void setup()
{
  // light up onboard LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  // initialize serial
  Serial.begin(115200);
  Serial.print("Connecting...");

  // Connect to WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  delay(1000);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(500);
  }
  Serial.println("");
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

  mwInitParam(&httpParam, 80, 0);
  httpParam.pxUrlHandler = urlHandlerList;
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
  mwHttpLoop(&httpParam, 500);
}
