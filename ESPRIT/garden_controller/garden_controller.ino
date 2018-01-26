/******************************************************************************
* Garden Controller based on Freematics Esprit ESP32 development board
* Distributed under BSD license
* Developed by Stanley Huang stanley@freematics.com.au
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
#include <Esprit.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <apps/sntp/sntp.h>
#include <httpd.h>
#include <ArduinoOTA.h>

#define PIN_PWM_A D3
#define PIN_DIR_A D12
#define PIN_PWM_B D11
#define PIN_DIR_B D13

#define PIN_VOLTAGE_SENSOR A4
#define PIN_CURRENT_SENSOR A5

#define PIN_LED 4

#define ENABLE_SOFT_AP 1
#define WIFI_AP_SSID "GARDEN"
#define WIFI_SSID "FREEMATICS"
#define WIFI_PASSWORD "PASSWORD"
#define WIFI_TIMEOUT 5000

extern "C"
{
uint8_t temprature_sens_read();
uint32_t hall_sens_read();
}

hw_timer_t * timer = 0;

bool updating = false;
struct {
  int power = 0;
  int onVoltage = 12500; /* mV */
  int offVoltage = 11000; /* mV */
  int maxTime = 1800 /* seconds */;
  uint32_t startTime = 0;
} pump;
struct {
  int power = 0;
  int brightness = 255; /* 0~255 */
  int onTime = 2000; /* hhmm */
  int maxTime = 7200; /* seconds */
  int offVoltage = 9000; /* mV */
  uint32_t startTime = 0;
} light;
struct {
  int voltage = 0;
  int current = 0;
} solar;

HttpParam httpParam;

void apply()
{
  if (!pump.power) {
    digitalWrite(PIN_PWM_B, LOW);
  } else {
    digitalWrite(PIN_PWM_B, HIGH);
    pump.startTime = millis();
  }
  if (light.power) {
    ledcWrite(0, light.brightness);
    light.startTime = millis();
  } else {
    ledcWrite(0, 0);
  }
}

void automation()
{
  if (pump.power) {
    if (solar.voltage < pump.offVoltage || millis() - pump.startTime > pump.maxTime) {
      pump.power = 0;
    }
  } else {
    if (solar.voltage >= pump.onVoltage) {
      pump.power = 1;
    }
  }
  if (light.power) {
    if (solar.voltage < light.offVoltage || millis() - light.startTime > light.maxTime) {
      pump.power = 0;
    }
  } else {
    if (light.onTime != -1) {
      time_t now;
      time(&now);
      struct tm timeinfo = { 0 };
      localtime_r(&now, &timeinfo);
      if (timeinfo.tm_year && timeinfo.tm_hour * 100 + timeinfo.tm_min >= light.onTime) {
        light.power= 1;
      }
    }
  }
}

void keepalive()
{
  if (WiFi.status() != WL_CONNECTED) {
    // reconnect WIFI
    if (httpParam.bWebserverRunning) mwServerShutdown(&httpParam);
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    // Wait for connection
    Serial.print("Reconnecting WIFI");
    for (unsigned long t = millis(); WiFi.status() != WL_CONNECTED && millis() - t < WIFI_TIMEOUT; ) {
        Serial.print(".");
        delay(500);
    }
    Serial.println();
  }
  if (WiFi.status() == WL_CONNECTED && !httpParam.bWebserverRunning) {
    Serial.println("Restarting HTTP server");
    mwServerStart(&httpParam);
  }
}

int handlerRoot(UrlHandlerParam* param)
{
  digitalWrite(PIN_LED, HIGH);
  //IPAddress ip = WiFi.softAPIP();
  IPAddress ip = WiFi.localIP();
  param->contentLength = snprintf(param->pucBuffer, param->bufSize,
    "<html><head><title>Garden Controller</title></head><body><h3>Hello from Garden Controller (%u.%u.%u.%u)</h3><ul><li>Up time: %u seconds</li><li>Connected clients: %u</li></body>",
    ip[0], ip[1], ip[2], ip[3],
    millis() / 1000, httpParam.stats.clientCount);
  param->fileType = HTTPFILETYPE_HTML;
  digitalWrite(PIN_LED, LOW);
  return FLAG_DATA_RAW;
}

int handlerControl(UrlHandlerParam* param)
{
  pump.power = mwGetVarValueInt(param->pxVars, "pump.power", pump.power);
  pump.maxTime = mwGetVarValueInt(param->pxVars, "pump.maxTime", pump.maxTime);
  pump.onVoltage = mwGetVarValueInt(param->pxVars, "pump.onVoltage", pump.onVoltage);
  pump.offVoltage = mwGetVarValueInt(param->pxVars, "pump.offVoltage", pump.offVoltage);
  light.power = mwGetVarValueInt(param->pxVars, "light.power", light.power);
  light.brightness = mwGetVarValueInt(param->pxVars, "light.brightness", light.brightness);
  light.onTime = mwGetVarValueInt(param->pxVars, "light.onTime", light.onTime);
  light.maxTime = mwGetVarValueInt(param->pxVars, "light.maxTime", light.maxTime);
  
  apply();
  automation();

  param->contentLength = snprintf(param->pucBuffer, param->bufSize,
    "{\"pump\":{\"power\":%u,\"maxTime\":%u,\"onVoltage\":%u,\"offVoltage\":%u},\"light\":{\"power\":%u,\"brightness\":%u,\"onTime\":%u,\"maxTime\":%u},\"voltage\":%u}",
    pump.power, pump.maxTime, pump.onVoltage, pump.offVoltage,
    light.power, light.brightness, light.onTime, light.maxTime, solar.voltage);
  param->fileType=HTTPFILETYPE_JSON;
  return FLAG_DATA_RAW;
}

int handlerInfo(UrlHandlerParam* param)
{
  char *buf = param->pucBuffer;
  int bufsize = param->bufSize;
  int bytes = snprintf(buf, bufsize, "{\"server\":{\"uptime\":%u,\"clients\":%u,\"requests\":%u",
    millis(), httpParam.stats.clientCount, httpParam.stats.reqCount);

  time_t now;
  time(&now);
  struct tm timeinfo = { 0 };
  localtime_r(&now, &timeinfo);
  if (timeinfo.tm_year) {
    bytes += snprintf(buf + bytes, bufsize - bytes, ",\"date\":\"%04u-%02u-%02u\",\"time\":\"%02u:%02u:%02u\"",
      timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
      timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  }
  buf[bytes++] = '}';

  int deviceTemp = (int)temprature_sens_read() * 165 / 255 - 40;
  bytes += snprintf(buf + bytes, bufsize - bytes,
    ",\"cpu\":{\"temperature\":%d,\"magnetic\":%d}",
    deviceTemp, hall_sens_read());

  bytes += snprintf(buf + bytes, bufsize - bytes,
    ",\"solar\":{\"voltage\":%d,\"current\":%d}",
    solar.voltage, solar.current);

  if (bytes < bufsize - 1) {
    buf[bytes++] = '}';
    buf[bytes] = 0;
  }

  param->contentLength = bytes;
  param->fileType=HTTPFILETYPE_JSON;
  return FLAG_DATA_RAW;
}

UrlHandler urlHandlerList[]={
	{"info", handlerInfo},
  {"control", handlerControl},
  {"", handlerRoot},
  {0}
};


void obtainTime()
{
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, (char*)"pool.ntp.org");
    sntp_init();
}

void httpLoop(void* param)
{
  for (;;) {
    Serial.print("Web server started on core ");
    Serial.println(xPortGetCoreID());
    while (!httpParam.bKillWebserver) {
      mwHttpLoop(&httpParam, 100);
      ArduinoOTA.handle();
    }
    mwServerExit(&httpParam);
    while (updating) {
      ArduinoOTA.handle();
    }
    while (!httpParam.bWebserverRunning) delay(500);
  }
}

void setup()
{
  delay(1000);
  Serial.begin(115200);
  Serial.print("ESP32 ");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.println("MHz");

  // light up onboard LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  pinMode(PIN_PWM_A, OUTPUT);
  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_PWM_B, OUTPUT);
  pinMode(PIN_DIR_B, OUTPUT);
  
  digitalWrite(PIN_DIR_A, HIGH);
  // set PWM output (10khz, 8bit)
  ledcSetup(0, 10000, 8);
  ledcAttachPin(PIN_PWM_A, 0);
  
  apply();

#if ENABLE_SOFT_AP
  WiFi.softAP(WIFI_AP_SSID, WIFI_PASSWORD);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
#endif

  do {
    Serial.print("Connecting...");

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

  obtainTime();

  Serial.print("Loop task running on core ");
  Serial.println(xPortGetCoreID());

  mwInitParam(&httpParam, 80, 0);
  httpParam.pxUrlHandler = urlHandlerList;
  if (mwServerStart(&httpParam) == 0) {
    // start HTTP server thread on core 0
    xTaskCreatePinnedToCore(httpLoop, "http", 20480, 0, 0, 0, 0);
  }
  // 
  // turn off onboard LED
  digitalWrite(PIN_LED, LOW);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
    // turn off all functions
    updating = true;
    pump.power = 0;
    light.power = 0;
    apply();
    mwServerShutdown(&httpParam);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  if (!MDNS.begin("garden")) {
    Serial.println("Error setting up MDNS responder!");
  } else {
    Serial.println("mDNS responder started");
  }
  // Add service to MDNS-SD
  MDNS.addService("http", "tcp", 80);
  MDNS.addService("ota", "udp", 3232);
}

uint32_t vsum = 0;
uint32_t asum = 0;
int count = 0;

void sense()
{
  static byte lastSec = 0;
  static byte lastMin = 0;
  uint32_t t = millis();
  byte s = t / 1000;
  byte m = t / 60000;
  vsum += analogRead(PIN_VOLTAGE_SENSOR);
  asum += analogRead(PIN_CURRENT_SENSOR);
  count++;
  if (s != lastSec) {
    // reach here every other second
    solar.voltage = vsum * 413 / 100 / count; /* mV */
    solar.current = asum * 4792 / 10000 / count; /* mA */
    Serial.print((float)solar.voltage / 1000, 1);
    Serial.print("V ");
    Serial.print((float)solar.current / 1000, 2);
    Serial.println("A");
    vsum = 0;
    asum = 0;
    count = 0;
    lastSec = s;
    automation();
  }
  if (m != lastMin) {
    keepalive();
    lastMin = m;
  }
}

void loop()
{
  if (updating) return;
  if (!pump.power) {
    sense();
    delay(20);
  } else {
    // generate 50Hz sqaure wave (20ms cycle)
    uint32_t t = millis();
    int t2;
    // change current direction
    digitalWrite(PIN_DIR_B, HIGH);
    // minimum 10ms duration
    if ((t2 = 10 - (int)(millis() - t)) > 0) delay(t2);
    // change current direction
    digitalWrite(PIN_DIR_B, LOW);
    // read sensors
    sense();
    // minimum 20ms duration
    if ((t2 = 20 - (int)(millis() - t)) > 0) delay(t2);
  }
}
