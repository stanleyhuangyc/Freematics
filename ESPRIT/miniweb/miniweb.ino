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

#include <WiFi.h>
#include <ESPmDNS.h>
#include <apps/sntp/sntp.h>
#include <httpd.h>

#ifdef ESP32
extern "C"
{
uint8_t temprature_sens_read();
uint32_t hall_sens_read();
}
#endif

#define WIFI_SSID "YOUR_SSID"
#define WIFI_PASSWORD "YOUR_PASSWORD"

HttpParam httpParam;

int handlerInfo(UrlHandlerParam* param)
{
  char *buf = param->pucBuffer;
  int bufsize = param->dataBytes;
  int bytes = snprintf(buf, bufsize, "{\"tick\":%u,", millis());

  time_t now;
  time(&now);
  struct tm timeinfo = { 0 };
  localtime_r(&now, &timeinfo);
  if (timeinfo.tm_year) {
    bytes += snprintf(buf + bytes, bufsize - bytes, "\"date\":\"%04u-%02u-%02u\",\"time\":\"%02u:%02u:%02u\",",
      timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
      timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  }

  IPAddress ip = WiFi.localIP();
  bytes += snprintf(buf + bytes, bufsize - bytes, "\"IP\":\"%u.%u.%u.%u\",", ip[0], ip[1], ip[2], ip[3]);

#ifdef ESP32
  int deviceTemp = (int)temprature_sens_read() * 165 / 255 - 40;
  bytes += snprintf(buf + bytes, bufsize - bytes, "\"temperature\":%d,", deviceTemp);
  bytes += snprintf(buf + bytes, bufsize - bytes, "\"magnetic\":%d", hall_sens_read());
#endif

  buf[bytes++] = '}';

  param->dataBytes = bytes;
  param->fileType=HTTPFILETYPE_JSON;
  return FLAG_DATA_RAW;
}

UrlHandler urlHandlerList[]={
	{"info", handlerInfo},
  {0}
};

void obtainTime(void)
{
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}

void setup()
{
  delay(500);
  Serial.begin(115200);
  Serial.print("Connecting...");
  delay(500);

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

  mwInitParam(&httpParam, 80, "app");
	//httpParam.hlBindIP = htonl(INADDR_LOOPBACK);
  httpParam.pxUrlHandler=urlHandlerList;
  httpParam.maxClients = 4;
  if (mwServerStart(&httpParam)) {
  		Serial.println("Error starting");
      for (;;);
  }
  Serial.println("MiniWeb started");
}

void loop()
{
  mwHttpLoop(&httpParam, 500);
}
