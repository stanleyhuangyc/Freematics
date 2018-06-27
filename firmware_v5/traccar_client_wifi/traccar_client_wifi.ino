/******************************************************************************
* Traccar GPS tracker client for Freematics ONE+ and Freematics Esprit
* Written by Stanley Huang <stanley@freematics.com.au>
* Distributed under BSD license
* Visit https://freematics.com/products for hardware information
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
#include <WiFi.h>
#include <TinyGPS.h>

#define PIN_LED 4
#define PIN_GPS_POWER 15
#define PIN_GPS_UART_RXD 32
#define PIN_GPS_UART_TXD 33

#define GPS_BAUDRATE 115200
#define WIFI_SSID "YOUR_SSID"
#define WIFI_PASSWORD "YOUR_PASSWORD"
#define WIFI_JOIN_INTERVAL 15000

#define TRACCAR_HOST "YOUR_TRACCAR_HOST"
#define TRACCAR_PORT 5055
#define TRACCAR_DEV_ID "YOUR_DEVICE_ID"

HardwareSerial Serial1(1);
TinyGPS gps;
WiFiClient client;

bool checkWiFi()
{
  // check WiFi connection and re-connect if disconnected
  static uint32_t wifiStartTime = 0;
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiStartTime == 0 || millis() - wifiStartTime > WIFI_JOIN_INTERVAL) {
        Serial.println();
        Serial.print("Connecting to WiFi - SSID:");
        Serial.println(WIFI_SSID);
        WiFi.disconnect(false);
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        wifiStartTime = millis();
    }
    return false;
  } else {
    if (wifiStartTime) {
      // just connected
      Serial.println();
      Serial.print("WiFi connected. IP:");
      Serial.println(WiFi.localIP());
      wifiStartTime = 0;
    }
    return true;
  }
}

void setup()
{
  // turn on indicator LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  // turn on GPS power
  pinMode(PIN_GPS_POWER, OUTPUT);
  digitalWrite(PIN_GPS_POWER, HIGH);

  // USB serial
  Serial.begin(115200);
  delay(500);
  Serial.println("TRACCAR CLIENT");

  // initialize WiFi
  checkWiFi();

  // start serial UART where GPS receiver is connected
  Serial1.begin(GPS_BAUDRATE, SERIAL_8N1, PIN_GPS_UART_RXD, PIN_GPS_UART_TXD);
  Serial.print("Waiting for GPS signal ");

  // turn off indicator LED
  digitalWrite(PIN_LED, LOW);
}

void loop()
{
  static unsigned long lastutc = 0;

  // check incoming NMEA data from GPS
  if (!Serial1.available()) {
    return;
  }

  // read a character of NMEA stream from GPS
  char c = Serial1.read();
  if (lastutc == 0) {
    // display progress before GPS has signal
    if (c == '\n') Serial.write('.');
  }

  // decode NMEA stream
  if (!gps.encode(c)) {
    return;
  }

  // check WiFi connection and connect to Traccar server if disconnected
  if (checkWiFi()) {
    if (!client.connected()) {
      Serial.print("Connecting to ");
      Serial.print(TRACCAR_HOST);
      Serial.print("...");
      if (client.connect(TRACCAR_HOST, TRACCAR_PORT)) {
        Serial.println("OK");
      } else {
        Serial.println("failed");
        delay(3000);
        return;
      }
    }
  }

  // check UTC timestamp from GPS
  unsigned long utcdate, utctime;
  gps.get_datetime(&utcdate, &utctime, 0);
  if (utctime == lastutc) {
    return;
  }

  // turn on  indicator LED
  digitalWrite(PIN_LED, HIGH);

  // now that new GPS coordinates are available
  long lat, lng;
  gps.get_position(&lat, &lng, 0);
  long speed = gps.speed();
  long alt = gps.altitude();
  int sats = gps.satellites();
  // generate ISO time string
  char isotime[32];
  sprintf(isotime, "%04u-%02u-%02uT%02u:%02u:%02u.%01uZ",
    (unsigned int)(utcdate % 100) + 2000, (unsigned int)(utcdate / 100) % 100, (unsigned int)(utcdate / 10000),
    (unsigned int)(utctime / 1000000), (unsigned int)(utctime % 1000000) / 10000, (unsigned int)(utctime % 10000) / 100, ((unsigned int)utctime % 100) / 10);

  // display GPS UTC time and coordinates
  Serial.print(isotime);
  Serial.print(" LAT:");
  Serial.print((float)lat / 1000000, 6);
  Serial.print(" LNG:");
  Serial.print((float)lng / 1000000, 6);
  Serial.print(" Speed:");
  Serial.print((int)speed * 1852 / 100000);
  Serial.print("km/h Alt:");
  Serial.print(alt / 100);
  Serial.print("m Sats:");
  Serial.println(sats);

  // arrange and send data in OsmAnd protocol
  // refer to https://www.traccar.org/osmand
  char data[128];
  sprintf(data, "&lat=%f&lon=%f&altitude=%f&speed=%f", (float)lat / 1000000, (float)lng / 1000000, (float)alt / 100, (float)speed / 1000);
  // send data
  client.print(String("GET /?id=") + TRACCAR_DEV_ID + "&timestamp=" + isotime + data + " HTTP/1.1\r\n" +
    "Connection: keep-alive\r\n\r\n");

  // waiting for server response while still decoding NMEA
  for (uint32_t t = millis(); !client.available() && millis() - t < 3000; ) {
    if (Serial1.available()) {
      gps.encode(Serial1.read());
    }
  }
  // output server response
  while(client.available()) {
    Serial.print((char)client.read());
  }

  // keep processed UTC timestamp
  lastutc = utctime;

  // turn off indicator LED
  digitalWrite(PIN_LED, LOW);
}