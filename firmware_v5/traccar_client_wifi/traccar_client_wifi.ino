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
#include <TinyGPS.h>
#include <WiFi.h>
#include "driver/gpio.h"

#define PIN_LED 4
#define PIN_GPS_POWER 15
#define PIN_GPS_UART_RXD 32
#define PIN_GPS_UART_TXD 33

#define GPS_BAUDRATE 115200
#define WIFI_SSID "FREEMATICS"
#define WIFI_PASSWORD "862150909018"

#define TRACCAR_PORT 5055
#define TRACCAR_HOST "car.freematics.com"
#define TRACCAR_DEV_ID "SIMULATOR"

HardwareSerial Serial1(1);
TinyGPS gps;
WiFiClient client;

#define GPIO_OUTPUT_IO_0    15
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0))

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  delay(100);
  // USB serial
  Serial.begin(115200);
  Serial.println("TRACCAR CLIENT");

/*
  Serial.print("Connecting WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.println("OK");
*/

  digitalWrite(PIN_LED, LOW);

  // turn on GPS power
  //pinMode(PIN_GPS_POWER, OUTPUT);
  //digitalWrite(PIN_GPS_POWER, HIGH);

    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = (gpio_int_type_t)GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = (gpio_pulldown_t)0;
    //disable pull-up mode
    io_conf.pull_up_en = (gpio_pullup_t)0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)GPIO_OUTPUT_IO_0, 1);

  // GPS serial
  //Serial1.begin(GPS_BAUDRATE, SERIAL_8N1, PIN_GPS_UART_RXD, PIN_GPS_UART_TXD);
  Serial.println("Waiting for GPS signal...");
  delay(1000);
}

void loop()
{
  if (!Serial1.available()) {
    return;
  }

  char c = Serial1.read();
  // output NMEA
  Serial.print(c);

  if (!gps.encode(c)) {
    return;
  }

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

  // new GPS data available
  static unsigned long lastutc = 0;
  unsigned long utcdate, utctime;
  gps.get_datetime(&utcdate, &utctime, 0);
  if (utctime != lastutc) {
    digitalWrite(PIN_LED, HIGH);
    lastutc = utctime;
    long lat, lng;
    gps.get_datetime(&utcdate, &utctime, 0);
    gps.get_position(&lat, &lng, 0);
    long speed = gps.speed(); // km/h
    long alt = gps.altitude(); // m
    int sats = gps.satellites();
    Serial.print("UTC:");
    Serial.print(utctime);
    Serial.print(" LAT:");
    Serial.print((float)lat / 1000000, 6);
    Serial.print(" LNG:");
    Serial.print((float)lng / 1000000, 6);
    Serial.print(" Speed:");
    Serial.print((float)speed * 1852 / 100000, 1);
    Serial.print("km/h Alt:");
    Serial.print(alt / 100);
    Serial.print("m Sats:");
    Serial.println(sats);

    // generate ISO time string
    char isotime[256];
    sprintf(isotime, "%04u-%02u-%02u'T'%02u:%02u:%02u.%u'Z'",
      (unsigned int)(utcdate % 10000), (unsigned int)(utcdate / 10000) % 100, (unsigned int)(utcdate / 1000000),
      (unsigned int)(utctime / 1000000), (unsigned int)(utctime % 1000000) / 10000, (unsigned int)(utctime % 10000) / 100, (unsigned int)utctime % 100);
    Serial.println(isotime);

    // send data in OsmAnd protocol
    // refer to https://www.traccar.org/osmand
    client.print(String("GET /id=") + TRACCAR_DEV_ID +
      "&lat=" + (float)lat / 1000000 + "&lon=" + (float)lng / 1000000 +
      "&timestamp=" + isotime + "&altitude=" + alt + "&speed=" + speed +
       " HTTP/1.1\r\n" +
      "Host: " + TRACCAR_HOST + "\r\n" +
      "Connection: keep-alive\r\n\r\n");

    while(client.available()) {
      Serial.print(client.read());
    }

    digitalWrite(PIN_LED, LOW);
  }
}
