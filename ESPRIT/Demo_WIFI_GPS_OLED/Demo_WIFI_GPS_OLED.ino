/*
  A Freematics ESPRIT demo sketch

  This sketch demonstrates an HTTP server accessible via http://esprit.local
  GPS data will be output on serial and OLED

  Instructions:
  - Connect GPS receiver to Serial1 header on ESPRIT board
  - Connect SH1106 OLED to I2C header on ESPRIT board
  - Update WiFi SSID and password as necessary (in config.h)
  - Import ESPRIT and TinyGPS library (once)
  - Compile and flash the sketch to the ESPRIT board

  Developed by Stanley Huang https://www.facebook.com/stanleyhuangyc
  Distributed under BSD license
  Visit http://freematics.com/products/freematics-esprit for product information

 */

#include <Wire.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiClient.h>
#include <TinyGPS.h>
#include <Esprit.h>
#include <SH1106.h>
#include "images.h"
#include "config.h"

#define WIFI_SSID "Freematics Esprit"

// TCP server at port 80 will respond to HTTP requests
WiFiServer server(80);
CNVS nvs;
LCD_SH1106 lcd;

const char waitsign[] = {'|', '/', '-', '\\'};

uint32_t lastTick = 0;

void showTickCross(bool yes)
{
    lcd.draw(yes ? tick : cross, 16, 16);
}

void setup(void)
{
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 32, 33);
    Serial.println("Freematics ESPRIT Demo");
    lcd.begin();
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.println("ESPRIT DEMO");


    Serial.print("CPU:");
    Serial.print(ESP.getCpuFreqMHz());
    Serial.println("MHz");
    lcd.print("CPU:");
    lcd.print(ESP.getCpuFreqMHz());
    lcd.println("MHz");

    nvs.init();
    size_t size = nvs.getFlashSize();
    Serial.print("Flash Size:");
    Serial.print(size >> 20);
    Serial.println("MB");
    lcd.print("Flash:");
    lcd.print(size >> 20);
    lcd.println("MB");

    int32_t restart_counter = 0;
    nvs.get("restart_counter", restart_counter);
    Serial.print("Reboots:");
    Serial.println(restart_counter);
    lcd.print("Reboots:");
    lcd.print(restart_counter);
    restart_counter++;
    nvs.set("restart_counter", restart_counter);
    nvs.commit();
    delay(3000);
    lcd.clear();

    // Connect to WiFi network
    lcd.print("WIFI");
    if (WiFi.softAP(WIFI_SSID)) {
      Serial.println("WIFI AP Started");
      showTickCross(true);
    }
    else {
      Serial.println("WIFI AP Error");
      showTickCross(false);
    }
    lcd.setCursor(0, 2);
    lcd.print(WiFi.softAPIP());
    Serial.println("");
    Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP());

    // Set up mDNS responder:
    // - first argument is the domain name, in this example
    //   the fully-qualified domain name is "esp8266.local"
    // - second argument is the IP address to advertise
    //   we send our IP address on the WiFi network
    if (MDNS.begin("esprit")) {
        Serial.println("Error setting up MDNS responder!");
    } else {
        Serial.println("mDNS responder started");
    }

    // Start TCP (HTTP) server
    server.begin();
    Serial.println("TCP server started");

    // Add service to MDNS-SD
    MDNS.addService("http", "tcp", 80);

    lcd.setCursor(0, 4);
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.print("GPS");
    if (!Serial2.available()) showTickCross(false);
}

class CGPS
{
public:
  CGPS():time(0),lat(0),lng(0) {}
  bool push(char c)
  {
    if (!gps.encode(c)) return false;
    gps.get_datetime((unsigned long*)&date, (unsigned long*)&time, 0);
    gps.get_position((long*)&lat, (long*)&lng, 0);
    sat = gps.satellites();
    speed = gps.speed() * 1852 / 100000; /* km/h */
    alt = gps.altitude();
    heading = gps.course() / 100;
    return true;
  }
  uint32_t date;
  uint32_t time;
  int32_t lat;
  int32_t lng;
  int16_t alt;
  uint8_t speed;
  uint8_t sat;
  int16_t heading;
private:
  TinyGPS gps;
};

CGPS gps;

void loop(void)
{
    // read GPS NMEA data from Serial1 and parse it
    if (Serial2.available()) {
      char c = Serial2.read();
      //Serial.write(c);
      if (c == '\n') {
        lcd.setCursor(27, 4);
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.print('(');
        if (gps.time == 0)
          lcd.print(millis() / 1000);
        else
          lcd.print(gps.time);
        lcd.print(')');
      }
      if (gps.push(c)) {
        lcd.setCursor(0, 6);
        lcd.setFontSize(FONT_SIZE_SMALL);
        lcd.print((float)gps.lat / 1000000);
        lcd.print('/');
        lcd.println((float)gps.lng / 1000000);
        lcd.print(gps.speed);
        lcd.print("kph Alt:");
        lcd.print(gps.alt / 100);
        lcd.print("m Sat:");
        lcd.print(gps.sat);
        lcd.print(' ');
      }
    }

    // Check if a TCP client has connected
    WiFiClient client = server.available();
    if (!client) {
        delay(1);
        return;
    }
    Serial.println("");
    Serial.println("New client");

    // Wait for data from client to become available
    while(client.connected() && !client.available()){
        delay(1);
    }

    // Read the first line of HTTP request
    String req = client.readStringUntil('\r');

    // First line of HTTP request looks like "GET /path HTTP/1.1"
    // Retrieve the "/path" part by finding the spaces
    int addr_start = req.indexOf(' ');
    int addr_end = req.indexOf(' ', addr_start + 1);
    if (addr_start == -1 || addr_end == -1) {
        Serial.print("Invalid request: ");
        Serial.println(req);
        return;
    }
    req = req.substring(addr_start + 1, addr_end);
    Serial.print("Request: ");
    Serial.println(req);
    client.flush();

    String s;
    if (req == "/")
    {
        IPAddress ip = WiFi.localIP();
        String ipStr = String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);
        s = "HTTP/1.1 200 OK\r\nContent-Type: text/json\r\n\r\n{\"tick\":";
        s += millis();
        s += ",\"IP\":\"";
        s += ipStr;
        s += "\"}";
        Serial.println("Sending 200");
    }
    else
    {
        s = "HTTP/1.1 404 Not Found\r\n\r\n";
        Serial.println("Sending 404");
    }
    client.print(s);

    Serial.println("Done with client");
}
