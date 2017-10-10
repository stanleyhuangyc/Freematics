/*
  A Freematics ESPRIT demo sketch

  This sketch demonstrates a BLE GATT server and an HTTP server running simultaneously
  HTTP server is accessible via http://esprit.local URL thanks to mDNS responder.
  IP address, BLE message and GPS data will be output on serial and OLED

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
#include "SH1106.h"
#include "images.h"
#include "config.h"

// TCP server at port 80 will respond to HTTP requests
WiFiServer server(80);
CNVS nvs;
LCD_SH1106 lcd;
TinyGPS gps;

const char waitsign[] = {'|', '/', '-', '\\'};

class MyGATTServer : public GATTServer
{
public:
    size_t onRequest(uint8_t* buffer, size_t len)
    {
        return sprintf((char*)buffer, "%u", millis());
    }
    void onReceive(uint8_t* buffer, size_t len)
    {
        lcd.setCursor(0, 3);
        lcd.setFontSize(FONT_SIZE_SMALL);
        if (len > 20) len = 20;
        Serial.print("BLE:");
        for (size_t i = 0; i < len; i++) {
            Serial.print((char)buffer[i]);
            lcd.write(buffer[i]);
        }
    }
};

uint32_t lastTick = 0;
MyGATTServer ble;

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

    // NVS
    nvs.init();
    size_t size = nvs.getFlashSize();
    Serial.print("Flash Size:");
    Serial.print(size >> 20);
    Serial.println("MB");
    lcd.print("Flash Size:");
    lcd.print(size >> 20);
    lcd.println("MB");

    int32_t restart_counter = 0;
    nvs.get("restart_counter", restart_counter);
    Serial.print("Reboots:");
    Serial.println(restart_counter);
    lcd.print("Reboots: ");
    lcd.print(restart_counter);
    restart_counter++;
    nvs.set("restart_counter", restart_counter);
    nvs.commit();
    delay(1000);
    lcd.clear();

    // BLE
    lcd.print("BLE");
    if (ble.init(BLE_DEVICE_NAME)) {
      Serial.println("BLE initialized");
      showTickCross(true);
    }
    else {
      Serial.println("BLE failed");
      showTickCross(false);
    }

 #if 0
    char *name = 0;
    lastTick = millis();
    while (!(name = ble.getDeviceName()) && millis() - lastTick < 5000) delay(200);
    lcd.println(name ? name : "No BLE device");
 #endif

    // Connect to WiFi network
    lcd.setCursor(64, 0);
    lcd.print("WIFI");
    lcd.setCursor(0, 2);
    lcd.print("SSID:");
    lcd.print(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);

    // Wait for connection
    byte count = 0;
    while (WiFi.status() != WL_CONNECTED) {
        lcd.setCursor(100, 0);
        lcd.print(waitsign[(count = (count + 1) % 4)]);
        delay(100);
        Serial.print(".");
    }
    lcd.setCursor(100, 0);
    showTickCross(true);
    lcd.setCursor(0, 2);
    lcd.print(WiFi.localIP());
    Serial.println("");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Set up mDNS responder:
    // - first argument is the domain name, in this example
    //   the fully-qualified domain name is "esp8266.local"
    // - second argument is the IP address to advertise
    //   we send our IP address on the WiFi network
    if (!MDNS.begin("esprit")) {
        Serial.println("Error setting up MDNS responder!");
        while(1) {
            delay(1000);
        }
    }
    Serial.println("mDNS responder started");

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
};

CGPS mygps;

void loop(void)
{
    // send serial input via BLE
    if (Serial.available()) {
        static uint8_t buffer[128];
        static uint8_t len = 0;
        char c = Serial.read();
        buffer[len++] = c;
        if (len >= sizeof(buffer) - 1 || millis() - lastTick > 100 || c == '\r' || c == '\n') {
            // send BLE data
            ble.send(buffer, len);
            len = 0;
            lastTick = millis();
        }
    }

    // read GPS NMEA data from Serial1 and parse it
    if (Serial2.available()) {
      char c = Serial2.read();
      //Serial.write(c);
      if (c == '\n') {
        lcd.setCursor(27, 4);
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.print('(');
        if (mygps.time == 0)
          lcd.print(millis() / 1000);
        else
          lcd.print(mygps.time);
        lcd.print(')');
      }
      if (mygps.push(c)) {
        lcd.setCursor(0, 6);
        lcd.setFontSize(FONT_SIZE_SMALL);
        lcd.print((float)mygps.lat / 1000000);
        lcd.print('/');
        lcd.println((float)mygps.lng / 1000000);
        lcd.print(mygps.speed);
        lcd.print("kph Alt:");
        lcd.print(mygps.alt / 100);
        lcd.print("m Sat:");
        lcd.print(mygps.sat);
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
