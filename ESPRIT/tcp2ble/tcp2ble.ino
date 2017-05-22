/*
  Freematics Esprit demo sketch

  This demo includes an HTTP server that is accessible
  via http://esp32.local URL thanks to mDNS responder.

  Instructions:
  - Update WiFi SSID and password as necessary.
  - Flash the sketch to the ESP32 board
  - Install host software:
    - For Linux, install Avahi (http://avahi.org/).
    - For Windows, install Bonjour (http://www.apple.com/support/bonjour/).
    - For Mac OSX and iOS support is built in through Bonjour already.
  - Point your browser to http://esp32.local, you should see a response.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiClient.h>
#include <Esprit.h>

#define WIFI_SSID "HOMEWIFI"
#define WIFI_PASSWORD "862150909018"

WiFiServer server(8000);

class MyGATTServer : public GATTServer
{
public:
    size_t onRequest(uint8_t* buffer, size_t len)
    {
        return sprintf((char*)buffer, "%u", millis());
    }
    void onReceive(uint8_t* buffer, size_t len)
    {
        for (size_t i = 0; i < len; i++) {
          char c = buffer[i];
          Serial.print(c);
          if (data.length() < 24) data += c;
        }
    }
    String data;
};

uint32_t lastTick = 0;
MyGATTServer ble;

void setup(void)
{
    Serial.begin(115200);

    // BLE
    if (ble.init("FREEMATICS")) {
      Serial.println("BLE initialized");
    }
    else {
      Serial.println("BLE failed");
    }
    
    // Connect to WiFi network
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(200);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Set up mDNS responder
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
}

void processSerial()
{
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
}

void loop(void)
{
    processSerial();
    
    // Check if a client has connected
    WiFiClient client = server.available();
    if (!client) {
        return;
    }
    Serial.println("");
    Serial.println("New client");

    // Wait for data from client to become available
    while(client.connected()){
      if (ble.data.length() > 0) {
        client.write(ble.data.c_str(), ble.data.length());
        ble.data = "";
      }
      if (client.available()) {
        String req = client.readStringUntil('\r');
        ble.send((uint8_t*)req.c_str(), req.length());
      }
      processSerial();
    }
    client.stop();
    Serial.println("Done with client");
}
