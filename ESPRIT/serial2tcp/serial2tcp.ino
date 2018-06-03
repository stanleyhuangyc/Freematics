
#include <WiFi.h>
#include <ESPmDNS.h>

#define SSID "SSID"
#define PASSWORD "PASSWORD"

WiFiServer server(8000);

void setup()
{
    Serial.begin(115200);
    pinMode(4, OUTPUT);      // set the LED pin mode
    digitalWrite(4, HIGH);
    // We start by connecting to a WiFi network
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) delay(500);
    // LED goes off when WiFi network is connected
    digitalWrite(4, LOW);
    // start server
    server.begin();
    // make accessible by local host name (esprit.local)
    MDNS.begin("esprit");
    // Add service to MDNS-SD
    MDNS.addService("serial", "tcp", 8000);
}

void loop()
{
  WiFiClient client = server.available();   // listen for incoming clients
  if (client) {                             // if you get a client,
    while (client.connected()) {            // loop while the client's connected
      if (Serial.available()) {
        client.write(Serial.read());
      }
      if (client.available()) {             // if there's bytes to read from the client,
        Serial.write(client.read());
      }
    }
    // close the connection:
    client.stop();
  }
}
