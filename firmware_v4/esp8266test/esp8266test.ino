/*************************************************************************
* This sketch demonstrates WIFI communication via the ESP8266 WIFI xBee
* plugged in Freematics ONE's xBee socket
* Distributed under BSD License
* Visit http://freematics.com/products/freematics-one for more information
* Developed by Stanley Huang <stanleyhuangyc@gmail.com>
*************************************************************************/

#include <FreematicsONE.h>

#define WIFI_SSID "WIFI_SSID"
#define WIFI_PASSWORD "WIFI_PASSWORD"
#define SERVER_URL "hub.freematics.com"
#define SERVER_PORT 80

COBDSPI one;

char buffer[384];

bool sendCommand(const char* cmd, int timeout = 1000, const char* expected = "OK\r\n")
{
  if (cmd) {
    one.xbWrite(cmd);
    delay(200);
  }
  return one.xbReceive(buffer, sizeof(buffer), timeout, &expected, 1) == 1;
}

void setup() {
  byte ret;
  
  one.begin();
  Serial.begin(115200);
  delay(500);
  // set xBee module baudrate
  Serial.print("Setting xBee baudrate...");
  if (one.xbBegin(9600) ) {
    Serial.println("OK");
  } else {
    for (;;); 
  }
    
  bool success = false;
  Serial.print("Checking ESP8266...");
  for (byte n = 0; !(success = sendCommand("AT\r\n")) && n < 10; n++) {
    delay(100);
    Serial.print('.');
  }
  if (success) {
    Serial.println("OK"); 
  } else {
    Serial.println("failed"); 
  }
  sendCommand("ATE0\r\n");
  
/*
  Serial.print("Resetting ESP8266...");
  if (sendCommand("AT+RST\r\n", 5000, "Ready")) {
    Serial.println("OK"); 
  } else {
    Serial.println("dead"); 
  }
*/

  sendCommand("AT+CWMODE=1\r\n", 1000, "OK");
  sendCommand("AT+CIPMUX=0\r\n");
  
  Serial.print("Connecting AP (SSID:");
  Serial.print(WIFI_SSID);
  Serial.print(")...");
  sprintf_P(buffer, PSTR("AT+CWJAP=\"%s\",\"%s\"\r\n"), WIFI_SSID, WIFI_PASSWORD);
  ret = sendCommand(buffer, 10000, "OK");
  if (ret == 1) {
    // get IP address
    if (sendCommand("AT+CIFSR\r\n", 1000, "OK")) {
      char *p = strchr(buffer, '\r');
      if (p) *p = 0;
      Serial.println(buffer);
    } else {
      Serial.println("N/A"); 
    }
  } else {
    Serial.println("failed"); 
    for (;;);
  }
  
  Serial.print("Connecting to ");
  Serial.print(SERVER_URL);
  Serial.print("...");
  sprintf_P(buffer, PSTR("AT+CIPSTART=\"TCP\",\"%s\",%d\r\n"), SERVER_URL, SERVER_PORT);
  if (sendCommand(buffer, 10000, "Linked")) {
    Serial.println("OK");
  } else {
    Serial.println("failed");
  }
}

void loop() {
  static unsigned long count = 0;
  char payload[256];
  unsigned int len = sprintf_P(payload, PSTR("GET /test HTTP/1.1\r\nUser-Agent:ONE\r\nConnection: keep-alive\r\nHost: %s\r\n\r\n"), SERVER_URL);
  sprintf_P(buffer, PSTR("AT+CIPSEND=%u\r\n"), len);
  if (sendCommand(buffer, 1000, ">")) {
    Serial.print('[');
    Serial.print(++count);
    Serial.print(']');
    Serial.print("Sending ");
    Serial.print(len);
    Serial.print(" bytes...");
    if (sendCommand(payload, 10000, "+IPD,") == 1) {
      Serial.println("OK");
      char *p = strstr(buffer, "+IPD,");
      if (!p) {
         Serial.println("Data error");
         return;
      }
      len = atoi(p + 5);
      Serial.print("Received ");
      Serial.print(len);
      Serial.println(" bytes");
      p = strchr(p, ':');
      if (p) {
        strcpy(payload, p + 1);
      } else {
        payload[0] = 0; 
      }
      if (sendCommand(0, 3000, "\r\nOK")) {
        strcat(payload, buffer);
      }
      // display response
      Serial.println(payload); 
    } else {
     Serial.println(buffer); 
     count = 0;
    }
  } else {
     Serial.println(buffer);
     count = 0;
  }

  delay(3000);
}
