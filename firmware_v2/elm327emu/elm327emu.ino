#include "config.h"
#include <SoftwareSerial.h>

SoftwareSerial SerialBT(A2, A3);

void setup()
{
  Serial.begin(38400);
  SerialBT.begin(STREAM_BAUDRATE);
  Serial.write("ATZ\r");
  delay(500);
  while (Serial.available()) Serial.read();
  Serial.write("ATE0\r");
  delay(100);
  while (Serial.available()) Serial.read();
}

char cmd[32];
byte cmdlen = 0;
byte sent = 0;

void loop()
{
  if (Serial.available()) {
    char c = Serial.read();
    SerialBT.write(c);
    if (++sent == 20 || c == '\r') {
      delay(20);
      sent = 0;
    } else {
      delay(5);
    }
  }

  if (SerialBT.available()) {
    char c = SerialBT.read();
    if (c == '\r' || c == '\n') {
      if (cmdlen > 0) {
        Serial.write(cmd, cmdlen);
        Serial.write('\r');
        if (cmdlen == 3 && !memcmp(cmd, "ATZ", 3)) {
          delay(1000);
          while (Serial.available()) Serial.read();
          Serial.println("ELM327 v1.4b");
        }
        cmdlen = 0;
      }
    } else if (cmdlen < sizeof(cmd)) {
      cmd[cmdlen++] = c;
    }
  }
}
