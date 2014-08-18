#include "config.h"
#include <SoftwareSerial.h>

SoftwareSerial SerialBT(A2, A3);

void setup()
{
  Serial.begin(38400);
  SerialBT.begin(STREAM_BAUDRATE);
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
    if (cmdlen < sizeof(cmd)) {
      char c = SerialBT.read();
      if (c == '\r' || c == '\n') {
        if (cmdlen > 0) {
          Serial.write(cmd, cmdlen);
          Serial.write('\r');
          cmdlen = 0;
        }
      } else {
        cmd[cmdlen++] = SerialBT.read();
      }
    }
  }
}
