#include "config.h"
#include <SoftwareSerial.h>

SoftwareSerial SerialBT(A2, A3);

void setup() {
  Serial.begin(38400);
  SerialBT.begin(STREAM_BAUDRATE);
}

void loop() {
  if (Serial.available())
    SerialBT.write(Serial.read());
  
  if (SerialBT.available())
    Serial.write(SerialBT.read());
}
