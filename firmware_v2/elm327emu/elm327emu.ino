#include <SoftwareSerial.h>

SoftwareSerial SerialBT(A2, A3);

void setup() {
  Serial.begin(38400);
  SerialBT.begin(38400);
}

void loop() {
  if (Serial.available())
    SerialBT.write(Serial.read());
  
  if (SerialBT.available())
    Serial.write(SerialBT.read());
}
