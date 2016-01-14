#include <SPI.h>
#include <FreematicsONE.h>
#include <TinyGPS.h>

COBDSPI one;
TinyGPS gps;

#define GPS_SERIAL_BAUDRATE 115200L

bool ready = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(500);
  one.begin();
  if (one.initGPS(GPS_SERIAL_BAUDRATE)) {
    Serial.println("GPS OK");
    Serial.println("Waiting for signal");
    delay(2000);
  }
}

void showGPS()
{
    // parsed GPS data is ready
    uint32_t time;
    uint32_t date;
    gps.get_datetime(&date, &time, 0);
    Serial.print("Date:");
    Serial.print(date);
    Serial.print(" Time:");
    Serial.print(time);

    int32_t lat, lon;
    gps.get_position(&lat, &lon, 0);
    Serial.print(" LAT:");
    Serial.print((float)lat / 100000, 5);
    Serial.print(" LNG:");
    Serial.print((float)lon / 100000, 5);
    
    Serial.print(" ALT:");
    Serial.print(gps.altitude() / 100);
    Serial.print("m");

    Serial.print(" Speed:");
    Serial.print(gps.speed() * 1852 / 100000);
    Serial.print("km/h");

    Serial.print(" SAT:");
    Serial.print(gps.satellites());

    Serial.println();
}

void loop() {
  // put your main code here, to run repeatedly:
  char buf[255];
  byte n = one.getGPSRawData(buf, sizeof(buf));
  if (n > 0) {
    bool updated = false;
    // need to skip heading ($GPS) and ending (>)
    for (byte m = 4; m < n - 1; m++) {
      if (!ready) Serial.write(buf[m]);
      if (gps.encode(buf[m])) {
        updated = true;
      }
    }
    if (updated) {
      ready = true;
      showGPS(); 
    }
  }
}
