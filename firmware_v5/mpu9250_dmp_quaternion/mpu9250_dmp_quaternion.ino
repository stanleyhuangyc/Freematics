#include <FreematicsONE.h>

MPU9250_DMP imu;

void setup()
{
  delay(1000);
  Serial.begin(115200);
  Serial.println("MPU-9250 DMP Quaternion Test");
  if (!imu.memsInit(true, 10))
  {
    Serial.println("Unable to communicate with MPU-9250");
    Serial.println("Check connections, and try again.");
    for (;;);
  }
  Serial.println("MPU-9250 OK");
}

void loop()
{
  float acc[3];
  ORIENTATION ori;
  if (imu.memsRead(acc, 0, 0, 0, &ori)) {
    Serial.print("Accelerometer: X=");
    Serial.print(acc[0]);
    Serial.print("g Y=");
    Serial.print(acc[1]);
    Serial.print("g Z=");
    Serial.print(acc[2]);
    Serial.println("g");
    Serial.print("Orientation: ");
    Serial.print(imu.yaw);
    Serial.print(' ');
    Serial.print(imu.pitch);
    Serial.print(' ');
    Serial.println(imu.roll);
  }
  delay(50);
  return;
}
