/*************************************************************************
* Testing sketch for Freematics OBD-II UART Adapter V1/V2/V2.1
* Performs AT command-set test
* Reads and prints several OBD-II PIDs value
* Reads and prints motion sensor data if available
* Distributed under BSD
* Visit https://freematics.com/products for more product information
* Written by Stanley Huang <stanley@freematics.com.au>
*************************************************************************/

#include <OBD2UART.h>

// On Arduino Leonardo, Micro, MEGA or DUE, hardware serial can be used for output as the adapter occupies Serial1
// On Arduino UNO and those have no Serial1, we use software serial for output as the adapter uses Serial
#ifdef ARDUINO_AVR_UNO
#include <SoftwareSerial.h>
SoftwareSerial mySerial(A2, A3);
#else
#define mySerial Serial
#endif

#if defined(ESP32) && !defined(Serial1)
HardwareSerial Serial1(1);
#endif

COBD obd;
bool hasMEMS;

void testATcommands()
{
    static const char cmds[][6] = {"ATZ\r", "ATI\r", "ATH0\r", "ATRV\r", "0100\r", "010C\r", "0902\r"};
    char buf[128];

    for (byte i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        const char *cmd = cmds[i];
        mySerial.print("Sending ");
        mySerial.println(cmd);
        if (obd.sendCommand(cmd, buf, sizeof(buf))) {
            char *p = strstr(buf, cmd);
            if (p)
                p += strlen(cmd);
            else
                p = buf;
            while (*p == '\r') p++;
            while (*p) {
                mySerial.write(*p);
                if (*p == '\r' && *(p + 1) != '\r')
                    mySerial.write('\n');
                p++;
            }
            mySerial.println();
        } else {
            mySerial.println("Timeout");
        }
        delay(1000);
    }
    mySerial.println();
}

void readPIDSingle()
{
    int value;
    mySerial.print('[');
    mySerial.print(millis());
    mySerial.print(']');
    mySerial.print("RPM=");
    if (obd.readPID(PID_RPM, value)) {
      mySerial.print(value);
    }
    mySerial.println();
}

void readPIDMultiple()
{
    static const byte pids[] = {PID_SPEED, PID_ENGINE_LOAD, PID_THROTTLE, PID_COOLANT_TEMP};
    int values[sizeof(pids)];
    if (obd.readPID(pids, sizeof(pids), values) == sizeof(pids)) {
      mySerial.print('[');
      mySerial.print(millis());
      mySerial.print(']');
      for (byte i = 0; i < sizeof(pids) ; i++) {
        mySerial.print((int)pids[i] | 0x100, HEX);
        mySerial.print('=');
        mySerial.print(values[i]);
        mySerial.print(' ');
       }
       mySerial.println();
    }
}

void readBatteryVoltage()
{
  mySerial.print('[');
  mySerial.print(millis());
  mySerial.print(']');
  mySerial.print("Battery:");
  mySerial.print(obd.getVoltage(), 1);
  mySerial.println('V');
}

void readMEMS()
{
  int16_t acc[3] = {0};
  int16_t gyro[3] = {0};
  int16_t mag[3] = {0};
  int16_t temp = 0;

  if (!obd.memsRead(acc, gyro, mag, &temp)) return;

  mySerial.print('[');
  mySerial.print(millis());
  mySerial.print(']');

  mySerial.print("ACC:");
  mySerial.print(acc[0]);
  mySerial.print('/');
  mySerial.print(acc[1]);
  mySerial.print('/');
  mySerial.print(acc[2]);

  mySerial.print(" GYRO:");
  mySerial.print(gyro[0]);
  mySerial.print('/');
  mySerial.print(gyro[1]);
  mySerial.print('/');
  mySerial.print(gyro[2]);

  mySerial.print(" MAG:");
  mySerial.print(mag[0]);
  mySerial.print('/');
  mySerial.print(mag[1]);
  mySerial.print('/');
  mySerial.print(mag[2]);

  mySerial.print(" TEMP:");
  mySerial.print((float)temp / 10, 1);
  mySerial.println("C");
}

void setup()
{
  mySerial.begin(115200);
  while (!mySerial);
  
  for (;;) {
    delay(1000);
    byte version = obd.begin();
    mySerial.print("Freematics OBD-II Adapter ");
    if (version > 0) {
      mySerial.println("detected");
      mySerial.print("OBD firmware version ");
      mySerial.print(version / 10);
      mySerial.print('.');
      mySerial.println(version % 10);
      break;
    } else {
      mySerial.println("not detected");
    }
  }

  // send some commands for testing and show response for debugging purpose
  testATcommands();

  hasMEMS = obd.memsInit();
  mySerial.print("MEMS:");
  mySerial.println(hasMEMS ? "Yes" : "No");
  
  // initialize OBD-II adapter
  do {
    mySerial.println("Connecting...");
  } while (!obd.init());
  mySerial.println("OBD connected!");

  char buf[64];
  if (obd.getVIN(buf, sizeof(buf))) {
      mySerial.print("VIN:");
      mySerial.println(buf);
  }
  
  uint16_t codes[6];
  byte dtcCount = obd.readDTC(codes, 6);
  if (dtcCount == 0) {
    mySerial.println("No DTC"); 
  } else {
    mySerial.print(dtcCount); 
    mySerial.print(" DTC:");
    for (byte n = 0; n < dtcCount; n++) {
      mySerial.print(' ');
      mySerial.print(codes[n], HEX);
    }
    mySerial.println();
  }
  delay(5000);
}

void loop()
{
  readPIDSingle();
  readPIDMultiple();
  readBatteryVoltage();
  if (hasMEMS) {
    readMEMS();
  }
}

