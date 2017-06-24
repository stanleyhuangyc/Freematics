/******************************************************************************
* Reference sketch for a vehicle telematics data feed for Freematics Hub
* Works with Freematics ONE+
* Developed by Stanley Huang https://www.facebook.com/stanleyhuangyc
* Distributed under BSD license
* Visit http://freematics.com/hub/api for Freematics Hub API reference
* Visit http://freematics.com/products/freematics-one for hardware information
* To obtain your Freematics Hub server key, contact support@freematics.com.au
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
******************************************************************************/

#include <FreematicsPlus.h>
#include "config.h"

// logger states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_CONNECTED 0x20
#define STATE_ALL_GOOD 0x40

#define PIN_LED 4

#if MEMS_TYPE
byte accCount = 0; // count of accelerometer readings
long accSum[3] = {0}; // sum of accelerometer data
int accCal[3] = {0}; // calibrated reference accelerometer data
byte deviceTemp = 0; // device temperature
#endif
int lastSpeed = 0;
uint32_t lastSpeedTime = 0;
uint32_t distance = 0;
uint32_t startTick = 0;
uint32_t lastSentTime = 0;

class CTeleLogger :
#if NET_DEVICE == NET_SERIAL
public CTeleClientSerialUSB
#elif NET_DEVICE == NET_BLE
public CTeleClientBLE
#elif NET_DEVICE == NET_WIFI
public CTeleClientWIFI
#elif NET_DEVICE == NET_SIM800
public CTeleClientSIM800
#elif NET_DEVICE == NET_SIM5360
public CTeleClientSIM5360
#else
public CTeleClient
#endif
#if ENABLE_OBD
,public COBDSPI
#else
,public CFreematicsESP32
#endif
#if MEMS_TYPE == MEMS_MPU6050
,public CMPU6050
#elif MEMS_TYPE == MEMS_MPU9250
,public CMPU9250
#endif
{
public:
  CTeleLogger():m_state(0) {}
  bool setup()
  {
    clearState(STATE_ALL_GOOD);

#if MEMS_TYPE
    if (!checkState(STATE_MEMS_READY)) {
      Serial.print("MEMS...");
      if (memsInit()) {
        setState(STATE_MEMS_READY);
        Serial.println("OK");
        bleSend("MEMS OK");
      } else {
        Serial.println("NO");
      }
    }
#endif

#if ENABLE_OBD
    // initialize OBD communication
    if (!checkState(STATE_OBD_READY)) {
      Serial.print("OBD...");
      if (!init() && !init()) {
        Serial.println("NO");
        return false;
      }
      Serial.println("OK");
      bleSend("OBD OK");
      setState(STATE_OBD_READY);
    }
#endif

    // initialize network module
    if (!checkState(STATE_NET_READY)) {
      Serial.print(netDeviceName());
      Serial.print("...");
      if (netBegin()) {
        Serial.println("OK");
        bleSend("NET OK");
        setState(STATE_NET_READY);
      } else {
        Serial.println("NO");
        return false;
      }
    }

#if NET_DEVICE == NET_WIFI
    Serial.print("WIFI(SSID:");
    Serial.print(WIFI_SSID);
    Serial.print(")...");
    if (netSetup(WIFI_SSID, WIFI_PASSWORD)) {
      bleSend("WIFI OK");
      setState(STATE_NET_READY | STATE_CONNECTED);
    } else {
      Serial.println("NO");
      return false;
    }
#elif NET_DEVICE == NET_SIM800 || NET_DEVICE == NET_SIM5360
    Serial.print("CELL(APN:");
    Serial.print(CELLULAR_APN);
    Serial.print(")");
    if (netSetup(CELLULAR_APN)) {
      bleSend("CELL OK");
      String op = getOperatorName();
      if (op.length()) {
        Serial.println(op);
        bleSend(op);
      } else {
        Serial.println("OK");
      }
      setState(STATE_CONNECTED);
    } else {
      Serial.println("NO");
      return false;
    }
#endif

#if USE_GPS
    // start serial communication with GPS receiver
    if (!checkState(STATE_GPS_READY)) {
      Serial.print("GPS...");
      if (gpsInit(GPS_SERIAL_BAUDRATE)) {
        setState(STATE_GPS_READY);
        Serial.print("OK(");
        Serial.print(internalGPS() ? "internal" : "external");
        Serial.println(')');
        bleSend("GPS OK");
      } else {
        Serial.println("NO");
      }
    }
#endif

#if NET_DEVICE == WIFI || NET_DEVICE == NET_SIM800 || NET_DEVICE == NET_SIM5360
    Serial.print("IP...");
    String ip = getIP();
    if (ip.length()) {
      Serial.println(ip);
      bleSend(ip);
    } else {
      Serial.println("NO");
    }
    Serial.print("CSQ...");
    int csq = getSignal();
    Serial.println(csq);
#endif

    startTick = millis();
    txCount = 0;

    if (!login()) {
      return false;
    }

    cache.init(STORAGE_SIZE);

#if NET_DEVICE == NET_SIM800 || NET_DEVICE == NET_SIM5360
    // log signal level
    if (csq) cache.log(PID_CSQ, csq);
#endif

    setState(STATE_ALL_GOOD);

#if MEMS_TYPE
    calibrateMEMS(CALIBRATION_TIME);
#endif
    return true;
  }
  void loop()
  {
    cache.timestamp(millis());

#if ENABLE_OBD
    // process OBD data if connected
    if (checkState(STATE_OBD_READY)) {
      processOBD();
    }
#endif

#if MEMS_TYPE
    // process MEMS data if available
    if (checkState(STATE_MEMS_READY)) {
        processMEMS();
        if ((txCount % 100) == 1) {
          cache.log(PID_DEVICE_TEMP, deviceTemp);
        }
    }
#endif

#if USE_GPS
    // process GPS data if connected
    if (checkState(STATE_GPS_READY)) {
      processGPS();
    }
#endif

#if ENABLE_OBD
    // read and log car battery voltage , data in 0.01v
    {
      int v = getVoltage() * 100;
      cache.log(PID_BATTERY_VOLTAGE, v);
    }
#endif

    if (millis() - lastSentTime >= DATA_SENDING_INTERVAL) {
      digitalWrite(PIN_LED, HIGH);
      Serial.print('[');
      Serial.print(txCount);
      Serial.print(']');
      // transmit data
      int bytesSent = transmit(cache.getBuffer(), cache.getBytes(), false);
      if (bytesSent > 0) {
        // output some stats
        char buf[16];
        sprintf(buf, "%uB sent", bytesSent);
        Serial.println(buf);
        bleSend(buf);
        cache.purge();
      } else {
        Serial.println("Unsent");
      }
      digitalWrite(PIN_LED, LOW);

      if (getConnErrors() >= MAX_CONN_ERRORS_RECONNECT) {
        netClose();
        netOpen(SERVER_URL, SERVER_PORT);
      }
      lastSentTime = millis();
    }

#if MEMS_TYPE
    if (deviceTemp >= COOLING_DOWN_TEMP && deviceTemp < 100) {
      // device too hot, cool down
      Serial.print("Cooling (");
      Serial.print(deviceTemp);
      Serial.println("C)");
      sleep(10000);
    }
#endif
  }
  bool login()
  {
    char payload[128];
    int bytes = 0;
#if ENABLE_OBD
    // load DTC
    uint16_t dtc[6];
    byte dtcCount = readDTC(dtc, sizeof(dtc) / sizeof(dtc[0]));
    if (dtcCount > 0) {
      Serial.print("#DTC:");
      Serial.println(dtcCount);
      bytes += sprintf(payload + bytes, ",DTC=", dtcCount);
      for (byte i = 0; i < dtcCount; i++) {
        bytes += sprintf(payload + bytes, "%X;", dtc[i]);
      }
      bytes--;
    }
    bytes += sprintf(payload + bytes, ",VIN=");
    if (getVIN(payload + bytes, sizeof(payload) - bytes)) {
      Serial.print("#VIN:");
      Serial.println(payload + bytes);
    } else {
      sprintf(payload + bytes, "DEFAULT_VIN");
    }
#else
    strcpy(payload, ",VIN=DEFAULT_VIN");
#endif

    // connect to telematics server
    for (byte attempts = 0; attempts < 3; attempts++) {
      Serial.print("#LOGIN...");
      if (netOpen(SERVER_URL, SERVER_PORT)) {
        Serial.println("OK");
      } else {
        Serial.println("NO");
        continue;
      }
      // login Freematics Hub
      if (!notifyServer(EVENT_LOGIN, SERVER_KEY, payload)) {
        netClose();
        continue;
      }

      Serial.print("#SERVER:");
      Serial.println(serverName());

      Serial.print("#FEED ID:");
      Serial.println(feedid);

      return true;
    }
    return false;
  }
  void standby()
  {
      if (checkState(STATE_NET_READY)) {
        notifyServer(EVENT_LOGOUT, SERVER_KEY, 0);
        netClose();
        Serial.print("#Network:");
        netEnd(); // turn off network module power (if supported)
        Serial.println("OFF");
        clearState(STATE_NET_READY);
      }
#if ENABLE_OBD
      if (errors > MAX_OBD_ERRORS) {
        // inaccessible OBD treated as end of trip
        feedid = 0;
      }
#endif
#if USE_GPS
      if (checkState(STATE_GPS_READY)) {
        Serial.print("#GPS:");
        gpsInit(0); // turn off GPS power
        Serial.println("OFF");
      }
#endif
      clearState(STATE_OBD_READY | STATE_GPS_READY | STATE_NET_READY | STATE_CONNECTED);
      Serial.println("Standby");
#if MEMS_TYPE
      calibrateMEMS(3000);
      if (checkState(STATE_MEMS_READY)) {
        enterLowPowerMode();
        for (;;) {
          // calculate relative movement
          unsigned long motion = 0;
          for (byte i = 0; i < 3; i++) {
            long n = accSum[i] / accCount - accCal[i];
            motion += n * n;
          }
          char buf[16];
          sprintf(buf, "M:%u", motion);
          Serial.println(buf);
          bleSend(buf);
          // check movement
          if (motion >= WAKEUP_MOTION_THRESHOLD) {
            leaveLowPowerMode();
            break;
          }
          // measure acceleration
          clearMEMS();
          for (int i = 0; i < 100; i++) {
            readMEMS();
            delay(10);
          }
        }
      }
#else
      do {
        //enterLowPowerMode();
        sleepSec(10);
        //leaveLowPowerMode();
      } while (!init());
#endif
      Serial.println("Wakeup");
  }
  bool checkState(byte flags) { return (m_state & flags) == flags; }
  void setState(byte flags) { m_state |= flags; }
  void clearState(byte flags) { m_state &= ~flags; }
private:
#if ENABLE_OBD
  void processOBD()
  {
      int speed = readSpeed();
      if (speed == -1) {
        return;
      }
      cache.log(0x100 | PID_SPEED, speed);
      cache.log(PID_TRIP_DISTANCE, distance);
      // poll more PIDs
      byte pids[]= {0, PID_RPM, PID_ENGINE_LOAD, PID_THROTTLE};
      const byte pidTier2[] = {PID_INTAKE_TEMP, PID_COOLANT_TEMP};
      static byte index2 = 0;
        int values[sizeof(pids)] = {0};
        // read multiple OBD-II PIDs, tier2 PIDs are less frequently read
        pids[0] = pidTier2[index2 = (index2 + 1) % sizeof(pidTier2)];
        byte results = readPID(pids, sizeof(pids), values);
        if (results == sizeof(pids)) {
          for (byte n = 0; n < sizeof(pids); n++) {
            cache.log(0x100 | pids[n], values[n]);
          }
        }
    }
    int readSpeed()
    {
        int value;
        if (readPID(PID_SPEED, value)) {
           uint32_t t = millis();
           distance += (value + lastSpeed) * (t - lastSpeedTime) / 3600 / 2;
           lastSpeedTime = t;
           lastSpeed = value;
           return value;
        } else {
          return -1;
        }
    }
#endif
#if MEMS_TYPE
    void processMEMS()
    {
         // log the loaded MEMS data
        if (accCount) {
          cache.log(PID_ACC, accSum[0] / accCount, accSum[1] / accCount, accSum[2] / accCount);
          clearMEMS();
        }
    }
#endif
    void processGPS()
    {
        static uint16_t lastUTC = 0;
        static uint8_t lastGPSDay = 0;
        GPS_DATA gd = {0};
        // read parsed GPS data
        if (gpsGetData(&gd)) {
            if (gd.date && lastUTC != (uint16_t)gd.time) {
              byte day = gd.date / 10000;
              cache.log(PID_GPS_TIME, gd.time);
              if (lastGPSDay != day) {
                cache.log(PID_GPS_DATE, gd.date);
                lastGPSDay = day;
              }
              cache.logCoordinate(PID_GPS_LATITUDE, gd.lat);
              cache.logCoordinate(PID_GPS_LONGITUDE, gd.lng);
              cache.log(PID_GPS_ALTITUDE, gd.alt);
              cache.log(PID_GPS_SPEED, gd.speed);
              cache.log(PID_GPS_SAT_COUNT, gd.sat);
              lastUTC = (uint16_t)gd.time;
              Serial.print("#UTC:");
              Serial.print(gd.time);
              Serial.print(" SAT:");
              Serial.println(gd.sat);
            }
        }
    }
#if MEMS_TYPE
    void calibrateMEMS(unsigned int duration)
    {
        // MEMS data collected while sleeping
        clearMEMS();
        if (duration > 0) {
          for (int i = 0; i < 100; i++) {
            readMEMS();
            delay(10);
          }
        } else {
          readMEMS();
        }
        // store accelerometer reference data
        if (accCount) {
          accCal[0] = accSum[0] / accCount;
          accCal[1] = accSum[1] / accCount;
          accCal[2] = accSum[2] / accCount;
        }
    }
    void clearMEMS()
    {
      accCount = 0;
      accSum[0] = 0;
      accSum[1] = 0;
      accSum[2] = 0;
    }
    void readMEMS()
    {
        // load accelerometer and temperature data
        int16_t acc[3] = {0};
        int16_t temp; // device temperature (in 0.1 celcius degree)
        memsRead(acc, 0, 0, &temp);
        if (accCount >= 128) {
          accSum[0] >>= 1;
          accSum[1] >>= 1;
          accSum[2] >>= 1;
          accCount >>= 1;
        }
        accSum[0] += acc[0];
        accSum[1] += acc[1];
        accSum[2] += acc[2];
        accCount++;
        deviceTemp = temp / 10;
    }
#endif
    void dataIdleLoop()
    {
      // do tasks while waiting for data on SPI
#if MEMS_TYPE
      if (checkState(STATE_MEMS_READY)) {
        readMEMS();
      }
#endif
    }
    CStorageRAM cache;
    byte m_state;
};

CTeleLogger logger;

void setup()
{
    // initialize USB serial
    Serial.begin(115200);
#if ENABLE_OBD
    Serial.println("Freematics ONE+ (ESP32)");
#else
    Serial.println("Freematics Esprit (ESP32)");
#endif
    // init LED pin
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    // perform initializations
    logger.begin();
#if ENABLE_BLE
    logger.bleBegin(BLE_DEVICE_NAME);
#endif
    delay(1000);
    digitalWrite(PIN_LED, LOW);
    logger.setup();
}

void loop()
{
    // error handling
    if (!logger.checkState(STATE_ALL_GOOD)
#if ENABLE_OBD
      || (logger.errors > MAX_OBD_ERRORS && !logger.init())
      || logger.getConnErrors() >= MAX_CONN_ERRORS
#endif
    ) {
      do {
        logger.standby();
      } while (!logger.setup());
    }
#if DATASET_INTERVAL
    uint32_t t = millis();
#endif
    // collect data
    logger.loop();

#if DATASET_INTERVAL
    // wait to reach preset data rate
    unsigned int elapsed = millis() - t;
    if (elapsed < DATASET_INTERVAL) logger.sleep(DATASET_INTERVAL - elapsed);
#endif
}
