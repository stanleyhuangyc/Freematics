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

#include <FreematicsONE.h>
#include "config.h"

// logger states
#define STATE_STORAGE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_CONNECTED 0x20
#define STATE_ALL_GOOD 0x40

#define PIN_LED 4

#if MEMS_MODE
byte accCount = 0; // count of accelerometer readings
float accSum[3] = {0}; // sum of accelerometer data
float accBias[3] = {0}; // calibrated reference accelerometer data
#endif
int lastSpeed = 0;
uint32_t lastSpeedTime = 0;
uint32_t distance = 0;
uint32_t startTick = 0;
uint32_t lastSentTime = 0;
uint32_t lastSyncTime = 0;
uint8_t deviceTemp = 0; // device temperature

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
{
public:
  bool setup()
  {
    clearState(STATE_ALL_GOOD);

#if MEMS_MODE
    if (!checkState(STATE_MEMS_READY)) {
      Serial.print("MEMS...");
      if (mems.memsInit()) {
        setState(STATE_MEMS_READY);
        Serial.println("OK");
        blePrint("MEMS OK");
      } else {
        Serial.println("NO");
      }
    }
#endif

#if ENABLE_OBD
    // initialize OBD communication
    if (!checkState(STATE_OBD_READY)) {
      Serial.print("OBD...");
      begin();
      if (!init()) {
        Serial.println("NO");
        return false;
      }
      Serial.println("OK");
      blePrint("OBD OK");
      setState(STATE_OBD_READY);
    }
#endif

if (!checkState(STATE_STORAGE_READY)) {
  // init storage
  cache.init(RAM_CACHE_SIZE);
#if STORAGE_TYPE != STORAGE_NONE
  if (store.init()) {
    setState(STATE_STORAGE_READY);
    if (store.begin()) {
      cache.setForward(&store);
    }
  }
#else
  setState(STATE_STORAGE_READY);
#endif
}

#if NET_DEVICE == NET_WIFI
    Serial.print("WIFI(SSID:");
    Serial.print(WIFI_SSID);
    Serial.print(")...");
    if (netBegin() && netSetup(WIFI_SSID, WIFI_PASSWORD)) {
      blePrint("WIFI OK");
      Serial.println("OK");
      setState(STATE_NET_READY | STATE_CONNECTED);
    } else {
      Serial.println("NO");
      return false;
    }
#elif NET_DEVICE == NET_SIM800 || NET_DEVICE == NET_SIM5360
    // initialize network module
    if (!checkState(STATE_NET_READY)) {
      Serial.print(netDeviceName());
      Serial.print("...");
      if (netBegin()) {
        Serial.println("OK");
        blePrint("NET OK");
        setState(STATE_NET_READY);
      } else {
        Serial.println("NO");
        return false;
      }
    }
    Serial.print("CELL(APN:");
    Serial.print(CELL_APN);
    Serial.print(")");
    if (netSetup(CELL_APN)) {
      blePrint("CELL OK");
      String op = getOperatorName();
      if (op.length()) {
        Serial.println(op);
        blePrint(op);
      } else {
        Serial.println("OK");
      }
      setState(STATE_CONNECTED);
    } else {
      Serial.println("NO");
      return false;
    }
#endif

#if ENABLE_GPS
    // start serial communication with GPS receiver
    if (!checkState(STATE_GPS_READY)) {
      Serial.print("GPS...");
      if (gpsInit(GPS_SERIAL_BAUDRATE)) {
        setState(STATE_GPS_READY);
        Serial.print("OK(");
        Serial.print(internalGPS() ? "internal" : "external");
        Serial.println(')');
        blePrint("GPS OK");
      } else {
        Serial.println("NO");
      }
    }
#endif

#if NET_DEVICE == NET_WIFI || NET_DEVICE == NET_SIM800 || NET_DEVICE == NET_SIM5360
    Serial.print("IP...");
    String ip = getIP();
    if (ip.length()) {
      Serial.println(ip);
      blePrint(ip);
    } else {
      Serial.println("NO");
    }
    int csq = getSignal();
    if (csq > 0) {
      Serial.print("CSQ...");
      Serial.print((float)csq / 10, 1);
      Serial.println("dB");
    }
#endif

    startTick = millis();
    txCount = 0;

    if (!login()) {
      return false;
    }
    lastSyncTime = millis();

#if NET_DEVICE == NET_SIM800 || NET_DEVICE == NET_SIM5360
    // log signal level
    if (csq) cache.log(PID_CSQ, csq);
#endif

    setState(STATE_ALL_GOOD);
    return true;
  }
  void loop()
  {
    cache.timestamp(millis());

#ifdef ESP32
    deviceTemp = (int)readChipTemperature() * 165 / 255 - 40;
#endif
    if ((txCount % 100) == 1) {
      cache.log(PID_DEVICE_TEMP, deviceTemp);
    }

#if ENABLE_OBD
    // process OBD data if connected
    if (checkState(STATE_OBD_READY)) {
      processOBD();
    }
#endif

#if ENABLE_GPS
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

#if MEMS_MODE
    // process MEMS data if available
    if (checkState(STATE_MEMS_READY)) {
      readMEMS();
      processMEMS();
    }
#endif

    uint32_t t = millis();
    if (t - lastSyncTime > SERVER_SYNC_INTERVAL) {
      // sync
      if (!syncServer()) {
        connErrors++;
      } else {
        connErrors = 0;
        lastSyncTime = t;
      }
    } else if (t - lastSentTime >= DATA_SENDING_INTERVAL && cache.samples() > 0) {
      digitalWrite(PIN_LED, HIGH);
      //Serial.println(cache.getBuffer()); // print the content to be sent
      Serial.print('[');
      Serial.print(txCount);
      Serial.print("] ");
      // transmit data
      int bytesSent = transmit(cache.getBuffer(), cache.getBytes(), false);
      if (bytesSent > 0) {
        // output some stats
        char buf[16];
        sprintf(buf, "%uB sent", bytesSent);
        Serial.print(buf);
        blePrint(buf);
#if STORAGE_TYPE != STORAGE_NONE
        if (checkState(STATE_STORAGE_READY)) {
          Serial.print(" | ");
          Serial.print(store.size() >> 10);
          Serial.print("KB stored");
        }
#endif
        Serial.println();
        cache.purge();
      } else {
        Serial.println("Unsent");
        blePrint("Unsent");
      }
      digitalWrite(PIN_LED, LOW);
      if (getConnErrors() >= MAX_CONN_ERRORS_RECONNECT) {
        netClose();
        netOpen(SERVER_HOST, SERVER_PORT);
        lastSyncTime = 0; // sync on next time
      }
      lastSentTime = t;
    }

#if ENABLE_OBD
    if (errors > MAX_OBD_ERRORS) {
      reset();
      if (!init()) {
        clearState(STATE_OBD_READY | STATE_GPS_READY | STATE_ALL_GOOD);
      }
    }
  #endif

    if (deviceTemp >= COOLING_DOWN_TEMP) {
      // device too hot, cool down
      Serial.println("Cooling down...");
      sleep(10000);
    }
  }
  bool login()
  {
    char payload[256];
    int bytes = 0;
#if ENABLE_OBD
    // load DTC
    uint16_t dtc[6];
    byte dtcCount = readDTC(dtc, sizeof(dtc) / sizeof(dtc[0]));
    if (dtcCount > 0) {
      Serial.print("DTC:");
      Serial.println(dtcCount);
      bytes += sprintf(payload + bytes, ",DTC=");
      for (byte i = 0; i < dtcCount; i++) {
        bytes += sprintf(payload + bytes, "%X;", dtc[i]);
      }
      bytes--;
    }
    bytes += sprintf(payload + bytes, ",VIN=");
    if (getVIN(payload + bytes, sizeof(payload) - bytes)) {
      Serial.print("VIN:");
      Serial.println(payload + bytes);
    } else {
      strcpy(payload + bytes, DEFAULT_VIN);
    }
#else
    sprintf(payload, ",VIN=%s", DEFAULT_VIN);
#endif

#if NET_DEVICE == NET_WIFI || NET_DEVICE == NET_SIM800 || NET_DEVICE == NET_SIM5360
    // connect to telematics server
    for (byte attempts = 0; attempts < 3; attempts++) {
      Serial.print("LOGIN...");
      if (netOpen(SERVER_HOST, SERVER_PORT)) {
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

      Serial.print("SERVER:");
      Serial.println(serverName());

      Serial.print("FEED ID:");
      Serial.println(feedid);

      return true;
    }
    return false;
#elif NET_DEVICE == NET_BLE
    blePrint(payload);
    return true;
#endif
  }
  bool verifyChecksum(const char* data)
  {
    uint8_t sum = 0;
    const char *s;
    for (s = data; *s && *s != '*'; s++) sum += *s;
    return (*s && hex2uint8(s + 1) == sum);
  }
  int transmit(const char* data, int bytes, bool wait)
  {
    if (m_waiting) {
      // wait for last data to be sent
      if (!netWaitSent(100)) {
        connErrors++;
        //Serial.println(m_buffer);
      } else {
        connErrors = 0;
        txCount++;
      }
      m_waiting = false;
    }
    // transmit data
    if (data[bytes - 1] == ',') bytes--;
    int bytesSent = netSend(data, bytes, wait);

    if (bytesSent == 0) {
      connErrors++;
      return 0;
    } else {
      if (!wait) {
        m_waiting = true;
      } else {
        connErrors = 0;
        txCount++;
      }
    }
    return bytesSent;
  }
  bool syncServer()
  {
    char buf[32];
    Serial.print("Syncing...");
    int len = sprintf(buf, "%X#EV=%u,TS=%lu", feedid, EVENT_SYNC, millis());
    if (!netSend(buf, len, true)) {
      Serial.println("Sync error");
      return false;
    }
    char *data = netReceive(&len);
    if (!data) {
      Serial.println("no reply");
      return false;
    }
    if (!verifyChecksum(data)) {
      Serial.println("checksum mismatch");
      return false;
    }
    char *p = strstr(data, "CMD=");
    if (p) m_cmd = atoi(p + 4);
    Serial.print("OK");
    if (m_cmd) {
      Serial.print(" CMD:");
      Serial.print(m_cmd, HEX);
    }
    Serial.println();
    return true;
  }
  bool notifyServer(byte event, const char* serverKey, const char* payload)
  {
    char buf[64];
    sprintf(buf, "%X#EV=%u,TS=%lu", feedid, (unsigned int)event, millis());
    String req = buf;
    if (serverKey) {
      req += ",SK=";
      req += serverKey;
    }
    if (payload) {
      req += payload;
    }
    for (byte attempts = 0; attempts < 3; attempts++) {
      if (!netSend(req.c_str(), req.length(), true)) {
        Serial.println("Data sending error");
        continue;
      }
#if NET_DEVICE == NET_BLE
        return true;
#endif

      // receive reply
      int len;
      char *data = netReceive(&len);
      if (!data) {
        Serial.println("No reply");
        continue;
      }
      connErrors = 0;
      // verify checksum
      if (!verifyChecksum(data)) {
        Serial.print("Checksum mismatch:");
        Serial.println(data);
        continue;
      }
      char pattern[16];
      sprintf(pattern, "EV=%u", event);
      if (!strstr(data, pattern)) {
        Serial.println("Invalid reply");
        continue;
      }
      if (event == EVENT_LOGIN) {
        char *p = strstr(data, "SN=");
        if (p) {
          char *q = strchr(p, ',');
          if (q) *q = 0;
          m_serverName = p;
        }
        feedid = atoi(data);
      }
      return true;
    }
    return false;
  }
  void resetNetwork()
  {
    netClose();
    netEnd();
    clearState(STATE_NET_READY);
    setup();
  }
  void standby()
  {
      if (checkState(STATE_NET_READY)) {
        notifyServer(EVENT_LOGOUT, SERVER_KEY, 0);
        netClose();
        Serial.print("Network:");
        netEnd(); // turn off network module power (if supported)
        Serial.println("OFF");
        clearState(STATE_NET_READY);
      }
#if STORAGE_TYPE != STORAGE_NONE
      if (checkState(STATE_STORAGE_READY)) {
        store.end();
        clearState(STATE_STORAGE_READY);
      }
#endif
#if ENABLE_OBD
      if (errors > MAX_OBD_ERRORS) {
        // inaccessible OBD treated as end of trip
        feedid = 0;
      }
#endif
#if ENABLE_GPS
      if (checkState(STATE_GPS_READY)) {
        Serial.print("GPS:");
        gpsInit(0); // turn off GPS power
        Serial.println("OFF");
      }
#endif
      clearState(STATE_OBD_READY | STATE_GPS_READY | STATE_NET_READY | STATE_CONNECTED);
      Serial.println("Standby");
      blePrint("Standby");
#if MEMS_MODE
      if (checkState(STATE_MEMS_READY)) {
        calibrateMEMS(CALIBRATION_TIME);
        for (;;) {
          // calculate relative movement
          clearMEMS();
          for (byte i = 0; i < 20; i++) {
            readMEMS();
            delay(50);
          }
          float motion = 0;
          Serial.print("ACC:");
          for (byte i = 0; i < 3; i++) {
            float a = accSum[i] / accCount - accBias[i];
            Serial.print(a);
            Serial.print(' ');
            motion += a * a;
          }
          motion *= 10000;
          char buf[16];
          sprintf(buf, "M:%ld", (long)motion);
          Serial.println(buf);
          blePrint(buf);
          if (motion >= WAKEUP_MOTION_THRESHOLD) {
            bool success = false;
            // check if OBD can be connected
            Serial.print("Checking OBD");
            blePrint("Checking OBD");
            for (uint32_t t = millis(); !success && millis() - t < MAX_OBD_RETRY_TIME;) {
              Serial.print('.');
              success = init();
            }
            Serial.println();
            if (success) {
              setState(STATE_OBD_READY);
              break;
            }
            calibrateMEMS(CALIBRATION_TIME);
          }
        }
      } else {
        while (!init()) Serial.print('.');
      }
#else
      while (!init()) Serial.print('.');
#endif
      Serial.println("Wakeup");
      blePrint("Wakeup");
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
      const byte pids[]= {PID_RPM, PID_ENGINE_LOAD, PID_THROTTLE};
      int value;
      for (byte i = 0; i < sizeof(pids) / sizeof(pids[0]); i++) {
        if (readPID(pids[i], value)) {
          cache.log(0x100 | pids[i], value);
        }
#if MEMS_MODE
        readMEMS();
#endif
      }
      static byte count = 0;
      if ((count++ % 50) == 0) {
        const byte pidTier2[] = {PID_INTAKE_TEMP, PID_COOLANT_TEMP, PID_BAROMETRIC, PID_AMBIENT_TEMP, PID_ENGINE_FUEL_RATE};
        byte pid = pidTier2[count / 50];
        if (isValidPID(pid) && readPID(pid, value)) {
          cache.log(0x100 | pid, value);
        }
#if MEMS_MODE
        readMEMS();
#endif
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
#if MEMS_MODE
    void processMEMS()
    {
         // log the loaded MEMS data
        if (accCount) {
          cache.log(PID_ACC,
            (int16_t)(accSum[0] / accCount * 100),
            (int16_t)(accSum[1] / accCount * 100),
            (int16_t)(accSum[2] / accCount * 100)
          );
          clearMEMS();
        }
    }
#endif
#if ENABLE_GPS
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
              char buf[32];
              sprintf(buf, "UTC:%08lu SAT:%u", gd.time, (unsigned int)gd.sat);
              Serial.println(buf);
              blePrint(buf);
            }
        }
    }
#endif
#if MEMS_MODE
    void calibrateMEMS(unsigned int duration)
    {
        // MEMS data collected while sleeping
        clearMEMS();
        for (uint32_t t = millis(); millis() - t < duration; ) {
          readMEMS();
          delay(20);
        }
        // store accelerometer reference data
        if (accCount) {
          accBias[0] = accSum[0] / accCount;
          accBias[1] = accSum[1] / accCount;
          accBias[2] = accSum[2] / accCount;
        }
        Serial.print("ACC Bias:");
        Serial.print(accBias[0]);
        Serial.print('/');
        Serial.print(accBias[1]);
        Serial.print('/');
        Serial.println(accBias[2]);
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
      if (checkState(STATE_MEMS_READY)) {
        // load and store accelerometer
        float acc[3];
        int16_t temp;
        mems.memsRead(acc, 0, 0, &temp);
        if (accCount >= 255) {
          clearMEMS();
        }
        accSum[0] += acc[0];
        accSum[1] += acc[1];
        accSum[2] += acc[2];
        accCount++;
#ifndef ESP32
        deviceTemp = temp / 10;
#endif
      }
    }
#endif
#if MEMS_MODE == MEMS_ACC
    MPU9250_ACC mems;
#elif MEMS_MODE == MEMS_9DOF
    MPU9250_9DOF mems;
#elif MEMS_MODE == MEMS_DMP
    MPU9250_DMP mems;
#endif
    CStorageRAM cache;
#if STORAGE_TYPE == STORAGE_SD
    CStorageSD store;
#endif
    byte m_state = 0;
    bool m_waiting = false;
    uint32_t m_cmd = 0;
};

CTeleLogger logger;

void setup()
{
    delay(1000);
    // initialize USB serial
    Serial.begin(115200);
#if ENABLE_OBD
    Serial.println("Freematics ONE+ (ESP32)");
#else
    Serial.println("Freematics Esprit (ESP32)");
#endif
    // init LED pin
    pinMode(PIN_LED, OUTPUT);
    // perform initializations
#if ENABLE_BLE
    logger.bleBegin(BLE_DEVICE_NAME);
    delay(500);
#endif
    digitalWrite(PIN_LED, HIGH);
    logger.setup();
    digitalWrite(PIN_LED, LOW);
}

void loop()
{
    // error handling
    if (!logger.checkState(STATE_ALL_GOOD)) {
      do {
        digitalWrite(PIN_LED, LOW);
        logger.standby();
        digitalWrite(PIN_LED, HIGH);
      } while (!logger.setup());
      digitalWrite(PIN_LED, LOW);
    }
    if (logger.getConnErrors() >= MAX_CONN_ERRORS) {
      digitalWrite(PIN_LED, HIGH);
      logger.resetNetwork();
      digitalWrite(PIN_LED, LOW);
      return;
    }
    // collect and transmit data
    logger.loop();
}
