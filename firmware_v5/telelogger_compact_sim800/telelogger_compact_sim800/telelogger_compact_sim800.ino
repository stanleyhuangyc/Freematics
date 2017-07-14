/******************************************************************************
* Simplified sketch of vehicle telematics data feed for Freematics Hub
* Works with Freematics ONE and ONE+
* Developed by Stanley Huang https://www.facebook.com/stanleyhuangyc
* Distributed under BSD license
* Visit http://freematics.com/hub for information about Freematics Hub
* Visit http://freematics.com/products for hardware information
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

int lastSpeed = 0;
uint32_t lastSpeedTime = 0;
uint32_t distance = 0;
uint8_t deviceTemp = 0; // device temperature
uint32_t lastSentTime = 0;

class CCache : public CStorageNull {
public:
    void purge() {
      m_cacheBytes = 0;
      m_samples = 0;
    }
    char* getBuffer() { return m_cache; }
    unsigned int getBytes() { return m_cacheBytes; }
    void dispatch(const char* buf, byte len)
    {
        // reserve some space for checksum
        int remain = RAM_CACHE_SIZE - m_cacheBytes - len - 2;
        if (remain < 0) {
          // m_cache full
          return;
        }
        if (m_dataTime) {
          // log timestamp
          uint32_t ts = m_dataTime;
          m_dataTime = 0;
          log(0, ts);
        }
        // store data in m_cache
        memcpy(m_cache + m_cacheBytes, buf, len);
        m_cacheBytes += len;
        m_cache[m_cacheBytes++] = ',';
        m_samples++;
    }
protected:
    unsigned int m_cacheBytes = 0;
    char m_cache[RAM_CACHE_SIZE] = {0};
};

class CTeleLogger : public CTeleClientSIM800
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

#if ENABLE_OBD
    // initialize OBD communication
    if (!checkState(STATE_OBD_READY)) {
      Serial.print("VER ");
      Serial.println(begin());

      Serial.print("OBD...");
      if (!init()) {
        Serial.println("NO");
        return false;
      }
      Serial.println("OK");
      setState(STATE_OBD_READY);
    }
#endif

#if ENABLE_GPS
    // start serial communication with GPS receiver
    if (!checkState(STATE_GPS_READY)) {
      Serial.print("GPS...");
      if (gpsInit(GPS_SERIAL_BAUDRATE)) {
        setState(STATE_GPS_READY);
        Serial.print("OK");
        blePrint("GPS OK");
      } else {
        Serial.println("NO");
      }
    }
#endif

    // initialize network module
    Serial.print(netDeviceName());
    Serial.print("...");
    if (netBegin()) {
      Serial.println("OK");
      setState(STATE_NET_READY);
    } else {
      Serial.println("NO");
      return false;
    }
    Serial.print("GPRS...");
    if (netSetup(CELL_APN)) {
      Serial.println("OK");
      setState(STATE_CONNECTED);
    } else {
      Serial.println("NO");
      return false;
    }

    Serial.print("IP:");
    Serial.println(getIP());

    txCount = 0;

    if (!login()) {
      return false;
    }

    setState(STATE_ALL_GOOD);
    return true;
  }
  void loop()
  {
#if RAM_CACHE_SIZE <= 128
    cache.purge();
#endif
    cache.timestamp(millis());

#ifdef ESP32
    deviceTemp = (int)readChipTemperature() * 165 / 255 - 40;
#endif
    if ((txCount % 100) == 1) {
      cache.log(PID_DEVICE_TEMP, deviceTemp);
    }

#if ENABLE_GPS
    // process GPS data if connected
    if (checkState(STATE_GPS_READY)) {
      processGPS();
    }
#endif

#if ENABLE_OBD
    // process OBD data if connected
    if (checkState(STATE_OBD_READY)) {
      processOBD();
    }
#endif

    // check GSM location if GPS not present
    if (!checkState(STATE_GPS_READY) && (txCount % GSM_LOCATION_INTERVAL) == 0) {
      NET_LOCATION loc = {0};
      if (getLocation(&loc)) {
        Serial.print("LAT:");
        Serial.print(loc.lat);
        Serial.print(" LNG:");
        Serial.print(loc.lng);
        cache.logCoordinate(PID_GPS_LATITUDE, loc.lat);
        cache.logCoordinate(PID_GPS_LONGITUDE, loc.lng);
      }
    }

#if ENABLE_OBD
    // read and log car battery voltage , data in 0.01v
    {
      int v = getVoltage() * 100;
      cache.log(PID_BATTERY_VOLTAGE, v);
    }
#endif

    uint32_t t = millis();
    if ((txCount % SERVER_SYNC_INTERVAL) == (SERVER_SYNC_INTERVAL - 1)) {
      // sync
      if (!syncServer()) {
        connErrors++;
      } else {
        connErrors = 0;
        txCount++;
      }
    } else if (t - lastSentTime >= DATA_SENDING_INTERVAL && cache.samples() > 0) {
      digitalWrite(PIN_LED, HIGH);
      //Serial.println(cache.getBuffer()); // print the content to be sent
      Serial.print('[');
      Serial.print(txCount);
      Serial.print("] ");
      // transmit data
      int bytesSent = transmit(cache.getBuffer(), cache.getBytes());
      if (bytesSent > 0) {
        // output some stats
        Serial.print(bytesSent);
        Serial.println("B sent");
        cache.purge();
      } else {
        Serial.println("Unsent");
      }
      digitalWrite(PIN_LED, LOW);
      if (getConnErrors() >= MAX_CONN_ERRORS_RECONNECT) {
        netClose();
        netOpen(SERVER_HOST, SERVER_PORT);
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
      if (!notifyServer(EVENT_LOGIN)) {
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
  }
  bool verifyChecksum(const char* data)
  {
    uint8_t sum = 0;
    const char *s;
    for (s = data; *s && *s != '*'; s++) sum += *s;
    return (*s && hex2uint8(s + 1) == sum);
  }
  int transmit(const char* data, int bytes)
  {
    // transmit data
    //if (data[bytes - 1] == ',') bytes--;
    int bytesSent = netSend(data, bytes, true);

    if (bytesSent == 0) {
      connErrors++;
      return 0;
    } else {
      connErrors = 0;
      txCount++;
    }
    return bytesSent;
  }
  bool syncServer()
  {
    char buf[32];
    Serial.print("Syncing...");
    int len = sprintf_P(buf, PSTR("%X#EV=%u,TS=%lu"), feedid, EVENT_SYNC, millis());
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
      return false;
    }
    Serial.println("OK");
    return true;
  }
  bool notifyServer(byte event)
  {
    for (byte attempts = 0; attempts < 3; attempts++) {
      char buf[64];
      byte n = sprintf_P(buf, PSTR("%X#EV=%u,TS=%lu,VIN=%s"), feedid, (unsigned int)event, millis(), DEFAULT_VIN);
      if (!netSend(buf, n, true)) {
        Serial.println("Unsent");
        continue;
      }
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
      sprintf_P(pattern, PSTR("EV=%u"), event);
      if (!strstr(data, pattern)) {
        Serial.println("Invalid reply");
        continue;
      }
      if (event == EVENT_LOGIN) {
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
        if (checkState(STATE_CONNECTED)) {
          notifyServer(EVENT_LOGOUT);
          netClose();
        }
        Serial.print(netDeviceName());
        netEnd(); // turn off network module power (if supported)
        Serial.println(" OFF");
        clearState(STATE_NET_READY);
      }
#if ENABLE_OBD
      if (errors > MAX_OBD_ERRORS) {
        // inaccessible OBD treated as end of trip
        feedid = 0;
      }
#endif
#if ENABLE_GPS
      if (checkState(STATE_GPS_READY)) {
        gpsInit(0); // turn off GPS power
      }
#endif
      clearState(STATE_OBD_READY | STATE_GPS_READY | STATE_NET_READY | STATE_CONNECTED);
      Serial.println("Standby");
      do {
        Serial.print('.');
        sleep(10000);
      } while (!init());
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
      cache.log(PID_TRIP_DISTANCE, distance);
      cache.log(0x100 | PID_SPEED, speed);
      // poll more PIDs
      const byte pids[]= {PID_RPM, PID_ENGINE_LOAD, PID_THROTTLE};
      int value;
      for (byte i = 0; i < sizeof(pids) / sizeof(pids[0]); i++) {
        if (readPID(pids[i], value)) {
          cache.log(0x100 | pids[i], value);
        }
      }
      static byte count = 0;
      if ((count++ % 50) == 0) {
        const byte pidTier2[] = {PID_INTAKE_TEMP, PID_COOLANT_TEMP, PID_BAROMETRIC, PID_AMBIENT_TEMP, PID_ENGINE_FUEL_RATE};
        byte pid = pidTier2[count / 50];
        if (isValidPID(pid) && readPID(pid, value)) {
          cache.log(0x100 | pid, value);
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
              Serial.print("UTC:");
              Serial.print(gd.time);
              Serial.print(" SAT:");
              Serial.println((int)gd.sat);
            }
        }
    }
#endif
#if MEMS_MODE
    void calibrateMEMS(float accBias[], unsigned int duration)
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
    CCache cache;
    byte m_state = 0;
    uint32_t m_cmd = 0;
};

CTeleLogger logger;

void setup()
{
    delay(1000);
    // initialize USB serial
    Serial.begin(115200);
    // init LED pin
    pinMode(PIN_LED, OUTPUT);
    // perform initializations
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
