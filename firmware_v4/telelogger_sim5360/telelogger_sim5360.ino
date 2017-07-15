/******************************************************************************
* Reference sketch for a vehicle telematics data feed for Freematics Hub
* Works with Freematics ONE/ONE+ with SIM5360 3G module
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
#include "datalogger.h"

// logger states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_CONNECTED 0x20
#define STATE_ALL_GOOD 0x40

#define EVENT_LOGIN 1
#define EVENT_LOGOUT 2

#if USE_MEMS
byte accCount = 0; // count of accelerometer readings
long accSum[3] = {0}; // sum of accelerometer data
int accCal[3] = {0}; // calibrated reference accelerometer data
byte deviceTemp = 0; // device temperature
#endif
int lastSpeed = 0;
uint32_t lastSpeedTime = 0;
uint32_t distance = 0;
uint32_t startTick = 0;

char udpIP[16] = {0};
uint16_t udpPort = SERVER_PORT;

class COBD3G : public COBDSPI {
public:
    COBD3G() { buffer[0] = 0; }
    bool netInit()
    {
      for (byte n = 0; n < 10; n++) {
        // try turning on module
        xbTogglePower();
        sleep(3000);
        // discard any stale data
        xbPurge();
        for (byte m = 0; m < 3; m++) {
          if (netSendCommand("AT\r"))
            return true;
        }
      }
      return false;
    }
    bool netSetup(const char* apn, bool only3G = false)
    {
      uint32_t t = millis();
      bool success = false;
      netSendCommand("ATE0\r");
      netSendCommand("AT+CNAO=2\r");
      if (only3G) netSendCommand("AT+CNMP=14\r"); // use WCDMA only
      do {
        do {
          Serial.print('.');
          sleep(500);
          success = netSendCommand("AT+CPSI?\r", 1000, "Online");
          if (success) {
            if (!strstr_P(buffer, PSTR("NO SERVICE")))
              break;
            success = false;
          } else {
            if (strstr_P(buffer, PSTR("Off"))) break;
          }
        } while (millis() - t < 60000);
        if (!success) break;

        t = millis();
        do {
          success = netSendCommand("AT+CREG?\r", 5000, "+CREG: 0,1");
        } while (!success && millis() - t < 30000);
        if (!success) break;

        do {
          success = netSendCommand("AT+CGREG?\r",1000, "+CGREG: 0,1");
        } while (!success && millis() - t < 30000);
        if (!success) break;

        do {
          sprintf_P(buffer, PSTR("AT+CGSOCKCONT=1,\"IP\",\"%s\"\r"), apn);
          success = netSendCommand(buffer);
        } while (!success && millis() - t < 30000);
        if (!success) break;

        success = netSendCommand("AT+CSOCKSETPN=1\r");
        if (!success) break;

        success = netSendCommand("AT+CIPMODE=0\r");
        if (!success) break;

        netSendCommand("AT+NETOPEN\r");
      } while(0);
      if (!success) Serial.println(buffer);
      return success;
    }
    const char* getIP()
    {
      uint32_t t = millis();
      char *ip = 0;
      do {
        if (netSendCommand("AT+IPADDR\r", 5000, "\r\nOK\r\n", true)) {
          char *p = strstr(buffer, "+IPADDR:");
          if (p) {
            ip = p + 9;
            if (*ip != '0') {
              break;
            }
          }
        }
        sleep(500);
        ip = 0;
      } while (millis() - t < 15000);
      return ip;
    }
    int getSignal()
    {
        if (netSendCommand("AT+CSQ\r", 500)) {
            char *p = strchr(buffer, ':');
            if (p) {
              p += 2;
              int db = atoi(p) * 10;
              p = strchr(p, '.');
              if (p) db += *(p + 1) - '0';
              return db;
            }
        }
        return -1;
    }
    char* getOperatorName()
    {
        // display operator name
        if (netSendCommand("AT+COPS?\r") == 1) {
            char *p = strstr(buffer, ",\"");
            if (p) {
                p += 2;
                char *s = strchr(p, '\"');
                if (s) *s = 0;
                return p;
            }
        }
        return 0;
    }
    bool udpOpen()
    {
      char *ip = queryIP(SERVER_URL);
      if (ip) {
        Serial.println(ip);
        strncpy(udpIP, ip, sizeof(udpIP) - 1);
      } else {
        return false;
      }
      sprintf_P(buffer, PSTR("AT+CIPOPEN=0,\"UDP\",\"%s\",%u,8000\r"), udpIP, udpPort);
      return netSendCommand(buffer, 3000);
    }
    void udpClose()
    {
      netSendCommand("AT+CIPCLOSE\r");
    }
    bool udpSend(const char* data, unsigned int len, bool wait = true)
    {
      sprintf_P(buffer, PSTR("AT+CIPSEND=0,%u,\"%s\",%u\r"), len, udpIP, udpPort);
      if (netSendCommand(buffer, 100, ">")) {
        xbWrite(data, len);
        if (!wait) return true;
        return waitCompletion(1000);
      } else {
        Serial.print("UDP data unsent ");
        Serial.println(buffer);
      }
      return false;
    }
    char* udpReceive(int* pbytes = 0)
    {
      if (netSendCommand(0, 3000, "+IPD")) {
        char *p = strstr(buffer, "+IPD");
        if (!p) return 0;
        int len = atoi(p + 4);
        if (pbytes) *pbytes = len;
        p = strchr(p, '\n');
        if (p) {
          *(++p + len) = 0;
          return p;
        }
      }
      return 0;
    }
    bool waitCompletion(int timeout)
    {
      return netSendCommand(0, timeout);
    }
    char* queryIP(const char* host)
    {
      sprintf_P(buffer, PSTR("AT+CDNSGIP=\"%s\"\r"), host);
      if (netSendCommand(buffer, 10000)) {
        char *p = strstr(buffer, host);
        if (p) {
          p = strstr(p, ",\"");
          if (p) {
            char *ip = p + 2;
            p = strchr(ip, '\"');
            if (p) *p = 0;
            return ip;
          }
        }
      }
      return 0;
    }
    bool netSendCommand(const char* cmd, unsigned int timeout = 2000, const char* expected = "\r\nOK\r\n", bool terminated = false)
    {
      if (cmd) {
        xbWrite(cmd);
      }
      sleep(50);
      buffer[0] = 0;
      byte ret = xbReceive(buffer, sizeof(buffer), timeout, &expected, 1);
      if (ret) {
        if (terminated) {
          char *p = strstr(buffer, expected);
          if (p) *p = 0;
        }
        return true;
      } else {
        return false;
      }
    }
    char buffer[128];
};

class CTeleClient2 : public COBD3G, public CDataLogger
{
public:
  CTeleClient2():connErrors(0),txCount(0),feedid(0) {}
  bool verifyChecksum(const char* data)
  {
    uint8_t sum = 0;
    const char *s;
    for (s = data; *s && *s != '*'; s++) sum += *s;
    return (*s && hex2uint8(s + 1) == sum);
  }
  bool notifyUDP(byte event)
  {
    waiting = false;
    for (byte attempts = 0; attempts < 3; attempts++) {
      // generate request
      setEvent(event);
      if (event == EVENT_LOGIN) {
        // load DTC
        uint16_t dtc[6];
        byte dtcCount = readDTC(dtc, sizeof(dtc) / sizeof(dtc[0]));
        if (dtcCount > 0) {
          Serial.print("#DTC:");
          Serial.println(dtcCount);
          cacheBytes += sprintf_P(cache + cacheBytes, PSTR(",DTC="), dtcCount);
          for (byte i = 0; i < dtcCount; i++) {
            cacheBytes += sprintf_P(cache + cacheBytes, PSTR("%X;"), dtc[i]);
          }
          cacheBytes--;
        }
        // load VIN
        cacheBytes += sprintf_P(cache + cacheBytes, PSTR(",VIN="));
        if (getVIN(cache + cacheBytes, CACHE_SIZE - cacheBytes)) {
          Serial.print("#VIN:");
          Serial.println(cache + cacheBytes);
          cacheBytes += strlen(cache + cacheBytes);
        } else {
          cacheBytes += sprintf_P(cache + cacheBytes, PSTR("DEFAULT_VIN"));
        }
      }
      transmitUDP(true);
      // receive reply
      int len;
      char *data = udpReceive(&len);
      if (!data) {
        Serial.println("No reply");
        continue;
      }
      connErrors = 0;
      // verify checksum
      if (!verifyChecksum(data)) {
        Serial.println("Wrong checksum");
        continue;
      }
      char pattern[16];
      sprintf_P(pattern, PSTR("EV=%u"), event);
      if (!strstr(data, pattern)) {
        Serial.println("Invalid reply");
        continue;
      }
      if (event == EVENT_LOGIN) {
        feedid = hex2uint16(data);
        Serial.print("#FEED ID:");
        Serial.println(feedid);
      }
      return true;
    }
    return false;
  }
  void transmitUDP(bool wait)
  {
    addDataChecksum();
    if (waiting) {
      // wait for last data to be sent
      if (!waitCompletion(100)) {
        connErrors++;
        Serial.println(buffer);
      } else {
        connErrors = 0;
        txCount++;
      }
      waiting = false;
    }

    // transmit data
    if (!udpSend(cache, cacheBytes, wait)) {
      connErrors++;
    } else {
      if (!wait) {
        waiting = true;
      } else {
        connErrors = 0;
        txCount++;
      }
    }
    // clear cache and write header for next transmission
    cacheBytes = sprintf_P(cache, PSTR("%X#"), feedid);
  }
  void setEvent(byte event)
  {
    cacheBytes = sprintf_P(cache, PSTR("%X#EV=%u,SK=%s,TS=%lu"), feedid, (unsigned int)event, SERVER_KEY, millis());
  }
  bool reconnect()
  {
      udpClose();
      udpOpen();
      return notifyUDP(EVENT_LOGIN);
  }
  void showStats()
  {
    Serial.print('#');
    Serial.print(txCount);
    Serial.print(' ');
    Serial.print(cacheBytes);
    Serial.print(" bytes ");
    Serial.print((millis() - startTick) / (txCount + 1));
    Serial.println("ms");
    // output data in cache
    //Serial.println(cache);
  }
  uint16_t feedid;
  uint32_t txCount;
  uint8_t connErrors;
  bool waiting;
};

class CTeleLogger : public CTeleClient2
{
public:
  CTeleLogger():m_state(0) {}
  bool setup()
  {
    clearState(STATE_ALL_GOOD);

#if USE_MEMS
    if (!checkState(STATE_MEMS_READY)) {
      Serial.print("#MEMS...");
      if (mems.memsInit()) {
        setState(STATE_MEMS_READY);
        Serial.println("OK");
      } else {
        Serial.println("NO");
      }
    }
#endif
    // initialize OBD communication
    if (!checkState(STATE_OBD_READY)) {
      Serial.print("#OBD...");
      if (!init()) {
        Serial.println("NO");
        return false;
      }
      Serial.println("OK");
      setState(STATE_OBD_READY);
    }

#if USE_GPS
    // start serial communication with GPS receiver
    if (!checkState(STATE_GPS_READY)) {
      Serial.print("#GPS...");
      if (gpsInit(GPS_SERIAL_BAUDRATE)) {
        setState(STATE_GPS_READY);
#ifdef ESP32
        Serial.print("OK(");
        Serial.print(internalGPS() ? "internal" : "external");
        Serial.println(')');
#else
        Serial.println("OK");
#endif
      } else {
        Serial.println("NO");
      }
    }
#endif

    // initialize SIM5360 xBee module (if present)
    if (!checkState(STATE_NET_READY)) {
      Serial.print("#SIM5360...");
      xbBegin(XBEE_BAUDRATE);
      if (netInit()) {
        Serial.println("OK");
        setState(STATE_NET_READY);
      } else {
        Serial.println("NO");
        return false;
      }
    }

    Serial.print("#3G(APN:");
    Serial.print(APN);
    Serial.print(")");
    if (netSetup(APN)) {
      char *op = getOperatorName();
      Serial.println(op ? op : "OK");
      setState(STATE_CONNECTED);
    } else {
      Serial.println("NO");
      Serial.print("#CSQ...");
      Serial.println(getSignal());
      return false;
    }

    Serial.print("#IP...");
    const char *ip = getIP();
    if (ip) {
      Serial.print(ip);
    } else {
      Serial.println("NO");
    }

    Serial.print("#CSQ...");
    int csq = getSignal();
    Serial.println(csq);

    startTick = millis();
    txCount = 0;

    // connect to cellular network
    for (byte attempts = 0; attempts < 3; attempts++) {
      Serial.print("#SERVER...");
      if (!udpOpen()) {
        Serial.println("NO");
        continue;
      }

      // login Freematics Hub
      if (!notifyUDP(EVENT_LOGIN)) {
        continue;
      }
      // log signal level
      logData(PID_CSQ, csq);

      setState(STATE_ALL_GOOD);
      break;
    }

#if USE_MEMS
    calibrateMEMS(CALIBRATION_TIME);
#endif
    return checkState(STATE_ALL_GOOD);
  }
  void loop()
  {
    logTimestamp(millis());

    // process OBD data if connected
    if (checkState(STATE_OBD_READY)) {
      processOBD();
    }

#if USE_MEMS
    // process MEMS data if available
    if (checkState(STATE_MEMS_READY)) {
        processMEMS();
    }
#endif

#if USE_GPS
    // process GPS data if connected
    if (checkState(STATE_GPS_READY)) {
      processGPS();
    }
#endif
#if CACHE_SIZE > 128
    // read and log car battery voltage , data in 0.01v
    {
      int v = getVoltage() * 100;
      logData(PID_BATTERY_VOLTAGE, v);
    }
#endif

#if USE_MEMS
    if ((txCount % 100) == 1) {
      logData(PID_DEVICE_TEMP, deviceTemp);
    }
    if (deviceTemp >= COOLING_DOWN_TEMP && deviceTemp < 100) {
      // device too hot, cool down
      Serial.print("Cooling (");
      Serial.print(deviceTemp);
      Serial.println("C)");
      sleep(10000);
    }
#endif
  }
  void standby()
  {
      if (checkState(STATE_NET_READY)) {
        notifyUDP(EVENT_LOGOUT);
        udpClose();
        Serial.print("#3G:");
        xbTogglePower(); // turn off GSM power
        Serial.println("OFF");
        clearState(STATE_NET_READY);
      }
      if (errors > MAX_OBD_ERRORS) {
        feedid = 0;
      }
#if USE_GPS
      if (checkState(STATE_GPS_READY)) {
        Serial.print("#GPS:");
        gpsInit(0); // turn off GPS power
        Serial.println("OFF");
      }
#endif
      clearState(STATE_OBD_READY | STATE_GPS_READY | STATE_NET_READY | STATE_CONNECTED);
      Serial.println("Standby");
#if USE_MEMS
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
          motion /= 100;
          Serial.println(motion);
          // check movement
          if (motion > WAKEUP_MOTION_THRESHOLD) {
            leaveLowPowerMode();
            break;
          }
          // MEMS data collected while sleeping
          sleep(3000);
        }
      }
#else
      do {
        enterLowPowerMode();
        sleepSec(10);
        leaveLowPowerMode();
      } while (!init());
#endif
      Serial.println("Wakeup");
  }
  bool checkState(byte flags) { return (m_state & flags) == flags; }
  void setState(byte flags) { m_state |= flags; }
  void clearState(byte flags) { m_state &= ~flags; }
private:
  void processOBD()
  {
      int speed = readSpeed();
      if (speed == -1) {
        return;
      }
      logData(0x100 | PID_SPEED, speed);
      logData(PID_TRIP_DISTANCE, distance);
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
            logData(0x100 | pids[n], values[n]);
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
#if USE_MEMS
    void processMEMS()
    {
         // log the loaded MEMS data
        if (accCount) {
          logData(PID_ACC, accSum[0] / accCount, accSum[1] / accCount, accSum[2] / accCount);
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
            if (lastUTC != (uint16_t)gd.time) {
              byte day = gd.date / 10000;
              logData(PID_GPS_TIME, gd.time);
              if (lastGPSDay != day) {
                logData(PID_GPS_DATE, gd.date);
                lastGPSDay = day;
              }
              logCoordinate(PID_GPS_LATITUDE, gd.lat);
              logCoordinate(PID_GPS_LONGITUDE, gd.lng);
              logData(PID_GPS_ALTITUDE, gd.alt);
              logData(PID_GPS_SPEED, gd.speed);
              logData(PID_GPS_SAT_COUNT, gd.sat);
              lastUTC = (uint16_t)gd.time;
              Serial.print("#UTC:");
              Serial.print(gd.time);
              Serial.print(" SAT:");
              Serial.println(gd.sat);
            }
        }
    }
#if USE_MEMS
    void calibrateMEMS(unsigned int duration)
    {
        // MEMS data collected while sleeping
        if (duration > 0) {
          accCount = 0;
          accSum[0] = 0;
          accSum[1] = 0;
          accSum[2] = 0;
          sleep(duration);
        }
        // store accelerometer reference data
        if (checkState(STATE_MEMS_READY) && accCount) {
          accCal[0] = accSum[0] / accCount;
          accCal[1] = accSum[1] / accCount;
          accCal[2] = accSum[2] / accCount;
        }
    }
    void readMEMS()
    {
        // load accelerometer and temperature data
        float acc[3];
        int16_t temp; // device temperature (in 0.1 celcius degree)
        mems.memsRead(acc, 0, 0, &temp);
        if (accCount >= 100) {
          accSum[0] >>= 1;
          accSum[1] >>= 1;
          accSum[2] >>= 1;
          accCount >>= 1;
        }
        accSum[0] += acc[0] * 100;
        accSum[1] += acc[1] * 100;
        accSum[2] += acc[2] * 100;
        accCount++;
        deviceTemp = temp / 10;
    }
#endif
    void dataIdleLoop()
    {
      // do tasks while waiting for data on SPI
#if USE_MEMS
      if (checkState(STATE_MEMS_READY)) {
        readMEMS();
      }
#endif
    }
    MPU9250_ACC mems;

    byte m_state;
};

CTeleLogger logger;

void setup()
{
    // initialize hardware serial (for USB and BLE)
    Serial.begin(115200);
    Serial.println("Freematics ONE");
    // perform initializations
    logger.begin();
    delay(1000);
    logger.setup();
}

void loop()
{
    // error handling
    if (!logger.checkState(STATE_ALL_GOOD) || (logger.errors > MAX_OBD_ERRORS && !logger.init()) || logger.connErrors >= MAX_CONN_ERRORS) {
      do {
        logger.standby();
      } while (!logger.setup());
    }
#if DATASET_INTERVAL
    uint32_t t = millis();
#endif
    // collect data
    logger.loop();
    // show stats info
    logger.showStats();
    // transmit data
    logger.transmitUDP(false);
    if (logger.connErrors >= MAX_CONN_ERRORS_RECONNECT) {
      logger.reconnect();
    }

#if DATASET_INTERVAL
    // wait to reach preset data rate
    unsigned int elapsed = millis() - t;
    if (elapsed < DATASET_INTERVAL) logger.sleep(DATASET_INTERVAL - elapsed);
#endif
}
