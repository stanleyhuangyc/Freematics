/******************************************************************************
* Reference sketch for a vehicle telematics data feed for Freematics Hub
* Works with Freematics ONE with ESP8266 WIFI module
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
#define STATE_ALL_GOOD 0x40

#define EVENT_LOGIN 1
#define EVENT_LOGOUT 2

#if USE_MEMS
float accBias[3] = {0}; // calibrated reference accelerometer data
byte deviceTemp = 0; // device temperature
#endif
int lastSpeed = 0;
uint32_t lastSpeedTime = 0;
uint32_t distance = 0;
uint32_t startTick;

char udpIP[16] = "47.74.68.134";
uint16_t udpPort = SERVER_PORT;

class COBDWIFI : public COBDSPI {
public:
    COBDWIFI() { buffer[0] = 0; }
    void netReset()
    {
      netSendCommand("AT+RST\r\n");
    }
    void netDisconnect()
    {
      netSendCommand("AT+CWQAP\r\n");
    }
    bool netInit()
    {
      // set xBee module serial baudrate
      bool success = false;
      // test the module by issuing AT command and confirming response of "OK"
      for (byte n = 0; !(success = netSendCommand("ATE0\r\n")) && n < 10; n++) {
        sleep(100);
      }
      if (success && netSendCommand("AT+CWMODE=1\r\n", 100) && netSendCommand("AT+CIPMUX=0\r\n", 100)) {
        return true;
      } else {
        return false;
      }
    }
    bool netSetup()
    {
      // generate and send AT command for joining AP
      sprintf_P(buffer, PSTR("AT+CWJAP=\"%s\",\"%s\"\r\n"), WIFI_SSID, WIFI_PASSWORD);
      byte ret = netSendCommand(buffer, 10000);
      if (ret == 1 || strstr_P(buffer, PSTR("WIFI CONNECTED"))) {
        // get IP address
        if (netSendCommand("AT+CIFSR\r\n", 5000) && !strstr_P(buffer, PSTR("0.0.0.0"))) {
          char *p = strchr(buffer, '\"');
          char *ip = p ? p + 1 : buffer;
          if ((p = strchr(ip, '\"')) || (p = strchr(ip, '\r'))) *p = 0;
          // output IP address
          Serial.println(ip);
          return true;
        }
      }
      Serial.println(buffer);
      return false;
    }
    void udpClose()
    {
      netSendCommand("AT+CIPCLOSE\r\n", 500);
    }
    bool udpOpen()
    {
      char *ip = queryIP(SERVER_URL);
      if (ip) {
        //Serial.println(ip);
        strncpy(udpIP, ip, sizeof(udpIP) - 1);
      }
      for (byte n = 0; n < 3; n++) {
        sprintf_P(buffer, PSTR("AT+CIPSTART=\"UDP\",\"%s\",%d,8000,0\r\n"), udpIP, udpPort);
        if (netSendCommand(buffer, 3000)) {
          return true;
        } else {
          // check if already connected
          if (strstr(buffer, "CONN")) return true;
        }
        Serial.println(buffer);
      }
      return false;
    }
    bool udpSend(const char* data, unsigned int len)
    {
      sprintf_P(buffer, PSTR("AT+CIPSEND=%u\r\n"), len);
      if (netSendCommand(buffer, 100, ">") && netSendCommand(data, 1000, "SEND OK")) {
        return true;
      } else {
        Serial.println(buffer);
        return false;
      }
    }
    char* udpReceive(int* pbytes = 0)
    {
      char *p = buffer;
      for (byte n = 0; n < 2; n++) {
        p = strstr(p, "+IPD,");
        if (!p) {
          if (!netSendCommand(0, 3000, "+IPD,")) return 0;
          p = buffer;
          continue;
        }
        p += 5;
        int len = atoi(p);
        if (pbytes) *pbytes = len;
        char *q = strchr(p, ':');
        if (q++) {
          *(q + len) = 0;
          return q;
        }
      }
      return 0;
    }
    char* queryIP(const char* host)
    {
      sprintf_P(buffer, PSTR("AT+CIPDOMAIN=\"%s\"\r\n"), host);
      if (netSendCommand(buffer, 5000, "+CIPDOMAIN")) {
        char *p;
        if ((p = strstr(buffer, "+CIPDOMAIN")) && (p = strchr(p, ':'))) {
          char *ip = p + 1;
          p = strchr(ip, '\r');
          if (p) *p = 0;
          return ip;
        }
      }
      return 0;
    }
    bool netSendCommand(const char* cmd, unsigned int timeout = 2000, const char* expected = "OK", bool terminated = false)
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
    char buffer[96];
    bool connected = false;
};

class CTeleClient2 : public COBDWIFI, public CDataLogger
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
      transmitUDP();
      // receive reply
      int len;
      char *data = udpReceive(&len);
      if (!data) {
        Serial.println(buffer);
        Serial.println("No reply");
        delay(1000);
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
  void transmitUDP()
  {
    addDataChecksum();
    // transmit data
    if (!udpSend(cache, cacheBytes)) {
      connErrors++;
    } else {
      connErrors = 0;
      txCount++;
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
    Serial.print((millis() - startTick) / txCount);
    Serial.print("ms");
    Serial.println();
    // output data in cache
    //Serial.println(cache);
  }
  uint16_t feedid;
  uint32_t txCount;
  uint8_t connErrors;
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

    if (!checkState(STATE_NET_READY)) {
      // attempt to join AP with pre-defined credential
      // make sure to change WIFI_SSID and WIFI_PASSWORD to your own ones
      // initialize ESP8266 xBee module (if present)
      Serial.print("#ESP8266...");
      xbBegin(XBEE_BAUDRATE);
      if (netInit()) {
        Serial.println("OK");
      } else {
        Serial.println("NO");
        return false;
      }
      
      connected = false;
      for (byte n = 0; n < 10 && !connected; n++) {
        Serial.print("#WIFI(SSID:");
        Serial.print(WIFI_SSID);
        Serial.print(")...");
        connected = netSetup();
        if (!connected) {
          Serial.println("NO");          
        }
      }
      if (connected) {
          Serial.print("#UDP...");
          if (udpOpen()) {
            setState(STATE_NET_READY);
            Serial.println("OK");
          } else {
            Serial.println("NO");
            return false;
          }
      } else {
        return false;
      }
    }
    // connect to internet server
    for (byte attempts = 0; attempts < 3; attempts++) {
      Serial.print("#SERVER...");
      if (!udpOpen()) {
        Serial.println("NO");
        continue;
      } else {
        Serial.println("OK");
      }

      // login Freematics Hub
      if (!notifyUDP(EVENT_LOGIN)) {
        continue;
      }
      setState(STATE_ALL_GOOD);
      break;
    }

#if USE_MEMS
    calibrateMEMS();
#endif

    startTick = millis();
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
          netDisconnect(); // disconnect from AP
          sleep(500);
          netReset();
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
      clearState(STATE_OBD_READY | STATE_GPS_READY | STATE_NET_READY);
      Serial.println("Standby");
#if USE_MEMS
      calibrateMEMS();
      if (checkState(STATE_MEMS_READY)) {
          enterLowPowerMode();
          for (;;) {
            // calculate relative movement
            float motion = 0;
            for (byte n = 0; n < 10; n++) {
              float acc[3];
              mems.memsRead(acc);
              for (byte i = 0; i < 3; i++) {
                float m = (acc[i] - accBias[i]);
                motion += m * m;
              }
              delay(100);
            }
            Serial.println(motion);
            // check movement
            if (motion > WAKEUP_MOTION_THRESHOLD) {
              leaveLowPowerMode();
              break;
          }
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
        // load accelerometer and temperature data
        float acc[3] = {};
        int16_t temp; // device temperature (in 0.1 celcius degree)
        mems.memsRead(acc, 0, 0, &temp);
        deviceTemp = temp / 10;
        logData(PID_ACC, (int)(acc[0] * 100), (int)(acc[1] * 100), (int)(acc[2] * 100));
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
    void calibrateMEMS()
    {
        // MEMS data collected while sleeping
        accBias[0] = 0;
        accBias[1] = 0;
        accBias[2] = 0;
        int n;
        for (n = 0; n < 100; n++) {
          float acc[3] = {};
          mems.memsRead(acc);
          accBias[0] += acc[0];
          accBias[1] += acc[1];
          accBias[2] += acc[2];          
        }
        accBias[0] /= n;
        accBias[1] /= n;
        accBias[2] /= n;
    }
#endif
#if USE_MEMS
    MPU9250_ACC mems;
#endif
    byte m_state;
};

CTeleLogger logger;

void setup()
{
    // initialize hardware serial (for USB and BLE)
    Serial.begin(115200);
    Serial.println("Freematics ONE");
    delay(100);
    // perform initializations
    logger.begin();
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
    logger.transmitUDP();
    if (logger.connErrors >= MAX_CONN_ERRORS_RECONNECT) {
      logger.reconnect();
    }
#if DATASET_INTERVAL
    // wait to reach preset data rate
    unsigned int elapsed = millis() - t;
    if (elapsed < DATASET_INTERVAL) logger.sleep(DATASET_INTERVAL - elapsed);
#endif
}
