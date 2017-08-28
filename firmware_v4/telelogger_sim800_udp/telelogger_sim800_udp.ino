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

#define EVENT_LOGIN 1
#define EVENT_LOGOUT 2
#define EVENT_SYNC 3

// logger states
#define STATE_STORAGE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_CONNECTED 0x20
#define STATE_ALL_GOOD 0x40

int lastSpeed = 0;
uint32_t lastSpeedTime = 0;
uint32_t distance = 0;
uint32_t lastSentTime = 0;

typedef struct {
  float lat;
  float lng;
  uint8_t year; /* year past 2000, e.g. 15 for 2015 */
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} NET_LOCATION;

#define XBEE_BAUDRATE 115200

class CTeleClientSIM800 : public COBDSPI
{
public:
    bool netBegin();
    void netEnd();
    bool netSetup(const char* apn, unsigned int timeout = 60000);
    const char* getIP();
    int getSignal();
    const char* getOperatorName();
    bool netOpen(const char* host, uint16_t port);
    bool netSend(const char* data, unsigned int len);
    void netClose();
    char* netReceive(int* pbytes = 0, unsigned int timeout = 5000);
    bool getLocation(NET_LOCATION* loc);
    const char* netDeviceName() { return "SIM800"; }
    int getConnErrors() { return connErrors; }
protected:
    bool netSendCommand(const char* cmd, unsigned int timeout = 1000, const char* expected = "\r\nOK", bool terminated = false);
    byte checksum(const char* data, int len)
    {
      // calculate and add checksum
      byte sum = 0;
      for (int i = 0; i < len; i++) sum += data[i];
      return sum;
    }
    uint16_t feedid = 0;
    uint32_t txCount = 0;
    uint8_t connErrors = 0;
    char m_buffer[80];
    uint8_t m_stage = 0;
};

bool CTeleClientSIM800::netBegin()
{
  if (m_stage == 0) {
    xbBegin(XBEE_BAUDRATE);
    m_stage = 1;
  }
  for (byte n = 0; n < 10; n++) {
    // try turning on module
    xbTogglePower();
    sleep(3000);
    // discard any stale data
    xbPurge();
    for (byte m = 0; m < 3; m++) {
      if (netSendCommand("AT\r")) {
        m_stage = 2;
        return true;
      }
    }
  }
  return false;
}

void CTeleClientSIM800::netEnd()
{
  if (m_stage == 2) {
    xbTogglePower();
    m_stage = 1;
  }
}

bool CTeleClientSIM800::netSetup(const char* apn, unsigned int timeout)
{
  uint32_t t = millis();
  bool success = false;
  netSendCommand("ATE0\r");
  do {
    success = netSendCommand("AT+CREG?\r", 3000, "+CREG: 0,1") != 0;
    Serial.print('.');
  } while (!success && millis() - t < timeout);
  if (!success) return false;
  do {
    success = netSendCommand("AT+CGATT?\r", 3000, "+CGATT: 1");
  } while (!success && millis() - t < timeout);
  sprintf(m_buffer, "AT+CSTT=\"%s\"\r", apn);
  if (!netSendCommand(m_buffer)) {
    return false;
  }
  netSendCommand("AT+CIICR\r");
  return success;
}

const char* CTeleClientSIM800::getIP()
{
  for (uint32_t t = millis(); millis() - t < 60000; ) {
    if (netSendCommand("AT+CIFSR\r", 3000, ".")) {
      char *p;
      for (p = m_buffer; *p && !isdigit(*p); p++);
      char *q = strchr(p, '\r');
      if (q) *q = 0;
      return p;
    }
  }
  return "";
}

int CTeleClientSIM800::getSignal()
{
    if (netSendCommand("AT+CSQ\r", 500)) {
        char *p = strchr(m_buffer, ':');
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
const char* CTeleClientSIM800::getOperatorName()
{
    // display operator name
    if (netSendCommand("AT+COPS?\r") == 1) {
        char *p = strstr(m_buffer, ",\"");
        if (p) {
            p += 2;
            char *s = strchr(p, '\"');
            if (s) *s = 0;
            return p;
        }
    }
    return "";
}

bool CTeleClientSIM800::netOpen(const char* host, uint16_t port)
{
  //netSendCommand("AT+CLPORT=\"UDP\",8000\r");
  netSendCommand("AT+CIPSRIP=1\r");
  //netSendCommand("AT+CIPUDPMODE=1\r");
  sprintf(m_buffer, "AT+CIPSTART=\"UDP\",\"%s\",\"%u\"\r", host, port);
  return netSendCommand(m_buffer, 3000);
}

void CTeleClientSIM800::netClose()
{
  netSendCommand("AT+CIPCLOSE\r");
}

bool CTeleClientSIM800::netSend(const char* data, unsigned int len)
{
  sprintf(m_buffer, "AT+CIPSEND=%u\r", len);
  if (netSendCommand(m_buffer, 200, ">")) {
    xbWrite(data);
    xbWrite("\r");
    if (netSendCommand(0, 5000, "\r\nSEND OK")) {
      return true;
    }
  }
  return false;
}

char* CTeleClientSIM800::netReceive(int* pbytes, unsigned int timeout)
{
  if (netSendCommand(0, timeout, "RECV FROM:")) {
    char *p = strstr(m_buffer, "RECV FROM:");
    if (p) p = strchr(p, '\r');
    if (!p) return 0;
    while (*(++p) == '\r' || *p =='\n');
    int len = strlen(p);
    if (len > 2) {
      if (pbytes) *pbytes = len;
      return p;
    } else {
      if (netSendCommand("AT\r", 1000, "\r\nOK", true)) {
        p = m_buffer;
        while (*p && (*p == '\r' || *p == '\n')) p++;
        if (pbytes) *pbytes = strlen(p);
        return p;
      }
    }
  }
  return 0;
}

bool CTeleClientSIM800::netSendCommand(const char* cmd, unsigned int timeout, const char* expected, bool terminated)
{
  if (cmd) {
    xbWrite(cmd);
  }
  m_buffer[0] = 0;
  byte ret = xbReceive(m_buffer, sizeof(m_buffer), timeout, expected);
  if (ret) {
    if (terminated) {
      char *p = strstr(m_buffer, expected);
      if (p) *p = 0;
    }
    return true;
  } else {
    return false;
  }
}

bool CTeleClientSIM800::getLocation(NET_LOCATION* loc)
{
  if (netSendCommand("AT+CIPGSMLOC=1,1\r", 3000)) do {
    char *p;
    if (!(p = strchr(m_buffer, ':'))) break;
    if (!(p = strchr(p, ','))) break;
    loc->lng = atof(++p);
    if (!(p = strchr(p, ','))) break;
    loc->lat = atof(++p);
    if (!(p = strchr(p, ','))) break;
    loc->year = atoi(++p) - 2000;
    if (!(p = strchr(p, '/'))) break;
    loc->month = atoi(++p);
    if (!(p = strchr(p, '/'))) break;
    loc->day = atoi(++p);
    if (!(p = strchr(p, ','))) break;
    loc->hour = atoi(++p);
    if (!(p = strchr(p, ':'))) break;
    loc->minute = atoi(++p);
    if (!(p = strchr(p, ':'))) break;
    loc->second = atoi(++p);
    return true;
  } while(0);
  return false;
}

class CStorageRAM {
public:
    bool init(unsigned int cacheSize)
    {
      if (m_cacheSize != cacheSize) {
        uninit();
        m_cache = new char[m_cacheSize = cacheSize];
      }
      return true;
    }
    void uninit()
    {
        if (m_cache) {
            delete m_cache;
            m_cache = 0;
            m_cacheSize = 0;
        }
    }
    void purge() { m_cacheBytes = 0; m_samples = 0; }
    unsigned int length() { return m_cacheBytes; }
    char* buffer() { return m_cache; }
    void dispatch(const char* buf, byte len)
    {
        // reserve some space for checksum
        int remain = m_cacheSize - m_cacheBytes - len - 3;
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

    void header(uint16_t feedid)
    {
        m_cacheBytes = sprintf(m_cache, "%X#", (unsigned int)feedid);
    }
    void tailer()
    {
        if (m_cache[m_cacheBytes - 1] == ',') m_cacheBytes--;
        m_cacheBytes += sprintf(m_cache + m_cacheBytes, "*%X", (unsigned int)checksum(m_cache, m_cacheBytes));
    }
    virtual void log(uint16_t pid, int16_t value)
    {
        char buf[16];
        byte len = sprintf_P(buf, PSTR("%X=%d"), pid, value);
        dispatch(buf, len);
    }
    virtual void log(uint16_t pid, int32_t value)
    {
        char buf[20];
        byte len = sprintf_P(buf, PSTR("%X=%ld"), pid, value);
        dispatch(buf, len);
    }
    virtual void log(uint16_t pid, uint32_t value)
    {
        char buf[20];
        byte len = sprintf_P(buf, PSTR("%X=%lu"), pid, value);
        dispatch(buf, len);
    }
    virtual void log(uint16_t pid, int value1, int value2, int value3)
    {
        char buf[24];
        byte len = sprintf_P(buf, PSTR("%X=%d;%d;%d"), pid, value1, value2, value3);
        dispatch(buf, len);
    }
    virtual void logCoordinate(uint16_t pid, int32_t value)
    {
        char buf[24];
        byte len = sprintf_P(buf, PSTR("%X=%d.%06lu"), pid, (int)(value / 1000000), abs(value) % 1000000);
        dispatch(buf, len);
    }
    virtual void timestamp(uint32_t ts)
    {
        m_dataTime = ts;
    }
    virtual uint16_t samples() { return m_samples; }
protected:
    byte checksum(const char* data, int len)
    {
        byte sum = 0;
        for (int i = 0; i < len; i++) sum += data[i];
        return sum;
    }
    uint32_t m_dataTime = 0;
    uint16_t m_samples = 0;
    unsigned int m_cacheSize = 0;
    unsigned int m_cacheBytes = 0;
    char* m_cache = 0;
};

class CTeleLogger : public CTeleClientSIM800
{
public:
  bool setup()
  {
    clearState(STATE_ALL_GOOD);
    distance = 0;

    // initialize OBD communication
    if (!checkState(STATE_OBD_READY)) {
      Serial.print("OBD...");
      if (!init()) {
        Serial.println("NO");
        return false;
      }
      Serial.println("OK");
      setState(STATE_OBD_READY);
    }

    // start serial communication with GPS receiver
    if (!checkState(STATE_GPS_READY)) {
      Serial.print("GPS...");
      if (gpsInit(GPS_SERIAL_BAUDRATE)) {
        setState(STATE_GPS_READY);
        Serial.println("OK");
      } else {
        Serial.println("NO");
      }
    }

    if (!checkState(STATE_MEMS_READY)) {
      Serial.print("MEMS...");
      if (mems.memsInit()) {
        setState(STATE_MEMS_READY);
        Serial.println("OK");
      } else {
        Serial.println("NO");
      }
    }

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
      int csq = getSignal();
      if (csq > 0) {
        Serial.print("CSQ...");
        Serial.print((float)csq / 10, 1);
        Serial.println("dB");
      }
      return false;
    }

    Serial.print("IP:");
    Serial.println(getIP());

    txCount = 0;

    if (!checkState(STATE_STORAGE_READY)) {
      cache.init(RAM_CACHE_SIZE);
      setState(STATE_STORAGE_READY);
    }

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
    if (cache.length() == 0) {
      cache.header(feedid);
    }
    cache.timestamp(millis());

    // process GPS data if connected
    if (checkState(STATE_GPS_READY)) {
      processGPS();
    }

    // process OBD data if connected
    if (checkState(STATE_OBD_READY)) {
      processOBD();
    }

    if (checkState(STATE_MEMS_READY)) {
      processMEMS();
    }

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

    // read and log car battery voltage , data in 0.01v
    {
      int v = getVoltage() * 100;
      cache.log(PID_BATTERY_VOLTAGE, v);
    }

    uint32_t t = millis();
    if (t - lastSentTime >= DATA_SENDING_INTERVAL && cache.samples() > 1) {
      //Serial.println(cache.getBuffer()); // print the content to be sent
      Serial.print('[');
      Serial.print(txCount);
      Serial.print("] ");
      cache.tailer();
      // transmit data
      if (transmit(cache.buffer(), cache.length())) {
        // output some stats
        Serial.print(cache.length());
        Serial.println("B sent");
        cache.purge();
      } else {
        Serial.println("Unsent");
      }
      if (getConnErrors() >= MAX_CONN_ERRORS_RECONNECT) {
        netClose();
        netOpen(SERVER_HOST, SERVER_PORT);
      }
      lastSentTime = t;

      if ((txCount % SERVER_SYNC_INTERVAL) == (SERVER_SYNC_INTERVAL - 1)) {
        // sync
        Serial.print("SYNC...");
        if (!notifyServer(EVENT_SYNC)) {
          connErrors++;
          Serial.println("NO");
        } else {
          connErrors = 0;
          txCount++;
          Serial.println("OK");
        }
      }
    }
  }
  bool login()
  {
    for (byte attempts = 0; attempts < 5; attempts++) {
      Serial.print("LOGIN...");
      if (!netOpen(SERVER_HOST, SERVER_PORT) || !notifyServer(EVENT_LOGIN)) {
        Serial.println("NO");
        netClose();
        sleep(1000);
        continue;
      }
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
  bool transmit(const char* data, int bytes)
  {
    // transmit data
    if (data[bytes - 1] == ',') bytes--;
    if (!netSend(data, bytes)) {
      connErrors++;
      return false;
    } else {
      connErrors = 0;
      txCount++;
      return true;
    }
  }
  bool notifyServer(byte event)
  {
    for (byte attempts = 0; attempts < 3; attempts++) {
      cache.header(feedid);
      char buf[32];
      char eventToken[8];
      byte n = sprintf_P(eventToken, PSTR("EV=%u"), (unsigned int)event);
      cache.dispatch(eventToken, n);
      n = sprintf_P(buf, PSTR("TS=%lu"), millis());
      cache.dispatch(buf, n);
      n = sprintf_P(buf, PSTR("VIN=%s"), DEVICE_ID);
      cache.dispatch(buf, n);
      cache.tailer();
      if (!netSend(cache.buffer(), cache.length())) {
        Serial.println("Unsent");
        continue;
      }
      // receive reply
      int len;
      char *data = netReceive(&len);
      if (!data) {
        //Serial.println("No reply");
        continue;
      }
      connErrors = 0;
      // verify checksum
      if (!verifyChecksum(data)) {
        Serial.println(data);
        continue;
      }
      if (!strstr(data, eventToken)) {
        //Serial.println("Invalid reply");
        continue;
      }
      if (event == EVENT_LOGIN) {
        feedid = hex2uint16(data);
      }
      return true;
    }
    return false;
  }
  void shutDownNet()
  {
    netClose();
    netEnd();
    clearState(STATE_NET_READY);
    Serial.print(netDeviceName());
    Serial.println(" OFF");
  }
  void standby()
  {
      if (checkState(STATE_NET_READY)) {
        if (checkState(STATE_CONNECTED)) {
          notifyServer(EVENT_LOGOUT);
        }
        shutDownNet();
      }
      cache.uninit();
      clearState(STATE_STORAGE_READY);
      if (errors > MAX_OBD_ERRORS) {
        // inaccessible OBD treated as end of trip
        feedid = 0;
      }
      if (checkState(STATE_GPS_READY)) {
        gpsInit(0); // turn off GPS power
      }
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
            }
        }
    }
    void processMEMS()
    {
        float acc[3];
        int16_t temp;
        mems.memsRead(acc, 0, 0, &temp);
        cache.log(PID_ACC, (int16_t)(acc[0] * 100), (int16_t)(acc[1] * 100), (int16_t)(acc[2] * 100));
        cache.log(PID_DEVICE_TEMP, temp / 10);
    }
    MPU9250_ACC mems;
    CStorageRAM cache;
    byte m_state = 0;
};

CTeleLogger logger;

void setup()
{
    delay(1000);
    // initialize USB serial
    Serial.begin(115200);
    Serial.println("Freematics ONE");
    logger.begin();
    logger.setup();
}

void loop()
{
    // error handling
    if (!logger.checkState(STATE_ALL_GOOD)) {
      do {
        logger.standby();
      } while (!logger.setup());
    }
    if (logger.getConnErrors() >= MAX_CONN_ERRORS) {
      logger.shutDownNet();
      logger.setup();
      return;
    }
    if (logger.errors > MAX_OBD_ERRORS) {
      logger.reset();
      logger.clearState(STATE_OBD_READY | STATE_GPS_READY);
      logger.setup();
      return;
    }

    // collect and transmit data
    logger.loop();
}
