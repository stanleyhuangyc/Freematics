/******************************************************************************
* Reference sketch for a vehicle telematics data feed for Freematics Hub
* Works with Freematics ONE
* Developed by Stanley Huang <stanley@freematics.com.au>
* Distributed under BSD license
* Visit https://freematics.com/hub for information about Freematics Hub
* Visit https://freematics.com/products for hardware information
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
#include <FreematicsNetwork.h>
#include <avr/pgmspace.h>
#include "telelogger.h"
#include "config.h"

// logger states
#define STATE_OBD_READY 0x1
#define STATE_GPS_READY 0x2
#define STATE_LOCATION_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_SERVER_CONNECTED 0x20
#define STATE_WORKING 0x40

#if MEMS_MODE
float accBias[3] = {0}; // calibrated reference accelerometer data
float acc[3] = {0};
#endif
char devid[8] = DEFAULT_DEVID;
uint8_t deviceTemp = 0; // device temperature

uint32_t txCount = 0;
uint8_t connErrors = 0;
uint16_t UTC = 0;
uint32_t lastSyncTime = 0;
uint32_t lastSentTime = 0;
uint32_t lastCmdToken = 0;
uint32_t lastMotionTime = 0;
uint16_t stationaryTime[] = STATIONARY_TIME_TABLE;
uint16_t sendingIntervals[] = SENDING_INTERVAL_TABLE;
uint16_t sendingInterval = 0;

void idleTasks(int timeout);

class State {
public:
  bool check(byte flags) { return (m_state & flags) == flags; }
  void set(byte flags) { m_state |= flags; }
  void clear(byte flags) { m_state &= ~flags; }
private:
  byte m_state = 0;
};

State state;

#if NET_DEVICE == NET_WIFI
UDPClientESP8266AT net;
#elif NET_DEVICE == NET_SIM800
UDPClientSIM800 net;
#elif NET_DEVICE == NET_SIM5360
UDPClientSIM5360 net;
#else
UDPClient net; // null client
#endif

#if MEMS_MODE
MPU9250_ACC mems;
#endif

CStorageRAM cache;

COBDSPI obd;
#define sys obd

void print(const char* PROGMEM s)
{
  char c;
  while ((c = pgm_read_byte(s++))) Serial.print(c);
}

void println(const char* PROGMEM s)
{
  print(s);
  Serial.println();
}

/*******************************************************************************
  Freematics Hub client implementation
*******************************************************************************/
bool verifyChecksum(char* data)
{
  uint8_t sum = 0;
  char *s = strrchr(data, '*');
  if (!s) return false;
  for (char *p = data; p < s; p++) sum += *p;
  if (hex2uint8(s + 1) == sum) {
    *s = 0;
    return true;
  }
  return false;
}

bool notify(byte event, const char* extra = 0)
{
  cache.header(devid);
  char buf[32];
  byte len = snprintf_P(buf, sizeof(buf), PSTR("EV=%X"), (unsigned int)event);
  cache.dispatch(buf, len);
  len = snprintf_P(buf, sizeof(buf), PSTR("TS=%lu"), millis());
  cache.dispatch(buf, len);
  if (extra && *extra) {
    cache.dispatch(extra, strlen(extra));
  }
  cache.tailer();
  //Serial.println(cache.buffer());
  for (byte attempts = 0; attempts < 3; attempts++) {
    if (!net.send(cache.buffer(), cache.length())) {
      println(PSTR("Unsent"));
      delay(1000);
      continue;
    }
    if (event == EVENT_ACK) return true; // no reply for ACK
    // receive reply
    delay(1000);
    int len;
    char *data = net.receive(&len);
    if (!data) {
      // no reply yet
      Serial.print('.');
      continue;
    }
    data[len] = 0;
    // verify checksum
    if (!verifyChecksum(data)) {
      Serial.println(data);
      continue;
    }
    char pattern[16];
    snprintf_P(pattern, sizeof(pattern), PSTR("EV=%u"), event);
    if (!strstr(data, pattern)) {
      continue;
    }
    lastSyncTime = millis();
    connErrors = 0;
    // success
    return true;
  }
  return false;
}

bool login()
{
  char extra[24] = {0};
  if (state.check(STATE_OBD_READY)) {
    char buf[128];
    if (obd.getVIN(buf, sizeof(buf))) {
      snprintf_P(extra, sizeof(extra), PSTR("VIN=%s"), buf);
      print(PSTR("VIN:"));
      Serial.println(buf);
    }
  }
  // connect to telematics server
  for (byte attempts = 0; attempts < 3; attempts++) {
    print(PSTR("LOGIN("));
    Serial.print(SERVER_HOST);
    print(PSTR(")..."));
    if (!net.open(SERVER_HOST, SERVER_PORT)) {
      println(PSTR("NO"));
      delay(1000);
      continue;
    }
    byte event = connErrors ? EVENT_RECONNECT : EVENT_LOGIN; 
    // login Freematics Hub
    if (!notify(event, extra)) {
      net.close();
      println(PSTR("NO ACK"));
      delay(3000);
      continue;
    }
    Serial.println("OK");
    return true;
  }
  return false;
}

bool transmit()
{
  bool success = false;
  long sec = millis() / 1000;
  Serial.print(sec / 60);
  Serial.print(':');
  Serial.print(sec % 60);
  Serial.print(' ');
  // transmit data
  if (net.send(cache.buffer(), cache.length())) {
    connErrors = 0;
    txCount++;
    // output some stats
    Serial.print('#');
    Serial.print(txCount);
    Serial.print(' ');
    Serial.print(cache.length());
    println(PSTR(" bytes sent"));
    lastSentTime = millis();
    success = true;
  } else {
    connErrors++;
  }
  if (connErrors >= MAX_CONN_ERRORS_RECONNECT) {
    net.close();
    net.open(SERVER_HOST, SERVER_PORT);
  }
  return success;
}

bool ping()
{
  if (!net.open(SERVER_HOST, SERVER_PORT)) {
    return false;
  }
  bool success = notify(EVENT_PING);
  if (success) lastSyncTime = millis();
  return success;
}

/*******************************************************************************
  Reading and processing OBD data
*******************************************************************************/
int logOBDPID(byte pid)
{
  int value;
  if (obd.readPID(pid, value)) {
    cache.log((uint16_t)0x100 | pid, (int16_t)value);
  } else {
    value = -1;
  }
  return value;
}

void processOBD()
{
  int speed = logOBDPID(PID_SPEED);
  if (speed == -1) {
    return;
  }
  if (speed > 0) lastMotionTime = millis();
  // poll more PIDs
  const byte pids[]= {PID_RPM, PID_ENGINE_LOAD, PID_THROTTLE};
  for (byte i = 0; i < sizeof(pids) / sizeof(pids[0]); i++) {
    logOBDPID(pids[i]);
  }
  const byte pidTier2[] = {PID_INTAKE_TEMP, PID_COOLANT_TEMP, PID_BAROMETRIC, PID_AMBIENT_TEMP};
  static byte idx = 0;
  byte pid = pidTier2[idx];
  if (obd.isValidPID(pid)) {
    logOBDPID(pid);
  }
  if (++idx >= sizeof(pidTier2)) idx = 0;
}

#if ENABLE_GPS
void processGPS()
{
  static uint8_t lastGPSDay = 0;
  GPS_DATA gd = {0};
  // read parsed GPS data
  if (sys.gpsGetData(&gd) && gd.sat >= 3) {
    if (gd.date && UTC != (uint16_t)gd.time) {
      byte day = gd.date / 10000;
      cache.log(PID_GPS_TIME, gd.time);
      if (lastGPSDay != day) {
        cache.log(PID_GPS_DATE, gd.date);
        lastGPSDay = day;
      }
      cache.logCoordinate(PID_GPS_LATITUDE, gd.lat);
      cache.logCoordinate(PID_GPS_LONGITUDE, gd.lng);
      cache.log(PID_GPS_ALTITUDE, gd.alt);
      cache.log(PID_GPS_SAT_COUNT, gd.sat);
      unsigned int kph = gd.speed  * 1852 / 100000;
      cache.log(PID_GPS_SPEED, kph);
      if (kph >= 2) lastMotionTime = millis();
      cache.log(PID_GPS_HEADING, gd.heading);
      Serial.print("[GPS] ");
      Serial.print(gd.lat);
      Serial.print(' ');
      Serial.print(gd.lng);
      Serial.print(' ');
      Serial.print(gd.alt);
      Serial.print("m ");
      Serial.print(kph);
      Serial.print("km/h UTC:");
      Serial.print(gd.time);
      Serial.print(" SAT:");
      Serial.println((unsigned int)gd.sat);
      UTC = (uint16_t)gd.time;
    }
  }
}
#endif

void processLocation()
{
#if NET_DEVICE == NET_SIM5360
  GPS_LOCATION* gi = net.getLocation();
  // read parsed GPS data
  if (gi) {
      if (UTC != (uint16_t)gi->time) {
        cache.log(PID_GPS_DATE, gi->date);
        cache.log(PID_GPS_TIME, gi->time);
        cache.logCoordinate(PID_GPS_LATITUDE, gi->lat);
        cache.logCoordinate(PID_GPS_LONGITUDE, gi->lng);
        cache.log(PID_GPS_ALTITUDE, gi->alt);
        unsigned int kph = (unsigned long)gi->speed * 1852 / 100000;
        cache.log(PID_GPS_SPEED, kph);
        if (kph >= 1) lastMotionTime = millis();
        cache.log(PID_GPS_HEADING, gi->heading);
        Serial.print("[GPS] ");
        Serial.print(gi->lat);
        Serial.print(' ');
        Serial.print(gi->lng);
        Serial.print(' ');
        Serial.print(gi->alt);
        Serial.print("m ");
        Serial.print(kph);
        Serial.print("km/h UTC:");
        Serial.println(gi->time);
        UTC = (uint16_t)gi->time;
      }
  }
#endif
}

#if MEMS_MODE
void processMEMS()
{
    // load and store accelerometer
    int16_t temp = 0;
    mems.read(acc, 0, 0, &temp);
    deviceTemp = temp / 10;
    cache.log(PID_ACC, (int16_t)((acc[0] - accBias[0]) * 100), (int16_t)((acc[1] - accBias[1]) * 100), (int16_t)((acc[2] - accBias[2]) * 100));
    cache.log(PID_DEVICE_TEMP, deviceTemp);
    // calculate instant motion
    float motion = 0;
    for (byte i = 0; i < 3; i++) {
      float m = (acc[i] - accBias[i]);
      motion += m * m;
    }
    if (motion >= MOTION_THRESHOLD * MOTION_THRESHOLD) {
      lastMotionTime = millis();
    }
}

void calibrateMEMS()
{
  accBias[0] = 0;
  accBias[1] = 0;
  accBias[2] = 0;
  int n;
  for (n = 0; n < 100; n++) {
    float acc[3] = {0};
    mems.read(acc);
    accBias[0] += acc[0];
    accBias[1] += acc[1];
    accBias[2] += acc[2];
    delay(10);
  }
  accBias[0] /= n;
  accBias[1] /= n;
  accBias[2] /= n;
}
#endif

/*******************************************************************************
  Initializing all components and network
*******************************************************************************/
bool initialize()
{
  state.clear(STATE_WORKING);

  byte ver;
  for (byte n = 0; n < 10; n++) {
    if ((ver = obd.getVersion())) break;
    delay(3000);
  }
  print(PSTR("VER."));
  Serial.println((int)ver);

#if MEMS_MODE
  if (!state.check(STATE_MEMS_READY)) {
    print(PSTR("MEMS..."));
    if (mems.begin()) {
      state.set(STATE_MEMS_READY);
      println(PSTR("OK"));
    } else {
      println(PSTR("NO"));
    }
  }
#endif

  // initialize network module
  if (!state.check(STATE_NET_READY)) {
    Serial.print(net.deviceName());
    print(PSTR("..."));
    if (net.begin(&sys)) {
      println(PSTR("OK"));
      state.set(STATE_NET_READY);
    } else {
      println(PSTR("NO"));
      return false;
    }
  }

#if NET_DEVICE == NET_SIM5360
  print(PSTR("IMEI:"));
  Serial.println(net.IMEI);
  // generate device ID  
  uint32_t seed = atol(net.IMEI + 5);
  for (byte i = 0; i < 7; i++, seed >>= 5) {
    byte x = (byte)seed & 0x1f;
    if (x >= 10) {
      x = x - 10 + 'A';
      switch (x) {
        case 'B': x = 'W'; break;
        case 'D': x = 'X'; break;
        case 'I': x = 'Y'; break;
        case 'O': x = 'Z'; break;
      }
    } else {
      x += '0';
    }
    devid[i] = x;
  }
  devid[7] = 0;
#endif

  print(PSTR("DEVID:"));
  Serial.println(devid);

  // initialize OBD communication
  if (!state.check(STATE_OBD_READY)) {
    print(PSTR("OBD..."));
    if (!obd.init()) {
      println(PSTR("NO"));
    } else {
      println(PSTR("OK"));
      state.set(STATE_OBD_READY);
    }
  }

#if ENABLE_GPS
  // start serial communication with GPS receiver
  if (!state.check(STATE_GPS_READY)) {
    print(PSTR("GPS..."));
    if (sys.gpsInit(GPS_SERIAL_BAUDRATE)) {
      state.set(STATE_GPS_READY);
      println(PSTR("OK"));
    } else {
      println(PSTR("NO"));
    }
  }
#endif

#if NET_DEVICE == NET_WIFI
  for (byte attempts = 0; attempts < 3; attempts++) {
    print(PSTR("WIFI(SSID:"));
    Serial.print(WIFI_SSID);
    print(PSTR(")..."));
    if (net.setup(WIFI_SSID, WIFI_PASSWORD)) {
      println(PSTR("OK"));
      state.set(STATE_NET_READY);
      break;
    } else {
      println(PSTR("NO"));
    }
  }
  if (!state.check(STATE_NET_READY)) {
    return false;
  }
#else
  print(PSTR("CELL..."));
  if (net.setup(CELL_APN, state.check(STATE_GPS_READY) ? false : true)) {
    String op = net.getOperatorName();
    if (op.length()) {
      Serial.println(op);
    } else {
      println(PSTR("OK"));
    }
  } else {
    println(PSTR("NO"));
    return false;
  }
#endif

#if MEMS_MODE
  if (state.check(STATE_MEMS_READY)) {
    calibrateMEMS();
  }
#endif

  print(PSTR("IP..."));
  String ip = net.getIP();
  if (ip.length()) {
    Serial.println(ip);
  } else {
    println(PSTR("NO"));
  }
  int csq = net.getSignal();
  if (csq > 0) {
    print(PSTR("CSQ..."));
    Serial.print((float)csq / 10, 1);
    println(PSTR("dB"));
  }

  txCount = 0;

  if (!state.check(STATE_OBD_READY) && obd.init()) {
    state.set(STATE_OBD_READY);
  }

  login();

  lastMotionTime = millis();

  state.set(STATE_SERVER_CONNECTED | STATE_WORKING);
  return true;
}

void shutDownNet()
{
  Serial.print(net.deviceName());
  net.close();
  net.end();
  println(PSTR(" OFF"));
  state.clear(STATE_NET_READY | STATE_SERVER_CONNECTED);
}

bool waitMotion(long timeout)
{
#if MEMS_MODE
  if (state.check(STATE_MEMS_READY)) {
    unsigned long t = millis();
    while (millis() - t < timeout) {
      delay(100);
      // calculate relative movement
      float motion = 0;
      float acc[3];
      int16_t temp = 0;
      mems.read(acc, 0, 0, &temp);
      deviceTemp = temp / 10;
      for (byte i = 0; i < 3; i++) {
        float m = (acc[i] - accBias[i]);
        motion += m * m;
      }
      // check movement
      if (motion >= MOTION_THRESHOLD * MOTION_THRESHOLD) {
        lastMotionTime = millis();
        return true;
      }
    }
    return false;
  }
#endif
  delay(timeout);
  return true;
}

/*******************************************************************************
  Executing a command
*******************************************************************************/
String executeCommand(const char* cmd)
{
  String result;
  Serial.println(cmd);
  if (!strcmp_P(cmd, PSTR("REBOOT"))) {
    shutDownNet();
    void(* resetFunc) (void) = 0;
    resetFunc();
    // never reach here
  } else if (!strcmp_P(cmd, PSTR("STANDBY"))) {
    state.clear(STATE_WORKING);
    result = "OK";
  } else if (!strncmp_P(cmd, PSTR("OBD"), 3) && cmd[4]) {
    // send OBD command
    String obdcmd = cmd + 4;
    obdcmd += '\r';
    char buf[128];
    if (obd.sendCommand(obdcmd.c_str(), buf, sizeof(buf), OBD_TIMEOUT_LONG) > 0) {
      Serial.println(buf);
      for (int n = 4; buf[n]; n++) {
        switch (buf[n]) {
        case '\r':
        case '\n':
          result += ' ';
          break;
        default:
          result += buf[n];
        }
      }
    } else {
      result = "ERROR";
    }
  } else {
    return "INVALID";
  }
  return result;
}

bool processCommand(char* data)
{
  char *p;
  if (!(p = strstr_P(data, PSTR("TK=")))) return false;
  uint32_t token = atol(p + 3);
  if (!(p = strstr_P(data, PSTR("CMD=")))) return false;
  char *cmd = p + 4;

  if (token > lastCmdToken) {
    // new command
    String result = executeCommand(cmd);
    // send command response
    char buf[128];
    snprintf_P(buf, sizeof(buf), PSTR("TK=%lu,MSG=%s"), token, result.c_str());
    for (byte attempts = 0; attempts < 3; attempts++) {
      Serial.println("ACK...");
      if (notify(EVENT_ACK, buf)) {
        Serial.println("sent");
        break;
      }
    }
  } else {
    // previously executed command
    char buf[64];
    snprintf_P(buf, sizeof(buf), PSTR("TK=%lu,DUP=1"), token);
    for (byte attempts = 0; attempts < 3; attempts++) {
      Serial.println("ACK...");
      if (notify(EVENT_ACK, buf)) {
        Serial.println("sent");
        break;
      }
    }
  }
  return true;
}

/*******************************************************************************
  Collecting and processing data
*******************************************************************************/
void process()
{
  uint32_t startTime = millis();

  idleTasks(0);

  cache.header(devid);
  cache.timestamp(startTime);

  // process GPS data if connected
  if (state.check(STATE_GPS_READY)) {
#if ENABLE_GPS
    processGPS();
#endif
  } else {
    processLocation();
  }

  // process OBD data if connected
  if (state.check(STATE_OBD_READY)) {
    processOBD();
    if (obd.errors > MAX_OBD_ERRORS) {
      state.clear(STATE_WORKING);
      return;
    }
  }

#if MEMS_MODE
  // process MEMS data if available
  if (state.check(STATE_MEMS_READY)) {
    processMEMS();
  }
#endif

  // read and log car battery voltage, data in 0.01v
  float volts = obd.getVoltage();
  if (volts) {
    cache.log(PID_BATTERY_VOLTAGE, (unsigned int)(volts * 100)); /* in 0.01V */
  }
  // when battery voltage goes too low, enter standby mode
#if STANDBY_LOW_VOLTAGE
  if (volts >= 6 && volts < STANDBY_LOW_VOLTAGE) {
    print(PSTR("BATTERY:"));
    Serial.println(volts, 1);
    state.clear(STATE_WORKING);
  }
#endif

  cache.tailer();

#if 0
  Serial.print('{');
  Serial.print(cache.buffer()); 
  Serial.println('}');
#endif

#if SERVER_SYNC_INTERVAL
  // check server sync signal interval
  if (millis() - lastSyncTime > 1000L * SERVER_SYNC_INTERVAL) {
    println(PSTR("NO SYNC"));
    connErrors++;
  }
#endif

  if (millis() - lastSentTime >= sendingInterval && cache.samples() > 0) {
    transmit();
  }

  // motion controlled data sending interval
  unsigned int motionless = (millis() - lastMotionTime) / 1000;
  bool gosleep = true;
  for (byte i = 0; i < sizeof(stationaryTime) / sizeof(stationaryTime[0]); i++) {
    if (motionless < stationaryTime[i] || stationaryTime[i] == 0) {
      sendingInterval = 1000L * sendingIntervals[i];
      gosleep = false;
      break;
    }
  }
  if (gosleep) {
    state.clear(STATE_WORKING);
  } else {
    long n = startTime + sendingInterval - millis();
    if (n > 0) waitMotion(n);
  }
}

/*******************************************************************************
  Implementing stand-by mode
*******************************************************************************/
void standby()
{
  unsigned long t = millis();
#if ENABLE_GPS
  if (state.check(STATE_GPS_READY)) {
    println(PSTR("GPS OFF"));
    sys.gpsInit(0); // turn off GPS power
  }
#endif
  UTC = 0;
  state.clear(STATE_OBD_READY | STATE_GPS_READY | STATE_NET_READY | STATE_SERVER_CONNECTED);
#if MEMS_MODE
  calibrateMEMS();
  for (;;) {
    shutDownNet();
    println(PSTR("STANDBY"));
    if (waitMotion(1000L * PING_BACK_INTERVAL - (millis() - t))) {
      // to wake up
      break;
    }
    t = millis();
    cache.header(devid);
    cache.timestamp(millis());
    cache.log(PID_ACC, (int16_t)((acc[0] - accBias[0]) * 100), (int16_t)((acc[1] - accBias[1]) * 100), (int16_t)((acc[2] - accBias[2]) * 100));
    cache.log(PID_DEVICE_TEMP, deviceTemp);
    cache.log(PID_BATTERY_VOLTAGE, (uint16_t)(obd.getVoltage() * 100));
    cache.tailer();
    // start ping
    print(PSTR("Ping..."));
#if NET_DEVICE == NET_WIFI
    if (!net.setup(WIFI_SSID, WIFI_PASSWORD))
#else
    if (!net.begin(&sys) || !net.setup(CELL_APN))
#endif
    {
      println(PSTR("NO"));
      continue;
    }
    Serial.println(net.getIP());
    state.set(STATE_NET_READY);
    if (ping()) {
      state.set(STATE_SERVER_CONNECTED);
      // ping back data
      transmit();
    }
  }
#else
  do {
    delay(5000);
  } while (obd.getVoltage() < JUMPSTART_VOLTAGE);
#endif
#if RESET_AFTER_WAKEUP
  delay(100);
  void(* resetFunc) (void) = 0;
  resetFunc();
#endif  
}

/*******************************************************************************
  Tasks to perform in idle/waiting time
*******************************************************************************/
void idleTasks(int timeout)
{
  // check incoming datagram
  do {
    int len = 0;
    char *data = net.receive(&len, timeout);
    if (data) {
      data[len] = 0;
      if (!verifyChecksum(data)) {
        Serial.println(data);
        break;
      }
      char *p = strstr_P(data, PSTR("EV="));
      if (!p) break;
      int eventID = atoi(p + 3);
      switch (eventID) {
      case EVENT_COMMAND:
        processCommand(data);
        break;
      }
      lastSyncTime = millis();
    }
  } while(0);
}

void setup()
{
  Serial.begin(115200);
  obd.begin();
  // initializing components
  initialize();
}

void loop()
{
  // standby mode
  if (!state.check(STATE_WORKING)) {
    if (state.check(STATE_SERVER_CONNECTED)) {
      print(PSTR("LOGOUT.."));
      if (notify(EVENT_LOGOUT))
        print(PSTR("OK"));
      Serial.println();
      state.clear(STATE_SERVER_CONNECTED);
      connErrors = 0;
    }
    standby();
    initialize();
    return;
  }
  // deal with network errors
  if (connErrors >= MAX_CONN_ERRORS) {
    println(PSTR("Network errors"));
    state.clear(STATE_SERVER_CONNECTED);
    shutDownNet();
    initialize();
    return;
  }
  // collect and transmit data
  process();
}
