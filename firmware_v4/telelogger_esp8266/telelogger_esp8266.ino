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
#include "config.h"

// logger states
#define STATE_STORAGE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_CONNECTED 0x20
#define STATE_ALL_GOOD 0x40
#define STATE_STANDBY 0x80

float accBias[3] = {0}; // calibrated reference accelerometer data
char vin[18] = DEFAULT_VIN;
int lastSpeed = 0;
uint32_t lastSpeedTime = 0;
uint32_t distance = 0;
uint32_t lastSentTime = 0;
uint32_t lastSyncTime = 0;
uint8_t deviceTemp = 0; // device temperature
uint32_t syncInterval = SERVER_SYNC_INTERVAL * 1000;

uint16_t feedid = 0;
uint32_t txCount = 0;
uint8_t connErrors = 0;

uint32_t lastCmdToken = 0;

void idleTasks();

class State {
public:
  bool check(byte flags) { return (m_state & flags) == flags; }
  void set(byte flags) { m_state |= flags; }
  void clear(byte flags) { m_state &= ~flags; }
private:
  byte m_state = 0;
};

State state;

UDPClientESP8266AT net;

MPU9250_ACC mems;

CStorageRAM cache;

COBDSPI obd;
#define sys obd

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

bool notifyServer(byte event, const char* serverKey, const char* payload)
{
  cache.header(feedid);
  char buf[16];
  byte len = sprintf_P(buf, PSTR("EV=%X"), (unsigned int)event);
  cache.dispatch(buf, len);
  len = sprintf_P(buf, PSTR("TS=%lu"), millis());
  cache.dispatch(buf, len);
  /*
  if (serverKey) {
    len = sprintf_P(buf, PSTR("SK=%s"), serverKey);
    cache.dispatch(buf, len);
  }
  */
  if (payload) {
    cache.dispatch(payload, strlen(payload));
  }
  cache.tailer();
  //Serial.println(cache.buffer());
  for (byte attempts = 0; attempts < 3; attempts++) {
    if (!net.send(cache.buffer(), cache.length())) {
      Serial.print('.');
      delay(1000);
      continue;
    }
    if (event == EVENT_ACK) return true; // no reply for ACK
    // receive reply
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
    sprintf(pattern, "EV=%u", event);
    if (!strstr(data, pattern)) {
      Serial.print("Invalid reply");
      continue;
    }
    if (event == EVENT_LOGIN) {
      // extract info from server response
      feedid = hex2uint16(data);
    }
    Serial.print(' ');
    connErrors = 0;
    // success
    return true;
  }
  return false;
}

bool login()
{
  // connect to telematics server
  for (byte attempts = 0; attempts < 3; attempts++) {
    Serial.print("LOGIN...");
    if (!net.open(SERVER_HOST, SERVER_PORT)) {
      Serial.println("NO");
      continue;
    }

    char *buf = net.buffer; /* re-use static buffer, for saving SRAM */
    sprintf(buf, "VIN=%s", vin);

    // login Freematics Hub
    if (!notifyServer(EVENT_LOGIN, SERVER_KEY, buf)) {
      net.close();
      Serial.println("NO");
      continue;
    } else {
      Serial.println("OK");
    }

    Serial.print("FEED ID:");
    Serial.println(feedid);

    return true;
  }
  return false;
}

void transmit()
{
  cache.tailer();
  //Serial.println(cache.buffer()); // print the content to be sent
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
    Serial.print("Packet#");
    Serial.print(txCount);
    Serial.print(' ');
    Serial.print(cache.length());
    Serial.println(" bytes");
    lastSentTime = millis();
  } else {
    connErrors++;
  }
  // purge cache and place a header
  cache.header(feedid);
  if (connErrors >= MAX_CONN_ERRORS_RECONNECT) {
    net.close();
    net.open(SERVER_HOST, SERVER_PORT);
  }
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
  // calculate distance for speed
  uint32_t t = millis();
  distance += (speed + lastSpeed) * (t - lastSpeedTime) / 3600 / 2;
  lastSpeedTime = t;
  lastSpeed = speed;
  cache.log(PID_TRIP_DISTANCE, distance);
  // poll more PIDs
  const byte pids[]= {PID_RPM, PID_ENGINE_LOAD, PID_THROTTLE};
  for (byte i = 0; i < sizeof(pids) / sizeof(pids[0]); i++) {
    logOBDPID(pids[i]);
  }
  static byte count = 0;
  if ((count++ % 50) == 0) {
    const byte pidTier2[] = {PID_INTAKE_TEMP, PID_COOLANT_TEMP, PID_BAROMETRIC, PID_AMBIENT_TEMP, PID_ENGINE_FUEL_RATE};
    byte pid = pidTier2[count / 50];
    if (obd.isValidPID(pid)) {
      logOBDPID(pid);
    }
  }
}

#if ENABLE_GPS
void processGPS()
{
  static uint16_t lastUTC = 0;
  static uint8_t lastGPSDay = 0;
  GPS_DATA gd = {0};
  // read parsed GPS data
  if (sys.gpsGetData(&gd)) {
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
      }
  }
}
#endif

void processMEMS()
{
  // load and store accelerometer
  float acc[3];
  int16_t temp;
  mems.read(acc, 0, 0, &temp);
  deviceTemp = temp / 10;
  cache.log(PID_ACC, (int16_t)((acc[0] - accBias[0]) * 100), (int16_t)((acc[1] - accBias[1]) * 100), (int16_t)((acc[2] - accBias[2]) * 100));
}

void calibrateMEMS()
{
  Serial.print("ACC BIAS...");
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
  Serial.print(accBias[0]);
  Serial.print('/');
  Serial.print(accBias[1]);
  Serial.print('/');
  Serial.println(accBias[2]);
}

/*******************************************************************************
  Initializing all components and network
*******************************************************************************/
bool initialize()
{
  state.clear(STATE_ALL_GOOD);
  distance = 0;

  if (!state.check(STATE_MEMS_READY)) {
    Serial.print("MEMS...");
    if (mems.begin()) {
      state.set(STATE_MEMS_READY);
      Serial.println("OK");
    } else {
      Serial.println("NO");
    }
  }

  // initialize OBD communication
  if (!state.check(STATE_OBD_READY)) {
    obd.begin();
    Serial.print("OBD...");
    if (!obd.init()) {
      Serial.println("NO");
      return false;
    }
    Serial.println("OK");
    state.set(STATE_OBD_READY);

    char buf[128];
    Serial.print("VIN:");
    if (obd.getVIN(buf, sizeof(buf))) {
      strncpy(vin, buf, sizeof(vin) - 1);
      Serial.print(vin);
    }
    Serial.println();
  }

#ifdef ENABLE_GPS
  // start serial communication with GPS receiver
  if (!state.check(STATE_GPS_READY)) {
    Serial.print("GPS...");
    if (sys.gpsInit(GPS_SERIAL_BAUDRATE)) {
      state.set(STATE_GPS_READY);
      Serial.println("OK");
    } else {
      Serial.println("NO");
    }
  }
#endif

  // initialize network module
  if (!state.check(STATE_NET_READY)) {
    Serial.print(net.deviceName());
    Serial.print("...");
    if (net.begin(&sys)) {
      Serial.println("OK");
      state.set(STATE_NET_READY);
    } else {
      Serial.println("NO");
      return false;
    }
  }
  for (byte attempts = 0; attempts < 3; attempts++) {
    Serial.print("WIFI(SSID:");
    Serial.print(WIFI_SSID);
    Serial.print(")...");
    if (net.setup(WIFI_SSID, WIFI_PASSWORD)) {
      Serial.println("OK");
      state.set(STATE_NET_READY);
      break;
    } else {
      Serial.println("NO");
    }
  }
  if (!state.check(STATE_NET_READY)) {
    return false;
  }

  if (state.check(STATE_MEMS_READY)) {
    calibrateMEMS();
  }

  Serial.print("IP...");
  String ip = net.getIP();
  if (ip.length()) {
    Serial.println(ip);
  } else {
    Serial.println("NO");
  }
  int csq = net.getSignal();
  if (csq > 0) {
    Serial.print("CSQ...");
    Serial.print((float)csq / 10, 1);
    Serial.println("dB");
  }

  txCount = 0;
  if (!login()) {
    return false;
  }
  state.set(STATE_CONNECTED);

  lastSyncTime = millis();

  state.set(STATE_ALL_GOOD);
  return true;
}

/*******************************************************************************
  Executing a command
*******************************************************************************/
String executeCommand(const char* cmd)
{
  String result;
  Serial.println(cmd);
  if (!strcmp(cmd, "REBOOT")) {
    // never reach here
  } else if (!strcmp(cmd, "STANDBY")) {
    state.clear(STATE_ALL_GOOD);
    result = "OK";
  } else if (!strcmp(cmd, "WAKEUP")) {
    state.clear(STATE_STANDBY);
    result = "OK";
  } else if (!strncmp(cmd, "OBD", 3) && cmd[4]) {
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
  if (!(p = strstr(data, "TK="))) return false;
  uint32_t token = atol(p + 3);
  if (!(p = strstr(data, "CMD="))) return false;
  char *cmd = p + 4;

  if (token > lastCmdToken) {
    // new command
    String result = executeCommand(cmd);
    // send command response
    char buf[256];
    snprintf(buf, sizeof(buf), "TK=%lu,MSG=%s", token, result.c_str());
    for (byte attempts = 0; attempts < 3; attempts++) {
      Serial.println("ACK...");
      if (notifyServer(EVENT_ACK, SERVER_KEY, buf)) {
        Serial.println("sent");
        break;
      }
    }
  } else {
    // previously executed command
    char buf[64];
    snprintf(buf, sizeof(buf), "TK=%lu,DUP=1", token);
    for (byte attempts = 0; attempts < 3; attempts++) {
      Serial.println("ACK...");
      if (notifyServer(EVENT_ACK, SERVER_KEY, buf)) {
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
  cache.header(feedid);
  cache.timestamp(millis());

  // process OBD data if connected
  if (state.check(STATE_OBD_READY)) {
    processOBD();
  }

  // process MEMS data if available
  if (state.check(STATE_MEMS_READY)) {
    processMEMS();
  }

  // read and log car battery voltage, data in 0.01v
  int v = obd.getVoltage() * 100;
  cache.log(PID_BATTERY_VOLTAGE, v);

  if (syncInterval > 10000 && millis() - lastSyncTime > syncInterval) {
    Serial.println("NO SYNC");
    connErrors++;
  } else if (cache.samples() > 0) {
    // start data chunk
    transmit();
  }

  idleTasks();

  if (obd.errors > MAX_OBD_ERRORS) {
    obd.reset();
    state.clear(STATE_OBD_READY | STATE_ALL_GOOD);
  }
}

/*******************************************************************************
  Implementing stand-by mode
*******************************************************************************/
void shutDownNet()
{
  Serial.print(net.deviceName());
  net.close();
  net.end();
  state.clear(STATE_NET_READY);
  Serial.println(" OFF");
}

void standby()
{
  if (state.check(STATE_NET_READY)) {
    if (state.check(STATE_CONNECTED)) {
      notifyServer(EVENT_LOGOUT, SERVER_KEY, 0);
    }
  }
  shutDownNet();
#if ENABLE_GPS
  if (state.check(STATE_GPS_READY)) {
    Serial.println("GPS OFF");
    sys.gpsInit(0); // turn off GPS power
  }
#endif
  if (obd.errors > MAX_OBD_ERRORS) {
    // inaccessible OBD treated as end of trip
    feedid = 0;
  }
  obd.reset();
  state.clear(STATE_OBD_READY | STATE_GPS_READY | STATE_NET_READY | STATE_CONNECTED);
  state.set(STATE_STANDBY);
  Serial.println("Standby");
  if (state.check(STATE_MEMS_READY)) {
    calibrateMEMS();
    while (state.check(STATE_STANDBY)) {
      delay(100);
      // calculate relative movement
      float motion = 0;
      float acc[3];
      mems.read(acc);
      for (byte i = 0; i < 3; i++) {
        float m = (acc[i] - accBias[i]);
        motion += m * m;
      }
      //Serial.println(motion);
      // check movement
      if (motion > WAKEUP_MOTION_THRESHOLD * WAKEUP_MOTION_THRESHOLD) {
        break;
      }
    }
  }
  state.clear(STATE_STANDBY);
  Serial.println("Wakeup");
}

/*******************************************************************************
  Tasks to perform in idle/waiting time
*******************************************************************************/
void idleTasks()
{
  // check incoming datagram
  do {
    int len = 0;
    char *data = net.receive(&len, 0);
    if (data) {
      data[len] = 0;
      if (!verifyChecksum(data)) {
        Serial.println(data);
        break;
      }
      char *p = strstr(data, "EV=");
      if (!p) break;
      int eventID = atoi(p + 3);
      switch (eventID) {
      case EVENT_COMMAND:
        processCommand(data);
        break;
      case EVENT_SYNC:
        lastSyncTime = millis();
        break;
      }
    }
  } while(0);
#if ENABLE_GPS
  // process GPS data if connected
  if (state.check(STATE_GPS_READY)) {
    processGPS();
  }
#endif
}

void setup()
{
  Serial.begin(115200);
  delay(1000);
  // initializing components
  initialize();
}

void loop()
{
  // error handling
  if (!state.check(STATE_ALL_GOOD)) {
    standby();
    for (byte n = 0; n < 3; n++) {
      initialize();
      if (state.check(STATE_ALL_GOOD)) break;
      delay(3000);
    }
    return;
  }

  if (connErrors >= MAX_CONN_ERRORS) {
    shutDownNet();
    initialize();
    return;
  }
  // collect and transmit data
  process();
}
