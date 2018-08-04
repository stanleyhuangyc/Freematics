/******************************************************************************
* Reference sketch for a vehicle telematics data feed for Freematics Hub
* Works with Freematics ONE+
* Developed by Stanley Huang <stanley@freematics.com.au>
* Distributed under BSD license
* Visit https://freematics.com/products for hardware information
* Visit https://freematics.com/hub for information about Freematics Hub
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
#include <httpd.h>
#include "config.h"
#include "telelogger.h"
#include "teleclient.h"
#if ENABLE_OLED
#include "FreematicsOLED.h"
#endif

// logger states
#define STATE_STORAGE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_CONNECTED 0x20
#define STATE_WORKING 0x40
#define STATE_STANDBY 0x80

typedef struct {
  byte pid;
  byte tier;
  int16_t value;
  uint32_t ts;
} PID_POLLING_INFO;

PID_POLLING_INFO obdData[]= {
  {PID_SPEED, 1},
  {PID_RPM, 1},
  {PID_THROTTLE, 1},
  {PID_ENGINE_LOAD, 1},
  {PID_FUEL_PRESSURE, 2},
  {PID_TIMING_ADVANCE, 2},
  {PID_COOLANT_TEMP, 3},
  {PID_INTAKE_TEMP, 3},
};

#if MEMS_MODE
float accBias[3] = {0}; // calibrated reference accelerometer data
float acc[3] = {0};
uint32_t lastMotionTime = 0;
#endif

// live data
char vin[18] = {0};
uint16_t dtc[6] = {0};
int16_t batteryVoltage = 0;
GPS_DATA gd = {0};
char isoTime[24];
uint32_t lastGPSDataTime = 0;
uint8_t deviceTemp = 0; // device temperature

// stats data
int lastSpeed = 0;
uint32_t lastSpeedTime = 0;
uint32_t distance = 0;
uint32_t timeoutsOBD = 0;
uint32_t timeoutsNet = 0;

uint32_t sendingInterval = DATA_SENDING_INTERVAL;
uint32_t syncInterval = SERVER_SYNC_INTERVAL * 1000;
uint8_t connErrors = 0;

int fileid = 0;
static uint8_t lastSizeKB = 0;

uint32_t lastCmdToken = 0;
String serialCommand;

byte ledMode = 0;

void idleTasks();
bool serverSetup(IPAddress& ip);
void serverProcess(int timeout);
String executeCommand(const char* cmd);
bool processCommand(char* data);

class State {
public:
  bool check(byte flags) { return (m_state & flags) == flags; }
  void set(byte flags) { m_state |= flags; }
  void clear(byte flags) { m_state &= ~flags; }
private:
  byte m_state = 0;
};

FreematicsESP32 sys;
State state;

#if MEMS_MODE == MEMS_ACC
MPU9250_ACC mems;
#elif MEMS_MODE == MEMS_9DOF
MPU9250_9DOF mems;
#elif MEMS_MODE == MEMS_DMP
MPU9250_DMP mems;
#endif

CStorageRAM cache;
#if STORAGE == STORAGE_SPIFFS
SPIFFSLogger store;
#elif STORAGE == STORAGE_SD
SDLogger store;
#endif

#if SERVER_PROTOCOL == PROTOCOL_UDP
TeleClientUDP teleClient;
#else
TeleClientHTTP teleClient;
#endif

COBD* obd = 0;

#if ENABLE_OLED
OLED_SH1106 oled;
#endif

void printTimeoutStats()
{
  Serial.print("Timeouts: OBD:");
  Serial.print(timeoutsOBD);
  Serial.print(" Network:");
  Serial.println(timeoutsNet);
}

#if LOG_EXT_SENSORS
void processExtInputs()
{
  int pins[] = {PIN_SENSOR1, PIN_SENSOR2};
  int pids[] = {PID_EXT_SENSOR1, PID_EXT_SENSOR2};
#if LOG_EXT_SENSORS == 1
  for (int i = 0; i < 2; i++) {
    cache.log(pids[i], digitalRead(pins[i]));
  }
#elif LOG_EXT_SENSORS == 2
  for (int i = 0; i < 2; i++) {
    cache.log(pids[i], analogRead(pins[i]));
  }
#endif
}
#endif

/*******************************************************************************
  HTTP API
*******************************************************************************/
#if ENABLE_HTTPD
int handlerLiveData(UrlHandlerParam* param)
{
    char *buf = param->pucBuffer;
    int bufsize = param->bufSize;
    int n = snprintf(buf, bufsize, "{\"obd\":{\"vin\":\"%s\",\"battery\":%d,\"pid\":[", vin, (int)batteryVoltage);
    uint32_t t = millis();
    for (int i = 0; i < sizeof(obdData) / sizeof(obdData[0]); i++) {
        n += snprintf(buf + n, bufsize - n, "{\"pid\":%u,\"value\":%d,\"age\":%u},",
            0x100 | obdData[i].pid, obdData[i].value, t - obdData[i].ts);
    }
    n--;
    n += snprintf(buf + n, bufsize - n, "]}");
#if MEMS_MODE
    n += snprintf(buf + n, bufsize - n, ",\"mems\":{\"acc\":[%d,%d,%d],\"stationary\":%u}",
        (int)((acc[0] - accBias[0]) * 100), (int)((acc[1] - accBias[1]) * 100), (int)((acc[2] - accBias[2]) * 100),
        millis() - lastMotionTime);
#endif
#if ENABLE_GPS
    if (lastGPSDataTime) {
      n += snprintf(buf + n, bufsize - n, ",\"gps\":{\"utc\":\"%s\",\"lat\":%d,\"lng\":%d,\"alt\":%d,\"speed\":%u,\"sat\":%d,\"age\":%u}",
          isoTime, (int)gd.lat, (int)gd.lng, (int)gd.alt, (unsigned int)gd.speed, (int)gd.sat, (unsigned int)(millis() - lastGPSDataTime));
    }
#endif
    buf[n++] = '}';
    param->contentLength = n;
    param->contentType=HTTPFILETYPE_JSON;
    return FLAG_DATA_RAW;
}

int handlerControl(UrlHandlerParam* param)
{
    char *cmd = mwGetVarValue(param->pxVars, "cmd", 0);
    if (!cmd) return 0;
    String result = executeCommand(cmd);
    param->contentLength = snprintf(param->pucBuffer, param->bufSize,
        "{\"result\":\"%s\"}", result.c_str());
    param->contentType=HTTPFILETYPE_JSON;
    return FLAG_DATA_RAW;
}
#endif

/*******************************************************************************
  Reading and processing OBD data
*******************************************************************************/
#if ENABLE_OBD
void processOBD()
{
  static int idx[2] = {0, 0};
  int tier = 1;
  uint32_t t = millis();
  for (byte i = 0; i < sizeof(obdData) / sizeof(obdData[0]); i++) {
    if (obdData[i].tier > tier) {
        // reset previous tier index
        idx[tier - 2] = 0;
        // keep new tier number
        tier = obdData[i].tier;
        // move up current tier index
        i += idx[tier - 2]++;
        // check if into next tier
        if (obdData[i].tier != tier) {
            idx[tier - 2]= 0;
            i--;
            continue;
        }
    }
    byte pid = obdData[i].pid;
    if (!obd->isValidPID(pid)) continue;
    int value;
    if (obd->readPID(pid, value)) {
        obdData[i].ts = millis();
        cache.log((uint16_t)pid | 0x100, value);
    } else {
        timeoutsOBD++;
        printTimeoutStats();
    }
    if (tier > 1) break;
  }

  // calculate distance for speed
  int speed = obdData[0].value;
  distance += (speed + lastSpeed) * (t - lastSpeedTime) / 3600 / 2;
  lastSpeedTime = t;
  lastSpeed = speed;
  cache.log(PID_TRIP_DISTANCE, distance);
}
#endif

#if ENABLE_GPS
void processGPS()
{
  static uint8_t lastGPSDay = 0;
  // read parsed GPS data
  if (!sys.gpsGetData(&gd) || gd.date == 0) {
    return;
  }
  lastGPSDataTime = millis();
  byte day = gd.date / 10000;
  cache.log(PID_GPS_TIME, gd.time);
  if (lastGPSDay != day) {
    cache.log(PID_GPS_DATE, gd.date);
    lastGPSDay = day;
  }
  cache.logCoordinate(PID_GPS_LATITUDE, gd.lat);
  cache.logCoordinate(PID_GPS_LONGITUDE, gd.lng);
  cache.log(PID_GPS_ALTITUDE, gd.alt / 100); /* m */
  float kph = (float)gd.speed * 1852 / 100000;
  cache.log(PID_GPS_SPEED, kph);
  cache.log(PID_GPS_SAT_COUNT, gd.sat);
  
  sprintf(isoTime, "%04u-%02u-%02uT%02u:%02u:%02u.%01uZ",
      (unsigned int)(gd.date % 100) + 2000, (unsigned int)(gd.date / 100) % 100, (unsigned int)(gd.date / 10000),
      (unsigned int)(gd.time / 1000000), (unsigned int)(gd.time % 1000000) / 10000, (unsigned int)(gd.time % 10000) / 100, ((unsigned int)gd.time % 100) / 10);
  
  Serial.print("GPS SATS:");
  Serial.print(gd.sat);
  //Serial.print(" ERR:");
  //Serial.print(gd.errors);
  Serial.print(" UTC:");
  Serial.println(isoTime);
}
#endif

#if MEMS_MODE
void processMEMS()
{
    // load and store accelerometer
    int16_t temp;
#if ENABLE_ORIENTATION
    ORIENTATION ori;
    float gyr[3];
    float mag[3];
    mems.read(acc, gyr, mag, &temp, &ori);
    cache.log(PID_ORIENTATION, (int16_t)(ori.yaw * 100), (int16_t)(ori.pitch * 100), (int16_t)(ori.roll * 100));
#else
    mems.read(acc, 0, 0, &temp);
#endif
    deviceTemp = temp / 10;
    cache.log(PID_ACC, (int16_t)((acc[0] - accBias[0]) * 100), (int16_t)((acc[1] - accBias[1]) * 100), (int16_t)((acc[2] - accBias[2]) * 100));
    // calculate instant motion
    float motion = 0;
    for (byte i = 0; i < 3; i++) {
      float m = (acc[i] - accBias[i]);
      motion += m * m;
    }
    if (motion > MOTION_THRESHOLD * MOTION_THRESHOLD) {
      lastMotionTime = millis();
    }
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
#endif

/*******************************************************************************
  Initializing all components and network
*******************************************************************************/
bool initialize()
{
  state.clear(STATE_WORKING);
  distance = 0;

#if MEMS_MODE
  if (!state.check(STATE_MEMS_READY)) {
    Serial.print("MEMS...");
    byte ret = mems.begin(ENABLE_ORIENTATION);
    if (ret) {
      state.set(STATE_MEMS_READY);
      if (ret == 2) Serial.print("9-DOF ");
      Serial.println("OK");
#if ENABLE_OLED
      oled.print("MEMS:");
      oled.println(ret == 2 ? "9-DOF" : "6-DOF");
#endif
      calibrateMEMS();
    } else {
      Serial.println("NO");
    }
  }
#endif

#if ENABLE_OBD
  // initialize OBD communication
  if (!state.check(STATE_OBD_READY)) {
    timeoutsOBD = 0;
    Serial.print("OBD...");
    if (obd->init()) {
      Serial.println("OK");
      state.set(STATE_OBD_READY);
      char buf[192];
      if (obd->getVIN(buf, sizeof(buf))) {
        strncpy(vin, buf, sizeof(vin) - 1);
        Serial.print("VIN:");
        Serial.println(vin);
      }
      int dtcCount = obd->readDTC(dtc, sizeof(dtc) / sizeof(dtc[0]));
      if (dtcCount > 0) {
        Serial.print("DTC:");
        Serial.println(dtcCount);
      }
    } else {
      Serial.println("NO");
    }
  }
#endif

#ifdef ENABLE_GPS
  // start serial communication with GPS receiver
  if (!state.check(STATE_GPS_READY)) {
    Serial.print("GPS...");
    memset(&gd, 0, sizeof(gd));
    if (sys.gpsInit(GPS_SERIAL_BAUDRATE)) {
      state.set(STATE_GPS_READY);
      Serial.println("OK");
#if ENABLE_OLED
      oled.println("GPS OK");
#endif
    } else {
      Serial.println("NO");
    }
  }
#endif

#if NET_DEVICE == NET_WIFI
#if ENABLE_OLED
  oled.print("Connecting WiFi...");
#endif

  for (byte attempts = 0; attempts < 10; attempts++) {
    if (!teleClient.net.begin()) continue;
    Serial.print("WiFi(SSID:");
    Serial.print(WIFI_SSID);
    Serial.print(")...");
    if (teleClient.net.setup(WIFI_SSID, WIFI_PASSWORD)) {
      Serial.println("OK");
      state.set(STATE_NET_READY);
#if ENABLE_OLED
      oled.print("OK");
#endif
      break;
    } else {
      Serial.println("NO");
    }
  }
  if (!state.check(STATE_NET_READY)) {
    return false;
  }
#elif NET_DEVICE == NET_SIM800 || NET_DEVICE == NET_SIM5360 || NET_DEVICE == NET_SIM7600
  // initialize network module
  if (!state.check(STATE_NET_READY)) {
    Serial.print("CELL...");
    if (teleClient.net.begin(&sys)) {
      Serial.println(teleClient.net.deviceName());
#if NET_DEVICE == NET_SIM5360 || NET_DEVICE == NET_SIM7600
      Serial.print("IMEI:");
      Serial.println(teleClient.net.IMEI);
#endif
      state.set(STATE_NET_READY);
#if ENABLE_OLED
      oled.print(teleClient.net.deviceName());
      oled.print(" OK\r");
#endif
    } else {
      Serial.println("NO");
#if ENABLE_OLED
      oled.println("No cell module");
#endif
      return false;
    }
  }
  Serial.print("NET(APN:");
  Serial.print(CELL_APN);
  Serial.print(")");
  if (teleClient.net.setup(CELL_APN)) {
    String op = teleClient.net.getOperatorName();
    if (op.length()) {
      Serial.println(op);
#if ENABLE_OLED
      oled.print(' ');
      oled.print(op);
      delay(1000);
#endif
    } else {
#if ENABLE_OLED
      oled.print("Cell Connected");
#endif
      Serial.println("OK");
    }
  } else {
    Serial.println("NO");
    return false;
  }
#endif
  timeoutsNet = 0;

#if ENABLE_OLED
  oled.clear();
  oled.print("VIN:");
  oled.println(vin);
#endif

#if NET_DEVICE == NET_WIFI || NET_DEVICE == NET_SIM800 || NET_DEVICE == NET_SIM5360 || NET_DEVICE == NET_SIM7600
  Serial.print("IP...");
  String ip = teleClient.net.getIP();
  if (ip.length()) {
    Serial.println(ip);
#if ENABLE_OLED
    oled.print("IP:");
    oled.println(ip);
#endif
  } else {
    Serial.println("NO");
  }
  int csq = teleClient.net.getSignal();
  if (csq > 0) {
    Serial.print("CSQ:");
    Serial.print((float)csq / 10, 1);
    Serial.println("dB");
#if ENABLE_OLED
    oled.print("CSQ:");
    oled.print((float)csq / 10, 1);
    oled.println("dB");
#endif
  }
#endif

  cache.init(RAM_CACHE_SIZE);

  teleClient.reset();  
  if (!teleClient.connect()) {
    return false;
  }
  connErrors = 0;
  state.set(STATE_CONNECTED);

  cache.header(teleClient.feedid);

  // check system time
  time_t utc;
  time(&utc);
  struct tm *btm = gmtime(&utc);
  if (btm->tm_year > 100) {
    // valid system time available
    char buf[64];
    sprintf(buf, "%04u-%02u-%02u %02u:%02u:%02u",
      1900 + btm->tm_year, btm->tm_mon + 1, btm->tm_mday, btm->tm_hour, btm->tm_min, btm->tm_sec);
    Serial.print("UTC:");
    Serial.println(buf);
#if ENABLE_OLED
    oled.setCursor(0, 7);
    oled.print("Packets");
    oled.setCursor(80, 7);
    oled.print("KB Sent");
    oled.setFontSize(FONT_SIZE_MEDIUM);
#endif
  }

#if STORAGE != STORAGE_NONE
  if (!state.check(STATE_STORAGE_READY)) {
    // init storage
    if (store.init()) {
      state.set(STATE_STORAGE_READY);
    }
  }
  if (state.check(STATE_STORAGE_READY)) {
    fileid = store.begin();
    if (fileid) {
      cache.setForward(&store);
    }
  }
#endif

#if MEMS_MODE
  lastMotionTime = millis();
#endif
  state.set(STATE_WORKING);
  return true;
}

void shutDownNet()
{
  if (state.check(STATE_NET_READY)) {
    if (state.check(STATE_CONNECTED)) {
      teleClient.notify(EVENT_LOGOUT, SERVER_KEY, 0);
    }
  }
  Serial.print(teleClient.net.deviceName());
  teleClient.net.close();
  teleClient.net.end();
  state.clear(STATE_NET_READY);
  Serial.println(" OFF");
}

/*******************************************************************************
  Executing a remote command
*******************************************************************************/
String executeCommand(const char* cmd)
{
  String result;
  Serial.println(cmd);
  if (!strncmp(cmd, "LED", 3) && cmd[4]) {
    ledMode = (byte)atoi(cmd + 4);
    digitalWrite(PIN_LED, (ledMode == 2) ? HIGH : LOW);
    result = "OK";
  } else if (!strcmp(cmd, "REBOOT")) {
  #if STORAGE != STORAGE_NONE
    if (state.check(STATE_STORAGE_READY)) {
      store.end();
      state.clear(STATE_STORAGE_READY);
    }
  #endif
    shutDownNet();
    ESP.restart();
    // never reach here
  } else if (!strcmp(cmd, "STANDBY")) {
    state.clear(STATE_WORKING);
    result = "OK";
  } else if (!strcmp(cmd, "WAKEUP")) {
    state.clear(STATE_STANDBY);
    result = "OK";
  } else if (!strncmp(cmd, "SET", 3) && cmd[3]) {
    const char* subcmd = cmd + 4;
    if (!strncmp(subcmd, "INTERVAL", 8) && subcmd[8]) {
      sendingInterval = atoi(subcmd + 8 + 1);
      result = "OK";
    } else if (!strncmp(subcmd, "SYNC", 4) && subcmd[4]) {
      syncInterval = atoi(subcmd + 4 + 1);
      result = "OK";
    } else {
      result = "ERROR";
    }
  } else if (!strcmp(cmd, "STATS")) {
    char buf[64];
    sprintf(buf, "TX:%u OBD:%u NET:%u", teleClient.txCount, timeoutsOBD, timeoutsNet);
    result = buf;
  #if ENABLE_OBD
  } else if (!strncmp(cmd, "OBD", 3) && cmd[4]) {
    // send OBD command
    String obdcmd = cmd + 4;
    obdcmd += '\r';
    char buf[256];
    if (obd->sendCommand(obdcmd.c_str(), buf, sizeof(buf), OBD_TIMEOUT_LONG) > 0) {
      Serial.println(buf);
      for (int n = 0; buf[n]; n++) {
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
  #endif
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
    snprintf(buf, sizeof(buf), "TK=%u,MSG=%s", token, result.c_str());
    for (byte attempts = 0; attempts < 3; attempts++) {
      Serial.println("ACK...");
      if (teleClient.notify(EVENT_ACK, SERVER_KEY, buf)) {
        Serial.println("sent");
        break;
      }
    }
  } else {
    // previously executed command
    char buf[64];
    snprintf(buf, sizeof(buf), "TK=%u,DUP=1", token);
    for (byte attempts = 0; attempts < 3; attempts++) {
      Serial.println("ACK...");
      if (teleClient.notify(EVENT_ACK, SERVER_KEY, buf)) {
        Serial.println("sent");
        break;
      }
    }
  }
  return true;
}

void showStats()
{
  uint32_t t = millis() - teleClient.startTime;
  char timestr[24];
  sprintf(timestr, "%02u:%02u.%c ", t / 60000, (t % 60000) / 1000, (t % 1000) / 100 + '0');
  Serial.print(timestr);
  Serial.print("| Packet #");
  Serial.print(teleClient.txCount);
  Serial.print(' ');
  Serial.print(cache.length());
  Serial.print(" bytes | Out: ");
  Serial.print(teleClient.txBytes >> 10);
  Serial.print(" KB | In: ");
  Serial.print(teleClient.rxBytes);
  Serial.print(" bytes");
#if STORAGE != STORAGE_NONE
  Serial.print(" | File:");
  Serial.print(store.size() >> 10);
  Serial.print("KB");
#endif
  Serial.println();
#if ENABLE_OLED
  oled.setCursor(0, 5);
  oled.printInt(teleClient.txCount, 2);
  oled.setCursor(80, 5);
  oled.printInt(teleClient.txBytes >> 10, 3);
#endif
}

/*******************************************************************************
  Collecting and processing data
*******************************************************************************/
void process()
{
  uint32_t startTime = millis();
  cache.timestamp(startTime);

#if ENABLE_OBD
  if ((teleClient.txCount % 100) == 1) {
    int temp = (int)readChipTemperature() * 165 / 255 - 40;
    cache.log(PID_DEVICE_TEMP, temp);
  }
  // process OBD data if connected
  if (state.check(STATE_OBD_READY)) {
    processOBD();
  }
#else
  int temp = (int)readChipTemperature() * 165 / 255 - 40;
  cache.log(PID_DEVICE_TEMP, temp);
  cache.log(PID_DEVICE_HALL, readChipHallSensor() / 200);
#endif

#if MEMS_MODE
  // process MEMS data if available
  if (state.check(STATE_MEMS_READY)) {
    processMEMS();
  }
#endif

#if ENABLE_OBD
  // read and log car battery voltage, data in 0.01v
  batteryVoltage = obd->getVoltage() * 100;
  cache.log(PID_BATTERY_VOLTAGE, batteryVoltage);
#endif

#if ENABLE_GPS
  // process GPS data if connected
  if (state.check(STATE_GPS_READY)) {
    processGPS();
  }
#endif

#if LOG_EXT_SENSORS
  processExtInputs();
#endif

#if STORAGE != STORAGE_NONE
  uint8_t sizeKB = (uint8_t)(store.size() >> 10);
  if (sizeKB != lastSizeKB) {
      store.flush();
      lastSizeKB = sizeKB;
  }
#endif

  if (syncInterval > 10000 && millis() - teleClient.lastSyncTime > syncInterval) {
    Serial.println("Connection instable");
    connErrors++;
    timeoutsNet++;
    printTimeoutStats();
  } else if (millis() - teleClient.lastSentTime >= sendingInterval && cache.samples() > 0) {
    // start data chunk
    if (ledMode == 0) digitalWrite(PIN_LED, HIGH);

#if SERVER_PROTOCOL == PROTOCOL_UDP
    cache.tailer();
#endif
    if (teleClient.transmit(cache.buffer(), cache.length())) {
      // successfully sent
      connErrors = 0;
      showStats();
    } else {
      connErrors++;
      timeoutsNet++;
      printTimeoutStats();
      if (connErrors >= MAX_CONN_ERRORS_RECONNECT) {
        if (teleClient.connect()) {
          connErrors = 0;
        }
      }
    }
    // purge cache
    cache.purge();
#if SERVER_PROTOCOL == PROTOCOL_UDP
    cache.header(teleClient.feedid);
#endif
    if (ledMode == 0) digitalWrite(PIN_LED, LOW);
  }

  if (deviceTemp >= COOLING_DOWN_TEMP) {
    // device too hot, cool down
    Serial.println("Cooling down");
    delay(10000);
  }

#if ENABLE_OBD
  if (obd->errors > MAX_OBD_ERRORS) {
    Serial.println("Reset OBD");
    obd->reset();
    state.clear(STATE_OBD_READY | STATE_WORKING);
  }
#endif

#if MEMS_MODE && MOTIONLESS_STANDBY
  if (!state.check(STATE_OBD_READY) && state.check(STATE_MEMS_READY)) {
    if (millis() - lastMotionTime > MOTIONLESS_STANDBY * 1000) {
      // enter standby mode
      state.clear(STATE_WORKING);
    }
  }
#endif

  // maintain minimum loop time
#if MIN_LOOP_TIME
  do {
#endif
    idleTasks();
#if ENABLE_HTTPD
    serverProcess(0);
#endif
#if MIN_LOOP_TIME
  } while (millis() - startTime < MIN_LOOP_TIME);
#endif
}

/*******************************************************************************
  Implementing stand-by mode
*******************************************************************************/
void standby()
{
  shutDownNet();
#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    store.end();
    state.clear(STATE_STORAGE_READY);
  }
#endif
#if ENABLE_GPS
  if (state.check(STATE_GPS_READY)) {
    Serial.println("GPS OFF");
    sys.gpsInit(0); // turn off GPS power
  }
#endif
#if ENABLE_OBD
  if (state.check(STATE_OBD_READY)) {
    if (obd->errors > MAX_OBD_ERRORS) {
      // inaccessible OBD treated as end of trip
    }
  }
#endif
  state.clear(STATE_OBD_READY | STATE_GPS_READY | STATE_NET_READY | STATE_CONNECTED);
  state.set(STATE_STANDBY);
  Serial.println("Standby");
#if ENABLE_OLED
  oled.clear();
#endif
#if MEMS_MODE
  if (state.check(STATE_MEMS_READY)) {
    calibrateMEMS();
    while (state.check(STATE_STANDBY)) {
      // calculate relative movement
      mems.read(acc);
      float motion = 0;
      for (byte i = 0; i < 3; i++) {
        float m = (acc[i] - accBias[i]);
        motion += m * m;
      }
      // check movement
      if (motion > MOTION_THRESHOLD * MOTION_THRESHOLD) {
        lastMotionTime = millis();
        Serial.print("Motion:");
        Serial.println(motion);
        break;
      }
#if ENABLE_HTTPD
      serverProcess(100);
#else
      delay(100);
#endif
    }
  }
#elif ENABLE_OBD
  float v;
  obd->reset();
  do {
    delay(5000);
    v = obd->getVoltage();
    batteryVoltage = v * 100;    
  } while (v < JUMPSTART_VOLTAGE);
#else
  delay(5000);
#endif
  state.clear(STATE_STANDBY);
  Serial.println("Wakeup");
#if RESET_AFTER_WAKEUP
#if MEMS_MODE
  mems.end();  
#endif
  ESP.restart();
#endif  
}

/*******************************************************************************
  Tasks to perform in idle/waiting time
*******************************************************************************/
void idleTasks()
{
  teleClient.inbound();

  // check serial input for command
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (serialCommand.length() > 0) {
        String result = executeCommand(serialCommand.c_str());
        serialCommand = "";
        Serial.println(result);
      }
    } else if (serialCommand.length() < 32) {
      serialCommand += c;
    }
  }
}

void setup()
{
#if ENABLE_OLED
    oled.begin();
    oled.setFontSize(FONT_SIZE_MEDIUM);
    oled.println("  FREEMATICS");
    oled.setFontSize(FONT_SIZE_SMALL);
    oled.println();
#endif
    delay(1000);

    // init LED pin
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

#if LOG_EXT_SENSORS
    pinMode(PIN_SENSOR1, INPUT);
    pinMode(PIN_SENSOR2, INPUT);
#endif

    // initialize USB serial
    Serial.begin(115200);

    // display system info
    int freq = ESP.getCpuFreqMHz();
    int flashSize = getFlashSize() >> 10;
    Serial.print("ESP32 ");
    Serial.print(freq);
    Serial.print("MHz ");
    Serial.print(flashSize);
    Serial.println("MB Flash");
#if ENABLE_OLED
    oled.clear();
    oled.print("CPU:");
    oled.print(freq);
    oled.print(" Mhz ");
    oled.print(flashSize);
    oled.println("MB Flash");
#endif
    // generate a unique ID in case VIN is inaccessible
    uint64_t mac = ESP.getEfuseMac();
    sprintf(vin, "DEVID%04X%08X", (uint32_t)(mac >> 32), (uint32_t)mac);

#if ENABLE_HTTPD
    IPAddress ip;
    Serial.print("HTTPD...");
    if (serverSetup(ip)) {
      Serial.println(ip);
#if ENABLE_OLED
      oled.print("AP:");
      oled.println(ip);
#endif
    } else {
      Serial.println("NO");
    }
#endif

    // startup initializations
    sys.begin();
#if ENABLE_OBD
    while (!(obd = createOBD()));
#endif

    // initializing components
    initialize();

    digitalWrite(PIN_LED, LOW);
}

void loop()
{
  // error handling
  if (!state.check(STATE_WORKING)) {
    standby();
    for (byte n = 0; n < 3; n++) {
      digitalWrite(PIN_LED, HIGH);
      initialize();
      digitalWrite(PIN_LED, LOW);
      if (state.check(STATE_WORKING)) break;
      delay(3000);
    }
    return;
  }

  if (connErrors >= MAX_CONN_ERRORS) {
    digitalWrite(PIN_LED, HIGH);
    shutDownNet();
    initialize();
    digitalWrite(PIN_LED, LOW);
    return;
  }

  // collect and transmit data
  process();
}
