/******************************************************************************
* Reference sketch for a vehicle telematics data feed for Freematics Hub
* Works with Freematics ONE+
* Developed by Stanley Huang <stanley@freematics.com.au>
* Distributed under BSD license
* Visit https://freematics.com/products for hardware information
* Visit https://hub.freematics.com to view live and history telemetry data
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
#ifdef BOARD_HAS_PSRAM
#include "esp_himem.h"
#endif
#if ENABLE_OLED
#include "FreematicsOLED.h"
#endif

// states
#define STATE_STORAGE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_NET_CONNECTED 0x20
#define STATE_WORKING 0x40
#define STATE_OBD_FOUND 0x80
#define STATE_STANDBY 0x100

typedef struct {
  byte pid;
  byte tier;
  int value;
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

CBufferManager bufman;
Task subtask;

#if MEMS_MODE
float accBias[3] = {0}; // calibrated reference accelerometer data
float accSum[3] = {0};
uint8_t accCount = 0;
#endif
int16_t deviceTemp = 0;

// live data
int16_t rssi = 0;
char vin[18] = {0};
uint16_t dtc[6] = {0};
int16_t batteryVoltage = 0;
GPS_DATA* gd = 0;
uint32_t lastGPStime = 0;

char devid[12] = {0};
char isoTime[26] = {0};

// stats data
uint32_t lastMotionTime = 0;
uint32_t timeoutsOBD = 0;
uint32_t timeoutsNet = 0;

uint32_t syncInterval = SERVER_SYNC_INTERVAL * 1000;

#if STORAGE != STORAGE_NONE
int fileid = 0;
static uint8_t lastSizeKB = 0;
#endif

uint32_t lastCmdToken = 0;
String serialCommand;

byte ledMode = 0;

bool serverSetup(IPAddress& ip);
void serverProcess(int timeout);
String executeCommand(const char* cmd);
bool processCommand(char* data);
void processMEMS(CBuffer* buffer);
bool processGPS(CBuffer* buffer);

class State {
public:
  bool check(uint16_t flags) { return (m_state & flags) == flags; }
  void set(uint16_t flags) { m_state |= flags; }
  void clear(uint16_t flags) { m_state &= ~flags; }
  uint16_t m_state = 0;
};

FreematicsESP32 sys;

class OBD : public COBD
{
protected:
  void idleTasks()
  {
    // do some quick tasks while waiting for OBD response
#if MEMS_MODE
    processMEMS(0);
#endif
  }
};

OBD obd;

#if MEMS_MODE == MEMS_ACC
MPU9250_ACC mems;
#elif MEMS_MODE == MEMS_9DOF
MPU9250_9DOF mems;
#elif MEMS_MODE == MEMS_DMP
MPU9250_DMP mems;
#endif

#if STORAGE == STORAGE_SPIFFS
SPIFFSLogger logger;
#elif STORAGE == STORAGE_SD
SDLogger logger;
#endif

#if SERVER_PROTOCOL == PROTOCOL_UDP
TeleClientUDP teleClient;
#else
TeleClientHTTP teleClient;
#endif

#if ENABLE_OLED
OLED_SH1106 oled;
#endif

State state;

void printTimeoutStats()
{
  Serial.print("Timeouts: OBD:");
  Serial.print(timeoutsOBD);
  Serial.print(" Network:");
  Serial.println(timeoutsNet);
}

#if LOG_EXT_SENSORS
void processExtInputs(CBuffer* buffer)
{
  int pins[] = {PIN_SENSOR1, PIN_SENSOR2};
  int pids[] = {PID_EXT_SENSOR1, PID_EXT_SENSOR2};
#if LOG_EXT_SENSORS == 1
  for (int i = 0; i < 2; i++) {
    buffer->add(pids[i], digitalRead(pins[i]));
  }
#elif LOG_EXT_SENSORS == 2
  for (int i = 0; i < 2; i++) {
    buffer->add(pids[i], analogRead(pins[i]));
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
    int n = snprintf(buf, bufsize, "{\"obd\":{\"vin\":\"%s\",\"battery\":%.1f,\"pid\":[", vin, (float)batteryVoltage / 100);
    uint32_t t = millis();
    for (int i = 0; i < sizeof(obdData) / sizeof(obdData[0]); i++) {
        n += snprintf(buf + n, bufsize - n, "{\"pid\":%u,\"value\":%d,\"age\":%u},",
            0x100 | obdData[i].pid, obdData[i].value, (unsigned int)(t - obdData[i].ts));
    }
    n--;
    n += snprintf(buf + n, bufsize - n, "]}");
#if MEMS_MODE
    if (accCount) {
      n += snprintf(buf + n, bufsize - n, ",\"mems\":{\"acc\":[%d,%d,%d],\"stationary\":%u}",
          (int)((accSum[0] / accCount - accBias[0]) * 100), (int)((accSum[1] / accCount - accBias[1]) * 100), (int)((accSum[2] / accCount - accBias[2]) * 100),
          (unsigned int)(millis() - lastMotionTime));
    }
#endif
    if (gd && gd->ts) {
      n += snprintf(buf + n, bufsize - n, ",\"gps\":{\"utc\":\"%s\",\"lat\":%f,\"lng\":%f,\"alt\":%f,\"speed\":%f,\"sat\":%d,\"age\":%u}",
          isoTime, gd->lat, gd->lng, gd->alt, gd->speed, (int)gd->sat, (unsigned int)(millis() - gd->ts));
    }
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
void processOBD(CBuffer* buffer)
{
  static int idx[2] = {0, 0};
  int tier = 1;
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
    if (!obd.isValidPID(pid)) continue;
    int value;
    if (obd.readPID(pid, value)) {
        obdData[i].ts = millis();
        obdData[i].value = value;
        buffer->add((uint16_t)pid | 0x100, value);
    } else {
        timeoutsOBD++;
        printTimeoutStats();
        break;
    }
    if (tier > 1) break;
  }
  int kph = obdData[0].value;
  if (kph >= 1) lastMotionTime = millis();
}
#endif

bool processGPS(CBuffer* buffer)
{
  if (state.check(STATE_GPS_READY)) {
    // read parsed GPS data
    if (!sys.gpsGetData(&gd)) {
      return false;
    }
  } else {
#if NET_DEVICE == NET_SIM5360 || NET_DEVICE == NET_SIM7600
    if (!teleClient.net.getLocation(&gd)) {
      return false;
    }
#endif
  }

  if (!gd || lastGPStime == gd->time) return false;

  float kph = gd->speed * 1.852f;
  if (kph >= 2) lastMotionTime = millis();

  if (buffer) {
    buffer->add(PID_GPS_DATE, gd->date);
    buffer->add(PID_GPS_TIME, gd->time);
    if (gd->lat && gd->lng && gd->alt) {
      buffer->add(PID_GPS_LATITUDE, gd->lat);
      buffer->add(PID_GPS_LONGITUDE, gd->lng);
      buffer->add(PID_GPS_ALTITUDE, gd->alt); /* m */
      buffer->add(PID_GPS_SPEED, kph);
      buffer->add(PID_GPS_HEADING, gd->heading);
      buffer->add(PID_GPS_SAT_COUNT, gd->sat);
      buffer->add(PID_GPS_HDOP, gd->hdop);
    }
  }
  
  // generate ISO time string
  char *p = isoTime + sprintf(isoTime, "%04u-%02u-%02uT%02u:%02u:%02u",
      (unsigned int)(gd->date % 100) + 2000, (unsigned int)(gd->date / 100) % 100, (unsigned int)(gd->date / 10000),
      (unsigned int)(gd->time / 1000000), (unsigned int)(gd->time % 1000000) / 10000, (unsigned int)(gd->time % 10000) / 100);
  unsigned char tenth = (gd->time % 100) / 10;
  if (tenth) p += sprintf(p, ".%c00", '0' + tenth);
  *p = 'Z';
  *(p + 1) = 0;

  Serial.print("[GPS] ");
  Serial.print(gd->lat, 6);
  Serial.print(' ');
  Serial.print(gd->lng, 6);
  Serial.print(' ');
  Serial.print((int)kph);
  Serial.print("km/h");
  if (gd->sat) {
    Serial.print(" SATS:");
    Serial.print(gd->sat);
  }
  Serial.print(" Course:");
  Serial.print(gd->heading);

  Serial.print(' ');
  Serial.println(isoTime);
  //Serial.println(gd->errors);
  lastGPStime = gd->time;
  return true;
}

bool waitMotionGPS(int timeout)
{
  unsigned long t = millis();
  lastMotionTime = 0;
  do {
    delay(200);
    if (!processGPS(0)) continue;
    if (lastMotionTime) return true;
  } while (millis() - t < timeout);
  return false;
}

#if MEMS_MODE
void processMEMS(CBuffer* buffer)
{
  if (!state.check(STATE_MEMS_READY)) return;

  // load and store accelerometer data
  int16_t temp = 0;
  float acc[3];
#if ENABLE_ORIENTATION
  ORIENTATION ori;
  float gyr[3];
  float mag[3];
  mems.read(acc, gyr, mag, &temp, &ori);
#else
  if (!mems.read(acc, 0, 0, &temp)) return;
#endif
  accSum[0] += acc[0];
  accSum[1] += acc[1];
  accSum[2] += acc[2];
  accCount++;

  if (buffer) {
    if (accCount) {
      float value[3];
      value[0] = accSum[0] / accCount - accBias[0];
      value[1] = accSum[1] / accCount - accBias[1];
      value[2] = accSum[2] / accCount - accBias[2];
      buffer->add(PID_ACC, value);
#if ENABLE_ORIENTATION
      value[0] = ori.yaw;
      value[1] = ori.pitch;
      value[2] = ori.roll;
      buffer->add(PID_ORIENTATION, value);
#endif
      temp /= 10;
      if (temp != deviceTemp) {
        buffer->add(PID_DEVICE_TEMP, (int)(deviceTemp = temp));
      }
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
    accSum[0] = 0;
    accSum[1] = 0;
    accSum[2] = 0;
    accCount = 0;
  }
}

void calibrateMEMS()
{
  if (state.check(STATE_MEMS_READY)) {
    Serial.print("ACC BIAS...");
    accBias[0] = 0;
    accBias[1] = 0;
    accBias[2] = 0;
    int n;
    unsigned long t = millis();
    for (n = 0; millis() - t < 1000; n++) {
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
}
#endif

void printTime()
{
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
  }
}

/*******************************************************************************
  Initializing all components and network
*******************************************************************************/
void initialize()
{
#if MEMS_MODE
  if (state.check(STATE_MEMS_READY)) {
    calibrateMEMS();
  }
#endif

#if ENABLE_GPS
  // start serial communication with GPS receiver
  if (!state.check(STATE_GPS_READY)) {
    Serial.print("GNSS...");
    if (sys.gpsBegin(GPS_SERIAL_BAUDRATE, false)) {
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

#if ENABLE_OBD
  // initialize OBD communication
  if (!state.check(STATE_OBD_READY)) {
    timeoutsOBD = 0;
    Serial.print("OBD...");
    if (obd.init()) {
      Serial.println("OK");
      state.set(STATE_OBD_READY | STATE_OBD_FOUND);
#if ENABLE_OLED
      oled.println("OBD OK");
#endif
    } else {
      Serial.println("NO");
      if (state.check(STATE_OBD_FOUND)) return;
    }
  }
#endif

#if 0
  if (!state.check(STATE_OBD_READY) && state.check(STATE_GPS_READY)) {
    // wait for movement from GPS when OBD not connected
    Serial.println("Waiting...");
    if (!waitMotionGPS(GPS_MOTION_TIMEOUT * 1000)) {
      return false;
    }
  }
#endif

#if STORAGE != STORAGE_NONE
  if (!state.check(STATE_STORAGE_READY)) {
    // init storage
    if (logger.init()) {
      state.set(STATE_STORAGE_READY);
    }
  }
  if (state.check(STATE_STORAGE_READY)) {
    fileid = logger.begin();
  }
#endif



  // re-try OBD if connection not established
#if ENABLE_OBD
  if (state.check(STATE_OBD_READY)) {
    char buf[128];
    if (obd.getVIN(buf, sizeof(buf))) {
      strncpy(vin, buf, sizeof(vin) - 1);
      Serial.print("VIN:");
      Serial.println(vin);
    }
    int dtcCount = obd.readDTC(dtc, sizeof(dtc) / sizeof(dtc[0]));
    if (dtcCount > 0) {
      Serial.print("DTC:");
      Serial.println(dtcCount);
    }
#if ENABLE_OLED
    oled.print("VIN:");
    oled.println(vin);
#endif
  }
#endif

  // check system time
  printTime();

  lastMotionTime = millis();
  state.set(STATE_WORKING);

#if ENABLE_OLED
  delay(1000);
  oled.clear();
  oled.print("DEVICE ID: ");
  oled.println(devid);
  oled.setCursor(0, 7);
  oled.print("Packets");
  oled.setCursor(80, 7);
  oled.print("KB Sent");
  oled.setFontSize(FONT_SIZE_MEDIUM);
#endif
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
      logger.end();
      state.clear(STATE_STORAGE_READY);
    }
  #endif
    teleClient.shutdown();
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
    if (!strncmp(subcmd, "SYNC", 4) && subcmd[4]) {
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
    char buf[256];
    sprintf(buf, "%s\r", cmd + 4);
    if (obd.link && obd.link->sendCommand(buf, buf, sizeof(buf), OBD_TIMEOUT_LONG) > 0) {
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
      if (teleClient.notify(EVENT_ACK, buf)) {
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
      if (teleClient.notify(EVENT_ACK, buf)) {
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
  Serial.print("[NET] ");
  Serial.print(timestr);
  Serial.print("| Packet #");
  Serial.print(teleClient.txCount);
  Serial.print(" | Out: ");
  Serial.print(teleClient.txBytes >> 10);
  Serial.print(" KB | In: ");
  Serial.print(teleClient.rxBytes);
  Serial.print(" bytes");
#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    Serial.print(" | File: ");
    Serial.print(logger.size() >> 10);
    Serial.print("KB");
  }
#endif
  Serial.println();
#if ENABLE_OLED
  oled.setCursor(0, 2);
  oled.println(timestr);
  oled.setCursor(0, 5);
  oled.printInt(teleClient.txCount, 2);
  oled.setCursor(80, 5);
  oled.printInt(teleClient.txBytes >> 10, 3);
#endif
}

bool waitMotion(long timeout)
{
  unsigned long t = millis();
#if MEMS_MODE
  if (state.check(STATE_MEMS_READY)) {
    do {
      serverProcess(100);
      // calculate relative movement
      float motion = 0;
      float acc[3];
      mems.read(acc);
      if (accCount == 10) {
        accCount = 0;
        accSum[0] = 0;
        accSum[1] = 0;
        accSum[2] = 0;
      }
      accSum[0] += acc[0];
      accSum[1] += acc[1];
      accSum[2] += acc[2];
      accCount++;
      for (byte i = 0; i < 3; i++) {
        float m = (acc[i] - accBias[i]);
        motion += m * m;
      }
      // check movement
      if (motion >= MOTION_THRESHOLD * MOTION_THRESHOLD) {
        //lastMotionTime = millis();
        return true;
      }
    } while ((long)(millis() - t) < timeout || timeout == -1);
    return false;
  }
#endif
  serverProcess(timeout);
  return false;
}

/*******************************************************************************
  Collecting and processing data
*******************************************************************************/
void process()
{
  uint32_t startTime = millis();

  CBuffer* buffer = bufman.get();
  if (!buffer) {
    buffer = bufman.getOldest();
    if (!buffer) return;
    while (buffer->state == BUFFER_STATE_LOCKED) delay(1);
    buffer->purge();
  }

  buffer->state = BUFFER_STATE_FILLING;

#if ENABLE_OBD
  // process OBD data if connected
  if (state.check(STATE_OBD_READY)) {
    processOBD(buffer);
    if (obd.errors >= MAX_OBD_ERRORS) {
      Serial.println("ECU OFF");
      state.clear(STATE_OBD_READY | STATE_WORKING);
      return;
    }
  }
#else
  buffer->add(PID_DEVICE_HALL, readChipHallSensor() / 200);
#endif

#if ENABLE_OBD
  if (sys.version > 12) {
    batteryVoltage = (float)(analogRead(A0) * 11 * 370) / 4095;
  } else {
    batteryVoltage = obd.getVoltage() * 100;
  }
  if (batteryVoltage) {
    buffer->add(PID_BATTERY_VOLTAGE, (int)batteryVoltage);
  }
#endif

#if LOG_EXT_SENSORS
  processExtInputs(buffer);
#endif

#if MEMS_MODE
  processMEMS(buffer);
#endif

  processGPS(buffer);

  if (!state.check(STATE_MEMS_READY)) {
    deviceTemp = readChipTemperature();
    buffer->add(PID_DEVICE_TEMP, deviceTemp);
  }

#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    uint8_t sizeKB = (uint8_t)(logger.size() >> 10);
    if (sizeKB != lastSizeKB) {
        logger.flush();
        lastSizeKB = sizeKB;
    }
  }
#endif

  buffer->timestamp = millis();
  buffer->state = BUFFER_STATE_FILLED;

#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    buffer->serialize(logger);
  }
#endif
  bufman.printStats();

#if DATASET_INTERVAL
  long t = (long)DATASET_INTERVAL - (millis() - startTime);
  if (t > 0 && t < DATASET_INTERVAL) delay(t);
#endif
  if (deviceTemp >= COOLING_DOWN_TEMP) {
    // device too hot, cool down
    Serial.println("Overheat");
    delay(10000);
  }
}

bool initNetwork()
{
#if NET_DEVICE != NET_WIFI
  // power on network module
  if (teleClient.net.begin(&sys)) {
    Serial.print(teleClient.net.deviceName());
#if NET_DEVICE == NET_SIM5360 || NET_DEVICE == NET_SIM7600
    Serial.print(" IMEI:");
    Serial.println(teleClient.net.IMEI);
#endif
    state.set(STATE_NET_READY);
#if ENABLE_OLED
    oled.print(teleClient.net.deviceName());
    oled.println(" OK\r");
#if NET_DEVICE == NET_SIM5360 || NET_DEVICE == NET_SIM7600
    oled.print("IMEI:");
    oled.println(teleClient.net.IMEI);
#endif
#endif
  } else {
    Serial.println("No Cell Module");
#if ENABLE_OLED
    oled.println("No Cell Module");
#endif
    return false;
  }
  if (!teleClient.net.checkSIM()) {
    Serial.println("NO SIM CARD");
    return false;
  }
#endif

#if NET_DEVICE == NET_WIFI
#if ENABLE_OLED
  oled.print("Connecting WiFi...");
#endif
  for (byte attempts = 0; attempts < 10; attempts++) {
    teleClient.net.begin(WIFI_SSID, WIFI_PASSWORD);
    if (teleClient.net.setup()) {
      state.set(STATE_NET_READY);
      Serial.print("WiFi IP:");
      String ip = teleClient.net.getIP();
      if (ip.length()) {
        state.set(STATE_NET_CONNECTED);
        Serial.print(ip);
#if ENABLE_OLED
        oled.println(ip);
#endif
        break;
      }
      Serial.println();
    } else {
      Serial.println("No WiFi");
      return false;
    }
  }
#else
  if (state.check(STATE_NET_READY) && !state.check(STATE_NET_CONNECTED)) {
    bool extGPS = state.check(STATE_GPS_READY);
    if (teleClient.net.setup(CELL_APN)) {
      String op = teleClient.net.getOperatorName();
      if (op.length()) {
        Serial.print("Operator:");
        Serial.println(op);
#if ENABLE_OLED
        oled.println(op);
#endif
      } else {
#if ENABLE_OLED
        oled.print("Cell Connected");
#endif
      }

      if (!extGPS && teleClient.net.setGPS(true)) {
        Serial.println("Cell GNSS ON");
      }

      String ip = teleClient.net.getIP();
      if (ip.length()) {
        Serial.print("IP:");
        Serial.println(ip);
#if ENABLE_OLED
        oled.print("IP:");
        oled.println(ip);
#endif
      }
      rssi = teleClient.net.getSignal();
      if (rssi) {
        Serial.print("RSSI:");
        Serial.print(rssi);
        Serial.println("dBm");
#if ENABLE_OLED
        oled.print("RSSI:");
        oled.print(rssi);
        oled.println("dBm");
#endif
      }
      state.set(STATE_NET_CONNECTED);
    } else {
      char *p = strstr(teleClient.net.getBuffer(), "+CPSI:");
      if (p) {
        char *q = strchr(p, '\r');
        if (q) *q = 0;
        Serial.println(p + 7);
#if ENABLE_OLED
        oled.println(p + 7);
#endif
      } else {
        Serial.print(teleClient.net.getBuffer());
      }
    }
    timeoutsNet = 0;
  }
#endif
  return state.check(STATE_NET_CONNECTED);
}

void telemetry(void* inst)
{
  uint8_t connErrors = 0;
  const uint16_t stationaryTime[] = STATIONARY_TIME_TABLE;
  const int sendingIntervals[] = SENDING_INTERVAL_TABLE;
  CStorageRAM store;
  store.init(SERIALIZE_BUFFER_SIZE);
  teleClient.reset();

  for (;;) {
    if (!state.check(STATE_WORKING)) {
      if (state.check(STATE_NET_CONNECTED)) {
        if (teleClient.notify(EVENT_LOGOUT)) {
          Serial.println("Logged Out");
        }
      }
      if (state.check(STATE_NET_READY)) {
        teleClient.shutdown();
      }
      state.clear(STATE_NET_READY | STATE_NET_CONNECTED);
      teleClient.reset();

      uint32_t t = millis();
      do {
        delay(1000);
      } while (state.check(STATE_STANDBY) && millis() - t < 1000L * PING_BACK_INTERVAL);
      if (state.check(STATE_STANDBY)) {
        // start ping
        Serial.print("Ping...");
#if NET_DEVICE == NET_WIFI
        if (!teleClient.net.begin(WIFI_SSID, WIFI_PASSWORD) || !teleClient.net.setup()) {
          Serial.println("No WiFi");
          continue;
        }
#else
        if (!teleClient.net.begin(&sys) || !teleClient.net.setup(CELL_APN)) {
          Serial.println("No network");
          continue;
        }
#endif
        Serial.print(teleClient.net.getIP());
        state.set(STATE_NET_READY);
        if (teleClient.ping()) {
          Serial.print(" OK");
        }
        Serial.println();   
      }
      continue;
    }
    
    if (!state.check(STATE_NET_CONNECTED)) {
      if (!initNetwork() || !teleClient.connect()) {
        teleClient.shutdown();
        state.clear(STATE_NET_READY | STATE_NET_CONNECTED);
        delay(3000);
        continue;
      }
    }

    connErrors = 0;
    teleClient.startTime = millis();

    for (;;) {
      CBuffer* buffer = bufman.getNewest();
      if (!buffer) {
        if (!state.check(STATE_WORKING)) break;
        delay(10);
        continue;
      }

      uint32_t startTime = millis();
      buffer->state = BUFFER_STATE_LOCKED;
#if SERVER_PROTOCOL == PROTOCOL_UDP
      store.header(devid);
#endif
      store.timestamp(buffer->timestamp);
      buffer->serialize(store);
      buffer->purge();
#if SERVER_PROTOCOL == PROTOCOL_UDP
      store.tailer();
#endif
      //Serial.println(store.buffer());

      // start transmission
      if (ledMode == 0) digitalWrite(PIN_LED, HIGH);
      if (teleClient.transmit(store.buffer(), store.length())) {
        // successfully sent
        connErrors = 0;
        showStats();
      } else {
        connErrors++;
        timeoutsNet++;
        printTimeoutStats();
      }
      if (ledMode == 0) digitalWrite(PIN_LED, LOW);

      store.purge();

      // process inbound data
      teleClient.inbound();

      if (syncInterval > 10000 && millis() - teleClient.lastSyncTime > syncInterval) {
        Serial.println("Instable connection");
        connErrors++;
        timeoutsNet++;
      }

#if ENABLE_OBD || ENABLE_GPS || MEMS_MODE
      // motion adaptive data transmission interval control
      while (state.check(STATE_WORKING)) {
        unsigned int motionless = (millis() - lastMotionTime) / 1000;
        int sendingInterval = -1;
        for (byte i = 0; i < sizeof(stationaryTime) / sizeof(stationaryTime[0]); i++) {
          if (motionless < stationaryTime[i] || stationaryTime[i] == 0) {
            sendingInterval = sendingIntervals[i];
            break;
          }
        }
        if (sendingInterval == -1) {
          // stationery timeout, trip ended
          Serial.print("Stationary for ");
          Serial.print(motionless);
          Serial.println(" secs");
          //teleClient.reset();
          state.clear(STATE_WORKING);
          break;
        }
        int n = startTime + sendingInterval - millis();
        if (n <= 0) break;
        waitMotion(min(n, 3000));
      }
#endif
      if (connErrors > MAX_CONN_ERRORS_RECONNECT) {
        teleClient.net.close();
        if (!teleClient.connect()) {
          teleClient.shutdown();
          state.clear(STATE_NET_READY | STATE_NET_CONNECTED);
          break;
        }
      }
    }
  }
}

/*******************************************************************************
  Implementing stand-by mode
*******************************************************************************/
void standby()
{
#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    logger.end();
    state.clear(STATE_STORAGE_READY);
  }
#endif
#if ENABLE_GPS
  if (state.check(STATE_GPS_READY)) {
    Serial.println("GPS OFF");
    sys.gpsEnd();
    state.clear(STATE_GPS_READY);
  }
  gd = 0;
#endif
#if ENABLE_OBD
if (state.check(STATE_OBD_READY)) {
    obd.enterLowPowerMode();
    state.clear(STATE_OBD_READY);
  }
#endif
  state.set(STATE_STANDBY);
#if ENABLE_OLED
  oled.print("STANDBY");
  delay(1000);
  oled.clear();
#endif
  Serial.println("STANDBY");
#if MEMS_MODE
  calibrateMEMS();
  waitMotion(-1);
#elif ENABLE_OBD
  do {
    delay(5000);
  } while (obd.getVoltage() < JUMPSTART_VOLTAGE);
#else
  delay(5000);
#endif
  Serial.println("Wakeup");
#if RESET_AFTER_WAKEUP
#if MEMS_MODE
  mems.end();  
#endif
  ESP.restart();
#endif  
  state.clear(STATE_STANDBY);
}

/*******************************************************************************
  Tasks to perform in idle/waiting time
*******************************************************************************/
void genDeviceID(char* buf)
{
    uint64_t seed = ESP.getEfuseMac() >> 8;
    for (int i = 0; i < 8; i++, seed >>= 5) {
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
      buf[i] = x;
    }
    buf[8] = 0;
}

void showSysInfo()
{
  Serial.print("CPU:");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.print("MHz Flash:");
  Serial.print(getFlashSize() >> 10);
  Serial.println("MB");
#ifdef BOARD_HAS_PSRAM
  Serial.print("IRAM:");
  Serial.print(ESP.getHeapSize() >> 10);
  Serial.print("KB");
  if (psramInit()) {
    Serial.print(" PSRAM:");
    Serial.print((ESP.getPsramSize() + esp_himem_get_phys_size()) >> 10);
    Serial.print("KB");
#if 0
    Serial.println();
    Serial.print("Writing PSRAM...");
    int size = ESP.getPsramSize();
    uint32_t *ptr = (uint32_t*)ps_malloc(size);
    if (!ptr) {
      Serial.print("unable to allocate ");
      Serial.print(size);
      Serial.println(" bytes");
    } else {
      uint32_t t = millis();
      for (int i = 0; i < size / 4; i++) {
        ptr[i] = 0xa5a5a5a5;
      }
      Serial.print("OK @");
      Serial.print(size  / (millis() - t));
      Serial.println("KB/s");
    }
    Serial.print("Verifying PSRAM...");
    int errors = 0;
    uint32_t t = millis();
    for (int i = 0; i < size / 4; i++) {
      if (ptr[i] != 0xa5a5a5a5) {
        Serial.print("mismatch @ 0x");
        Serial.println(i * 4, 16);
        errors++;
      }
    }
    if (errors == 0) {
      Serial.print("OK @");
      Serial.print(size  / (millis() - t));
      Serial.println("KB/s");
    }
    free(ptr);
#endif
  }
  Serial.println();
#endif

  int rtc = rtc_clk_slow_freq_get();
  if (rtc) {
    Serial.print("RTC:");
    Serial.println(rtc);
  }
  
#if ENABLE_OLED
  oled.clear();
  oled.print("CPU:");
  oled.print(ESP.getCpuFreqMHz());
  oled.print("Mhz ");
  oled.print(getFlashSize() >> 10);
  oled.println("MB Flash");
#endif

    // generate a unique ID in case VIN is inaccessible
    genDeviceID(devid);
    Serial.print("DEVICE ID: ");
    Serial.println(devid);
#if ENABLE_OLED
    oled.print("DEVICE ID: ");
    oled.println(devid);
#endif
}

#if CONFIG_MODE_TIMEOUT
void configMode()
{
  uint32_t t = millis();

  do {
    if (Serial.available()) {
      // enter config mode
      Serial.println("#CONFIG MODE#");
      Serial1.begin(LINK_UART_BAUDRATE, SERIAL_8N1, PIN_LINK_UART_RX, PIN_LINK_UART_TX);
      do {
        if (Serial.available()) {
          Serial1.write(Serial.read());
          t = millis();
        }
        if (Serial1.available()) {
          Serial.write(Serial1.read());
          t = millis();
        }
      } while (millis() - t < CONFIG_MODE_TIMEOUT);
      Serial.println("#RESET#");
      delay(100);
      ESP.restart();
    }
  } while (millis() - t < CONFIG_MODE_TIMEOUT);
}
#endif

void setup()
{
    delay(500);

#if ENABLE_OLED
    oled.begin();
    oled.setFontSize(FONT_SIZE_SMALL);
#endif
    // initialize USB serial
    Serial.begin(115200);

    // init LED pin
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

#if CONFIG_MODE_TIMEOUT
    configMode();
#endif

#if LOG_EXT_SENSORS
    pinMode(PIN_SENSOR1, INPUT);
    pinMode(PIN_SENSOR2, INPUT);
#endif

    // show system information
    showSysInfo();

#if ENABLE_OBD
    if (sys.begin()) {
      Serial.print("Firmware: R");
      Serial.println(sys.version);
    }
    obd.begin(sys.link);
#endif

    sys.buzzer(2000);

#if MEMS_MODE
  if (!state.check(STATE_MEMS_READY)) {
    Serial.print("MEMS...");
    byte ret = mems.begin(ENABLE_ORIENTATION);
    if (ret) {
      state.set(STATE_MEMS_READY);
      if (ret == 2) Serial.print("9-DOF ");
      Serial.println("OK");
    } else {
      Serial.println("NO");
    }
  }
#endif

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

    sys.buzzer(0);

    state.set(STATE_WORKING);
    subtask.create(telemetry, "telemetry", 0, 8192);
    // initialize components
    initialize();

    digitalWrite(PIN_LED, LOW);
}

void loop()
{
  // error handling
  if (!state.check(STATE_WORKING)) {
    standby();
    digitalWrite(PIN_LED, HIGH);
    initialize();
    digitalWrite(PIN_LED, LOW);
    return;
  }

  // collect and transmit data
  process();

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
