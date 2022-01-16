/******************************************************************************
* Arduino sketch of a vehicle data data logger and telemeter for Freematics Hub
* Works with Freematics ONE+ Model A and Model B
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
#include "telemesh.h"
#include "teleclient.h"
#if BOARD_HAS_PSRAM
#include "esp32/himem.h"
#endif
#include "driver/adc.h"
#if ENABLE_OLED
#include "FreematicsOLED.h"
#endif
extern "C" {
#include "ble_spp_server.h"
}

// states
#define STATE_STORAGE_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_READY 0x4
#define STATE_MEMS_READY 0x8
#define STATE_NET_READY 0x10
#define STATE_NET_CONNECTED 0x20
#define STATE_WORKING 0x40
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

#if ENABLE_MEMS
float accBias[3] = {0}; // calibrated reference accelerometer data
float accSum[3] = {0};
uint8_t accCount = 0;
#endif
float deviceTemp = 0;

// live data
int16_t rssi = 0;
char vin[18] = {0};
uint16_t dtc[6] = {0};
int16_t batteryVoltage = 0;
GPS_DATA* gd = 0;

char devid[12] = {0};
char isoTime[32] = {0};

// stats data
uint32_t lastMotionTime = 0;
uint32_t timeoutsOBD = 0;
uint32_t timeoutsNet = 0;
uint32_t lastStatsTime = 0;

int32_t syncInterval = SERVER_SYNC_INTERVAL * 1000;
int32_t dataInterval = 1000;

#if STORAGE != STORAGE_NONE
int fileid = 0;
uint16_t lastSizeKB = 0;
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
#if ENABLE_MEMS
    processMEMS(0);
#endif
    delay(5);
  }
};

OBD obd;

MEMS_I2C* mems = 0;

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
  int pids[] = {PID_EXT_SENSOR1, PID_EXT_SENSOR2};
#if LOG_EXT_SENSORS == 1
  int pins[] = {PIN_SENSOR1, PIN_SENSOR2};
  for (int i = 0; i < 2; i++) {
    buffer->add(pids[i], digitalRead(pins[i]));
  }
#elif LOG_EXT_SENSORS == 2
  int reading[] = {adc1_get_raw(ADC1_CHANNEL_0), adc1_get_raw(ADC1_CHANNEL_1)};
  Serial.print("GPIO0:");
  Serial.print((float)reading[0] * 3.15 / 4095 - 0.01);
  Serial.print(" GPIO1:");
  Serial.print((float)reading[1] * 3.15 / 4095 - 0.01);
  for (int i = 0; i < 2; i++) {
    buffer->add(pids[i], reading[i]);
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
#if ENABLE_MEMS
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
  if (kph >= 2) lastMotionTime = millis();
}
#endif

bool processGPS(CBuffer* buffer)
{
  static uint32_t lastGPStime = 0;
  static float lastGPSLat = 0;
  static float lastGPSLng = 0;

  if (!gd) {
    lastGPStime = 0;
    lastGPSLat = 0;
    lastGPSLng = 0;
  }
#if GNSS == GNSS_INTERNAL || GNSS == GNSS_EXTERNAL
  if (state.check(STATE_GPS_READY)) {
    // read parsed GPS data
    if (!sys.gpsGetData(&gd)) {
      return false;
    }
  }
#elif NET_DEVICE >= NET_SIM5360
    if (!teleClient.net.getLocation(&gd)) {
      return false;
    }
#endif

  if (!gd || lastGPStime == gd->time || gd->date == 0 || (gd->lng == 0 && gd->lat == 0)) return false;

  if ((lastGPSLat || lastGPSLng) && (abs(gd->lat - lastGPSLat) > 0.001 || abs(gd->lng - lastGPSLng > 0.001))) {
    // invalid coordinates data
    lastGPSLat = 0;
    lastGPSLng = 0;
    return false;
  }
  lastGPSLat = gd->lat;
  lastGPSLng = gd->lng;
  
  float kph = (float)((int)(gd->speed * 1.852f * 10)) / 10;
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
      serverProcess(100);
    if (!processGPS(0)) continue;
    if (lastMotionTime) return true;
  } while (millis() - t < timeout);
  return false;
}

#if ENABLE_MEMS
void processMEMS(CBuffer* buffer)
{
  if (!state.check(STATE_MEMS_READY)) return;

  // load and store accelerometer data
  float temp = 0;
  float acc[3];
  float gyr[3];
  float mag[3];
#if ENABLE_ORIENTATION
  ORIENTATION ori;
  if (!mems->read(acc, gyr, mag, &temp, &ori)) return;
#else
  if (!mems->read(acc, gyr, mag, &temp)) return;
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
      if (temp != deviceTemp) {
        deviceTemp = temp;
        buffer->add(PID_DEVICE_TEMP, (int)temp);
      }
#if 0
      // calculate motion
      float motion = 0;
      for (byte i = 0; i < 3; i++) {
        motion += value[i] * value[i];
      }
      if (motion >= MOTION_THRESHOLD * MOTION_THRESHOLD) {
        lastMotionTime = millis();
        Serial.print("Motion:");
        Serial.println(motion);
      }
#endif
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
    accBias[0] = 0;
    accBias[1] = 0;
    accBias[2] = 0;
    int n;
    unsigned long t = millis();
    for (n = 0; millis() - t < 1000; n++) {
      float acc[3];
      if (!mems->read(acc)) continue;
      accBias[0] += acc[0];
      accBias[1] += acc[1];
      accBias[2] += acc[2];
      delay(10);
    }
    accBias[0] /= n;
    accBias[1] /= n;
    accBias[2] /= n;
    Serial.print("ACC BIAS:");
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
  Initializing all data logging components
*******************************************************************************/
void initialize()
{
#if ENABLE_BUZZER
  // turn on buzzer at 2000Hz frequency 
  sys.buzzer(2000);
  delay(100);
  // turn off buzzer
  sys.buzzer(0);
#endif

  // dump buffer data
  bufman.purge();

#if ENABLE_MEMS
  if (state.check(STATE_MEMS_READY)) {
    calibrateMEMS();
  }
#endif

#if GNSS == GNSS_INTERNAL || GNSS == GNSS_EXTERNAL
  // start GPS receiver
  if (!state.check(STATE_GPS_READY)) {
    Serial.print("GNSS:");
#if GNSS == GNSS_EXTERNAL
    if (sys.gpsBegin())
#else
    if (sys.gpsBegin(0))
#endif
    {
      state.set(STATE_GPS_READY);
      Serial.println("OK");
#if ENABLE_OLED
      oled.println("GNSS OK");
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
    if (obd.init()) {
      Serial.println("OBD:OK");
      state.set(STATE_OBD_READY);
#if ENABLE_OLED
      oled.println("OBD OK");
#endif
    } else {
      Serial.println("OBD:NO");
      //state.clear(STATE_WORKING);
      //return;
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
      memcpy(vin, buf, sizeof(vin) - 1);
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
  char buf[32];
#if ENABLE_BLE
  int len = sprintf(buf, "T:%u P:%u B:%u", t, teleClient.txCount, teleClient.txBytes);
  ble_send(SPP_IDX_SPP_STATUS_VAL, buf, len);
#endif
  sprintf(buf, "%02u:%02u.%c ", t / 60000, (t % 60000) / 1000, (t % 1000) / 100 + '0');
  Serial.print("[NET] ");
  Serial.print(buf);
  Serial.print("| Packet #");
  Serial.print(teleClient.txCount);
  Serial.print(" | Out: ");
  Serial.print(teleClient.txBytes >> 10);
  Serial.print(" KB | In: ");
  Serial.print(teleClient.rxBytes);
  Serial.print(" bytes");
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
#if ENABLE_MEMS
  unsigned long t = millis();
  if (state.check(STATE_MEMS_READY)) {
    do {
      serverProcess(100);
      // calculate relative movement
      float motion = 0;
      float acc[3];
      if (!mems->read(acc)) continue;
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
        Serial.println(motion);
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
      if (!obd.init()) {
        Serial.println("ECU OFF");
        state.clear(STATE_OBD_READY | STATE_WORKING);
        return;
      }
    }
  }
#else
  buffer->add(PID_DEVICE_HALL, readChipHallSensor() / 200);
#endif

#if ENABLE_OBD
  if (sys.devType > 12) {
    batteryVoltage = (float)(analogRead(A0) * 12 * 370) / 4095;
  } else if (state.check(STATE_OBD_READY)) {
    batteryVoltage = obd.getVoltage() * 100;
  }
  if (batteryVoltage) {
    buffer->add(PID_BATTERY_VOLTAGE, (int)batteryVoltage);
  }
#endif

#if LOG_EXT_SENSORS
  processExtInputs(buffer);
#endif

#if ENABLE_MEMS
  processMEMS(buffer);
#endif

  processGPS(buffer);

  if (!state.check(STATE_MEMS_READY)) {
    deviceTemp = readChipTemperature();
    buffer->add(PID_DEVICE_TEMP, deviceTemp);
  }

  buffer->timestamp = millis();
  buffer->state = BUFFER_STATE_FILLED;

  // display file buffer stats
  if (startTime - lastStatsTime >= 3000) {
    bufman.printStats();
    lastStatsTime = startTime;
  }

#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    buffer->serialize(logger);
    uint16_t sizeKB = (uint16_t)(logger.size() >> 10);
    if (sizeKB != lastSizeKB) {
      logger.flush();
      lastSizeKB = sizeKB;
      Serial.print("[FILE] ");
      Serial.print(sizeKB);
      Serial.println("KB");
    }
  }
#endif

  const int dataIntervals[] = DATA_INTERVAL_TABLE;
#if ENABLE_OBD || ENABLE_MEMS
  // motion adaptive data interval control
  const uint16_t stationaryTime[] = STATIONARY_TIME_TABLE;
  unsigned int motionless = (millis() - lastMotionTime) / 1000;
  bool stationary = true;
  for (byte i = 0; i < sizeof(stationaryTime) / sizeof(stationaryTime[0]); i++) {
    dataInterval = dataIntervals[i];
    if (motionless < stationaryTime[i] || stationaryTime[i] == 0) {
      stationary = false;
      break;
    }
  }
  if (stationary) {
    // stationery timeout
    Serial.print("Stationary for ");
    Serial.print(motionless);
    Serial.println(" secs");
    // trip ended, go into standby
    state.clear(STATE_WORKING);
    return;
  }
#else
  dataInterval = dataIntervals[0];
#endif
  long t = dataInterval - (millis() - startTime);
  if (t > 0 && t < dataInterval) delay(t);
}

bool initNetwork()
{
#if NET_DEVICE == NET_WIFI
#if ENABLE_OLED
  oled.print("Connecting WiFi...");
#endif
  for (byte attempts = 0; attempts < 3; attempts++) {
    Serial.print("Joining ");
    Serial.println(WIFI_SSID);
    teleClient.net.begin(WIFI_SSID, WIFI_PASSWORD);
    if (teleClient.net.setup()) {
      state.set(STATE_NET_READY);
      String ip = teleClient.net.getIP();
      if (ip.length()) {
        state.set(STATE_NET_CONNECTED);
        Serial.print("IP:");
        Serial.println(ip);
#if ENABLE_OLED
        oled.println(ip);
#endif
        break;
      }
    } else {
      Serial.println("No WiFi");
    }
  }
#else
  // power on network module
  if (teleClient.net.begin(&sys)) {
    state.set(STATE_NET_READY);
  } else {
    Serial.println("CELL:NO");
#if ENABLE_OLED
    oled.println("No Cell Module");
#endif
    return false;
  }
#if NET_DEVICE >= SIM800
#if ENABLE_OLED
    oled.print(teleClient.net.deviceName());
    oled.println(" OK\r");
    oled.print("IMEI:");
    oled.println(teleClient.net.IMEI);
#endif
  Serial.print("CELL:");
  Serial.println(teleClient.net.deviceName());
  if (!teleClient.net.checkSIM(SIM_CARD_PIN)) {
    Serial.println("NO SIM CARD");
    return false;
  }
  Serial.print("IMEI:");
  Serial.println(teleClient.net.IMEI);
  if (state.check(STATE_NET_READY) && !state.check(STATE_NET_CONNECTED)) {
    if (teleClient.net.setup(CELL_APN)) {
      String op = teleClient.net.getOperatorName();
      if (op.length()) {
        Serial.print("Operator:");
        Serial.println(op);
#if ENABLE_OLED
        oled.println(op);
#endif
      }

#if GNSS == GNSS_CELLULAR
      if (teleClient.net.setGPS(true)) {
        Serial.println("CELL GNSS:OK");
      }
#endif

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
#else
  state.set(STATE_NET_CONNECTED);
#endif
#endif
  return state.check(STATE_NET_CONNECTED);
}

/*******************************************************************************
  Initializing network, maintaining connection and doing transmissions
*******************************************************************************/
void telemetry(void* inst)
{
  uint8_t connErrors = 0;
  CStorageRAM store;
  store.init(SERIALIZE_BUFFER_SIZE);
  teleClient.reset();

  for (;;) {
    if (state.check(STATE_STANDBY)) {
      if (state.check(STATE_NET_READY)) {
        teleClient.shutdown();
      }
      state.clear(STATE_NET_READY | STATE_NET_CONNECTED);
      teleClient.reset();
      bufman.purge();

#if GNSS == GNSS_INTERNAL || GNSS == GNSS_EXTERNAL
      if (state.check(STATE_GPS_READY)) {
        Serial.println("GNSS OFF");
#if GNSS_ALWAYS_ON
        sys.gpsEnd(false);
#else
        sys.gpsEnd(true);
#endif
        state.clear(STATE_GPS_READY);
      }
      gd = 0;
#endif

      uint32_t t = millis();
      do {
        delay(1000);
      } while (state.check(STATE_STANDBY) && millis() - t < 1000L * PING_BACK_INTERVAL);
      if (state.check(STATE_STANDBY)) {
        // start ping
#if 0
#if GNSS == GNSS_EXTERNAL || GNSS == GNSS_INTERNAL
#if GNSS == GNSS_EXTERNAL
        if (sys.gpsBegin())
#else
        if (sys.gpsBegin(0))
#endif
        {
          state.set(STATE_GPS_READY);
          for (uint32_t t = millis(); millis() - t < 60000; ) {
            if (sys.gpsGetData(&gd)) {
              break;
            }
          }
        }
#endif
#endif
        if (initNetwork()) {
          Serial.print("Ping...");
          bool success = teleClient.ping();
          Serial.println(success ? "OK" : "NO");
        }
        teleClient.shutdown();
        state.clear(STATE_NET_READY | STATE_NET_CONNECTED);
      }
      continue;
    }
    
    if (!state.check(STATE_NET_CONNECTED)) {
      if (!initNetwork() || !teleClient.connect()) {
        teleClient.shutdown();
        state.clear(STATE_NET_READY | STATE_NET_CONNECTED);
        delay(30000);
        continue;
      }
    }

    connErrors = 0;
    teleClient.startTime = millis();

    for (;;) {
      CBuffer* buffer = bufman.getNewest();
      if (!buffer) {
        if (!state.check(STATE_WORKING)) break;
        delay(50);
        continue;
      }

      buffer->state = BUFFER_STATE_LOCKED;
#if SERVER_PROTOCOL == PROTOCOL_UDP
      store.header(devid);
#endif
      store.timestamp(buffer->timestamp);
      buffer->serialize(store);
      buffer->purge();
      store.tailer();
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

      teleClient.inbound();
      if (syncInterval > 10000 && millis() - teleClient.lastSyncTime > syncInterval) {
        Serial.println("Instable connection");
        connErrors++;
        timeoutsNet++;
      }
      if (connErrors > MAX_CONN_ERRORS_RECONNECT) {
        teleClient.net.close();
        if (!teleClient.connect()) {
          teleClient.shutdown();
          state.clear(STATE_NET_READY | STATE_NET_CONNECTED);
          break;
        }
      }

      if (deviceTemp >= COOLING_DOWN_TEMP) {
        // device too hot, cool down by pause transmission
        Serial.println("Overheat");
        delay(10000);
        bufman.purge();
      }

    }
  }
}

/*******************************************************************************
  Implementing stand-by mode
*******************************************************************************/
void standby()
{
  state.set(STATE_STANDBY);
#if STORAGE != STORAGE_NONE
  if (state.check(STATE_STORAGE_READY)) {
    logger.end();
  }
#endif
  state.clear(STATE_WORKING | STATE_OBD_READY | STATE_STORAGE_READY);
  // this will put co-processor into sleep mode
#if ENABLE_OLED
  oled.print("STANDBY");
  delay(1000);
  oled.clear();
#endif
  Serial.println("STANDBY");
  obd.enterLowPowerMode();
#if ENABLE_MEMS
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
#if ENABLE_MEMS
  mems->end();  
#endif
  ESP.restart();
#endif  
  state.clear(STATE_STANDBY);
  // this will wake up co-processor
  sys.resetLink();
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
  Serial.print("MHz FLASH:");
  Serial.print(getFlashSize() >> 10);
  Serial.println("MB");
  Serial.print("IRAM:");
  Serial.print(ESP.getHeapSize() >> 10);
  Serial.print("KB");
#if BOARD_HAS_PSRAM
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
#endif
  Serial.println();

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

  Serial.print("DEVICE ID:");
  Serial.println(devid);
#if ENABLE_OLED
  oled.print("DEVICE ID:");
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
    if (ledMode == 0) digitalWrite(PIN_LED, HIGH);

    // generate unique device ID
    genDeviceID(devid);

#if CONFIG_MODE_TIMEOUT
    configMode();
#endif

#if LOG_EXT_SENSORS == 1
    pinMode(PIN_SENSOR1, INPUT);
    pinMode(PIN_SENSOR2, INPUT);
#elif LOG_EXT_SENSORS == 2
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);
#endif

    // show system information
    showSysInfo();

#if ENABLE_OBD
    if (sys.begin()) {
      Serial.print("TYPE:");
      Serial.println(sys.devType);
      obd.begin(sys.link);
    }
#else
    sys.begin(false, true);
#endif

#if ENABLE_MEMS
  if (!state.check(STATE_MEMS_READY)) {
    Serial.print("MEMS:");
    mems = new MPU9250;
    byte ret = mems->begin(ENABLE_ORIENTATION);
    if (ret) {
      state.set(STATE_MEMS_READY);
      Serial.println("MPU-9250");
    } else {
      mems->end();
      delete mems;
      mems = new ICM_20948_I2C;
      ret = mems->begin(ENABLE_ORIENTATION);
      if (ret) {
        state.set(STATE_MEMS_READY);
        Serial.println("ICM-20948");
      } else {
        Serial.println("NO");
      }
    }
  }
#endif

#if ENABLE_HTTPD
    IPAddress ip;
    if (serverSetup(ip)) {
      Serial.println("HTTPD:");
      Serial.println(ip);
#if ENABLE_OLED
      oled.println(ip);
#endif
    } else {
      Serial.println("HTTPD:NO");
    }
#endif

#if ENABLE_BLE
    // init BLE
    ble_init();
#endif

    state.set(STATE_WORKING);
    // initialize network and maintain connection
    subtask.create(telemetry, "telemetry", 2, 4096);
    // initialize components
    initialize();

    digitalWrite(PIN_LED, LOW);
}

void loop()
{
  // error handling
  if (!state.check(STATE_WORKING)) {
    standby();
    if (ledMode == 0) digitalWrite(PIN_LED, HIGH);
    initialize();
    digitalWrite(PIN_LED, LOW);
    return;
  }

  // collect and log data
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
