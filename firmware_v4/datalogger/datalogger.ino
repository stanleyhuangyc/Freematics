/*************************************************************************
* OBD-II/MEMS/GPS Data Logging Sketch for Freematics ONE
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one for more information
* Developed by Stanley Huang <support@freematics.com.au>
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*************************************************************************/

#include <SPI.h>
#include <SD.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <FreematicsONE.h>
#include "config.h"
#include "datalogger.h"

// states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_FOUND 0x4
#define STATE_CELL_GPS_FOUND 0x8
#define STATE_GPS_READY 0x10
#define STATE_MEMS_READY 0x20
#define STATE_FILE_READY 0x40

uint16_t MMDD = 0;
uint32_t UTC = 0;

#if USE_MEMS
MPU9250_ACC mems;
float acc[3];
float accBias[3];
bool memsUpdated = false;

void calibrateMEMS()
{
    // MEMS data collected while sleeping
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

SIGNAL(WDT_vect) {
  wdt_disable();
  wdt_reset();
  WDTCSR &= ~_BV(WDIE);
}

void sleep(uint8_t wdt_period) {
	wdt_enable(wdt_period);
	wdt_reset();
	WDTCSR |= _BV(WDIE);
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_mode();
	wdt_disable();
	WDTCSR &= ~_BV(WDIE);
}

void resetMCU()
{
	void(* resetFunc) (void) = 0; //declare reset function @ address 0
	resetFunc();
}

class ONE : public COBDSPI, public CDataLogger
{
public:
    ONE():state(0) {}
    void setup()
    {
      Serial.print("OBD:");
      if (init()) {
        Serial.println("OK");
        state |= STATE_OBD_READY;
      } else {
        Serial.println("NO");
      }
#if USE_GPS
      Serial.print("GPS:");
      if (gpsInit(GPS_SERIAL_BAUDRATE)) {
        state |= STATE_GPS_FOUND;
        Serial.println("OK");
      } else {
        Serial.println("NO");
      }
#endif
#if USE_CELL_GPS
      if (!(state & STATE_GPS_FOUND)) {
        Serial.print("CELL GPS:");
        if (cellInit()) {
          Serial.println("OK");
          state |= STATE_CELL_GPS_FOUND;
        } else {
          Serial.println("NO");
        }
      }
#endif

#if ENABLE_DATA_LOG
      if (!(state & STATE_SD_READY)) {
        Serial.print("SD:");
        pinMode(SD_CS_PIN, OUTPUT);
        if (SD.begin(SD_CS_PIN)) {
          Serial.println("OK");
          state |= STATE_SD_READY;
        } else {
          Serial.println("NO");
        }
      }
#endif

      // retrieve VIN
      if (state & STATE_OBD_READY) {
        char buffer[128];
        if (getVIN(buffer, sizeof(buffer))) {
          Serial.print("VIN:");
          Serial.println(buffer);
        }
      }

      calibrateMEMS();
    }
    void logLocationData(GPS_DATA* gd)
    {
      if (gd->time && gd->time != UTC) {
        byte day = gd->date / 10000;
        dataTime = millis();
        if (MMDD % 100 != day) {
          log(PID_GPS_DATE, gd->date);
        }
        log(PID_GPS_TIME, gd->time);
        log(PID_GPS_LATITUDE, gd->lat);
        log(PID_GPS_LONGITUDE, gd->lng);
        log(PID_GPS_ALTITUDE, gd->alt);
        log(PID_GPS_SPEED, gd->speed);
        log(PID_GPS_SAT_COUNT, gd->sat);
        // save current date in MMDD format
        unsigned int DDMM = gd->date / 100;
        UTC = gd->time;
        MMDD = (DDMM % 100) * 100 + (DDMM / 100);
      }
    }
#if USE_GPS
    void processGPS()
    {
        // issue the command to get parsed GPS data
        GPS_DATA gd = {0};
        if (gpsGetData(&gd)) {
          logLocationData(&gd);
          // set GPS ready flag
          state |= STATE_GPS_READY;
        }
    }
#endif
#if USE_CELL_GPS
    void processCellGPS()
    {
      GPS_DATA gd;
      if (cellGetGPSInfo(gd)) {
        logLocationData(&gd);
        state |= STATE_GPS_READY;
      }
    }
#endif
#if ENABLE_DATA_LOG
    void flushData(uint32_t fileSize)
    {
      // flush SD data every 1KB
        static uint8_t lastFileSize = 0;
        byte dataSizeKB = fileSize >> 10;
        if (dataSizeKB != lastFileSize) {
            flushFile();
            lastFileSize = dataSizeKB;
#if MAX_DATA_FILE_SIZE
            if (fileSize >= 1024L * MAX_DATA_FILE_SIZE) {
              closeFile();
              state &= ~STATE_FILE_READY;
            }
#endif
        }
    }
#endif
    void reconnect()
    {
        Serial.println("Reconnect");
        if (init()) return;
        // try to re-connect to OBD
#if ENABLE_DATA_LOG
        closeFile();
#endif
        // turn off GPS power
#if USE_GPS
        gpsInit(0);
#endif
        state &= ~(STATE_OBD_READY | STATE_GPS_READY | STATE_FILE_READY);
        end();
        Serial.println("Standby");
#if USE_MEMS
        if (state & STATE_MEMS_READY) {
          calibrateMEMS();
          for (;;) {
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
#else
        for (;;) {
          sleep(WDTO_4S);
          float v = getVoltage();
          Serial.println(v);
          if (v >= JUMPSTART_VOLTAGE) break;
        }
#endif
        Serial.println("Wakeup");
        delay(100);
        resetMCU();
    }
    // library idle callback from sleep()
    void idleTasks()
    {
      memsUpdated = mems.read(acc);
    }
#if USE_CELL_GPS
    bool cellSendCommand(const char* cmd, char* buf, int bufsize, const char* expected = "\r\nOK", unsigned int timeout = 1000)
    {
      if (cmd) xbWrite(cmd);
      memset(buf, 0, bufsize);
      byte ret = xbReceive(buf, bufsize, timeout, &expected, 1);
      if (ret) {
        return true;
      } else {
        return false;
      }
    }
    bool cellInit()
    {
      xbBegin(115200);
      for (byte n = 0; n < 2; n++) {
        char buf[128];
        if (!cellSendCommand("AT\r", buf, sizeof(buf))) cellSendCommand(0, buf, sizeof(buf), "START", 5000);
        if (cellSendCommand("ATE0\r", buf, sizeof(buf)) && cellSendCommand("ATI\r", buf, sizeof(buf), "SIM5360")) {
          cellSendCommand("AT+CVAUXV=61\r", buf, sizeof(buf));
          cellSendCommand("AT+CVAUXS=1\r", buf, sizeof(buf));
          if (cellSendCommand("AT+CGPS?\r", buf, sizeof(buf), "+CGPS: 1")) return true;
          delay(2000);
          if (cellSendCommand("AT+CGPS=1,1\r", buf, sizeof(buf))) return true;
        }
        xbTogglePower();
      }
      return false;
    }
    long parseDegree(const char* s)
    {
      char *p;
      unsigned long left = atol(s);
      unsigned long tenk_minutes = (left % 100UL) * 100000UL;
      if ((p = strchr(s, '.')))
      {
        unsigned long mult = 10000;
        while (isdigit(*++p))
        {
          tenk_minutes += mult * (*p - '0');
          mult /= 10;
        }
      }
      return (left / 100) * 1000000 + tenk_minutes / 6;
    }
    bool cellGetGPSInfo(GPS_DATA& gd)
    {
      char *p;
      char buf[160];
      if (cellSendCommand("AT+CGPSINFO\r", buf, sizeof(buf), "+CGPSINFO:")) do {
        if (!(p = strchr(buf, ':'))) break;
        if (*(++p) == ',') break;
        gd.lat = parseDegree(p);
        if (!(p = strchr(p, ','))) break;
        if (*(++p) == 'S') gd.lat = -gd.lat;
        if (!(p = strchr(p, ','))) break;
        gd.lng = parseDegree(++p);
        if (!(p = strchr(p, ','))) break;
        if (*(++p) == 'W') gd.lng = -gd.lng;
        if (!(p = strchr(p, ','))) break;
        gd.date = atol(++p);
        if (!(p = strchr(p, ','))) break;
        gd.time = atol(++p);
        if (!(p = strchr(p, ','))) break;
        gd.alt = atoi(++p);
        if (!(p = strchr(p, ','))) break;
        gd.speed = atof(++p) * 100;
        if (!(p = strchr(p, ','))) break;
        gd.heading = atoi(++p);
        Serial.print("UTC:");
        Serial.print(gd.date);
        Serial.print(' ');
        Serial.print(gd.time);
        Serial.print(" LAT:");
        Serial.print(gd.lat);
        Serial.print(" LNG:");
        Serial.println(gd.lng);
        return true;
      } while (0);
      return false;
    }
#endif
    byte state;
};

ONE one;

void setup()
{
    // Disable the ADC
    ADCSRA = ADCSRA & B01111111;
    // Disable the analog comparator
    ACSR = B10000000;
    // Disable digital input buffers on all analog input pins
    DIDR0 = DIDR0 | B00111111;

    // change to 9600bps when using BLE
    Serial.begin(115200);
    
    byte ver = one.begin();
    Serial.print("Firmware:V");
    Serial.println(ver);

#if USE_MEMS
    Serial.print("MEMS:");
    if (mems.begin()) {
      one.state |= STATE_MEMS_READY;
      Serial.println("OK");
    } else {
      Serial.println("NO");
    }
#endif

    one.setup();
}

void loop()
{
#if ENABLE_DATA_LOG
    if (!(one.state & STATE_FILE_READY) && (one.state & STATE_SD_READY)) {
#if USE_GPS
      if (one.state & STATE_GPS_FOUND) {
        // GPS connected
        one.processGPS();
        if (one.state & STATE_GPS_READY) {
          uint32_t dateTime = (uint32_t)MMDD * 10000 + UTC / 10000;
          if (one.openFile(dateTime)) {
            MMDD = 0;
            one.state |= STATE_FILE_READY;
          }
        } else {
          Serial.println("Waiting for GPS...");
        }
      }
      else
#endif
      {
        // no GPS connected
        if (one.openFile(0)) {
          one.state |= STATE_FILE_READY;
        }
      }
      delay(1000);
      return;
    }
#endif
    if (one.state & STATE_OBD_READY) {
        byte pids[]= {PID_RPM, PID_SPEED, PID_THROTTLE, PID_ENGINE_LOAD};
        one.dataTime = millis();
        for (byte i = 0; i < sizeof(pids) / sizeof(pids[0]); i++) {
          int value;
          byte pid = pids[i];
          if (one.readPID(pid, value)) {
            one.log((uint16_t)pids[i] | 0x100, value);
          }
#if USE_MEMS
          if (memsUpdated) {
            one.log(PID_ACC, (int16_t)((acc[0] - accBias[0]) * 100), (int16_t)((acc[1] - accBias[1]) * 100), (int16_t)((acc[2] - accBias[2]) * 100));
            memsUpdated = false;
          }
#endif
        }
        if (one.errors >= 3) {
            one.reset();
            one.reconnect();
        }
    } else {
      if (!OBD_ATTEMPT_TIME || millis() < OBD_ATTEMPT_TIME * 1000L) {
        if (one.init()) {
            one.state |= STATE_OBD_READY;
        }
      }
    }

    // log battery voltage (from voltmeter), data in 0.01v
    int v = one.getVoltage() * 100;
    one.log(PID_BATTERY_VOLTAGE, v);
    
#if USE_GPS
    if (one.state & STATE_GPS_FOUND) {
      one.processGPS();
    }
#endif
#if USE_CELL_GPS
    if (one.state & STATE_CELL_GPS_FOUND) {
      one.processCellGPS();
    }
#endif

#if !ENABLE_DATA_OUT
    uint32_t t = millis();
    Serial.print(one.dataCount);
    Serial.print(" samples ");
    Serial.print((float)one.dataCount * 1000 / t, 1);
    Serial.print(" sps ");
#if ENABLE_DATA_LOG
    if (one.state & STATE_FILE_READY) {
      uint32_t fileSize = sdfile.size();
      one.flushData(fileSize);
      Serial.print(fileSize);
      Serial.print(" bytes");
    }
#endif
    Serial.println();
#endif
}
