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
#include <FreematicsONE.h>
#include "config.h"
#include "datalogger.h"

// states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2

#define STATE_GPS_FOUND 0x4
#define STATE_GPS_READY 0x8
#define STATE_MEMS_READY 0x10
#define STATE_FILE_READY 0x20

uint16_t MMDD = 0;
uint32_t UTC = 0;

class ONE : public COBDSPI, public CDataLogger
{
public:
    ONE():state(0) {}
    void setup()
    {
      Serial.print("OBD ");
      if (init()) {
        Serial.println("OK");
      } else {
        Serial.println("NO");
        reconnect();
      }
      state |= STATE_OBD_READY;

#if USE_GPS
      Serial.print("GPS ");
      if (gpsInit(GPS_SERIAL_BAUDRATE)) {
        state |= STATE_GPS_FOUND;
        Serial.println("OK");
        //waitGPS();
      } else {
        Serial.println("NO");
      }
#endif

#if ENABLE_DATA_LOG
      if (!(state & STATE_SD_READY)) {
        Serial.print("SD ");
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
      char buffer[128];
      if ((state & STATE_OBD_READY) && getVIN(buffer, sizeof(buffer))) {
        Serial.print("VIN:");
        Serial.println(buffer);
      }
    }
#if USE_GPS
    void logGPSData()
    {
        // issue the command to get parsed GPS data
        GPS_DATA gd = {0};
        if (gpsGetData(&gd)) {
            dataTime = millis();
            if (gd.time && gd.time != UTC) {
              byte day = gd.date / 10000;
              if (MMDD % 100 != day) {
                log(PID_GPS_DATE, gd.date);
              }
              log(PID_GPS_TIME, gd.time);
              log(PID_GPS_LATITUDE, gd.lat);
              log(PID_GPS_LONGITUDE, gd.lng);
              log(PID_GPS_ALTITUDE, gd.alt);
              log(PID_GPS_SPEED, gd.speed);
              log(PID_GPS_SAT_COUNT, gd.sat);
              // save current date in MMDD format
              unsigned int DDMM = gd.date / 100;
              UTC = gd.time;
              MMDD = (DDMM % 100) * 100 + (DDMM / 100);
              // set GPS ready flag
              state |= STATE_GPS_READY;
            }
        }
    }
    void waitGPS()
    {
        int elapsed = 0;
        GPS_DATA gd = {0};
        for (uint32_t t = millis(); millis() - t < 300000;) {
          int t1 = (millis() - t) / 1000;
          if (t1 != elapsed) {
            Serial.print("Waiting for GPS (");
            Serial.print(elapsed);
            Serial.println(")");
            elapsed = t1;
          }
          // read parsed GPS data
          if (gpsGetData(&gd) && gd.sat != 0 && gd.sat != 255) {
            Serial.print("SAT:");
            Serial.println(gd.sat);
            break;
          }
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
        state &= ~(STATE_OBD_READY | STATE_GPS_READY);
        Serial.println("Standby");
        // put OBD chips into low power mode
        while (!init()) {
          Serial.print('.');
          delay(10000);
        }
        Serial.println("Wakeup");
        setup();
    }
    byte state;
};

ONE one;
#if USE_MEMS
MPU9250_ACC mems;
#endif

void setup()
{
    Serial.begin(115200);
    Serial.println("Freematics ONE");
    delay(1000);
    byte ver = one.begin();
    Serial.print("Firmware Ver. ");
    Serial.println(ver);

#if USE_MEMS
    Serial.print("MEMS ");
    if (mems.memsInit()) {
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
        one.logGPSData();
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
      one.sleep(1000);
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
          // process MEMS data every time a PID is read to ensure data frequency
          float acc[3];
          bool updated = mems.memsRead(acc);
          if (updated) {
            one.log(PID_ACC, (int16_t)(acc[0] * 100), (int16_t)(acc[1] * 100), (int16_t)(acc[2] * 100));
          }
#endif
        }
        if (one.errors >= 3) {
            one.reset();
            one.reconnect();
        }
    } else {
      if (!OBD_ATTEMPT_TIME || millis() < OBD_ATTEMPT_TIME * 1000) {
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
      one.logGPSData();
    }
#endif

#if !ENABLE_DATA_OUT
    uint32_t t = millis();
    Serial.print(one.dataCount);
    Serial.print(" samples ");
    Serial.print((float)one.dataCount * 1000 / t, 1);
    Serial.print(" sps ");
#if ENABLE_DATA_LOG
    uint32_t fileSize = sdfile.size();
    one.flushData(fileSize);
    Serial.print(fileSize);
    Serial.print(" bytes");
#endif
    Serial.println();
#endif
}
