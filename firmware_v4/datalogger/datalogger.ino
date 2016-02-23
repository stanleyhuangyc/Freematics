/*************************************************************************
* OBD-II/MEMS/GPS Data Logging Sketch for Freematics ONE
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one for more information
* Developed by Stanley Huang <stanleyhuangyc@gmail.com>
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <FreematicsONE.h>
#include "config.h"
#if ENABLE_DATA_LOG
#include <SD.h>
#endif
#include "datalogger.h"

// states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_FOUND 0x4
#define STATE_GPS_READY 0x8
#define STATE_MEMS_READY 0x10
#define STATE_FILE_READY 0x20
#define STATE_SLEEPING 0x40

#if VERBOSE && !ENABLE_DATA_OUT
#define SerialInfo Serial
#else
#define SerialInfo SerialRF
#endif

static uint8_t lastFileSize = 0;
static uint16_t fileIndex = 0;

uint16_t MMDD = 0;
uint32_t UTC = 0;

static const byte PROGMEM pidTier1[]= {PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_THROTTLE};
static const byte PROGMEM pidTier2[] = {PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_DISTANCE};

#define TIER_NUM1 sizeof(pidTier1)
#define TIER_NUM2 sizeof(pidTier2)

#if USE_MPU6050
MEMS_DATA mems;
#endif

static const byte PROGMEM parts[][3] = {
  {'S','D'},
  {'A','C','C'},
  {'O','B','D'},
  {'G','P','S'},
};

typedef enum {
  PART_SD = 0,
  PART_MEMS,
  PART_OBD,
  PART_GPS,
} PART_ID;

class ONE : public COBDSPI, public CDataLogger
{
public:
    ONE():state(0) {}
    void showStatus(byte partID, bool OK)
    {
#if ENABLE_DATA_OUT
      char buf[4];
      memcpy_P(buf, parts[partID], 3);
      buf[3] = 0;
      SerialRF.print(buf);
      SerialRF.print(' ');
      SerialRF.println(OK ? "OK" : "NO");
#endif
    }
    void setup()
    {
        bool success;
        state = 0;
        
        begin();

#if ENABLE_DATA_LOG
        uint16_t volsize = initSD();
        if (volsize) {
          SerialRF.print("SD ");
          SerialRF.print(volsize);
          SerialRF.println("MB");
        } else {
          showStatus(PART_SD, false);
        }
#endif

#if USE_MPU6050
        Wire.begin();
        if (success = memsInit()) {
          state |= STATE_MEMS_READY;
        }
        showStatus(PART_MEMS, success);
#endif

        if (success = init()) {
            state |= STATE_OBD_READY;
        }
        showStatus(PART_OBD, success);

#if USE_GPS
        delay(500);
        if (success = initGPS(GPS_SERIAL_BAUDRATE)) {
            state |= STATE_GPS_FOUND;
        }
        showStatus(PART_GPS, success);
#endif

        delay(1000);
    }
#if USE_GPS
    void logGPSData()
    {
#if LOG_GPS_NMEA_DATA
        // issue the command to get NMEA data (one line per request)
        char buf[255];
        byte n = getGPSRawData(buf, sizeof(buf));
        if (n) {
            dataTime = millis();
            // strip heading $GPS and ending \r\n
            logData(buf + 4, n - 6);
        }
#endif
#if LOG_GPS_PARSED_DATA
        // issue the command to get parsed GPS data
        GPS_DATA gd = {0};
        if (getGPSData(&gd)) {
            dataTime = millis();
            if (gd.time && gd.time != UTC) {
              byte day = gd.date / 10000;
              if (MMDD % 100 != day) {
                logData(PID_GPS_DATE, gd.date);
              }
              logData(PID_GPS_TIME, gd.time);
              logData(PID_GPS_LATITUDE, gd.lat);
              logData(PID_GPS_LONGITUDE, gd.lng);
              logData(PID_GPS_ALTITUDE, gd.alt);
              logData(PID_GPS_SPEED, gd.speed);
              logData(PID_GPS_SAT_COUNT, gd.sat);
              if (UTC == 0) {
                // first GPS data arrived
                // to re-create a new date/time named log file
                closeFile();
                state &= ~STATE_FILE_READY;
              }
              // save current date in MMDD format
              unsigned int DDMM = gd.date / 100;
              UTC = gd.time;
              MMDD = (DDMM % 100) * 100 + (DDMM / 100);
              // set GPS ready flag
              state |= STATE_GPS_READY;
            }
        }
#endif
    }
#endif
    void logMEMSData()
    {
        // log the loaded MEMS data
        logData(PID_ACC, mems.value.x_accel / ACC_DATA_RATIO, mems.value.y_accel / ACC_DATA_RATIO, mems.value.z_accel / ACC_DATA_RATIO);
    }
#if ENABLE_DATA_LOG
    uint16_t initSD()
    {
        state &= ~STATE_SD_READY;
        pinMode(SS, OUTPUT);
        Sd2Card card;
        uint32_t volumesize = 0;
        if (card.init(SPI_HALF_SPEED, SD_CS_PIN)) {
            SdVolume volume;
            if (volume.init(card)) {
              volumesize = volume.blocksPerCluster();
              volumesize >>= 1; // 512 bytes per block
              volumesize *= volume.clusterCount();
              volumesize /= 1000;
            }
        }
        if (SD.begin(SD_CS_PIN)) {
          state |= STATE_SD_READY;
          return volumesize; 
        } else {
          return 0;
        }
    }
    void flushData()
    {
        // flush SD data every 1KB
        byte dataSizeKB = dataSize >> 10;
        if (dataSizeKB != lastFileSize) {
#if VERBOSE
            // display logged data size
            SerialInfo.print(dataSize);
            SerialInfo.println(" bytes");
#endif
            flushFile();
            lastFileSize = dataSizeKB;
#if MAX_LOG_FILE_SIZE
            if (dataSize >= 1024L * MAX_LOG_FILE_SIZE) {
              closeFile();
              state &= ~STATE_FILE_READY;
            }
#endif
        }
    }
#endif
    void reconnect()
    {
        if (init()) {
          // reconnected
          return; 
        }
        SerialRF.println("Sleeping");
#if ENABLE_DATA_LOG
        closeFile();
#endif
        state &= ~(STATE_OBD_READY | STATE_GPS_READY | STATE_GPS_FOUND);
        // cut off GPS power
        initGPS(0);
        // check if OBD is accessible again
        for (;;) {
            int value;
            if (read(PID_RPM, value))
                break;
            sleep(2);
        }
        // reset device
        void(* resetFunc) (void) = 0; //declare reset function at address 0
        resetFunc();
    }
    bool logOBDData(byte pid)
    {
        int value;
        if (!read(pid, value)) {
            // error occurred
            Serial.println("Error");
            errors++;
            return false;
        }
        dataTime = millis();
        logData((uint16_t)pid | 0x100, value);
        errors = 0;
        return true;
    }
    void dataIdleLoop()
    {
      // do something while waiting for data on SPI
#if USE_MPU6050
      if (state & STATE_MEMS_READY) {
        memsRead(&mems);
      }
#endif
    }
    byte state;
};

static ONE one;

void setup()
{
#if VERBOSE
    SerialInfo.begin(STREAM_BAUDRATE);
#endif
    one.initSender();
    one.setup();
}

void loop()
{
    if (!(one.state & STATE_FILE_READY) && (one.state & STATE_SD_READY)) {
      if (one.state & STATE_GPS_FOUND) {
        // GPS connected
        if (one.state & STATE_GPS_READY) {
          uint32_t dateTime = (uint32_t)MMDD * 10000 + UTC / 10000;
          if (one.openFile(dateTime) != 0) {
            UTC = 0;
            MMDD = 0;
            one.state |= STATE_FILE_READY;
          }
        } else {
          delay(3000); 
        }
      } else {
        // no GPS connected 
        if (one.openFile(0) != 0) {
          one.state |= STATE_FILE_READY;
        }
      }
    }
    if (one.state & STATE_OBD_READY) {
        static byte index2 = 0;
        for (byte n = 0; n < TIER_NUM1; n++) {
          byte pid = pgm_read_byte(pidTier1 + n);
          one.logOBDData(pid);
        }
        byte pid = pgm_read_byte(pidTier2 + index2);
        one.logOBDData(pid);
        index2 = (index2 + 1) % TIER_NUM2;
        if (one.errors >= 10) {
            one.reconnect();
        }
    } else if (!OBD_ATTEMPT_TIME || millis() < OBD_ATTEMPT_TIME * 1000) {
        if (one.init()) {
            one.state |= STATE_OBD_READY;
        }
    }
#if USE_MPU6050
    if (one.state & STATE_MEMS_READY) {
      one.logMEMSData();
    }
#endif
#if USE_GPS
    if (one.state & STATE_GPS_FOUND) {
      one.logGPSData();
    }
#endif
#if ENABLE_DATA_LOG
    one.flushData();
#endif
}
