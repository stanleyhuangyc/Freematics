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

#if USE_MPU6050 || USE_MPU9250
long accSum[3]; // sum of accelerometer x/y/z data
byte accCount = 0; // count of accelerometer readings
int temp; // device temperature (in 0.1 celcius degree)
#endif

class ONE : public COBDSPI, public CDataLogger
#if USE_MPU6050
,public CMPU6050
#elif USE_MPU9250
,public CMPU9250
#endif
{
public:
    ONE():state(0) {}
    void setup()
    {
        state = 0;
        
        delay(1000);
        begin();
        SerialRF.print("Firmware Ver. ");
        SerialRF.println(version);

#if ENABLE_DATA_LOG
        SerialRF.print("SD ");
        uint16_t volsize = initSD();
        if (volsize) {
          SerialRF.print(volsize);
          SerialRF.println("MB");
        } else {
          SerialRF.println("NO");
        }
#endif

#if USE_MPU6050 || USE_MPU9250
        Wire.begin();
#if USE_MPU6050
        SerialRF.print("MPU6050 ");
#else
        SerialRF.print("MPU9250 ");
#endif
        if (memsInit()) {
          state |= STATE_MEMS_READY;
          SerialRF.println("OK");
        } else {
          SerialRF.println("NO");
        }
#endif

        SerialRF.print("OBD ");
        if (init()) {
          state |= STATE_OBD_READY;
          SerialRF.println("OK");
        } else {
          SerialRF.println("NO");
        }

#if USE_GPS
        delay(100);
        SerialRF.print("GPS ");
        if (initGPS(GPS_SERIAL_BAUDRATE)) {
          state |= STATE_GPS_FOUND;
          SerialRF.println("OK");
        } else {
          SerialRF.println("NO");
        }
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
        // try to re-connect to OBD
        SerialRF.println("Reconnecting");
        for (byte n = 0; n < 3; n++) {
          if (init()) {
            // reconnected
            return; 
          }
          delay(1000);
        }
#if ENABLE_DATA_LOG
        closeFile();
#endif
        // turn off GPS power
        initGPS(0);
        state &= ~(STATE_OBD_READY | STATE_GPS_READY | STATE_MEMS_READY);
        state |= STATE_SLEEPING;
        // check if OBD is accessible again
        SerialRF.println("Sleeping");
        for (;;) {
            int value;
            Serial.print('.');
            if (readPID(PID_SPEED, value)) {
              // a successful readout
              break;
            }
            enterLowPowerMode();
            // deep sleep for 8 seconds
            sleep(8);
            leaveLowPowerMode();
        }
        // reset device
        void(* resetFunc) (void) = 0; //declare reset function at address 0
        resetFunc();
    }
#if USE_MPU6050 || USE_MPU9250
    void dataIdleLoop()
    {
      // do something while waiting for data on SPI
      if (state & STATE_MEMS_READY) {
        // load accelerometer and temperature data
        int acc[3] = {0};
        memsRead(acc, 0, 0, &temp);
        if (accCount < 250) {
          accSum[0] += acc[0];
          accSum[1] += acc[1];
          accSum[2] += acc[2];
          accCount++;
        }
      } else {
        delay(10);
      }
    }
#endif
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
#if ENABLE_DATA_LOG
    if (!(one.state & STATE_FILE_READY) && (one.state & STATE_SD_READY)) {
      if (one.state & STATE_GPS_FOUND) {
        // GPS connected
        if (one.state & STATE_GPS_READY) {
          uint32_t dateTime = (uint32_t)MMDD * 10000 + UTC / 10000;
          if (one.openFile(dateTime) != 0) {
            SerialRF.print("FILE ");
            SerialRF.println(dateTime);
            MMDD = 0;
            one.state |= STATE_FILE_READY;
          } else {
            SerialRF.println("File error");
          }
        }
      } else {
        // no GPS connected 
        int index = one.openFile(0);
        if (index != 0) {
          one.state |= STATE_FILE_READY;
          SerialRF.print("FILE ");
          SerialRF.println(index);
        } else {
          SerialRF.println("File error");
        }
      }
    }
#endif
    if (one.state & STATE_OBD_READY) {
        static byte index2 = 0;
        const byte pids[]= {PID_RPM, PID_SPEED, PID_THROTTLE, PID_ENGINE_LOAD};
        int values[sizeof(pids)];
        // read multiple OBD-II PIDs
        if (one.readPID(pids, sizeof(pids), values) == sizeof(pids)) {
          one.dataTime = millis();
          for (byte n = 0; n < sizeof(pids); n++) {
            one.logData((uint16_t)pids[n] | 0x100, values[n]);
          }
        }
        static byte lastSec = 0;
        const byte pids2[] = {PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_DISTANCE};
        byte sec = (uint8_t)(millis() >> 10);
        if (sec != lastSec) {
          // goes in every other second
          int value;
          byte pid = pids2[index2 = (index2 + 1) % (sizeof(pids2))];
          // read single OBD-II PID
          if (one.isValidPID(pid) && one.readPID(pid, value)) {
            one.dataTime = millis();
            one.logData((uint16_t)pid | 0x100, value);
            lastSec = sec;
          }
        }
        if (one.errors >= 10) {
            one.reconnect();
        }
    } else {
      if (!OBD_ATTEMPT_TIME || millis() < OBD_ATTEMPT_TIME * 1000) {
        if (one.init()) {
            one.state |= STATE_OBD_READY;
        }
      } else {
        one.dataIdleLoop();
      }
    }
    
    // log battery voltage (from voltmeter), data in 0.01v
    int v = one.getVoltage() * 100;
    one.dataTime = millis();
    one.logData(PID_BATTERY_VOLTAGE, v);
    
#if USE_MPU6050 || USE_MPU9250
    if ((one.state & STATE_MEMS_READY) && accCount) {
       // log the loaded MEMS data
      one.logData(PID_ACC, accSum[0] / accCount / ACC_DATA_RATIO, accSum[1] / accCount / ACC_DATA_RATIO, accSum[2] / accCount / ACC_DATA_RATIO);
      one.logData(PID_MEMS_TEMP, temp);
      accSum[0] = 0;
      accSum[1] = 0;
      accSum[2] = 0;
      accCount = 0;
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
