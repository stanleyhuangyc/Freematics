/*************************************************************************
* OBD-II/MEMS/GPS Data Logging Sketch for Freematics ONE
* Distributed under GPL v2.0
* Visit http://freematics.com for more information
* Developed by Stanley Huang <stanleyhuangyc@gmail.com>
*************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <I2Cdev.h>
#include <MPU9150.h>
#include <Narcoleptic.h>
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
#define STATE_SLEEPING 0x20

#if VERBOSE && !ENABLE_DATA_OUT
#define SerialInfo Serial
#else
#define SerialInfo SerialRF
#endif

void(* resetFunc) (void) = 0; //declare reset function at address 0

static uint8_t lastFileSize = 0;
static uint16_t fileIndex = 0;

static uint16_t lastUTC = 0;
static uint8_t lastGPSDay = 0;

static const byte PROGMEM pidTier1[]= {PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_THROTTLE};
static const byte PROGMEM pidTier2[] = {PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_DISTANCE};

#define TIER_NUM1 sizeof(pidTier1)
#define TIER_NUM2 sizeof(pidTier2)

#if USE_MPU6050 || USE_MPU9150
MPU6050 accelgyro;
#if USE_MPU9150
int16_t mems[3][3] = {0};
#else
int16_t mems[2][3] = {0};
#endif
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
        
#if ENABLE_DATA_LOG
        uint16_t volsize = initSD();
        if (volsize) {
          SerialRF.print("SD ");
          SerialRF.print(volsize);
          SerialRF.println("MB");
          success = openLogFile() != 0;
        }
        showStatus(PART_SD, success);
#endif

        begin();

#if USE_MPU6050 || USE_MPU9150
        Wire.begin();
        accelgyro.initialize();
        if (success = accelgyro.testConnection()) {
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
            if (lastUTC != (uint16_t)gd.time) {
              dataTime = millis();
              logData(PID_GPS_TIME, gd.time);
              byte day = gd.date / 10000;
              if (lastGPSDay != day) {
                logData(PID_GPS_DATE, gd.date);
                lastGPSDay = day;
              }
              logData(PID_GPS_LATITUDE, gd.lat);
              logData(PID_GPS_LONGITUDE, gd.lng);
              logData(PID_GPS_ALTITUDE, gd.alt);
              logData(PID_GPS_SPEED, gd.speed);
              logData(PID_GPS_SAT_COUNT, gd.sat);
              lastUTC = (uint16_t)gd.time;
            }
        }
#endif
    }
#endif
#if USE_MPU6050 || USE_MPU9150
    void loadMEMSData()
    {
        if (state & STATE_MEMS_READY) {
#if USE_MPU9150
          accelgyro.getMotion9(&mems[0][0], &mems[0][1], &mems[0][2], &mems[1][0], &mems[1][1], &mems[1][2], &mems[2][0], &mems[2][1], &mems[2][2]);
#else
          accelgyro.getMotion6(&mems[0][0], &mems[0][1], &mems[0][2], &mems[1][0], &mems[1][1], &mems[1][2]);
#endif
        }
    }
#endif
#if ENABLE_DATA_LOG
    int openLogFile()
    {
        uint16_t index = openFile();
        if (!index) {
            delay(1000);
            index = openFile();
        }
        /*
        if (index) {
            if (sdfile.println(ID_STR) > 0) {
              state |= STATE_SD_READY;
            } else {
              index = 0;
            }
        }
        */
#if VERBOSE
        SerialInfo.print("File ID: ");
        SerialInfo.println(index);
        delay(3000);
#endif
        return index;
    }
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
              lastUTC = 0;
              lastGPSDay = 0;
              if (openLogFile() == 0) {
                  state &= ~STATE_SD_READY;
              }
            }
#endif
        }
    }
#endif
    void standby()
    {
#if ENABLE_DATA_LOG
        closeFile();
#endif
        state &= ~(STATE_OBD_READY | STATE_GPS_READY | STATE_GPS_FOUND);
        // cut off GPS power
        initGPS(0);
        // check if OBD is accessible again
        byte n = 0;
        bool toReset = false;
        while (!init()) {
#if VERBOSE
            SerialInfo.write('.');
#endif
            Narcoleptic.delay(3000);
            if (n >= 20) {
              toReset = true;
            } else {
              n++; 
            }
        }
        if (toReset) resetFunc();
    }
    bool logOBDData(byte pid)
    {
        int value;
        if (!read(pid, value)) {
            // error occurred
            Serial.println("Error");
            recover();
            errors++;
            return false;
        }
        dataTime = millis();
        logData((uint16_t)pid | 0x100, value);
        errors = 0;
        // log the loaded MEMS data
#if USE_MPU9150
        if (state & STATE_MEMS_READY) {
          // assume PID_GYRO = PID_AAC + 1, PID_MAG = PID_AAC + 2
          for (byte n = 0; n < 3; n++) {
            logData(PID_ACC + n, mems[n][0] >> 4, mems[n][1] >> 4, mems[n][2] >> 4);
          }
        }
#else
        if (state & STATE_MEMS_READY) {
          for (byte n = 0; n < 2; n++) {
            logData(PID_ACC + n, mems[n][0] >> 4, mems[n][1] >> 4, mems[n][2] >> 4);
          }
        }
#endif
        return true;
    }
    void dataIdleLoop()
    {
      // do something while waiting for data on SPI
      if (m_state != OBD_CONNECTED) return;
#if ENABLE_DATA_LOG
      flushData();
#endif
#if USE_MPU6050 || USE_MPU9150
      loadMEMSData();
#endif
    }
    void xbSend(const char* cmd)
    {
      setTarget(TARGET_GSM);
      write(cmd);
    }
  byte xbRecv(char* response, byte bufsize, int timeout = 1000, const char* expected = 0)
  {
      uint32_t t = millis();
      do {
        setTarget(TARGET_OBD);
        write("ATGRD\r");
        byte n = receive(response, bufsize, timeout);
        if (n > 0) {
          if (!expected || strstr(response, expected))
            return n;
        }
      } while (millis() - t < timeout);
      return 0;
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
            one.standby();
        }
    } else if (!OBD_ATTEMPT_TIME || millis() < OBD_ATTEMPT_TIME * 1000) {
        if (one.init()) {
            one.state |= STATE_OBD_READY;
        }
    }
#if USE_GPS
    if (one.state & STATE_GPS_FOUND) {
        one.logGPSData();
    }
#endif
}
