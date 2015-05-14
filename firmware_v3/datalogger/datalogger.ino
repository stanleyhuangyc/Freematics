/*************************************************************************
* Arduino OBD-II/G-Force Data Logger and Transmitter
* Distributed under GPL v2.0
* Copyright (c) 2013-15 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include <OBD.h>
#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <I2Cdev.h>
#include <MPU9150.h>
#include "config.h"
#if USE_SOFTSERIAL
#include <SoftwareSerial.h>
#endif
#include "datalogger.h"

// logger states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_FOUND 0x4
#define STATE_GPS_READY 0x8
#define STATE_MEMS_READY 0x10
#define STATE_SLEEPING 0x20

#if USE_SOFTSERIAL
#if VERBOSE && !ENABLE_DATA_OUT
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
SoftwareSerial SerialInfo(A8, A9); /* for BLE Shield on MEGA*/
#else
SoftwareSerial SerialInfo(A2, A3); /* for BLE Shield on UNO/leonardo*/
#endif
#else
#define SerialInfo SerialRF
#endif
#else
#define SerialInfo Serial
#endif

static uint32_t lastFileSize = 0;
static uint16_t fileIndex = 0;
static uint8_t attempts = 0;

static const byte PROGMEM pidTier1[]= {PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_THROTTLE};
static const byte PROGMEM pidTier2[] = {PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_FUEL_LEVEL, PID_DISTANCE};

#define TIER_NUM1 sizeof(pidTier1)
#define TIER_NUM2 sizeof(pidTier2)

#if USE_MPU6050 || USE_MPU9150
MPU6050 accelgyro;
#endif

class CLogger : public COBD, public CDataLogger
{
public:
    CLogger():state(0) {}
    void setup()
    {
        state = 0;
#if USE_MPU6050 || USE_MPU9150
        Wire.begin();
        accelgyro.initialize();
        if (accelgyro.testConnection()) state |= STATE_MEMS_READY;
#endif

        if (init()) {
            state |= STATE_OBD_READY;
        }

#if USE_GPS
        if (initGPS()) {
            state |= STATE_GPS_FOUND;
        } else {
#if VERBOSE
            SerialInfo.println("No GPS");
            delay(3000);
#endif
        }
#endif
    }
#if USE_GPS
    bool isDataChanged(byte index, byte value)
    {
        static byte prevData[8] = {0};
        if (prevData[index] != value) {
            prevData[index] = value;
            return true;
        } else {
            return false;
        }
    }
    void logGPSData()
    {
        char buf[128];
        byte n = 0;
#if LOG_GPS_PARSED_DATA
        // issue the command to get parsed data
        // the return data is in following format:
        // $GPS,date,time,lat,lon,altitude,speed,course,sat
        byte index;
        bool valid = false;
        write("ATGPS\r");
        dataTime = millis();
        for (;;) {
            if (available()) {
                char c = read();
#if VERBOSE
                SerialInfo.write(c);
#endif
                if (c == ',' || c == '>' || c <= 0x0d) {
                    buf[n] = 0;
                    if (!valid) {
                        // check header
                        valid = strcmp(buf + n - 4, "$GPS") == 0;
                        index = 0;
                    } else {
                        switch (index) {
                        case 0:
                        case 1:
                            {
                                uint32_t value = (uint32_t)atol(buf);
                                if (isDataChanged(index, value)) {
                                    logData(index == 0 ? PID_GPS_DATE : PID_GPS_TIME, value);
                                }
                            }
                            break;
                        case 2:
                        case 3:
                            {
                                int32_t value = atol(buf);
                                if (isDataChanged(index, value)) {
                                    logData(index == 2 ? PID_GPS_LATITUDE : PID_GPS_LONGITUDE, value);
                                }
                            }
                            break;
                        case 4:
                        case 5:
                        case 6:
                        case 7:
                            {
                                int value = atoi(buf);
                                if (isDataChanged(index, value)) {
                                    switch (index) {
                                    case 4:
                                        logData(PID_GPS_ALTITUDE, value);
                                        break;
                                    case 5:
                                        logData(PID_GPS_SPEED, value);
                                        if (!(state & STATE_OBD_READY)) {
                                            logData(0x100 | PID_SPEED, value);
                                        }
                                        break;
                                    case 6:
                                        logData(PID_GPS_HEADING, value);
                                        break;
                                    case 7:
                                        logData(PID_GPS_SAT_COUNT, value);
                                        break;
                                    }
                                }
                            }
                            break;
                        }
                        index++;
                    }
                    n = 0;
                    if (c == '>') {
                        // prompt char received, discard following data
                        while (available()) read();
                        break;
                    }
                } else if (n < sizeof(buf) - 1) {
                    buf[n++] = c;
                }
            } else if (millis() - dataTime > 100) {
                // timeout
                break;
            }
        }
#endif
#if LOG_GPS_NMEA_DATA
        // issue the command to get NMEA data (one line per request)
        write("ATGRR\r");
        dataTime = millis();
        n = 0;
        for (;;) {
            if (available()) {
                char c = read();
                if (c == '>') {
                    // prompt char received
                    break;
                } else if (n < sizeof(buf) && c >= 32 && c <= 126) {
                    buf[n++] = c;
#if VERBOSE
                    SerialInfo.write(c);
#endif
                }
            } else if (millis() - dataTime > 100) {
                // timeout
                break;
            }
        }
        buf[n++] = '\r';
        buf[n] = 0;
        recordData(buf);
#if VERBOSE
        SerialInfo.println();
#endif
#endif
    }
#endif
#if USE_MPU6050 || USE_MPU9150
    void logMEMSData()
    {
        if (!(state & STATE_MEMS_READY))
            return;

        int16_t ax, ay, az;
        int16_t gx, gy, gz;
#if USE_MPU9150
        int16_t mx, my, mz;
        accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
#else
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
#endif

        dataTime = millis();
#if VERBOSE
        SerialInfo.print("A:");
        SerialInfo.print(ax);
        SerialInfo.print('/');
        SerialInfo.print(ay);
        SerialInfo.print('/');
        SerialInfo.print(az);
        SerialInfo.print(" G:");
        SerialInfo.print(gx);
        SerialInfo.print('/');
        SerialInfo.print(gy);
        SerialInfo.print('/');
        SerialInfo.print(gz);
#if USE_MPU9150
        SerialInfo.print(" M:");
        SerialInfo.print(mx);
        SerialInfo.print('/');
        SerialInfo.print(my);
        SerialInfo.print('/');
        SerialInfo.println(mz);
#else
        SerialInfo.println();
#endif
#endif
        // log x/y/z of accelerometer
        logData(PID_ACC, ax >> 4, ay >> 4, az >> 4);
        // log x/y/z of gyro meter
        logData(PID_GYRO, gx >> 4, gy >> 4, gz >> 4);
#if USE_MPU9150
        // log x/y/z of gyro meter
        logData(PID_COMPASS, mx >> 4, my >> 4, mz >> 4);
#endif
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
        if (index) {
            if (sdfile.print("#FREEMATICS\r") > 0) {
              state |= STATE_SD_READY;
            } else {
              index = 0; 
            }
        }
#if VERBOSE
        SerialInfo.print("File ID: ");
        SerialInfo.println(index);
        delay(3000);
#endif
        return index;
    }
    bool initSD()
    {
        state &= ~STATE_SD_READY;
        pinMode(SS, OUTPUT);
        Sd2Card card;
        if (card.init(SPI_HALF_SPEED, SD_CS_PIN)) {
#if VERBOSE
            const char* type;
            switch(card.type()) {
            case SD_CARD_TYPE_SD1:
                type = "SD1";
                break;
            case SD_CARD_TYPE_SD2:
                type = "SD2";
                break;
            case SD_CARD_TYPE_SDHC:
                type = "SDHC";
                break;
            default:
                type = "SDx";
            }

            SerialInfo.print("SD type: ");
            SerialInfo.print(type);

            SdVolume volume;
            if (!volume.init(card)) {
                SerialInfo.println(" No FAT!");
                return false;
            }

            uint32_t volumesize = volume.blocksPerCluster();
            volumesize >>= 1; // 512 bytes per block
            volumesize *= volume.clusterCount();
            volumesize >>= 10;
            SerialInfo.print(" SD size: ");
            SerialInfo.print((int)((volumesize + 511) / 1000));
            SerialInfo.println("GB");
            delay(3000);
#endif
        }

        return SD.begin(SD_CS_PIN);
    }
    void flushData()
    {
        // flush SD data every 1KB
        if (dataSize - lastFileSize >= 1024) {
#if VERBOSE
            // display logged data size
            SerialInfo.print(dataSize);
            SerialInfo.println(" bytes");
#endif
#if MAX_FILE_SIZE
            if (dataSize >= 1024L * MAX_FILE_SIZE) {
              closeFile();
              if (openLogFile() == 0) {
                  state &= ~STATE_SD_READY;
              }
            } else {
              flushFile();
            }
#endif
            lastFileSize = dataSize;
        }
    }
#endif
    void reconnect()
    {
#if ENABLE_DATA_LOG
        closeFile();
#endif
        state &= ~STATE_OBD_READY;
        state |= STATE_SLEEPING;
        //digitalWrite(SD_CS_PIN, LOW);
#if VERBOSE
        SerialInfo.print("Retry");
#endif
        while (!init()) {
#if VERBOSE
            SerialInfo.write('.');
#endif
        }
        state &= ~STATE_SLEEPING;
#if ENABLE_DATA_LOG
        if (openLogFile() == 0) {
            state &= ~STATE_SD_READY;
        }
#endif
    }
    bool logOBDData(byte pid)
    {
        int value;

        // send a query command
        sendQuery(pid);
        pid = 0; // this lets PID also obtained and filled from response
        // receive and parse the response
        char buffer[OBD_RECV_BUF_SIZE];
        char* data = getResponse(pid, buffer);
        dataTime = millis();
        if (!data) {
            recover();
            errors++;
            return false;
        }
#if VERBOSE
        SerialInfo.println(data);
#endif
        value = normalizeData(pid, data);
        showData(pid, value);
        // log data to SD card
        logData(0x100 | pid, value);
        errors = 0;
        return true;
    }
    void showData(byte pid, int value)
    {
#if VERBOSE
        SerialInfo.print('[');
        SerialInfo.print(millis());
        SerialInfo.print("][");
        SerialInfo.print(pid, HEX);
        SerialInfo.print("]=");
        SerialInfo.println(value);
#endif
    }
    byte state;
};

static CLogger logger;

void setup()
{
#if VERBOSE
    SerialInfo.begin(STREAM_BAUDRATE);
    SerialInfo.println("Freematics");
    delay(3000);
#endif

    logger.begin();
    logger.initSender();
#if ENABLE_DATA_LOG
    delay(500);
    logger.initSD();
    logger.openLogFile();
#endif

    logger.setup();
}

void loop()
{
    static uint32_t lastTime = 0;
    uint32_t t = millis();

    if (logger.state & STATE_OBD_READY) {
        static byte index1 = 0;
        static byte index2 = 0;
        byte pid = pgm_read_byte(pidTier1 + index1++);
        logger.logOBDData(pid);
        if (index1 == TIER_NUM1) {
            index1 = 0;
            pid = pgm_read_byte(pidTier2 + index2);
            logger.logOBDData(pid);
            index2 = (index2 + 1) % TIER_NUM2;
        }
        if (logger.errors >= 2) {
            logger.reconnect();
        }
    } else if (!OBD_ATTEMPTS || attempts <= OBD_ATTEMPTS - 1) {
        if (logger.init()) {
            logger.state |= STATE_OBD_READY;
        }
        attempts++;
    }

#if USE_MPU6050 || USE_MPU9150
    logger.logMEMSData();
#endif

#if USE_GPS
    if (logger.state & STATE_GPS_FOUND) {
        logger.logGPSData();
    }
#endif

    if (millis() - lastTime > LONG_INTERVAL * 1000) {
        // log slowly changing data
        int v = logger.getVoltage();
        logger.logData(PID_BATTERY_VOLTAGE, v);
        logger.logData(PID_DATA_SIZE, logger.dataSize);
        lastTime = millis();
    }

#if ENABLE_DATA_LOG
    logger.flushData();
#endif

#if MIN_LOOP_TIME
    while (millis() - t < MIN_LOOP_TIME);
#endif
}
