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

#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F\r"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C\r"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F\r"

static uint16_t lastFileSize = 0;
static uint16_t fileIndex = 0;
static uint8_t attempts = 0;

static byte pidTier1[]= {PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_THROTTLE};
static byte pidTier2[] = {PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_AMBIENT_TEMP, PID_FUEL_LEVEL, PID_BAROMETRIC, PID_DISTANCE, PID_RUNTIME};

#define TIER_NUM1 sizeof(pidTier1)
#define TIER_NUM2 sizeof(pidTier2)

#if USE_MEMS
MPU6050 accelgyro;
#endif

class CLogger : public COBD, public CDataLogger
{
public:
    CLogger():state(0) {}
    void setup()
    {
        state = 0;
#if USE_MEMS
        Wire.begin();
        accelgyro.initialize();
        if (accelgyro.testConnection()) state |= STATE_MEMS_READY;
#endif

#if USE_GPS
        if (initGPS()) {
            state |= STATE_GPS_FOUND;
        }
#endif
    }
#if USE_GPS
    void logGPSData()
    {
        // issue the command to get NMEA data (one line per request)
#if LOG_GPS_PARSED_DATA
        // issue the command to get parsed data
        // the return data is in following format:
        // $GPS,date,time,lat,lon,altitude,speed,course,sat
        write("ATGPS\r");
        dataTime = millis();
        byte n = 0;
        byte tokens;
        bool valid = false;
        char buf[16];
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
                        tokens = 0;
                    } else {
                        switch (tokens) {
                        case 0:
                            logData(PID_GPS_DATE, (uint32_t)atol(buf));
                            break;
                        case 1:
                            logData(PID_GPS_TIME, (uint32_t)atol(buf));
                            break;
                        case 2:
                            logData(PID_GPS_LATITUDE, atol(buf));
                            break;
                        case 3:
                            logData(PID_GPS_LONGITUDE, atol(buf));
                            break;
                        case 4:
                            logData(PID_GPS_ALTITUDE, atoi(buf));
                            break;
                        case 5:
                            {
                                int speed = atoi(buf);
                                logData(PID_GPS_SPEED, speed);
                                if (!(state & STATE_OBD_READY)) {
                                    logData(0x100 | PID_SPEED, speed);
                                }
                            }
                            break;
                        case 6:
                            logData(PID_GPS_HEADING, atoi(buf));
                            break;
                        case 7:
                            logData(PID_GPS_SAT_COUNT, atoi(buf));
                            break;
                        }
                        tokens++;
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
        write("ATGRR\r");
        for (;;) {
            if (available()) {
                char c = read();
                if (c == '>') {
                    // prompt char received
                    break;
                } else {
#if VERBOSE
                    SerialInfo.write(c);
#endif
                    logData(c);

                }
            } else if (millis() - dataTime > 100) {
                // timeout
                break;
            }
        }
#endif
    }
#endif
#if USE_MEMS
    void logMEMSData()
    {
        if (!(state & STATE_MEMS_READY))
            return;

        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        int16_t mx, my, mz;

        accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

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
        SerialInfo.print(" M:");
        SerialInfo.print(mx);
        SerialInfo.print('/');
        SerialInfo.print(my);
        SerialInfo.print('/');
        SerialInfo.println(mz);
#endif
        // log x/y/z of accelerometer
        logData(PID_ACC, ax >> 4, ay >> 4, az >> 4);
        // log x/y/z of gyro meter
        logData(PID_GYRO, gx >> 4, gy >> 4, gz >> 4);
        // log x/y/z of gyro meter
        logData(PID_COMPASS, mx >> 4, my >> 4, mz >> 4);
    }
#endif
    void logOBDData()
    {
        static byte index1 = 0;
        static byte index2 = 0;

        if (index1 == TIER_NUM1) {
            index1 = 0;
            queryOBDData(pidTier2[index2]);
            index2 = (index2 + 1) % TIER_NUM2;
        } else {
            queryOBDData(pidTier1[index1++]);
        }
        if (errors >= 5) {
            reconnect();
        }
    }
#if ENABLE_DATA_LOG
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
#endif
        }

        if (!SD.begin(SD_CS_PIN)) {
            return false;
        }

        uint16_t index = openFile();
        if (index) {
#if VERBOSE
            SerialInfo.print("File ID: ");
            SerialInfo.println(index);
#endif
            state |= STATE_SD_READY;
        } else {
#if VERBOSE
            SerialInfo.println("File error");
#endif
        }
        return true;
    }
    void flushData()
    {
        // flush SD data every 1KB
        if ((state & STATE_SD_READY) && (uint16_t)dataSize - lastFileSize >= 1024) {
            flushFile();
            lastFileSize = (uint16_t)dataSize;
#if VERBOSE
            // display logged data size
            SerialInfo.print((int)(dataSize >> 10));
            SerialInfo.println("KB");
#endif
        }
    }
#endif
#if USE_GPS
    bool initGPS()
    {
        char buf[OBD_RECV_BUF_SIZE];
        // setting GPS baudrate
        write("ATBR2 38400\r");
        if (receive(buf) && strstr(buf, "OK")) {
            /*
            write("ATSGC ");
            write(PMTK_SET_NMEA_UPDATE_10HZ);
            receive();
            */
            return true;
        } else {
            return false;
        }
    }
#endif
    byte state;
private:
    void queryOBDData(byte pid)
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
        }
#if VERBOSE
        SerialInfo.println(data);
#endif
        value = normalizeData(pid, data);
        showData(pid, value);
        // log data to SD card
        logData(0x100 | pid, value);
        errors = 0;
    }
    void reconnect()
    {
#if ENABLE_DATA_LOG
        closeFile();
#endif
        state &= ~STATE_OBD_READY;
        state |= STATE_SLEEPING;
        //digitalWrite(SD_CS_PIN, LOW);
#if VERBOSE
        SerialInfo.print("Reconnecting ");
#endif
        while (!init()) {
#if VERBOSE
            SerialInfo.write('.');
#endif
        }
        state &= ~STATE_SLEEPING;
#if ENABLE_DATA_LOG
        if (!openFile()) {
            state &= ~STATE_SD_READY;
        }
#endif
    }
    void showData(byte pid, int value)
    {
#if VERBOSE
        SerialInfo.print('[');
        SerialInfo.print(millis());
        SerialInfo.print("] [");
        SerialInfo.print(pid, HEX);
        SerialInfo.print("]=");
        SerialInfo.println(value);
#endif
    }
};

static CLogger logger;

void setup()
{
#if VERBOSE
    SerialInfo.begin(STREAM_BAUDRATE);
    SerialInfo.println("Freematics");
#endif

    logger.begin();
    logger.initSender();
    logger.setup();

#if ENABLE_DATA_LOG
    logger.initSD();
#endif
}

void loop()
{
    uint32_t t = millis();

    if (logger.state & STATE_OBD_READY) {
        logger.logOBDData();
    } else if (attempts <= OBD_ATTEMPTS - 1) {
        if (logger.init()) {
            logger.state |= STATE_OBD_READY;
        }
        attempts++;
    }

#if USE_MEMS
    logger.logMEMSData();
#endif

#if USE_GPS
    if (logger.state & STATE_GPS_FOUND) {
        logger.logGPSData();
    }
#endif

    /*
    int v = logger.getVoltage();
    logger.logData(PID_VOLTAGE, v);
    */

#if ENABLE_DATA_LOG
    logger.flushData();
#endif

    while (millis() - t < MIN_LOOP_TIME);
}
