/*************************************************************************
* Arduino OBD-II/G-Force Data Logger and Transmitter
* Distributed under GPL v2.0
* Copyright (c) 2013-14 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include <OBD.h>
#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <MPU6050.h>
#include "config.h"
#if USE_SOFTSERIAL
#include <SoftwareSerial.h>
#endif
#if USE_GPS && PARSE_GPS_DATA
#include <TinyGPSPlus.h>
#endif
#include "datalogger.h"

// logger states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_FOUND 0x4
#define STATE_GPS_READY 0x8
#define STATE_ACC_READY 0x10
#define STATE_SLEEPING 0x20

#if USE_SOFTSERIAL
#if VERBOSE && !ENABLE_DATA_OUT
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
SoftwareSerial SerialInfo(A8, A9); /* for BLE Shield on MEGA*/
#else
SoftwareSerial SerialInfo(A2, A3); /* for BLE Shield on UNO/leonardo*/
#endif
#endif
#else
#define SerialInfo Serial
#endif

#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F\r"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C\r"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F\r"

#if USE_GPS && PARSE_GPS_DATA
TinyGPSPlus gps;
#endif

static uint16_t lastFileSize = 0;
static uint16_t fileIndex = 0;
static uint32_t startTime = 0;
static uint16_t elapsed = 0;

static byte pidTier1[]= {PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_THROTTLE};
static byte pidTier2[] = {PID_TIMING_ADVANCE};
static byte pidTier3[] = {PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_AMBIENT_TEMP, PID_FUEL_LEVEL, PID_BAROMETRIC, PID_DISTANCE, PID_RUNTIME};

#define TIER_NUM1 sizeof(pidTier1)
#define TIER_NUM2 sizeof(pidTier2)
#define TIER_NUM3 sizeof(pidTier3)

class COBDLogger : public COBD, public CDataLogger
{
public:
    COBDLogger():state(0) {}
    void setup()
    {
        showStates();

#if USE_MPU6050
        Wire.begin();
        if (MPU6050_init() == 0) {
            state |= STATE_ACC_READY;
            showStates();
        }
#endif

        for (byte n = 0; n < 3; n++) {
            if (init()) {
                state |= STATE_OBD_READY;
                showStates();
                break;
            }
            showStates();
        }

#if USE_GPS
        // setting GPS baudrate
        write("ATBR2 38400\r");
        receive();

        /*
        write(PMTK_SET_NMEA_UPDATE_10HZ);
        receive();
        */
#endif

#if ENABLE_DATA_LOG
        uint16_t index = openFile();

#if VERBOSE
        if (index) {
            SerialInfo.print("File ID: ");
            SerialInfo.println(index);
        } else {
            SerialInfo.println("No log file");
        }
#endif

        // open file for logging
        if (!(state & STATE_SD_READY)) {
            if (checkSD()) {
                state |= STATE_SD_READY;
                showStates();
            }
        }
#endif
        //showECUCap();
        startTime = millis();
    }
    void loop()
    {
        logGPSData();

        if (!(state & STATE_OBD_READY)) {
            logACCData();
            if (millis() - startTime > 10000) {
            // try reconnecting OBD-II
            if (init()) {
                state |= STATE_OBD_READY;
                showStates();
            }
            startTime = millis();
            }
            return;
        }

        static byte index = 0;
        static byte index2 = 0;
        static byte index3 = 0;

        logOBDData(pidTier1[index++]);
        if (index == TIER_NUM1) {
            index = 0;
            if (index2 == TIER_NUM2) {
                index2 = 0;
                logOBDData(pidTier3[index3]);
                index3 = (index3 + 1) % TIER_NUM3;
            } else {
                logOBDData(pidTier2[index2++]);
            }
        }
        if (errors >= 5) {
            reconnect();
        }
    }
#if ENABLE_DATA_LOG
    bool checkSD()
    {
        Sd2Card card;
        SdVolume volume;
        state &= ~STATE_SD_READY;
        pinMode(SS, OUTPUT);
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
            SerialInfo.println(type);
#endif
            if (!volume.init(card)) {
#if VERBOSE
                SerialInfo.println("No FAT!");
#endif
                return false;
            }

            uint32_t volumesize = volume.blocksPerCluster();
            volumesize >>= 1; // 512 bytes per block
            volumesize *= volume.clusterCount();
            volumesize >>= 10;
#if VERBOSE
            SerialInfo.print("SD size: ");
            SerialInfo.print((int)((volumesize + 511) / 1000));
            SerialInfo.println("GB");
#endif
        } else {
#if VERBOSE
            SerialInfo.println("No SD detected");
#endif
            return false;
        }

        if (!SD.begin(SD_CS_PIN)) {
#if VERBOSE
            SerialInfo.print("Bad SD");
#endif
            return false;
        }

        state |= STATE_SD_READY;
        return true;
    }
#endif
private:
    void logOBDData(byte pid)
    {
        int value;

        // send a query command
        sendQuery(pid);

        // do something else while waiting for reponse
        logACCData();

        pid = 0; // this lets PID also get from response
        // receive and parse the response
        if (getResult(pid, value)) {
            dataTime = millis();
            showData(pid, value);
            // log data to SD card
            logData(0x100 | pid, value);
            errors = 0;
        } else {
            errors++;
            return;
        }
    }
    void logGPSData()
    {
#if USE_GPS
        write("ATGRR\r");

        for (uint32_t t = millis();;) {
            if (available()) {
                char c = read();
                if (c == '>') {
                    // prompt char received
                    break;
                } else {
#if VERBOSE
                    SerialInfo.write(c);
#endif
#if PARSE_GPS_DATA
                    gps.encode(c);
#else
                    logData(c);
#endif
                }
            } else if (millis() - t > 100) {
                // timeout
                break;
            }
        }

#if PARSE_GPS_DATA
        if (gps.location.isUpdated()) {
            logData(PID_GPS_TIME, gps.date.value(), gps.time.value());
            logData(PID_GPS_COORDINATES, (float)gps.location.lat(), (float)gps.location.lng());
            logData(PID_GPS_ALTITUDE, (int)gps.altitude.meters());
            logData(PID_GPS_SPEED, (float)gps.speed.kmph());
        }
#endif
#endif
    }
    void logACCData()
    {
#if USE_MPU6050
        if (state & STATE_ACC_READY) {
            accel_t_gyro_union accData;
            MPU6050_readout(&accData);
            dataTime = millis();
#if VERBOSE
            SerialInfo.print("X:");
            SerialInfo.print(accData.value.x_accel);
            SerialInfo.print(" Y:");
            SerialInfo.print(accData.value.y_accel);
            SerialInfo.print(" Z:");
            SerialInfo.println(accData.value.z_accel);
#endif
            // log x/y/z of accelerometer
            logData(PID_ACC, accData.value.x_accel, accData.value.y_accel, accData.value.z_accel);
            // log x/y/z of gyro meter
            //logData(PID_GYRO, accData.value.x_gyro, accData.value.y_gyro, accData.value.z_gyro);
        }
#endif
    }
    void showECUCap()
    {
#if VERBOSE
        byte pidlist[] = {PID_RPM, PID_SPEED, PID_THROTTLE, PID_ENGINE_LOAD, PID_CONTROL_MODULE_VOLTAGE, PID_MAF_FLOW, PID_INTAKE_MAP, PID_FUEL_LEVEL, PID_FUEL_PRESSURE, PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_AMBIENT_TEMP, PID_TIMING_ADVANCE, PID_BAROMETRIC};
        const char* namelist[] = {"RPM", "SPEED", "THROTTLE", "ENG.LOAD", "CTRL VOLT", "MAF", "MAP", "FUEL LV.", "FUEL PRE.", "COOLANT", "INTAKE","AMBIENT", "IGNITION", "BARO"};
        for (byte i = 0; i < sizeof(pidlist) / sizeof(pidlist[0]); i++) {
            SerialInfo.print(namelist[i]);
            SerialInfo.print(':');
            SerialInfo.println(isValidPID(pidlist[i]) ? 'Yes' : 'No');
        }
#endif
    }
    void reconnect()
    {
#if ENABLE_DATA_LOG
        closeFile();
#endif
        startTime = millis();
        state &= ~(STATE_OBD_READY | STATE_ACC_READY);
        state |= STATE_SLEEPING;
        //digitalWrite(SD_CS_PIN, LOW);
        for (int i = 1; !init(); i++) {
#if VERBOSE
            SerialInfo.print("Reconnect #");
            SerialInfo.println(i);
#endif
        }
        state &= ~STATE_SLEEPING;
        fileIndex++;
        setup();
    }
    byte state;

    // screen layout related stuff
    void showStates()
    {
#if VERBOSE
        SerialInfo.print("OBD:");
        SerialInfo.print((state & STATE_OBD_READY) ? "Yes" : "No");
        SerialInfo.print(" ACC:");
        SerialInfo.println((state & STATE_ACC_READY) ? "Yes" : "No");
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
    void dataIdleLoop()
    {
#if ENABLE_DATA_LOG
        // flush SD data every 1KB
        if ((uint16_t)dataSize - lastFileSize >= 1024) {
            flushFile();
            lastFileSize = (uint16_t)dataSize;
#if VERBOSE
            // display logged data size
            SerialInfo.print("Log: ");
            SerialInfo.print((int)(dataSize >> 10));
            SerialInfo.println("KB");
#endif
        }
#endif
    }
};

static COBDLogger logger;

void setup()
{
#if VERBOSE
    SerialInfo.begin(STREAM_BAUDRATE);
#endif

    logger.begin();
    logger.initSender();

#if ENABLE_DATA_LOG
    logger.checkSD();
#endif
    logger.setup();
}

void loop()
{
    logger.loop();
}
