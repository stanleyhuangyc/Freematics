/*************************************************************************
* Arduino OBD-II/G-Force Data Logger and Transmitter
* Distributed under GPL v2.0
* Copyright (c) 2013-14 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include <OBD.h>
#include <SD.h>
#include <MPU6050.h>
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
#define STATE_ACC_READY 0x10
#define STATE_SLEEPING 0x20

#define SerialInfo Serial

static uint32_t lastFileSize = 0;
static int lastSpeed = -1;
static uint32_t lastSpeedTime = 0;
static int speed = 0;
static uint32_t distance = 0;
static uint16_t fileIndex = 0;
static uint32_t startTime = 0;

static byte pollPattern[]= {
    PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_RPM, PID_SPEED, PID_THROTTLE
};

static byte pollPattern2[] = {
    PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_AMBIENT_TEMP, PID_FUEL_LEVEL,
};

#define PATTERN_NUM sizeof(pollPattern)
#define PATTERN_NUM2 sizeof(pollPattern2)
#define SLOW_POLL_INTERVAL 10000 /* ms */

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

        do {
            showStates();
        } while (!init());

        state |= STATE_OBD_READY;

        showStates();

        uint16_t flags = FLAG_CAR | FLAG_OBD;
        if (state & STATE_ACC_READY) flags |= FLAG_ACC;
#if ENABLE_DATA_LOG
        uint16_t index = openFile(LOG_TYPE_DEFAULT, flags);

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
        showECUCap();
    }
    void loop()
    {
        static byte index = 0;
        static byte index2 = 0;
        static uint32_t lastTime = 0;

        logOBDData(pollPattern[index]);
        index = (index + 1) % PATTERN_NUM;

        if (index == 0) {
            if (isValidPID(PID_INTAKE_MAP))
                logOBDData(PID_INTAKE_MAP);
            else if (isValidPID(PID_MAF_FLOW))
                logOBDData(PID_MAF_FLOW);
        }

        if (millis() - lastTime > SLOW_POLL_INTERVAL) {
            byte pid = pollPattern2[index2];
            if (isValidPID(pid)) {
                lastTime = millis();
                logOBDData(pid);
                index2 = (index2 + 1) % PATTERN_NUM2;
            }
        }

#if USE_MPU6050
        if (state & STATE_ACC_READY) {
            processAccelerometer();
        }
#endif

#if ENABLE_DATA_LOG
        // flush SD data every 1KB
        if (dataSize - lastFileSize >= 1024) {
            flushFile();
            lastFileSize = dataSize;
            // display logged data size
            SerialInfo.print("Log size: ");
            SerialInfo.print((int)(dataSize >> 10));
            SerialInfo.println("KB");
        }
#endif

        if (errors >= 2) {
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

#if VERBOSE
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
#if USE_MPU6050
    void processAccelerometer()
    {
        accel_t_gyro_union data;
        MPU6050_readout(&data);
        dataTime = millis();
        // log x/y/z of accelerometer
        logData(PID_ACC, data.value.x_accel, data.value.y_accel, data.value.z_accel);
        //showGForce(data.value.y_accel);
        // log x/y/z of gyro meter
        logData(PID_GYRO, data.value.x_gyro, data.value.y_gyro, data.value.z_gyro);

#if VERBOSE
        SerialInfo.print(dataTime);
        SerialInfo.print(" X:");
        SerialInfo.print(data.value.x_accel);
        SerialInfo.print(" Y:");
        SerialInfo.print(data.value.y_accel);
        SerialInfo.print(" Z:");
        SerialInfo.println(data.value.z_accel);
#endif
    }
#endif
    void logOBDData(byte pid)
    {
        int value;
#ifdef OBD_MIN_INTERVAL
        uint32_t start = millis();
#endif

        // send a query to OBD adapter for specified OBD-II pid
        if (read(pid, value)) {
            dataTime = millis();
            showLoggerData(pid, value);
            // log data to SD card, only log mode 01 PIDs
            logData(0x100 | pid, value);
        }

        // if OBD response is very fast, go on processing other data for a while
#ifdef OBD_MIN_INTERVAL
        while (millis() - start < OBD_MIN_INTERVAL) {
            dataIdleLoop();
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
            SerialInfo.print("Reconnecting #");
            SerialInfo.print(i);
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
    void showLoggerData(byte pid, int value)
    {
#if VERBOSE
        SerialInfo.print('[');
        SerialInfo.print(millis());
        SerialInfo.print("] ");
        switch (pid) {
        case PID_RPM:
            Serial.print("RPM");
            break;
        case PID_SPEED:
            Serial.print("Speed");
            break;
        case PID_THROTTLE:
            Serial.print("Throttle");
            break;
        default:
            SerialInfo.print('[');
            SerialInfo.print(pid, HEX);
            SerialInfo.print(']');
        }
        SerialInfo.print('=');
        SerialInfo.println(value);
#endif
    }
};

static COBDLogger logger;

void setup()
{
#if VERBOSE
    SerialInfo.begin(115200);
    while (!SerialInfo);
    SerialInfo.println("Freematics OBD-II Adapter");
#endif

    logger.begin();
    logger.initSender();

#if ENABLE_DATA_LOG
    logger.checkSD();
#else
    lcd.clear();
#endif
    logger.setup();
}

void loop()
{
    logger.loop();
}
