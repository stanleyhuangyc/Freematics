#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Data logging/streaming out
**************************************/

#ifndef ENABLE_DATA_LOG

// enable(1)/disable(0) data logging (if SD card is present)
#define ENABLE_DATA_LOG 1

// enable(1)/disable(0) data streaming
#define ENABLE_DATA_OUT 0

// file size limit
#define MAX_DATA_FILE_SIZE 1024 /* KB */

/**************************************
* Hardware setup
**************************************/

// enable(1)/disable(0) OBD-II reading
#define USE_OBD 1

// enable(1)/disable(0) MEMS sensor
#define USE_MEMS 1

// enable(1)/disable(0) GPS module
#define USE_GPS 1

#endif

// GPS parameters
#define GPS_SERIAL_BAUDRATE 115200L

// time for OBD-II attempt before entering non-OBD mode
#define OBD_ATTEMPT_TIME 180000

// motion detection
#define WAKEUP_MOTION_THRESHOLD 20 /* for wakeup on movement */

// SD pin
#ifdef ESP32
#define SD_CS_PIN 5
#else
#define SD_CS_PIN 10
#endif

#endif // CONFIG_H_INCLUDED
