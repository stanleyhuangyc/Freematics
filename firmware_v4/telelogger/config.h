#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Data logging/streaming out
**************************************/
#define ENABLE_DATA_OUT 1
#define USE_SOFTSERIAL 0

#define STREAM_BAUDRATE 115200

//this defines the format of log file
#define LOG_FORMAT FORMAT_TEXT
#define STREAM_FORMAT FORMAT_TEXT

// change this to your own URL
#define APN "CMNET"
#define URL_PUSH "http://cn.freematics.com.au/push"

/**************************************
* Accelerometer & Gyro
**************************************/
#define USE_MPU6050 0
#define ACC_DATA_RATIO 160
#define GYRO_DATA_RATIO 256
#define COMPASS_DATA_RATIO 8

/**************************************
* Timeout/interval options
**************************************/
#define OBD_MIN_INTERVAL 100 /* ms */
#define ACC_DATA_INTERVAL 200 /* ms */
#define GPS_DATA_INTERVAL 200 /* ms */

/**************************************
* Choose SD pin here
**************************************/
#define SD_CS_PIN 10 // SD breakout

/**************************************
* Other options
**************************************/
//#define DEBUG 1
#define DEBUG_BAUDRATE 9600
#define MAX_CONN_ERRORS 10
#define MAX_CACHED_ITEM 2
#endif // CONFIG_H_INCLUDED
