#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Data logging/streaming out
**************************************/
#define ENABLE_DATA_CACHE 1
#define CACHE_SIZE 384 /* bytes */
#define ENABLE_DATA_OUT 0
#define STREAM_BAUDRATE 115200
#define STREAM_FORMAT FORMAT_TEXT

/**************************************
* GPRS/network settings
**************************************/
#define XBEE_BAUDRATE 115200
// change APN to your carrier's setting
#define APN "connect"
// change YOUR_SERVER_KEY to your server key
#define SERVER_URL "http://hub.freematics.com/YOUR_SERVER_KEY"
#define USE_GSM_LOCATION 1
// maximum consecutive errors before performing a module reset
#define MAX_CONN_ERRORS 5
// maximum allowed connecting time
#define MAX_CONN_TIME 10000 /* ms */

/**************************************
* MEMS sensors
**************************************/
#define USE_MPU6050 1
#define USE_MPU9250 0
#define ACC_DATA_RATIO 172
#define GYRO_DATA_RATIO 256
#define COMPASS_DATA_RATIO 8

/**************************************
* GPS
**************************************/
#define USE_GPS 1
#define GPS_SERIAL_BAUDRATE 115200L

/**************************************
* Motion detection
**************************************/
#define START_MOTION_THRESHOLD 200000 /* for wakeup on movement */
#define RECALIBRATION_TIME 3000 /* ms */ 

/**************************************
* Other options
**************************************/
#define OBD_INIT_TIMEOUT 60000 /* ms */
#define MIN_LOOP_TIME 1000 /* ms */
#define COOLING_DOWN_TEMP 60 /* celsius degrees */

#endif // CONFIG_H_INCLUDED
