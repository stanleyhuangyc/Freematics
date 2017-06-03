#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Data logging/streaming out
**************************************/
#define CACHE_SIZE 160 /* bytes */
#define DATASET_INTERVAL 0 /* ms */
#define ENABLE_DATA_OUT 0

/**************************************
* GPRS/network settings
**************************************/
#define XBEE_BAUDRATE 115200
// change APN to your carrier's setting
#define APN "connect"
// change followings to your Freematics Hub credentials
#define SERVER_URL "hub.freematics.com"
#define SERVER_PORT 8081
#define SERVER_KEY "TEST_SERVER_KEY"

// maximum consecutive OBD-II access errors before entering standby
#define MAX_OBD_ERRORS 10
// maximum consecutive communication errors before entering standby
#define MAX_CONN_ERRORS 10
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
#define WAKEUP_MOTION_THRESHOLD 50 /* for wakeup on movement */
#define CALIBRATION_TIME 1000 /* ms */

/**************************************
* Other options
**************************************/
#define COOLING_DOWN_TEMP 65 /* celsius degrees */

#endif // CONFIG_H_INCLUDED
