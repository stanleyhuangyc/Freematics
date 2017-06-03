#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Data logging settings
**************************************/
#define CACHE_SIZE 128 /* bytes */
#define DATASET_INTERVAL 0 /* ms */
#define ENABLE_DATA_OUT 0
#define STREAM_BAUDRATE 115200
#define STREAM_FORMAT FORMAT_TEXT

/**************************************
* WIFI and networking settings
**************************************/

#define XBEE_BAUDRATE 9600
// change SSID and PASSWORD to your own
#define WIFI_SSID "HOMEWIFI"
#define WIFI_PASSWORD "862150909018"
// change YOUR_SERVER_KEY to your server key
#define SERVER_KEY "TEST_SERVER_KEY"
#define SERVER_URL "hub.freematics.com"
#define SERVER_PORT 8081

// maximum consecutive OBD-II access errors before entering standby
#define MAX_OBD_ERRORS 10
// maximum consecutive communication errors before entering standby
#define MAX_CONN_ERRORS 10
// maximum allowed connecting time
#define MAX_CONN_TIME 5000 /* ms */
// maximum consecutive HTTP requests on a TCP connection (keep-alive)
#define MAX_HTTP_CONNS 99

/**************************************
* Accelerometer & Gyro
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
#define WAKEUP_MOTION_THRESHOLD 200000 /* for wakeup on movement */

/**************************************
* Other options
**************************************/
#define COOLING_DOWN_TEMP 65 /* celsius degrees */

#endif // CONFIG_H_INCLUDED
