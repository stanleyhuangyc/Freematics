#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Data logging/streaming out
**************************************/
#ifdef ESP32
#define CACHE_SIZE 512 /* bytes */
#else
#define CACHE_SIZE 256 /* bytes */
#endif
#define ENABLE_DATA_OUT 0
#define STREAM_BAUDRATE 115200
#define STREAM_FORMAT FORMAT_TEXT

/**************************************
* GPRS/network settings
**************************************/
#define XBEE_BAUDRATE 115200
// change APN to your carrier's setting
#define APN "connect"
// change TEST_SERVER_KEY to your server key if you have one
#define SERVER_URL "http://hub.freematics.com/TEST_SERVER_KEY"
#define USE_GSM_LOCATION 1
// maximum consecutive errors before resetting
#define MAX_CONN_ERRORS 5
// maximum allowed connecting time
#define MAX_CONN_TIME 10000 /* ms */

/**************************************
* MEMS sensors
**************************************/
#define USE_MEMS 1

/**************************************
* GPS
**************************************/
#define USE_GPS 1
#define GPS_SERIAL_BAUDRATE 115200L

/**************************************
* Motion detection
**************************************/
#define WAKEUP_MOTION_THRESHOLD 100

/**************************************
* Other options
**************************************/
#define COOLING_DOWN_TEMP 65 /* celsius degrees */

#endif // CONFIG_H_INCLUDED
