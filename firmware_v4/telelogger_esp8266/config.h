#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Data logging settings
**************************************/
#define CACHE_SIZE 128 /* bytes */
#define DATASET_INTERVAL 0 /* ms */
#define ENABLE_DATA_OUT 0

/**************************************
* WIFI and server settings
**************************************/
// change YOUR_SSID and YOUR_PASSWORD to your own
#define WIFI_SSID "FREEMATICS"
#define WIFI_PASSWORD "PASSWORD"
#define XBEE_BAUDRATE 115200 /* 9600 for ESP8266 with older firmware */

/**************************************
* Server settings
**************************************/
#define SERVER_KEY "TEST_SERVER_KEY"
#define SERVER_URL "hub.freematics.com"
#define SERVER_PORT 8081

/**************************************
* Error handling settings
**************************************/
// maximum consecutive OBD-II access errors before entering standby
#define MAX_OBD_ERRORS 10
// maximum consecutive communication errors before entering standby
#define MAX_CONN_ERRORS 10
// maximum consecutive communication errors before reconnecting
#define MAX_CONN_ERRORS_RECONNECT 3
// maximum allowed connecting time
#define MAX_CONN_TIME 5000 /* ms */

/**************************************
* MEMS settings
**************************************/
#define USE_MEMS 1

/**************************************
* GPS settings
**************************************/
#define USE_GPS 1
#define GPS_SERIAL_BAUDRATE 115200L

/**************************************
* Motion detection
**************************************/
#define WAKEUP_MOTION_THRESHOLD 0.2 /* G */

#endif // CONFIG_H_INCLUDED
