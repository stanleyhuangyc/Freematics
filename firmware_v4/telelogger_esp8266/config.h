#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* OBD-II configurations
**************************************/
// maximum consecutive OBD-II access errors before entering standby
#define MAX_OBD_ERRORS 3
// maximum allowed time for re-establishing OBD connection
#define MAX_OBD_RETRY_TIME 15000 /* ms */
// VIN used when real one unavailable
#define DEFAULT_VIN "FREEMATICS"

/**************************************
* Networking configurations
**************************************/
// WIFI settings
#define WIFI_SSID "HOTSPOT"
#define WIFI_PASSWORD "PASSWORD"
// Freematics Hub server settings
#define SERVER_HOST "hub.freematics.com"
#define SERVER_PORT 8081
#define SERVER_KEY "TEST_SERVER_KEY"

// maximum consecutive communication errors before entering standby
#define MAX_CONN_ERRORS 20
// maximum consecutive communication errors before reconnecting
#define MAX_CONN_ERRORS_RECONNECT 3
// maximum allowed connecting time
#define MAX_CONN_TIME 10000 /* ms */

/**************************************
* Data storage configurations
**************************************/
#define SERVER_SYNC_INTERVAL 60 /* seconds, 0 to disable */

/**************************************
* GPS
**************************************/
// change the following line to enable (1)/disable (0) GPS
#define ENABLE_GPS 1
#define GPS_SERIAL_BAUDRATE 115200L

/**************************************
* Standby/wakeup
**************************************/
// motion threshold for waking up
#define WAKEUP_MOTION_THRESHOLD 0.15f /* in unit of G */

#endif // CONFIG_H_INCLUDED
