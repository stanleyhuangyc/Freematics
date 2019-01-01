#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Configuration Definitions
**************************************/
#define NET_WIFI 1
#define NET_SIM800 2
#define NET_SIM5360 3

/**************************************
* OBD-II configurations
**************************************/
// maximum consecutive OBD-II access errors before disconnecting
#define MAX_OBD_ERRORS 10

/**************************************
* Networking configurations
**************************************/
#ifndef NET_DEVICE
// change the following line to change network device
#define NET_DEVICE NET_SIM5360
// WIFI settings
#define WIFI_SSID "HOTSPOT"
#define WIFI_PASSWORD "PASSWORD"
// APN settings for cellular network (if required)
#define CELL_APN ""
// Freematics Hub server settings
#define SERVER_HOST "hub.freematics.com"
#define SERVER_PORT 8081
#endif

#define DEFAULT_DEVID "DEFAULT"

// maximum consecutive communication errors before entering standby
#define MAX_CONN_ERRORS 20
// maximum consecutive communication errors before reconnecting
#define MAX_CONN_ERRORS_RECONNECT 3
// maximum allowed connecting time
#define MAX_CONN_TIME 10000 /* ms */
// expected maximum server sync signal interval
#define SERVER_SYNC_INTERVAL 180 /* seconds, 0 to disable */
// data interval configurations
#define DATA_INTERVAL 1000 /* ms */
#define STATIONARY_TIME_TABLE {30, 60} /* seconds */
#define SENDING_INTERVAL_TABLE {1, 5} /* seconds */
#define PING_BACK_INTERVAL 900 /* seconds */

/**************************************
* MEMS motion sensors
**************************************/
#define MEMS_MODE 1 /* 0 to disable */
#define MOTION_THRESHOLD 0.25f /* in unit of G */

/**************************************
* GPS
**************************************/
#ifndef ENABLE_GPS
// change the following line to enable (1)/disable (0) GPS
#define ENABLE_GPS 1
#endif
#define GPS_SERIAL_BAUDRATE 115200L

/**************************************
* Standby/wakeup
**************************************/
#define JUMPSTART_VOLTAGE 14 /* V */
#define STANDBY_LOW_VOLTAGE 0 /* V */
#define RESET_AFTER_WAKEUP 0

#endif // CONFIG_H_INCLUDED
