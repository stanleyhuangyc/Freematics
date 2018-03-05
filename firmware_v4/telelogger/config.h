#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Configuration Definitions
**************************************/
#define NET_NONE 0
#define NET_WIFI 1
#define NET_SIM800 2
#define NET_SIM5360 3

#define MEMS_DISABLED 0
#define MEMS_ACC 1
#define MEMS_9DOF 2

/**************************************
* OBD-II configurations
**************************************/
#ifndef ENABLE_OBD
#define ENABLE_OBD 1
#endif
// maximum consecutive OBD-II access errors before entering standby
#define MAX_OBD_ERRORS 3
// maximum allowed time for re-establishing OBD connection
#define MAX_OBD_RETRY_TIME 15000 /* ms */

// VIN used when real one unavailable
#define DEFAULT_VIN "FREEMATICS"

/**************************************
* Networking configurations
**************************************/
#ifndef NET_DEVICE
// change the following line to change network device
#define NET_DEVICE NET_WIFI
// WIFI settings
#define WIFI_SSID "FREEMATICS"
#define WIFI_PASSWORD "862150909018"
// APN settings for cellular network
#define CELL_APN "connect"
// Freematics Hub server settings
#define SERVER_HOST "hub.freematics.com"
#define SERVER_PORT 8081
#endif

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
* MEMS sensors
**************************************/
#define MEMS_MODE MEMS_ACC

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
// motion threshold for waking up
#define WAKEUP_MOTION_THRESHOLD 0.15f /* in unit of G */
// engine jumpstart voltage
#define JUMPSTART_VOLTAGE 14 /* V */

#endif // CONFIG_H_INCLUDED
