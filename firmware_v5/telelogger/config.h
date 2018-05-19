#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Configuration Definitions
**************************************/
#define NET_NONE 0
#define NET_WIFI 1
#define NET_SIM800 2
#define NET_SIM5360 3
#define NET_SIM7600 4

#define MEMS_DISABLED 0
#define MEMS_ACC 1
#define MEMS_9DOF 2
#define MEMS_DMP 3

#define STORAGE_NONE 0
#define STORAGE_SPIFFS 1
#define STORAGE_SD 2

/**************************************
* OBD-II configurations
**************************************/
#ifndef ENABLE_OBD
#define ENABLE_OBD 1
#endif
// maximum consecutive OBD-II access errors before entering standby
#define MAX_OBD_ERRORS 3

// VIN used when real one unavailable
#ifdef DEVICE_ID
#define DEFAULT_VIN DEVICE_ID
#else
#define DEFAULT_VIN "FreematicsDevice"
#endif

// minimum processing loop time
#define MIN_LOOP_TIME 500 /* ms */

/**************************************
* Networking configurations
**************************************/
#ifndef NET_DEVICE
// change the following line to change network device
#define NET_DEVICE NET_WIFI
// WIFI settings
#define WIFI_SSID "HOTSPOT"
#define WIFI_PASSWORD "PASSWORD"
// APN settings for cellular network
#define CELL_APN "connect"
// Freematics Hub server settings
#define SERVER_HOST "hub.freematics.com"
#define SERVER_PORT 8081
#endif

#define SERVER_KEY "TEST_SERVER_KEY"

#ifndef ENABLE_HTTPD
#define ENABLE_HTTPD 1
#endif
#define WIFI_AP_SSID "TELELOGGER"
#define WIFI_AP_PASSWORD "PASSWORD"

// maximum consecutive communication errors before entering standby
#define MAX_CONN_ERRORS 20
// maximum consecutive communication errors before reconnecting
#define MAX_CONN_ERRORS_RECONNECT 3
// maximum allowed connecting time
#define MAX_CONN_TIME 10000 /* ms */

/**************************************
* BLE configurations
**************************************/
#ifdef DEVICE_ID
#define BLE_DEVICE_NAME DEVICE_ID
#else
#define BLE_DEVICE_NAME "Freematics ONE+"
#endif

/**************************************
* Data storage configurations
**************************************/
#ifndef STORAGE
// change the following line to change storage type
#define STORAGE STORAGE_SD
#endif

#define RAM_CACHE_SIZE 1024 /* bytes */

#ifndef DATA_SENDING_INTERVAL
#define DATA_SENDING_INTERVAL 1000 /* ms */
#endif
#define SERVER_SYNC_INTERVAL 60 /* seconds, 0 to disable */


/**************************************
* MEMS sensors
**************************************/
#define ENABLE_ORIENTATION 0
#ifndef MEMS_MODE
#define MEMS_MODE MEMS_9DOF
#endif

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
#define RESET_AFTER_WAKEUP 0
// motion threshold for waking up
#define WAKEUP_MOTION_THRESHOLD 0.3f /* in unit of G */
// engine jumpstart voltage
#define JUMPSTART_VOLTAGE 14 /* V */

/**************************************
* Other options
**************************************/
#define COOLING_DOWN_TEMP 80 /* celsius degrees */

#endif // CONFIG_H_INCLUDED
