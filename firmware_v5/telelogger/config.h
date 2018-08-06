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

#define PROTOCOL_UDP 1
#define PROTOCOL_HTTP_GET 2
#define PROTOCOL_HTTP_POST 3

/**************************************
* OBD-II configurations
**************************************/
#ifndef ENABLE_OBD
#define ENABLE_OBD 1
#endif

// maximum consecutive OBD-II access errors before entering standby
#define MAX_OBD_ERRORS 3

// minimum processing loop time
#define MIN_LOOP_TIME 500 /* ms */

/**************************************
* Networking configurations
**************************************/
#ifndef NET_DEVICE
// change the following line to change network device
#define NET_DEVICE NET_WIFI
// WIFI settings
#define WIFI_SSID "FREEMATICS"
#define WIFI_PASSWORD "PASSWORD"
// APN settings for cellular network
#define CELL_APN "mdata.net.au"
// Freematics Hub server settings
#define SERVER_HOST "hub.freematics.com"
#define SERVER_PORT 0
#define SERVER_PROTOCOL PROTOCOL_UDP
#endif

#if !SERVER_PORT
#undef SERVER_PORT
#if SERVER_PROTOCOL == PROTOCOL_UDP
#define SERVER_PORT 8081
#else
#define SERVER_PORT 8080
#endif
#endif

#define SERVER_PATH "/api/post"
#define SERVER_KEY "TEST_SERVER_KEY"

#define WIFI_AP_SSID "TELELOGGER"
#define WIFI_AP_PASSWORD "PASSWORD"

// maximum consecutive communication errors before entering standby
#define MAX_CONN_ERRORS 20
// maximum consecutive communication errors before reconnecting
#define MAX_CONN_ERRORS_RECONNECT 3
// maximum allowed connecting time
#define MAX_CONN_TIME 10000 /* ms */

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
#define DATA_RECEIVING_TIMEOUT 5000 /* ms */

/**************************************
* MEMS sensors
**************************************/
#define ENABLE_ORIENTATION 0
#ifndef MEMS_MODE
#define MEMS_MODE MEMS_ACC
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
#define MOTION_THRESHOLD 0.3f /* moving vehicle motion threshold in G */
// time before entering motion-less standby, 0 for disabled
#define MOTIONLESS_STANDBY 0 /* seconds */
// engine jumpstart voltage
#define JUMPSTART_VOLTAGE 14 /* V */

/**************************************
* Additional features
**************************************/
#define ENABLE_HTTPD 0
#define ENABLE_OLED 0

#define PIN_SENSOR1 34
#define PIN_SENSOR2 35

#define COOLING_DOWN_TEMP 80 /* celsius degrees */

#endif // CONFIG_H_INCLUDED
