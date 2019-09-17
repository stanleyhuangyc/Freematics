#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Circular Buffer Configuration
**************************************/
#define BUFFER_SLOTS 32 /* max number of buffer */
#define BUFFER_LENGTH 128 /* bytes per slot */
#define SERIALIZE_BUFFER_SIZE 1024 /* bytes */

/**************************************
* Configuration Definitions
**************************************/
#define NET_WIFI 1
#define NET_SIM800 2
#define NET_SIM5360 3
#define NET_SIM7600 4
#define NET_WIFI_MESH 5
#define NET_SERIAL 6

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

// maximum consecutive OBD access errors before entering standby
#define MAX_OBD_ERRORS 3

/**************************************
* Networking configurations
**************************************/
#ifndef NET_DEVICE
// change the following line to change network device
#define NET_DEVICE NET_SIM7600
// WiFi settings
#define WIFI_SSID "SSID"
#define WIFI_PASSWORD "PASSWORD"
// WiFi Mesh settings
#define WIFI_MESH_ID "123456"
#define WIFI_MESH_CHANNEL 13
// APN settings for cellular network (if required)
#define CELL_APN ""
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

#define WIFI_AP_SSID "TELELOGGER"
#define WIFI_AP_PASSWORD "PASSWORD"

// maximum consecutive communication errors before reconnecting
#define MAX_CONN_ERRORS_RECONNECT 3
// maximum allowed connecting time
#define MAX_CONN_TIME 10000 /* ms */
// data receiving timeout
#define DATA_RECEIVING_TIMEOUT 5000 /* ms */
// expected maximum server sync signal interval
#define SERVER_SYNC_INTERVAL 120 /* seconds, 0 to disable */
// data interval configurations
#define STATIONARY_TIME_TABLE {10, 30, 300} /* seconds */
#define SENDING_INTERVAL_TABLE {0, 2000, 5000} /* ms */
#define DATASET_INTERVAL 200
#define PING_BACK_INTERVAL 300 /* seconds */

/**************************************
* Data storage configurations
**************************************/
#ifndef STORAGE
// change the following line to change storage type
#define STORAGE STORAGE_SD
#endif

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
#define GPS_MOTION_TIMEOUT 180 /* seconds */

/**************************************
* Standby/wakeup
**************************************/
#define RESET_AFTER_WAKEUP 1
// motion threshold for waking up
#define MOTION_THRESHOLD 0.3f /* moving vehicle motion threshold in G */
// engine jumpstart voltage
#define JUMPSTART_VOLTAGE 14 /* V */

/**************************************
* Additional features
**************************************/
#define ENABLE_HTTPD 0
#define ENABLE_OLED 0
#define CONFIG_MODE_TIMEOUT 0

#define PIN_SENSOR1 35
#define PIN_SENSOR2 36

#define COOLING_DOWN_TEMP 65 /* celsius degrees */

#endif // CONFIG_H_INCLUDED
