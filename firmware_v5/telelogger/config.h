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
#define STORAGE_NONE 0
#define STORAGE_SPIFFS 1
#define STORAGE_SD 2

#define GNSS_NONE 0
#define GNSS_INTERNAL 1
#define GNSS_EXTERNAL 2
#define GNSS_CELLULAR 3

#define PROTOCOL_UDP 1
#define PROTOCOL_HTTP 2
#define PROTOCOL_HTTPS 3

#define PROTOCOL_METHOD_GET 0
#define PROTOCOL_METHOD_POST 1

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
#ifndef ENABLE_WIFI
// WiFi settings
#define ENABLE_WIFI 0
#define WIFI_SSID "FREEMATICS"
#define WIFI_PASSWORD "PASSWORD"
// cellular network settings
#define CELL_APN "hologram"
// Freematics Hub server settings
#define SERVER_HOST "hub.freematics.com"
#define SERVER_PROTOCOL PROTOCOL_UDP
#endif 

// SIM card setting
#define SIM_CARD_PIN ""

// HTTPS settings
#define SERVER_METHOD PROTOCOL_METHOD_POST
#define SERVER_PATH "/hub/api"

#if !SERVER_PORT
#undef SERVER_PORT
#if SERVER_PROTOCOL == PROTOCOL_UDP
#define SERVER_PORT 8081
#elif SERVER_PROTOCOL == PROTOCOL_HTTP
#define SERVER_PORT 80
#elif SERVER_PROTOCOL == PROTOCOL_HTTPS
#define SERVER_PORT 443
#endif
#endif

// WiFi Mesh settings
#define WIFI_MESH_ID "123456"
#define WIFI_MESH_CHANNEL 13

// WiFi AP settings
#define WIFI_AP_SSID "TELELOGGER"
#define WIFI_AP_PASSWORD "PASSWORD"

// maximum consecutive communication errors before resetting network
#define MAX_CONN_ERRORS_RECONNECT 5
// maximum allowed connecting time
#define MAX_CONN_TIME 10000 /* ms */
// data receiving timeout
#define DATA_RECEIVING_TIMEOUT 5000 /* ms */
// expected maximum server sync signal interval
#define SERVER_SYNC_INTERVAL 120 /* seconds, 0 to disable */
// data interval settings
#define STATIONARY_TIME_TABLE {30, 60, 180} /* seconds */
#define DATA_INTERVAL_TABLE {1000, 2000, 5000} /* ms */
#define PING_BACK_INTERVAL 900 /* seconds */
#define SIGNAL_CHECK_INTERVAL 10 /* seconds */

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
#ifndef ENABLE_MEMS
#define ENABLE_MEMS 1
#endif

/**************************************
* GPS
**************************************/
#ifndef GNSS
// change the following line to change GNSS setting
#define GNSS GNSS_INTERNAL
#endif
// keeping GNSS power on during standby 
#define GNSS_ALWAYS_ON 0

/**************************************
* Standby/wakeup
**************************************/
// motion threshold for waking up
#define MOTION_THRESHOLD 0.4f /* moving vehicle motion threshold in G */
// engine jumpstart voltage for waking up (when MEMS unavailable) 
#define JUMPSTART_VOLTAGE 14 /* V */
// reset device after waking up
#define RESET_AFTER_WAKEUP 1

/**************************************
* Additional features
**************************************/
#define CONFIG_MODE_TIMEOUT 0

#define PIN_SENSOR1 34
#define PIN_SENSOR2 26

#define COOLING_DOWN_TEMP 65 /* celsius degrees */

// enable(1)/disable(0) http server
#define ENABLE_HTTPD 0

// enable(1)/disable(0) BLE SPP server (for Freematics Controller App).
#define ENABLE_BLE 0

#endif // CONFIG_H_INCLUDED
