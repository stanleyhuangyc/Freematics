#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

#ifdef CONFIG_ENABLE_OBD
#define ENABLE_OBD CONFIG_ENABLE_OBD
#endif
#ifdef CONFIG_ENABLE_MEMS
#define ENABLE_MEMS CONFIG_ENABLE_MEMS
#endif
#ifdef CONFIG_GNSS
#define GNSS CONFIG_GNSS
#endif
#ifdef CONFIG_STORAGE
#define STORAGE CONFIG_STORAGE
#endif
#ifdef CONFIG_BOARD_HAS_PSRAM
#define BOARD_HAS_PSRAM 1
#endif
#ifdef CONFIG_ENABLE_WIFI
#define ENABLE_WIFI CONFIG_ENABLE_WIFI
#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASSWORD CONFIG_WIFI_PASSWORD
#endif
#ifdef CONFIG_ENABLE_BLE
#define ENABLE_BLE CONFIG_ENABLE_BLE
#endif
#ifdef CONFIG_ENABLE_HTTPD
#define ENABLE_HTTPD CONFIG_ENABLE_HTTPD
#endif
#ifdef CONFIG_SERVER_HOST
#define SERVER_HOST CONFIG_SERVER_HOST
#define SERVER_PORT CONFIG_SERVER_PORT
#define SERVER_PROTOCOL CONFIG_SERVER_PROTOCOL
#endif
#ifdef CONFIG_CELL_APN
#define CELL_APN CONFIG_CELL_APN
#endif

/**************************************
* Circular Buffer Configuration
**************************************/
#if BOARD_HAS_PSRAM
#define BUFFER_SLOTS 1024 /* max number of buffer slots */
#define BUFFER_LENGTH 384 /* bytes per slot */
#define SERIALIZE_BUFFER_SIZE 4096 /* bytes */
#else
#define BUFFER_SLOTS 32 /* max number of buffer slots */
#define BUFFER_LENGTH 256 /* bytes per slot */
#define SERIALIZE_BUFFER_SIZE 1024 /* bytes */
#endif

/**************************************
* Configuration Definitions
**************************************/
#define STORAGE_NONE 0
#define STORAGE_SPIFFS 1
#define STORAGE_SD 2

#define GNSS_NONE 0
#define GNSS_STANDALONE 1
#define GNSS_CELLULAR 2

#define PROTOCOL_UDP 1
#define PROTOCOL_HTTPS_GET 2
#define PROTOCOL_HTTPS_POST 3

/**************************************
* OBD-II configurations
**************************************/
#ifndef ENABLE_OBD
#define ENABLE_OBD 1
#endif

// maximum consecutive OBD access errors before entering standby
#define MAX_OBD_ERRORS 3

// Encode ignition state into the RPM PID (0x10C) so it can be used as a reliable
// ignition indicator (e.g. by Traccar via "rpm > 0"):
//   1 = engine auto-stop (ECU alive, RPM 0) is reported as RPM 1, and ignition
//       OFF (ECU not responding) is reported as RPM 0. rpm >= 1 means ON.
//   0 = original behavior (raw RPM only; nothing sent when ECU is off).
#define RPM_IGNITION_INDICATOR 0

// When RPM_IGNITION_INDICATOR is enabled, the number of ignition-off (RPM 0)
// reports to transmit after the ECU stops responding before the device enters
// standby. Ensures the server is promptly notified that the ignition is off.
#define IGNITION_OFF_REPORTS 3

/**************************************
* Networking configurations
**************************************/
#ifndef ENABLE_WIFI
#define ENABLE_WIFI 1
// WiFi settings
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#endif 

#ifndef SERVER_HOST
// cellular network settings
#define CELL_APN ""
// Freematics Hub server settings
#define SERVER_HOST "hub.freematics.com"
#define SERVER_PROTOCOL PROTOCOL_UDP
#endif

// SIM card setting
#define SIM_CARD_PIN ""
#define APN_USERNAME NULL
#define APN_PASSWORD NULL

// HTTPS settings
#define SERVER_PATH "/hub/api"

#if !SERVER_PORT
#undef SERVER_PORT
#if SERVER_PROTOCOL == PROTOCOL_UDP
#define SERVER_PORT 8081
#else
#define SERVER_PORT 443
#endif
#endif

// WiFi Mesh settings
#define WIFI_MESH_ID "123456"
#define WIFI_MESH_CHANNEL 13

// WiFi AP settings
#define WIFI_AP_SSID "TELELOGGER"
#define WIFI_AP_PASSWORD "PASSWORD"

// Controls how long (in seconds) the vehicle must be motionless before stepping down
// to a slower data rate. Each value pairs with the corresponding DATA_INTERVAL_TABLE
// entry. Once all thresholds are exceeded, the device enters standby mode.
// Example: {60, 600, 1800} = stay at tier-1 rate for 60s, tier-2 for 10min, tier-3
// for 30min, then standby.
#define STATIONARY_TIME_TABLE {10, 60, 180} /* seconds */

// Sets the data transmission interval for each motion tier, paired with
// STATIONARY_TIME_TABLE. Tier 1 (moving), tier 2 (recently stopped), tier 3 (parked).
// Example: {15, 60, 300} = every 15s while moving, every 60s when briefly
// stopped, every 5min when parked with engine on.
#define DATA_INTERVAL_TABLE {1, 2, 5} /* seconds */

// While in standby (engine off, OBD unavailable), the device wakes periodically,
// transmits one position packet, then returns to sleep. This controls that interval.
// Increase to reduce data usage during long parking periods.
#define PING_BACK_INTERVAL 900 /* seconds */

// maximum consecutive communication errors before resetting network
#define MAX_CONN_ERRORS_RECONNECT 5
// maximum allowed connected time
#define MAX_CONN_TIME 10000 /* ms */
// data receiving timeout
#define DATA_RECEIVING_TIMEOUT 5000 /* ms */
// expected maximum server sync signal interval
#define SERVER_SYNC_INTERVAL 120 /* seconds, 0 to disable */
// how often to check signal strength
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
#ifndef ENABLE_MEMS
#define ENABLE_MEMS 1
#endif

/**************************************
* GPS
**************************************/
#ifndef GNSS
// change the following line to change GNSS setting
#define GNSS GNSS_STANDALONE
#endif
// keeping GNSS power on during standby 
#define GNSS_ALWAYS_ON 0
// GNSS reset timeout while no signal
#define GNSS_RESET_TIMEOUT 300 /* seconds */

/**************************************
* Standby/wakeup
**************************************/
// motion threshold for waking up
#define MOTION_THRESHOLD 0.4f /* vehicle motion threshold in G */
// engine jumpstart voltage for waking up (when MEMS unavailable) 
#define JUMPSTART_VOLTAGE 14 /* V */
// reset device after waking up
#define RESET_AFTER_WAKEUP 1

/**************************************
* Additional features
**************************************/
#define PIN_SENSOR1 34
#define PIN_SENSOR2 26

#define COOLING_DOWN_TEMP 75 /* celsius degrees */

// enable(1)/disable(0) http server
#ifndef ENABLE_HTTPD
#define ENABLE_HTTPD 0
#endif

// enable(1)/disable(0) BLE SPP server (for Freematics Controller App).
#ifndef ENABLE_BLE
#define ENABLE_BLE 1
#endif


#endif // CONFIG_H_INCLUDED
