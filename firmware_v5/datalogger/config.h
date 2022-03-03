#define MEMS_DISABLED 0
#define MEMS_ACC 1
#define MEMS_9DOF 2
#define MEMS_DMP 3

#define STORAGE_NONE 0
#define STORAGE_SD 1
#define STORAGE_SPIFFS 2

/**************************************
* Data logging
**************************************/
#ifndef HAVE_CONFIG
// enable(1)/disable(0) serial data output
#define ENABLE_SERIAL_OUT 0
// specify storage type
#define STORAGE STORAGE_SD
#endif

/**************************************
* WIFI and HTTP server
**************************************/
#ifndef HAVE_CONFIG
#define ENABLE_HTTPD 0
#define ENABLE_WIFI_AP 0
#define ENABLE_WIFI_STATION 0
#define WIFI_AP_SSID "DATALOGGER"
#define WIFI_AP_PASSWORD "PASSWORD"
#define WIFI_SSID "FREEMATICS"
#define WIFI_PASSWORD "..."
#endif

#define WIFI_JOIN_TIMEOUT 30000
#define ENABLE_NMEA_SERVER 0
#define NMEA_TCP_PORT 4000

/**************************************
* Hardware setup
**************************************/
#ifndef HAVE_CONFIG
// enable(1)/disable(0) OBD-II reading
#define USE_OBD 1
// GNSS option: 0:disable 1:external 2:SIM5360/7600 3:SIM7070
#define USE_GNSS 1
// enable(1)/disable(0) MEMS motion sensor
#define USE_MEMS 1
// enable(1)/disable(0) quaternion calculation to get orientation
#define ENABLE_ORIENTATION 0
#endif

#define ENABLE_BLE 0

// GPS parameters
#define GPS_SERIAL_BAUDRATE 115200L
// motion detection
#define WAKEUP_MOTION_THRESHOLD 0.03 /* G */
// minimum loop time
#define MIN_LOOP_TIME 100 /* ms */ 