/**************************************
* Definations
**************************************/
#define STORAGE_NONE 0
#define STORAGE_SD 1
#define STORAGE_SPIFFS 2

/**************************************
* Rate/speed
**************************************/
#define GPS_SERIAL_BAUDRATE 115200L
#define WIFI_JOIN_TIMEOUT 30000
#define MIN_LOOP_TIME 200

#ifndef HAVE_CONFIG

/**************************************
* Generic
**************************************/
// enable(1)/disable(0) GPS module
#define USE_GPS 1
// enable(1)/disable(0) data streaming
#define ENABLE_SERIAL_OUT 0
// specify storage type
#define STORAGE STORAGE_SPIFFS

/**************************************
* WIFI and HTTP server
**************************************/
#define ENABLE_HTTPD 1
#define ENABLE_WIFI_AP 1
#define ENABLE_WIFI_STATION 1
#define WIFI_AP_SSID "GPSLOGGER"
#define WIFI_AP_PASSWORD "PASSWORD"
#define WIFI_SSID "HOTSPOT"
#define WIFI_PASSWORD "..."

#endif
