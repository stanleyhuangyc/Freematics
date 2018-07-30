/**************************************
* Definations
**************************************/
#define STORAGE_NONE 0
#define STORAGE_SD 1
#define STORAGE_SPIFFS 2

#ifndef HAVE_CONFIG

/**************************************
* Generic
**************************************/
// enable(1)/disable(0) GPS module
#define USE_GPS 1
// specify storage type
#define STORAGE STORAGE_SPIFFS
// enable(1)/disable(0) screen display
#define ENABLE_DISPLAY 1

/**************************************
* WiFi and HTTP server
**************************************/
#define ENABLE_HTTPD 1
#define ENABLE_WIFI_AP 1
#define ENABLE_WIFI_STATION 1
#define WIFI_AP_SSID "GPSLOGGER"
#define WIFI_AP_PASSWORD "PASSWORD"
#define WIFI_SSID "YOUR_SSID"
#define WIFI_PASSWORD "PASSWORD"

/**************************************
* Traccar client
**************************************/
#define ENABLE_TRACCAR_CLIENT 1
#define TRACCAR_HOST "YOUR_TRACCAR_HOST"
#define TRACCAR_DEV_ID "YOUR_TRACCAR_ID"

#endif

/**************************************
* Misc
**************************************/
#define GPS_SERIAL_BAUDRATE 115200L
#define GPS_SIGNAL_TIMEOUT 30000 /* ms */
#define WIFI_JOIN_TIMEOUT 30000 /* ms */
#define MIN_LOOP_TIME 200 /* ms */
#define TRACCAR_PORT 5055
#define TRACCAR_SERVER_TIMEOUT 10000
#define NMEA_TCP_PORT 4000