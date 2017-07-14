#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

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
#ifdef DEVICE_ID
#define DEFAULT_VIN DEVICE_ID
#else
#define DEFAULT_VIN "DEFAULT_VIN"
#endif


/**************************************
* Networking configurations
**************************************/
#ifndef NET_DEVICE
// APN settings for cellular network
#define CELL_APN "connect"
// Freematics Hub server settings
#define SERVER_HOST "47.74.68.134"
#define SERVER_PORT 8081
#endif

#define DATA_SENDING_INTERVAL 1000 /* ms */
#define SERVER_SYNC_INTERVAL 100 /* requests */
#define GSM_LOCATION_INTERVAL 10 /* requests */

// maximum consecutive communication errors before entering standby
#define MAX_CONN_ERRORS 20
// maximum consecutive communication errors before reconnecting
#define MAX_CONN_ERRORS_RECONNECT 3
// maximum allowed connecting time
#define MAX_CONN_TIME 10000 /* ms */

/**************************************
* GPS
**************************************/
#ifndef ENABLE_GPS
// change the following line to enable (1)/disable (0) GPS
#define ENABLE_GPS 1
#endif
#define GPS_SERIAL_BAUDRATE 115200L

/**************************************
* Data storage configurations
**************************************/
#ifdef ESP32
#define RAM_CACHE_SIZE 1500 /* bytes */
#else
#define RAM_CACHE_SIZE 128 /* bytes */
#endif

/**************************************
* Motion detection
**************************************/
#define WAKEUP_MOTION_THRESHOLD 30 /* for wakeup on movement */
#define CALIBRATION_TIME 3000 /* ms */

/**************************************
* Other options
**************************************/
#define COOLING_DOWN_TEMP 80 /* celsius degrees */

#endif // CONFIG_H_INCLUDED
