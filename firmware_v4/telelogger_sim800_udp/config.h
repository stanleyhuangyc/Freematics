#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

// maximum consecutive OBD-II access errors before entering standby
#define MAX_OBD_ERRORS 3

// APN settings for cellular network
#define CELL_APN "connect"
// Freematics Hub server settings
#define SERVER_HOST "47.74.68.134"
#define SERVER_PORT 8081

#define DATA_SENDING_INTERVAL 500 /* ms */
#define SERVER_SYNC_INTERVAL 100 /* requests */
#define GSM_LOCATION_INTERVAL 10 /* requests */

// maximum consecutive communication errors before entering standby
#define MAX_CONN_ERRORS 20
// maximum consecutive communication errors before reconnecting
#define MAX_CONN_ERRORS_RECONNECT 3
// maximum allowed connecting time
#define MAX_CONN_TIME 10000 /* ms */

#define DEVICE_ID "My Tracker"

#define GPS_SERIAL_BAUDRATE 115200L

#define RAM_CACHE_SIZE 80 /* bytes */

#endif // CONFIG_H_INCLUDED
