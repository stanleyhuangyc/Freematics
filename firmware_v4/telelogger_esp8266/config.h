#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Data logging/streaming out
**************************************/
#define ENABLE_DATA_OUT 0

#define STREAM_BAUDRATE 115200

//this defines the format of log file
#define STREAM_FORMAT FORMAT_TEXT

#define ENABLE_DATA_CACHE 1
#define MAX_CACHE_SIZE 256

// change this to your own
#define WIFI_SSID "YOUR_SSID"
#define WIFI_PASSWORD "PASSWORD"
#define SERVER_URL "live.freematics.com.au"
#define SERVER_PORT 8080

/**************************************
* Accelerometer & Gyro
**************************************/
#define USE_MPU6050 1
#define ACC_DATA_RATIO 172
#define GYRO_DATA_RATIO 256
#define COMPASS_DATA_RATIO 8

/**************************************
* GPS
**************************************/
#define USE_GPS 1
#define GPS_SERIAL_BAUDRATE 115200L

/**************************************
* Other options
**************************************/
#define OBD_CONN_TIMEOUT 5000 /* ms */
#define MAX_CONN_ERRORS 3
#define MAX_CONN_TIME 5000 /* ms */
#define MAX_HTTP_CONNS 99

#define SD_CS_PIN 10

#define USE_FRIENDLY_PID_NAME 1

#endif // CONFIG_H_INCLUDED
