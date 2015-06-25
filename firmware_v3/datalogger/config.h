#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Data logging/streaming out
**************************************/

// enable(1)/disable(0) data logging (if SD card is present)
#define ENABLE_DATA_LOG 1

// enable(1)/disable(0) data streaming
#define ENABLE_DATA_OUT 1

// uses software(1)/hardware(0) serial for data streaming
#define USE_SOFTSERIAL 1

// followings define the format of data streaming, enable one of them only
// FORMAT_BIN is required by Freematics OBD iOS App
//#define STREAM_FORMAT FORMAT_BIN
// FORMAT_CSV is for CSV based text output
//#define STREAM_FORMAT FORMAT_CSV
// FORMAT_LINE is for readable text output
#define STREAM_FORMAT FORMAT_TEXT

/* Default streaming baudrates:
   9600bps for BLE and DUO
   38400bps for BT 2.1
*/
#define STREAM_BAUDRATE 9600

// logging interval for slowly changing data
#define LONG_INTERVAL 10 /* seconds */

// minimum loop time
#define MIN_LOOP_TIME 0 /* ms */

// minimum data interval
#define MIN_DATA_INTERVAL 50 /* ms */

// maximum size per file, a new file will be created on reaching this size
#define MAX_LOG_FILE_SIZE 256 /* KB */

// outputs more debug information
#define VERBOSE 0

/**************************************
* Hardware setup
**************************************/

// number of attempts of connecting OBD-II (0 for always)
#define OBD_ATTEMPTS 3

// SD pin
#define SD_CS_PIN 10

// enable(1)/disable(0) MEMS sensor
//#define USE_MPU6050 1
#define USE_MPU9150 1

// enable(1)/disable(0) GPS module
#define USE_GPS 1
#define LOG_GPS_NMEA_DATA 0
#define LOG_GPS_PARSED_DATA 1

#endif // CONFIG_H_INCLUDED
