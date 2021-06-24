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
// FORMAT_TEXT is for readable text output
#define STREAM_FORMAT FORMAT_TEXT

// serial baudrate for data out streaming
#define STREAM_BAUDRATE 9600

// maximum size per file, a new file will be created on reaching this size
#define MAX_LOG_FILE_SIZE 256 /* KB */

// outputs more debug information
#define VERBOSE 0

/**************************************
* Hardware setup
**************************************/

// attempts for connecting OBD-II
#define OBD_ATTEMPT_TIME 30 /* seconds */

// OBD-II UART baudrate
#define OBD_UART_BAUDRATE 115200L

// SD pin
#define SD_CS_PIN 10

// enable(1)/disable(0) MEMS sensor
#define USE_MPU6050 1
//#define USE_MPU9150 1

// enable(1)/disable(0) GPS module
#define USE_GPS 1
#define LOG_GPS_NMEA_DATA 1
#define LOG_GPS_PARSED_DATA 1

// GPS parameters
#define GPS_SERIAL_BAUDRATE 38400
#define GPS_DATA_TIMEOUT 200 /* ms */

#endif // CONFIG_H_INCLUDED
