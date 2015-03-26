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

// this defines the format of data streaming
// FORMAT_BIN is required by Freematics OBD iOS App
//#define STREAM_FORMAT FORMAT_BIN
// FORMAT_CSV is for text output
#define STREAM_FORMAT FORMAT_BIN

/* Default streaming baudrates:
   9600bps for BLE
   38400bps for BT 2.1
*/
#define STREAM_BAUDRATE 9600

// outputs debug information
#define VERBOSE 0

// minimum loop time
#define MIN_LOOP_TIME 50

/**************************************
* Hardware setup
**************************************/

// number of attempts of connecting OBD-II (0 for always)
#define OBD_ATTEMPTS 3

// SD pin
#define SD_CS_PIN 10

// enable(1)/disable(0) MEMS sensor
#define USE_MEMS 1

// enable(1)/disable(0) GPS module
#define USE_GPS 1
#define LOG_GPS_NMEA_DATA 1
#define LOG_GPS_PARSED_DATA 1

#endif // CONFIG_H_INCLUDED
