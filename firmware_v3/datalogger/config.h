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
#define STREAM_FORMAT FORMAT_CSV

/* Default streaming baudrates:
   9600bps for BLE
   38400bps for BT 2.1
*/
#define STREAM_BAUDRATE 9600

// outputs debug information
#define VERBOSE 1

/**************************************
* Choose SD pin here
**************************************/
//#define SD_CS_PIN SS // generic
//#define SD_CS_PIN 4 // ethernet shield
//#define SD_CS_PIN 7 // microduino
#define SD_CS_PIN 10 // SD breakout

/**************************************
* Other options
**************************************/
// minimum time for a loop (set in case OBD-II polling is too fast)
#define LOOP_INTERVAL 100 /* ms */

// enable(1)/disable(0) accelerometer/gyro
#define USE_ACCEL 1

// enable(1)/disable(0) GPS module
#define USE_GPS 0
#define LOG_GPS_NMEA_DATA 1
#define LOG_GPS_PARSED_DATA 0

#endif // CONFIG_H_INCLUDED
