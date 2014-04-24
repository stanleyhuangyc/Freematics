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

// this defines the format of log file
#define LOG_FORMAT FORMAT_CSV

// this defines the format of data streaming
#define STREAM_FORMAT FORMAT_BIN

/* Default streaming baudrates:
   9600bps for BLE
   38400bps for BT 2.1
*/
#define STREAM_BAUDRATE 9600

// outputs debug information
#define VERBOSE 0

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
// enable(1)/disable(0) MPU6050 (I2C accelerometer/gyro)
#define USE_MPU6050 1

// enable(1)/disable(0) GPS module
#define USE_GPS 0
#define PARSE_GPS_DATA 0

#endif // CONFIG_H_INCLUDED
