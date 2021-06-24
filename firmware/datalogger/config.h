#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Data logging/streaming out
**************************************/
#define ENABLE_DATA_OUT 1
#define ENABLE_DATA_LOG 1
#define USE_SOFTSERIAL 0
//this defines the format of log file
#define LOG_FORMAT FORMAT_CSV
#define STREAM_FORMAT FORMAT_CSV

/**************************************
* Default working mode
**************************************/
#define MODE_DEFAULT MODE_LOGGER

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
#define USE_MPU6050 1
//#define OBD_MIN_INTERVAL 50 /* ms */
#define VERBOSE 0

#endif // CONFIG_H_INCLUDED
