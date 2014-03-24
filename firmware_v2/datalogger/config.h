#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* Data logging/streaming out
**************************************/
#define ENABLE_DATA_OUT 0
#define ENABLE_DATA_LOG 1
#define USE_SOFTSERIAL 1
//this defines the format of log file
#define LOG_FORMAT FORMAT_CSV
#define STREAM_FORMAT FORMAT_CSV
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
#define USE_MPU6050 1

#endif // CONFIG_H_INCLUDED
