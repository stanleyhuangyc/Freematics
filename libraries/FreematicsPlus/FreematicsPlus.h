/*************************************************************************
* Arduino library for ESP32 based Freematics ONE+ and Freematics Esprit
* Distributed under BSD license
* Visit https://freematics.com for more information
* (C)2017-2018 Developed by Stanley Huang <stanley@freematics.com.au>
*************************************************************************/

#include <Arduino.h>
#include "esp_system.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_spi_flash.h"
#include "FreematicsBase.h"
#include "FreematicsNetwork.h"
#include "FreematicsMEMS.h"
#include "FreematicsDMP.h"
#include "FreematicsOBD.h"

#define PIN_LED 4
#define PIN_SD_CS 5

#define PIN_BEE_PWR 27
#define PIN_BEE_UART_RXD 16
#define PIN_BEE_UART_TXD 17
#define BEE_UART_NUM UART_NUM_1
#define BEE_BAUDRATE 115200L

#define PIN_GPS_POWER 15
#define PIN_GPS_UART_RXD 32
#define PIN_GPS_UART_TXD 33
#define GPS_UART_NUM UART_NUM_2
#define GPS_BAUDRATE 115200L
#define GPS_SOFT_SERIAL 0

#define UART_BUF_SIZE 256
#define NMEA_BUF_SIZE 512

#define GPS_TIMEOUT 1000 /* ms */

uint8_t readChipTemperature();
int32_t readChipHallSensor();
uint16_t getFlashSize(); /* KB */

class Task
{
public:
	bool create(void (*task)(void*), const char* name, int priority = 0, int stacksize = 1024);
  void destroy();
  void suspend();
  void resume();
  bool running();
  void sleep(uint32_t ms);
private:
	void* xHandle = 0;
};

class Mutex
{
public:
  Mutex();
  void lock();
  void unlock();
private:
  void* xSemaphore;
};

class FreematicsESP32 : public CFreematics
{
public:
  void begin(int cpuMHz = CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ);
  // initialize GPS (set baudrate to 0 to power off GPS)
  bool gpsInit(unsigned long baudrate = GPS_BAUDRATE, bool buffered = false);
  // get parsed GPS data (returns the number of data parsed since last invoke)
  bool gpsGetData(GPS_DATA** pgd);
  // get buffered NMEA data
  int gpsGetNMEA(char* buffer, int bufsize);
  // send command string to GPS
  void gpsSendCommand(const char* string, int len);
  // start xBee UART communication
  bool xbBegin(unsigned long baudrate = BEE_BAUDRATE);
  // read data to xBee UART
  int xbRead(char* buffer, int bufsize, unsigned int timeout = 1000);
  // send data to xBee UART
  void xbWrite(const char* cmd);
    // send data to xBee UART
  void xbWrite(const char* data, int len);
  // receive data from xBee UART (returns 0/1/2)
  int xbReceive(char* buffer, int bufsize, unsigned int timeout = 1000, const char** expected = 0, byte expectedCount = 0);
  // purge xBee UART buffer
  void xbPurge();
  // toggle xBee module power
  void xbTogglePower();
};
