/*************************************************************************
* Arduino library for ESP32 based Freematics ONE+ and Freematics Esprit
* Distributed under BSD license
* Visit https://freematics.com for more information
* (C)2017-2019 Developed by Stanley Huang <stanley@freematics.com.au>
*************************************************************************/

#include <Arduino.h>
#include "esp_system.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_spi_flash.h"
#include "soc/rtc.h"
#include "FreematicsBase.h"
#include "FreematicsNetwork.h"
#include "FreematicsMEMS.h"
#include "FreematicsDMP.h"
#include "FreematicsOBD.h"

#define PIN_LED 4
#define PIN_SD_CS 5

#define PIN_LINK_SPI_CS 2
#define PIN_LINK_SPI_READY 13
#define SPI_FREQ 1000000

#define LINK_UART_BAUDRATE 115200
#define LINK_UART_NUM UART_NUM_2
#define LINK_UART_BUF_SIZE 256
#define PIN_LINK_UART_RX 13
#define PIN_LINK_UART_TX 14
#define PIN_LINK_RESET 15

#define PIN_BEE_PWR 27
#define PIN_BEE_UART_RXD 16
#define PIN_BEE_UART_TXD 17
#define PIN_BEE_UART_RXD2 32
#define PIN_BEE_UART_TXD2 33
#define PIN_BEE_UART_RXD3 35
#define PIN_BEE_UART_TXD3 2
#define BEE_UART_NUM UART_NUM_1
#define BEE_BAUDRATE 115200L

#define PIN_GPS_POWER 15
#define PIN_GPS_POWER2 12
#define PIN_GPS_UART_RXD 32
#define PIN_GPS_UART_TXD 33
#define GPS_UART_NUM UART_NUM_2
#define GPS_SOFT_BAUDRATE 38400L

#define PIN_BUZZER 25
#define PIN_MOLEX_2 34
#define PIN_MOLEX_4 26

#define UART_BUF_SIZE 256
#define NMEA_BUF_SIZE 512

#define USE_GNSS 0x1
#define USE_CELL 0x2
#define USE_UART_LINK 0x4
#define GNSS_SOFT_SERIAL 0x8
#define GNSS_USE_LINK 0x10

int readChipTemperature();
int readChipHallSensor();
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

class CLink_UART : public CLink {
public:
	bool begin(unsigned int baudrate = LINK_UART_BAUDRATE, int rxPin = PIN_LINK_UART_RX, int txPin = PIN_LINK_UART_TX);
	void end();
	// send command and receive response
	int sendCommand(const char* cmd, char* buf, int bufsize, unsigned int timeout = 1000);
	// receive data from  UART
	int receive(char* buffer, int bufsize, unsigned int timeout);
	// write data to UART
	void send(const char* str);
  // read one byte from UART
  int read();
  // change serial baudrate
  bool changeBaudRate(unsigned int baudrate);
};

class CLink_SPI : public CLink {
public:
	bool begin(unsigned int freq = SPI_FREQ, int rxPin = 0, int txPin = 0);
	void end();
	// send command and receive response
	int sendCommand(const char* cmd, char* buf, int bufsize, unsigned int timeout = 1000);
	// receive data from SPI
	int receive(char* buffer, int bufsize, unsigned int timeout);
	// write data to SPI
	void send(const char* str);
private:
	const uint8_t header[4] = {0x24, 0x4f, 0x42, 0x44};
};

class FreematicsESP32 : public CFreematics
{
public:
  bool begin(bool useGNSS = true, bool useCellular = true);
  // start GPS
  bool gpsBegin(int baudrate = 115200, bool buffered = false);
  // turn off GPS
  void gpsEnd();
  // get parsed GPS data (returns the number of data parsed since last invoke)
  bool gpsGetData(GPS_DATA** pgd);
  // get buffered NMEA data
  int gpsGetNMEA(char* buffer, int bufsize);
  // send command string to GPS
  void gpsSendCommand(const char* string, int len);
  // start xBee UART communication
  bool xbBegin(unsigned long baudrate = BEE_BAUDRATE, int pinRx = PIN_BEE_UART_RXD, int pinTx = PIN_BEE_UART_TXD);
  void xbEnd();
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
  // control internal buzzer (if present)
  void buzzer(int freq);
  // reset co-processor
  void resetLink();
  // reactivate co-processor
  bool reactivateLink();
	// get co-processor version
	byte getVersion();
	// co-processor firmware version number
	byte version = 0;
	// co-processor link
	CLink *link = 0;
private:
  byte m_flags = 0;
  byte m_pinGPSPower = 0;
};
