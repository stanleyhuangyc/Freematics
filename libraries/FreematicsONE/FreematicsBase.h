/*************************************************************************
* Base class for Freematics telematics products
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one for more information
* (C)2017 Stanley Huang <support@freematics.com.au
*************************************************************************/

#ifndef FREEMATICS_BASE
#define FREEMATICS_BASE

#include <Arduino.h>

#define EVENT_LOGIN 1
#define EVENT_LOGOUT 2
#define EVENT_SYNC 3

#ifdef ESP32
#define PIN_XBEE_PWR 27
#define PIN_GPS_POWER 15
#endif

#define GPS_READ_TIMEOUT 200 /* ms */
#define GPS_INIT_TIMEOUT 1000 /* ms */

typedef struct {
    uint32_t date;
    uint32_t time;
    int32_t lat;
    int32_t lng;
    int16_t alt;
    uint8_t speed;
    uint8_t sat;
    int16_t heading;
} GPS_DATA;

typedef struct {
  float lat;
  float lng;
  uint8_t year; /* year past 2000, e.g. 15 for 2015 */
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} NET_LOCATION;

typedef struct {
  float pitch;
  float yaw;
  float roll;
} ORIENTATION;

int dumpLine(char* buffer, int len);
uint16_t hex2uint16(const char *p);
byte hex2uint8(const char *p);

class CFreematics
{
public:
	virtual byte begin() { return 1; }
	virtual bool init() { return true; }
	// hibernate (lower power consumption)
	virtual void hibernate(unsigned int ms) { delay(ms); }
	// normal delay
	virtual void sleep(unsigned int ms) { delay(ms); }
	// enter low power mode
	virtual void enterLowPowerMode() {}
	// leave low power mode
	virtual void leaveLowPowerMode() {}
	// initialize GPS (set baudrate to 0 to power off GPS)
	virtual bool gpsInit(unsigned long baudrate = 115200L) = 0;
	// get parsed GPS data
	virtual bool gpsGetData(GPS_DATA* gdata) = 0;
	// send command string to GPS
	virtual void gpsSendCommand(const char* cmd) {};
	// type of GPS
	virtual bool internalGPS() { return false; }
	// start xBee UART communication
	virtual bool xbBegin(unsigned long baudrate = 115200L) = 0;
	// read data to xBee UART
	virtual int xbRead(char* buffer, int bufsize, unsigned int timeout = 1000) = 0;
	// send data to xBee UART
	virtual void xbWrite(const char* cmd) = 0;
  // send data to xBee UART
	virtual void xbWrite(const char* data, int len) = 0;
	// receive data from xBee UART (returns 0/1/2)
	virtual int xbReceive(char* buffer, int bufsize, unsigned int timeout = 1000, const char** expected = 0, byte expectedCount = 0) = 0;
	// purge xBee UART buffer
	virtual void xbPurge() = 0;
	// toggle xBee module power
	virtual void xbTogglePower() = 0;
};

#endif
