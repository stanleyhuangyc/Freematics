#ifdef ESP32
#include "FreematicsONE.h"
#include "FreematicsNetwork.h"

class Task
{
public:
  Task():xHandle(0) {}
	bool create(void (*task)(void*), const char* name, int priority = 0);
  void destroy();
  static void sleep(uint32_t ms);
private:
	void* xHandle;
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

class CFreematicsESP32 : public CFreematics
{
public:
  // initialize GPS (set baudrate to 0 to power off GPS)
  virtual bool gpsInit(unsigned long baudrate = 115200L);
  // get parsed GPS data
  virtual bool gpsGetData(GPS_DATA* gdata);
  // send command string to GPS
  virtual void gpsSendCommand(const char* cmd);
	// start xBee UART communication
	virtual bool xbBegin(unsigned long baudrate = 115200L);
	// read data to xBee UART
	virtual int xbRead(char* buffer, int bufsize, int timeout = 1000);
	// send data to xBee UART
	virtual void xbWrite(const char* cmd);
  // send data to xBee UART
	virtual void xbWrite(const char* data, int len);
	// receive data from xBee UART (returns 0/1/2)
	virtual byte xbReceive(char* buffer, int bufsize, int timeout = 1000, const char** expected = 0, byte expectedCount = 0);
	// purge xBee UART buffer
	virtual void xbPurge();
	// toggle xBee module power
	virtual void xbTogglePower();
};
#else

#error This library works with ESP32 targets only.

#endif
