/*************************************************************************
* Arduino library for Freematics ONE+ (ESP32)
* Distributed under BSD license
* Visit http://freematics.com for more information
* (C)2017 Developed by Stanley Huang <support@freematics.com.au>
*************************************************************************/

#include <Arduino.h>

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

class CFreematicsESP32
{
public:
	// start xBee UART communication
	virtual bool xbBegin(unsigned long baudrate = 115200L);
	// read data to xBee UART
	virtual int xbRead(char* buffer, int bufsize, unsigned int timeout = 1000);
	// send data to xBee UART
	virtual void xbWrite(const char* cmd);
  // send data to xBee UART
	virtual void xbWrite(const char* data, int len);
	// receive data from xBee UART (returns 0/1/2)
	virtual int xbReceive(char* buffer, int bufsize, unsigned int timeout = 1000, const char** expected = 0, byte expectedCount = 0);
	// purge xBee UART buffer
	virtual void xbPurge();
	// toggle xBee module power
	virtual void xbTogglePower();
  // delay specified number of ms while receiving and processing GPS data (ought to be regularly called)
  virtual void sleep(unsigned int ms);
  // hibernate (lower power consumption)
  virtual void hibernate(unsigned int ms);
};
