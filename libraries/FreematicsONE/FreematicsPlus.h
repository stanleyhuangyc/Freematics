/*************************************************************************
* Arduino library for Freematics ONE+ (ESP32)
* Distributed under BSD license
* Visit http://freematics.com for more information
* (C)2017 Developed by Stanley Huang <support@freematics.com.au>
*************************************************************************/

#ifndef ESP32
#error This library works with ESP32 targets only.
#else

#include <Arduino.h>
#include "FreematicsBase.h"
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

class CFreematicsESP32 : public virtual CFreematics
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
	virtual int xbRead(char* buffer, int bufsize, unsigned int timeout = 1000);
	// send data to xBee UART
	virtual void xbWrite(const char* cmd);
  // send data to xBee UART
	virtual void xbWrite(const char* data, int len);
	// receive data from xBee UART (returns 0/1/2)
	virtual byte xbReceive(char* buffer, int bufsize, unsigned int timeout = 1000, const char** expected = 0, byte expectedCount = 0);
	// purge xBee UART buffer
	virtual void xbPurge();
	// toggle xBee module power
	virtual void xbTogglePower();
};

class CStorageNull;

class CStorageNull {
public:
    CStorageNull():m_dataTime(0),m_next(0) {}
    virtual bool init() { return true; }
    virtual bool begin() { return true; }
    virtual void end() {}
    virtual void log(uint16_t pid, int16_t value)
    {
        char buf[16];
        byte len = sprintf_P(buf, PSTR("%X=%d"), pid, value);
        dispatch(buf, len);
    }
    virtual void log(uint16_t pid, int32_t value)
    {
        char buf[20];
        byte len = sprintf_P(buf, PSTR("%X=%ld"), pid, value);
        dispatch(buf, len);
    }
    virtual void log(uint16_t pid, uint32_t value)
    {
        char buf[20];
        byte len = sprintf_P(buf, PSTR("%X=%lu"), pid, value);
        dispatch(buf, len);
    }
    virtual void log(uint16_t pid, int value1, int value2, int value3)
    {
        char buf[24];
        byte len = sprintf_P(buf, PSTR("%X=%d;%d;%d"), pid, value1, value2, value3);
        dispatch(buf, len);
    }
    virtual void logCoordinate(uint16_t pid, int32_t value)
    {
        char buf[24];
        byte len = sprintf_P(buf, PSTR("%X=%d.%06lu"), pid, (int)(value / 1000000), abs(value) % 1000000);
        dispatch(buf, len);
    }
    virtual void timestamp(uint32_t ts)
    {
        log(0, m_dataTime = ts);
        if (m_next) m_next->timestamp(ts);
    }
    virtual void setForward(CStorageNull* next) { m_next = next; }
    virtual void flush() {}
    virtual void purge() {}
    virtual void dispatch(const char* buf, byte len)
    {
        // output data via serial
        Serial.write((uint8_t*)buf, len);
        Serial.write('\n');
        if (m_next) m_next->dispatch(buf, len);
    }
protected:
    virtual void addHeader(int feedid) {}
    virtual void addTail() {}
    virtual void removeTail() {}
    uint32_t m_dataTime;
    CStorageNull* m_next;
};

class CStorageRAM: public CStorageNull {
public:
    CStorageRAM():m_cacheBytes(0),m_cacheSize(0),m_cache(0) {}
    virtual bool init(unsigned int cacheSize)
    {
      if (m_cacheSize != cacheSize) {
        if (m_cache) {
          delete m_cache;
          m_cache = 0;
        }
        if (cacheSize) m_cache = new char[m_cacheSize = cacheSize];
      }
      return true;
    }
    void purge() { m_cacheBytes = 0; }
    unsigned int getBytes() { return m_cacheBytes; }
    char* getBuffer() { return m_cache; }
    void dispatch(const char* buf, byte len)
    {
        // reserve some space for checksum
        int remain = m_cacheSize - m_cacheBytes - len - 2;
        if (remain < 0) {
          // m_cache full
          return;
        }
        // store data in m_cache
        memcpy(m_cache + m_cacheBytes, buf, len);
        m_cacheBytes += len;
        m_cache[m_cacheBytes++] = ',';
        if (m_next) m_next->dispatch(buf, len);
    }
protected:
    unsigned int m_cacheSize;
    unsigned int m_cacheBytes;
    char* m_cache;
private:
};

#ifdef ESP32
#define SD_CS_PIN 5
#else
#define SD_CS_PIN 10
#endif

class CStorageSD : public CStorageNull {
public:
  CStorageSD() {}
  bool init()
  {
      pinMode(SD_CS_PIN, OUTPUT);
      if(!SD.begin(SD_CS_PIN)){
          Serial.println("Card unmounted");
          return false;
      }
      int cardSize = SD.cardSize();
      Serial.printf("SD Card Size: %uMB\n", cardSize);
      return true;
    }
    bool begin(uint32_t dateTime = 0)
    {
      uint16_t fileIndex;
      char path[20] = "DATA";
      if (SD.exists(path)) {
          if (dateTime) {
             // using date and time as file name
             sprintf(path + 4, "/%08lu.CSV", dateTime);
          } else {
            // use index number as file name
            for (fileIndex = 1; fileIndex; fileIndex++) {
                sprintf(path + 4, "/DAT%05u.CSV", fileIndex);
                if (!SD.exists(path)) {
                    break;
                }
            }
            if (fileIndex == 0)
                return false;
          }
      } else {
          SD.mkdir(path);
          fileIndex = 1;
          sprintf(path + 4, "/DAT%05u.CSV", 1);
      }
      // O_READ | O_WRITE | O_CREAT = 0x13
      Serial.print("File:");
      Serial.println(path);
      sdfile = SD.open(path, 0x13);
      if (!sdfile) {
          Serial.println("File error");
          return false;
      }
      return true;
  }
  void end()
  {
      if (sdfile) sdfile.close();
  }
  void flush()
  {
      if (sdfile) sdfile.flush();
  }
  void dispatch(const char* buf, byte len)
  {
      if (sdfile) {
        // output data via serial
        sdfile.write((uint8_t*)buf, len);
        sdfile.write('\n');

        uint32_t logsize = sdfile.size();
      }
  }
  uint32_t size()
  {
    return sdfile.size();
  }
private:
  SDClass SD;
  File sdfile;
};

#endif
