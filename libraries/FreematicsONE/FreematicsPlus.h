/*************************************************************************
* Arduino library for Freematics ONE+ (ESP32)
* Distributed under BSD license
* Visit http://freematics.com for more information
* (C)2017 Developed by Stanley Huang <support@freematics.com.au>
*************************************************************************/

#include <Arduino.h>
#include "FreematicsBase.h"
#include "FreematicsNetwork.h"

#ifdef ESP32

bool gps_decode_start();
void gps_decode_stop();
bool gps_get_data(GPS_DATA* gdata);
int gps_write_string(const char* string);
void gps_decode_task(int timeout);
void bee_start();
int bee_write_string(const char* string);
int bee_write_data(uint8_t* data, int len);
int bee_read(uint8_t* buffer, size_t bufsize, unsigned int timeout);
void bee_flush();
uint8_t readChipTemperature();

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
	virtual int xbReceive(char* buffer, int bufsize, unsigned int timeout = 1000, const char** expected = 0, byte expectedCount = 0);
	// purge xBee UART buffer
	virtual void xbPurge();
	// toggle xBee module power
	virtual void xbTogglePower();
  // delay specified number of ms while still receiving and processing GPS data
  virtual void sleep(unsigned int ms);
  // hibernate (lower power consumption)
  virtual void hibernate(unsigned int ms);
};

#endif

class CStorageNull;

class CStorageNull {
public:
    virtual bool init() { return true; }
    virtual void uninit() {}
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
        m_dataTime = ts;
        if (m_next) m_next->timestamp(ts);
    }
    virtual void setForward(CStorageNull* next) { m_next = next; }
    virtual void purge() { m_samples = 0; }
    virtual uint16_t samples() { return m_samples; }
    virtual void dispatch(const char* buf, byte len)
    {
        // output data via serial
        Serial.write((uint8_t*)buf, len);
        Serial.write('\n');
        m_samples++;
        if (m_next) m_next->dispatch(buf, len);
    }
protected:
    byte checksum(const char* data, int len)
    {
        byte sum = 0;
        for (int i = 0; i < len; i++) sum += data[i];
        return sum;
    }
    virtual void header(uint16_t feedid) {}
    virtual void tailer() {}
    uint32_t m_dataTime = 0;
    uint16_t m_samples = 0;
    CStorageNull* m_next = 0;
};

class CStorageRAM: public CStorageNull {
public:
    bool init(unsigned int cacheSize)
    {
      if (m_cacheSize != cacheSize) {
        uninit();
        m_cache = new char[m_cacheSize = cacheSize];
      }
      return true;
    }
    void uninit()
    {
        if (m_cache) {
            delete m_cache;
            m_cache = 0;
            m_cacheSize = 0;
        }
    }
    void purge() { m_cacheBytes = 0; m_samples = 0; }
    unsigned int length() { return m_cacheBytes; }
    char* buffer() { return m_cache; }
    void dispatch(const char* buf, byte len)
    {
        // reserve some space for checksum
        int remain = m_cacheSize - m_cacheBytes - len - 3;
        if (remain < 0) {
          // m_cache full
          return;
        }
        if (m_dataTime) {
          // log timestamp
          uint32_t ts = m_dataTime;
          m_dataTime = 0;
          log(0, ts);
        }
        // store data in m_cache
        memcpy(m_cache + m_cacheBytes, buf, len);
        m_cacheBytes += len;
        m_cache[m_cacheBytes++] = ',';
        m_samples++;
        if (m_next) m_next->dispatch(buf, len);
    }

    void header(uint16_t feedid)
    {
        m_cacheBytes = sprintf(m_cache, "%X#", (unsigned int)feedid);
    }
    void tailer()
    {
        if (m_cache[m_cacheBytes - 1] == ',') m_cacheBytes--;
        m_cacheBytes += sprintf(m_cache + m_cacheBytes, "*%X", (unsigned int)checksum(m_cache, m_cacheBytes));
    }
    void untailer()
    {
        char *p = strrchr(m_cache, '*');
        if (p) {
            *p = ',';
            m_cacheBytes = p + 1 - m_cache;
        }
    }
protected:
    unsigned int m_cacheSize = 0;
    unsigned int m_cacheBytes = 0;
    char* m_cache = 0;
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
          Serial.println("No SD card");
          return false;
      }
      int cardSize = SD.cardSize();
      Serial.print("SD card size:");
      Serial.print(cardSize);
      Serial.println("MB");
      return true;
    }
    bool begin(uint32_t dateTime = 0)
    {
      uint16_t fileIndex;
      char path[20];
      if (dateTime) {
        // using year as directory name
        sprintf(path, "%04u", (unsigned int)(dateTime / 10000));
        SD.mkdir(path);
        // using date and time as file name
        sprintf(path + 4, "/%08lu.CSV", dateTime);
      } else {
        strcpy(path, "DATA");
        SD.mkdir(path);
        // use index number as file name
        for (fileIndex = 1; fileIndex; fileIndex++) {
          sprintf(path + 4, "/DAT%05u.CSV", fileIndex);
          if (!SD.exists(path)) {
              break;
          }
        }
        if (fileIndex == 0) return false;
      }

      Serial.print("File:");
      Serial.println(path);
      // O_READ | O_WRITE | O_CREAT = 0x13
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
  void dispatch(const char* buf, byte len)
  {
      if (sdfile) {
        sdfile.write((uint8_t*)buf, len);
        sdfile.write('\n');
        uint16_t sizeKB = sdfile.size() >> 10;
        if (sizeKB != m_sizeKB) {
            sdfile.flush();
            m_sizeKB = sizeKB;
        }
      }
  }
  uint32_t size()
  {
    return sdfile.size();
  }
private:
  SDClass SD;
  File sdfile;
  uint16_t m_sizeKB = 0;
};
