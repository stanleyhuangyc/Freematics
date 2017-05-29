/*************************************************************************
* Telematics Data Logger Class
* Distributed under BSD license
* Developed by Stanley Huang https://www.facebook.com/stanleyhuangyc
*************************************************************************/

// additional custom PID for data logger
#define PID_DATA_SIZE 0x80

#if ENABLE_DATA_LOG
SDClass SD;
File sdfile;
#endif

class CDataLogger {
public:
    CDataLogger():m_dataTime(0)
    {
#if ENABLE_DATA_CACHE
        cacheBytes = 0;
#endif
    }
    void record(const char* buf, byte len)
    {
#if ENABLE_DATA_LOG
        char tmp[12];
        // absolute timestamp
        sdfile.print(m_dataTime);
        sdfile.write(',');
        sdfile.write(tmp, n);
        sdfile.write(buf, len);
        sdfile.write('\n');
#endif
    }
    void dispatch(const char* buf, byte len)
    {
#if ENABLE_DATA_CACHE
        // reserve some space for timestamp, ending white space and zero terminator
        int l = cacheBytes + len + 12 - CACHE_SIZE;
        if (l >= 0) {
          // cache full
#if CACHE_SHIFT
#if ENABLE_MULTI_THREADING
          cacheLock.lock();
#endif
          // discard the oldest data
          for (l = CACHE_SIZE / 2; cache[l] && cache[l] != ' '; l++);
          if (cache[l]) {
            cacheBytes -= l;
            memcpy(cache, cache + l + 1, cacheBytes);
          } else {
            cacheBytes = 0;
          }
#if ENABLE_MULTI_THREADING
          cacheLock.unlock();
#endif
#else
          return;
#endif
        }
#if ENABLE_MULTI_THREADING
        cacheLock.lock();
#endif
        if (cacheBytes + len < CACHE_SIZE - 1) {
          memcpy(cache + cacheBytes, buf, len);
          cacheBytes += len;
          cache[cacheBytes++] = ' ';
        }
        cache[cacheBytes] = 0;
#if ENABLE_MULTI_THREADING
        cacheLock.unlock();
#endif
#endif
#if ENABLE_DATA_OUT
        Serial.write((uint8_t*)buf, len);
        Serial.write('\n');
#endif
    }
    void logData(const char* buf, byte len)
    {
        dispatch(buf, len);
        record(buf, len);
    }
    void logData(uint16_t pid)
    {
        char buf[8];
        byte len = translatePIDName(pid, buf);
        dispatch(buf, len);
        record(buf, len);
    }
    void logData(uint16_t pid, int16_t value)
    {
        char buf[16];
        byte n = translatePIDName(pid, buf);
        byte len = sprintf_P(buf + n, PSTR("%d"), value) + n;
        dispatch(buf, len);
        record(buf, len);
    }
    void logData(uint16_t pid, int32_t value)
    {
        char buf[20];
        byte n = translatePIDName(pid, buf);
        byte len = sprintf_P(buf + n, PSTR("%ld"), value) + n;
        dispatch(buf, len);
        record(buf, len);
    }
    void logData(uint16_t pid, uint32_t value)
    {
        char buf[20];
        byte n = translatePIDName(pid, buf);
        byte len = sprintf_P(buf + n, PSTR("%lu"), value) + n;
        dispatch(buf, len);
        record(buf, len);
    }
    void logData(uint16_t pid, int value1, int value2, int value3)
    {
        char buf[24];
        byte n = translatePIDName(pid, buf);
        n += sprintf_P(buf + n, PSTR("%d/%d/%d"), value1, value2, value3);
        dispatch(buf, n);
        record(buf, n);
    }
    void logCoordinate(uint16_t pid, int32_t value)
    {
        char buf[24];
        byte len = translatePIDName(pid, buf);
        len += sprintf_P(buf + len, PSTR("%d.%06lu"), (int)(value / 1000000), abs(value) % 1000000);
        dispatch(buf, len);
        record(buf, len);
    }
    void logTimestamp()
    {
        logData(0, m_dataTime = millis());
    }
#if ENABLE_DATA_LOG
    uint16_t openFile(uint32_t dateTime = 0)
    {
        uint16_t fileIndex;
        char path[20] = "/DATA";

        dataSize = 0;
        if (SD.exists(path)) {
            if (dateTime) {
               // using date and time as file name
               sprintf(path + 5, "/%08lu.CSV", dateTime);
               fileIndex = 1;
            } else {
              // use index number as file name
              for (fileIndex = 1; fileIndex; fileIndex++) {
                  sprintf(path + 5, "/DAT%05u.CSV", fileIndex);
                  if (!SD.exists(path)) {
                      break;
                  }
              }
              if (fileIndex == 0)
                  return 0;
            }
        } else {
            SD.mkdir(path);
            fileIndex = 1;
            sprintf(path + 5, "/DAT%05u.CSV", 1);
        }

        sdfile = SD.open(path, FILE_WRITE);
        if (!sdfile) {
            return 0;
        }
        return fileIndex;
    }
    void closeFile()
    {
        sdfile.close();
    }
    void flushFile()
    {
        sdfile.flush();
    }
#endif
#if ENABLE_DATA_CACHE
    void purgeCache()
    {
      cacheBytes = 0;
    }
    char cache[CACHE_SIZE];
    unsigned int cacheBytes;
#if ENABLE_MULTI_THREADING
    Mutex cacheLock;
#endif
#endif
private:
    byte translatePIDName(uint16_t pid, char* text)
    {
        return sprintf_P(text, PSTR("%X:"), pid);
    }
    uint32_t m_dataTime;
};
