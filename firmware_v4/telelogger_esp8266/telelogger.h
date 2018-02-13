/*************************************************************************
* Freematics Hub Helper Class
* Distributed under BSD license
* Developed by Stanley Huang <stanley@freematics.com.au>
*************************************************************************/

#define EVENT_LOGIN 1
#define EVENT_LOGOUT 2
#define EVENT_SYNC 3
#define EVENT_RECONNECT 4
#define EVENT_COMMAND 5
#define EVENT_ACK 6

#define MAX_SYNC_INTERVAL 60000

#if ENABLE_DATA_LOG
SDClass SD;
File sdfile;
#endif

class CDataHub {
public:
    void record(const char* buf, byte len)
    {
    }
    void dispatch(const char* buf, byte len)
    {
#if ENABLE_DATA_OUT
        // output data via serial
        Serial.write((uint8_t*)buf, len);
        Serial.write('\n');
#endif
        // reserve some space for checksum
        int remain = CACHE_SIZE - cacheBytes - len - 4;
        if (remain < 0) {
          // cache full
          return;
        }
        // store data in cache
        memcpy(cache + cacheBytes, buf, len);
        cacheBytes += len;
        cache[cacheBytes++] = ',';
#if ENABLE_DATA_LOG
        // write data to file
        sdfile.write(buf, len);
        sdfile.write('\n');
#endif
    }
    void logData(const char* buf, byte len)
    {
        dispatch(buf, len);
        record(buf, len);
    }
    void logData(uint16_t pid, int16_t value)
    {
        char buf[16];
        byte len = sprintf_P(buf, PSTR("%X=%d"), pid, value);
        dispatch(buf, len);
    }
    void logData(uint16_t pid, int32_t value)
    {
        char buf[20];
        byte len = sprintf_P(buf, PSTR("%X=%ld"), pid, value);
        dispatch(buf, len);
    }
    void logData(uint16_t pid, uint32_t value)
    {
        char buf[20];
        byte len = sprintf_P(buf, PSTR("%X=%lu"), pid, value);
        dispatch(buf, len);
    }
    void logData(uint16_t pid, int value1, int value2, int value3)
    {
        char buf[24];
        byte len = sprintf_P(buf, PSTR("%X=%d;%d;%d"), pid, value1, value2, value3);
        dispatch(buf, len);
    }
    void logCoordinate(uint16_t pid, int32_t value)
    {
        char buf[24];
        byte len = sprintf_P(buf, PSTR("%X=%d.%06lu"), pid, (int)(value / 1000000), abs(value) % 1000000);
        dispatch(buf, len);
    }
    void logTimestamp(uint32_t ts)
    {
        logData(0, m_dataTime = ts);
    }
    void addDataChecksum()
    {
      if (cacheBytes + 4 >= CACHE_SIZE) {
        // remove last data
        while (cacheBytes > 0 && cache[--cacheBytes] != ',');
      } else if (cacheBytes > 0 && cache[cacheBytes - 1] == ',') {
        cacheBytes--; // last delimiter unneeded
      }
      // calculate and add checksum
      byte sum = 0;
      for (unsigned int i = 0; i < cacheBytes; i++) sum += cache[i];
      cacheBytes += sprintf_P(cache + cacheBytes, PSTR("*%X"), sum);
    }
    bool verifyChecksum(const char* data)
    {
        uint8_t sum = 0;
        const char *s;
        for (s = data; *s && *s != '*'; s++) sum += *s;
        return (*s && hex2uint8(s + 1) == sum);
    }
    void setEvent(byte event)
    {
        cacheBytes = sprintf_P(cache, PSTR("%X#EV=%u,SK=%s,TS=%lu"), feedid, (unsigned int)event, SERVER_KEY, millis());
    }
    void purgeCache()
    {
        cacheBytes = sprintf_P(cache, PSTR("%X#"), feedid);
    }
#if ENABLE_DATA_LOG
    uint16_t openFile(uint32_t dateTime = 0)
    {
        uint16_t fileIndex;
        char path[20] = "/DATA";

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
    char cache[CACHE_SIZE];
    unsigned int cacheBytes = 0;
    uint16_t feedid = 0;
private:
    uint32_t m_dataTime = 0;
};
