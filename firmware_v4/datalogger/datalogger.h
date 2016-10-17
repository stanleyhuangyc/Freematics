/*************************************************************************
* Telematics Data Logger Class
* Distributed under BSD license
* Written by Stanley Huang <support@freematics.com.au>
* Visit http://freematics.com for more information
*************************************************************************/

// additional custom PID for data logger
#define PID_DATA_SIZE 0x80

#if ENABLE_DATA_OUT

#if defined(RF_SERIAL)
#define SerialRF RF_SERIAL
#else
#define SerialRF Serial
#endif

#endif

#if ENABLE_DATA_LOG
static File sdfile;
#endif

typedef struct {
    uint8_t pid;
    char name[3];
} PID_NAME;

const PID_NAME pidNames[] PROGMEM = {
{PID_ACC, {'A','C','C'}},
{PID_GYRO, {'G','Y','R'}},
{PID_COMPASS, {'M','A','G'}},
{PID_GPS_LATITUDE, {'L','A','T'}},
{PID_GPS_LONGITUDE, {'L','N','G'}},
{PID_GPS_ALTITUDE, {'A','L','T'}},
{PID_GPS_SPEED, {'S','P','D'}},
{PID_GPS_HEADING, {'C','R','S'}},
{PID_GPS_SAT_COUNT, {'S','A','T'}},
{PID_GPS_TIME, {'U','T','C'}},
{PID_GPS_DATE, {'D','T','E'}},
{PID_BATTERY_VOLTAGE, {'B','A','T'}},
{PID_DATA_SIZE, {'D','A','T'}},
};

class CDataLogger {
public:
    CDataLogger():m_lastDataTime(0),dataTime(0),dataSize(0)
    {
#if ENABLE_DATA_CACHE
        cacheBytes = 0;
#endif
    }
    void initSender()
    {
#if ENABLE_DATA_OUT
        SerialRF.begin(STREAM_BAUDRATE);
#endif
    }
    byte genTimestamp(char* buf, bool absolute)
    {
      byte n;
      if (absolute || dataTime >= m_lastDataTime + 60000) {
        // absolute timestamp
        n = sprintf(buf, "#%lu", dataTime);
      } else {
        // relative timestamp
        n += sprintf(buf, "%u", (unsigned int)(dataTime - m_lastDataTime));
      }
      buf[n++] = ',';      
      return n;
    }
    void record(const char* buf, byte len)
    {
#if ENABLE_DATA_LOG
        char tmp[12];
        byte n = genTimestamp(tmp, dataSize == 0);
        dataSize += sdfile.write(tmp, n);
        dataSize += sdfile.write(buf, len);
        sdfile.println();
        dataSize += 3;
#endif
        m_lastDataTime = dataTime;
    }
    void dispatch(const char* buf, byte len)
    {
#if ENABLE_DATA_CACHE
        if (cacheBytes + len < MAX_CACHE_SIZE - 10) {
          cacheBytes += genTimestamp(cache + cacheBytes, cacheBytes == 0);
          memcpy(cache + cacheBytes, buf, len);
          cacheBytes += len;
          cache[cacheBytes++] = ' ';
          cache[cacheBytes] = 0;
        }
#else
        //char tmp[12];
        //byte n = genTimestamp(tmp, dataTime >= m_lastDataTime + 100);
        //SerialRF.write(tmp, n);
#endif
#if ENABLE_DATA_OUT
        SerialRF.write(buf, len);
        SerialRF.println();
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
    void logData(uint16_t pid, int value)
    {
        char buf[16];
        byte n = translatePIDName(pid, buf);
        byte len = sprintf(buf + n, "%d", value) + n;
        dispatch(buf, len);
        record(buf, len);
    }
    void logData(uint16_t pid, int32_t value)
    {
        char buf[20];
        byte n = translatePIDName(pid, buf);
        byte len = sprintf(buf + n, "%ld", value) + n;
        dispatch(buf, len);
        record(buf, len);
    }
    void logData(uint16_t pid, uint32_t value)
    {
        char buf[20];
        byte n = translatePIDName(pid, buf);
        byte len = sprintf(buf + n, "%lu", value) + n;
        dispatch(buf, len);
        record(buf, len);
    }
    void logData(uint16_t pid, int value1, int value2, int value3)
    {
        char buf[24];
        byte n = translatePIDName(pid, buf);
        byte len = sprintf(buf + n, "%d,%d,%d", value1, value2, value3) + n;
        dispatch(buf, len);
        record(buf, len);
    }
    void logCoordinate(uint16_t pid, int32_t value)
    {
        char buf[24];
        byte len = translatePIDName(pid, buf);
        len += sprintf(buf + len, "%d.%06lu", (int)(value / 1000000), abs(value) % 1000000);
        dispatch(buf, len);
        record(buf, len);
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
        dataSize = 0;
    }
    void flushFile()
    {
        sdfile.flush();
    }
#endif
    uint32_t dataTime;
    uint32_t dataSize;
#if ENABLE_DATA_CACHE
    char cache[MAX_CACHE_SIZE];
    int cacheBytes;
#endif
private:
    byte translatePIDName(uint16_t pid, char* text)
    {
#if USE_FRIENDLY_PID_NAME
        for (uint16_t n = 0; n < sizeof(pidNames) / sizeof(pidNames[0]); n++) {
            uint16_t id = pgm_read_byte(&pidNames[n].pid);
            if (pid == id) {
                memcpy_P(text, pidNames[n].name, 3);
                text[3] = ',';
                return 4;
            }
        }
#endif
        return sprintf(text, "%X,", pid);
    }
    uint32_t m_lastDataTime;
};
