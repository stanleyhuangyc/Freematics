/*************************************************************************
* Telematics Data Logger Class
* Distributed under BSD license
* Written by Stanley Huang <support@freematics.com.au>
* Visit http://freematics.com for more information
*************************************************************************/

SDClass SD;
File sdfile;

class NullLogger {
public:
    virtual int begin() { return 0; }
    virtual void record(const char* buf, byte len) {}
    virtual void log(uint16_t pid, int16_t value)
    {
        char buf[24];
        byte len = sprintf_P(buf, PSTR("%lu,%X,%d"), m_dataTime, pid, value) ;
        record(buf, len);
    }
    virtual void log(uint16_t pid, int32_t value)
    {
        char buf[28];
        byte len = sprintf_P(buf, PSTR("%lu,%X,%ld"), m_dataTime, pid, value);
        record(buf, len);
    }
    virtual void log(uint16_t pid, uint32_t value)
    {
        char buf[28];
        byte len = sprintf_P(buf, PSTR("%lu,%X,%lu"), m_dataTime, pid, value);
        record(buf, len);
    }
    virtual void log(uint16_t pid, int value1, int value2, int value3)
    {
        char buf[32];
        byte len = sprintf_P(buf, PSTR("%lu,%X,%d;%d;%d"), m_dataTime, pid, value1, value2, value3);
        record(buf, len);
    }
    virtual void logCoordinate(uint16_t pid, int32_t value)
    {
        char buf[32];
        byte len = sprintf_P(buf, PSTR("%lu,%X,%d.%06lu"), m_dataTime, pid, (int)(value / 1000000), abs(value) % 1000000);
        record(buf, len);
    }
    virtual bool open(uint32_t dateTime = 0) { m_dataCount = 0; }
    virtual void close() {}
    virtual void flush() {}
    virtual void setTimestamp(uint32_t ts) { m_dataTime = ts; }
    virtual uint32_t getDataCount() { return m_dataCount; }
protected:
    uint32_t m_dataTime = 0;
    uint32_t m_dataCount = 0;
};

class SDLogger : public NullLogger {
public:
    int begin()
    {
        m_dataCount = 0;
        pinMode(PIN_SD_CS, OUTPUT);
        if (SD.begin(PIN_SD_CS)) {
          return SD.cardSize();
        } else {
          return -1;
        }
    }
    void record(const char* buf, byte len)
    {
        sdfile.write((uint8_t*)buf, len);
        sdfile.write('\n');
#if ENABLE_DATA_OUT
        // skip timestamp
        char *p = strchr(buf, ',');
        if (p++) {
          Serial.println(p);
        }
#endif
        m_dataCount++;
    }
    bool open(uint32_t dateTime = 0)
    {
        char path[16]; // = "/DATA";

        m_dataCount = 0;
        if (dateTime) {
           // using date and time as file name
           sprintf_P(path, PSTR("%08lu.CSV"), dateTime);
        } else {
          // use index number as file name
          uint16_t fileIndex;
          for (fileIndex = 1; fileIndex; fileIndex++) {
              sprintf_P(path, PSTR("DAT%05u.CSV"), fileIndex);
              if (!SD.exists(path)) {
                  break;
              }
          }
          if (fileIndex == 0)
              return false;
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
    void close()
    {
        sdfile.close();
    }
    void flush()
    {
        sdfile.flush();
    }
};
