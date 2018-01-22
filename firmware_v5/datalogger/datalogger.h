/*************************************************************************
* Telematics Data Logger Class
* Distributed under BSD license
* Written by Stanley Huang <support@freematics.com.au>
* Visit http://freematics.com for more information
*************************************************************************/

SDClass SD;
File sdfile;

class CDataLogger {
public:
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
        dataCount++;
    }
    void log(uint16_t pid, int16_t value)
    {
        char buf[24];
        byte len = sprintf_P(buf, PSTR("%lu,%X,%d"), dataTime, pid, value) ;
        record(buf, len);
    }
    void log(uint16_t pid, int32_t value)
    {
        char buf[28];
        byte len = sprintf_P(buf, PSTR("%lu,%X,%ld"), dataTime, pid, value);
        record(buf, len);
    }
    void log(uint16_t pid, uint32_t value)
    {
        char buf[28];
        byte len = sprintf_P(buf, PSTR("%lu,%X,%lu"), dataTime, pid, value);
        record(buf, len);
    }
    void log(uint16_t pid, int value1, int value2, int value3)
    {
        char buf[32];
        byte len = sprintf_P(buf, PSTR("%lu,%X,%d;%d;%d"), dataTime, pid, value1, value2, value3);
        record(buf, len);
    }
    void logCoordinate(uint16_t pid, int32_t value)
    {
        char buf[32];
        byte len = sprintf_P(buf, PSTR("%lu,%X,%d.%06lu"), dataTime, pid, (int)(value / 1000000), abs(value) % 1000000);
        record(buf, len);
    }
    bool openFile(uint32_t dateTime = 0)
    {
        char path[16]; // = "/DATA";

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
    void closeFile()
    {
        sdfile.close();
    }
    void flushFile()
    {
        sdfile.flush();
    }
    uint32_t dataTime = 0;
    uint32_t dataCount = 0;
};
