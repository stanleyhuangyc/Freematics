/*************************************************************************
* OBD/MEMS/GPS Data Logger for Freematics ONE+
* Data logging and file storage on microSD or SPIFFS
*
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one-plus for more information
* Developed by Stanley Huang <stanley@freematics.com.au>
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*************************************************************************/

class NullLogger {
public:
    virtual int begin()
    {
        return 0;
    }
    virtual void log(uint16_t pid, int16_t value)
    {
        char buf[24];
        byte len = sprintf_P(buf, PSTR("%lu,%X,%d"), m_dataTime, pid, value) ;
        write(buf, len);
    }
    virtual void log(uint16_t pid, int32_t value)
    {
        char buf[28];
        byte len = sprintf_P(buf, PSTR("%lu,%X,%ld"), m_dataTime, pid, value);
        write(buf, len);
    }
    virtual void log(uint16_t pid, uint32_t value)
    {
        char buf[28];
        byte len = sprintf_P(buf, PSTR("%lu,%X,%lu"), m_dataTime, pid, value);
        write(buf, len);
    }
    virtual void log(uint16_t pid, int value1, int value2, int value3)
    {
        char buf[32];
        byte len = sprintf_P(buf, PSTR("%lu,%X,%d;%d;%d"), m_dataTime, pid, value1, value2, value3);
        write(buf, len);
    }
    virtual void logCoordinate(uint16_t pid, int32_t value)
    {
        char buf[32];
        byte len = sprintf_P(buf, PSTR("%lu,%X,%d.%06lu"), m_dataTime, pid, (int)(value / 1000000), abs(value) % 1000000);
        write(buf, len);
    }
    virtual bool open(uint32_t dateTime = 0) { m_dataCount = 0; }
    virtual uint32_t size() { return 0; }
    virtual void close() {}
    virtual void flush() {}
    virtual void write(const char* buf, byte len)
    {
        Serial.println(buf);
    }
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
    void write(const char* buf, byte len)
    {
        file.write((uint8_t*)buf, len);
        file.write('\n');
        m_dataCount++;
#if ENABLE_SERIAL_OUT
        Serial.println(buf);
#endif
    }
    bool open(uint32_t dateTime = 0)
    {
        char path[24] = "/DATA";

        SD.mkdir(path);
        m_dataCount = 0;
        if (dateTime) {
           // using date and time as file name
           sprintf(path, "/DATA/%08lu.CSV", dateTime);
        } else {
          // use index number as file name
          uint16_t fileIndex;
          for (fileIndex = 1; fileIndex; fileIndex++) {
              sprintf(path, "/DATA/LOG%05u.CSV", fileIndex);
              if (!SD.exists(path)) {
                  break;
              }
          }
          if (fileIndex == 0)
              return false;
        }
        Serial.print("File:");
        Serial.println(path);
        file = SD.open(path, SD_FILE_WRITE);
        if (!file) {
            Serial.println("File error");
            return false;
        }
        return true;
    }
    uint32_t size()
    {
        return file.size();
    }
    void close()
    {
        file.close();
    }
    void flush()
    {
        file.flush();
    }
private:
    SDClass SD;
    SDLib::File file;
};

class SPIFFSLogger : public NullLogger {
public:
    int begin()
    {
        if (SPIFFS.begin(true)) {
            return SPIFFS.totalBytes() - SPIFFS.usedBytes();
        }
        return -1;
    }
    void write(const char* buf, byte len)
    {
        file.write((uint8_t*)buf, len);
        file.write('\n');
        m_dataCount++;
#if ENABLE_SERIAL_OUT
        Serial.println(buf);
#endif
    }
    bool open(uint32_t dateTime = 0)
    {
        char path[24];
        unsigned int idx = 0;
        m_dataCount = 0;
        fs::File root = SPIFFS.open("/");
        if (!root) {
            return false;
        } else {
            fs::File file;
            while(file = root.openNextFile()) {
                if (!strncmp(file.name(), "/DATA/", 6)) {
                    int n = atoi(file.name() + 6);
                    if (n > idx) idx = n;
                }
            }
        }
        sprintf(path, "/DATA/%08u.CSV", ++idx);
        Serial.print("New file: ");
        Serial.println(path);
        file = SPIFFS.open(path, FILE_WRITE);
        if (!file) {
            Serial.println("File error");
            return false;
        }
        return true;
    }
    uint32_t size()
    {
        return file.size();
    }
    void close()
    {
        file.close();
    }
    void flush()
    {
        file.flush();
    }
private:
    fs::File file;
};
