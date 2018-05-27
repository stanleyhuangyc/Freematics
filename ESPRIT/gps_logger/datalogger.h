/*************************************************************************
* Helper classes for data logging and storage on microSD or SPIFFS
*
* Distributed under BSD license
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
        m_dataCount = 0;
        return 1;
    }
    virtual void log(uint16_t pid, int value)
    {
        char buf[24];
        byte len = sprintf(buf, "%X,%d", pid, value);
        write(buf, len);
        m_dataCount++;
    }
    virtual void log(uint16_t pid, unsigned int value)
    {
        char buf[24];
        byte len = sprintf(buf, "%X,%u", pid, value);
        write(buf, len);
        m_dataCount++;
    }
    virtual void log(uint16_t pid, float value)
    {
        char buf[24];
        byte len = sprintf(buf, "%X,%f", pid, value);
        write(buf, len);
        m_dataCount++;
    }
    virtual void log(uint16_t pid, int value1, int value2, int value3)
    {
        char buf[24];
        byte len = sprintf(buf, "%X,%d,%d,%d", pid, value1, value2, value3);
        write(buf, len);
        m_dataCount++;
    }
    virtual void setTimestamp(uint32_t ts)
    {
        char buf[16];
        byte len = sprintf(buf, "0,%u", m_dataTime = ts);
        write(buf, len);
    }
    virtual uint32_t open() { return 0; }
    virtual uint32_t size() { return 0; }
    virtual void close() {}
    virtual void flush() {}
    virtual void write(const char* buf, byte len)
    {
        Serial.println(buf);
    }
    virtual uint32_t getDataCount() { return m_dataCount; }
protected:
    uint32_t m_dataTime = 0;
    uint32_t m_dataCount = 0;
    uint32_t m_id = 0;
    NullLogger* m_next = 0;
};

SDClass SD;

class SDLogger : public NullLogger {
public:
    SDLogger(NullLogger* next = 0) { m_next = next; }
    int begin()
    {
        pinMode(PIN_SD_CS, OUTPUT);
        if (SD.begin(PIN_SD_CS)) {
          return SD.cardSize();
        } else {
          return -1;
        }
    }
    void write(const char* buf, byte len)
    {
        if (m_next) m_next->write(buf, len);
        m_file.write((uint8_t*)buf, len);
        m_file.write('\n');
    }
    uint32_t open()
    {
        char path[24] = "/DATA";
        m_dataCount = 0;
        SD.mkdir(path);
        SDLib::File root = SD.open(path);
        if (!root) {
            return 0;
        } else {
            SDLib::File file;
            m_id = 0;
            while(file = root.openNextFile()) {
                unsigned int n = atoi(file.name());
                if (n > m_id) m_id = n;
            }
            m_id++;
        }

        sprintf(path + strlen(path), "/%u.CSV", m_id);
        Serial.print("File:");
        Serial.println(path);
        m_file = SD.open(path, SD_FILE_READ | SD_FILE_WRITE);
        if (!m_file) {
            Serial.println("File error");
            m_id = 0;
            return 0;
        }
        return m_id;
    }
    uint32_t size()
    {
        return m_file.size();
    }
    void close()
    {
        m_file.close();
    }
    void flush()
    {
        m_file.flush();
    }
private:
    SDLib::File m_file;
};

class SPIFFSLogger : public NullLogger {
public:
    SPIFFSLogger(NullLogger* next = 0) { m_next = next; }
    int begin()
    {
        if (SPIFFS.begin(true)) {
            return SPIFFS.totalBytes() - SPIFFS.usedBytes();
        }
        return -1;
    }
    void write(const char* buf, byte len)
    {
        if (m_next) m_next->write(buf, len);
        if (m_id == 0) return;
        if (m_file.write((uint8_t*)buf, len) != len) {
            purge();
            if (m_file.write((uint8_t*)buf, len) != len) {
                Serial.println("Error writing data");
                return;
            }
        }
        m_file.write('\n');
    }
    uint32_t open()
    {
        m_dataCount = 0;
        fs::File root = SPIFFS.open("/");
        if (!root) {
            return 0;
        } else {
            fs::File file;
            m_id = 0;
            while(file = root.openNextFile()) {
                if (!strncmp(file.name(), "/DATA/", 6)) {
                    unsigned int n = atoi(file.name() + 6);
                    if (n > m_id) m_id = n;
                }
            }
            m_id++;
        }
        char path[24];
        sprintf(path, "/DATA/%u.CSV", m_id);
        Serial.print("File: ");
        Serial.println(path);
        m_file = SPIFFS.open(path, FILE_WRITE);
        if (!m_file) {
            Serial.println("File error");
            m_id = 0;
            return 0;
        }
        return m_id;
    }
    uint32_t size()
    {
        return m_file.size();
    }
    void close()
    {
        m_file.close();
        m_id = 0;
    }
    void flush()
    {
        m_file.flush();
    }
private:
    void purge()
    {
        // remove oldest file when unused space is insufficient
        fs::File root = SPIFFS.open("/");
        fs::File file;
        int idx = 0;
        while(file = root.openNextFile()) {
            if (!strncmp(file.name(), "/DATA/", 6)) {
                unsigned int n = atoi(file.name() + 6);
                if (n != 0 && (idx == 0 || n < idx)) idx = n;
            }
        }
        if (idx) {
            m_file.close();
            char path[32];
            sprintf(path, "/DATA/%u.CSV", idx);
            SPIFFS.remove(path);
            Serial.print(path);
            Serial.println(" removed");
            sprintf(path, "/DATA/%u.CSV", m_id);
            m_file = SPIFFS.open(path, FILE_APPEND);
            if (!m_file) m_id = 0;
        }
    }
    fs::File m_file;
};
