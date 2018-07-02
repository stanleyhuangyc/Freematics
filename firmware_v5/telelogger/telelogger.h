class CStorageNull;

class CStorageNull {
public:
    virtual bool init() { return true; }
    virtual void uninit() {}
    virtual void log(uint16_t pid, int value)
    {
        char buf[24];
        byte len = sprintf_P(buf, PSTR("%X=%d"), pid, value);
        dispatch(buf, len);
    }
    virtual void log(uint16_t pid, unsigned int value)
    {
        char buf[24];
        byte len = sprintf_P(buf, PSTR("%X=%u"), pid, value);
        dispatch(buf, len);
    }
    virtual void log(uint16_t pid, float value)
    {
        char buf[24];
        byte len = sprintf_P(buf, PSTR("%X=%.2f"), pid, value);
        dispatch(buf, len);
    }
    virtual void log(uint16_t pid, int value1, int value2, int value3)
    {
        char buf[48];
        byte len = sprintf_P(buf, PSTR("%X=%d;%d;%d"), pid, value1, value2, value3);
        dispatch(buf, len);
    }
    virtual void logCoordinate(uint16_t pid, int32_t value)
    {
        char buf[24];
        byte len = sprintf_P(buf, PSTR("%X=%d.%06u"), pid, (int)(value / 1000000), abs(value) % 1000000);
        dispatch(buf, len);
    }
    virtual void timestamp(uint32_t ts)
    {
        log(0, ts);
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

SDClass SD;

class CStorageSD : public CStorageNull {
public:
    bool init()
    {
        pinMode(PIN_SD_CS, OUTPUT);
        if(!SD.begin(PIN_SD_CS)){
            Serial.println("No SD card");
            return false;
        }
        int cardSize = SD.cardSize();
        Serial.print("SD card size:");
        Serial.print(cardSize);
        Serial.println("MB");
        return true;
    }
    int begin()
    {
        char path[24] = "/DATA";
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
            return 0;
        }
        return m_id;
    }
    void end()
    {
        m_file.close();
        m_id = 0;
    }
    void dispatch(const char* buf, byte len)
    {
        if (m_file.write((uint8_t*)buf, len) == len) {
            m_file.write('\n');
            uint16_t sizeKB = m_file.size() >> 10;
            if (sizeKB != m_sizeKB) {
                m_file.flush();
                m_sizeKB = sizeKB;
            }
        }
        if (m_next) m_next->dispatch(buf, len);
    }
    uint32_t size()
    {
        return m_file.size();
    }
private:
    SDLib::File m_file;
    int m_id = 0;
    uint16_t m_sizeKB = 0;
};

class CStorageSPIFFS : public CStorageNull {
public:
    bool init()
    {
        return SPIFFS.begin(true);
    }
    void dispatch(const char* buf, byte len)
    {
        if (m_file.write((uint8_t*)buf, len) != len) {
            purge();
            if (m_file.write((uint8_t*)buf, len) != len) {
                Serial.println("Error writing data");
            }
        }
        m_file.write('\n');
        if (m_next) m_next->dispatch(buf, len);
    }
    int begin()
    {
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
    void end()
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
    int m_id = 0;
};
