#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <SPIFFS.h>

class CStorage;

class CStorage {
public:
    virtual bool init() { return true; }
    virtual void uninit() {}
    virtual void log(uint16_t pid, int value)
    {
        char buf[24];
        byte len = sprintf(buf, "%X%c%d", pid, m_delimiter, value);
        dispatch(buf, len);
    }
    virtual void log(uint16_t pid, uint32_t value)
    {
        char buf[24];
        byte len = sprintf(buf, "%X%c%u", pid, m_delimiter, value);
        dispatch(buf, len);
    }
    virtual void log(uint16_t pid, float value)
    {
        char buf[24];
        byte len = sprintf(buf, "%X%c%f", pid, m_delimiter, value);
        dispatch(buf, len);
    }
    virtual void log(uint16_t pid, float value[])
    {
        char buf[48];
        byte len = sprintf(buf, "%X%c%.2f;%.2f;%.2f", pid, m_delimiter, value[0], value[1], value[2]);
        dispatch(buf, len);
    }
    virtual void timestamp(uint32_t ts)
    {
        log(0, ts);
    }
    virtual void purge() { m_samples = 0; }
    virtual uint16_t samples() { return m_samples; }
    virtual void dispatch(const char* buf, byte len)
    {
        // output data via serial
        Serial.write((uint8_t*)buf, len);
        Serial.write(' ');
        m_samples++;
    }
protected:
    byte checksum(const char* data, int len)
    {
        byte sum = 0;
        for (int i = 0; i < len; i++) sum += data[i];
        return sum;
    }
    virtual void header(const char* devid) {}
    virtual void tailer() {}
    uint16_t m_samples = 0;
    char m_delimiter = ':';
};

class CStorageRAM: public CStorage {
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
    }

    void header(const char* devid)
    {
        m_cacheBytes = sprintf(m_cache, "%s#", devid);
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

class FileLogger : public CStorage {
public:
    FileLogger() { m_delimiter = ','; }
    virtual void dispatch(const char* buf, byte len)
    {
        if (m_id == 0) return;

        if (m_file.write((uint8_t*)buf, len) != len) {
            // try again
            if (m_file.write((uint8_t*)buf, len) != len) {
                Serial.println("Error writing. End file logging.");
                end();
                return;
            }
        }
        m_file.write('\n');
        m_size += (len + 1);
    }
    virtual uint32_t size()
    {
        return m_size;
    }
    virtual void end()
    {
        m_file.close();
        m_id = 0;
        m_size = 0;
    }
    virtual void flush()
    {
        m_file.flush();
    }
protected:
    int getFileID(File& root)
    {
        if (root) {
            File file;
            int id = 0;
            while(file = root.openNextFile()) {
                Serial.println(file.name());
                if (!strncmp(file.name(), "/DATA/", 6)) {
                    unsigned int n = atoi(file.name() + 6);
                    if (n > id) id = n;
                }
            }
            return id + 1;
        } else {
            return 1;
        }
    }
    uint32_t m_dataTime = 0;
    uint32_t m_dataCount = 0;
    uint32_t m_size = 0;
    uint32_t m_id = 0;
    File m_file;
};

class SDLogger : public FileLogger {
public:
    bool init()
    {
        SPI.begin();
        if (SD.begin(PIN_SD_CS, SPI, SPI_FREQ)) {
            unsigned int total = SD.totalBytes() >> 20;
            unsigned int used = SD.usedBytes() >> 20;
            Serial.print("SD:");
            Serial.print(total);
            Serial.print(" MB total, ");
            Serial.print(used);
            Serial.println(" MB used");
            return true;
        } else {
            Serial.println("NO SD CARD");
            return false;
        }
    }
    uint32_t begin()
    {
        File root = SD.open("/DATA");
        m_id = getFileID(root);
        SD.mkdir("/DATA");
        char path[24];
        sprintf(path, "/DATA/%u.CSV", m_id);
        Serial.print("File: ");
        Serial.println(path);
        m_file = SD.open(path, FILE_WRITE);
        if (!m_file) {
            Serial.println("File error");
            m_id = 0;
        }
        m_dataCount = 0;
        return m_id;
    }
    void flush()
    {
        char path[24];
        sprintf(path, "/DATA/%u.CSV", m_id);
        m_file.close();
        m_file = SD.open(path, FILE_APPEND);
        if (!m_file) {
            Serial.println("File error");
        }
    }
};

class SPIFFSLogger : public FileLogger {
public:
    bool init()
    {
        bool mounted = SPIFFS.begin();
        if (!mounted) {
            Serial.println("Formatting SPIFFS...");
            mounted = SPIFFS.begin(true);
        }
        if (mounted) {
            Serial.print("SPIFFS:");
            Serial.print(SPIFFS.totalBytes());
            Serial.print(" bytes total, ");
            Serial.print(SPIFFS.usedBytes());
            Serial.println(" bytes used");
        } else {
            Serial.println("No SPIFFS");
        }
        return mounted;
    }
    uint32_t begin()
    {
        File root = SPIFFS.open("/");
        m_id = getFileID(root);
        char path[24];
        sprintf(path, "/DATA/%u.CSV", m_id);
        Serial.print("File: ");
        Serial.println(path);
        m_file = SPIFFS.open(path, FILE_WRITE);
        if (!m_file) {
            Serial.println("File error");
            m_id = 0;
        }
        m_dataCount = 0;
        return m_id;
    }
private:
    void purge()
    {
        // remove oldest file when unused space is insufficient
        File root = SPIFFS.open("/");
        File file;
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
};
