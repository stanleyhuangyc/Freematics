#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <SPIFFS.h>

class CStorage;

class CStorage {
public:
    virtual bool init() { return true; }
    virtual void uninit() {}
    virtual void log(uint16_t pid, uint8_t values[], uint8_t count);
    virtual void log(uint16_t pid, uint16_t values[], uint8_t count);
    virtual void log(uint16_t pid, uint32_t values[], uint8_t count);
    virtual void log(uint16_t pid, int32_t values[], uint8_t count);
    virtual void log(uint16_t pid, float values[], uint8_t count, const char* fmt = "%f");
    virtual void timestamp(uint32_t ts);
    virtual void purge() { m_samples = 0; }
    virtual uint16_t samples() { return m_samples; }
    virtual void dispatch(const char* buf, byte len);
protected:
    byte checksum(const char* data, int len);
    virtual void header(const char* devid) {}
    virtual void tailer() {}
    int m_samples = 0;
    char m_delimiter = ':';
};

class CStorageRAM: public CStorage {
public:
    void init(char* cache, unsigned int cacheSize)
    {
        m_cacheSize = cacheSize;
        m_cache = cache;
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
    void dispatch(const char* buf, byte len);
    void header(const char* devid);
    void tailer();
    void untailer();
protected:
    unsigned int m_cacheSize = 0;
    unsigned int m_cacheBytes = 0;
    char* m_cache = 0;
};

class FileLogger : public CStorage {
public:
    FileLogger() { m_delimiter = ','; }
    virtual void dispatch(const char* buf, byte len);
    virtual uint32_t size() { return m_size; }
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
    int getFileID(File& root);
    uint32_t m_dataTime = 0;
    uint32_t m_dataCount = 0;
    uint32_t m_size = 0;
    uint32_t m_id = 0;
    File m_file;
};

class SDLogger : public FileLogger {
public:
    bool init();
    uint32_t begin();
    void flush();
};

class SPIFFSLogger : public FileLogger {
public:
    bool init();
    uint32_t begin();
private:
    void purge();
};
