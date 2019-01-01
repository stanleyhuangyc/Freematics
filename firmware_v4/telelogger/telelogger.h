#define EVENT_LOGIN 1
#define EVENT_LOGOUT 2
#define EVENT_SYNC 3
#define EVENT_RECONNECT 4
#define EVENT_COMMAND 5
#define EVENT_ACK 6
#define EVENT_PING 7

#define CACHE_SIZE 160

class CStorageNull;

class CStorageNull {
public:
    virtual bool init() { return true; }
    virtual void uninit() {}
    virtual bool begin() { return true; }
    virtual void end() {}
    virtual void log(uint16_t pid, int value)
    {
        char buf[16];
        byte len = sprintf_P(buf, PSTR("%X:%d"), pid, value);
        dispatch(buf, len);
    }
    virtual void log(uint16_t pid, unsigned int value)
    {
        char buf[16];
        byte len = sprintf_P(buf, PSTR("%X:%u"), pid, value);
        dispatch(buf, len);
    }
    virtual void log(uint16_t pid, int32_t value)
    {
        char buf[20];
        byte len = sprintf_P(buf, PSTR("%X:%ld"), pid, value);
        dispatch(buf, len);
    }
    virtual void log(uint16_t pid, uint32_t value)
    {
        char buf[20];
        byte len = sprintf_P(buf, PSTR("%X:%lu"), pid, value);
        dispatch(buf, len);
    }
    virtual void log(uint16_t pid, int value1, int value2, int value3)
    {
        char buf[24];
        byte len = sprintf_P(buf, PSTR("%X:%d;%d;%d"), pid, value1, value2, value3);
        dispatch(buf, len);
    }
    virtual void logCoordinate(uint16_t pid, int32_t value)
    {
        char buf[24];
        byte len = sprintf_P(buf, PSTR("%X:%d.%06lu"), pid, (int)(value / 1000000), abs(value) % 1000000);
        dispatch(buf, len);
    }
    virtual void timestamp(uint32_t ts)
    {
        m_dataTime = ts;
        if (m_next) m_next->timestamp(ts);
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
    uint32_t m_dataTime = 0;
    uint16_t m_samples = 0;
    CStorageNull* m_next = 0;
};

class CStorageRAM: public CStorageNull {
public:
    void purge() { m_cacheBytes = 0; m_samples = 0; }
    unsigned int length() { return m_cacheBytes; }
    char* buffer() { return m_cache; }
    void dispatch(const char* buf, byte len)
    {
        // reserve some space for checksum
        int remain = CACHE_SIZE - m_cacheBytes - len - 3;
        if (remain < 0) {
          // m_cache full
          return;
        }
        if (m_dataTime) {
          // log timestamp
          uint32_t ts = m_dataTime;
          m_dataTime = 0;
          log(0, ts);
        }
        // store data in m_cache
        memcpy(m_cache + m_cacheBytes, buf, len);
        m_cacheBytes += len;
        m_cache[m_cacheBytes++] = ',';
        m_samples++;
        if (m_next) m_next->dispatch(buf, len);
    }

    void header(uint16_t feedid, const char* devid = 0)
    {
        if (feedid || !devid)
            m_cacheBytes = sprintf_P(m_cache, PSTR("%X#"), (unsigned int)feedid);
        else
            m_cacheBytes = sprintf_P(m_cache, PSTR("%s#"), devid);
    }
    void tailer()
    {
        if (m_cache[m_cacheBytes - 1] == ',') m_cacheBytes--;
        m_cacheBytes += sprintf_P(m_cache + m_cacheBytes, PSTR("*%X"), (unsigned int)checksum(m_cache, m_cacheBytes));
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
    unsigned int m_cacheBytes = 0;
    char m_cache[CACHE_SIZE];
};
