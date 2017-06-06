/*************************************************************************
* Telematics Data Logger Class
* Distributed under BSD license
* Developed by Stanley Huang https://www.facebook.com/stanleyhuangyc
*************************************************************************/

#if ENABLE_DATA_LOG
SDClass SD;
File sdfile;
#endif

class CRAMCache {
public:
    CRAMCache():m_dataTime(0),m_cacheBytes(0),m_cache(0) {}
    void initCache(unsigned int cacheSize)
    {
      if (m_cache) delete m_cache;
      if (cacheSize) m_cache = new char[m_cacheSize = cacheSize];
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
    unsigned int getCacheBytes() { return m_cacheBytes; }
    char* getCache() { return m_cache; }
protected:
  void addHeader(int feedid)
  {
    if (m_cacheBytes + 4 >= m_cacheSize) {
      m_cacheBytes = m_cacheSize - 4;
    }
    // clear cache and write header for next transmission
    m_cacheBytes = sprintf_P(m_cache, PSTR("%X#"), feedid);
  }
  void addTail()
  {
    // calculate and add checksum
    byte sum = 0;
    for (unsigned int i = 0; i < m_cacheBytes; i++) sum += m_cache[i];
    m_cacheBytes += sprintf_P(m_cache + m_cacheBytes, PSTR("*%X"), sum);
  }
  void removeTail()
  {
    unsigned int i;
    for (i = m_cacheBytes - 1; m_cache[i] != '*' && i > 0; i--);
    if (i > 0) {
      m_cacheBytes = i;
    }
  }
  unsigned int m_cacheSize;
  unsigned int m_cacheBytes;
  char* m_cache;
private:
    void dispatch(const char* buf, byte len)
    {
  #if ENABLE_DATA_OUT
        // output data via serial
        Serial.write((uint8_t*)buf, len);
        Serial.write('\n');
  #endif
        // reserve some space for checksum
        int remain = m_cacheSize - m_cacheBytes - len - 4;
        if (remain < 0) {
          // m_cache full
          return;
        }
        // store data in m_cache
        memcpy(m_cache + m_cacheBytes, buf, len);
        m_cacheBytes += len;
        m_cache[m_cacheBytes++] = ',';
  #if ENABLE_DATA_LOG
        // write data to file
        sdfile.write(buf, len);
        sdfile.write('\n');
  #endif
    }
    uint32_t m_dataTime;
};

class CTeleClient
{
public:
  CTeleClient():connErrors(0),txCount(0),feedid(0) {}
  bool notifyUDP(byte event, const char* serverKey, const char* vin);
  void transmitUDP(bool wait);
  bool reconnect();
  void showStats();
  uint8_t getConnErrors() { return connErrors; }
protected:
  virtual bool udpOpen(const char* host, uint16_t port) { return false; }
  virtual void udpClose() {}
  virtual bool udpSend(const char* data, unsigned int len, bool wait = true) { return false; }
  virtual char* udpReceive(int* pbytes = 0) { return 0; }
  virtual bool waitCompletion(int timeout) { return false; }

  virtual unsigned int getCacheBytes() { return 0; }
  virtual char* getCache() { return 0; }

  virtual void addHeader(int feedid) {}
  virtual void addTail() {}
  virtual void removeTail() {}
  uint16_t feedid;
  uint32_t txCount;
  uint8_t connErrors;
private:
  bool verifyChecksum(const char* data);
  bool waiting;
};

class CTeleClientSIM5360 : public CTeleClient, public virtual CFreematics
{
public:
    CTeleClientSIM5360() {
      m_buffer[0] = 0;
      udpIP[0] = 0;
    }
    bool netInit();
    bool netSetup(const char* apn, bool only3G = false);
    const char* getIP();
    int getSignal();
    char* getOperatorName();
    bool udpOpen(const char* host, uint16_t port);
    void udpClose();
    bool udpSend(const char* data, unsigned int len, bool wait = true);
    char* udpReceive(int* pbytes = 0);
    bool waitCompletion(int timeout);
    char* queryIP(const char* host);
    char* getBuffer() { return m_buffer; }
  private:
    // send command and check for expected response
    virtual bool netSendCommand(const char* cmd, unsigned int timeout = 1000, const char* expected = "OK\r\n", bool terminated = false);
    char m_buffer[128];
    char udpIP[16];
    uint16_t udpPort;
};
