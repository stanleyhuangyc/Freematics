#include "config.h"

#define EVENT_LOGIN 1
#define EVENT_LOGOUT 2
#define EVENT_SYNC 3
#define EVENT_RECONNECT 4
#define EVENT_COMMAND 5
#define EVENT_ACK 6
#define EVENT_PING 7

#define BUFFER_STATE_EMPTY 0
#define BUFFER_STATE_FILLING 1
#define BUFFER_STATE_FILLED 2
#define BUFFER_STATE_LOCKED 3

#define ELEMENT_UINT8 0
#define ELEMENT_UINT16 1
#define ELEMENT_UINT32 2
#define ELEMENT_INT32 3
#define ELEMENT_FLOAT 4

typedef struct {
    uint16_t pid;
    uint8_t type;
    uint8_t count;
} ELEMENT_HEAD;

class CBuffer
{
public:
    CBuffer();
    void add(uint16_t pid, uint8_t type, void* values, int bytes, uint8_t count = 1);
    void purge();
    void serialize(CStorage& store);
    uint32_t timestamp;
    uint16_t offset;
    uint8_t total;
    uint8_t state;
private:
    uint8_t* data;
};

class CBufferManager
{
public:
    void init();
    void purge();
    void free(CBuffer* slot);
    CBuffer* getFree();
    CBuffer* getOldest();
    CBuffer* getNewest();
    void printStats();
private:
    CBuffer* slots[BUFFER_SLOTS];
    CBuffer* last = 0;
};

class TeleClient
{
public:
    virtual void reset()
    {
        txCount = 0;
        txBytes = 0;
        rxBytes = 0;
        login = false;
        startTime = millis();
    }
    virtual bool notify(byte event, const char* payload = 0) { return true; }
    virtual bool connect() { return true; }
    virtual bool transmit(const char* packetBuffer, unsigned int packetSize)  { return true; }
    virtual void inbound() {}
    uint32_t txCount = 0;
    uint32_t txBytes = 0;
    uint32_t rxBytes = 0;
    uint32_t lastSyncTime = 0;
    uint16_t feedid = 0;
    uint32_t startTime = 0;
    uint8_t packets = 0;
    bool login = false;
};

class TeleClientUDP : public TeleClient
{
public:
    bool notify(byte event, const char* payload = 0);
    bool connect(bool quick = false);
    bool transmit(const char* packetBuffer, unsigned int packetSize);
    bool ping();
    void inbound();
    bool verifyChecksum(char* data);
    void shutdown();
#if ENABLE_WIFI
    WifiUDP wifi;
#endif
    CellUDP cell;
};

class TeleClientHTTP : public TeleClient
{
public:
    bool notify(byte event, const char* payload = 0);
    bool connect(bool quick = false);
    bool transmit(const char* packetBuffer, unsigned int packetSize);
    bool ping();
    void shutdown();
#if ENABLE_WIFI
    WifiHTTP wifi;
#endif
    CellHTTP cell;
};