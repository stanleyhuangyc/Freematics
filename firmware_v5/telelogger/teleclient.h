#include "config.h"

#define EVENT_LOGIN 1
#define EVENT_LOGOUT 2
#define EVENT_SYNC 3
#define EVENT_RECONNECT 4
#define EVENT_COMMAND 5
#define EVENT_ACK 6
#define EVENT_PING 7

class TeleClient
{
public:
    virtual void reset()
    {
        txCount = 0;
        txBytes = 0;
        rxBytes = 0;
        login = false;
    }
    virtual bool notify(byte event, const char* payload = 0) { return true; }
    virtual bool connect() { return true; }
    virtual bool transmit(const char* packetBuffer, unsigned int packetSize)  { return true; }
    virtual void inbound() {}
    virtual bool begin() { return true; }
    virtual void end() {}
    uint32_t txCount = 0;
    uint32_t txBytes = 0;
    uint32_t rxBytes = 0;
    uint32_t lastSyncTime = 0;
    uint32_t lastSentTime = 0;
    uint16_t feedid = 0;
    uint32_t startTime = 0;
    bool login = false;
};

class TeleClientUDP : public TeleClient
{
public:
    bool notify(byte event, const char* payload = 0);
    bool connect();
    bool transmit(const char* packetBuffer, unsigned int packetSize);
    bool ping();
    void inbound();
    bool verifyChecksum(char* data);
#if NET_DEVICE == NET_WIFI
    UDPClientWIFI net;
#elif NET_DEVICE == NET_SIM800
    UDPClientSIM800 net;
#elif NET_DEVICE == NET_SIM5360
    UDPClientSIM5360 net;
#elif NET_DEVICE == NET_SIM7600
    UDPClientSIM7600 net;
#else
    NullClient net;
#endif
};

class TeleClientHTTP : public TeleClient
{
public:
    bool connect();
    bool transmit(const char* packetBuffer, unsigned int packetSize);
    bool ping();
#if NET_DEVICE == NET_WIFI
    HTTPClientWIFI net;
#elif NET_DEVICE == NET_SIM800
    HTTPClientSIM800 net;
#elif NET_DEVICE == NET_SIM5360
    HTTPClientSIM5360 net;
#elif NET_DEVICE == NET_SIM7600
    HTTPClientSIM7600 net;
#else
    NullClient net;
#endif
};