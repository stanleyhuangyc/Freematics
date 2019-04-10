/*************************************************************************
* Telematics Data Logger Class
* Distributed under BSD license
* Developed by Stanley Huang https://www.facebook.com/stanleyhuangyc
*************************************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "FreematicsBase.h"

#define HTTP_CONN_TIMEOUT 5000

typedef enum {
  HTTP_GET = 0,
  HTTP_POST,
} HTTP_METHOD;

typedef enum {
    HTTP_DISCONNECTED = 0,
    HTTP_CONNECTED,
    HTTP_SENT,
    HTTP_ERROR,
} HTTP_STATES;

typedef struct {
    float lat;
    float lng;
    uint8_t year; /* year past 2000, e.g. 15 for 2015 */
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} NET_LOCATION;

class HTTPClient
{
public:
    HTTP_STATES state() { return m_state; }
protected:
    unsigned int genHeader(char* header, HTTP_METHOD method, const char* path, bool keepAlive, const char* payload, int payloadSize);
    HTTP_STATES m_state = HTTP_DISCONNECTED;
};

class ClientWIFI
{
public:
    bool begin(const char* ssid, const char* password);
    void end();
    bool setup(unsigned int timeout = 15000);
    String getIP();
    int getSignal() { return 0; }
    const char* deviceName() { return "WiFi"; }
protected:
    void listAPs();
    char m_buffer[256] = {0};
};

class UDPClientWIFI : public ClientWIFI
{
public:
    bool open(const char* host, uint16_t port);
    void close();
    bool send(const char* data, unsigned int len);
    char* receive(int* pbytes = 0, unsigned int timeout = 5000);
    String queryIP(const char* host);
private:
    IPAddress udpIP;
    uint16_t udpPort;
    WiFiUDP udp;
};

class HTTPClientWIFI : public HTTPClient, public ClientWIFI
{
public:
    bool open(const char* host, uint16_t port);
    void close();
    int send(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload = 0, int payloadSize = 0);
    char* receive(int* pbytes = 0, unsigned int timeout = HTTP_CONN_TIMEOUT);
    int code = 0;
private:
    WiFiClient client;
};

class ClientSIM800
{
public:
    bool begin(CFreematics* device);
    void end();
    bool setup(const char* apn, bool gps = false, unsigned int timeout = 60000);
    String getIP();
    int getSignal();
    String getOperatorName();
    bool checkSIM();
    bool getLocation(NET_LOCATION* loc);
    String queryIP(const char* host);
    const char* deviceName() { return "SIM800"; }
protected:
    bool sendCommand(const char* cmd, unsigned int timeout = 1000, const char* expected = "\r\nOK");
    char m_buffer[256] = {0};
    uint8_t m_stage = 0;
    CFreematics* m_device = 0;
};

class UDPClientSIM800 : public ClientSIM800
{
public:
    bool open(const char* host, uint16_t port);
    bool send(const char* data, unsigned int len);
    void close();
    char* receive(int* pbytes = 0, unsigned int timeout = 5000);
private:
    char* checkIncoming(int* pbytes);
};

class HTTPClientSIM800 : public HTTPClient, public ClientSIM800
{
public:
    bool open(const char* host, uint16_t port);
    int send(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload = 0, int payloadSize = 0);
    char* receive(int* pbytes = 0, unsigned int timeout = HTTP_CONN_TIMEOUT);
    void close();
protected:
    String m_host;
    uint16_t m_port;
};

class ClientSIM5360
{
public:
    virtual bool begin(CFreematics* device);
    virtual void end();
    virtual bool setup(const char* apn, bool gps = false, bool roaming = false, unsigned int timeout = 30000);
    String getIP();
    int getSignal();
    String getOperatorName();
    bool checkSIM();
    String queryIP(const char* host);
    bool getLocation(GPS_DATA** pgd)
    {
        if (m_gps) {
            if (pgd) *pgd = m_gps;
            return m_gps->ts != 0;
        } else {
            return false;
        }
    }
    const char* deviceName() { return m_model; }
    char IMEI[16] = {0};
protected:
    // send command and check for expected response
    bool sendCommand(const char* cmd, unsigned int timeout = 1000, const char* expected = "\r\nOK\r\n");
    void checkGPS();
    float parseDegree(const char* s);
    char m_buffer[384] = {0};
    uint8_t m_stage = 0;
    char m_model[12] = {0};
    CFreematics* m_device = 0;
    GPS_DATA* m_gps = 0;
};

class UDPClientSIM5360 : public ClientSIM5360
{
public:
    bool open(const char* host, uint16_t port);
    void close();
    bool send(const char* data, unsigned int len);
    char* receive(int* pbytes = 0, unsigned int timeout = 5000);
protected:
    char* checkIncoming(int* pbytes);
    String udpIP;
    uint16_t udpPort = 0;
};

class HTTPClientSIM5360 : public HTTPClient, public ClientSIM5360
{
public:
    bool open(const char* host, uint16_t port);
    void close();
    int send(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload = 0, int payloadSize = 0);
    char* receive(int* pbytes = 0, unsigned int timeout = HTTP_CONN_TIMEOUT);
};

class ClientSIM7600 : public ClientSIM5360
{
public:
    bool setup(const char* apn, bool gps = false, bool roaming = false, unsigned int timeout = 30000);
};

class UDPClientSIM7600 : public ClientSIM7600
{
public:
    bool open(const char* host, uint16_t port);
    void close();
    bool send(const char* data, unsigned int len);
    char* receive(int* pbytes = 0, unsigned int timeout = 5000);
protected:
    char* checkIncoming(int* pbytes);
    String udpIP;
    uint16_t udpPort = 0;
};

class HTTPClientSIM7600 : public HTTPClient, public ClientSIM7600
{
public:
    bool open(const char* host, uint16_t port);
    void close();
    int send(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload = 0, int payloadSize = 0);
    char* receive(int* pbytes = 0, unsigned int timeout = HTTP_CONN_TIMEOUT);
};