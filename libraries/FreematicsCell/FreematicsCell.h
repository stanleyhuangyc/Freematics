/*************************************************************************
* Telematics Data Logger Class
* Distributed under BSD license
* Developed by Stanley Huang https://www.facebook.com/stanleyhuangyc
*************************************************************************/

#include <Arduino.h>

#define PIN_BEE_UART_RXD 16
#define PIN_BEE_UART_TXD 17
#define PIN_BEE_PWR 27
#define BEE_UART_NUM UART_NUM_1
#define UART_BUF_SIZE 256
#define BEE_BAUDRATE 115200
#define HTTP_CONN_TIMEOUT 10000

typedef enum {
  METHOD_GET = 0,
  METHOD_POST,
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

typedef struct {
	uint32_t ts;
	uint32_t date;
	uint32_t time;
	float lat;
	float lng;
	float alt; /* meter */
	float speed; /* knot */
	uint16_t heading; /* degree */
	uint8_t hdop;
	uint8_t sat;
	uint16_t sentences;
	uint16_t errors;
} GPS_DATA;

class FreematicsBee
{
public:
  // start xBee UART communication
  bool xbBegin(unsigned long baudrate = BEE_BAUDRATE, int pinRx = PIN_BEE_UART_RXD, int pinTx = PIN_BEE_UART_TXD);
  void xbEnd();
  // read data to xBee UART
  int xbRead(char* buffer, int bufsize, unsigned int timeout = 1000);
  // send data to xBee UART
  void xbWrite(const char* cmd);
    // send data to xBee UART
  void xbWrite(const char* data, int len);
  // receive data from xBee UART (returns 0/1/2)
  int xbReceive(char* buffer, int bufsize, unsigned int timeout = 1000, const char** expected = 0, byte expectedCount = 0);
  // purge xBee UART buffer
  void xbPurge();
  // toggle xBee module power
  void xbTogglePower(int pinPowerKey = PIN_BEE_PWR);
};

class HTTPClient
{
public:
    HTTP_STATES state() { return m_state; }
protected:
    String genHeader(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload, int payloadSize);
    HTTP_STATES m_state = HTTP_DISCONNECTED;
    String m_host;
};

class ClientSIM800 : public FreematicsBee
{
public:
    bool begin();
    void end();
    bool setup(const char* apn, bool gps = false, unsigned int timeout = 60000);
    String getIP();
    int getSignal();
    String getOperatorName();
    bool checkSIM();
    bool getLocation(NET_LOCATION* loc);
    String queryIP(const char* host);
    char* getBuffer() { return m_buffer; }
    const char* deviceName() { return "SIM800"; }
    const char* IMEI = "N/A";
protected:
    bool sendCommand(const char* cmd, unsigned int timeout = 1000, const char* expected = "\r\nOK");
    char m_buffer[256] = {0};
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
    bool send(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload = 0, int payloadSize = 0);
    char* receive(int* pbytes = 0, unsigned int timeout = HTTP_CONN_TIMEOUT);
    void close();
protected:
    String m_host;
    uint16_t m_port;
};

class ClientSIM5360 : public FreematicsBee
{
public:
    virtual bool begin();
    virtual void end();
    virtual bool setup(const char* apn, unsigned int timeout = 30000);
    virtual bool setGPS(bool on);
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
    char* getBuffer() { return m_buffer; }
    const char* deviceName() { return m_model; }
    char IMEI[16] = {0};
protected:
    // send command and check for expected response
    bool sendCommand(const char* cmd, unsigned int timeout = 1000, const char* expected = "\r\nOK\r\n");
    void checkGPS();
    float parseDegree(const char* s);
    char m_buffer[384] = {0};
    char m_model[12] = {0};
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
    bool open(const char* host = 0, uint16_t port = 0);
    void close();
    bool send(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload = 0, int payloadSize = 0);
    char* receive(int* pbytes = 0, unsigned int timeout = HTTP_CONN_TIMEOUT);
};

class ClientSIM7600 : public ClientSIM5360
{
public:
    bool setup(const char* apn, unsigned int timeout = 30000);
    void end();
    bool setGPS(bool on);
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
    bool open(const char* host = 0, uint16_t port = 0);
    void close();
    bool send(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload = 0, int payloadSize = 0);
    char* receive(int* pbytes = 0, unsigned int timeout = HTTP_CONN_TIMEOUT);
};