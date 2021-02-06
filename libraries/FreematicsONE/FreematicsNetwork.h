/*************************************************************************
* Freematics Hub Client implementations for ESP8266-AT, SIM800, SIM5360
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one for more information
* (C)2017-2019 Stanley Huang <stanley@freematics.com.au
*************************************************************************/

#pragma once

#include <Arduino.h>
#include "FreematicsBase.h"

class NullClient
{
public:
    virtual GPS_DATA* getLocation() { return m_gps; }
    virtual bool startGPS() { return false; }
    virtual void stopGPS() {}
protected:
    GPS_DATA* m_gps = 0;
};

class UDPClientESP8266AT : public NullClient
{
public:
    bool begin(CFreematics* device, bool nocheck = false);
    void end();
    bool setup(const char* ssid, const char* password, unsigned int timeout = 15000);
    String getIP();
    float getSignal() { return 0; }
    bool open(const char* host, uint16_t port);
    void close();
    bool send(const char* data, unsigned int len);
    char* receive(int* pbytes = 0, unsigned int timeout = 5000);
    char* getBuffer() { return buffer; }
private:
    bool sendCommand(const char* cmd, unsigned int timeout = 2000, const char* expected = "OK");
    char* rxBuf = 0;
    int rxLen = 0;
    CFreematics* m_device = 0;
    char buffer[128];
};

class UDPClientSIM800 : public NullClient
{
public:
    bool begin(CFreematics* device, bool nocheck = false);
    void end();
    bool setup(const char* apn, unsigned int timeout = 30000, bool gps = false, const char* pin = 0);
    String getIP();
    int getSignal();
    String getOperatorName();
    bool checkSIM();
    bool open(const char* host, uint16_t port);
    bool send(const char* data, unsigned int len);
    void close();
    char* receive(int* pbytes = 0, unsigned int timeout = 5000);
    String queryIP(const char* host);
    GPS_DATA* getLocation();
    char* getBuffer() { return m_buffer; }
private:
    bool sendCommand(const char* cmd, unsigned int timeout = 1000, const char* expected = "\r\nOK", bool terminated = false);
    char* checkIncoming(int* pbytes);
    char m_buffer[80] = {0};
    uint8_t m_stage = 0;
    CFreematics* m_device = 0;
};

class UDPClientSIM5360 : public NullClient
{
public:
    bool begin(CFreematics* device, bool nocheck = false);
    void end();
    bool setup(const char* apn, unsigned int timeout = 30000, const char* pin = 0);
    String getIP();
    int getSignal();
    String getOperatorName();
    bool checkSIM();
    bool startGPS();
    void stopGPS();
    bool open(const char* host, uint16_t port);
    void close();
    bool send(const char* data, unsigned int len);
    char* receive(int* pbytes = 0, unsigned int timeout = 5000);
    char* getBuffer() { return m_buffer; }
private:
    // send command and check for expected response 
    bool sendCommand(const char* cmd, unsigned int timeout = 1000, const char* expected = "\r\nOK");
    char* checkIncoming(char* ipd, int* pbytes);
    long parseDegree(const char* s);
    void checkGPS();
    char m_buffer[160] = {0};
    byte udpIP[4] = {0};
    uint16_t udpPort = 0;
    uint8_t m_stage = 0;
    CFreematics* m_device = 0;
};
