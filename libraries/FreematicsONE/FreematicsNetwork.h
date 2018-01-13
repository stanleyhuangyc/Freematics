/*************************************************************************
* Telematics Data Logger Class
* Distributed under BSD license
* Developed by Stanley Huang https://www.facebook.com/stanleyhuangyc
*************************************************************************/

#include <Arduino.h>

#ifdef ESP32
#include <WiFi.h>
#include <WiFiUdp.h>

#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "bt.h"
#include "bta_api.h"
#include "esp_gatt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_main.h"
#endif

class CTeleClient
{
public:
  virtual bool netBegin() { return true; }
  virtual void netEnd() {}
  virtual bool netOpen(const char* host, uint16_t port) { return true; }
  virtual void netClose() {}
  virtual bool netSend(const char* data, unsigned int len) { return false; }
  virtual char* netReceive(int* pbytes = 0, unsigned int timeout = 5000) { return 0; }
  virtual bool getLocation(NET_LOCATION* loc) { return false; }
  virtual const char* netDeviceName() { return ""; }
  int transmit(const char* data, int bytes, bool wait);
  uint8_t getConnErrors() { return connErrors; }
#ifdef ESP32
  bool bleBegin(const char* bleDeviceName);
  void bleEnd();
  bool bleSend(const char* data, unsigned int len);
  void blePrint(String s);
  virtual size_t onRequest(uint8_t* buffer, size_t len)
  {
    // being requested for data
    buffer[0] = 'O';
    buffer[1] = 'K';
    return 2;
  }
  virtual void onReceive(uint8_t* buffer, size_t len)
  {
    // data received is in buffer
  }
#else
  bool bleBegin(const char* bleDeviceName) { return false; }
  void bleEnd() {}
  bool bleSend(const char* data, unsigned int len) { return false; }
  void blePrint(String s) {}
#endif
protected:
  byte checksum(const char* data, int len)
  {
    // calculate and add checksum
    byte sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return sum;
  }
  virtual String getServerName() { return m_serverName; }
  uint32_t getTotalBytesSent() { return m_bytesCount; }
  uint16_t feedid = 0;
  uint32_t txCount = 0;
  uint8_t connErrors = 0;
  uint32_t m_bytesCount = 0;
  String m_serverName;
};

class CTeleClientSerialUSB : public CTeleClient
{
public:
    bool netSend(const char* data, unsigned int len)
    {
      Serial.write((uint8_t*)data, len);
      Serial.write('\n');
      return true;
    }
    const char* netDeviceName() { return "Serial"; }
};

#ifdef ESP32

class CTeleClientBLE : public CTeleClient
{
public:
    bool netSend(const char* data, unsigned int len)
    {
      return bleSend(data, len);
    }
    char* netReceive(int* pbytes = 0, unsigned int timeout = 1000) { return 0; }
    const char* netDeviceName() { return "BLE"; }
};

class CTeleClientWIFI : public CTeleClient, public virtual CFreematics
{
public:
    void netEnd();
    bool netSetup(const char* ssid, const char* password, unsigned int timeout = 10000);
    String getIP();
    int getSignal() { return 0; }
    bool netOpen(const char* host, uint16_t port);
    bool netSend(const char* data, unsigned int len);
    char* netReceive(int* pbytes = 0, unsigned int timeout = 5000);
    String queryIP(const char* host);
    String serverName() { return m_serverName.length() ? m_serverName : udpIP.toString(); }
    const char* netDeviceName() { return "WIFI"; }
  private:
    char m_buffer[256] = {0};
    IPAddress udpIP;
    uint16_t udpPort;
    WiFiUDP udp;
};

#endif

class CTeleClientSIM800 : public CTeleClient, public virtual CFreematics
{
public:
    bool netBegin();
    void netEnd();
    bool netSetup(const char* apn, unsigned int timeout = 60000);
    String getIP();
    int getSignal();
    String getOperatorName();
    bool netOpen(const char* host, uint16_t port);
    bool netSend(const char* data, unsigned int len);
    void netClose();
    char* netReceive(int* pbytes = 0, unsigned int timeout = 5000);
    bool getLocation(NET_LOCATION* loc);
    String queryIP(const char* host);
    String serverName() { return m_serverName; }
    const char* netDeviceName() { return "SIM800"; }
protected:
    bool netSendCommand(const char* cmd, unsigned int timeout = 1000, const char* expected = "\r\nOK", bool terminated = false);
    char* checkIncoming(int* pbytes);
#ifdef ESP32
    char m_buffer[256] = {0};
#else
    char m_buffer[80] = {0};
#endif
    uint8_t m_stage = 0;
};

class CTeleClientSIM5360 : public CTeleClient, public virtual CFreematics
{
public:
    bool netBegin();
    void netEnd();
    bool netSetup(const char* apn, bool only3G = false, unsigned int timeout = 60000);
    String getIP();
    int getSignal();
    String getOperatorName();
    bool netOpen(const char* host, uint16_t port);
    void netClose();
    bool netSend(const char* data, unsigned int len);
    char* netReceive(int* pbytes = 0, unsigned int timeout = 5000);
    String queryIP(const char* host);
    String serverName() { return m_serverName.length() ? m_serverName : udpIP; }
    const char* netDeviceName() { return "SIM5360"; }
protected:
    // send command and check for expected response
    bool netSendCommand(const char* cmd, unsigned int timeout = 1000, const char* expected = "\r\nOK\r\n", bool terminated = false);
    char* checkIncoming(int* pbytes);
#ifdef ESP32
    char m_buffer[256] = {0};
#else
    char m_buffer[96] = {0};
#endif
    char udpIP[16] = {0};
    uint16_t udpPort = 0;
    uint8_t m_stage = 0;
};
