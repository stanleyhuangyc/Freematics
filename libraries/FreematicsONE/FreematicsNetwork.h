/*************************************************************************
* Telematics Data Logger Class
* Distributed under BSD license
* Developed by Stanley Huang https://www.facebook.com/stanleyhuangyc
*************************************************************************/

#ifdef ESP32
#include <WiFi.h>
#include <WiFiUdp.h>

#include "esp_system.h"
#include "esp_log.h"
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

#define XBEE_BAUDRATE 115200

class CTeleClient
{
public:
  CTeleClient():connErrors(0),txCount(0),m_bytesCount(0),feedid(0),m_waiting(false) {}
  virtual bool netBegin() { return true; }
  virtual void netEnd() {}
  virtual bool netOpen(const char* host, uint16_t port) { return true; }
  virtual void netClose() {}
  virtual int netSend(const char* data, unsigned int len, bool wait = true) { return false; }
  virtual char* netReceive(int* pbytes = 0, int timeout = 5000) { return 0; }
  virtual String netDeviceName() { return ""; }
  bool notifyServer(byte event, const char* serverKey = 0, const char* extra = 0);
  int transmit(const char* data, int bytes, bool wait);
  uint8_t getConnErrors() { return connErrors; }
#ifdef ESP32
  bool bleBegin(const char* bleDeviceName);
  void bleEnd();
  bool bleSend(const char* data, unsigned int len);
  bool bleSend(String s);
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
  bool bleSend(String s) { return false; }
#endif
protected:
  byte getChecksum(const char* data, int len)
  {
    // calculate and add checksum
    byte sum = 0;
    for (unsigned int i = 0; i < len; i++) sum += data[i];
    return sum;
  }
  virtual bool netWaitSent(int timeout) { return true; }
  virtual String getServerName() { return m_serverName; }
  uint32_t getTotalBytesSent() { return m_bytesCount; }
  uint16_t feedid;
  uint32_t txCount;
  uint8_t connErrors;
  uint32_t m_bytesCount;
  String m_serverName;
private:
  bool verifyChecksum(const char* data);
  bool m_waiting;
};

class CTeleClientSerialUSB : public CTeleClient
{
public:
    int netSend(const char* data, unsigned int len, bool wait = true)
    {
      Serial.write((uint8_t*)data, len);
      Serial.write('\n');
    }
    String netDeviceName() { return "Serial"; }
};

#ifdef ESP32

class CTeleClientBLE : public CTeleClient
{
public:
    int netSend(const char* data, unsigned int len, bool wait = true)
    {
      return bleSend(data, len) ? len : 0;
    }
    char* netReceive(int* pbytes = 0, int timeout = 1000) { return 0; }
    String netDeviceName() { return "BLE"; }
};

class CTeleClientWIFI : public CTeleClient, public virtual CFreematics
{
public:
    CTeleClientWIFI() {
      m_buffer[0] = 0;
      udpIP[0] = 0;
    }
    void netEnd();
    bool netSetup(const char* ssid, const char* password, int timeout = 10000);
    String getIP();
    int getSignal() { return 0; }
    bool netOpen(const char* host, uint16_t port);
    int netSend(const char* data, unsigned int len, bool wait = true);
    char* netReceive(int* pbytes = 0, int timeout = 5000);
    String queryIP(const char* host);
    String serverName() { return m_serverName.length() ? m_serverName : udpIP.toString(); }
    String netDeviceName() { return "WIFI"; }
  private:
    char m_buffer[256];
    IPAddress udpIP;
    uint16_t udpPort;
    WiFiUDP udp;
};

#endif

class CTeleClientSIM800 : public CTeleClient, public virtual CFreematics
{
public:
    CTeleClientSIM800() {
      m_buffer[0] = 0;
      udpIP[0] = 0;
      m_stage = 0;
    }
    bool netBegin();
    void netEnd();
    bool netSetup(const char* apn, int timeout = 60000);
    String getIP();
    int getSignal();
    String getOperatorName();
    bool netOpen(const char* host, uint16_t port);
    int netSend(const char* data, unsigned int len, bool wait = true);
    void netClose();
    char* netReceive(int* pbytes = 0, int timeout = 5000);
    String queryIP(const char* host);
    String serverName() { return m_serverName.length() ? m_serverName : udpIP; }
    String netDeviceName() { return "SIM800"; }
  private:
    bool netWaitSent(int timeout);
    bool netSendCommand(const char* cmd, unsigned int timeout = 1000, const char* expected = "OK\r\n", bool terminated = false);
    char m_buffer[256];
    char udpIP[16];
    uint16_t udpPort;
    uint8_t m_stage;
};

class CTeleClientSIM5360 : public CTeleClient, public virtual CFreematics
{
public:
    CTeleClientSIM5360() {
      m_buffer[0] = 0;
      udpIP[0] = 0;
      m_stage = 0;
    }
    bool netBegin();
    void netEnd();
    bool netSetup(const char* apn, bool only3G = false, int timeout = 60000);
    String getIP();
    int getSignal();
    String getOperatorName();
    bool netOpen(const char* host, uint16_t port);
    void netClose();
    int netSend(const char* data, unsigned int len, bool wait = true);
    char* netReceive(int* pbytes = 0, int timeout = 5000);
    String queryIP(const char* host);
    String serverName() { return m_serverName.length() ? m_serverName : udpIP; }
    String netDeviceName() { return "SIM5360"; }
  private:
    bool netWaitSent(int timeout);
    // send command and check for expected response
    bool netSendCommand(const char* cmd, unsigned int timeout = 1000, const char* expected = "\r\nOK\r\n", bool terminated = false);
    char m_buffer[256];
    char udpIP[16];
    uint16_t udpPort;
    uint8_t m_stage;
};
