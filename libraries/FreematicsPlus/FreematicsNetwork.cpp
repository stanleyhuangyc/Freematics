/*************************************************************************
* Freematics Hub Client implementations for various communication devices
* including BLE, WIFI, 2G, 3G
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-one for more information
* (C)2017 Stanley Huang <support@freematics.com.au
*************************************************************************/

#include <Arduino.h>
#include "FreematicsBase.h"
#include "FreematicsNetwork.h"

#define XBEE_BAUDRATE 115200

/*******************************************************************************
  Implementation for ESP32 built-in WIFI (Arduino WIFI library)
*******************************************************************************/

bool CTeleClientWIFI::netSetup(const char* ssid, const char* password, unsigned int timeout)
{
  WiFi.begin(ssid, password);
  for (uint32_t t = millis(); millis() - t < timeout;) {
    if (WiFi.status() == WL_CONNECTED) return true;
    delay(50);
  }
  return false;
}

String CTeleClientWIFI::getIP()
{
  return WiFi.localIP().toString();
}

bool CTeleClientWIFI::netOpen(const char* host, uint16_t port)
{
  if (udp.beginPacket(host, port)) {
    udpIP = udp.remoteIP();
    udpPort = port;
    udp.endPacket();
    return true;
  } else {
    return false;
  }
}

bool CTeleClientWIFI::netSend(const char* data, unsigned int len)
{
  if (udp.beginPacket(udpIP, udpPort)) {
    if (udp.write((uint8_t*)data, len) == len && udp.endPacket()) {
      m_bytesCount += len;
      return true;
    }
  }
  return false;
}

char* CTeleClientWIFI::netReceive(int* pbytes, unsigned int timeout)
{
  uint32_t t = millis();
  do {
    int bytes = udp.parsePacket();
    if (bytes > 0) {
      bytes = udp.read(m_buffer, sizeof(m_buffer));
      if (pbytes) *pbytes = bytes;
      return m_buffer;
    }
  } while (millis() - t < timeout);
  return 0;
}

String CTeleClientWIFI::queryIP(const char* host)
{
  return udpIP.toString();
}

void CTeleClientWIFI::netEnd()
{
  WiFi.disconnect(true);
  // deactivate WIFI
  //esp_wifi_set_mode(WIFI_MODE_NULL);
}

/*******************************************************************************
  Implementation for SIM800 (SIM800 AT command-set)
*******************************************************************************/

bool CTeleClientSIM800::netBegin()
{
  if (m_stage == 0) {
    xbBegin(XBEE_BAUDRATE);
    m_stage = 1;
  }
  for (byte n = 0; n < 10; n++) {
    // try turning on module
    xbTogglePower();
    delay(3000);
    // discard any stale data
    xbPurge();
    for (byte m = 0; m < 3; m++) {
      if (netSendCommand("AT\r")) {
        m_stage = 2;
        return true;
      }
    }
  }
  return false;
}

void CTeleClientSIM800::netEnd()
{
  netSendCommand("AT+CPOWD=1\r");
  m_stage = 1;
}

bool CTeleClientSIM800::netSetup(const char* apn, unsigned int timeout)
{
  uint32_t t = millis();
  bool success = false;
  netSendCommand("ATE0\r");
  do {
    success = netSendCommand("AT+CREG?\r", 3000, "+CREG: 0,1") != 0;
    Serial.print('.');
  } while (!success && millis() - t < timeout);
  if (!success) return false;
  do {
    success = netSendCommand("AT+CGATT?\r", 3000, "+CGATT: 1");
  } while (!success && millis() - t < timeout);
  sprintf(m_buffer, "AT+CSTT=\"%s\"\r", apn);
  if (!netSendCommand(m_buffer)) {
    return false;
  }
  netSendCommand("AT+CIICR\r");
  return success;
}

String CTeleClientSIM800::getIP()
{
  for (uint32_t t = millis(); millis() - t < 60000; ) {
    if (netSendCommand("AT+CIFSR\r", 3000, ".")) {
      char *p;
      for (p = m_buffer; *p && !isdigit(*p); p++);
      char *q = strchr(p, '\r');
      if (q) *q = 0;
      return p;
    }
  }
  return "";
}

int CTeleClientSIM800::getSignal()
{
    if (netSendCommand("AT+CSQ\r", 500)) {
        char *p = strchr(m_buffer, ':');
        if (p) {
          p += 2;
          int db = atoi(p) * 10;
          p = strchr(p, '.');
          if (p) db += *(p + 1) - '0';
          return db;
        }
    }
    return -1;
}
String CTeleClientSIM800::getOperatorName()
{
    // display operator name
    if (netSendCommand("AT+COPS?\r") == 1) {
        char *p = strstr(m_buffer, ",\"");
        if (p) {
            p += 2;
            char *s = strchr(p, '\"');
            if (s) *s = 0;
            return p;
        }
    }
    return "";
}

bool CTeleClientSIM800::netOpen(const char* host, uint16_t port)
{
#if 0
  if (host) {
    if (!isdigit(host[0])) {
      String ip = queryIP(host);
      if (ip.length()) {
        strncpy(udpIP, ip.c_str(), sizeof(udpIP) - 1);
      }
    }
  }
#endif
  //netSendCommand("AT+CLPORT=\"UDP\",8000\r");
  netSendCommand("AT+CIPSRIP=1\r");
  //netSendCommand("AT+CIPUDPMODE=1\r");
  sprintf(m_buffer, "AT+CIPSTART=\"UDP\",\"%s\",\"%u\"\r", host, port);
  return netSendCommand(m_buffer, 3000);
}

void CTeleClientSIM800::netClose()
{
  netSendCommand("AT+CIPCLOSE\r");
}

bool CTeleClientSIM800::netSend(const char* data, unsigned int len)
{
  sprintf(m_buffer, "AT+CIPSEND=%u\r", len);
  if (netSendCommand(m_buffer, 200, ">")) {
    xbWrite(data, len);
    xbWrite("\r", 1);
    if (netSendCommand(0, 5000, "\r\nSEND OK")) {
      return true;
    }
  }
  return false;
}

char* CTeleClientSIM800::netReceive(int* pbytes, unsigned int timeout)
{
	char *data = checkIncoming(pbytes);
	if (data) return data;
  if (netSendCommand(0, timeout, "RECV FROM:")) {
		return checkIncoming(pbytes);
  }
  return 0;
}

char* CTeleClientSIM800::checkIncoming(int* pbytes)
{
	char *p = strstr(m_buffer, "RECV FROM:");
	if (p) p = strchr(p, '\r');
	if (!p) return 0;
	while (*(++p) == '\r' || *p =='\n');
	int len = strlen(p);
	if (len > 2) {
		if (pbytes) *pbytes = len;
		return p;
	} else {
		if (netSendCommand("AT\r", 1000, "\r\nOK", true)) {
			p = m_buffer;
			while (*p && (*p == '\r' || *p == '\n')) p++;
			if (pbytes) *pbytes = strlen(p);
			return p;
		}
	}
	return 0;
}

String CTeleClientSIM800::queryIP(const char* host)
{
  sprintf(m_buffer, "AT+CDNSGIP=\"%s\"\r", host);
  if (netSendCommand(m_buffer, 10000)) {
    char *p = strstr(m_buffer, host);
    if (p) {
      p = strstr(p, ",\"");
      if (p) {
        char *ip = p + 2;
        p = strchr(ip, '\"');
        if (p) *p = 0;
        return ip;
      }
    }
  }
  return "";
}

bool CTeleClientSIM800::netSendCommand(const char* cmd, unsigned int timeout, const char* expected, bool terminated)
{
  if (cmd) {
    xbWrite(cmd);
  }
  m_buffer[0] = 0;
  byte ret = xbReceive(m_buffer, sizeof(m_buffer), timeout, &expected, 1);
  if (ret) {
    if (terminated) {
      char *p = strstr(m_buffer, expected);
      if (p) *p = 0;
    }
    return true;
  } else {
    return false;
  }
}

bool CTeleClientSIM800::getLocation(NET_LOCATION* loc)
{
  if (netSendCommand("AT+CIPGSMLOC=1,1\r", 3000)) do {
    char *p;
    if (!(p = strchr(m_buffer, ':'))) break;
    if (!(p = strchr(p, ','))) break;
    loc->lng = atof(++p);
    if (!(p = strchr(p, ','))) break;
    loc->lat = atof(++p);
    if (!(p = strchr(p, ','))) break;
    loc->year = atoi(++p) - 2000;
    if (!(p = strchr(p, '/'))) break;
    loc->month = atoi(++p);
    if (!(p = strchr(p, '/'))) break;
    loc->day = atoi(++p);
    if (!(p = strchr(p, ','))) break;
    loc->hour = atoi(++p);
    if (!(p = strchr(p, ':'))) break;
    loc->minute = atoi(++p);
    if (!(p = strchr(p, ':'))) break;
    loc->second = atoi(++p);
    return true;
  } while(0);
  return false;
}

/*******************************************************************************
  Implementation for SIM5360
*******************************************************************************/

bool CTeleClientSIM5360::netBegin()
{
  if (m_stage == 0) {
    xbBegin(XBEE_BAUDRATE);
    m_stage = 1;
  }
  for (byte n = 0; n < 10; n++) {
    // try turning on module
    xbTogglePower();
    delay(3000);
    // discard any stale data
    xbPurge();
    for (byte m = 0; m < 5; m++) {
      if (netSendCommand("AT\r")) {
        m_stage = 2;
        return true;
      }
    }
  }
  return false;
}

void CTeleClientSIM5360::netEnd()
{
  if (m_stage == 2 || netSendCommand("AT\r")) {
    xbTogglePower();
    m_stage = 1;
  }
}

bool CTeleClientSIM5360::netSetup(const char* apn, bool only3G, unsigned int timeout)
{
  uint32_t t = millis();
  bool success = false;
  netSendCommand("ATE0\r");
  if (only3G) netSendCommand("AT+CNMP=14\r"); // use WCDMA only
  do {
    do {
      Serial.print('.');
      delay(500);
      success = netSendCommand("AT+CPSI?\r", 1000, "Online");
      if (success) {
        if (!strstr(m_buffer, "NO SERVICE"))
          break;
        success = false;
      } else {
        if (strstr(m_buffer, "Off")) break;
      }
    } while (millis() - t < timeout);
    if (!success) break;

    do {
      success = netSendCommand("AT+CREG?\r", 5000, "+CREG: 0,1");
    } while (!success && millis() - t < timeout);
    if (!success) break;

    do {
      success = netSendCommand("AT+CGREG?\r",1000, "+CGREG: 0,1");
    } while (!success && millis() - t < timeout);
    if (!success) break;

    do {
      sprintf(m_buffer, "AT+CGSOCKCONT=1,\"IP\",\"%s\"\r", apn);
      success = netSendCommand(m_buffer);
    } while (!success && millis() - t < timeout);
    if (!success) break;

    netSendCommand("AT+CSOCKSETPN=1\r");
    netSendCommand("AT+CIPMODE=0\r");
    netSendCommand("AT+NETOPEN\r");
  } while(0);
  if (!success) Serial.println(m_buffer);
  return success;
}

String CTeleClientSIM5360::getIP()
{
  uint32_t t = millis();
  do {
    if (netSendCommand("AT+IPADDR\r", 5000, "\r\nOK\r\n", true)) {
      char *p = strstr(m_buffer, "+IPADDR:");
      if (p) {
        char *ip = p + 9;
        if (*ip != '0') {
					char *q = strchr(ip, '\r');
					if (q) *q = 0;
          return ip;
        }
      }
    }
    delay(500);
  } while (millis() - t < 15000);
  return "";
}

int CTeleClientSIM5360::getSignal()
{
    if (netSendCommand("AT+CSQ\r", 500)) {
        char *p = strchr(m_buffer, ':');
        if (p) {
          p += 2;
          int db = atoi(p) * 10;
          p = strchr(p, '.');
          if (p) db += *(p + 1) - '0';
          return db;
        }
    }
    return -1;
}
String CTeleClientSIM5360::getOperatorName()
{
    // display operator name
    if (netSendCommand("AT+COPS?\r") == 1) {
        char *p = strstr(m_buffer, ",\"");
        if (p) {
            p += 2;
            char *s = strchr(p, '\"');
            if (s) *s = 0;
            return p;
        }
    }
    return "";
}

bool CTeleClientSIM5360::netOpen(const char* host, uint16_t port)
{
  if (host) {
    String ip = queryIP(host);
    if (ip.length()) {
      strncpy(udpIP, ip.c_str(), sizeof(udpIP) - 1);
    } else {
      return false;
    }
    udpPort = port;
  }
  sprintf(m_buffer, "AT+CIPOPEN=0,\"UDP\",\"%s\",%u,8000\r", udpIP, udpPort);
  return netSendCommand(m_buffer, 3000);
}

void CTeleClientSIM5360::netClose()
{
  netSendCommand("AT+CIPCLOSE\r");
}

bool CTeleClientSIM5360::netSend(const char* data, unsigned int len)
{
  sprintf(m_buffer, "AT+CIPSEND=0,%u,\"%s\",%u\r", len, udpIP, udpPort);
  if (netSendCommand(m_buffer, 100, ">")) {
    xbWrite(data, len);
    return netSendCommand(0, 1000);
  }
  return false;
}

char* CTeleClientSIM5360::netReceive(int* pbytes, unsigned int timeout)
{
	char *data = checkIncoming(pbytes);
	if (data) return data;
  if (netSendCommand(0, timeout, "+IPD")) {
		return checkIncoming(pbytes);
  }
  return 0;
}

char* CTeleClientSIM5360::checkIncoming(int* pbytes)
{
	char *p = strstr(m_buffer, "+IPD");
	if (!p) return 0;
	*p = '-'; // mark this datagram as checked
	int len = atoi(p + 4);
	if (pbytes) *pbytes = len;
	p = strchr(p, '\n');
	if (p) {
		*(++p + len) = 0;
		return p;
	}
	return 0;
}

String CTeleClientSIM5360::queryIP(const char* host)
{
  sprintf(m_buffer, "AT+CDNSGIP=\"%s\"\r", host);
  if (netSendCommand(m_buffer, 10000)) {
    char *p = strstr(m_buffer, host);
    if (p) {
      p = strstr(p, ",\"");
      if (p) {
        char *ip = p + 2;
        p = strchr(ip, '\"');
        if (p) *p = 0;
        return ip;
      }
    }
  }
  return "";
}

bool CTeleClientSIM5360::netSendCommand(const char* cmd, unsigned int timeout, const char* expected, bool terminated)
{
  if (cmd) {
    xbWrite(cmd);
  }
  m_buffer[0] = 0;
  byte ret = xbReceive(m_buffer, sizeof(m_buffer), timeout, &expected, 1);
  if (ret) {
    if (terminated) {
      char *p = strstr(m_buffer, expected);
      if (p) *p = 0;
    }
    return true;
  } else {
    return false;
  }
}
