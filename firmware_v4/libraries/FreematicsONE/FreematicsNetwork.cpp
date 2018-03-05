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
  Implementation for ESP8266 WIFI (ESP8266 AT command-set)
*******************************************************************************/
bool UDPClientESP8266AT::begin(CFreematics* device)
{
  m_device = device;
  m_device->xbBegin(XBEE_BAUDRATE);
  return true;
}

void UDPClientESP8266AT::end()
{
  sendCommand("AT+CWQAP\r\n");
}

bool UDPClientESP8266AT::setup(const char* ssid, const char* password, unsigned int timeout)
{
  bool gotIP = false;
  // test the module by issuing ATE0 command and confirming response of "OK"
  const char* cmds[] = {"ATE0\r\n", "AT+CWMODE=1\r\n", "AT+CIPMUX=0\r\n", "AT+CWDHCP=1,1\r\n"};
  for (byte n = 0; n < sizeof(cmds) / sizeof(cmds[0]); n++) {
    delay(200);
    if (!sendCommand(cmds[n], 100)) {
      Serial.println(cmds[n]);
      return false;
    }
    if (rxLen) {
      if (strstr_P(rxBuf, PSTR("WIFI GOT IP"))) {
        // WIFI automatically connected
        gotIP = true;
      }
    }
  }
  // generate and send AT command for joining AP
  if (!gotIP) {
    sprintf_P(buffer, PSTR("AT+CWJAP=\"%s\",\"%s\"\r\n"), ssid, password);
    if (!sendCommand(buffer, 7000)) return false;
  }
  return true;
}

String UDPClientESP8266AT::getIP()
{
  // get IP address
  if (sendCommand("AT+CIFSR\r\n") && !strstr_P(buffer, PSTR("0.0.0.0"))) {
    char *p = strchr(buffer, '\"');
    char *ip = p ? p + 1 : buffer;
    if ((p = strchr(ip, '\"')) || (p = strchr(ip, '\r'))) *p = 0;
    // output IP address
    return ip;
  }
  return "";
}

String UDPClientESP8266AT::queryIP(const char* host)
{
  sprintf_P(buffer, PSTR("AT+CIPDOMAIN=\"%s\"\r\n"), host);
  if (sendCommand(buffer, 5000, "+CIPDOMAIN")) {
    char *p;
    if ((p = strstr_P(buffer, PSTR("+CIPDOMAIN"))) && (p = strchr(p, ':'))) {
      char *ip = p + 1;
      p = strchr(ip, '\r');
      if (p) *p = 0;
      return ip;
    }
  }
  return "";
}

bool UDPClientESP8266AT::open(const char* host, uint16_t port)
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
  for (byte n = 0; n < 3; n++) {
    sprintf_P(buffer, PSTR("AT+CIPSTART=\"UDP\",\"%s\",%d,8000,0\r\n"), udpIP, udpPort);
    if (sendCommand(buffer, 3000)) {
      return true;
    } else {
      // check if already connected
      if (strstr_P(buffer, PSTR("CONN"))) return true;
    }
    Serial.println(buffer);
  }
  return false;
}

void UDPClientESP8266AT::close()
{
  sendCommand("AT+CIPCLOSE\r\n", 500);
}

bool UDPClientESP8266AT::send(const char* data, unsigned int len)
{
  sprintf_P(buffer, PSTR("AT+CIPSEND=%u\r\n"), len);
  if (sendCommand(buffer, 100, ">") && sendCommand(data, 1000, "SEND OK")) {
    return true;
  } else {
    Serial.println(buffer);
    return false;
  }
}

char* UDPClientESP8266AT::receive(int* pbytes, unsigned int timeout)
{
  if (!rxLen && !sendCommand(0, timeout, "+IPD,")) {
      return 0;
  }
  if (rxLen) {
    if (pbytes) *pbytes = rxLen;
    rxLen = 0;
    return rxBuf;
  }
  return 0;
}

bool UDPClientESP8266AT::sendCommand(const char* cmd, unsigned int timeout, const char* expected)
{
  if (cmd) {
    m_device->xbWrite(cmd);
  }
  buffer[0] = 0;
  byte ret = m_device->xbReceive(buffer, sizeof(buffer), timeout, &expected, 1);
  // reception
  char *p = strstr_P(buffer, PSTR("+IPD,"));
  if (p) {
    p += 5;
    rxLen = atoi(p);
    char *q = strchr(p, ':');
    if (q++) {
      int maxLen = buffer + sizeof(buffer) - q;
      if (rxLen > maxLen) rxLen = maxLen;
      if (rxBuf) free(rxBuf);
      rxBuf = (char*)malloc(rxLen + 1);
      memcpy(rxBuf, q, rxLen);
      rxBuf[rxLen] = 0;
    } else {
      rxLen = 0;
    }
  }
  return ret != 0;
}

/*******************************************************************************
  Implementation for SIM800 (SIM800 AT command-set)
*******************************************************************************/

bool UDPClientSIM800::begin(CFreematics* device)
{
  m_device = device;
  if (m_stage == 0) {
    device->xbBegin(XBEE_BAUDRATE);
    m_stage = 1;
  }
  for (byte n = 0; n < 10; n++) {
    // try turning on module
    device->xbTogglePower();
    delay(2000);
    // discard any stale data
    device->xbPurge();
    for (byte m = 0; m < 3; m++) {
      if (sendCommand("AT\r")) {
        m_stage = 2;
        return true;
      }
    }
  }
  return false;
}

void UDPClientSIM800::end()
{
  sendCommand("AT+CPOWD=1\r");
  m_stage = 1;
}

bool UDPClientSIM800::setup(const char* apn, unsigned int timeout)
{
  uint32_t t = millis();
  bool success = false;
  sendCommand("ATE0\r");
  do {
    success = sendCommand("AT+CREG?\r", 3000, "+CREG: 0,1") != 0;
    Serial.print('.');
  } while (!success && millis() - t < timeout);
  if (!success) return false;
  do {
    success = sendCommand("AT+CGATT?\r", 3000, "+CGATT: 1");
  } while (!success && millis() - t < timeout);
  sprintf(m_buffer, "AT+CSTT=\"%s\"\r", apn);
  if (!sendCommand(m_buffer)) {
    return false;
  }
  sendCommand("AT+CIICR\r");
  return success;
}

String UDPClientSIM800::getIP()
{
  for (uint32_t t = millis(); millis() - t < 60000; ) {
    if (sendCommand("AT+CIFSR\r", 3000, ".")) {
      char *p;
      for (p = m_buffer; *p && !isdigit(*p); p++);
      char *q = strchr(p, '\r');
      if (q) *q = 0;
      return p;
    }
  }
  return "";
}

int UDPClientSIM800::getSignal()
{
    if (sendCommand("AT+CSQ\r", 500)) {
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
String UDPClientSIM800::getOperatorName()
{
    // display operator name
    if (sendCommand("AT+COPS?\r") == 1) {
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

bool UDPClientSIM800::open(const char* host, uint16_t port)
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
  //sendCommand("AT+CLPORT=\"UDP\",8000\r");
  sendCommand("AT+CIPSRIP=1\r");
  //sendCommand("AT+CIPUDPMODE=1\r");
  sprintf(m_buffer, "AT+CIPSTART=\"UDP\",\"%s\",\"%u\"\r", host, port);
  return sendCommand(m_buffer, 3000);
}

void UDPClientSIM800::close()
{
  sendCommand("AT+CIPCLOSE\r");
}

bool UDPClientSIM800::send(const char* data, unsigned int len)
{
  sprintf(m_buffer, "AT+CIPSEND=%u\r", len);
  if (sendCommand(m_buffer, 200, ">")) {
    m_device->xbWrite(data, len);
    m_device->xbWrite("\r", 1);
    if (sendCommand(0, 5000, "\r\nSEND OK")) {
      return true;
    }
  }
  return false;
}

char* UDPClientSIM800::receive(int* pbytes, unsigned int timeout)
{
	char *data = checkIncoming(pbytes);
	if (data) return data;
  if (sendCommand(0, timeout, "RECV FROM:")) {
		return checkIncoming(pbytes);
  }
  return 0;
}

char* UDPClientSIM800::checkIncoming(int* pbytes)
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
		if (sendCommand("AT\r", 1000, "\r\nOK", true)) {
			p = m_buffer;
			while (*p && (*p == '\r' || *p == '\n')) p++;
			if (pbytes) *pbytes = strlen(p);
			return p;
		}
	}
	return 0;
}

String UDPClientSIM800::queryIP(const char* host)
{
  sprintf(m_buffer, "AT+CDNSGIP=\"%s\"\r", host);
  if (sendCommand(m_buffer, 10000)) {
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

bool UDPClientSIM800::sendCommand(const char* cmd, unsigned int timeout, const char* expected, bool terminated)
{
  if (cmd) {
    m_device->xbWrite(cmd);
  }
  m_buffer[0] = 0;
  byte ret = m_device->xbReceive(m_buffer, sizeof(m_buffer), timeout, &expected, 1);
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

bool UDPClientSIM800::getLocation(NET_LOCATION* loc)
{
  if (sendCommand("AT+CIPGSMLOC=1,1\r", 3000)) do {
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

bool UDPClientSIM5360::begin(CFreematics* device)
{
  m_device = device;
  if (m_stage == 0) {
    device->xbBegin(XBEE_BAUDRATE);
    m_stage = 1;
  }
  for (byte n = 0; n < 10; n++) {
    // try turning on module
    device->xbTogglePower();
    delay(2000);
    // discard any stale data
    device->xbPurge();
    for (byte m = 0; m < 5; m++) {
      if (sendCommand("AT\r")) {
        m_stage = 2;
        return true;
      }
    }
  }
  return false;
}

void UDPClientSIM5360::end()
{
  if (m_stage == 2 || sendCommand("AT\r")) {
    m_device->xbTogglePower();
    m_stage = 1;
  }
}

bool UDPClientSIM5360::setup(const char* apn, bool only3G, unsigned int timeout)
{
  uint32_t t = millis();
  bool success = false;
  sendCommand("ATE0\r");
  if (only3G) sendCommand("AT+CNMP=14\r"); // use WCDMA only
  do {
    do {
      Serial.print('.');
      delay(500);
      success = sendCommand("AT+CPSI?\r", 1000, "Online");
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
      success = sendCommand("AT+CREG?\r", 5000, "+CREG: 0,1");
    } while (!success && millis() - t < timeout);
    if (!success) break;

    do {
      success = sendCommand("AT+CGREG?\r",1000, "+CGREG: 0,1");
    } while (!success && millis() - t < timeout);
    if (!success) break;

    do {
      sprintf(m_buffer, "AT+CGSOCKCONT=1,\"IP\",\"%s\"\r", apn);
      success = sendCommand(m_buffer);
    } while (!success && millis() - t < timeout);
    if (!success) break;

    sendCommand("AT+CSOCKSETPN=1\r");
    //sendCommand("AT+CSOCKAUTH=,,\"password\",\"password\"\r");
    sendCommand("AT+CIPMODE=0\r");
    sendCommand("AT+NETOPEN\r");
  } while(0);
  if (!success) Serial.println(m_buffer);
  return success;
}

String UDPClientSIM5360::getIP()
{
  uint32_t t = millis();
  do {
    if (sendCommand("AT+IPADDR\r", 3000, "\r\nOK\r\n", true)) {
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

int UDPClientSIM5360::getSignal()
{
    if (sendCommand("AT+CSQ\r", 500)) {
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
String UDPClientSIM5360::getOperatorName()
{
    // display operator name
    if (sendCommand("AT+COPS?\r") == 1) {
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

bool UDPClientSIM5360::open(const char* host, uint16_t port)
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
  return sendCommand(m_buffer, 3000);
}

void UDPClientSIM5360::close()
{
  sendCommand("AT+CIPCLOSE\r");
}

bool UDPClientSIM5360::send(const char* data, unsigned int len)
{
  sprintf(m_buffer, "AT+CIPSEND=0,%u,\"%s\",%u\r", len, udpIP, udpPort);
  if (sendCommand(m_buffer, 100, ">")) {
    m_device->xbWrite(data, len);
    return sendCommand(0, 1000);
  }
  return false;
}

char* UDPClientSIM5360::receive(int* pbytes, unsigned int timeout)
{
	char *data = checkIncoming(pbytes);
	if (data) return data;
  if (sendCommand(0, timeout, "+IPD")) {
		return checkIncoming(pbytes);
  }
  return 0;
}

char* UDPClientSIM5360::checkIncoming(int* pbytes)
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

String UDPClientSIM5360::queryIP(const char* host)
{
  sprintf(m_buffer, "AT+CDNSGIP=\"%s\"\r", host);
  if (sendCommand(m_buffer, 10000)) {
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

bool UDPClientSIM5360::sendCommand(const char* cmd, unsigned int timeout, const char* expected, bool terminated)
{
  if (cmd) {
    m_device->xbWrite(cmd);
  }
  m_buffer[0] = 0;
  byte ret = m_device->xbReceive(m_buffer, sizeof(m_buffer), timeout, &expected, 1);
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
