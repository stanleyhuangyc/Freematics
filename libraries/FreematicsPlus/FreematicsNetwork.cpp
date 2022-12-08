/*************************************************************************
* Helper classes for various network communication devices
* Distributed under BSD license
* Visit https://freematics.com for more inform
on
* (C)2012-2021 Developed by Stanley Huang <stanley@freematics.com.au>
*************************************************************************/

#include <Arduino.h>
#include "FreematicsBase.h"
#include "FreematicsNetwork.h"

String HTTPClient::genHeader(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload, int payloadSize)
{
  String header;
  // generate a simplest HTTP header
  header = method == METHOD_GET ? "GET " : "POST ";
  header += path;
  header += " HTTP/1.1\r\nConnection: ";
  header += keepAlive ? "keep-alive" : "close";
  header += "\r\nHost: ";
  header += m_host;
  if (method != METHOD_GET) {
    header += "\r\nContent-length: ";
    header += String(payloadSize);
  }
  header += "\r\n\r\n";
  return header;
}

/*******************************************************************************
  Implementation for WiFi (on top of Arduino WiFi library)
*******************************************************************************/

bool ClientWIFI::setup(unsigned int timeout)
{
  for (uint32_t t = millis(); millis() - t < timeout;) {
    if (WiFi.status() == WL_CONNECTED) {
      return true;
    }
    delay(50);
  }
  return false;
}

String ClientWIFI::getIP()
{
  return WiFi.localIP().toString();
}

bool ClientWIFI::begin(const char* ssid, const char* password)
{
  //listAPs();
  WiFi.begin(ssid, password);
  return true;
}

void ClientWIFI::end()
{
  WiFi.disconnect(true);
}

void ClientWIFI::listAPs()
{
  int n = WiFi.scanNetworks();
  if (n <= 0) {
      Serial.println("No WiFi AP found");
  } else {
      Serial.println("Nearby WiFi APs:");
      for (int i = 0; i < n; ++i) {
          // Print SSID and RSSI for each network found
          Serial.print(i + 1);
          Serial.print(": ");
          Serial.print(WiFi.SSID(i));
          Serial.print(" (");
          Serial.print(WiFi.RSSI(i));
          Serial.println("dB)");
      }
  }
}

bool WifiUDP::open(const char* host, uint16_t port)
{
  if (udp.beginPacket(host, port)) {
    udpIP = udp.remoteIP();
    udpPort = port;
    if (udp.endPacket()) {
      return true;
    }
  }
  return false;
}

bool WifiUDP::send(const char* data, unsigned int len)
{
  if (udp.beginPacket(udpIP, udpPort)) {
    if (udp.write((uint8_t*)data, len) == len && udp.endPacket()) {
      return true;
    }
  }
  return false;
}

int WifiUDP::receive(char* buffer, int bufsize, unsigned int timeout)
{
  uint32_t t = millis();
  do {
    int bytes = udp.parsePacket();
    if (bytes > 0) {
      bytes = udp.read(buffer, bufsize);
      return bytes;
    }
    delay(1);
  } while (millis() - t < timeout);
  return 0;
}

String WifiUDP::queryIP(const char* host)
{
  return udpIP.toString();
}

void WifiUDP::close()
{
  udp.stop();
}

bool WifiHTTP::open(const char* host, uint16_t port)
{
  if (!host) return true;
  if (client.connect(host, port)) {
    m_state = HTTP_CONNECTED;
    m_host = host;
    return true;
  } else {
    m_state = HTTP_ERROR;
    return false;
  }
}

void WifiHTTP::close()
{
  client.stop();
  m_state = HTTP_DISCONNECTED;
}

bool WifiHTTP::send(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload, int payloadSize)
{
  String header = genHeader(method, path, keepAlive, payload, payloadSize);
  int len = header.length();
  if (client.write(header.c_str(), len) != len) {
    m_state = HTTP_DISCONNECTED;
    return false;
  }
  if (payloadSize) {
    if (client.write(payload, payloadSize) != payloadSize) {
      m_state = HTTP_ERROR;
      return false;
    }
  }
  m_state = HTTP_SENT;
  return true;
}

char* WifiHTTP::receive(char* buffer, int bufsize, int* pbytes, unsigned int timeout)
{
  int bytes = 0;
  int contentBytes = 0;
  int contentLen = 0;
  char* content = 0;
  bool keepAlive = true;

  for (uint32_t t = millis(); millis() - t < timeout && bytes < bufsize; ) {
    if (!client.available()) {
      delay(1);
      continue;
    }
    buffer[bytes++] = client.read();
    buffer[bytes] = 0;
    if (content) {
      if (++contentBytes == contentLen) break;
    } else if (strstr(buffer, "\r\n\r\n")) {
      // parse HTTP header
      char *p = strstr(buffer, "/1.1 ");
      if (!p) p = strstr(buffer, "/1.0 ");
      if (p) {
        if (p) m_code = atoi(p + 5);
      }
      keepAlive = strstr(buffer, ": close\r\n") == 0;
      p = strstr(buffer, "Content-Length: ");
      if (!p) p = strstr(buffer, "Content-length: ");
      if (p) {
        contentLen = atoi(p + 16);
      }
      content = buffer + bytes;
    }
  }
  if (!content) {
    m_state = HTTP_ERROR;
    return 0;
  }

  m_state = HTTP_CONNECTED;
  if (pbytes) *pbytes = contentBytes;
  if (!keepAlive) close();
  return content;
}

/*******************************************************************************
  SIM7600/SIM7070/SIM5360
*******************************************************************************/
bool CellSIMCOM::begin(CFreematics* device)
{
  getBuffer();
  m_device = device;
  for (byte n = 0; n < 30; n++) {
    device->xbTogglePower(200);
    device->xbPurge();
    if (!check(2000)) continue;
    if (sendCommand("ATE0\r") && sendCommand("ATI\r")) {
      // retrieve module info
      //Serial.print(m_buffer);
      char *p = strstr(m_buffer, "Model:");
      if (!p) {
        sendCommand("AT+SIMCOMATI\r");
        p = strstr(m_buffer, "QCN:");
        if (p) {
          char *q = strchr(p += 4, '_');
          if (q) {
            int l = q - p;
            if (l >= sizeof(m_model)) l = sizeof(m_model) - 1;
            memcpy(m_model, p, l);
            m_model[l] = 0;
          }
        }
        m_type = CELL_SIM7070;
      } else {
        p = strchr(p, '_');
        if (p++) {
          int i = 0;
          while (i < sizeof(m_model) - 1 && p[i] && p[i] != '\r' && p[i] != '\n') {
            m_model[i] = p[i];
            i++;
          } 
          m_model[i] = 0;
        }
        m_type = strstr(m_model, "5360") ? CELL_SIM5360 : CELL_SIM7600;
      }
      p = strstr(m_buffer, "IMEI:");
      if (p) strncpy(IMEI, p + 6, sizeof(IMEI) - 1);
      return true;
    }
  }
  end();
  return false;
}

void CellSIMCOM::end()
{
  setGPS(false);
  if (!sendCommand(m_type == CELL_SIM7070 ? "AT+CPOWD=1\r" : "AT+CPOF\r")) {
    if (m_device) m_device->xbTogglePower(2510);
  }
}

bool CellSIMCOM::setup(const char* apn, unsigned int timeout)
{
  uint32_t t = millis();
  bool success = false;

  if (m_type == CELL_SIM7070) {
    do {
      do {
        success = false;
        sendCommand("AT+CFUN=1\r");
        do {
          delay(500);
          if (sendCommand("AT+CGREG?\r",1000, "+CGREG: 0,")) {
            char *p = strstr(m_buffer, "+CGREG: 0,");
            if (p) {
              char ret = *(p + 10);
              success = ret == '1' || ret == '5';
            }
          }
        } while (!success && millis() - t < timeout);
        if (!success) break;
        success = sendCommand("AT+CGACT?\r", 1000, "+CGACT: 1,");
        break;
      } while (millis() - t < timeout);
      if (!success) break;

      sendCommand("AT+CGNAPN\r");
      if (apn && *apn) {
        sprintf(m_buffer, "AT+CNCFG=0,1,\"%s\"\r", apn);
        sendCommand(m_buffer);
      }
      sendCommand("AT+CNACT=0,1\r");
      sendCommand("AT+CNSMOD?\r");
      sendCommand("AT+CSCLK=0\r");
    } while(0);
  } else {
    do {
      do {
        m_device->xbWrite("AT+CPSI?\r");
        m_buffer[0] = 0;
        const char* answers[] = {"NO SERVICE", ",Online", ",Offline", ",Low Power Mode"};
        int ret = m_device->xbReceive(m_buffer, RECV_BUF_SIZE, 500, answers, 4);
        if (ret == 2) {
          success = true;
          break;
        }
        if (ret == -1 || ret == 4) break;
        delay(500);
      } while (millis() - t < timeout);
      if (!success) break;

      success = false;
      do {
        delay(100);
        if (sendCommand("AT+CREG?\r", 1000, "+CREG: 0,")) {
          char *p = strstr(m_buffer, "+CREG: 0,");
          success = (p && (*(p + 9) == '1' || *(p + 9) == '5'));
        }
      } while (!success && millis() - t < timeout);
      if (!success) break;
      
      //sendCommand("AT+CSOCKAUTH=1,1,\"APN_PASSWORD\",\"APN_USERNAME\"\r");

      if (m_type == CELL_SIM7600) {
        success = false;
        do {
          delay(100);
          if (sendCommand("AT+CGREG?\r",1000, "+CGREG: 0,")) {
            char *p = strstr(m_buffer, "+CGREG: 0,");
            success = (p && (*(p + 10) == '1' || *(p + 10) == '5'));
          }
        } while (!success && millis() - t < timeout);
        if (!success) break;
      }

      sendCommand("AT+CSOCKSETPN=1\r");
      sendCommand("AT+CIPMODE=0\r");
      sendCommand("AT+NETOPEN\r");
    } while(0);
  }
  if (!success) Serial.println(m_buffer);
  return success;
}

bool CellSIMCOM::setGPS(bool on)
{
  if (on) {
    if (m_type == CELL_SIM7070) {
      sendCommand("AT+CGNSPWR=1\r");
      sendCommand("AT+CGNSMOD=1,1,0,0,0\r");
      if (sendCommand("AT+CGNSINF\r", 1000, "+CGNSINF:")) {
        if (!m_gps) {
          m_gps = new GPS_DATA;
          memset(m_gps, 0, sizeof(GPS_DATA));
        }
        return true;
      }
    } else {
      sendCommand("AT+CVAUXV=61\r", 100);
      sendCommand("AT+CVAUXS=1\r", 100);
      for (byte n = 0; n < 3; n++) {
        if ((sendCommand("AT+CGPS=1,1\r") && sendCommand("AT+CGPSINFO=1\r")) || sendCommand("AT+CGPS?\r", 100, "+CGPS: 1")) {
          if (!m_gps) {
            m_gps = new GPS_DATA;
            memset(m_gps, 0, sizeof(GPS_DATA));
          }
          return true;
        }
        sendCommand("AT+CGPS=0\r", 100);
      }
    }
  } else if (m_gps) {
    if (m_type == CELL_SIM7070) {
      sendCommand("AT+CGNSPWR=0\r");
    } else {
      //sendCommand("AT+CVAUXS=0\r");
      sendCommand("AT+CGPS=0\r", 100);
    }
    GPS_DATA *g = m_gps;
    m_gps = 0;
    delete g;
    return true;
  }
  return false;
}

bool CellSIMCOM::getLocation(GPS_DATA** pgd)
{
  if (m_gps) {
      if (pgd) *pgd = m_gps;
      return m_gps->ts != 0;
  } else {
      return false;
  }
}

String CellSIMCOM::getIP()
{
  if (m_type == CELL_SIM7070) {
    sendCommand("AT+CNACT=0,1\r");
    for (int i = 0; i < 30; i++) {
      delay(500);
      if (sendCommand("AT+CNACT?\r", 1000)) {
        char *ip = strstr(m_buffer, "+CNACT:");
        if (ip) {
          ip = strchr(ip, '\"');
          if (ip++ && *ip != '0') {
            char *q = strchr(ip, '\"');
            if (q) *q = 0;
            return ip;
          }
        }
      }
    }
  } else {
    uint32_t t = millis();
    do {
      if (sendCommand("AT+IPADDR\r", 3000, "\r\nOK\r\n")) {
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
  } 
  return "";
}

int CellSIMCOM::RSSI()
{
  if (sendCommand("AT+CSQ\r")) {
      char *p = strchr(m_buffer, ':');
      if (p) {
        int csq = atoi(p + 2);
        if (csq != 99) {
          return csq * 2 - 113;
        }
      }
  }
  return 0;
}

String CellSIMCOM::getOperatorName()
{
  if (sendCommand("AT+COPS?\r")) {
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

bool CellSIMCOM::check(unsigned int timeout)
{
  uint32_t t = millis();
  do {
      if (sendCommand("AT\r", 250)) return true;
  } while (millis() - t < timeout);
  return false;
}

bool CellSIMCOM::checkSIM(const char* pin)
{
  bool success;
  if (pin && *pin) {
    snprintf(m_buffer, RECV_BUF_SIZE, "AT+CPIN=\"%s\"\r", pin);
    sendCommand(m_buffer);
  }
  for (byte n = 0; n < 20 && !(success = sendCommand("AT+CPIN?\r", 500, ": READY")); n++);
  return success;  
}

String CellSIMCOM::queryIP(const char* host)
{
  if (m_type == CELL_SIM7070) {
    sprintf(m_buffer, "AT+CDNSGIP=\"%s\",1,3000\r", host);
    if (sendCommand(m_buffer, 10000, "+CDNSGIP:")) {
      char *p = strstr(m_buffer, host);
      if (p) {
        p = strstr(p, "\",\"");
        if (p) {
          char *ip = p + 3;
          p = strchr(ip, '\"');
          if (p) *p = 0;
          return ip;
        }
      }
    }
  } else {
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
  }
  return "";
}

bool CellSIMCOM::sendCommand(const char* cmd, unsigned int timeout, const char* expected)
{
  if (cmd) {
    m_device->xbWrite(cmd);
    delay(50);
  }
  m_buffer[0] = 0;
  const char* answers[] = {"\r\nOK", "\r\nERROR"};
  byte ret = m_device->xbReceive(m_buffer, RECV_BUF_SIZE, timeout, expected ? &expected : answers, expected ? 1 : 2);
  return ret == 1;
}

float CellSIMCOM::parseDegree(const char* s)
{
  char *p;
  unsigned long left = atol(s);
  unsigned long tenk_minutes = (left % 100UL) * 100000UL;
  if ((p = strchr(s, '.')))
  {
    unsigned long mult = 10000;
    while (isdigit(*++p))
    {
      tenk_minutes += mult * (*p - '0');
      mult /= 10;
    }
  }
  return (left / 100) + (float)tenk_minutes / 6 / 1000000;
}

void CellSIMCOM::checkGPS()
{
  // check and parse GPS data
  char *p;
  if (m_type == CELL_SIM7070) {
    if (m_gps && sendCommand("AT+CGNSINF\r", 100, "+CGNSINF:")) do {
      if (!(p = strchr(m_buffer, ':'))) break;
      p += 2;
      if (strncmp(p, "1,1,", 4)) break;
      p += 4;
      m_gps->time = atol(p + 8) * 100 + atoi(p + 15);
      *(p + 8) = 0;
      int day = atoi(p + 6);
      *(p + 6) = 0;
      int month = atoi(p + 4);
      *(p + 4) = 0;
      int year = atoi(p + 2);
      m_gps->date = year + month * 100 + day * 10000;
      if (!(p = strchr(p + 9, ','))) break;
      m_gps->lat = atof(++p);
      if (!(p = strchr(p, ','))) break;
      m_gps->lng = atof(++p);
      if (!(p = strchr(p, ','))) break;
      m_gps->alt = atof(++p);
      if (!(p = strchr(p, ','))) break;
      m_gps->speed = atof(++p) * 1000 / 1852;
      if (!(p = strchr(p, ','))) break;
      m_gps->heading = atoi(++p);
      m_gps->ts = millis();
    } while (0);
  } else {
    if (m_gps && (p = strstr(m_buffer, "+CGPSINFO:"))) do {
      if (!(p = strchr(p, ':'))) break;
      if (*(++p) == ',') break;
      m_gps->lat = parseDegree(p);
      if (!(p = strchr(p, ','))) break;
      if (*(++p) == 'S') m_gps->lat = -m_gps->lat;
      if (!(p = strchr(p, ','))) break;
      m_gps->lng = parseDegree(++p);
      if (!(p = strchr(p, ','))) break;
      if (*(++p) == 'W') m_gps->lng = -m_gps->lng;
      if (!(p = strchr(p, ','))) break;
      m_gps->date = atoi(++p);
      if (!(p = strchr(p, ','))) break;
      m_gps->time = atof(++p) * 100;
      if (!(p = strchr(p, ','))) break;
      m_gps->alt = atof(++p);
      if (!(p = strchr(p, ','))) break;
      m_gps->speed = atof(++p);
      if (!(p = strchr(p, ','))) break;
      m_gps->heading = atoi(++p);
      m_gps->ts = millis();
    } while (0);
  }
}

char* CellSIMCOM::getBuffer()
{
  if (!m_buffer) m_buffer = (char*)malloc(RECV_BUF_SIZE);
  return m_buffer;
}

bool CellUDP::open(const char* host, uint16_t port)
{
  if (host) {
    udpIP = queryIP(host);
    if (!udpIP.length()) {
      udpIP = host;
    }
    udpPort = port;
  }
  if (!udpIP.length()) return false;
  if (m_type == CELL_SIM7070) {
    sendCommand("AT+CNACT=0,1\r");
    sendCommand("AT+CACID=0\r");
    sprintf(m_buffer, "AT+CAOPEN=0,0,\"UDP\",\"%s\",%u\r", udpIP.c_str(), udpPort);
    if (!sendCommand(m_buffer, 3000)) {
      Serial.println(m_buffer);
      return false;
    }
    return true;
  } else {
    sprintf(m_buffer, "AT+CIPOPEN=0,\"UDP\",\"%s\",%u,8000\r", udpIP.c_str(), udpPort);
    if (!sendCommand(m_buffer, 3000)) {
      Serial.println(m_buffer);
      return false;
    }
    return true;
  }
}

bool CellUDP::close()
{
  if (m_type == CELL_SIM7070) {
    sendCommand("AT+CACLOSE=0\r");
    return sendCommand("AT+CNACT=0,0\r");
  } else {
    return sendCommand("AT+CIPCLOSE=0\r");
  }
}

bool CellUDP::send(const char* data, unsigned int len)
{
  if (m_type == CELL_SIM7070) {
    sendCommand("AT+CASTATE?\r");
    sprintf(m_buffer, "AT+CASEND=0,%u\r", len);
    sendCommand(m_buffer, 100, "\r\n>");
    if (sendCommand(data, 1000)) return true;
  } else {
    int n = sprintf(m_buffer, "AT+CIPSEND=0,%u,\"%s\",%u\r", len, udpIP.c_str(), udpPort);
    m_device->xbWrite(m_buffer, n);
    delay(10);
    m_device->xbWrite(data, len);
    const char* answers[] = {"\r\nERROR", "OK\r\n\r\n+CIPSEND:", "\r\nRECV FROM:"};
    byte ret = m_device->xbReceive(m_buffer, RECV_BUF_SIZE, 1000, answers, 3);
    if (ret > 1) return true;
  }
  return false;
}

char* CellUDP::receive(int* pbytes, unsigned int timeout)
{
  if (m_type == CELL_SIM7070) {
    if (!strstr(m_buffer, "+CADATAIND: 0")) {
      if (!sendCommand(0, timeout, "+CADATAIND: 0")) return 0;
    }
    if (sendCommand("AT+CARECV=0,384\r", timeout)) {
      char *p = strstr(m_buffer, "+CARECV: ");
      if (p) {
        if (pbytes) *pbytes = atoi(p + 9);
        p = strchr(m_buffer, ',');
        return p ? p + 1 : m_buffer;
      }
    }
  } else {
    char *data = checkIncoming(pbytes);
    if (data) return data;
    if (sendCommand(0, timeout, "+IPD")) {
      return checkIncoming(pbytes);
    }
  }  
  return 0;
}

char* CellUDP::checkIncoming(int* pbytes)
{
  checkGPS();
  char *p = strstr(m_buffer, "+IPD");
	if (p) {
    *p = '-'; // mark this datagram as checked
    int len = atoi(p + 4);
    if (pbytes) *pbytes = len;
    p = strchr(p, '\n');
    if (p) {
      if (strlen(++p) > len) *(p + len) = 0;
      return p;
    }
  }
	return 0;
}

void CellHTTP::init()
{
  if (m_type != CELL_SIM7070) {
    sendCommand("AT+CHTTPSSTOP\r");
    sendCommand("AT+CHTTPSSTART\r");
  }
}

bool CellHTTP::open(const char* host, uint16_t port)
{
  if (m_type == CELL_SIM7070) {
    sendCommand("AT+CNACT=0,1\r");
    sendCommand("AT+CACID=0\r");

    sprintf(m_buffer, "AT+SHCONF=\"URL\",\"http://%s:%u\"\r", host, port);
    if (!sendCommand(m_buffer)) {
      return false;
    }
    sendCommand("AT+SHCONF=\"HEADERLEN\",256\r");
    sendCommand("AT+SHCONF=\"BODYLEN\",1024\r");
    sendCommand("AT+SHCONN\r", HTTP_CONN_TIMEOUT);
    if (sendCommand("AT+SHSTATE?\r")) {
      if (strstr(m_buffer, "+SHSTATE: 1")) {
        m_state = HTTP_CONNECTED;
        m_host = host;
        sendCommand("AT+SHCHEAD\r");
        sendCommand("AT+SHAHEAD=\"User-Agent\",\"curl/7.47.0\"\r"); 
        sendCommand("AT+SHAHEAD=\"Cache-control\",\"no-cache\"\r");
        sendCommand("AT+SHAHEAD=\"Connection\",\"keep-alive\"\r");
        sendCommand("AT+SHAHEAD=\"Accept\",\"*/*\"\r");
        m_state = HTTP_CONNECTED;
        return true;
      }
    }
  } else {
    memset(m_buffer, 0, RECV_BUF_SIZE);
    sprintf(m_buffer, "AT+CHTTPSOPSE=\"%s\",%u,%u\r", host, port, port == 443 ? 2: 1);
    if (sendCommand(m_buffer, 1000)) {
      if (sendCommand(0, HTTP_CONN_TIMEOUT, "+CHTTPSOPSE:")) {
        m_state = HTTP_CONNECTED;
        m_host = host;
        checkGPS();
        return true;
      }
    }
    checkGPS();
  }
  Serial.println(m_buffer);
  m_state = HTTP_ERROR;
  return false;
}

bool CellHTTP::close()
{
  m_state = HTTP_DISCONNECTED;
  if (m_type == CELL_SIM7070) {
    return sendCommand("AT+SHDISC\r");
  } else if (m_type == CELL_SIM5360) {
    return sendCommand("AT+CHTTPSCLSE\r", 1000, "+CHTTPSCLSE:");
  } else {
    return sendCommand("AT+CIPCLOSE=0\r");
  }
}

bool CellHTTP::send(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload, int payloadSize)
{
  if (m_type == CELL_SIM7070) {
    if (method == METHOD_POST) {
      sprintf(m_buffer, "AT+SHBOD=%u,100\r", payloadSize);
      if (sendCommand(m_buffer, 100, "\r\n>")) {
        sendCommand(payload);
      }
    }
    snprintf(m_buffer, RECV_BUF_SIZE, "AT+SHREQ=\"%s\",%u\r", path, method == METHOD_GET ? 1 : 3);
    if (sendCommand(m_buffer, HTTP_CONN_TIMEOUT)) {
      char *p;
      int len = 0;
      if (strstr(m_buffer, "+SHREQ:") || sendCommand(0, HTTP_CONN_TIMEOUT, "+SHREQ:")) {
        if ((p = strstr(m_buffer, "+SHREQ:")) && (p = strchr(p, ','))) {
          m_code = atoi(++p);
          if ((p = strchr(p, ','))) len = atoi(++p);
        }
      }
      if (len > 0) {
        if (len > RECV_BUF_SIZE - 16) len = RECV_BUF_SIZE - 16;
        sprintf(m_buffer, "AT+SHREAD=0,%u\r", len);
        if (sendCommand(m_buffer)) {
          m_state = HTTP_SENT;
          return true;
        }
      }
    }
  } else {
    String header = genHeader(method, path, keepAlive, payload, payloadSize);
    int len = header.length();
    sprintf(m_buffer, "AT+CHTTPSSEND=%u\r", len + payloadSize);
    if (!sendCommand(m_buffer, 100, ">")) {
      m_state = HTTP_DISCONNECTED;
      return false;
    }
    // send HTTP header
    m_device->xbWrite(header.c_str());
    // send POST payload if any
    if (payload) m_device->xbWrite(payload, payloadSize);
    if (sendCommand(0, 200, "+CHTTPSSEND:")) {
      m_state = HTTP_SENT;
      return true;
    }
  }
  Serial.println(m_buffer);
  m_state = HTTP_ERROR;
  return false;
}

char* CellHTTP::receive(int* pbytes, unsigned int timeout)
{
  if (m_type == CELL_SIM7070) {
    if (!sendCommand(0, timeout, "+SHREAD:")) {
      return 0;
    }
    m_state = HTTP_CONNECTED;

    char *p = strstr(m_buffer, "+SHREAD:");
    if (p) {
      int bytes = atoi(p += 9);
      if (pbytes) *pbytes = bytes;
      p = strchr(p, '\n');
      if (p++) {
        checkGPS();
        return p;
      }
    }
    checkGPS();
    return 0;
  } else {
    // start receiving
    int received = 0;
    char* payload = 0;
    bool keepalive;

    // wait for RECV EVENT
    if (!sendCommand(0, timeout, "RECV EVENT")) {
      checkGPS();
      return 0;
    }
    bool legacy = false;
    char *p = strstr(m_buffer, "RECV EVENT");
    if (p) {
      if (*(p - 1) == ' ')
        legacy = true;
      else if (*(p - 1) != ':')
        return 0;
    }
    
    checkGPS();

    /*
      +CHTTPSRECV:XX\r\n
      [XX bytes from server]\r\n
      +CHTTPSRECV: 0\r\n
    */
    // TODO: implement for multiple chunks of data
    // only deals with first chunk now
    sprintf(m_buffer, "AT+CHTTPSRECV=%u\r", RECV_BUF_SIZE - 36);
    if (sendCommand(m_buffer, timeout, legacy ? "\r\n+CHTTPSRECV: 0" : "\r\n+CHTTPSRECV:0")) {
      char *p = strstr(m_buffer, "\r\n+CHTTPSRECV: DATA");
      if (p) {
        if ((p = strchr(p, ','))) {
          received = atoi(p + 1);
          char *q = strchr(p, '\n');
          payload = q ? (q + 1) : p;
          if (m_buffer + RECV_BUF_SIZE - payload > received) {
            payload[received] = 0;
          }
        }
      }
    }
    if (received == 0) {
      m_state = HTTP_ERROR;
      return 0;
    }

    p = strstr(payload, "/1.1 ");
    if (!p) p = strstr(payload, "/1.0 ");
    if (p) {
      if (p) m_code = atoi(p + 5);
    }
    keepalive = strstr(m_buffer, ": close\r\n") == 0;

    m_state = HTTP_CONNECTED;
    if (!keepalive) close();
    if (pbytes) *pbytes = received;
    return payload;
  }
}

/*************************************************************************************************
  SIM7600
*************************************************************************************************/

void ClientSIM7600::end()
{
  setGPS(false);
  sendCommand("AT+CPOF\r");
  delay(1000);
}

bool ClientSIM7600::setup(const char* apn, unsigned int timeout)
{
  uint32_t t = millis();
  bool success = false;
  //sendCommand("AT+CNMP=13\r"); // GSM only
  //sendCommand("AT+CNMP=14\r"); // WCDMA only
  //sendCommand("AT+CNMP=38\r"); // LTE only
  do {
    do {
      m_device->xbWrite("AT+CPSI?\r");
      m_buffer[0] = 0;
      const char* answers[] = {"NO SERVICE", ",Online", ",Offline", ",Low Power Mode"};
      int ret = m_device->xbReceive(m_buffer, RECV_BUF_SIZE, 500, answers, 4);
      if (ret == 2) {
        success = true;
        break;
      }
      if (ret == -1 || ret == 4) break;
      delay(500);
    } while (millis() - t < timeout);
    if (!success) break;

    do {
      delay(100);
      if (sendCommand("AT+CREG?\r", 1000, "+CREG: 0,")) {
        char *p = strstr(m_buffer, "+CREG: 0,");
        success = (p && (*(p + 9) == '1' || *(p + 9) == '5'));
      }
    } while (!success && millis() - t < timeout);
    if (!success) break;

    success = false;
    do {
      delay(100);
      if (sendCommand("AT+CGREG?\r",1000, "+CGREG: 0,")) {
        char *p = strstr(m_buffer, "+CGREG: 0,");
        success = (p && (*(p + 10) == '1' || *(p + 10) == '5'));
      }
    } while (!success && millis() - t < timeout);
    if (!success) break;

    if (apn && *apn) {
      sprintf(m_buffer, "AT+CGSOCKCONT=1,\"IP\",\"%s\"\r", apn);
      sendCommand(m_buffer);
    }
    if (!success) break;

    //sendCommand("AT+CSOCKAUTH=1,1,\"APN_PASSWORD\",\"APN_USERNAME\"\r");

    sendCommand("AT+CSOCKSETPN=1\r");
    sendCommand("AT+CIPMODE=0\r");
    sendCommand("AT+NETOPEN\r");
  } while(0);
  return success;
}

bool ClientSIM7600::setGPS(bool on)
{
  if (on) {
    sendCommand("AT+CVAUXV=61\r");
    sendCommand("AT+CVAUXS=1\r");
    for (byte n = 0; n < 3; n++) {
      if ((sendCommand("AT+CGPS=1,1\r") && sendCommand("AT+CGPSINFO=1\r")) || sendCommand("AT+CGPS?\r", 100, "+CGPS: 1")) {
        if (!m_gps) {
          m_gps = new GPS_DATA;
          memset(m_gps, 0, sizeof(GPS_DATA));
        }
        return true;
      }
      sendCommand("AT+CGPS=0\r", 100);
    }
  } else if (m_gps) {
    //sendCommand("AT+CVAUXS=0\r", 100);
    sendCommand("AT+CGPS=0\r", 100);
    delete m_gps;
    m_gps = 0;
    return true;
  }
  return false;
}

bool UDPClientSIM7600::open(const char* host, uint16_t port)
{
  if (host) {
    udpIP = queryIP(host);
    if (!udpIP.length()) {
      udpIP = host;
    }
    udpPort = port;
  }
  if (!udpIP.length()) return false;
  sprintf(m_buffer, "AT+CIPOPEN=0,\"UDP\",\"%s\",%u,8000\r", udpIP.c_str(), udpPort);
  if (!sendCommand(m_buffer, 3000)) {
    close();
    Serial.println(m_buffer);
    return false;
  }
  return true;
}

bool UDPClientSIM7600::close()
{
  return sendCommand("AT+CIPCLOSE=0\r");
}

bool UDPClientSIM7600::send(const char* data, unsigned int len)
{
  int n = sprintf(m_buffer, "AT+CIPSEND=0,%u,\"%s\",%u\r", len, udpIP.c_str(), udpPort);
  m_device->xbWrite(m_buffer, n);
  delay(10);
  m_device->xbWrite(data, len);

  const char* answers[] = {"\r\nERROR", "OK\r\n\r\n+CIPSEND:", "\r\nRECV FROM:"};
  byte ret = m_device->xbReceive(m_buffer, RECV_BUF_SIZE, 1000, answers, 3);
  if (ret > 1) return true;
  return false;
}

char* UDPClientSIM7600::receive(int* pbytes, unsigned int timeout)
{
	char *data = checkIncoming(pbytes);
	if (data) return data;
  if (sendCommand(0, timeout, "+IPD")) {
		return checkIncoming(pbytes);
  }
  return 0;
}

char* UDPClientSIM7600::checkIncoming(int* pbytes)
{
  checkGPS();
  char *p = strstr(m_buffer, "+IPD");
	if (p) {
    *p = '-'; // mark this datagram as checked
    int len = atoi(p + 4);
    if (pbytes) *pbytes = len;
    p = strchr(p, '\n');
    if (p) {
      if (strlen(++p) > len) *(p + len) = 0;
      return p;
    }
  }
	return 0;
}

bool HTTPClientSIM7600::open(const char* host, uint16_t port)
{
  if (!host) {
    close();
    sendCommand("AT+CHTTPSSTOP\r");
    sendCommand("AT+CHTTPSSTART\r");
    return true;
  }

  memset(m_buffer, 0, RECV_BUF_SIZE);
  sprintf(m_buffer, "AT+CHTTPSOPSE=\"%s\",%u,%u\r", host, port, port == 443 ? 2: 1);
  if (sendCommand(m_buffer, 1000)) {
    if (sendCommand(0, HTTP_CONN_TIMEOUT, "+CHTTPSOPSE:")) {
      m_state = HTTP_CONNECTED;
      m_host = host;
      checkGPS();
      return true;
    }
  }
  checkGPS();
  Serial.println(m_buffer);
  m_state = HTTP_ERROR;
  return false;
}

bool HTTPClientSIM7600::close()
{
  m_state = HTTP_DISCONNECTED;
  return sendCommand("AT+CHTTPSCLSE\r", 1000, "+CHTTPSCLSE:");
}

bool HTTPClientSIM7600::send(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload, int payloadSize)
{
  String header = genHeader(method, path, keepAlive, payload, payloadSize);
  int len = header.length();
  sprintf(m_buffer, "AT+CHTTPSSEND=%u\r", len + payloadSize);
  if (!sendCommand(m_buffer, 100, ">")) {
    m_state = HTTP_DISCONNECTED;
    return false;
  }
  // send HTTP header
  m_device->xbWrite(header.c_str());
  // send POST payload if any
  if (payload) m_device->xbWrite(payload, payloadSize);
  if (sendCommand(0, 200, "+CHTTPSSEND:")) {
    m_state = HTTP_SENT;
    return true;
  }
  //Serial.println(m_buffer);
  m_state = HTTP_ERROR;
  return false;
}

char* HTTPClientSIM7600::receive(int* pbytes, unsigned int timeout)
{
  // start receiving
  int received = 0;
  char* payload = 0;

  // wait for +CHTTPS:RECV EVENT
  if (!sendCommand(0, timeout, "RECV EVENT")) {
    checkGPS();
    return 0;
  }
  
  bool legacy = false;
  char *p = strstr(m_buffer, "RECV EVENT");
  if (p) {
    if (*(p - 1) == ' ')
      legacy = true;
    else if (*(p - 1) != ':')
      return 0;
  }
  // FIXME
  if (strstr(m_buffer, " 200 ")) {
    m_code = 200;
  }

  checkGPS();

  /*
    +CHTTPSRECV: DATA,XX\r\n
    [XX bytes from server]\r\n
    +CHTTPSRECV:0\r\n
  */
  // TODO: implement for multiple chunks of data
  // only deals with first chunk now
  sprintf(m_buffer, "AT+CHTTPSRECV=%u\r", RECV_BUF_SIZE - 36);
  if (sendCommand(m_buffer, timeout, legacy ? "\r\n+CHTTPSRECV: 0" : "\r\n+CHTTPSRECV:0")) {
    char *p = strstr(m_buffer, "\r\n+CHTTPSRECV: DATA");
    if (p) {
      if ((p = strchr(p, ','))) {
        received = atoi(p + 1);
        char *q = strchr(p, '\n');
        payload = q ? (q + 1) : p;
        if (m_buffer + RECV_BUF_SIZE - payload > received) {
          payload[received] = 0;
        }
      }
    }
  }
  if (received == 0) {
    m_state = HTTP_ERROR;
    return 0;
  } else {
    m_state = HTTP_CONNECTED;
    if (pbytes) *pbytes = received;
    return payload;
  }
}

/*************************************************************************************************
  SIM7070
*************************************************************************************************/

bool ClientSIM7070::begin(CFreematics* device)
{
  m_type = CELL_SIM7070;
  m_device = device;
  for (byte n = 0; n < 3; n++) {
    // try turning on module
    device->xbTogglePower();
    // discard any stale data
    device->xbPurge();
    delay(3000);
    for (byte m = 0; m < 5; m++) {
      if (sendCommand("AT\r") && sendCommand("ATE0\r") && sendCommand("AT+SIMCOMATI\r")) {
        // retrieve module info
        //Serial.print(m_buffer);
        char *p = strstr(m_buffer, "IMEI:");
        if (p) strncpy(IMEI, p + 6, sizeof(IMEI) - 1);
        p = strstr(m_buffer, "QCN:");
        if (p) {
          char *q = strchr(p += 4, '_');
          if (q) {
            int l = q - p;
            if (l >= sizeof(m_model)) l = sizeof(m_model) - 1;
            memcpy(m_model, p, l);
            m_model[l] = 0;
          }
        }
        return true;
      }
    }
  }
  return false;
}

void ClientSIM7070::end()
{
  setGPS(false);
  sendCommand("AT+CPOWD=1\r");
  delay(1000);
}

bool ClientSIM7070::setup(const char* apn, unsigned int timeout)
{
  uint32_t t = millis();
  bool success = false;
  do {
    do {
      success = false;
      sendCommand("AT+CFUN=1\r");
      do {
        delay(500);
        if (sendCommand("AT+CGREG?\r",1000, "+CGREG: 0,")) {
          char *p = strstr(m_buffer, "+CGREG: 0,");
          if (p) {
            char ret = *(p + 10);
            success = ret == '1' || ret == '5';
          }
        }
      } while (!success && millis() - t < timeout);
      if (!success) break;
      success = sendCommand("AT+CGACT?\r", 1000, "+CGACT: 1,");
      break;
    } while (millis() - t < timeout);
    if (!success) break;

    /*
    if (apn && *apn) {
      sprintf(m_buffer, "AT+CGSOCKCONT=1,\"IP\",\"%s\"\r", apn);
      sendCommand(m_buffer);
    }
    if (!success) break;
    */

    //sendCommand("AT+CSOCKAUTH=1,1,\"APN_PASSWORD\",\"APN_USERNAME\"\r");

    //sendCommand("AT+CSOCKSETPN=1\r");
    //sendCommand("AT+CIPMODE=0\r");
    //sendCommand("AT+NETOPEN\r");

    sendCommand("AT+CGNAPN\r");
    sprintf(m_buffer, "AT+CNCFG=0,1,\"%s\"\r", apn);
    sendCommand(m_buffer);
    sendCommand("AT+CNACT=0,1\r");
    sendCommand("AT+CNSMOD?\r");
    sendCommand("AT+CSCLK=0\r");
  } while(0);
  return success;
}

bool ClientSIM7070::setGPS(bool on)
{
  if (on) {
      sendCommand("AT+CGNSPWR=1\r");
      sendCommand("AT+CGNSMOD=1,1,0,0,0\r");
      if (sendCommand("AT+CGNSINF\r", 1000, "+CGNSINF:")) {
        if (!m_gps) {
          m_gps = new GPS_DATA;
          memset(m_gps, 0, sizeof(GPS_DATA));
        }
        return true;
      }
    return false;
  } else {
    sendCommand("AT+CGNSPWR=0\r");
    if (m_gps) {
      GPS_DATA *g = m_gps;
      m_gps = 0;
      delete g;
    }
    return true;
  }
}

void ClientSIM7070::checkGPS()
{
  // check and parse GPS data
  char *p;
  if (m_gps && sendCommand("AT+CGNSINF\r", 100, "+CGNSINF:")) do {
    if (!(p = strchr(m_buffer, ':'))) break;
    p += 2;
    if (strncmp(p, "1,1,", 4)) break;
    p += 4;
    m_gps->time = atol(p + 8) * 100 + atoi(p + 15);
    *(p + 8) = 0;
    int day = atoi(p + 6);
    *(p + 6) = 0;
    int month = atoi(p + 4);
    *(p + 4) = 0;
    int year = atoi(p + 2);
    m_gps->date = year + month * 100 + day * 10000;
    if (!(p = strchr(p + 9, ','))) break;
    m_gps->lat = atof(++p);
    if (!(p = strchr(p, ','))) break;
    m_gps->lng = atof(++p);
    if (!(p = strchr(p, ','))) break;
    m_gps->alt = atof(++p);
    if (!(p = strchr(p, ','))) break;
    m_gps->speed = atof(++p) * 1000 / 1852;
    if (!(p = strchr(p, ','))) break;
    m_gps->heading = atoi(++p);
    m_gps->ts = millis();
  } while (0);
}

String ClientSIM7070::getIP()
{
  sendCommand("AT+CNACT=0,1\r");
  for (int i = 0; i < 30; i++) {
    delay(500);
    if (sendCommand("AT+CNACT?\r", 1000)) {
      char *ip = strstr(m_buffer, "+CNACT:");
      if (ip) {
        ip = strchr(ip, '\"');
        if (ip++ && *ip != '0') {
          char *q = strchr(ip, '\"');
          if (q) *q = 0;
          return ip;
        }
      }
    }
  }
  return "";
}

String ClientSIM7070::queryIP(const char* host)
{
  sprintf(m_buffer, "AT+CDNSGIP=\"%s\",1,3000\r", host);
  if (sendCommand(m_buffer, 5000, "+CDNSGIP:")) {
    char *p = strstr(m_buffer, host);
    if (p) {
      p = strstr(p, "\",\"");
      if (p) {
        char *ip = p + 3;
        p = strchr(ip, '\"');
        if (p) *p = 0;
        return ip;
      }
    }
  }
  return "";
}

bool UDPClientSIM7070::open(const char* host, uint16_t port)
{
  /*
  if (host) {
    udpIP = queryIP(host);
    if (!udpIP.length()) {
      udpIP = host;
    }
    udpPort = port;
  }
  */
  close();
  sendCommand("AT+CNACT=0,1\r");
  sendCommand("AT+CACID=0\r");
  sprintf(m_buffer, "AT+CAOPEN=0,0,\"UDP\",\"%s\",%u\r", host, port);
  if (!sendCommand(m_buffer, 5000)) {
    close();
    Serial.println(m_buffer);
    return false;
  }
  return true;
}

bool UDPClientSIM7070::close()
{
  sendCommand("AT+CACLOSE=0\r");
  return sendCommand("AT+CNACT=0,0\r");
}

bool UDPClientSIM7070::send(const char* data, unsigned int len)
{
  sendCommand("AT+CASTATE?\r");
  sprintf(m_buffer, "AT+CASEND=0,%u\r", len);
  sendCommand(m_buffer, 100, "\r\n>");
  if (sendCommand(data, 1000)) return true;
  return false;
}

char* UDPClientSIM7070::receive(int* pbytes, unsigned int timeout)
{
  if (!strstr(m_buffer, "+CADATAIND: 0")) {
    if (!sendCommand(0, timeout, "+CADATAIND: 0")) return 0;
  }
  if (sendCommand("AT+CARECV=0,384\r", timeout, "+CARECV")) {
    char *p = strchr(m_buffer, ',');
    return p ? p + 1 : m_buffer;
  }
  return 0;
}

bool HTTPClientSIM7070::open(const char* host, uint16_t port)
{
  if (!host) {
    //close();
    return true;
  }

  sendCommand("AT+CNACT=0,1\r");
  sendCommand("AT+CACID=0\r");

  sprintf(m_buffer, "AT+SHCONF=\"URL\",\"http://%s:%u\"\r", host, port);
  if (!sendCommand(m_buffer)) {
    return false;
  }
  sendCommand("AT+SHCONF=\"HEADERLEN\",256\r");
  sendCommand("AT+SHCONF=\"BODYLEN\",1024\r");
  sendCommand("AT+SHCONN\r", HTTP_CONN_TIMEOUT);
  if (sendCommand("AT+SHSTATE?\r")) {
    if (strstr(m_buffer, "+SHSTATE: 1")) {
      m_state = HTTP_CONNECTED;
      m_host = host;
      sendCommand("AT+SHCHEAD\r");
      sendCommand("AT+SHAHEAD=\"User-Agent\",\"curl/7.47.0\"\r"); 
      sendCommand("AT+SHAHEAD=\"Cache-control\",\"no-cache\"\r");
      sendCommand("AT+SHAHEAD=\"Connection\",\"keep-alive\"\r");
      sendCommand("AT+SHAHEAD=\"Accept\",\"*/*\"\r");
      m_state = HTTP_CONNECTED;
      return true;
    }
  }
  m_state = HTTP_ERROR;
  return false;
}

bool HTTPClientSIM7070::close()
{
  //sendCommand("AT+CNACT=0,0\r");
  m_state = HTTP_DISCONNECTED;
  return sendCommand("AT+SHDISC\r");
}

bool HTTPClientSIM7070::send(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload, int payloadSize)
{
  if (method == METHOD_POST) {
    sprintf(m_buffer, "AT+SHBOD=%u,100\r", payloadSize);
    if (sendCommand(m_buffer, 100, "\r\n>")) {
      sendCommand(payload);
    }
  }
  snprintf(m_buffer, RECV_BUF_SIZE, "AT+SHREQ=\"%s\",%u\r", path, method == METHOD_GET ? 1 : 3);
  if (sendCommand(m_buffer, HTTP_CONN_TIMEOUT)) {
    char *p;
    int len = 0;
    if (strstr(m_buffer, "+SHREQ:") || sendCommand(0, HTTP_CONN_TIMEOUT, "+SHREQ:")) {
      if ((p = strstr(m_buffer, "+SHREQ:")) && (p = strchr(p, ','))) {
        m_code = atoi(++p);
        if ((p = strchr(p, ','))) len = atoi(++p);
      }
    }
    if (len > 0) {
      if (len > RECV_BUF_SIZE - 16) len = RECV_BUF_SIZE - 16;
      sprintf(m_buffer, "AT+SHREAD=0,%u\r", len);
      if (sendCommand(m_buffer)) {
        m_state = HTTP_SENT;
        return true;
      }
    }
  }
  m_state = HTTP_ERROR;
  return false;
}

char* HTTPClientSIM7070::receive(int* pbytes, unsigned int timeout)
{
  if (!sendCommand(0, timeout, "+SHREAD:")) {
    return 0;
  }
  m_state = HTTP_CONNECTED;

  char *p = strstr(m_buffer, "+SHREAD:");
  if (p) {
    int bytes = atoi(p += 9);
    if (pbytes) *pbytes = bytes;
    p = strchr(p, '\n');
    if (p++) {
      checkGPS();
      return p;
    }
  }
  checkGPS();
  return 0;
}
