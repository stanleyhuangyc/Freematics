/*************************************************************************
* Freematics Hub Client implementations for various communication devices
* Distributed under BSD license
* Visit https://freematics.com for more information
* (C)2012-2019 Developed by Stanley Huang <stanley@freematics.com.au>
*************************************************************************/

#include "FreematicsCell.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"

int dumpLine(char* buffer, int len)
{
	int bytesToDump = len >> 1;
	for (int i = 0; i < len; i++) {
		// find out first line end and discard the first line
		if (buffer[i] == '\r' || buffer[i] == '\n') {
			// go through all following \r or \n if any
			while (++i < len && (buffer[i] == '\r' || buffer[i] == '\n'));
			bytesToDump = i;
			break;
		}
	}
	memmove(buffer, buffer + bytesToDump, len - bytesToDump);
	return bytesToDump;
}

bool FreematicsBee::xbBegin(unsigned long baudrate, int pinRx, int pinTx)
{
    uart_config_t uart_config = {
        .baud_rate = (int)baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

#if VERBOSE_XBEE
    Serial.print("Bee Rx:");
    Serial.print(pinRx);
    Serial.print(" Tx:");
    Serial.println(pinTx);
#endif
    //Configure UART parameters
    uart_param_config(BEE_UART_NUM, &uart_config);
    //Set UART pins
    uart_set_pin(BEE_UART_NUM, pinTx, pinRx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver
    uart_driver_install(BEE_UART_NUM, UART_BUF_SIZE, 0, 0, NULL, 0);

#ifdef PIN_BEE_PWR
	pinMode(PIN_BEE_PWR, OUTPUT);
	digitalWrite(PIN_BEE_PWR, LOW);
#endif
    return true;
}

void FreematicsBee::xbEnd()
{
    uart_driver_delete(BEE_UART_NUM);
    digitalWrite(PIN_BEE_PWR, LOW);
}

void FreematicsBee::xbWrite(const char* cmd)
{
    uart_write_bytes(BEE_UART_NUM, cmd, strlen(cmd));
#if VERBOSE_XBEE
    Serial.print("=== SENT@");
    Serial.print(millis());
    Serial.println(" ===");
	Serial.println(cmd);
	Serial.println("==================");
#endif
}

void FreematicsBee::xbWrite(const char* data, int len)
{
    uart_write_bytes(BEE_UART_NUM, data, len);
}

int FreematicsBee::xbRead(char* buffer, int bufsize, unsigned int timeout)
{
    int recv = 0;
    uint32_t t = millis();
    do {
        uint8_t c;
        int len = uart_read_bytes(BEE_UART_NUM, &c, 1, 0);
        if (len == 1) {
            if (c >= 0xA && c <= 0x7E) {
                buffer[recv++] = c;
            }
        } else if (recv > 0) {
            break;
        }
    } while (recv < bufsize && millis() - t < timeout);
    return recv;
}

int FreematicsBee::xbReceive(char* buffer, int bufsize, unsigned int timeout, const char** expected, byte expectedCount)
{
    int bytesRecv = 0;
	uint32_t t = millis();
	do {
		if (bytesRecv >= bufsize - 16) {
			bytesRecv -= dumpLine(buffer, bytesRecv);
		}
		int n = xbRead(buffer + bytesRecv, bufsize - bytesRecv - 1, 50);
		if (n > 0) {
#if VERBOSE_XBEE
			Serial.print("=== RECV@");
            Serial.print(millis());
            Serial.println(" ===");
			buffer[bytesRecv + n] = 0;
			Serial.print(buffer + bytesRecv);
			Serial.println("==================");
#endif
			bytesRecv += n;
			buffer[bytesRecv] = 0;
			for (byte i = 0; i < expectedCount; i++) {
				// match expected string(s)
				if (expected[i] && strstr(buffer, expected[i])) return i + 1;
			}
		} else if (n == -1) {
			// an erroneous reading
#if VERBOSE_XBEE
			Serial.print("RECV ERROR");
#endif
			break;
		}
	} while (millis() - t < timeout);
	buffer[bytesRecv] = 0;
	return 0;
}

void FreematicsBee::xbPurge()
{
    uart_flush(BEE_UART_NUM);
}

void FreematicsBee::xbTogglePower(int pinPowerKey)
{
#ifdef PIN_BEE_PWR
    digitalWrite(pinPowerKey, HIGH);
    delay(100);
#if VERBOSE_XBEE
	Serial.println("xBee power pin set to low");
#endif
	digitalWrite(pinPowerKey, LOW);
	delay(1010);
#if VERBOSE_XBEE
	Serial.println("xBee power pin set to high");
#endif
    digitalWrite(pinPowerKey, HIGH);
#endif
    delay(100);
    digitalWrite(pinPowerKey, LOW);
}

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
  Implementation for SIM800 (SIM800 AT command-set)
*******************************************************************************/

bool ClientSIM800::begin()
{
  for (byte n = 0; n < 10; n++) {
    // try turning on module
    xbTogglePower();
    // discard any stale data
    delay(2000);
    xbPurge();
    for (byte m = 0; m < 3; m++) {
      if (sendCommand("AT\r")) {
        return true;
      }
    }
  }
  return false;
}

void ClientSIM800::end()
{
  sendCommand("AT+CPOWD=1\r");
}

bool ClientSIM800::setup(const char* apn, bool gps, unsigned int timeout)
{
  uint32_t t = millis();
  bool success = false;
  sendCommand("ATE0\r");
  do {
    success = sendCommand("AT+CREG?\r", 3000, "+CREG: 0,1") != 0;
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

String ClientSIM800::getIP()
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

int ClientSIM800::getSignal()
{
  if (sendCommand("AT+CSQ\r", 500)) {
      char *p = strchr(m_buffer, ':');
      if (p) {
        int csq = atoi(p + 2);
        if (csq == 0)
          return -115;
        else if (csq == 1)
          return -111;
        else if (csq != 99)
          return csq * 2 - 114;
      }
  }
  return 0;
}

String ClientSIM800::getOperatorName()
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

bool ClientSIM800::checkSIM()
{
  return (sendCommand("AT+CPIN?\r") && strstr(m_buffer, "READY"));
}

String ClientSIM800::queryIP(const char* host)
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

bool ClientSIM800::sendCommand(const char* cmd, unsigned int timeout, const char* expected)
{
  if (cmd) {
    xbWrite(cmd);
  }
  m_buffer[0] = 0;
  byte ret = xbReceive(m_buffer, sizeof(m_buffer), timeout, &expected, 1);
  if (ret) {
    return true;
  } else {
    return false;
  }
}

bool ClientSIM800::getLocation(NET_LOCATION* loc)
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

bool UDPClientSIM800::open(const char* host, uint16_t port)
{
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
    xbWrite(data, len);
    xbWrite("\r", 1);
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
  if (sendCommand("AT+CIPUDPMODE?\r", timeout, "RECV FROM:")) {
		return checkIncoming(pbytes);
  }
  return 0;
}

char* UDPClientSIM800::checkIncoming(int* pbytes)
{
	char *p = strstr(m_buffer, "RECV FROM:");
	if (p) {
    *p = '-'; // mark this datagram as checked
    p = strchr(p, '\n');
    if (p) {
      if (pbytes) *pbytes = strlen(p);
      return p + 1;
    }
  }
  return 0;
}

bool HTTPClientSIM800::open(const char* host, uint16_t port)
{
  if (!host) {
    close();
    return sendCommand("AT+HTTPINIT\r");
  }
  m_host = host;
  m_port = port;
  m_state = HTTP_CONNECTED;
  return true;
}

bool HTTPClientSIM800::send(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload, int payloadSize)
{
  sendCommand("AT+HTTPPARA = \"CID\",1\r");
  sprintf(m_buffer, "AT+HTTPPARA=\"URL\",\"%s:%u%s\"\r", m_host.c_str(), m_port, path);
  if (!sendCommand(m_buffer)) {  
  } else if (method == METHOD_GET) {
    if (sendCommand("AT+HTTPACTION=0\r", HTTP_CONN_TIMEOUT)) {
      m_state = HTTP_SENT;
      return true;
    }
  } else {
    sprintf(m_buffer, "AT+HTTPDATA=%u,10000\r", payloadSize);
    if (sendCommand(m_buffer)) {
      if (sendCommand("AT+HTTPACTION=1\r", HTTP_CONN_TIMEOUT))
        m_state = HTTP_SENT;
        return true;
    }
  }
  m_state = HTTP_ERROR;
  Serial.println(m_buffer);
  return false;
}

void HTTPClientSIM800::close()
{
  sendCommand("AT+HTTPTERM\r");
  m_state = HTTP_DISCONNECTED;
}

char* HTTPClientSIM800::receive(int* pbytes, unsigned int timeout)
{
  char *p = strstr(m_buffer, "+HTTPACTION:");
  if (!p) {
    if (!sendCommand(0, timeout, "+HTTPACTION")) return 0;
  }
  if (sendCommand("AT+HTTPREAD\r", 1000)) {
    Serial.println(m_buffer);
    p = strstr(m_buffer, "+HTTPREAD: ");
    if (p) {
      p += 11;
      int bytes = atoi(p);
      p = strchr(p, '\n');
      if (p++) {
        p[bytes] = 0;
        if (pbytes) *pbytes = bytes;
        return p;
      }
    }
  }
  m_state = HTTP_ERROR;
  return 0;  
}

/*******************************************************************************
  Implementation for SIM5360
*******************************************************************************/

bool ClientSIM5360::begin()
{
  for (byte n = 0; n < 3; n++) {
    // try turning on module
    xbTogglePower();
    delay(3000);
    // discard any stale data
    xbPurge();
    for (byte m = 0; m < 5; m++) {
      if (sendCommand("AT\r") && sendCommand("ATE0\r") && sendCommand("ATI\r")) {
        // retrieve module info
        char *p = strstr(m_buffer, "Model:");
        if (p) p = strchr(p, '_');
        if (p++) {
          int i = 0;
          while (i < sizeof(m_model) - 1 && p[i] && p[i] != '\r' && p[i] != '\n') {
            m_model[i] = p[i];
            i++;
          }
          m_model[i] = 0;
        }
        p = strstr(m_buffer, "IMEI:");
        if (p) strncpy(IMEI, p + 6, sizeof(IMEI) - 1);
        return true;
      }
    }
  }
  return false;
}

void ClientSIM5360::end()
{
  setGPS(false);
  sendCommand("AT+CPOF\r");
}

bool ClientSIM5360::setup(const char* apn, unsigned int timeout)
{
  uint32_t t = millis();
  bool success = false;
  //sendCommand("AT+CNMP=13\r"); // GSM only
  //sendCommand("AT+CNMP=14\r"); // WCDMA only
  do {
    do {
      success = sendCommand("AT+CPSI?\r", 1000, "Online");
      if (strstr(m_buffer, "Off")) {
        success = false;
        break;
      }
      if (success) {
        if (!strstr(m_buffer, "NO SERVICE"))
          break;
        success = false;
      }
    } while (millis() - t < timeout);
    if (!success) break;

    success = false;
    do {
      if (sendCommand("AT+CREG?\r", 1000, "+CREG: 0,")) {
        char *p = strstr(m_buffer, "+CREG: 0,");
        success = (p && (*(p + 9) == '1' || *(p + 9) == '5'));
      }
    } while (!success && millis() - t < timeout);
    if (!success) break;

    success = false;
    do {
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
  if (!success) Serial.println(m_buffer);
  return success;
}

bool ClientSIM5360::setGPS(bool on)
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
      sendCommand("AT+CGPS=0\r");
    }
    return false;
  } else {
    sendCommand("AT+CVAUXS=0\r");
    sendCommand("AT+CGPS=0\r");
    if (m_gps) {
      delete m_gps;
      m_gps = 0;
    }
    return true;
  }
}

String ClientSIM5360::getIP()
{
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
  return "";
}

int ClientSIM5360::getSignal()
{
  if (sendCommand("AT+CSQ\r", 500)) {
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

String ClientSIM5360::getOperatorName()
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

bool ClientSIM5360::checkSIM()
{
  bool success;
  for (byte n = 0; n < 10 && !(success = sendCommand("AT+CPIN?\r", 500, ": READY")); n++);
  return success;  
}

String ClientSIM5360::queryIP(const char* host)
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

bool ClientSIM5360::sendCommand(const char* cmd, unsigned int timeout, const char* expected)
{
  if (cmd) {
    xbWrite(cmd);
  }
  m_buffer[0] = 0;
  byte ret = xbReceive(m_buffer, sizeof(m_buffer), timeout, &expected, 1);
  if (ret) {
    return true;
  } else {
    return false;
  }
}

float ClientSIM5360::parseDegree(const char* s)
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

void ClientSIM5360::checkGPS()
{
  // check and parse GPS data
  char *p;
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

bool UDPClientSIM5360::open(const char* host, uint16_t port)
{
  sendCommand("AT+NETOPEN\r");
  if (host) {
    udpIP = queryIP(host);
    if (!udpIP.length()) {
      udpIP = host;
    }
    udpPort = port;
  }
  sprintf(m_buffer, "AT+CIPOPEN=0,\"UDP\",\"%s\",%u,8000\r", udpIP.c_str(), udpPort);
  if (!sendCommand(m_buffer, 3000)) {
    close();
    Serial.println(m_buffer);
    return false;
  }
  return true;
}

void UDPClientSIM5360::close()
{
  sendCommand("AT+CIPCLOSE=0\r");
  sendCommand("AT+NETCLOSE\r");
}

bool UDPClientSIM5360::send(const char* data, unsigned int len)
{
  int n = sprintf(m_buffer, "AT+CIPSEND=0,%u,\"%s\",%u\r", len, udpIP.c_str(), udpPort);
  xbWrite(m_buffer, n);
  delay(10);
  xbWrite(data, len);
  if (sendCommand(0, 500)) return true;
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

bool HTTPClientSIM5360::open(const char* host, uint16_t port)
{
  if (!host) {
    close();
    for (int i = 0; i < 30; i++) {
      sendCommand("AT+CHTTPSSTOP\r");
      if (sendCommand("AT+CHTTPSSTART\r")) {
        return true;
      }
    }
    return false;
  }
  sprintf(m_buffer, "AT+CHTTPSOPSE=\"%s\",%u,%u\r", host, port, port == 443 ? 2: 1);
  if (sendCommand(m_buffer, HTTP_CONN_TIMEOUT)) {
    m_state = HTTP_CONNECTED;
    m_host = host;
    return true;
  }
  Serial.println(m_buffer);
  m_state = HTTP_ERROR;
  return false;
}

void HTTPClientSIM5360::close()
{
  sendCommand("AT+CHTTPSCLSE\r", 1000, "+CHTTPSCLSE:");
  m_state = HTTP_DISCONNECTED;
}

bool HTTPClientSIM5360::send(HTTP_METHOD method, const char* path, bool keepAlive, const char* payload, int payloadSize)
{
  String header = genHeader(method, path, keepAlive, payload, payloadSize);
  int len = header.length();
  sprintf(m_buffer, "AT+CHTTPSSEND=%u\r", len + payloadSize);
  if (!sendCommand(m_buffer, 100, ">")) {
    m_state = HTTP_DISCONNECTED;
    return false;
  }
  // send HTTP header
  xbWrite(header.c_str());
  // send POST payload if any
  if (method == METHOD_POST && payload) xbWrite(payload);
  if (sendCommand(0, 200)) {
    m_state = HTTP_SENT;
    return true;
  }
  //Serial.println(m_buffer);
  m_state = HTTP_ERROR;
  return false;
}

char* HTTPClientSIM5360::receive(int* pbytes, unsigned int timeout)
{
  // start receiving
  int received = 0;
  char* payload = 0;

  // wait for RECV EVENT
  if (!sendCommand(0, timeout, "\r\n+CHTTPS: RECV EVENT")) {
    checkGPS();
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
  sprintf(m_buffer, "AT+CHTTPSRECV=%u\r", sizeof(m_buffer) - 36);
  if (sendCommand(m_buffer, timeout, "\r\n+CHTTPSRECV: 0")) {
    char *p = strstr(m_buffer, "\r\n+CHTTPSRECV: DATA");
    if (p) {
      if ((p = strchr(p, ','))) {
        received = atoi(p + 1);
        char *q = strchr(p, '\n');
        payload = q ? (q + 1) : p;
        if (m_buffer + sizeof(m_buffer) - payload > received) {
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

void ClientSIM7600::end()
{
  sendCommand("AT+CRESET\r");
  setGPS(false);
  delay(1000);
  sendCommand("AT+CPOF\r");
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
      success = sendCommand("AT+CPSI?\r", 1000, "Online");
      if (strstr(m_buffer, "Off")) {
        success = false;
        break;
      }
      if (success) {
        if (!strstr(m_buffer, "NO SERVICE"))
          break;
        success = false;
      }
    } while (millis() - t < timeout);
    if (!success) break;

    success = false;
    do {
      if (sendCommand("AT+CREG?\r", 1000, "+CREG: 0,")) {
        char *p = strstr(m_buffer, "+CREG: 0,");
        success = (p && (*(p + 9) == '1' || *(p + 9) == '5'));
      }
    } while (!success && millis() - t < timeout);
    if (!success) break;

    success = false;
    do {
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
      sendCommand("AT+CGPS=0\r");
    }
    return false;
  } else {
    sendCommand("AT+CVAUXS=0\r");
    sendCommand("AT+CGPS=0\r");
    if (m_gps) {
      delete m_gps;
      m_gps = 0;
    }
    return true;
  }
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
  sprintf(m_buffer, "AT+CIPOPEN=0,\"UDP\",\"%s\",%u,8000\r", udpIP.c_str(), udpPort);
  if (!sendCommand(m_buffer, 3000)) {
    close();
    Serial.println(m_buffer);
    return false;
  }
  return true;
}

void UDPClientSIM7600::close()
{
  sendCommand("AT+CIPCLOSE=0\r");
}

bool UDPClientSIM7600::send(const char* data, unsigned int len)
{
  int n = sprintf(m_buffer, "AT+CIPSEND=0,%u,\"%s\",%u\r", len, udpIP.c_str(), udpPort);
  xbWrite(m_buffer, n);
  delay(10);
  xbWrite(data, len);
  if (sendCommand(0, 500)) return true;
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
    for (int i = 0; i < 30; i++) {
      sendCommand("AT+CHTTPSSTOP\r");
      if (sendCommand("AT+CHTTPSSTART\r", 1000, "+CHTTPSSTART: 0")) {
        return true;
      }
    }
    return false;
  }

  sprintf(m_buffer, "AT+CHTTPSOPSE=\"%s\",%u,%u\r", host, port, port == 443 ? 2: 1);
  if (sendCommand(m_buffer, 1000)) {
    if (sendCommand(0, 10000, "+CHTTPSOPSE: 0")) {
      m_state = HTTP_CONNECTED;
      m_host = host;
      return true;
    }
  }
  Serial.println(m_buffer);
  m_state = HTTP_ERROR;
  return false;
}

void HTTPClientSIM7600::close()
{
  sendCommand("AT+CHTTPSCLSE\r", 1000, "+CHTTPSCLSE:");
  m_state = HTTP_DISCONNECTED;
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
  xbWrite(header.c_str());
  // send POST payload if any
  if (payload) xbWrite(payload);
  if (sendCommand(0, 200, "+CHTTPSSEND: 0")) {
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

  // wait for RECV EVENT
  if (!sendCommand(0, timeout, "\r\n+CHTTPS: RECV EVENT")) {
    checkGPS();
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
  sprintf(m_buffer, "AT+CHTTPSRECV=%u\r", sizeof(m_buffer) - 36);
  if (sendCommand(m_buffer, timeout, "\r\n+CHTTPSRECV: 0")) {
    char *p = strstr(m_buffer, "\r\n+CHTTPSRECV: DATA");
    if (p) {
      if ((p = strchr(p, ','))) {
        received = atoi(p + 1);
        char *q = strchr(p, '\n');
        payload = q ? (q + 1) : p;
        if (m_buffer + sizeof(m_buffer) - payload > received) {
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