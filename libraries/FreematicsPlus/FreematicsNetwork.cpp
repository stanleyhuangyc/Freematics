#include <Arduino.h>
#include "FreematicsONE.h"
#include "FreematicsNetwork.h"

bool CTeleClientSIM5360::netInit()
{
  for (byte n = 0; n < 10; n++) {
    // try turning on module
    xbTogglePower();
    sleep(3000);
    // discard any stale data
    xbPurge();
    for (byte m = 0; m < 3; m++) {
      if (netSendCommand("AT\r"))
        return true;
    }
  }
  return false;
}

bool CTeleClientSIM5360::netSetup(const char* apn, bool only3G)
{
  uint32_t t = millis();
  bool success = false;
  netSendCommand("ATE0\r");
  if (only3G) netSendCommand("AT+CNMP=14\r"); // use WCDMA only
  do {
    do {
      Serial.print('.');
      sleep(500);
      success = netSendCommand("AT+CPSI?\r", 1000, "Online");
      if (success) {
        if (!strstr_P(m_buffer, PSTR("NO SERVICE")))
          break;
        success = false;
      } else {
        if (strstr_P(m_buffer, PSTR("Off"))) break;
      }
    } while (millis() - t < 60000);
    if (!success) break;

    t = millis();
    do {
      success = netSendCommand("AT+CREG?\r", 5000, "+CREG: 0,1");
    } while (!success && millis() - t < 30000);
    if (!success) break;

    do {
      success = netSendCommand("AT+CGREG?\r",1000, "+CGREG: 0,1");
    } while (!success && millis() - t < 30000);
    if (!success) break;

    do {
      sprintf_P(m_buffer, PSTR("AT+CGSOCKCONT=1,\"IP\",\"%s\"\r"), apn);
      success = netSendCommand(m_buffer);
    } while (!success && millis() - t < 30000);
    if (!success) break;

    success = netSendCommand("AT+CSOCKSETPN=1\r");
    if (!success) break;

    success = netSendCommand("AT+CIPMODE=0\r");
    if (!success) break;

    netSendCommand("AT+NETOPEN\r");
  } while(0);
  if (!success) Serial.println(m_buffer);
  return success;
}

const char* CTeleClientSIM5360::getIP()
{
  uint32_t t = millis();
  char *ip = 0;
  do {
    if (netSendCommand("AT+IPADDR\r", 5000, "\r\nOK\r\n", true)) {
      char *p = strstr(m_buffer, "+IPADDR:");
      if (p) {
        ip = p + 9;
        if (*ip != '0') {
          break;
        }
      }
    }
    sleep(500);
    ip = 0;
  } while (millis() - t < 15000);
  return ip;
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
char* CTeleClientSIM5360::getOperatorName()
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
    return 0;
}

bool CTeleClientSIM5360::udpOpen(const char* host, uint16_t port)
{
  if (host) {
    char *ip = queryIP(host);
    if (ip) {
      Serial.println(ip);
      strncpy(udpIP, ip, sizeof(udpIP) - 1);
    } else {
      return false;
    }
    udpPort = port;
  }
  sprintf_P(m_buffer, PSTR("AT+CIPOPEN=0,\"UDP\",\"%s\",%u,8000\r"), udpIP, udpPort);
  return netSendCommand(m_buffer, 3000);
}

void CTeleClientSIM5360::udpClose()
{
  netSendCommand("AT+CIPCLOSE\r");
}

bool CTeleClientSIM5360::udpSend(const char* data, unsigned int len, bool wait)
{
  sprintf_P(m_buffer, PSTR("AT+CIPSEND=0,%u,\"%s\",%u\r"), len, udpIP, udpPort);
  if (netSendCommand(m_buffer, 100, ">")) {
    xbWrite(data, len);
    if (!wait) return true;
    return waitCompletion(1000);
  } else {
    Serial.println("UDP data unsent");
  }
  return false;
}

char* CTeleClientSIM5360::udpReceive(int* pbytes)
{
  if (netSendCommand(0, 3000, "+IPD")) {
    char *p = strstr(m_buffer, "+IPD");
    if (!p) return 0;
    int len = atoi(p + 4);
    if (pbytes) *pbytes = len;
    p = strchr(p, '\n');
    if (p) {
      *(++p + len) = 0;
      return p;
    }
  }
  return 0;
}

bool CTeleClientSIM5360::waitCompletion(int timeout)
{
  return netSendCommand(0, timeout);
}

char* CTeleClientSIM5360::queryIP(const char* host)
{
  sprintf_P(m_buffer, PSTR("AT+CDNSGIP=\"%s\"\r"), host);
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
  return 0;
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

bool CTeleClient::verifyChecksum(const char* data)
{
  uint8_t sum = 0;
  const char *s;
  for (s = data; *s && *s != '*'; s++) sum += *s;
  return (*s && hex2uint8(s + 1) == sum);
}

bool CTeleClient::notifyUDP(byte event, const char* serverKey, const char* vin)
{
  char payload[64];
  int bytes = sprintf_P(payload, PSTR("%X#EV=%u,SK=%s,TS=%lu"), feedid, (unsigned int)event, serverKey, millis());
  if (event == EVENT_LOGIN) {
    /*
    // load DTC
    uint16_t dtc[6];
    byte dtcCount = readDTC(dtc, sizeof(dtc) / sizeof(dtc[0]));
    if (dtcCount > 0) {
      Serial.print("#DTC:");
      Serial.println(dtcCount);
      bytes += sprintf_P(payload + bytes, PSTR(",DTC="), dtcCount);
      for (byte i = 0; i < dtcCount; i++) {
        bytes += sprintf_P(payload + bytes, PSTR("%X;"), dtc[i]);
      }
      bytes--;
    }
    */
    bytes += sprintf_P(payload + bytes, PSTR(",VIN=%s"), vin);
    /*
    if (getVIN(payload + bytes, sizeof(payload) - bytes)) {
      Serial.print("#VIN:");
      Serial.println(payload + bytes);
      bytes += strlen(payload + bytes);
    } else {
      bytes += sprintf_P(payload + bytes, PSTR("DEFAULT_VIN"));
    }
    */
  }
  waiting = false;
  for (byte attempts = 0; attempts < 3; attempts++) {
    transmitUDP(true);
    // receive reply
    int len;
    char *data = udpReceive(&len);
    if (!data) {
      Serial.println("No reply");
      continue;
    }
    connErrors = 0;
    // verify checksum
    if (!verifyChecksum(data)) {
      Serial.println("Wrong checksum");
      continue;
    }
    char pattern[16];
    sprintf_P(pattern, PSTR("EV=%u"), event);
    if (!strstr(data, pattern)) {
      Serial.println("Invalid reply");
      continue;
    }
    if (event == EVENT_LOGIN) {
      feedid = atoi(data);
      Serial.print("#FEED ID:");
      Serial.println(feedid);
      addHeader(feedid); // for first transmission
    }
    return true;
  }
  return false;
}
void CTeleClient::transmitUDP(bool wait)
{
  // add checksum
  addTail();

  if (waiting) {
    // wait for last data to be sent
    if (!waitCompletion(100)) {
      connErrors++;
      //Serial.println(m_buffer);
    } else {
      connErrors = 0;
      txCount++;
    }
    waiting = false;
  }

  // transmit data
  if (!udpSend(getCache(), getCacheBytes(), wait)) {
    connErrors++;
    // remove tail for re-sending
    removeTail();
  } else {
    if (!wait) {
      waiting = true;
    } else {
      connErrors = 0;
      txCount++;
    }
    // clear cache and place header
    addHeader(feedid);
  }
}
bool CTeleClient::reconnect()
{
    udpClose();
    udpOpen(0, 0);
    //return notifyUDP(EVENT_LOGIN);
    return true;
}
void CTeleClient::showStats()
{
  Serial.print('#');
  Serial.print(txCount);
  Serial.print(' ');
  Serial.print(getCacheBytes());
  Serial.print(" bytes ");
  Serial.print(millis() / (txCount + 1));
  Serial.println("ms");
  // output data in cache
  //Serial.println(cache);
}
