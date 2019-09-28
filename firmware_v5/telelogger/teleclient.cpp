/******************************************************************************
* Freematics Hub client and Traccar client implementations
* Works with Freematics ONE+
* Developed by Stanley Huang <stanley@freematics.com.au>
* Distributed under BSD license
* Visit https://freematics.com/products for hardware information
* Visit https://hub.freematics.com to view live and history telemetry data
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
******************************************************************************/

#include <FreematicsPlus.h>
#include "telelogger.h"
#include "telemesh.h"
#include "teleclient.h"

bool processCommand(char* data);

extern int16_t rssi;
extern char devid[];
extern char vin[];
extern GPS_DATA* gd;
extern char isoTime[];

CBuffer::CBuffer()
{
  purge();
}

void CBuffer::add(uint16_t pid, int value)
{
  if (offset < BUFFER_LENGTH - sizeof(uint16_t) - sizeof(int)) {
    setType(ELEMENT_INT);
    *(uint16_t*)(data + offset) = pid;
    offset += 2;
    *(int*)(data + offset) = value;
    offset += sizeof(int);
    count++;
  } else {
    Serial.println("FULL");
  }
}
void CBuffer::add(uint16_t pid, uint32_t value)
{
  if (offset < BUFFER_LENGTH - sizeof(uint16_t) - sizeof(uint32_t)) {
    setType(ELEMENT_UINT);
    *(uint16_t*)(data + offset) = pid;
    offset += 2;
    *(uint32_t*)(data + offset) = value;
    offset += sizeof(uint32_t);
    count++;
  } else {
    Serial.println("FULL");
  }
}
void CBuffer::add(uint16_t pid, float value)
{
  if (offset < BUFFER_LENGTH - sizeof(uint16_t) - sizeof(float)) {
    setType(ELEMENT_FLOAT);
    *(uint16_t*)(data + offset) = pid;
    offset += 2;
    *(float*)(data + offset) = value;
    offset += sizeof(float);
    count++;
  } else {
    Serial.println("FULL");
  }
}
void CBuffer::add(uint16_t pid, float value[])
{
  if (offset < BUFFER_LENGTH - sizeof(uint16_t) + sizeof(float) * 3) {
    setType(ELEMENT_FLOATX3);
    *(uint16_t*)(data + offset) = pid;
    offset += 2;
    memcpy(data + offset, value, sizeof(float) * 3);
    offset += sizeof(float) * 3;
    count++;
  } else {
    Serial.println("FULL");
  }
}

void CBuffer::purge()
{
  state = BUFFER_STATE_EMPTY;
  timestamp = 0;
  offset = 0;
  count = 0;
  memset(types, 0, sizeof(types));
}

void CBuffer::setType(uint32_t dataType)
{
  types[count / 16] |= (dataType << ((count % 16) * 2));
}

void CBuffer::serialize(CStorage& store)
{
  int of = 0;
  for (int n = 0; n < count; n++) {
    uint16_t pid = *(uint16_t*)(data + of);
    of += sizeof(uint16_t);
    switch ((types[n / 16] >> ((n % 16) * 2)) & 0x3) {
    case ELEMENT_INT:
      {
        int value = *(int*)(data + of);
        of += sizeof(value);
        store.log(pid, value);
      }
      break;
    case ELEMENT_UINT:
      {
        uint32_t value = *(uint32_t*)(data + of);
        of += sizeof(value);
        store.log(pid, value);
      }
      break;
    case ELEMENT_FLOAT:
      {
        float value = *(float*)(data + of);
        of += sizeof(value);
        store.log(pid, value);
      }
      break;
    case ELEMENT_FLOATX3:
      {
        float value[3];
        memcpy(value, data + of, sizeof(value));
        of += sizeof(value);
        store.log(pid, value);
      }
      break;
    }
  }
}

bool TeleClientUDP::verifyChecksum(char* data)
{
  uint8_t sum = 0;
  char *s = strrchr(data, '*');
  if (!s) return false;
  for (char *p = data; p < s; p++) sum += *p;
  if (hex2uint8(s + 1) == sum) {
    *s = 0;
    return true;
  }
  return false;
}

bool TeleClientUDP::notify(byte event, const char* payload)
{
  char buf[48];
  CStorageRAM netbuf;
  netbuf.init(128);
  netbuf.header(devid);
  netbuf.dispatch(buf, sprintf(buf, "EV=%X", (unsigned int)event));
  netbuf.dispatch(buf, sprintf(buf, "TS=%lu", millis()));
  netbuf.dispatch(buf, sprintf(buf, "ID=%s", devid));
  if (rssi) {
    netbuf.dispatch(buf, sprintf(buf, "SSI=%d", (int)rssi));
  }
  if (vin[0]) {
    netbuf.dispatch(buf, sprintf(buf, "VIN=%s", vin));
  }
  if (payload) {
    netbuf.dispatch(payload, strlen(payload));
  }
  netbuf.tailer();
  //Serial.println(netbuf.buffer());
  for (byte attempts = 0; attempts < 3; attempts++) {
    // send notification datagram
    //Serial.println(netbuf.buffer());
    if (!net.send(netbuf.buffer(), netbuf.length())) {
      // error sending data
      break;
    }
#if NET_DEVICE != NET_SERIAL && NET_DEVICE != NET_WIFI_MESH
    if (event == EVENT_ACK) return true; // no reply for ACK
    char *data = 0;
    // receive reply
    uint32_t t = millis();
    do {
      if ((data = net.receive())) break;
      // no reply yet
      delay(100);
    } while (millis() - t < DATA_RECEIVING_TIMEOUT);
    if (!data) {
      //Serial.println("Timeout");
      continue;
    }
    // verify checksum
    if (!verifyChecksum(data)) {
      Serial.print("Checksum mismatch:");
      Serial.println(data);
      continue;
    }
    char pattern[16];
    sprintf(pattern, "EV=%u", event);
    if (!strstr(data, pattern)) {
      Serial.println("Invalid reply");
      continue;
    }
    if (event == EVENT_LOGIN) {
      // extract info from server response
      char *p = strstr(data, "TM=");
      if (p) {
        // set local time from server
        unsigned long tm = atol(p + 3);
        struct timeval tv = { .tv_sec = (time_t)tm, .tv_usec = 0 };
        settimeofday(&tv, NULL);
      }
      p = strstr(data, "SN=");
      if (p) {
        char *q = strchr(p, ',');
        if (q) *q = 0;
      }
      feedid = hex2uint16(data);
      login = true;
    } else if (event == EVENT_LOGOUT) {
      login = false;
    }
#endif
    // success
    return true;
  }
  return false;
}

bool TeleClientUDP::connect()
{
  byte event = login ? EVENT_RECONNECT : EVENT_LOGIN;
  bool success = false;
  // connect to telematics server
  for (byte attempts = 0; attempts < 5; attempts++) {
    Serial.print(event == EVENT_LOGIN ? "LOGIN(" : "RECONNECT(");
    Serial.print(SERVER_HOST);
    Serial.print(':');
    Serial.print(SERVER_PORT);
    Serial.println(")...");
    if (!net.open(SERVER_HOST, SERVER_PORT)) {
      Serial.println("Unable to connect");
      delay(1000);
      continue;
    }
    // log in or reconnect to Freematics Hub
    if (!notify(event)) {
      net.close();
      Serial.println("Server timeout");
      delay(1000);
      continue;
    }
    success = true;
    break;
  }
  startTime = millis();
  if (success) {
    lastSyncTime = startTime;
  }
  return success;
}

bool TeleClientUDP::ping()
{
  bool success = false;
  for (byte n = 0; n < 2 && !success; n++) {
    success = net.open(SERVER_HOST, SERVER_PORT);
    if (success) success = notify(EVENT_PING);
  }
  if (success) lastSyncTime = millis();
  return success;
}

bool TeleClientUDP::transmit(const char* packetBuffer, unsigned int packetSize)
{
  inbound();
  // transmit data
  if (net.send(packetBuffer, packetSize)) {
    txBytes += packetSize;
    txCount++;
    return true;
  }
  return false;
}

void TeleClientUDP::inbound()
{
  // check incoming datagram
  do {
    int len = 0;
    char *data = net.receive(&len, 0);
    if (!data) break;
    data[len] = 0;
    rxBytes += len;
    if (!verifyChecksum(data)) {
      Serial.print("Checksum mismatch:");
      Serial.println(data);
      break;
    }
    char *p = strstr(data, "EV=");
    if (!p) break;
    int eventID = atoi(p + 3);
    switch (eventID) {
    case EVENT_COMMAND:
      processCommand(data);
      break;
    case EVENT_SYNC:
        {
          uint16_t id = hex2uint16(data);
          if (id && id != feedid) {
            feedid = id;
            Serial.print("FEED ID:");
            Serial.println(feedid);
          }
        }
        break;
    }
    lastSyncTime = millis();
  } while(0);
}

void TeleClientUDP::shutdown()
{
  if (login) {
    notify(EVENT_LOGOUT);
    login = false;
  }
  net.close();
  net.end();
  Serial.print(net.deviceName());
  Serial.println(" OFF");
}

#if NET_DEVICE == NET_WIFI || NET_DEVICE == NET_SIM800 || NET_DEVICE == NET_SIM5360 || NET_DEVICE == NET_SIM7600

bool TeleClientHTTP::notify(byte event, const char* payload)
{
  char url[256];
  snprintf(url, sizeof(url), "%s/notify/%s?EV=%u&SSI=%d&VIN=%s", SERVER_PATH, devid,
    (unsigned int)event, (int)rssi, vin);
  if (event == EVENT_LOGOUT) login = false;
  return net.send(METHOD_GET, url, true) && net.receive();
}

bool TeleClientHTTP::transmit(const char* packetBuffer, unsigned int packetSize)
{
  if (net.state() != HTTP_CONNECTED) {
    // reconnect if disconnected
    if (!connect()) {
      return false;
    }
  }

  char url[256];
  bool success;
  int len;
#if SERVER_METHOD == PROTOCOL_METHOD_GET
  if (gd && gd->ts) {
    len = snprintf(url, sizeof(url), "%s/push?id=%s&timestamp=%s&lat=%f&lon=%f&altitude=%d&speed=%f&heading=%d",
      SERVER_PATH, devid, isoTime,
      gd->lat, gd->lng, (int)gd->alt, gd->speed, (int)gd->heading);
  } else {
    len = snprintf(url, sizeof(url), "%s/push?id=%s", SERVER_PATH, devid);
  }
  success = net.send(METHOD_GET, url, true);
#else
  len = snprintf(url, sizeof(url), "%s/post/%s", SERVER_PATH, devid);
  success = net.send(METHOD_POST, url, true, packetBuffer, packetSize);
  len += packetSize;
#endif
  if (!success) {
    Serial.println("Connection closed");
    net.close();
    return false;
  } else {
    txBytes += len;
    txCount++;
  }

  // check response
  int bytes = 0;
  char* response = net.receive(&bytes);
  if (!response) {
    // close connection on receiving timeout
    Serial.println("No HTTP response");
    net.close();
    return false;
  }
  Serial.println(response);
  if (strstr(response, " 200 ")) {
    // successful
    lastSyncTime = millis();
    rxBytes += bytes;
  }
  return true;
}

bool TeleClientHTTP::connect()
{
  if (!started) {
    started = net.open();
    Serial.println("HTTPS stack started");
  }

  // connect to HTTP server
  if (!net.open(SERVER_HOST, SERVER_PORT)) {
    Serial.println("Error connecting to server");
    return false;
  }

  if (!login) {
    Serial.print("LOGIN(");
    Serial.print(SERVER_HOST);
    Serial.print(':');
    Serial.print(SERVER_PORT);
    Serial.println(")...");
    // log in or reconnect to Freematics Hub
    if (notify(EVENT_LOGIN)) {
      lastSyncTime = millis();
      login = true;
    }
  }
  return true;
}

bool TeleClientHTTP::ping()
{
  return connect();
}

void TeleClientHTTP::shutdown()
{
  Serial.print(net.deviceName());
  if (login) {
    notify(EVENT_LOGOUT);
    login = false;
  }
  net.close();
  net.end();
  Serial.println(" OFF");
  started = false;
}

#endif