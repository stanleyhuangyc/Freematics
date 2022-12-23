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
#include "telestore.h"
#include "telemesh.h"
#include "teleclient.h"

extern int16_t rssi;
extern char devid[];
extern char vin[];
extern GPS_DATA* gd;
extern char isoTime[];

CBuffer::CBuffer()
{
#if HAS_LARGE_RAM
  data = (uint8_t*)heap_caps_malloc(BUFFER_LENGTH, MALLOC_CAP_SPIRAM);
#else
  data = (uint8_t*)malloc(BUFFER_LENGTH);
#endif
  purge();
}

void CBuffer::add(uint16_t pid, uint8_t type, void* values, int bytes, uint8_t count)
{
  if (offset < BUFFER_LENGTH - sizeof(ELEMENT_HEAD) - bytes) {
    ELEMENT_HEAD hdr = {pid, type, count};
    *(ELEMENT_HEAD*)(data + offset) = hdr;
    offset += sizeof(ELEMENT_HEAD);
    memcpy(data + offset, values, bytes);
    offset += bytes;
    total++;
  } else {
    Serial.println("FULL");
  }
}

void CBuffer::purge()
{
  state = BUFFER_STATE_EMPTY;
  timestamp = 0;
  offset = 0;
  total = 0;
}

void CBuffer::serialize(CStorage& store)
{
  uint16_t of = 0;
  for (int n = 0; n < total && of < offset; n++) {
    ELEMENT_HEAD* hdr = (ELEMENT_HEAD*)(data + of);
    of += sizeof(ELEMENT_HEAD);
    switch (hdr->type) {
    case ELEMENT_UINT8:
      store.log(hdr->pid, (uint8_t*)(data + of), hdr->count);
      of += (uint16_t)hdr->count * sizeof(uint8_t);
      break;
    case ELEMENT_UINT16:
      store.log(hdr->pid, (uint16_t*)(data + of), hdr->count);
      of += (uint16_t)hdr->count * sizeof(uint16_t);
      break;
    case ELEMENT_UINT32:
      store.log(hdr->pid, (uint32_t*)(data + of), hdr->count);
      of += (uint16_t)hdr->count * sizeof(uint32_t);
      break;
    case ELEMENT_INT32:
      store.log(hdr->pid, (int32_t*)(data + of), hdr->count);
      of += (uint16_t)hdr->count * sizeof(int32_t);
      break;
    case ELEMENT_FLOAT:
      store.log(hdr->pid, (float*)(data + of), hdr->count);
      of += (uint16_t)hdr->count * sizeof(float);
      break;
    case ELEMENT_FLOAT_D1:
      store.log(hdr->pid, (float*)(data + of), hdr->count, "%.1f");
      of += (uint16_t)hdr->count * sizeof(float);
      break;
    case ELEMENT_FLOAT_D2:
      store.log(hdr->pid, (float*)(data + of), hdr->count, "%.2f");
      of += (uint16_t)hdr->count * sizeof(float);
      break;
    default:
      return;
    }
  }
}

void CBufferManager::init()
{
  for (int n = 0; n < BUFFER_SLOTS; n++) {
      slots[n] = new CBuffer();
  }
}

void CBufferManager::purge()
{
  for (int n = 0; n < BUFFER_SLOTS; n++) slots[n]->purge();
}

CBuffer* CBufferManager::getFree()
{
  if (last) {
    CBuffer* slot = last;
    last = 0;
    if (slot->state == BUFFER_STATE_EMPTY) return slot;
  }
  uint32_t ts = 0xffffffff;
  int m = 0;
  // search for free slot, if none, mark the oldest one
  for (int n = 0; n < BUFFER_SLOTS; n++) {
    if (slots[n]->state == BUFFER_STATE_EMPTY) {
      return slots[n];
    } else if (slots[n]->state == BUFFER_STATE_FILLED && slots[n]->timestamp < ts) {
        m = n;
        ts = slots[n]->timestamp;
    }
  }
  // dispose oldest data when buffer is full
  while (slots[m]->state == BUFFER_STATE_LOCKED) delay(1);
  slots[m]->purge();
  return slots[m];
}

CBuffer* CBufferManager::getOldest()
{
  uint32_t ts = 0xffffffff;
  int m = -1;
  for (int n = 0; n < BUFFER_SLOTS; n++) {
    if (slots[n]->state == BUFFER_STATE_FILLED && slots[n]->timestamp < ts) {
        m = n;
        ts = slots[n]->timestamp;
    }
  }
  if (m >= 0) {
    slots[m]->state = BUFFER_STATE_LOCKED;
    return slots[m];
  }
  return 0;
}

CBuffer* CBufferManager::getNewest()
{
  uint32_t ts = 0;
  int m = -1;
  for (int n = 0; n < BUFFER_SLOTS; n++) {
    if (slots[n]->state == BUFFER_STATE_FILLED && slots[n]->timestamp > ts) {
      m = n;
      ts = slots[n]->timestamp;
    }
  }
  if (m >= 0) {
    slots[m]->state = BUFFER_STATE_LOCKED;
    return slots[m];
  }
  return 0;
}

void CBufferManager::free(CBuffer* slot)
{
  slot->purge();
  last = slot;  
}

void CBufferManager::printStats()
{
  int bytes = 0;
  int count = 0;
  int samples = 0;
  for (int n = 0; n < BUFFER_SLOTS; n++) {
    if (slots[n]->state != BUFFER_STATE_FILLED) continue;
    bytes += slots[n]->offset;
    samples += slots[n]->total;
    count++;
  }
  if (slots) {
    Serial.print("[BUF] ");
    Serial.print(samples);
    Serial.print(" samples | ");
    Serial.print(bytes);
    Serial.print(" bytes | ");
    Serial.print(count);
    Serial.print('/');
    Serial.println(BUFFER_SLOTS);
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
  char cache[128];
  CStorageRAM netbuf;
  netbuf.init(cache, 128);
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
#if ENABLE_WIFI
    if (wifi.connected())
    {
      if (!wifi.send(netbuf.buffer(), netbuf.length())) break;
    }
    else
#endif
    {
      if (!cell.send(netbuf.buffer(), netbuf.length())) break;
    }
    if (event == EVENT_ACK) return true; // no reply for ACK
    char *data = 0;
    int bytesRecv = 0;
    // receive reply
#if ENABLE_WIFI
    if (wifi.connected())
    {
      data = cell.getBuffer();
      bytesRecv = wifi.receive(data, RECV_BUF_SIZE - 1);
      if (bytesRecv > 0) {
        data[bytesRecv] = 0;
      }
    }
    else
#endif
    {
      data = cell.receive(&bytesRecv); 
    }
    if (!data || bytesRecv == 0) {
      Serial.println("[UDP] Timeout");
      continue;
    }
    rxBytes += bytesRecv;
    // verify checksum
    if (!verifyChecksum(data)) {
      Serial.print("[UDP] Checksum mismatch:");
      Serial.println(data);
      continue;
    }
    char pattern[16];
    sprintf(pattern, "EV=%u", event);
    if (!strstr(data, pattern)) {
      Serial.print("[UDP] Invalid reply: ");
      Serial.println(data);
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
    // success
    return true;
  }
  return false;
}

bool TeleClientUDP::connect(bool quick)
{
  byte event = login ? EVENT_RECONNECT : EVENT_LOGIN;
  bool success = false;
#if ENABLE_WIFI
  if (wifi.connected())
  {
    if (quick) return wifi.open(SERVER_HOST, SERVER_PORT);
  }
  else
#endif
  {
    cell.close();
    if (quick) {
      return cell.open(0, 0);
    }
  }

  packets = 0;

  // connect to telematics server
  for (byte attempts = 0; attempts < 3; attempts++) {
    Serial.print(event == EVENT_LOGIN ? "LOGIN(" : "RECONNECT(");
    Serial.print(SERVER_HOST);
    Serial.print(':');
    Serial.print(SERVER_PORT);
    Serial.println(")...");
#if ENABLE_WIFI
    if (wifi.connected())
    {
      if (!wifi.open(SERVER_HOST, SERVER_PORT)) {
        Serial.println("[WIFI] Unable to connect");
        delay(1000);
        continue;
      }
    }
    else
#endif
    {
      if (!cell.open(SERVER_HOST, SERVER_PORT)) {
        if (!cell.check()) break;
        Serial.println("[NET] Unable to connect");
        delay(3000);
        continue;
      }
    }
    // log in or reconnect to Freematics Hub
    if (!notify(event)) {
#if ENABLE_WIFI
      if (wifi.connected())
      {
        wifi.close();
      }
      else
#endif
      {
        if (!cell.check()) break;
        cell.close();
      }
      Serial.println("[NET] Server timeout");
      continue;
    }
    success = true;
    break;
  }
  if (event == EVENT_LOGIN) startTime = millis();
  if (success) {
    lastSyncTime = millis();
  }
  return success;
}

bool TeleClientUDP::ping()
{
  bool success = false;
  for (byte n = 0; n < 3 && !success; n++) {
#if ENABLE_WIFI
    if (wifi.connected())
    {
      success = wifi.open(SERVER_HOST, SERVER_PORT);
    }
    else
#endif
    {
      success = cell.open(SERVER_HOST, SERVER_PORT);
    }
    if (success) {
      if ((success = notify(EVENT_PING))) break;
#if ENABLE_WIFI
      if (wifi.connected())
      {
        wifi.close();
      }
      else
#endif
      {
        cell.close();
      }
      delay(1000);
    }
  }
  if (success) lastSyncTime = millis();
  return success;
}

bool TeleClientUDP::transmit(const char* packetBuffer, unsigned int packetSize)
{
#if ENABLE_WIFI
  // transmit data via wifi
  if (wifi.connected()) {
    if (wifi.send(packetBuffer, packetSize)) {
      txBytes += packetSize;
      txCount++;
      Serial.print("[WIFI] ");
      Serial.print(packetSize);
      Serial.println(" bytes sent");
      return true;  
    }
    return false;
  }
#endif

  // transmit data via cellular
  if (++packets >= 64) {
    cell.close();
    cell.open(0, 0);
    packets = 0;
  }
  Serial.print("[CELL] ");
  Serial.print(packetSize);
  Serial.println(" bytes being sent");
  if (cell.send(packetBuffer, packetSize)) {
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
    char *data = 0;
#if ENABLE_WIFI
    if (wifi.connected())
    {
      data = cell.getBuffer();
      len = wifi.receive(data, RECV_BUF_SIZE - 1, 10);
    }
    else
#endif
    {
      data = cell.receive(&len, 50);
    }
    if (!data || len == 0) break;
    data[len] = 0;
    Serial.print("[UDP] ");
    Serial.println(data);
    rxBytes += len;
    if (!verifyChecksum(data)) {
      Serial.print("[UDP] Checksum mismatch:");
      Serial.println(data);
      break;
    }
    char *p = strstr(data, "EV=");
    if (!p) break;
    int eventID = atoi(p + 3);
    switch (eventID) {
    case EVENT_SYNC:
        feedid = hex2uint16(data);
        Serial.print("[UDP] FEED ID:");
        Serial.println(feedid);
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
    Serial.println("[NET] Logout");
  }
#if ENABLE_WIFI
  if (wifi.connected()) {
    wifi.end();
    Serial.println("[WIFI] Deactivated");
    return;
  }
#endif
  cell.end();
  Serial.println("[CELL] Deactivated");
}

bool TeleClientHTTP::notify(byte event, const char* payload)
{
  char url[256];
  snprintf(url, sizeof(url), "%s/notify/%s?EV=%u&SSI=%d&VIN=%s", SERVER_PATH, devid,
    (unsigned int)event, (int)rssi, vin);
  if (event == EVENT_LOGOUT) login = false;
#if ENABLE_WIFI
  if (wifi.connected())
  {
    return wifi.send(METHOD_GET, url, true) && wifi.receive(cell.getBuffer(), RECV_BUF_SIZE - 1) && wifi.code() == 200;
  }
  else
#endif
  {
    return cell.send(METHOD_GET, url, true) && cell.receive() && cell.code() == 200;
  }
}

bool TeleClientHTTP::transmit(const char* packetBuffer, unsigned int packetSize)
{
#if ENABLE_WIFI
  if ((wifi.connected() && wifi.state() != HTTP_CONNECTED) || cell.state() != HTTP_CONNECTED) {
#else
  if (cell.state() != HTTP_CONNECTED) {
#endif
    // reconnect if disconnected
    if (!connect(true)) {
      return false;
    }
  }

  char url[256];
  bool success = false;
  int len;
#if SERVER_METHOD == PROTOCOL_METHOD_GET
  if (gd && gd->ts) {
    len = snprintf(url, sizeof(url), "%s/push?id=%s&timestamp=%s&lat=%f&lon=%f&altitude=%d&speed=%f&heading=%d",
      SERVER_PATH, devid, isoTime,
      gd->lat, gd->lng, (int)gd->alt, gd->speed, (int)gd->heading);
  } else {
    len = snprintf(url, sizeof(url), "%s/push?id=%s", SERVER_PATH, devid);
  }
  success = cell.send(METHOD_GET, url, true);
#else
  len = snprintf(url, sizeof(url), "%s/post/%s", SERVER_PATH, devid);
#if ENABLE_WIFI
  if (wifi.connected()) {
    Serial.print("[WIFI] ");
    Serial.println(url);
    success = wifi.send(METHOD_POST, url, true, packetBuffer, packetSize);
  }
  else
#endif
  {
    Serial.print("[CELL] ");
    Serial.println(url);
    success = cell.send(METHOD_POST, url, true, packetBuffer, packetSize);
  }
  len += packetSize;
#endif
  if (!success) {
    Serial.println("Connection closed");
    return false;
  } else {
    txBytes += len;
    txCount++;
  }

  // check response
  int recvBytes = 0;
  char* content = 0;
#if ENABLE_WIFI
  if (wifi.connected())
  {
    content = wifi.receive(cell.getBuffer(), RECV_BUF_SIZE - 1, &recvBytes);
  }
  else
#endif
  {
    content = cell.receive(&recvBytes);
  }
  if (!content) {
    // close connection on receiving timeout
    Serial.println("No HTTP response");
    return false;
  }
  Serial.print("[HTTP] ");
  Serial.println(content);
#if ENABLE_WIFI
  if ((wifi.connected() && wifi.code() == 200) || cell.code() == 200) {
#else
  if (cell.code() == 200) {
#endif
    // successful
    lastSyncTime = millis();
    rxBytes += recvBytes;
  }
  return true;
}

bool TeleClientHTTP::connect(bool quick)
{
  if (!quick) {
#if ENABLE_WIFI
    if (!wifi.connected()) cell.init();
#else
    cell.init();
#endif
  } else {
#if ENABLE_WIFI
    if (!wifi.connected()) cell.close();
#else
    cell.close();
#endif
  }

  // connect to HTTP server
  bool success = false;

#if ENABLE_WIFI
  if (wifi.connected()) success = wifi.open(SERVER_HOST, SERVER_PORT);
#endif
  if (!success) {
    for (byte attempts = 0; !success && attempts < 3; attempts++) {
      success = cell.open(SERVER_HOST, SERVER_PORT);
      if (!success) {
        if (!cell.check()) break;
        cell.close();
        cell.init();
      }
    }
  }
  if (!success) {
    Serial.println("[CELL] Unable to connect");
    return false;
  }
  if (quick) return true;
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
  if (login) {
    notify(EVENT_LOGOUT);
    login = false;
    Serial.println("[NET] Logout");
  }
#if ENABLE_WIFI
  if (wifi.connected()) {
    wifi.end();
    Serial.println("[WIFI] Deactivated");
    return;
  }
#endif
  cell.close();
  cell.end();
  Serial.println("[CELL] Deactivated");
}
