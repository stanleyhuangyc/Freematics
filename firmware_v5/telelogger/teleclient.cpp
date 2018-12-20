/******************************************************************************
* Freematics Hub client and Traccar client implementations
* Works with Freematics ONE+
* Developed by Stanley Huang <stanley@freematics.com.au>
* Distributed under BSD license
* Visit https://freematics.com/products for hardware information
* Visit https://freematics.com/hub for information about Freematics Hub
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
#include "teleclient.h"

bool processCommand(char* data);

extern char devid[];
extern char vin[];
extern GPS_DATA* gd;
extern char isoTime[];

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
  netbuf.header(feedid);
  byte len = sprintf(buf, "EV=%X", (unsigned int)event);
  netbuf.dispatch(buf, len);
  len = sprintf(buf, "TS=%lu", millis());
  netbuf.dispatch(buf, len);
  len = sprintf(buf, "ID=%s", devid);
  netbuf.dispatch(buf, len);
  if (vin[0]) {
    len = sprintf(buf, "VIN=%s", vin);
    netbuf.dispatch(buf, len);
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
    }
    // success
    return true;
  }
  return false;
}

bool TeleClientUDP::connect()
{
  byte event = feedid == 0 ? EVENT_LOGIN : EVENT_RECONNECT;
  // connect to telematics server
  for (byte attempts = 0; attempts < 5; attempts++) {
    Serial.print(event == EVENT_LOGIN ? "LOGIN(" : "RECONNECT(");
    Serial.print(SERVER_HOST);
    Serial.print(':');
    Serial.print(SERVER_PORT);
    Serial.print(")...");
    if (!net.open(SERVER_HOST, SERVER_PORT)) {
      Serial.println("Network error");
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
    Serial.println("OK");

    Serial.print("FEED ID:");
    Serial.println(feedid);

    lastSyncTime = millis();

#if ENABLE_OLED
    oled.print("FEED ID:");
    oled.println(feedid);
#endif
    return true;
  }
  return false;
}

bool TeleClientUDP::ping()
{
    if (!net.open(SERVER_HOST, SERVER_PORT)) {
      return false;
    }
    bool success = notify(EVENT_PING);
    if (success) lastSyncTime = millis();
    return success;
}

bool TeleClientUDP::transmit(const char* packetBuffer, unsigned int packetSize)
{
  //Serial.println(cache.buffer()); // print the content to be sent
  // transmit data
  if (net.send(packetBuffer, packetSize)) {
    txBytes += packetSize;
    txCount++;
    lastSentTime = millis();
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
      lastSyncTime = millis();
      break;
    }
  } while(0);
}

bool TeleClientHTTP::transmit(const char* packetBuffer, unsigned int packetSize)
{
  if (net.state() == HTTP_SENT) {
    // check response
    int bytes = 0;
    char* resp = net.receive(&bytes, 3000);
    if (resp) {
      if (strstr(resp, "200 OK")) {
        // successful
        lastSyncTime = millis();
        rxBytes += bytes;
      } else {
        Serial.print(resp);
      }
    } else {
      // close connection on receiving timeout
      Serial.println("No HTTP response");
      net.close();
      return false;
    }
  }

  if (net.state() != HTTP_CONNECTED) {
    // reconnect if disconnected
    if (!connect()) {
      Serial.println("Error connecting to HTTP server");
      return false;
    }
  }

  char url[256];
  if (gd && gd->ts) {
    sprintf(url, "%s?id=%s&timestamp=%s&lat=%f&lon=%f&altitude=%d&speed=%f&heading=%d",
      SERVER_PATH, devid, isoTime,
      gd->lat, gd->lng, (int)gd->alt, gd->speed, (int)gd->heading);
  } else {
    sprintf(url, "%s?id=%s", SERVER_PATH, devid);
  }

  lastSentTime = millis();
  int ret;
#if SERVER_PROTOCOL == PROTOCOL_HTTP_POST
  ret = net.send(HTTP_POST, url, true, packetBuffer, packetSize);
#else
  ret = net.send(HTTP_GET, url, true);
#endif
  if (ret == 0) {
    Serial.println("Connection closed by server");
    net.close();
  } else if (ret < 0) {
    Serial.println("Error sending HTTP request");
    net.close();
  } else {
    txBytes += ret;
    txCount++;
  }  
  return ret > 0;
}

bool TeleClientHTTP::connect()
{
  Serial.print("Connecting ");
  Serial.print(SERVER_HOST);
  Serial.print(':');
  Serial.print(SERVER_PORT);
  Serial.print("...");
  // connect to HTTP server
  if (!net.open(SERVER_HOST, SERVER_PORT)) {
    Serial.println("NO");
    return false;
  }
  Serial.println("OK");
  lastSyncTime = millis();
  return true;
}

bool TeleClientHTTP::ping()
{
  return connect();
}