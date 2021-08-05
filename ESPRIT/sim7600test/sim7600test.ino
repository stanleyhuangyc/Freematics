/******************************************************************************
* HTTPS testing sketch for Freematics ONE+ with SIM7600
* https://freematics.com/products/freematics-one-plus/
* Developed by Stanley Huang, distributed under BSD license
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

// testing URL: https://hub.freematics.com/test
#define SERVER_HOST "hub.freematics.com"
#define SERVER_PORT 443
#define SERVER_PATH "/test"
#define CELL_APN "internet"
#define CONN_TIMEOUT 5000

#define RUNTIME 180 // seconds

FreematicsESP32 sys;
HTTPClientSIM7600 net;
int errors = 0;
uint32_t startTime = 0;

bool init_net()
{
    Serial.print("Init cellular module...");
    if (net.begin(&sys)) {
      Serial.print(net.deviceName());
      Serial.println(" OK");
    } else {
      Serial.println("NO");
      return false;
    }
    Serial.print("IMEI:");
    Serial.println(net.IMEI);

    if (net.checkSIM()) {
      Serial.println("SIM Card OK");
    } else {
      Serial.println("No SIM Card");
    }

    if (net.setGPS(true)) {
      Serial.println("Cellular GNSS On");
    }

    Serial.print("Registering on network...");
    if (net.setup(CELL_APN)) {
      Serial.println("OK");
    } else {
      Serial.println("NO");
      return false;
    }

    String op = net.getOperatorName();
    if (op.length()) {
      Serial.print("Operator:");
      Serial.println(op);
    }

    Serial.print("Obtaining IP address...");
    String ip = net.getIP();
    if (ip) {
      Serial.println(ip);
    } else {
      Serial.println("N/A");
    }

    int signal = net.getSignal();
    if (signal) {
      Serial.print("RSSI:");
      Serial.print(signal);
      Serial.println("dBm");
    }

    Serial.print("Init HTTPS stack...");
    net.open();
    Serial.println("OK");
    return true;
}

void setup()
{
  Serial.begin(115200);

  sys.begin(false, true);

  // initialize cellular module
  while (!init_net());
}

void loop()
{
  if (errors > 10) {
    // re-initialize cellular module
    net.end();
    if (init_net()) {
      Serial.println("OK");
      errors = 0;
    } else {
      Serial.println("NO");
      delay(3000);
      return;
    }
  }

  // connect to HTTP server
  if (net.state() != HTTP_CONNECTED) {
    Serial.print("Connecting...");
    if (net.open(SERVER_HOST, SERVER_PORT)) {
      Serial.println("OK");
    } else {
      Serial.println("failed");
      net.close();
      errors++;
      return;
    }
  }

  // get GNSS data
  char isoTime[26] = {0};
  GPS_DATA* gd = 0;
  if (net.getLocation(&gd) && gd && gd->date) {
    // generate ISO time string
    char *p = isoTime + sprintf(isoTime, "%04u-%02u-%02uT%02u:%02u:%02u",
        (unsigned int)(gd->date % 100) + 2000, (unsigned int)(gd->date / 100) % 100, (unsigned int)(gd->date / 10000),
        (unsigned int)(gd->time / 1000000), (unsigned int)(gd->time % 1000000) / 10000, (unsigned int)(gd->time % 10000) / 100);
    unsigned char tenth = (gd->time % 100) / 10;
    if (tenth) p += sprintf(p, ".%c00", '0' + tenth);
    *p = 'Z';
    *(p + 1) = 0;

    Serial.print(isoTime);
    Serial.print(" LAT:");
    Serial.print(gd->lat, 6);
    Serial.print(" LNG:");
    Serial.print(gd->lng, 6);
    Serial.print(" ALT:");
    Serial.print((int)gd->alt);
    Serial.print("m ");
    Serial.print(gd->speed * 1.852f, 1);
    Serial.print("km/h");
    Serial.print(" Course:");
    Serial.println(gd->heading);
  }

  // form up HTTP request path
  char path[384];
  if (gd && gd->date) {
    sprintf(path, "%s/?timestamp=%s&lat=%f&lon=%f&altitude=%d&speed=%f&heading=%d",
      SERVER_PATH, isoTime, gd->lat, gd->lng, (int)gd->alt, gd->speed, (int)gd->heading);
  } else {
    strcpy(path, SERVER_PATH);
  }
  Serial.print("Request path: ");
  Serial.println(path);

  // send HTTP request
  Serial.print("Sending request...");
  if (!net.send(METHOD_GET, path, true)) {
    Serial.println("failed");
    net.close();
    errors++;
    return;
  } else {
    Serial.println("OK");
  }

  // receive HTTP response
  Serial.print("Receiving...");
  char *response;
  int bytes;
  response = net.receive(&bytes);
  if (response) {
    Serial.println("OK");
    Serial.println("-----HTTP RESPONSE-----");
    Serial.println(response);
    Serial.println("-----------------------");
    errors = 0;
  } else {
    Serial.println("failed");
    net.close();
    errors++;
  }

  if (millis() - startTime > RUNTIME) {
    // 
  }
  
  Serial.println("Waiting 5 seconds...");
  delay(15000);
}
