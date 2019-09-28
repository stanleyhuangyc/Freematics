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
#define CELL_APN ""
#define CONN_TIMEOUT 5000

FreematicsESP32 sys;
HTTPClientSIM7600 net;
int errors = 0;

bool init_net()
{
    // initialize SIM5360 xBee module (if present)
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
    if (!net.checkSIM()) {
      Serial.println("No SIM Card");
      return false;
    }
    Serial.println("SIM Card OK");

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
    if (net.open()) {
      Serial.println("OK");
    } else {
      Serial.println("NO");
    }
    return true;
}

void setup()
{
  Serial.begin(115200);
  // use following for Freematics ONE+
  sys.begin();
  // use following for Freematics Esprit or other ESP32 dev board
  //sys.xbBegin(115200, 16, 17);

  // initialize cellular module
  while (!init_net());
}

void loop()
{
  if (errors > 0) {
    net.close();
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
  }

  // connect to HTTP server
  if (net.state() != HTTP_CONNECTED) {
    Serial.print("Connecting...");
    if (net.open(SERVER_HOST, SERVER_PORT)) {
      Serial.println("OK");
    } else {
      Serial.println("failed");
      errors++;
      return;
    }
  }

  // send HTTP request
  Serial.print("Sending request...");
  if (!net.send(METHOD_GET, SERVER_PATH, true)) {
    Serial.println("failed");
    errors++;
    net.close();
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
    errors++;
  }

  Serial.println("Waiting 5 seconds...");
  delay(5000);
}
