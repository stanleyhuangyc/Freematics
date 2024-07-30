/******************************************************************************
* Simple CAN bus sniffing example for Freematics ONE+ (Model B only)
* Written by Stanley Huang <stanley@freematics.com.au>
* Distributed under BSD license
* Visit https://freematics.com/products for hardware information
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

FreematicsESP32 sys;
COBD obd;

void setup()
{
  // USB serial
  Serial.begin(115200);
  Serial.println("CAN Sniffing Demo");

  // initialize co-processor link
  while (!sys.begin());
  // initialize OBD library
  obd.begin(sys.link);

  // stop sniffing in case previously running
  obd.sniff(false);

  // start on 11-bit/500Kbps CAN bus
  while (!obd.init(PROTO_ISO15765_11B_500K)) Serial.print('.');
  
  // send a CAN message (clearing DTC)
  Serial.println("Sending a CAN message...");
  obd.setCANID(0x7E0);
  obd.setHeaderMask(0xFFFFFF);
  obd.setHeaderFilter(0x7E8);
  byte msg[] = {0x14, 0xFF, 0x00};
  char buf[128];
  if (obd.sendCANMessage(msg, sizeof(msg), buf, sizeof(buf))) {
    // print the ECU response to the message
    Serial.println(buf);
  }

  // we are interested in CAN messages with header 7E*
  obd.setHeaderFilter(0x7E0);
  obd.setHeaderMask(0xFFFFFFF0);

  // start CAN bus sniffing
  obd.sniff();
  Serial.println("CAN sniffer started");
}

void loop()
{
  byte data[128];
  // load one received CAN message into buffer
  int bytes = obd.receiveData(data, sizeof(data));
  if (bytes > 0) {
    // print timestamp
    Serial.print('[');
    Serial.print(millis());
    Serial.print("] ");
    // print received CAN message
    for (int n = 0; n < bytes; n++) {
      byte d = data[n];
      if (d < 0x10) Serial.print('0');
      Serial.print(d, HEX);
      Serial.print(' ');
    }
    Serial.println();
  }
}
