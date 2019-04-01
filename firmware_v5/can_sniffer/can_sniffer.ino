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

  // initialize co-processor link
  while (!sys.begin());
  Serial.print("Firmware: V");
  Serial.println(sys.version);
  // initialize OBD library
  obd.begin(sys.link);

  // stop sniffing in case previously running
  obd.sniff(false);

  // start on 11-bit/500Kbps CAN bus
  while (!obd.init(PROTO_CAN_11B_500K));
  
  // we are interested in CAN messages with header 7E*
  obd.setHeaderFilter(0x7E0);
  obd.setHeaderMask(0xFFFFFFF0);

  // start CAN bus sniffing
  obd.sniff();
}

void loop()
{
  byte buf[128];
  // load one received CAN message into buffer
  int bytes = obd.receiveCAN(buf, sizeof(buf));
  if (bytes > 0) {
    // print timestamp
    Serial.print('[');
    Serial.print(millis());
    Serial.print("] ");
    // print received CAN message
    for (int n = 0; n < bytes; n++) {
      byte d = buf[n];
      if (d < 0x10) Serial.print('0');
      Serial.print(d, HEX);
      Serial.print(' ');
    }
    Serial.println();
  }
}
