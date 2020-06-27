/******************************************************************************
* J1939 broadcast data monitor for Freematics ONE+ (Model H only)
* Written by Stanley Huang <stanley@freematics.com.au>
* Distributed under BSD license
* For details about Freematics ONE+ Model H, please visit:
* https://freematics.com/products/freematics-one-plus-model-h/
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

  // initialize OBD library
  obd.begin(sys.link);

  // start with J1939 protocol
  while (!obd.init(PROTO_J1939));

  Serial.println("J1939 data monitor started");
}

void loop()
{
  byte buf[128];
  // load one received CAN message into buffer
  int bytes = obd.receiveData(buf, sizeof(buf));
  if (bytes > 0) {
    if (bytes < 3) {
      Serial.println("Invalid data");
      return;
    }
    // print timestamp
    Serial.print('[');
    Serial.print(millis());
    Serial.print("] ");
    // print received CAN message
    uint32_t pgn = (uint32_t)buf[0] << 16 | (uint32_t)buf[1] << 8 | buf[2];
    Serial.print("PGN:");
    Serial.print(pgn);
    Serial.print(" Data:");
    for (int n = 3; n < bytes; n++) {
      byte d = buf[n];
      if (d < 0x10) Serial.print('0');
      Serial.print(d, HEX);
      Serial.print(' ');
    }
    Serial.println();
  }
}
