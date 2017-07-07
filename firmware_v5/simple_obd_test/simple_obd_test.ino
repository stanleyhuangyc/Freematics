/******************************************************************************
* Simple OBD-II test sketch for Freematics ONE/ONE+
* Written by Stanley Huang https://www.facebook.com/stanleyhuangyc
* Distributed under BSD license
* Visit http://freematics.com/products for hardware information
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
******************************************************************************/

#include <FreematicsONE.h>

COBDSPI obd;
bool connected = false;

void setup() {
  // put your setup code here, to run once:
  delay(1000);
  Serial.begin(115200);
  byte ver = obd.begin();
  Serial.print("OBD Firmware Version ");
  Serial.println(ver);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!connected) {
    Serial.print("Connecting to OBD...");
    if (obd.init()) {
      Serial.println("OK");
      connected = true;
    } else {
      Serial.println();
      return;
    }
  }
  int value;
  Serial.print('[');
  Serial.print(millis());
  Serial.print(']');
  if (obd.readPID(PID_RPM, value)) {
    Serial.print(" RPM=");
    Serial.print(value);
  }
  if (obd.readPID(PID_SPEED, value)) {
    Serial.print(" SPEED=");
    Serial.print(value);
  }
  float v = obd.getVoltage();
  Serial.print(" VOLTAGE=");
  Serial.print(v);
  Serial.print('V');
#ifdef ESP32
  int temp = (int)readChipTemperature() * 165 / 255 - 40;
  Serial.print(" ESP32.TEMP=");
  Serial.print(temp);
#endif
  Serial.println();
  if (obd.errors > 3) {
    Serial.println("OBD disconnected");
    connected = false;
    obd.uninit();
  }
  delay(500);
}
