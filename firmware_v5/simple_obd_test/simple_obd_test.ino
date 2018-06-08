/******************************************************************************
* Simple OBD test sketch for Freematics ONE+
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

#define PIN_LED 4

COBDSPI obd;
bool connected = false;
unsigned long count = 0;

#define CONNECT_OBD 1

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  delay(1000);
  digitalWrite(PIN_LED, LOW);
  Serial.begin(115200);
  byte ver = obd.begin();
  Serial.print("OBD Firmware Version ");
  Serial.println(ver);
}

void loop() {
  uint32_t ts = millis();
  digitalWrite(PIN_LED, HIGH);
  // put your main code here, to run repeatedly:
#if CONNECT_OBD
  if (!connected) {
    digitalWrite(PIN_LED, HIGH);
    Serial.print("Connecting to OBD...");
    if (obd.init()) {
      Serial.println("OK");
      connected = true;
    } else {
      Serial.println();
    }
    digitalWrite(PIN_LED, LOW);
    return;
  }
#endif
  int value;
  Serial.print('[');
  Serial.print(millis());
  Serial.print("] #");
  Serial.print(count++);
#if CONNECT_OBD
  if (obd.readPID(PID_RPM, value)) {
    Serial.print(" RPM:");
    Serial.print(value);
  }
  if (obd.readPID(PID_SPEED, value)) {
    Serial.print(" SPEED:");
    Serial.print(value);
  }
#endif
  Serial.print(" BATTERY:");
  Serial.print(obd.getVoltage());
  Serial.print('V');
#ifdef ESP32
  int temp = (int)readChipTemperature() * 165 / 255 - 40;
  Serial.print(" CPU TEMP:");
  Serial.print(temp);
#endif
  Serial.println();
  if (obd.errors > 2) {
    Serial.println("OBD disconnected");
    connected = false;
    obd.reset();
  }
  digitalWrite(PIN_LED, LOW);

  delay(100);
}
