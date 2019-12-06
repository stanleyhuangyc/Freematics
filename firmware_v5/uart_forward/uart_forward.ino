/******************************************************************************
* Simple GPS test sketch for Freematics ONE+
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

#include <Arduino.h>
#include "esp_task_wdt.h"

#define PIN_UART_RXD 13
#define PIN_UART_TXD 14
#define PIN_POWER 15
#define PIN_POWER2 12

#define UART_BAUDRATE 115200

void setup()
{
  esp_task_wdt_init(600, 0);
  
  // USB serial
  Serial.begin(115200);
  // target serial
  Serial1.begin(UART_BAUDRATE, SERIAL_8N1, PIN_UART_RXD, PIN_UART_TXD);
  // set some pins
  pinMode(PIN_POWER, OUTPUT);
  digitalWrite(PIN_POWER, HIGH);
  pinMode(PIN_POWER2, OUTPUT);
  digitalWrite(PIN_POWER2, HIGH);
}

void loop()
{
  if (Serial.available()) {
    Serial1.write(Serial.read());
  }
  if (Serial1.available()) {
    Serial.write(Serial1.read());
  }
}
