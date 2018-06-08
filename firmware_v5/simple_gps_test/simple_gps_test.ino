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
#include <TinyGPS.h>

#define PIN_LED 4
#define PIN_GPS_POWER 15
#define PIN_GPS_UART_RXD 32
#define PIN_GPS_UART_TXD 33

TinyGPS gps;
HardwareSerial Serial1(1);
bool locked = false;

void setup()
{
  delay(100);
  // USB serial
  Serial.begin(115200);
  Serial.println("SIMPLE GPS TEST");
  // GPS serial
  Serial1.begin(115200, SERIAL_8N1, PIN_GPS_UART_RXD, PIN_GPS_UART_TXD);
  // turn on GPS power
  pinMode(PIN_GPS_POWER, OUTPUT);
  digitalWrite(PIN_GPS_POWER, HIGH);
  delay(10);
}

void loop()
{
  if (Serial1.available()) {
    char c = Serial1.read();
    if (gps.encode(c)) {
      locked = true;
      unsigned long utcdate, utctime;
      long lat, lng;
      gps.get_datetime(&utcdate, &utctime, 0);
      gps.get_position(&lat, &lng, 0);
      int speed = gps.speed() * 1852 / 100000; // km/h
      int alt = gps.altitude() / 1000; // m
      int sats = gps.satellites();
      Serial.print("UTC:");
      Serial.print(utctime);
      Serial.print(" LAT:");
      Serial.print(lat);
      Serial.print(" LNG:");
      Serial.print(lng);
      Serial.print(" Speed:");
      Serial.print(speed);
      Serial.print(" Alt:");
      Serial.print(alt);
      Serial.print(" Sats:");
      Serial.println(sats);
    }
    if (!locked) {
      // output NMEA stream before satellites gets locked
      Serial.write(c);
    }
  }
}
