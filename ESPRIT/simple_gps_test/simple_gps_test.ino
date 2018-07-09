/******************************************************************************
* Simple GPS test sketch for Freematics Esprit
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

#define PIN_GPS_UART_RXD 32
#define PIN_GPS_UART_TXD 33
#define GPS_BAUDRATE 115200

TinyGPS gps;
HardwareSerial Serial1(1);

void setup()
{
  delay(100);
  // start USB serial
  Serial.begin(115200);
  Serial.println("SIMPLE GPS TEST");
  // start GPS serial
  Serial1.begin(GPS_BAUDRATE, SERIAL_8N1, PIN_GPS_UART_RXD, PIN_GPS_UART_TXD);
}

void loop()
{
  if (!Serial1.available()) return;
  char c = Serial1.read();
  // uncomment following line to output NMEA stream
  // Serial.write(c); 
  if (gps.encode(c)) {
    unsigned long utcdate, utctime;
    gps.get_datetime(&utcdate, &utctime, 0);
    float speed = (float)gps.speed() * 1852 / 100000; // km/h
    int alt = gps.altitude() / 100; // m
    int sats = gps.satellites();
    long lat, lng;
    gps.get_datetime(&utcdate, &utctime, 0);
    gps.get_position(&lat, &lng, 0);
    Serial.print("UTC:");
    Serial.print(utctime);
    Serial.print(" LAT:");
    Serial.print((float)lat / 1000000, 6);
    Serial.print(" LNG:");
    Serial.print((float)lng / 1000000, 6);
    Serial.print(" Speed:");
    Serial.print(speed, 1);
    Serial.print("km/h Alt:");
    Serial.print(alt);
    Serial.print("m Sats:");
    Serial.println(sats);
  }
}
