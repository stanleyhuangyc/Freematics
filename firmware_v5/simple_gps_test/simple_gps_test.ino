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
#include "esp_task_wdt.h"

#define PIN_LED 4
#define PIN_GPS_POWER 15
#define PIN_GPS_UART_RXD 32
#define PIN_GPS_UART_TXD 33

#define USE_SOFT_SERIAL 0
#define GPS_BAUDRATE 115200

TinyGPS gps;
#if !USE_SOFT_SERIAL
HardwareSerial Serial1(1);
#endif

TaskHandle_t xThread;

unsigned int updates = 0;

#if USE_SOFT_SERIAL

uint32_t inline IRAM_ATTR getCycleCount()
{
  uint32_t ccount;
  __asm__ __volatile__("esync; rsr %0,ccount":"=a" (ccount));
  return ccount;
}

uint8_t inline IRAM_ATTR readRxPin()
{
#if PIN_GPS_UART_RXD < 32
  return (uint8_t)(GPIO.in >> PIN_GPS_UART_RXD) << 7;
#else
  return (uint8_t)(GPIO.in1.val >> (PIN_GPS_UART_RXD - 32)) << 7;
#endif
}

#endif

void IRAM_ATTR taskGPS(void* arg)
{
#if USE_SOFT_SERIAL
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  for (;;) {
    while (readRxPin());
    uint32_t start = getCycleCount();
    uint8_t c = 0;
    taskYIELD();
    taskENTER_CRITICAL(&mux);
    for (uint32_t i = 1; i <= 7; i++) {
      while (getCycleCount() - start < i * F_CPU / GPS_BAUDRATE + F_CPU / GPS_BAUDRATE / 3);
      c = (c | readRxPin()) >> 1;
    }
    taskEXIT_CRITICAL(&mux);
    if (c && gps.encode(c)) {
      updates++;
    }
    while (getCycleCount() - start < (uint32_t)9 * F_CPU / GPS_BAUDRATE + F_CPU / GPS_BAUDRATE / 2) taskYIELD();
  }
#else
  for (;;) {
    if (Serial1.available()) {
      if (gps.encode(Serial1.read())) updates++;
    }
  }
#endif
}

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  delay(100);
  // USB serial
  Serial.begin(115200);
  Serial.println("SIMPLE GPS TEST");
  // GPS serial
#if USE_SOFT_SERIAL
  pinMode(PIN_GPS_UART_RXD, INPUT);
#else
  Serial1.begin(115200, SERIAL_8N1, PIN_GPS_UART_RXD, PIN_GPS_UART_TXD);
#endif
  // turn on GPS power
  pinMode(PIN_GPS_POWER, OUTPUT);
  digitalWrite(PIN_GPS_POWER, HIGH);
  // start GPS decoding task
  xTaskCreate(taskGPS, "taskGPS", 1024, 0, 1, &xThread);
  Serial.println("Wait for GPS signal...");
  digitalWrite(PIN_LED, LOW);
  // set watchdog
  esp_task_wdt_init(3600, 0);
}

void loop()
{
  if (updates) {
    updates = 0;
    static unsigned long lastutc = 0;
    unsigned long utcdate, utctime;
    gps.get_datetime(&utcdate, &utctime, 0);
    if (utctime != lastutc) {
      digitalWrite(PIN_LED, HIGH);
      lastutc = utctime;
      long lat, lng;
      gps.get_datetime(&utcdate, &utctime, 0);
      gps.get_position(&lat, &lng, 0);
      int speed = gps.speed() * 1852 / 100000; // km/h
      int alt = gps.altitude() / 100; // m
      int sats = gps.satellites();
      Serial.print("UTC:");
      Serial.print(utctime);
      Serial.print(" LAT:");
      Serial.print((float)lat / 1000000, 6);
      Serial.print(" LNG:");
      Serial.print((float)lng / 1000000, 6);
      Serial.print(" Speed:");
      Serial.print(speed);
      Serial.print("km/h Alt:");
      Serial.print(alt);
      Serial.print("m Sats:");
      Serial.println(sats);
#if USE_SOFT_SERIAL
      static unsigned long start = 0;
      unsigned long chars;
      unsigned short good, bad;
      gps.stats(&chars, &good, &bad);
      if (good == 0) start = millis() - 1;
      Serial.print("Stats: ");
      Serial.print(chars);
      Serial.print(' ');
      Serial.print(good);
      Serial.print(' ');
      Serial.print((float)(good * 1000) / (millis() - start), 1);
      Serial.print(' ');
      Serial.println(bad);
#endif
      digitalWrite(PIN_LED, LOW);
    }
  }
  delay(50);
}
