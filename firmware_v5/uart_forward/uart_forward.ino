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

#define PIN_LED 4
#define PIN_UART_RXD 32
#define PIN_UART_TXD 33
#define PIN_POWER 15

#define USE_SOFT_SERIAL 0
#define UART_BAUDRATE 115200

TaskHandle_t xThread;

#if USE_SOFT_SERIAL

uint32_t inline IRAM_ATTR getCycleCount()
{
  uint32_t ccount;
  __asm__ __volatile__("esync; rsr %0,ccount":"=a" (ccount));
  return ccount;
}

uint8_t inline IRAM_ATTR readRxPin()
{
#if PIN_UART_RXD < 32
  return (uint8_t)(GPIO.in >> PIN_UART_RXD) << 7;
#else
  return (uint8_t)(GPIO.in1.val >> (PIN_UART_RXD - 32)) << 7;
#endif
}

void inline setTxPinHigh()
{
#if PIN_UART_TXD < 32
  GPIO.out_w1ts = ((uint32_t)1 << PIN_UART_TXD);
#else
  GPIO.out1_w1ts.val = ((uint32_t)1 << (PIN_UART_TXD - 32));
#endif
}

void inline setTxPinLow()
{
#if PIN_UART_TXD < 32
  GPIO.out_w1tc = ((uint32_t)1 << PIN_UART_TXD);
#else
  GPIO.out1_w1tc.val = ((uint32_t)1 << (PIN_UART_TXD - 32));
#endif
}

void serialTx(uint8_t c)
{
  uint32_t start = getCycleCount();
  // start bit
  setTxPinLow();
  for (uint32_t i = 1; i <= 8; i++, c >>= 1) {
    while (getCycleCount() - start < i * F_CPU / UART_BAUDRATE);
    if (c & 0x1)
      setTxPinHigh();
    else
      setTxPinLow();
  }
  while (getCycleCount() - start < (uint32_t)9 * F_CPU / UART_BAUDRATE) taskYIELD();
  setTxPinHigh();
  while (getCycleCount() - start < (uint32_t)10 * F_CPU / UART_BAUDRATE) taskYIELD();
}

#endif

void IRAM_ATTR taskRx(void* arg)
{
#if USE_SOFT_SERIAL
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  for (;;) {
    while (readRxPin());
    uint32_t start = getCycleCount();
    uint8_t c = 0;
    taskYIELD();
    taskENTER_CRITICAL(&mux);
    for (uint32_t i = 1; i <= 8; i++) {
      while (getCycleCount() - start < i * F_CPU / UART_BAUDRATE + F_CPU / UART_BAUDRATE / 3);
      c = (c | readRxPin()) >> 1;
    }
    taskEXIT_CRITICAL(&mux);
    Serial.write(c);
    while (getCycleCount() - start < (uint32_t)9 * F_CPU / UART_BAUDRATE + F_CPU / UART_BAUDRATE / 2) taskYIELD();
  }
#else
  for (;;) {
    if (Serial1.available()) {
      char c = Serial1.read();
      Serial.write(c);
    } else {
      taskYIELD();
    }
  }
#endif
}

void setup()
{
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_POWER, OUTPUT);
  digitalWrite(PIN_POWER, HIGH);
  delay(100);
  // USB serial
  Serial.begin(115200);
  // GPS serial
#if USE_SOFT_SERIAL
  pinMode(PIN_UART_RXD, INPUT);
  pinMode(PIN_UART_TXD, OUTPUT);
  digitalWrite(PIN_UART_TXD, HIGH);
#else
  Serial1.begin(UART_BAUDRATE, SERIAL_8N1, PIN_UART_RXD, PIN_UART_TXD);
#endif
  // start serial UART Rx task
  xTaskCreate(taskRx, "taskRx", 1024, 0, 1, &xThread);
  // set watchdog
  esp_task_wdt_init(3600, 0);
}

void loop()
{
  if (Serial.available()) {
    digitalWrite(PIN_LED, HIGH);
    char c = Serial.read();
#if USE_SOFT_SERIAL
    serialTx(c);
#else
    Serial1.write(c);
#endif
    digitalWrite(PIN_LED, LOW);
  }
}
