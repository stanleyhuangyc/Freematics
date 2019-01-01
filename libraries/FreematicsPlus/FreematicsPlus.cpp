/*************************************************************************
* Arduino library for ESP32 based Freematics ONE+ and Freematics Esprit
* Distributed under BSD license
* Visit https://freematics.com for more information
* (C)2017-2018 Developed by Stanley Huang <support@freematics.com.au>
*************************************************************************/

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_pm.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "soc/uart_struct.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "FreematicsPlus.h"
#include "FreematicsGPS.h"

static TinyGPS gps;
static byte gpsPendingData = 0;
static char* nmeaBuffer = 0;
static int nmeaBytes = 0;
Mutex nmeaBufferMutex;
static Task taskGPS;
static GPS_DATA* gpsData = 0;

static uint32_t inline getCycleCount()
{
  uint32_t ccount;
  __asm__ __volatile__("esync; rsr %0,ccount":"=a" (ccount));
  return ccount;
}

static uint8_t inline readRxPin()
{
#if PIN_GPS_UART_RXD < 32
  return (uint8_t)(GPIO.in >> PIN_GPS_UART_RXD) << 7;
#else
  return (uint8_t)(GPIO.in1.val >> (PIN_GPS_UART_RXD - 32)) << 7;
#endif
}

static void gps_decode_task(void* inst)
{
    for (;;) {
        uint8_t c = 0;
        int len = uart_read_bytes(GPS_UART_NUM, &c, 1, 60000 / portTICK_RATE_MS);
        if (len != 1) continue;
        if (nmeaBuffer && nmeaBytes < NMEA_BUF_SIZE) {
            nmeaBufferMutex.lock();
            nmeaBuffer[nmeaBytes++] = c;
            nmeaBufferMutex.unlock();
        }
        if (gps.encode(c)) {
            gpsPendingData++;
        }
    }
}

static void inline setTxPinHigh()
{
#if PIN_GPS_UART_TXD < 32
  GPIO.out_w1ts = ((uint32_t)1 << PIN_GPS_UART_TXD);
#else
  GPIO.out1_w1ts.val = ((uint32_t)1 << (PIN_GPS_UART_TXD - 32));
#endif
}

static void inline setTxPinLow()
{
#if PIN_GPS_UART_TXD < 32
  GPIO.out_w1tc = ((uint32_t)1 << PIN_GPS_UART_TXD);
#else
  GPIO.out1_w1tc.val = ((uint32_t)1 << (PIN_GPS_UART_TXD - 32));
#endif
}

static void softSerialTx(uint32_t baudrate, uint8_t c)
{
  uint32_t start = getCycleCount();
  // start bit
  setTxPinLow();
  for (uint32_t i = 1; i <= 8; i++, c >>= 1) {
    while (getCycleCount() - start < i * F_CPU / baudrate);
    if (c & 0x1)
      setTxPinHigh();
    else
      setTxPinLow();
  }
  while (getCycleCount() - start < (uint32_t)9 * F_CPU / baudrate);
  setTxPinHigh();
  while (getCycleCount() - start < (uint32_t)10 * F_CPU / baudrate);
}

static void gps_soft_decode_task(void* inst)
{
    // start receiving and decoding
    for (;;) {
        uint8_t c = 0;
        do {
            taskYIELD();
        } while (readRxPin());
        uint32_t start = getCycleCount();
        for (uint32_t i = 1; i <= 7; i++) {
            taskYIELD();
            while (getCycleCount() - start < i * F_CPU / GPS_SOFT_BAUDRATE + F_CPU / GPS_SOFT_BAUDRATE / 3);
            c = (c | readRxPin()) >> 1;
        }
        if (gps.encode(c)) {
            gpsPendingData++;
        }
        do {
            taskYIELD();
        } while (getCycleCount() - start < (uint32_t)9 * F_CPU / GPS_SOFT_BAUDRATE + F_CPU / GPS_SOFT_BAUDRATE / 2);
    }
}

extern "C" {
uint8_t temprature_sens_read();
int32_t hall_sens_read();
}

// get chip temperature sensor
int readChipTemperature()
{
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR, 3, SENS_FORCE_XPD_SAR_S);
    SET_PERI_REG_BITS(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_CLK_DIV, 10, SENS_TSENS_CLK_DIV_S);
    CLEAR_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP);
    CLEAR_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_DUMP_OUT);
    SET_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP_FORCE);
    SET_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP);
    ets_delay_us(100);
    SET_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_DUMP_OUT);
    ets_delay_us(5);
    int res = GET_PERI_REG_BITS2(SENS_SAR_SLAVE_ADDR3_REG, SENS_TSENS_OUT, SENS_TSENS_OUT_S);
    return (res - 32) * 5 / 9;
}

int readChipHallSensor()
{
  return hall_sens_read();
}

uint16_t getFlashSize()
{
    return (spi_flash_get_chip_size() >> 10);
}

bool Task::create(void (*task)(void*), const char* name, int priority, int stacksize)
{
    if (xHandle) return false;
    /* Create the task, storing the handle. */
    BaseType_t xReturned = xTaskCreate(task, name, stacksize, (void*)this, priority, &xHandle);
    return xReturned == pdPASS;
}

void Task::destroy()
{
    if (xHandle) {
        void* x = xHandle;
        xHandle = 0;
        vTaskDelete((TaskHandle_t)x);
    }
}

void Task::sleep(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

bool Task::running()
{
    return xHandle != 0;
}

void Task::suspend()
{
    if (xHandle) vTaskSuspend(xHandle);
}

void Task::resume()
{
    if (xHandle) vTaskResume(xHandle);
}

Mutex::Mutex()
{
  xSemaphore = xSemaphoreCreateMutex();
  xSemaphoreGive(xSemaphore);
}

void Mutex::lock()
{
  xSemaphoreTake(xSemaphore, portMAX_DELAY);
}

void Mutex::unlock()
{
  xSemaphoreGive(xSemaphore);
}

void FreematicsESP32::begin(int cpuMHz)
{
#if 0
    if (cpuMHz) {
        esp_pm_config_esp32_t pm_config;
        pm_config.max_freq_mhz = cpuMHz;
        pm_config.min_freq_mhz = RTC_CPU_FREQ_XTAL;
        pm_config.light_sleep_enable = true;
        esp_err_t ret = esp_pm_configure(&pm_config);
        if (ret == ESP_ERR_NOT_SUPPORTED) {
            Serial.println("PM Disabled");
        }
    }
#endif

    // set watchdog timeout to 600 seconds
    esp_task_wdt_init(600, 0);
}

void FreematicsESP32::gpsEnd()
{
    // uninitialize
    taskGPS.destroy();
    if (!(gpsFlags & GF_SOFT_SERIAL)) uart_driver_delete(GPS_UART_NUM);
    if (nmeaBuffer) {
        nmeaBufferMutex.lock();
        delete nmeaBuffer;
        nmeaBuffer = 0;
        nmeaBufferMutex.unlock();
    }
	//turn off GPS power
    digitalWrite(PIN_GPS_POWER, LOW);
    gpsFlags = 0;
}

bool FreematicsESP32::gpsBegin(int baudrate, bool buffered, bool softserial)
{
    pinMode(PIN_GPS_POWER, OUTPUT);
    digitalWrite(PIN_GPS_POWER, LOW);

    if (!softserial) {
        uart_config_t uart_config = {
            .baud_rate = baudrate,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
        };
        // configure UART parameters
        uart_param_config(GPS_UART_NUM, &uart_config);
        // set UART pins
        uart_set_pin(GPS_UART_NUM, PIN_GPS_UART_TXD, PIN_GPS_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        // install UART driver
        uart_driver_install(GPS_UART_NUM, UART_BUF_SIZE, 0, 0, NULL, 0);
        // turn on GPS power
        digitalWrite(PIN_GPS_POWER, HIGH);
        delay(100);
        // start decoding task
        taskGPS.create(gps_decode_task, "GPS", 1);
    } else {
        pinMode(PIN_GPS_UART_RXD, INPUT);
        pinMode(PIN_GPS_UART_TXD, OUTPUT);
        setTxPinHigh();

        // turn on GPS power
        digitalWrite(PIN_GPS_POWER, HIGH);
        delay(200);

        // switch M8030 GNSS to 38400bps
        const uint8_t packet1[] = {0x0, 0x0, 0xB5, 0x62, 0x06, 0x0, 0x14, 0x0, 0x01, 0x0, 0x0, 0x0, 0xD0, 0x08, 0x0, 0x0, 0x0, 0x96, 0x0, 0x0, 0x7, 0x0, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x93, 0x90};
        const uint8_t packet2[] = {0xB5, 0x62, 0x06, 0x0, 0x1, 0x0, 0x1, 0x8, 0x22};
        for (int i = 0; i < sizeof(packet1); i++) softSerialTx(baudrate, packet1[i]);
        delay(10);
        for (int i = 0; i < sizeof(packet2); i++) softSerialTx(baudrate, packet2[i]);

        // start GPS decoding task (soft serial)
        taskGPS.create(gps_soft_decode_task, "GPS", 1);
        gpsFlags |= GF_SOFT_SERIAL;
    }

    // test run for a while to see if there is data decoded
    uint16_t s1 = 0, s2 = 0;
    gps.stats(&s1, 0);
    for (int i = 0; i < 20; i++) {
        delay(100);
        gps.stats(&s2, 0);
        if (s1 != s2) {
            // data is coming in
            if (!gpsData) gpsData = new GPS_DATA;
            memset(gpsData, 0, sizeof(GPS_DATA));
            if (buffered) {
                if (!nmeaBuffer) nmeaBuffer = new char[NMEA_BUF_SIZE];
                gpsFlags |= GF_BUFFERED;
            } 
            nmeaBytes = 0;
            return true;
        }
    }

    // when no data coming in
    gpsEnd();
    return false;
}

bool FreematicsESP32::gpsGetData(GPS_DATA** pgd)
{
    if (!gpsData) return false;
    gps.stats(&gpsData->sentences, &gpsData->errors);
    if (!gpsPendingData) return false;
    if (pgd) *pgd = gpsData;
    long lat, lng;
    gps.get_position(&lat, &lng, 0);
    if (gpsData->lat || gpsData->lng) {
        // filter out invalid coordinates
        if (abs(lat - gpsData->lat * 1000000) >= 100000) lat = 0;
        if (abs(lng - gpsData->lng * 1000000) >= 100000) lng = 0;
    }
    if (lat == 0 || lng == 0) return false;
    gpsData->ts = millis();
    gpsData->lat = (float)lat / 1000000;
    gpsData->lng = (float)lng / 1000000;
    gps.get_datetime((unsigned long*)&gpsData->date, (unsigned long*)&gpsData->time, 0);
    long alt = gps.altitude();
    if (alt != TinyGPS::GPS_INVALID_ALTITUDE) gpsData->alt = (float)alt / 100;
    unsigned long knot = gps.speed();
    if (knot != TinyGPS::GPS_INVALID_SPEED) gpsData->speed = (float)knot / 100;
    unsigned long course = gps.course();
    if (course < 36000) gpsData->heading = course / 100;
    unsigned short sat = gps.satellites();
    if (sat != TinyGPS::GPS_INVALID_SATELLITES) gpsData->sat = sat;
    unsigned long hdop = gps.hdop();
    gpsData->hdop = hdop > 2550 ? 255 : hdop / 10;
    gpsPendingData = 0;
    return true;
}

int FreematicsESP32::gpsGetNMEA(char* buffer, int bufsize)
{
    int bytes = 0;
    if (nmeaBytes > 0) {
        if (bufsize < nmeaBytes) {
            nmeaBufferMutex.lock();
            memcpy(buffer, nmeaBuffer, bytes = bufsize);
            memmove(nmeaBuffer, nmeaBuffer + bufsize, nmeaBytes -= bufsize);
            nmeaBufferMutex.unlock();
        } else {
            nmeaBufferMutex.lock();
            memcpy(buffer, nmeaBuffer, bytes = nmeaBytes);
            nmeaBytes = 0;
            nmeaBufferMutex.unlock();
        }
    }
    return bytes;
}

void FreematicsESP32::gpsSendCommand(const char* string, int len)
{
#if !GPS_SOFT_SERIAL
    if (taskGPS.running())
        uart_write_bytes(GPS_UART_NUM, string, len);
#endif
}

bool FreematicsESP32::xbBegin(unsigned long baudrate)
{
    uart_config_t uart_config = {
        .baud_rate = (int)baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    //Configure UART parameters
    uart_param_config(BEE_UART_NUM, &uart_config);
    //Set UART pins
    uart_set_pin(BEE_UART_NUM, PIN_BEE_UART_TXD, PIN_BEE_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver
    uart_driver_install(BEE_UART_NUM, UART_BUF_SIZE, 0, 0, NULL, 0);

#ifdef PIN_BEE_PWR
	pinMode(PIN_BEE_PWR, OUTPUT);
	digitalWrite(PIN_BEE_PWR, HIGH);
#endif
    return true;
}

void FreematicsESP32::xbWrite(const char* cmd)
{
    uart_write_bytes(BEE_UART_NUM, cmd, strlen(cmd));
#ifdef XBEE_DEBUG
	Serial.print("[SENT]");
	Serial.print(data);
	Serial.println("[/SENT]");
#endif
}

void FreematicsESP32::xbWrite(const char* data, int len)
{
    uart_write_bytes(BEE_UART_NUM, data, len);
}

int FreematicsESP32::xbRead(char* buffer, int bufsize, unsigned int timeout)
{
    int recv = 0;
    uint32_t t = millis();
    do {
        uint8_t c;
        int len = uart_read_bytes(BEE_UART_NUM, &c, 1, 0);
        if (len == 1) {
            if (c >= 0xA && c <= 0x7E) {
                buffer[recv++] = c;
            }
        } else if (recv > 0) {
            break;
        }
    } while (recv < bufsize && millis() - t < timeout);
    return recv;
}

int FreematicsESP32::xbReceive(char* buffer, int bufsize, unsigned int timeout, const char** expected, byte expectedCount)
{
    int bytesRecv = 0;
	uint32_t t = millis();
	do {
		if (bytesRecv >= bufsize - 16) {
			bytesRecv -= dumpLine(buffer, bytesRecv);
		}
		int n = xbRead(buffer + bytesRecv, bufsize - bytesRecv - 1, 50);
		if (n > 0) {
#ifdef XBEE_DEBUG
			Serial.print("[RECV]");
			buffer[bytesRecv + n] = 0;
			Serial.print(buffer + bytesRecv);
			Serial.println("[/RECV]");
#endif
			bytesRecv += n;
			buffer[bytesRecv] = 0;
			for (byte i = 0; i < expectedCount; i++) {
				// match expected string(s)
				if (expected[i] && strstr(buffer, expected[i])) return i + 1;
			}
		} else if (n == -1) {
			// an erroneous reading
#ifdef XBEE_DEBUG
			Serial.print("RECV ERROR");
#endif
			break;
		}
	} while (millis() - t < timeout);
	buffer[bytesRecv] = 0;
	return 0;
}

void FreematicsESP32::xbPurge()
{
    uart_flush(BEE_UART_NUM);
}

void FreematicsESP32::xbTogglePower()
{
#ifdef PIN_BEE_PWR
    digitalWrite(PIN_BEE_PWR, HIGH);
    delay(50);
#ifdef XBEE_DEBUG
	Serial.println("xBee power pin set to low");
#endif
	digitalWrite(PIN_BEE_PWR, LOW);
	delay(2000);
#ifdef XBEE_DEBUG
	Serial.println("xBee power pin set to high");
#endif
    digitalWrite(PIN_BEE_PWR, HIGH);
    delay(1000);
    digitalWrite(PIN_BEE_PWR, LOW);
#endif
}
