/*************************************************************************
* Arduino library for Freematics ONE+ (ESP32)
* Distributed under BSD license
* Visit http://freematics.com for more information
* (C)2017 Developed by Stanley Huang <support@freematics.com.au>
*************************************************************************/

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "soc/uart_struct.h"
#include "FreematicsSD.h"
#include "FreematicsPlus.h"
#include "FreematicsGPS.h"

static TinyGPS* gps = 0;
static bool newGPSData = false;

#define BEE_UART_PIN_RXD  (16)
#define BEE_UART_PIN_TXD  (17)
#define BEE_UART_NUM UART_NUM_1

#define GPS_UART_PIN_RXD  (32)
#define GPS_UART_PIN_TXD  (33)
#define GPS_UART_NUM UART_NUM_2

#define UART_BUF_SIZE (2048)

void gps_decode_task(int timeout)
{
    if (!gps) {
        delay(timeout);
        return;
    }
    uint8_t buf[32];
    int len = uart_read_bytes(GPS_UART_NUM, buf, sizeof(buf), timeout / portTICK_RATE_MS);
    for (int n = 0; n < len; n++) {
        if (gps->encode(buf[n])) {
            newGPSData = true;
        }
    }
}

bool gps_get_data(GPS_DATA* gdata)
{
	if (!newGPSData || !gps) {
		return false;
	}
    gps->get_datetime((unsigned long*)&gdata->date, (unsigned long*)&gdata->time, 0);
    gps->get_position((long*)&gdata->lat, (long*)&gdata->lng, 0);
    gdata->speed = gps->speed() * 1852 / 100000; /* km/h */
    gdata->alt = gps->altitude();
    gdata->heading = gps->course() / 100;
    gdata->sat = gps->satellites();
    if (gdata->sat > 200) gdata->sat = 0;
    newGPSData = false;
    return true;
}

int gps_write_string(const char* string)
{
    if (!gps) return 0;
    return uart_write_bytes(BEE_UART_NUM, string, strlen(string));
}

void gps_decode_stop()
{
    if (gps) {
        uart_driver_delete(GPS_UART_NUM);
        delete gps;
        gps = 0;
    }
}

bool gps_decode_start()
{
    if (gps) return true;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    //Configure UART parameters
    uart_param_config(GPS_UART_NUM, &uart_config);
    //Set UART pins
    uart_set_pin(GPS_UART_NUM, GPS_UART_PIN_TXD, GPS_UART_PIN_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver (we don't need an event queue here)
    //In this example we don't even use a buffer for sending data.
    uart_driver_install(GPS_UART_NUM, UART_BUF_SIZE, 0, 0, NULL, 0);

    // quick check of input data format
    uint32_t t = millis();
    uint8_t match[] = {'$', ',', '\n'};
    int idx = 0;
    do {
        uint8_t c;
        if (uart_read_bytes(GPS_UART_NUM, &c, 1, 200 / portTICK_RATE_MS) == -1) continue;
        if (c == match[idx]) idx++;
    } while (millis() - t < GPS_INIT_TIMEOUT && idx < sizeof(match));
    if (idx < sizeof(match)) {
        // no matching pattern
        uart_driver_delete(GPS_UART_NUM);
        return false;
    }

    gps = new TinyGPS;
    return true;
}

void bee_start(int baudrate)
{
    uart_config_t uart_config = {
        .baud_rate = baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    //Configure UART parameters
    uart_param_config(BEE_UART_NUM, &uart_config);
    //Set UART pins
    uart_set_pin(BEE_UART_NUM, BEE_UART_PIN_TXD, BEE_UART_PIN_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver (we don't need an event queue here)
    //In this example we don't even use a buffer for sending data.
    uart_driver_install(BEE_UART_NUM, UART_BUF_SIZE, 0, 0, NULL, 0);
}

int bee_write_string(const char* string)
{
    return uart_write_bytes(BEE_UART_NUM, string, strlen(string));
}

int bee_write_data(uint8_t* data, int len)
{
    return uart_write_bytes(BEE_UART_NUM, (const char*)data, len);
}

int bee_read(uint8_t* buffer, size_t bufsize, unsigned int timeout)
{
    int recv = 0;
    uint32_t t = millis();
    do {
        uint8_t c;
        int len = uart_read_bytes(BEE_UART_NUM, &c, 1, 0);
        gps_decode_task(0);
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

void bee_flush()
{
    uart_flush(BEE_UART_NUM);
}

extern "C" uint8_t temprature_sens_read();

// get chip temperature sensor
uint8_t readChipTemperature()
{
  return temprature_sens_read();
}

bool Task::create(void (*task)(void*), const char* name, int priority)
{
    if (xHandle) return false;
    /* Create the task, storing the handle. */
    BaseType_t xReturned = xTaskCreate(task, name, 4096, (void*)this, priority, &xHandle);
    return xReturned == pdPASS;
}

void Task::destroy()
{
  if (xHandle) {
    vTaskDelete((TaskHandle_t)xHandle);
    xHandle = 0;
  }
}

void Task::sleep(uint32_t ms)
{
  vTaskDelay(ms / portTICK_PERIOD_MS);
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

bool CFreematicsESP32::gpsInit(unsigned long baudrate)
{
	bool success = false;
	if (baudrate) {
		pinMode(PIN_GPS_POWER, OUTPUT);
		// turn on GPS power
		digitalWrite(PIN_GPS_POWER, HIGH);
		if (gps_decode_start()) {
			// success
			return true;
		}
	} else {
    gps_decode_stop();
		success = true;
	}
	//turn off GPS power
	digitalWrite(PIN_GPS_POWER, LOW);
	return success;
}

bool CFreematicsESP32::gpsGetData(GPS_DATA* gdata)
{
  return gps_get_data(gdata);
}

void CFreematicsESP32::gpsSendCommand(const char* cmd)
{
	gps_write_string(cmd);
}

bool CFreematicsESP32::xbBegin(unsigned long baudrate)
{
#ifdef PIN_XBEE_PWR
	pinMode(PIN_XBEE_PWR, OUTPUT);
	digitalWrite(PIN_XBEE_PWR, HIGH);
#endif
	bee_start(baudrate);
  return true;
}

void CFreematicsESP32::xbWrite(const char* cmd)
{
	bee_write_string(cmd);
#ifdef XBEE_DEBUG
	Serial.print("[SENT]");
	Serial.print(data);
	Serial.println("[/SENT]");
#endif
}

void CFreematicsESP32::xbWrite(const char* data, int len)
{
	bee_write_data((uint8_t*)data, len);
}

int CFreematicsESP32::xbRead(char* buffer, int bufsize, unsigned int timeout)
{
	return bee_read((uint8_t*)buffer, bufsize, timeout);
}

int CFreematicsESP32::xbReceive(char* buffer, int bufsize, unsigned int timeout, const char** expected, byte expectedCount)
{
    int bytesRecv = 0;
	uint32_t t = millis();
	do {
		if (bytesRecv >= bufsize - 16) {
			bytesRecv -= dumpLine(buffer, bytesRecv);
		}
		int n = xbRead(buffer + bytesRecv, bufsize - bytesRecv - 1, 100);
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

void CFreematicsESP32::xbPurge()
{
	bee_flush();
}

void CFreematicsESP32::xbTogglePower()
{
#ifdef PIN_XBEE_PWR
  digitalWrite(PIN_XBEE_PWR, HIGH);
  sleep(50);
#ifdef XBEE_DEBUG
	Serial.println("xBee power pin set to low");
#endif
	digitalWrite(PIN_XBEE_PWR, LOW);
	sleep(2000);
#ifdef XBEE_DEBUG
	Serial.println("xBee power pin set to high");
#endif
  digitalWrite(PIN_XBEE_PWR, HIGH);
  sleep(1000);
  digitalWrite(PIN_XBEE_PWR, LOW);
#endif
}

void CFreematicsESP32::sleep(unsigned int ms)
{
	uint32_t t = millis();
	for (;;) {
		uint32_t elapsed = millis() - t;
		if (elapsed >= ms) break;
		gps_decode_task(ms - elapsed);
	}
}

void CFreematicsESP32::hibernate(unsigned int ms)
{
  // this puts ESP32 into sleep mode but will also turn off BLE
  esp_sleep_enable_timer_wakeup((unsigned long)ms * 1000);
  esp_light_sleep_start();
}
