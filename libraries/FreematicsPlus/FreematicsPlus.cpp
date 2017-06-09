/*************************************************************************
* Arduino library for Freematics ONE+ (ESP32)
* Distributed under BSD license
* Visit http://freematics.com for more information
* (C)2017 Developed by Stanley Huang <support@freematics.com.au>
*************************************************************************/

#ifdef ESP32
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
#include "FreematicsPlus.h"
#include "TinyGPS.h"

static TinyGPS gps;
static bool newGPSData = false;
static TaskHandle_t xGPSTaskHandle = 0;

#define BEE_UART_PIN_RXD  (16)
#define BEE_UART_PIN_TXD  (17)
#define BEE_UART_NUM UART_NUM_1

#define GPS_UART_PIN_RXD  (32)
#define GPS_UART_PIN_TXD  (33)
#define GPS_UART_NUM UART_NUM_2

#define UART_BUF_SIZE (2048)

static void gps_decode_task(void* arg)
{
    for (;;) {
        uint8_t c;
        int len = uart_read_bytes(GPS_UART_NUM, &c, 1, 1000 / portTICK_RATE_MS);
        if (len == -1) continue;
        if (c < 0xA || c > 0x7E) continue;
        if (gps.encode(c)) {
            newGPSData = true;
        }
        //Serial.println(gps._failed_checksum);
    }
}

bool gps_get_data(GPS_DATA* gdata)
{
	if (!newGPSData) {
		return false;
	}
    gps.get_datetime((unsigned long*)&gdata->date, (unsigned long*)&gdata->time, 0);
    gps.get_position((long*)&gdata->lat, (long*)&gdata->lng, 0);
    gdata->sat = gps.satellites();
    gdata->speed = gps.speed() * 1852 / 100000; /* km/h */
    gdata->alt = gps.altitude();
    gdata->heading = gps.course() / 100;
    newGPSData = false;
    return true;
}

int gps_write_string(const char* string)
{
    return uart_write_bytes(BEE_UART_NUM, string, strlen(string));
}

bool gps_decode_start()
{
    if (xGPSTaskHandle) return true;

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

    // check input
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

    xTaskCreate(gps_decode_task, "gps_decode_task", 1024, NULL, 10, &xGPSTaskHandle);
    return true;
}

void bee_start()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
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

int bee_read(uint8_t* buffer, size_t bufsize, int timeout)
{
    int len = uart_read_bytes(BEE_UART_NUM, buffer, bufsize, timeout / portTICK_RATE_MS);
    if (len > 0) {
      // strip invalid characters
      int j = 0;
      for (int i = 0; i < len; i++) {
        if (buffer[i] >= 0xA && buffer[i] <= 0x7E) {
          buffer[j++] = buffer[i];
        }
      }
      return j;
    } else {
      return 0;
    }
}

void bee_flush()
{
    uart_flush(BEE_UART_NUM);
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
	char buf[128];
	if (baudrate) {
		pinMode(PIN_GPS_POWER, OUTPUT);
		// turn on GPS power
		digitalWrite(PIN_GPS_POWER, HIGH);
		if (gps_decode_start()) {
			// success
			return true;
		}
	} else {
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
	bee_start();
  return true;
}

void CFreematicsESP32::xbWrite(const char* cmd)
{
	bee_write_string(cmd);
#ifdef XBEE_DEBUG
	Serial.print("<<<");
	Serial.print(cmd);
	Serial.println("<<<");
#endif
}

void CFreematicsESP32::xbWrite(const char* data, int len)
{
	bee_write_data((uint8_t*)data, len);
}

int CFreematicsESP32::xbRead(char* buffer, int bufsize, int timeout)
{
	return bee_read((uint8_t*)buffer, bufsize, timeout);
}

byte CFreematicsESP32::xbReceive(char* buffer, int bufsize, int timeout, const char** expected, byte expectedCount)
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
			Serial.print(">>>");
			buffer[bytesRecv + n] = 0;
			Serial.print(buffer + bytesRecv);
			Serial.println(">>>");
#endif
			bytesRecv += n;
			buffer[bytesRecv] = 0;
            for (byte i = 0; i < expectedCount; i++) {
                // match expected string(s)
                if (expected[i] && strstr(buffer, expected[i])) return i + 1;
            }
		} else if (n == -1) {
            // an erroneous reading
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
#ifdef XBEE_DEBUG
	Serial.println("xBee power pin set to low");
#endif
	digitalWrite(PIN_XBEE_PWR, LOW);
	sleep(2000);
#ifdef XBEE_DEBUG
	Serial.println("xBee power pin set to high");
#endif
	digitalWrite(PIN_XBEE_PWR, HIGH);
#endif
}

#endif
