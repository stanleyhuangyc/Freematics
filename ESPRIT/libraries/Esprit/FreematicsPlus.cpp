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

#ifdef ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "soc/uart_struct.h"
#include "FreematicsPlus.h"

#define BEE_UART_PIN_RXD  (16)
#define BEE_UART_PIN_TXD  (17)
#define BEE_UART_NUM UART_NUM_1

#define GPS_UART_PIN_RXD  (32)
#define GPS_UART_PIN_TXD  (33)
#define GPS_UART_NUM UART_NUM_2

#define UART_BUF_SIZE (2048)

static int dumpLine(char* buffer, int len)
{
	int bytesToDump = len >> 1;
	for (int i = 0; i < len; i++) {
		// find out first line end and discard the first line
		if (buffer[i] == '\r' || buffer[i] == '\n') {
			// go through all following \r or \n if any
			while (++i < len && (buffer[i] == '\r' || buffer[i] == '\n'));
			bytesToDump = i;
			break;
		}
	}
	memmove(buffer, buffer + bytesToDump, len - bytesToDump);
	return bytesToDump;
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

int bee_read(uint8_t* buffer, size_t bufsize, unsigned int timeout)
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

void CFreematicsESP32::sleep(unsigned int ms)
{
  delay(ms);
}

void CFreematicsESP32::hibernate(unsigned int ms)
{
  // this puts ESP32 into sleep mode but will also turn off BLE
  esp_sleep_enable_timer_wakeup((unsigned long)ms * 1000);
  esp_light_sleep_start();
}

#endif
