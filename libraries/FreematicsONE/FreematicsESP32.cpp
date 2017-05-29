/*************************************************************************
* Freematics ESP32 helper functions
* Distributed under BSD license
* Visit http://freematics.com for more information
* (C)2017 Developed by Stanley Huang <support@freematics.com.au>
*************************************************************************/

#include <Arduino.h>
#include <FreematicsONE.h>
#ifdef ESP32
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

#define UART_BUF_SIZE (1024)

static void gps_decode_task(void* arg)
{
    for (;;) {
        uint8_t c;
        int len = uart_read_bytes(GPS_UART_NUM, &c, 1, 1000 / portTICK_RATE_MS);
        if (len == -1) continue;
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

int bee_read(uint8_t* buffer, size_t bufsize, int timeout)
{
    return uart_read_bytes(BEE_UART_NUM, buffer, bufsize, timeout / portTICK_RATE_MS);
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

#else

bool Task::create(void (*task)(void*), const char* name, int priority)
{
  return false;
}

void Task::destroy()
{
}

void Task::sleep(uint32_t ms)
{
  delay(ms);
}

Mutex::Mutex()
{
}

void Mutex::lock()
{
}

void Mutex::unlock()
{
}

#endif
