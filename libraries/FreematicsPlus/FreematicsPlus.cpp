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
#include "esp_bt.h"
#include "bta_api.h"
#include "esp_gatt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "soc/uart_struct.h"
#include "FreematicsSD.h"
#include "FreematicsPlus.h"
#include "FreematicsGPS.h"

static TinyGPS gps;
static int validGPSData = 0;
static int sentencesGPSData = 0;
static Task taskGPS;

#if GPS_SOFT_SERIAL

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

void gps_decode_task(void* inst)
{
#if GPS_SOFT_SERIAL
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#endif
    uint8_t pattern[] = {'$', '\n'};
    int idx = 0;
    for (;;) {
        uint8_t c = 0;
#if GPS_SOFT_SERIAL
        while (readRxPin());
        uint32_t start = getCycleCount();
        taskYIELD();
        taskENTER_CRITICAL(&mux);
        for (uint32_t i = 1; i <= 7; i++) {
            while (getCycleCount() - start < i * F_CPU / GPS_BAUDRATE + F_CPU / GPS_BAUDRATE / 3);
            c = (c | readRxPin()) >> 1;
        }
        taskEXIT_CRITICAL(&mux);
#else
        int len = uart_read_bytes(GPS_UART_NUM, &c, 1, 60000 / portTICK_RATE_MS);
        if (len != 1) continue;
#endif
        if (c) {
            Serial.write(c);
            if (c == pattern[idx]) {
                if (++idx >= sizeof(pattern)) {
                    idx = 0;
                    sentencesGPSData++;
                }
            }
            if (gps.encode(c)) {
                validGPSData++;
            }
        }
#if GPS_SOFT_SERIAL
        while (getCycleCount() - start < (uint32_t)9 * F_CPU / GPS_BAUDRATE + F_CPU / GPS_BAUDRATE / 2) taskYIELD();
#endif
    }
}

int gps_get_data(GPS_DATA* gdata)
{
    if (!validGPSData) return 0;
    gps.get_datetime((unsigned long*)&gdata->date, (unsigned long*)&gdata->time, 0);
    gps.get_position((long*)&gdata->lat, (long*)&gdata->lng, 0);
    gdata->speed = gps.speed() * 1852 / 100000; /* km/h */
    gdata->alt = gps.altitude();
    gdata->heading = gps.course() / 100;
    gdata->sat = gps.satellites();
    if (gdata->sat > 200) gdata->sat = 0;
    int ret = validGPSData;
    validGPSData = 0;
    return ret;
}

int gps_write_string(const char* string)
{
    if (!taskGPS.running()) return 0;
    return uart_write_bytes(GPS_UART_NUM, string, strlen(string));
}

bool gps_decode_start()
{
    // quick check of input data format
    validGPSData = 0;
    sentencesGPSData = 0;

    if (taskGPS.running()) {
        taskGPS.resume();
    } else {
        // start GPS decoding thread if not started
        taskGPS.create(gps_decode_task, "GPS", 1);
    }

    for (int i = 0; i < 20 && sentencesGPSData < 3; i++) {
        delay(100);
    }
    return sentencesGPSData >= 3;
}

void gps_decode_stop()
{
    taskGPS.suspend();
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
    uart_set_pin(BEE_UART_NUM, PIN_BEE_UART_TXD, PIN_BEE_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver
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

extern "C" {
uint8_t temprature_sens_read();
int32_t hall_sens_read();
}

// get chip temperature sensor
uint8_t readChipTemperature()
{
  return temprature_sens_read();
}

int32_t readChipHallSensor()
{
  return hall_sens_read();
}

uint16_t getFlashSize()
{
#if 0
    char out;
    uint16_t size = 16384;
    do {
        if (spi_flash_read((size_t)size * 1024 - 1, &out, 1) == ESP_OK)
            return size;
    } while ((size >>= 1));
    return 0;
#endif
    return (spi_flash_get_chip_size() >> 10);
}

bool Task::create(void (*task)(void*), const char* name, int priority)
{
    if (xHandle) return false;
    /* Create the task, storing the handle. */
    BaseType_t xReturned = xTaskCreate(task, name, 1024, (void*)this, priority, &xHandle);
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
#if GPS_SOFT_SERIAL
    pinMode(PIN_GPS_UART_RXD, INPUT);
#else
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
    uart_set_pin(GPS_UART_NUM, PIN_GPS_UART_TXD, PIN_GPS_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver
    uart_driver_install(GPS_UART_NUM, UART_BUF_SIZE, 0, 0, NULL, 0);
#endif

    // Configure dynamic frequency scaling
    rtc_cpu_freq_t max_freq;
    rtc_clk_cpu_freq_from_mhz(cpuMHz, &max_freq);
    esp_pm_config_esp32_t pm_config = {
            .max_cpu_freq = max_freq,
            .min_cpu_freq = RTC_CPU_FREQ_XTAL
    };
    esp_err_t ret = esp_pm_configure(&pm_config);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        //Serial.println("Power-saving disabled");
    }

    // set watchdog timeout to 30 seconds
    esp_task_wdt_init(30, 0);
}

bool FreematicsESP32::gpsInit(unsigned long baudrate)
{
	bool success = false;
	if (baudrate) {
		pinMode(PIN_GPS_POWER, OUTPUT);
		// turn on GPS power
		digitalWrite(PIN_GPS_POWER, HIGH);
        delay(10);
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

int FreematicsESP32::gpsGetData(GPS_DATA* gdata)
{
  return gps_get_data(gdata);
}

void FreematicsESP32::gpsSendCommand(const char* cmd)
{
	gps_write_string(cmd);
}

bool FreematicsESP32::xbBegin(unsigned long baudrate)
{
#ifdef PIN_BEE_PWR
	pinMode(PIN_BEE_PWR, OUTPUT);
	digitalWrite(PIN_BEE_PWR, HIGH);
#endif
	bee_start(baudrate);
  return true;
}

void FreematicsESP32::xbWrite(const char* cmd)
{
	bee_write_string(cmd);
#ifdef XBEE_DEBUG
	Serial.print("[SENT]");
	Serial.print(data);
	Serial.println("[/SENT]");
#endif
}

void FreematicsESP32::xbWrite(const char* data, int len)
{
	bee_write_data((uint8_t*)data, len);
}

int FreematicsESP32::xbRead(char* buffer, int bufsize, unsigned int timeout)
{
	return bee_read((uint8_t*)buffer, bufsize, timeout);
}

int FreematicsESP32::xbReceive(char* buffer, int bufsize, unsigned int timeout, const char** expected, byte expectedCount)
{
    int bytesRecv = 0;
	uint32_t t = millis();
	do {
		if (bytesRecv >= bufsize - 16) {
			bytesRecv -= dumpLine(buffer, bytesRecv);
		}
		int n = bee_read((uint8_t*)buffer + bytesRecv, bufsize - bytesRecv - 1, 100);
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
	bee_flush();
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

extern "C" {
void gatts_init(const char* device_name);
int gatts_send(uint8_t* data, size_t len);
}

bool GATTServer::initBLE()
{
    btStart();
    esp_err_t ret = esp_bluedroid_init();
    if (ret) {
        Serial.println("Bluetooth failed");
        return false;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        Serial.println("Error enabling bluetooth");
        return false;
    }
    return true;
}

static GATTServer* gatts_inst;

bool GATTServer::begin(const char* deviceName)
{
    gatts_inst = this;
    if (!initBLE()) return false;
    gatts_init(deviceName);
    return true;
}

bool GATTServer::send(uint8_t* data, size_t len)
{
    return gatts_send(data, len);
}

size_t GATTServer::write(uint8_t c)
{
    bool success = false;
    if (sendbuf.length() >= MAX_BLE_MSG_LEN) {
        success = send((uint8_t*)sendbuf.c_str(), sendbuf.length());
        sendbuf = "";
    }
    sendbuf += (char)c;
    if (c == '\n') {
        success = send((uint8_t*)sendbuf.c_str(), sendbuf.length());
        sendbuf = "";
    }
    return success ? sendbuf.length() : 0;
}

extern "C" {

size_t gatts_read_callback(uint8_t* buffer, size_t len)
{
	if (gatts_inst) {
		return gatts_inst->onRequest(buffer, len);
	} else {
		return 0;
	}
}

void gatts_write_callback(uint8_t* data, size_t len)
{
    if (gatts_inst) gatts_inst->onReceive(data, len);
}

}
