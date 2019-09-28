/*************************************************************************
* Arduino library for ESP32 based Freematics ONE+ and Freematics Esprit
* Distributed under BSD license
* Visit https://freematics.com for more information
* (C)2017-2019 Developed by Stanley Huang <stanley@freematics.com.au>
*************************************************************************/

#include <Arduino.h>
#include <SPI.h>
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

#define VERBOSE_LINK 0
#define VERBOSE_XBEE 0

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

bool CLink_UART::begin(unsigned int baudrate, int rxPin, int txPin)
{
#if VERBOSE_LINK
    Serial.println("[UART BEGIN]");
#endif
    uart_config_t uart_config = {
        .baud_rate = (int)baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    //Configure UART parameters
    uart_param_config(LINK_UART_NUM, &uart_config);
    //Set UART pins
    uart_set_pin(LINK_UART_NUM, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver
    if (uart_driver_install(LINK_UART_NUM, LINK_UART_BUF_SIZE, 0, 0, NULL, 0) != ESP_OK)
		return false;
	return true;
}

void CLink_UART::end()
{
#if VERBOSE_LINK
    Serial.println("[UART END]");
#endif
	uart_driver_delete(LINK_UART_NUM);
}

int CLink_UART::receive(char* buffer, int bufsize, unsigned int timeout)
{
	unsigned char n = 0;
	unsigned long startTime = millis();
	unsigned long elapsed;
	for (;;) {
		elapsed = millis() - startTime;
		if (elapsed > timeout) break;
		if (n >= bufsize - 1) break;
		int len = uart_read_bytes(LINK_UART_NUM, (uint8_t*)buffer + n, bufsize - n - 1, 1);
		if (len < 0) break;
		if (len == 0) continue;
		buffer[n + len] = 0;
		if (strchr(buffer + n, '>')) {
			n += len;
			break;
		}
		n += len;
		if (strstr(buffer, "...")) {
			buffer[0] = 0;
			n = 0;
			timeout += OBD_TIMEOUT_LONG;
		}
	}
#if VERBOSE_LINK
	Serial.print("[UART RECV]");
	Serial.println(buffer);
#endif
	return n;
}

void CLink_UART::send(const char* str)
{
#if VERBOSE_LINK
	Serial.print("[UART SEND]");
	Serial.println(str);
#endif
	uart_write_bytes(LINK_UART_NUM, str, strlen(str));
}

int CLink_UART::sendCommand(const char* cmd, char* buf, int bufsize, unsigned int timeout)
{
	send(cmd);
	return receive(buf, bufsize, timeout);
}

int CLink_UART::read()
{
    uint8_t c;
    if (uart_read_bytes(LINK_UART_NUM, &c, 1, 1) == 1)
        return c;
    else
        return -1;
}

bool CLink_UART::changeBaudRate(unsigned int baudrate)
{
	char buf[32];
	sprintf(buf, "ATBR1 %u\r", baudrate);
	sendCommand(buf, buf, sizeof(buf), 1000);
	delay(50);
	end();
	return begin(baudrate);
}

bool CLink_SPI::begin(unsigned int freq, int rxPin, int txPin)
{
#if VERBOSE_LINK
    Serial.println("[SPI BEGIN]");
#endif
	pinMode(PIN_LINK_SPI_READY, INPUT);
	pinMode(PIN_LINK_SPI_CS, OUTPUT);
	digitalWrite(PIN_LINK_SPI_CS, HIGH);
	SPI.begin();
	SPI.setFrequency(freq);
	return true;
}

void CLink_SPI::end()
{
#if VERBOSE_LINK
    Serial.println("[SPI END]");
#endif
	SPI.end();
}

int CLink_SPI::receive(char* buffer, int bufsize, unsigned int timeout)
{
	int n = 0;
	bool eos = false;
	bool matched = false;
	portMUX_TYPE m = portMUX_INITIALIZER_UNLOCKED;
	uint32_t t = millis();
	do {
		while (digitalRead(PIN_LINK_SPI_READY) == HIGH) {
			if (millis() - t > 3000) return -1;
            delay(1);
		}
		portENTER_CRITICAL(&m);
		digitalWrite(PIN_LINK_SPI_CS, LOW);
		while (digitalRead(PIN_LINK_SPI_READY) == LOW && millis() - t < timeout) {
			char c = SPI.transfer(' ');
			if (c == 0 && c == 0xff) continue;
			if (!eos) eos = (c == 0x9);
			if (eos) continue;
			if (!matched) {
				// match header
				if (n == 0 && c != header[0]) continue;
				if (n == bufsize - 1) continue;
				buffer[n++] = c;
				if (n == sizeof(header)) {
					matched = memcmp(buffer, header, sizeof(header)) == 0;
					if (matched) {
						n = 0;
					} else {
						memmove(buffer, buffer + 1, --n);
					}
				}
				continue;
			}
			if (n > 3 && c == '.' && buffer[n - 1] == '.' && buffer[n - 2] == '.') {
				// SEARCHING...
				n = 0;
				timeout += OBD_TIMEOUT_LONG;
			} else {
				if (n == bufsize - 1) {
					int bytesDumped = dumpLine(buffer, n);
					n -= bytesDumped;
#if VERBOSE_LINK
					Serial.println("[SPI BUFFER FULL]");
#endif
				}
				buffer[n++] = c;
			}
		}
		digitalWrite(PIN_LINK_SPI_CS, HIGH);
		portEXIT_CRITICAL(&m);
	} while (!eos && millis() - t < timeout);
#if VERBOSE_LINK
	if (!eos) {
		// timed out
		Serial.println("[SPI RECV TIMEOUT]");
	}
#endif
	buffer[n] = 0;
#if VERBOSE_LINK
	Serial.print("[SPI RECV]");
	Serial.println(buffer);
#endif
	// wait for READY pin to restore high level so SPI bus is released
    if (eos) while (digitalRead(PIN_LINK_SPI_READY) == LOW) delay(1);
	return n;
}

void CLink_SPI::send(const char* str)
{
	portMUX_TYPE m = portMUX_INITIALIZER_UNLOCKED;
#if VERBOSE_LINK
	Serial.print("[SPI SEND]");
	Serial.println(str);
#endif
	int len = strlen(str);
	uint8_t tail = 0x1B;
	portENTER_CRITICAL(&m);
	digitalWrite(PIN_LINK_SPI_CS, LOW);
	delay(1);
	SPI.writeBytes((uint8_t*)header, sizeof(header));
	SPI.writeBytes((uint8_t*)str, len);
	SPI.writeBytes((uint8_t*)&tail, 1);
	delay(1);
	digitalWrite(PIN_LINK_SPI_CS, HIGH);
	portEXIT_CRITICAL(&m);
}

int CLink_SPI::sendCommand(const char* cmd, char* buf, int bufsize, unsigned int timeout)
{
	uint32_t t = millis();
	int n = 0;
	for (byte i = 0; i < 30 && millis() - t < timeout; i++) {
		send(cmd);
		n = receive(buf, bufsize, timeout);
		if (n == -1) {
			Serial.print('_');
			n = 0;
			continue;
		}
		if (n == 0 || (buf[1] != 'O' && !memcmp(buf + 5, "NO DATA", 7))) {
			// data not ready
			delay(50);
		} else {
	  		break;
		}
	}
	return n;
}

void FreematicsESP32::gpsEnd()
{
    // uninitialize
    if (!(m_flags & GNSS_USE_LINK)) {
        taskGPS.destroy();
        if (!(m_flags & GNSS_SOFT_SERIAL)) uart_driver_delete(GPS_UART_NUM);
        if (nmeaBuffer) {
            nmeaBufferMutex.lock();
            delete nmeaBuffer;
            nmeaBuffer = 0;
            nmeaBufferMutex.unlock();
        }
    }
	//turn off GPS power
    if (m_pinGPSPower) {
        digitalWrite(m_pinGPSPower, LOW);
    } else if (link) {
        char buf[16];
        link->sendCommand("ATGPSOFF", buf, sizeof(buf), 500);
    }
}

bool FreematicsESP32::gpsBegin(int baudrate, bool buffered)
{
    if (m_pinGPSPower) pinMode(m_pinGPSPower, OUTPUT);
    if (m_flags & GNSS_USE_LINK) {
        if (m_pinGPSPower) {
            digitalWrite(m_pinGPSPower, HIGH);
            delay(100);
        } else if (link) {
            char buf[16];
            link->sendCommand("ATGPSON", buf, sizeof(buf), 500);
        } 
    } else if (!(m_flags & GNSS_SOFT_SERIAL)) {
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
        if (m_pinGPSPower) digitalWrite(m_pinGPSPower, HIGH);
        delay(100);
        // start decoding task
        taskGPS.create(gps_decode_task, "GPS", 1);
    } else {
        pinMode(PIN_GPS_UART_RXD, INPUT);
        pinMode(PIN_GPS_UART_TXD, OUTPUT);
        setTxPinHigh();

        // turn on GPS power
        if (m_pinGPSPower) digitalWrite(m_pinGPSPower, LOW);
        delay(10);
        if (m_pinGPSPower) digitalWrite(m_pinGPSPower, HIGH);
        delay(100);

        // start GPS decoding task (soft serial)
        taskGPS.create(gps_soft_decode_task, "GPS", 1);
    }

    if (!(m_flags & GNSS_USE_LINK)) {
        // test run for a while to see if there is data decoded
        uint16_t s1 = 0, s2 = 0;
        gps.stats(&s1, 0);
        for (int i = 0; i < 10; i++) {
            if (m_flags & GNSS_SOFT_SERIAL) {
                // switch M8030 GNSS to 38400bps
                const uint8_t packet1[] = {0x0, 0x0, 0xB5, 0x62, 0x06, 0x0, 0x14, 0x0, 0x01, 0x0, 0x0, 0x0, 0xD0, 0x08, 0x0, 0x0, 0x0, 0x96, 0x0, 0x0, 0x7, 0x0, 0x3, 0x0, 0x0, 0x0, 0x0, 0x0, 0x93, 0x90};
                const uint8_t packet2[] = {0xB5, 0x62, 0x06, 0x0, 0x1, 0x0, 0x1, 0x8, 0x22};
                for (int i = 0; i < sizeof(packet1); i++) softSerialTx(baudrate, packet1[i]);
                delay(10);
                for (int i = 0; i < sizeof(packet2); i++) softSerialTx(baudrate, packet2[i]);
                delay(280);
            }
            delay(200);
            gps.stats(&s2, 0);
            if (s1 != s2) {
                // data is coming in
                if (!gpsData) gpsData = new GPS_DATA;
                memset(gpsData, 0, sizeof(GPS_DATA));
                if (buffered) {
                    if (!nmeaBuffer) nmeaBuffer = new char[NMEA_BUF_SIZE];
                } 
                nmeaBytes = 0;
                return true;
            }
        }

        // when no data coming in
        gpsEnd();
        return false;
    } else {
        uint32_t t = millis();
        char buf[128];
        bool success = false;
        do {
            if (gpsGetNMEA(buf, sizeof(buf)) > 0 && strstr(buf, ("$G"))) {
                success = true;
                break;
            }
        } while (millis() - t < 2000);
        if (success) {
            gpsData = new GPS_DATA;
            memset(gpsData, 0, sizeof(GPS_DATA));
            return true;
        }
        return false;
    }
}

bool FreematicsESP32::gpsGetData(GPS_DATA** pgd)
{
    if (!gpsData) return false;
    if (pgd) *pgd = gpsData;
    if (m_flags & GNSS_USE_LINK) {
        char buf[160];
        if (link->sendCommand("ATGPS\r", buf, sizeof(buf), 100) == 0) {
            return false;
        }
        char *s = strstr(buf, "$GNIFO,");
        if (!s) return false;
        s += 7;
        float lat = 0;
        float lng = 0;
        float alt = 0;
        bool good = false;
        do {
            uint32_t date = atoi(s);
            if (!(s = strchr(s, ','))) break;
            uint32_t time = atoi(++s);
            if (!(s = strchr(s, ','))) break;
            if (date % 100 <= 19) break;
            gpsData->date = date;
            gpsData->time = time;
            lat = (float)atoi(++s) / 1000000;
            if (!(s = strchr(s, ','))) break;
            lng = (float)atoi(++s) / 1000000;
            if (!(s = strchr(s, ','))) break;
            alt = (float)atoi(++s) / 100;
            good = true;
            if (!(s = strchr(s, ','))) break;
            gpsData->speed = (float)atoi(++s) / 100;
            if (!(s = strchr(s, ','))) break;
            gpsData->heading = atoi(++s) / 100;
            if (!(s = strchr(s, ','))) break;
            gpsData->sat = atoi(++s);
            if (!(s = strchr(s, ','))) break;
            gpsData->hdop = atoi(++s);
        } while(0);
        if (good && (gpsData->lat || gpsData->lng)) {
            // filter out invalid coordinates
            good = (abs(lat - gpsData->lat * 1000000) < 100000 && abs(lng - gpsData->lng * 1000000) < 100000);
        }
        if (!good) return false;
        gpsData->lat = lat;
        gpsData->lng = lng;
        gpsData->alt = alt;
        return true;
    } else {
        gps.stats(&gpsData->sentences, &gpsData->errors);
        if (!gpsPendingData) return false;
        long lat, lng;
        bool good = true;
        gps.get_position(&lat, &lng, 0);
        if (gpsData->lat || gpsData->lng) {
            // filter out invalid coordinates
            good = (abs(lat - gpsData->lat * 1000000) < 100000 && abs(lng - gpsData->lng * 1000000) < 100000);
        }
        if (!good) return false;
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
}

int FreematicsESP32::gpsGetNMEA(char* buffer, int bufsize)
{
    if (m_flags & GNSS_USE_LINK) {
        return link->sendCommand("ATGRR\r", buffer, bufsize, 200);
    } else {
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
}

void FreematicsESP32::gpsSendCommand(const char* string, int len)
{
#if !GPS_SOFT_SERIAL
    if (taskGPS.running())
        uart_write_bytes(GPS_UART_NUM, string, len);
#endif
}

bool FreematicsESP32::xbBegin(unsigned long baudrate, int pinRx, int pinTx)
{
    uart_config_t uart_config = {
        .baud_rate = (int)baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    if (version >= 14) {
        pinRx = PIN_BEE_UART_RXD3;
        pinTx = PIN_BEE_UART_TXD3;
    } else if (version == 13) {
        pinRx = PIN_BEE_UART_RXD2;
        pinTx = PIN_BEE_UART_TXD2;
    }

#if VERBOSE_XBEE
    Serial.print("Bee Rx:");
    Serial.print(pinRx);
    Serial.print(" Tx:");
    Serial.println(pinTx);
#endif
    //Configure UART parameters
    uart_param_config(BEE_UART_NUM, &uart_config);
    //Set UART pins
    uart_set_pin(BEE_UART_NUM, pinTx, pinRx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver
    uart_driver_install(BEE_UART_NUM, UART_BUF_SIZE, 0, 0, NULL, 0);

#ifdef PIN_BEE_PWR
	pinMode(PIN_BEE_PWR, OUTPUT);
	digitalWrite(PIN_BEE_PWR, LOW);
#endif
    return true;
}

void FreematicsESP32::xbEnd()
{
    uart_driver_delete(BEE_UART_NUM);
    digitalWrite(PIN_BEE_PWR, LOW);
}

void FreematicsESP32::xbWrite(const char* cmd)
{
    uart_write_bytes(BEE_UART_NUM, cmd, strlen(cmd));
#if VERBOSE_XBEE
    Serial.print("=== SENT@");
    Serial.print(millis());
    Serial.println(" ===");
	Serial.println(cmd);
	Serial.println("==================");
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
#if VERBOSE_XBEE
			Serial.print("=== RECV@");
            Serial.print(millis());
            Serial.println(" ===");
			buffer[bytesRecv + n] = 0;
			Serial.print(buffer + bytesRecv);
			Serial.println("==================");
#endif
			bytesRecv += n;
			buffer[bytesRecv] = 0;
			for (byte i = 0; i < expectedCount; i++) {
				// match expected string(s)
				if (expected[i] && strstr(buffer, expected[i])) return i + 1;
			}
		} else if (n == -1) {
			// an erroneous reading
#if VERBOSE_XBEE
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
    delay(100);
#if VERBOSE_XBEE
	Serial.println("xBee power pin set to low");
#endif
	digitalWrite(PIN_BEE_PWR, LOW);
	delay(1010);
#if VERBOSE_XBEE
	Serial.println("xBee power pin set to high");
#endif
    digitalWrite(PIN_BEE_PWR, HIGH);
#endif
    delay(100);
    digitalWrite(PIN_BEE_PWR, LOW);
}

void FreematicsESP32::buzzer(int freq)
{
    if (version >= 14) {
        if (freq) {
            ledcWriteTone(0, 2000);
            ledcWrite(0, 255);
        } else {
            ledcWrite(0, 0);
        }
    }
}

byte FreematicsESP32::getVersion()
{
    if (!link) return 0;
    char buf[32];
    if (link->sendCommand("ATI\r", buf, sizeof(buf), 1000)) {
        char *p = strstr(buf, "OBD");
        if (p && (p = strchr(p, ' '))) {
            p += 2;
            if (isdigit(*p) && *(p + 1) == '.' && isdigit(*(p + 2))) {
                version = (*p - '0') * 10 + (*(p + 2) - '0');
                return version;
            }
        }
    }
	return 0;
}

bool FreematicsESP32::reactivateLink()
{
    if (!link) return false;
    for (int n = 0; n < 30; n++) {
        char buf[32];
        if (link->sendCommand("ATI\r", buf, sizeof(buf), 1000)) return true;
    }
    return false;
}

void FreematicsESP32::resetLink()
{
    if (version >= 14) {
        digitalWrite(PIN_LINK_RESET, LOW);
        delay(50);
        digitalWrite(PIN_LINK_RESET, HIGH);
    } else {
        char buf[16];
        if (link) link->sendCommand("ATR\r", buf, sizeof(buf), 100);
    }
}

bool FreematicsESP32::begin(bool useGNSS, bool useCellular)
{
    if (link) return false;

    pinMode(PIN_LINK_RESET, OUTPUT);
    digitalWrite(PIN_LINK_RESET, HIGH);

    // set watchdog timeout to 600 seconds
    esp_task_wdt_init(600, 0);

    m_flags = 0;
    m_pinGPSPower = 0;

    do {
        CLink_UART *linkUART = new CLink_UART;
        if (linkUART->begin()) {
            link = linkUART;
            for (byte n = 0; n < 3 && !(version = getVersion()); n++);
            if (version) {
                if (version >= 14) {
                    m_flags |= GNSS_USE_LINK;
                    // set up buzzer
                    ledcSetup(0, 2000, 8);
                    ledcAttachPin(PIN_BUZZER, 0);
                } else if (version == 13) {
                    m_pinGPSPower = PIN_GPS_POWER2;
                    m_flags |= GNSS_USE_LINK;
                } else {
                    m_pinGPSPower = PIN_GPS_POWER;
                    m_flags |= GNSS_SOFT_SERIAL;
                }
                m_flags |= USE_UART_LINK;
                break;
            }
            link = 0;
            linkUART->end();
        }
        delete linkUART;
        linkUART = 0;
        CLink_SPI *linkSPI = new CLink_SPI;
        if (linkSPI->begin()) {
            link = linkSPI;
            for (byte n = 0; n < 30 && !(version = getVersion()); n++);
            if (version >= 11) {
                m_pinGPSPower = PIN_GPS_POWER;
                break;
            }
            link = 0;
            linkSPI->end();
        }
        delete linkSPI;
        linkSPI = 0;
        return false;
    } while(0);

    if (useCellular) {
        xbBegin(XBEE_BAUDRATE);
        m_flags |= USE_CELL;
    }
    if (useGNSS) {
        m_flags |= USE_GNSS;
    }
    return true;
}
