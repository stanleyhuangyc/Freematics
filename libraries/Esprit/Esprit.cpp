/****************************************************************************
*
* Arduino Library for Freematics ESPRIT
* Distributed under BSD license
*
* This library currently provides access to BLE (GATT server), NVS, Flash I/O
* and additional hardware serial UARTs, within the scope of Arduino.
*
* (C)2017 Developed by Stanley Huang <support@freematics.com.au>
* Visit http://freematics.com/products/freematics-esprit for more information
*
****************************************************************************/

#include <Arduino.h>
#ifndef _SIM
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "bt.h"
#include "bta_api.h"
#include "esp_gatt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_main.h"

HardwareSerial Serial1(1);
HardwareSerial Serial2(2);

#endif
#include "Esprit.h"

#ifndef _SIM

extern "C" {
  void gatts_init(const char* device_name);
  int gatts_send(uint8_t* data, size_t len);
}

bool initBLE()
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

static GATTServer* gatts_inst = 0;

bool GATTServer::init(const char* deviceName)
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

bool CNVS::flashRead(void* buffer, size_t bufsize, size_t offset)
{
	if (offset == -1) {
		offset = m_offset;
	}
	else {
		m_offset = offset;
	}
	bool success = spi_flash_read(m_offset, buffer, bufsize) == ESP_OK;
	if (success) {
		m_offset += bufsize;
	}
	return success;
}

bool CNVS::flashWrite(void* buffer, size_t bufsize, size_t offset)
{
	if (offset == -1) {
		offset = m_offset;
	}
	else {
		m_offset = offset;
	}
	bool success = spi_flash_write(FLASH_STORAGE_BASE_ADDR, buffer, bufsize) == ESP_OK;
	if (success) {
		m_offset += bufsize;
	}
	return success;
}

#else

bool CNVS::flashRead(void* buffer, size_t bufsize, size_t offset)
{
	if (m_fpFlash) {
		if (offset != -1) {
			fseek(m_fpFlash, offset, SEEK_SET);
		}
		return fread(buffer, 1, bufsize, m_fpFlash) == bufsize;
	}
	return false;
}

bool CNVS::flashWrite(void* buffer, size_t bufsize, size_t offset)
{
	if (m_fpFlash) {
		if (offset != -1) {
			fseek(m_fpFlash, offset, SEEK_SET);
		}
		return fwrite(buffer, 1, bufsize, m_fpFlash) == bufsize;
	}
	return false;
}

#endif
