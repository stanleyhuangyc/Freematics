/****************************************************************************
*
* Arduino Helper Library for Freematics Esprit Development Board
* Distributed under BSD license
*
* This library currently provides access to BLE (GATT server), NVS, Flash I/O
* and additional hardware serial UARTs, within the scope of Arduino.
*
* Developed by Stanley Huang <stanley@freematics.com.au>
* Visit http://freematics.com/products/freematics-esprit for more information
*
****************************************************************************/

#include <Arduino.h>
#ifndef _SIM
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#endif
#include "Esprit.h"

#ifndef _SIM

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
