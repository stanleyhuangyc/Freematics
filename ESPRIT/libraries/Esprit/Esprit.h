/****************************************************************************
* Arduino Library for Freematics ESPRIT
* Distributed under BSD license
* Visit http://freematics.com/products/freematics-esprit for more information
* (C)2017 Developed by Stanley Huang <support@freematics.com.au>
****************************************************************************/

#ifndef Esprit_h
#define Esprit_h

#ifndef _SIM
#include "esp_system.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_spi_flash.h"
#include "FreematicsPlus.h"

extern HardwareSerial Serial1;
extern HardwareSerial Serial2;

#else

#define SIM_FLASH_SIZE 4 * 1024 * 1024

#endif

#define FLASH_STORAGE_BASE_ADDR 0x100000

class CNVS
{
public:
#ifdef _SIM
	CNVS(const char* flashFilePath, size_t flashSize)
	{
		FILE* fp = fopen(flashFilePath, "rb+");
		size_t size = 0;
		if (fp) {
			fseek(fp, 0, SEEK_END);
			size = ftell(fp);
		}
		if (size != flashSize) {
			if (fp) fclose(fp);
			fp = fopen(flashFilePath, "wb+");
			char c = 0;
			for (size_t t = 0; t < flashSize; t++) {
				fputc(c, fp);
			}
		}
		if (fp) {
			fseek(fp, 0, SEEK_SET);
			m_flashSize = flashSize;
			m_fpFlash = fp;
		}
		else {
			m_fpFlash = 0;
			m_flashSize = 0;
		}
	}
#endif
  bool init(const char* ns = "storage")
  {
#ifndef _SIM
	  m_flashSize = spi_flash_get_chip_size();
    return nvs_open(ns, NVS_READWRITE, &m_handle) == ESP_OK;
#endif
  }
#ifndef _SIM
  bool set(const char* key, int8_t value) { return nvs_set_i8(m_handle, key, value) == ESP_OK; }
  bool set(const char* key, uint8_t value) { return nvs_set_u8(m_handle, key, value) == ESP_OK; }
  bool set(const char* key, int16_t value) { return nvs_set_i16(m_handle, key, value) == ESP_OK; }
  bool set(const char* key, uint16_t value) { return nvs_set_u16(m_handle, key, value) == ESP_OK; }
  bool set(const char* key, int32_t value) { return nvs_set_i32(m_handle, key, value) == ESP_OK; }
  bool set(const char* key, uint32_t value) { return nvs_set_u32(m_handle, key, value) == ESP_OK; }
  bool set(const char* key, void* value, size_t len) { return nvs_set_blob(m_handle, key, value, len) == ESP_OK; }
  bool set(const char* key, const char* value) { return nvs_set_str(m_handle, key, value) == ESP_OK; }
  bool get(const char* key, int8_t& value) { return nvs_get_i8(m_handle, key, &value) == ESP_OK; }
  bool get(const char* key, uint8_t& value) { return nvs_get_u8(m_handle, key, &value) == ESP_OK; }
  bool get(const char* key, int16_t& value) { return nvs_get_i16(m_handle, key, &value) == ESP_OK; }
  bool get(const char* key, uint16_t& value) { return nvs_get_u16(m_handle, key, &value) == ESP_OK; }
  bool get(const char* key, int32_t& value) { return nvs_get_i32(m_handle, key, &value) == ESP_OK; }
  bool get(const char* key, uint32_t& value) { return nvs_get_u32(m_handle, key, &value) == ESP_OK; }
  bool get(const char* key, void* buffer, size_t& len) { return nvs_get_blob(m_handle, key, buffer, &len) == ESP_OK; }
  bool get(const char* key, char* buffer, size_t& len) { return nvs_get_str(m_handle, key, buffer, &len) == ESP_OK; }
  bool erase() { return nvs_erase_all(m_handle); }
  bool erase(const char* key) { return nvs_erase_key(m_handle, key); }
  bool commit() { return nvs_commit(m_handle); }
#endif
  bool flashRead(void* buffer, size_t bufsize, size_t offet = -1);
  bool flashWrite(void* buffer, size_t bufsize, size_t offet = -1);
  size_t getFlashSize() { return m_flashSize; }
private:
#ifndef _SIM
	nvs_handle m_handle;
	uint32_t m_offset;
#else
	FILE* m_fpFlash;
#endif
	size_t m_flashSize;
};

#ifndef _SIM
class GATTServer
{
public:
  bool init(const char* deviceName = 0);
  bool send(uint8_t* data, size_t len);
  virtual size_t onRequest(uint8_t* buffer, size_t len)
  {
    // being requested for data
    buffer[0] = 'O';
    buffer[1] = 'K';
    return 2;
  }
  virtual void onReceive(uint8_t* buffer, size_t len)
  {
    // data received is in buffer
  }
};
#endif

#endif
