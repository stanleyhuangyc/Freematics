#include <Arduino.h>
#include <mdf_common.h>
#include <mwifi.h>
#include "WiFiMesh.h"

static mdf_err_t event_loop_cb(mdf_event_loop_t event, void *ctx)
{
    switch (event) {
        case MDF_EVENT_MWIFI_STARTED:
            Serial.println("MESH is started");
            break;

        case MDF_EVENT_MWIFI_PARENT_CONNECTED:
            Serial.println("Parent is connected on station interface");
            break;

        case MDF_EVENT_MWIFI_PARENT_DISCONNECTED:
            Serial.println("Parent is disconnected on station interface");
            break;

        default:
            break;
    }

    return MDF_OK;
}

static mdf_err_t wifi_init()
{
    mdf_err_t ret          = nvs_flash_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }

    //MDF_ERROR_ASSERT(ret);

    tcpip_adapter_init();
    esp_event_loop_init(NULL, NULL);
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_mesh_set_6m_rate(false);
    return esp_wifi_start();
}

bool WiFiMeshNode::begin(int channel, const char* id)
{
  mwifi_init_config_t cfg = MWIFI_INIT_CONFIG_DEFAULT();
  mwifi_config_t config   = {0};
  config.channel   = channel;
  config.mesh_type = MWIFI_MESH_NODE;
  memcpy(config.mesh_id, id, sizeof(config.mesh_id));

  mdf_event_loop_init(event_loop_cb);
  wifi_init();
  mwifi_init(&cfg);
  mwifi_set_config(&config);
  return mwifi_start() == MDF_OK;
}

bool WiFiMeshNode::send(char* data, size_t len)
{
  if (mwifi_is_connected()) {
    mwifi_data_type_t data_type = {0x0};
    if (mwifi_write(NULL, &data_type, data, len, true) == MDF_OK)
      return true;
  } else {
      return true;
  }
  return false;
}

size_t WiFiMeshNode::receive(char *buffer, size_t size, int timeout)
{
  if (mwifi_is_connected()) {
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
    mwifi_data_type_t data_type      = {0x0};
    if (mwifi_read(src_addr, &data_type, buffer, &size, timeout / portTICK_PERIOD_MS) == MDF_OK) {
      buffer[size] = 0;
      return size;
    }
  }
  return 0;
}

bool WiFiMeshRoot::begin(int channel, const char* id)
{
  mwifi_init_config_t cfg = MWIFI_INIT_CONFIG_DEFAULT();
  mwifi_config_t config   = {0};
  config.channel   = channel;
  config.mesh_type = MWIFI_MESH_ROOT;
  memcpy(config.mesh_id, id, sizeof(config.mesh_id));

  mdf_event_loop_init(event_loop_cb);
  wifi_init();
  mwifi_init(&cfg);
  mwifi_set_config(&config);
  return mwifi_start() == MDF_OK;
}

bool WiFiMeshRoot::send(char* data, size_t len)
{
  if (mwifi_is_started()) {
    mwifi_data_type_t data_type = {0x0};
    if (mwifi_root_write(src_addr, 1, &data_type, data, len, true) == MDF_OK)
      return true;
  } else {
      return true;
  }
  return false;
}

size_t WiFiMeshRoot::receive(char *buffer, size_t size, int timeout)
{
  if (mwifi_is_started()) {
    mwifi_data_type_t data_type = {0x0};
    if (mwifi_root_read(src_addr, &data_type, buffer, &size, timeout / portTICK_PERIOD_MS) == MDF_OK) {
      buffer[size] = 0;
      return size;
    }
  }
  return 0;
}
