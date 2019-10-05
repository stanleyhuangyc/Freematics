#include "FreematicsPlus.h"
#include "config.h"
#include "telemesh.h"
#if NET_DEVICE == NET_WIFI_MESH
#include <mdf_common.h>
#include <mwifi.h>

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

bool ClientWiFiMesh::begin(CFreematics* device)
{
  if (m_inited) return true;

  mwifi_init_config_t cfg = MWIFI_INIT_CONFIG_DEFAULT();
  mwifi_config_t config   = {0};
  config.channel   = WIFI_MESH_CHANNEL;
  config.mesh_type = MWIFI_MESH_NODE;
  memcpy(config.mesh_id, WIFI_MESH_ID, sizeof(WIFI_MESH_ID) - 1);

  mdf_event_loop_init(event_loop_cb);
  wifi_init();
  mwifi_init(&cfg);
  mwifi_set_config(&config);
  if (mwifi_start() == MDF_OK) {
    m_inited = true;
    return true;
  }
  return false;
}

bool ClientWiFiMesh::send(const char* data, unsigned int len)
{
  if (mwifi_is_connected()) {
    mwifi_data_type_t data_type = {0x0};
    if (mwifi_write(NULL, &data_type, data, len, true) == MDF_OK)
      return true;
  }
  return false;
}

char* ClientWiFiMesh::receive(int* pbytes, unsigned int timeout)
{
  if (mwifi_is_connected()) {
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
    mwifi_data_type_t data_type      = {0x0};
    size_t size   = sizeof(m_buffer) - 1;
    if (mwifi_read(src_addr, &data_type, m_buffer, &size, timeout / portTICK_PERIOD_MS) == MDF_OK) {
      if (pbytes) *pbytes = size;
      return m_buffer;
    }
  }
  return 0;
}

#endif