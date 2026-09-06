#include "boot_id.hpp"

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

namespace bootid {

namespace {
const char* kTag = "bootid";
const char* kNamespace = "pod";
const char* kKey = "bootid";
}

uint16_t next() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    nvs_handle_t h;
    ESP_ERROR_CHECK(nvs_open(kNamespace, NVS_READWRITE, &h));

    uint16_t id = 0;
    err = nvs_get_u16(h, kKey, &id);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(kTag, "read failed (%s), restarting the count", esp_err_to_name(err));
        id = 0;
    }

    ++id;

    ESP_ERROR_CHECK(nvs_set_u16(h, kKey, id));
    ESP_ERROR_CHECK(nvs_commit(h));
    nvs_close(h);

    return id;
}

}  // namespace bootid
