//
// Created by chris on 28.04.26.
//

#include "wifi_manager.hpp"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char* TAG = "WifiManager";

WifiManager::WifiManager() {}

esp_err_t WifiManager::init() {
    if (!wifi_event_group)
        wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_init());
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_loop_create_default());

    return load_credentials();
}

esp_err_t WifiManager::load_credentials() {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("wifi", NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No Wi-Fi namespace yet");
        creds_loaded = false;
        return ESP_OK;
    }

    size_t ssid_len = sizeof(creds.ssid);
    size_t pass_len = sizeof(creds.pass);

    err = nvs_get_str(nvs, "ssid", creds.ssid, &ssid_len);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No stored SSID");
        nvs_close(nvs);
        creds_loaded = false;
        return ESP_OK;
    }

    err = nvs_get_str(nvs, "pass", creds.pass, &pass_len);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No stored password");
        nvs_close(nvs);
        creds_loaded = false;
        return ESP_OK;
    }

    nvs_close(nvs);
    creds_loaded = true;
    ESP_LOGI(TAG, "Loaded Wi-Fi credentials from NVS (SSID='%s')", creds.ssid);
    return ESP_OK;
}

esp_err_t WifiManager::save_credentials(const char* ssid, const char* pass) {
    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("wifi", NVS_READWRITE, &nvs));

    ESP_ERROR_CHECK(nvs_set_str(nvs, "ssid", ssid));
    ESP_ERROR_CHECK(nvs_set_str(nvs, "pass", pass));
    ESP_ERROR_CHECK(nvs_commit(nvs));
    nvs_close(nvs);

    strncpy(creds.ssid, ssid, sizeof(creds.ssid));
    creds.ssid[sizeof(creds.ssid)-1] = '\0';
    strncpy(creds.pass, pass, sizeof(creds.pass));
    creds.pass[sizeof(creds.pass)-1] = '\0';
    creds_loaded = true;

    ESP_LOGI(TAG, "Saved Wi-Fi credentials to NVS");
    return ESP_OK;
}

esp_err_t WifiManager::clear_credentials() {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("wifi", NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    nvs_erase_key(nvs, "ssid");
    nvs_erase_key(nvs, "pass");
    nvs_commit(nvs);
    nvs_close(nvs);

    creds_loaded = false;
    memset(&creds, 0, sizeof(creds));
    ESP_LOGI(TAG, "Cleared Wi-Fi credentials from NVS");
    return ESP_OK;
}

esp_err_t WifiManager::start() {
    retry_count = 0;

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
                                               ESP_EVENT_ANY_ID,
                                               &WifiManager::event_handler,
                                               this));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                                               IP_EVENT_STA_GOT_IP,
                                               &WifiManager::event_handler,
                                               this));

    if (creds_loaded) {
        ESP_LOGI(TAG, "Starting in STA mode with stored credentials");
        return start_sta();
    } else {
        ESP_LOGW(TAG, "No credentials, starting AP provisioning");
        return start_ap_provisioning();
    }
}

esp_err_t WifiManager::start_sta() {
    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid,  creds.ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, creds.pass, sizeof(wifi_config.sta.password));

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    ap_mode = false;
    ESP_LOGI(TAG, "Wi-Fi STA starting…");
    return esp_wifi_start();
}

esp_err_t WifiManager::start_ap_provisioning() {
    ESP_LOGW(TAG, "Starting AP provisioning mode");

    esp_netif_create_default_wifi_ap();

    wifi_config_t ap_config = {};
    strcpy((char*)ap_config.ap.ssid, "Frozen-Setup");
    ap_config.ap.ssid_len = strlen("Frozen-Setup");
    ap_config.ap.channel = 1;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_OPEN; // or WPA2 if you want
    ap_config.ap.ssid_hidden = 0;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ap_mode = true;
    ESP_LOGI(TAG, "AP started: SSID='Frozen-Setup'");
    return ESP_OK;
}

bool WifiManager::wait_for_sta(TickType_t timeout) {
    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        timeout
    );

    return bits & WIFI_CONNECTED_BIT;
}

void WifiManager::event_handler(void* arg,
                                esp_event_base_t base,
                                int32_t id,
                                void* data) {
    auto* self = static_cast<WifiManager*>(arg);

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "STA started, connecting…");
        esp_wifi_connect();
    }

    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_count < MAX_RETRY) {
            retry_count++;
            int delay_ms = 200 * retry_count;
            ESP_LOGW(TAG, "STA disconnected, retry %d/%d (delay %d ms)",
                     retry_count, MAX_RETRY, delay_ms);
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
            esp_wifi_connect();
        } else {
            ESP_LOGE(TAG, "STA failed after %d retries, switching to AP provisioning", MAX_RETRY);
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
            self->start_ap_provisioning();
        }
    }

    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        auto* event = (ip_event_got_ip_t*)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
