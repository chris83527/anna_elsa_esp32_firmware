//
// Created by chris on 28.04.26.
//

#include "wifi_manager.hpp"
#include "esp_log.h"
#include "NvsController.h"

static const char* TAG = "WifiManager";

WifiManager::WifiManager(NvsController& controller) : nvsController(controller)
{
}

esp_err_t WifiManager::init()
{
    if (!wifi_event_group)
        wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_netif_create_default_wifi_ap();

    return load_credentials();
}

esp_err_t WifiManager::load_credentials()
{
    esp_err_t err;

    size_t ssid_len = sizeof(creds.ssid);
    size_t pass_len = sizeof(creds.pass);

    err = nvsController.readStringValueFromNVS("ssid", creds.ssid, ssid_len);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No stored SSID");
        creds_loaded = false;
        return ESP_OK;
    }
    err = nvsController.readStringValueFromNVS( "pass", creds.pass, pass_len);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No stored password");
        creds_loaded = false;
        return ESP_OK;
    }

    creds_loaded = true;
    ESP_LOGI(TAG, "Loaded Wi-Fi credentials from NVS (SSID='%s')", creds.ssid);
    return ESP_OK;
}

esp_err_t WifiManager::save_credentials(const char* ssid, const char* pass)
{
    esp_err_t err;

    err = nvsController.writeStringValueToNVS("ssid", ssid);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not write SSID to NVS: %s", esp_err_to_name(err));
        return err;
    }
    err = nvsController.writeStringValueToNVS("pass", pass);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not write password to NVS: %s", esp_err_to_name(err));
        return err;
    }

    strncpy(creds.ssid, ssid, sizeof(creds.ssid));
    creds.ssid[sizeof(creds.ssid) - 1] = '\0';
    strncpy(creds.pass, pass, sizeof(creds.pass));
    creds.pass[sizeof(creds.pass) - 1] = '\0';
    creds_loaded = true;

    ESP_LOGI(TAG, "Saved Wi-Fi credentials to NVS");
    return ESP_OK;
}

esp_err_t WifiManager::clear_credentials()
{
    esp_err_t err;

    err = nvsController.eraseValueFromNVS("ssid");
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not clear SSID from NVS: %s", esp_err_to_name(err));
        return err;
    }

    err = nvsController.eraseValueFromNVS("pass");
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not clear password from NVS: %s", esp_err_to_name(err));
        return err;
    }

    creds_loaded = false;
    memset(&creds, 0, sizeof(creds));
    ESP_LOGI(TAG, "Cleared Wi-Fi credentials from NVS");
    return ESP_OK;
}

void WifiManager::rssi_thread() {
    ESP_LOGI("WifiManager", "RSSI monitor thread started");

    while (true) {
        if (connected.load()) {
            wifi_ap_record_t info{};
            if (esp_wifi_sta_get_ap_info(&info) == ESP_OK) {
                last_rssi.store(info.rssi);
            }
        }

        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

void WifiManager::wifi_thread()
{
    ESP_LOGI(TAG, "Wi-Fi background thread starting");

    if (creds_loaded)
    {
        ESP_LOGI(TAG, "Starting STA mode");
        start_sta();
    }
    else
    {
        ESP_LOGW(TAG, "No credentials, starting AP provisioning");
        start_ap_provisioning();
    }

    // Start watchdog
    watchdog_worker = std::thread([this]() {
        ESP_LOGI(TAG, "Wi-Fi watchdog thread started");
        int disconnected_secs = 0;
        while (!stop_watchdog.load()) {
            if (!ap_mode.load()) { // only care in STA mode
                if (!connected.load()) {
                    disconnected_secs++;
                    if (disconnected_secs >= 60) { // 60s threshold
                        ESP_LOGW(TAG, "Wi-Fi disconnected for 60s, restarting STA");
                        disconnected_secs = 0;
                        esp_wifi_stop();
                        start_sta();
                    }
                } else {
                    disconnected_secs = 0;
                }
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });
    watchdog_worker.detach();

    while (true)
    {
        EventBits_t bits = xEventGroupWaitBits(
            wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdTRUE,
            pdFALSE,
            portMAX_DELAY
        );

        if (bits & WIFI_CONNECTED_BIT) {
            connected.store(true);
            if (!rssi_worker.joinable()) {
                rssi_worker = std::thread(&WifiManager::rssi_thread, this);
                rssi_worker.detach();
            }
        }

        if (bits & WIFI_FAIL_BIT)
        {
            connected.store(false);
            ESP_LOGE(TAG, "Wi-Fi failed, switching to AP mode");
            start_ap_provisioning();
        }
    }
}

esp_err_t WifiManager::start_ap() {
    ESP_LOGI(TAG, "Starting AP interface (dual-mode)");

    //esp_netif_create_default_wifi_ap();

    wifi_config_t ap_config = {};
    strcpy(reinterpret_cast<char*>(ap_config.ap.ssid), "Frozen-Setup");
    ap_config.ap.ssid_len = strlen("Frozen-Setup");
    ap_config.ap.channel = 1;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

    return ESP_OK;
}


std::vector<WifiNetwork> WifiManager::scan_networks() {
    std::vector<WifiNetwork> result;

    wifi_scan_config_t scan_config = {};
    scan_config.show_hidden = true;   // still scan hidden, but we filter them

    esp_wifi_scan_start(&scan_config, true);

    uint16_t count = 0;
    esp_wifi_scan_get_ap_num(&count);

    std::vector<wifi_ap_record_t> records(count);
    esp_wifi_scan_get_ap_records(&count, records.data());

    for (auto& r : records) {
        // ⭐ Skip hidden networks
        if (r.ssid[0] == '\0')
            continue;

        WifiNetwork n;
        n.ssid = reinterpret_cast<const char*>(r.ssid);
        n.rssi = r.rssi;
        n.auth = r.authmode;
        n.is_hidden = false;
        result.push_back(n);
    }

    // ⭐ Sort strongest first
    std::sort(result.begin(), result.end(),
              [](const WifiNetwork& a, const WifiNetwork& b) {
                  return a.rssi > b.rssi;
              });

    return result;
}

esp_err_t WifiManager::start_sta() {
    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid,  creds.ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, creds.pass, sizeof(wifi_config.sta.password));

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));   // <‑‑ dual mode
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    // Start AP interface too
    start_ap();

    ESP_LOGI(TAG, "Wi-Fi AP+STA starting…");
    return esp_wifi_start();
}


esp_err_t WifiManager::start_ap_provisioning() {
    ESP_LOGW(TAG, "Starting AP-only provisioning mode");

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    start_ap();

    ap_mode.store(true);
    connected.store(false);

    return esp_wifi_start();
}

esp_err_t WifiManager::start_async()
{
    worker = std::thread(&WifiManager::wifi_thread, this);
    worker.detach(); // background forever
    return ESP_OK;
}

void WifiManager::event_handler(void* arg,
                                esp_event_base_t base,
                                int32_t id,
                                void* data) {
    auto* self = static_cast<WifiManager*>(arg);

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }

    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        self->connected.store(false);

        if (retry_count < MAX_RETRY) {
            retry_count++;
            int delay_ms = 200 * retry_count;
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    }

    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const auto* event = static_cast<ip_event_got_ip_t*>(data);
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
