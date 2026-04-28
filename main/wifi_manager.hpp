// wifi_manager.hpp
#pragma once

#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_err.h"

class WifiManager {
public:
    WifiManager();

    esp_err_t init();   // call once after NVS init
    esp_err_t start();  // decides STA vs AP based on NVS

    bool wait_for_sta(TickType_t timeout = portMAX_DELAY);

    // Provisioning API (call from your HTTP handler)
    esp_err_t save_credentials(const char* ssid, const char* pass);
    esp_err_t clear_credentials();

    bool has_credentials() const { return creds_loaded; }
    bool is_ap_mode() const { return ap_mode; }

private:
    static void event_handler(void* arg, esp_event_base_t base, int32_t id, void* data);

    esp_err_t load_credentials();
    esp_err_t start_sta();
    esp_err_t start_ap_provisioning();

    struct Credentials {
        char ssid[32];
        char pass[64];
    } creds{};

    bool creds_loaded = false;
    bool ap_mode = false;

    static inline EventGroupHandle_t wifi_event_group = nullptr;
    static constexpr int WIFI_CONNECTED_BIT = BIT0;
    static constexpr int WIFI_FAIL_BIT      = BIT1;

    static inline int retry_count = 0;
    static constexpr int MAX_RETRY = 10;
};
