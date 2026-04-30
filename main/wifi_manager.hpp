#pragma once

#include <thread>
#include <atomic>
#include <string>

#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_err.h"

class WifiManager {
public:
    WifiManager();

    esp_err_t init();          // call once after NVS init
    esp_err_t start_async();   // non-blocking Wi-Fi start

    [[nodiscard]] bool is_connected() const { return connected.load(); }
    [[nodiscard]] bool is_ap_mode()   const { return ap_mode.load(); }
    [[nodiscard]] bool has_credentials() const { return creds_loaded; }

    // Provisioning API
    esp_err_t save_credentials(const char* ssid, const char* pass);
    esp_err_t clear_credentials();

private:
    [[noreturn]] void wifi_thread();        // background thread
    esp_err_t load_credentials();
    esp_err_t start_sta();
    esp_err_t start_ap_provisioning();

    static void event_handler(void* arg, esp_event_base_t base, int32_t id, void* data);

    struct Credentials {
        char ssid[32];
        char pass[64];
    } creds{};

    bool creds_loaded = false;
    std::atomic<bool> ap_mode{false};
    std::atomic<bool> connected{false};

    std::thread worker;

    static inline EventGroupHandle_t wifi_event_group = nullptr;
    static constexpr int WIFI_CONNECTED_BIT = BIT0;
    static constexpr int WIFI_FAIL_BIT      = BIT1;

    static inline int retry_count = 0;
    static constexpr int MAX_RETRY = 10;
};
