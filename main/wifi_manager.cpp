//
// Created by chris on 28.04.26.
//

#include "wifi_manager.hpp"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_chip_info.h"
#include "mdns.h"

static const char* TAG = "WifiManager";

WifiManager::WifiManager()
{
}

WifiManager::~WifiManager()
{
    stop_rssi.store(true);
    stop_watchdog.store(true);
}

void WifiManager::stop()
{
    stop_rssi.store(true);
    stop_watchdog.store(true);
}

esp_err_t WifiManager::init()
{
    if (!wifi_event_group)
        wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    init_mdns();

    return ESP_OK;
}

bool WifiManager::has_credentials() const
{
    wifi_config_t cfg{};
    esp_wifi_get_config(WIFI_IF_STA, &cfg);
    return cfg.sta.ssid[0] != '\0';
}

esp_err_t WifiManager::save_credentials(const char* ssid, const char* pass)
{
    wifi_config_t cfg{};
    strncpy((char*)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid));
    strncpy((char*)cfg.sta.password, pass, sizeof(cfg.sta.password));

    return esp_wifi_set_config(WIFI_IF_STA, &cfg); // saved to NVS
}

std::string WifiManager::generate_hostname()
{
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);

    char buf[32];
    snprintf(buf, sizeof(buf),
             "frozen-%02x%02x%02x",
             mac[3], mac[4], mac[5]);

    return std::string(buf);
}

void WifiManager::rssi_thread()
{
    ESP_LOGI("WifiManager", "RSSI monitor thread started");

    while (true)
    {
        if (connected.load())
        {
            wifi_ap_record_t info{};
            if (esp_wifi_sta_get_ap_info(&info) == ESP_OK)
            {
                last_rssi.store(info.rssi);
            }
        }

        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

void WifiManager::wifi_thread()
{
    ESP_LOGI(TAG, "Wi-Fi background thread starting");

    if (has_credentials())
    {
        ESP_LOGI(TAG, "Starting STA mode");
        ap_mode.store(false);
        start_sta();
    }
    else
    {
        ESP_LOGW(TAG, "No credentials, starting AP provisioning");
        ap_mode.store(true);
        start_ap_provisioning();
    }

    // Start watchdog
    watchdog_worker = std::thread([this]()
    {
        ESP_LOGI(TAG, "Wi-Fi watchdog thread started");

        int disconnected_secs = 0;

        while (!stop_watchdog.load())
        {
            if (!has_credentials())
            {
                disconnected_secs = 0;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            // only care in STA mode
            if (!connected.load())
            {
                disconnected_secs++;
                if (disconnected_secs >= 60)
                {
                    // 60s threshold
                    ESP_LOGW(TAG, "Wi-Fi disconnected for 60s, restarting STA");
                    disconnected_secs = 0;
                    esp_wifi_disconnect();
                    esp_wifi_connect();
                }
            }
            else
            {
                disconnected_secs = 0;
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });
    watchdog_worker.detach();

    //
    // 3. Optional: background RSSI monitor
    //
    rssi_worker = std::thread([this]()
    {
        while (!stop_rssi.load())
        {
            if (connected.load())
            {
                int rssi = get_rssi();
                last_rssi.store(rssi);
            }
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    });

    rssi_worker.detach();

    while (true)
    {
        EventBits_t bits = xEventGroupWaitBits(
            wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdTRUE,
            pdFALSE,
            portMAX_DELAY
        );

        if (bits & WIFI_CONNECTED_BIT)
        {
            connected.store(true);
            if (!rssi_worker.joinable())
            {
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

esp_err_t WifiManager::start_ap()
{
    ESP_LOGI(TAG, "Starting AP interface (dual-mode)");

    wifi_config_t ap_config = {};
    strcpy(reinterpret_cast<char*>(ap_config.ap.ssid), "Frozen-Setup");
    ap_config.ap.ssid_len = strlen("Frozen-Setup");
    ap_config.ap.channel = 1;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

    return ESP_OK;
}


std::vector<WifiNetwork> WifiManager::scan_networks()
{
    std::vector<WifiNetwork> result;

    wifi_scan_config_t scan_config = {};
    scan_config.show_hidden = true; // still scan hidden, but we filter them

    esp_wifi_scan_start(&scan_config, true);

    uint16_t count = 0;
    esp_wifi_scan_get_ap_num(&count);

    std::vector<wifi_ap_record_t> records(count);
    esp_wifi_scan_get_ap_records(&count, records.data());

    for (auto& r : records)
    {
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
              [](const WifiNetwork& a, const WifiNetwork& b)
              {
                  return a.rssi > b.rssi;
              });

    return result;
}

esp_err_t WifiManager::start_sta()
{
    wifi_config_t wifi_config{};
    esp_wifi_get_config(WIFI_IF_STA, &wifi_config);

    if (wifi_config.sta.ssid[0] == '\0')
    {
        ESP_LOGW(TAG, "No STA credentials in NVS");
        return ESP_FAIL;
    }

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA)); // <‑‑ dual mode
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    // Start AP interface too
    start_ap();

    ESP_LOGI(TAG, "Wi-Fi AP+STA starting…");
    return esp_wifi_start();
}


esp_err_t WifiManager::start_ap_provisioning()
{
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
                                void* data)
{
    auto* self = static_cast<WifiManager*>(arg);

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }

    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
    {
        self->connected.store(false);

        if (retry_count < MAX_RETRY)
        {
            retry_count++;
            int delay_ms = 200 * retry_count;
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
            esp_wifi_connect();
        }
        else
        {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
    }

    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
        const auto* event = static_cast<ip_event_got_ip_t*>(data);
        ESP_LOGI(TAG, "Got IP: ", IPSTR, IP2STR(&event->ip_info.ip));
        retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

esp_err_t WifiManager::init_mdns()
{
    ESP_LOGI(TAG, "Initializing mDNS");


    esp_err_t err = mdns_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "mDNS init failed: %s", esp_err_to_name(err));
        return err;
    }

    // ⭐ Generate dynamic hostname
    hostname = generate_hostname();

    mdns_hostname_set(hostname.c_str());
    mdns_instance_name_set("Woods Amusements Frozen Device");

    // Advertise HTTP service
    mdns_service_add("Frozen Web", "_http", "_tcp", 80, nullptr, 0);

    ESP_LOGI(TAG, "mDNS hostname: %s.local", hostname.c_str());

    return ESP_OK;
}
