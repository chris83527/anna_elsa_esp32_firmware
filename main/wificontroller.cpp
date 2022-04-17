/*
 * The MIT License
 *
 * Copyright 2022 chris.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/* 
 * File:   wificontroller.cpp
 * Author: chris
 * 
 * Created on April 16, 2022, 4:14 PM
 */

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "wificontroller.h"

static const char *TAG = "WifiController";

namespace Wifi {

    // Statics
    char Wifi::_macAddrCstr[]{};
    std::mutex Wifi::_mutex{};
    Wifi::state_e Wifi::_state{state_e::NOT_INITIALIZED};
    wifi_init_config_t Wifi::_wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t Wifi::_wifi_cfg{};

    WifiController::WifiController(void) {
        if (!_macAddrCstr[0]) {
            if (ESP_OK != _getMac()) {
                esp_restart();
            }
        }
    }

    void WifiController::wifiEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
        if (WIFI_EVENT == event_base) {
            const wifi_event_t event_type{static_cast<wifi_event_t> (event_id)};

            switch (event_type) {
                case WIFI_EVENT_STA_START:
                {
                    std::lock_guard<std::mutex> state_guard(_mutex);
                    _state = state_e::READY_TO_CONNECT;
                    break;
                }

                case WIFI_EVENT_STA_CONNECTED:
                {
                    std::lock_guard<std::mutex> state_guard(_mutex);
                    _state = state_e::WAITING_FOR_IP;
                    break;
                }

                case WIFI_EVENT_STA_DISCONNECTED:
                {
                    std::lock_guard<std::mutex> state_guard(_mutex);
                    _state = state_e::DISCONNECTED;
                    break;
                }

                default:
                    break;
            }
        }
    }

    void WifiController::ipEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
        if (IP_EVENT == event_base) {
            const ip_event_t event_type{static_cast<ip_event_t> (event_id)};

            switch (event_type) {
                case IP_EVENT_STA_GOT_IP:
                {
                    std::lock_guard<std::mutex> state_guard(_mutex);
                    _state = state_e::CONNECTED;
                    break;
                }

                case IP_EVENT_STA_LOST_IP:
                {
                    std::lock_guard<std::mutex> state_guard(_mutex);
                    if (state_e::DISCONNECTED != _state) {
                        _state = state_e::WAITING_FOR_IP;
                    }
                    break;
                }

                default:
                    break;
            }
        }
    }

    esp_err_t WifiController::begin(void) {
        std::lock_guard<std::mutex> connect_guard(_mutex);

        esp_err_t status{ESP_OK};

        switch (_state) {
            case state_e::READY_TO_CONNECT:
            case state_e::DISCONNECTED:
                status = esp_wifi_connect();
                if (ESP_OK == status) {
                    _state = state_e::CONNECTING;
                }
                break;
            case state_e::CONNECTING:
            case state_e::WAITING_FOR_IP:
            case state_e::CONNECTED:
                break;
            case state_e::NOT_INITIALIZED:
            case state_e::INITIALIZED:
            case state_e::ERROR:
                status = ESP_FAIL;
                break;
        }
        return status;
    }

    esp_err_t WifiController::_init() {
        std::lock_guard<std::mutex> mutex_guard(_mutex);

        esp_err_t status{ESP_OK};

        if (state_e::NOT_INITIALIZED == _state) {
            status |= esp_netif_init();
            if (ESP_OK == status) {
                const esp_netif_t * const p_netif = esp_netif_create_default_wifi_sta();

                if (!p_netif) {
                    status = ESP_FAIL;
                }
            }

            if (ESP_OK == status) {
                status = esp_wifi_init(&_wifi_init_cfg);
            }

            if (ESP_OK == status) {
                status = esp_event_handler_instance_register(WIFI_EVENT,
                        ESP_EVENT_ANY_ID,
                        &wifiEventHandler,
                        nullptr,
                        nullptr);
            }

            if (ESP_OK == status) {
                status = esp_event_handler_instance_register(IP_EVENT,
                        ESP_EVENT_ANY_ID,
                        &ipEventHandler,
                        nullptr,
                        nullptr);
            }

            if (ESP_OK == status) {
                status = esp_wifi_set_mode(WIFI_MODE_STA);
            }

            if (ESP_OK == status) {
                _wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
                _wifi_cfg.sta.pmf_cfg.capable = true;
                _wifi_cfg.sta.pmf_cfg.required = false;

                status = esp_wifi_set_config(WIFI_IF_STA, &_wifi_cfg);
            }

            if (ESP_OK == status) {
                status = esp_wifi_start(); // start Wifi
            }

            if (ESP_OK == status) {
                _state = state_e::INITIALIZED;
            }
        } else if (state_e::ERROR == _state) {
            _state = state_e::NOT_INITIALIZED;
        }

        return status;
    }

    void WifiController::setCredentials(const char *ssid, const char *password) {
        memcpy(_wifi_cfg.sta.ssid, ssid, std::min(strlen(ssid), sizeof (_wifi_cfg.sta.ssid)));
        memcpy(_wifi_cfg.sta.password, password, std::min(strlen(password), sizeof (_wifi_cfg.sta.password)));
    }

    esp_err_t WifiController::initialise() {
        return _init();
    }

    // Get default MAC from API and convert to ASCII HEX

    esp_err_t WifiController::_getMac(void) {
        uint8_t mac_byte_buffer[6]{};

        const esp_err_t status{esp_efuse_mac_get_default(mac_byte_buffer)};

        if (ESP_OK == status) {
            snprintf(_macAddrCstr, sizeof (_macAddrCstr), "%02X%02X%02X%02X%02X%02X",
                    mac_byte_buffer[0],
                    mac_byte_buffer[1],
                    mac_byte_buffer[2],
                    mac_byte_buffer[3],
                    mac_byte_buffer[4],
                    mac_byte_buffer[5]);
        }

        return status;
    }

} // namespace WIFI

