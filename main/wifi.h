/*
 * The MIT License
 *
 * Copyright 2023 chris.
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
 * File:   wifi.h
 * Author: chris
 *
 * Created on January 20, 2023, 7:42 PM
 */

#pragma once

#include <cstring>
//#include <algorithm> 
#include <mutex>

#include "esp_wifi.h"
#include "esp_event.h"

//#include "lwip/err.h" // Needed for NTP
//#include "lwip/sys.h" // Needed for NTP

namespace WIFI {

    class Wifi {
    public:

        enum class state_e {
            NOT_INITIALIZED,
            INITIALIZED,
            READY_TO_CONNECT,
            CONNECTING,
            WAITING_FOR_IP,
            CONNECTED,
            DISCONNECTED,
            ERROR
        };

    private:
        static esp_err_t _init();
        static wifi_init_config_t _wifi_init_cfg;
        static wifi_config_t _wifi_cfg;

        static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                int32_t event_id, void *event_data);
        static void ip_event_handler(void *arg, esp_event_base_t event_base,
                int32_t event_id, void *event_data);

        static state_e _state;
        static esp_err_t _get_mac(void);
        static char _mac_addr_cstr[13];
        static std::mutex _mutx;

    public:
        Wifi(void);

        void SetCredentials(const char *ssid, const char *password);
        esp_err_t Init();
        esp_err_t Begin(void);

        constexpr static const state_e &GetState(void) {
            return _state;
        }

        constexpr static const char *GetMac(void) {
            return _mac_addr_cstr;
        }
    }; // Wifi class

} // namaspace WIFI