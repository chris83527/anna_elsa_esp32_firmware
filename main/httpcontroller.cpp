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
 * File:   HttpController.cpp
 * Author: chris
 * 
 * Created on March 2, 2023, 5:53 PM
 */

#include "httpcontroller.h"

static const char *TAG = "httpcontroller";

HttpController::HttpController() {
}

HttpController::HttpController(const HttpController& orig) {
}

HttpController::~HttpController() {
}

HttpController::initialise(const int port, std::string& basePath, std::string& ssid, std::string& password) {
    this->httpServer = std::make_unique<HttpServer>(new HttpServer());
    
    wifi.SetCredentials(ssid.c_str(), password.c_str()); // TODO: Move these out
    wifi.Init();
    //xTaskCreate(&wifiStatusTask, "wifi_status_task", 2048, this, 1, NULL);
    wifiStatusThread.reset(new std::thread([this]() {
        wifiStatusTask();
    }));
        
}

void HttpController::wifiStatusTask() {

    while (1) {
        WIFI::Wifi::state_e wifiState = this->wifi.GetState();

        switch (wifiState) {
            case WIFI::Wifi::state_e::READY_TO_CONNECT:
                //std::cout << "Wifi Status: READY_TO_CONNECT\n";
                this->wifi.Begin();
                break;
            case WIFI::Wifi::state_e::DISCONNECTED:
                //std::cout << "Wifi Status: DISCONNECTED\n";
                this->wifi.Begin();
                break;
            case WIFI::Wifi::state_e::CONNECTING:
                //std::cout << "Wifi Status: CONNECTING\n";
                break;
            case WIFI::Wifi::state_e::WAITING_FOR_IP:
                //std::cout << "Wifi Status: WAITING_FOR_IP\n";
                break;
            case WIFI::Wifi::state_e::ERROR:
                //std::cout << "Wifi Status: ERROR\n";
                break;
            case WIFI::Wifi::state_e::CONNECTED:
                //std::cout << "Wifi Status: CONNECTED\n";
                break;
            case WIFI::Wifi::state_e::NOT_INITIALIZED:
                //std::cout << "Wifi Status: NOT_INITIALIZED\n";
                break;
            case WIFI::Wifi::state_e::INITIALIZED:
                //std::cout << "Wifi Status: INITIALIZED\n";
                break;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}


