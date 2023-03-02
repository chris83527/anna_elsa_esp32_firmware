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
 * File:   HttpController.h
 * Author: chris
 *
 * Created on March 2, 2023, 5:53 PM
 */

#ifndef HTTPCONTROLLER_H
#define HTTPCONTROLLER_H

#include <thread>
#include <cstring>

#include "esp_wifi.h"
#include "esp_log.h"
#include "wifi.h"

#include "HttpServer.h"

class HttpController {
public:
    HttpController();
    HttpController(const HttpController& orig);
    virtual ~HttpController();

    void initialise(const int port, std::string& basePath, std::string& ssid, std::string& password);
private:

    std::make_unique<HttpServer> httpServer;
    void wifiStatusTask(void);

    WIFI::Wifi::state_e wifiState{ WIFI::Wifi::state_e::NOT_INITIALIZED};
    WIFI::Wifi wifi;
    std::make_unique<std::thread> wifiStatusThread; 
};

#endif /* HTTPCONTROLLER_H */

