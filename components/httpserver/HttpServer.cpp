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
 * File:   HttpServer.cpp
 * Author: chris
 * 
 * Created on February 25, 2023, 9:17 PM
 */

#include "HttpServer.h"

HttpServer::HttpServer() {
}

HttpServer::HttpServer(const HttpServer& orig) {
}

HttpServer::~HttpServer() {
}

void HttpServer::initialise(const std::string& basePath) {
    this->basePath = basePath;  
    
    ESP_LOGI(TAG, "Starting webserver with base path %s", basePath.c_str());      
}

void HttpServer::registerUriHandler(std::string& Uri, std::function<void(HttpRequest& request, HttpResponse& response) const> callback) {
    /* URI handler for getting uploaded files */
    httpd_uri_t urihandler = {
        .uri = Uri.c_str(), // Match all URIs of type /path/to/file
        .method = HTTP_GET,
        .handler = download_get_handler,
        .user_ctx = this->server_data // Pass server data as context
    };
    httpd_register_uri_handler(server, &urihandler);
}
