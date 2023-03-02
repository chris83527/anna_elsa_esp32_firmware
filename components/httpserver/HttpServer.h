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
 * File:   HttpServer.h
 * Author: chris
 *
 * Created on February 25, 2023, 9:17 PM
 */

#ifndef HTTPSERVER_H
#define HTTPSERVER_H

#include <cstring>
#include <thread>

#include "esp_log.h"
#include "esp_vfs.h"
#include "esp_littlefs.h"
#include "esp_https_server.h"

#include "HttpResponse.h"
#include "HttpRequest.h"

/* Max length a file path can have on storage */
#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN)

/* Max size of an individual file. Make sure this
 * value is same as that set in upload_script.html */
#define MAX_FILE_SIZE   (200*1024) // 200 KB
#define MAX_FILE_SIZE_STR "200KB"

/* Scratch buffer size */
#define SCRATCH_BUFSIZE  8192

class HttpServer {
public:
    HttpServer();
    HttpServer(const HttpServer& orig);
    virtual ~HttpServer();
    
    void initialise(std::string& basePath);
    void registerUriHandler(const std::string& uri, std::function<void(HttpRequest& request, HttpResponse& response) const> callback);
private:
    std::string basePath;
    struct file_server_data {
        /* Base path of file storage */
        char base_path[ESP_VFS_PATH_MAX + 1];
        /* Scratch buffer for temporary storage during file transfer */
        char scratch[SCRATCH_BUFSIZE];
    };
    
    file_server_data *server_data = nullptr;
    
    httpd_handle_t server = nullptr;
};

#endif /* HTTPSERVER_H */

