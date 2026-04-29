#pragma once


#include <memory>

#include "esp_http_server.h"
#include "esp_err.h"

#include "wifi_manager.hpp"

class HttpController {
public:
    HttpController(WifiManager& wifi);
    ~HttpController();

    esp_err_t start();
    void stop();

    // Call periodically to push live data to all WS clients
    void broadcast_status(int uptime_sec, int heap_free);

private:
    static esp_err_t root_handler(httpd_req_t *req);
    static esp_err_t asset_handler(httpd_req_t *req);
    static esp_err_t provision_handler(httpd_req_t *req);
    static esp_err_t api_status_handler(httpd_req_t *req);
    static esp_err_t ws_handler(httpd_req_t *req);
    static esp_err_t ota_upload_handler(httpd_req_t *req);
    static esp_err_t captive_redirect_handler(httpd_req_t *req);

    static esp_err_t send_file(httpd_req_t *req, const char *path);
    static void ws_broadcast(httpd_handle_t server, int uptime_sec, int heap_free);

    WifiManager& wifi;

    httpd_handle_t server;
};
