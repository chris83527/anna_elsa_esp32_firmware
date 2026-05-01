#pragma once


#include <memory>

#include "esp_http_server.h"
#include "esp_err.h"

#include "wifi_manager.hpp"

class MainController;

class HttpController {
public:
    HttpController(WifiManager& wifi, MainController* mainController) : wifi(wifi), mainController(*mainController) {};
    ~HttpController();

    esp_err_t start();
    void stop();

    // Call periodically to push live data to all WS clients
    void broadcast_status();

private:
    static esp_err_t root_handler(httpd_req_t *req);
    static esp_err_t asset_handler(httpd_req_t *req);
    static esp_err_t provision_handler(httpd_req_t *req);
    static esp_err_t api_status_handler(httpd_req_t *req);
    static esp_err_t ws_handler(httpd_req_t *req);
    static esp_err_t ota_upload_handler(httpd_req_t *req);
    static esp_err_t captive_redirect_handler(httpd_req_t *req);
    static esp_err_t wifi_scan_handler(httpd_req_t *req);
    static esp_err_t wifi_ws_handler(httpd_req_t *req);

    static esp_err_t send_file(httpd_req_t *req, const char *path);
    static void ws_broadcast(HttpController* httpController, httpd_handle_t server, int rssi);

    WifiManager& wifi;
    MainController& mainController;

    httpd_handle_t server;
};
