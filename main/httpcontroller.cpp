//
// Created by chris on 28.04.26.
//

#include "httpcontroller.hpp"

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"

#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <memory>
#include <string>

#include "wifi_manager.hpp"

static const char *TAG = "HttpController";

WifiManager wifi_manager;

HttpController::HttpController(WifiManager& wifi) : server(nullptr) { wifi_manager = wifi; }
HttpController::~HttpController() { stop(); }

esp_err_t HttpController::start() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_ERROR_CHECK(httpd_start(&server, &config));

    httpd_uri_t root = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = root_handler,
        .user_ctx  = nullptr,
        .is_websocket = false,
        .supported_subprotocol = nullptr,

    };

    httpd_uri_t assets = {
        .uri       = "/assets/*",
        .method    = HTTP_GET,
        .handler   = asset_handler,
        .user_ctx  = nullptr
    };

    httpd_uri_t api_status = {
        .uri       = "/api/status",
        .method    = HTTP_GET,
        .handler   = api_status_handler,
        .user_ctx  = nullptr
    };

    httpd_uri_t ws = {
        .uri         = "/ws",
        .method      = HTTP_GET,
        .handler     = ws_handler,
        .user_ctx    = nullptr,
        .is_websocket = true
    };

    httpd_uri_t ota = {
        .uri       = "/update",
        .method    = HTTP_POST,
        .handler   = ota_upload_handler,
        .user_ctx  = nullptr
    };

    httpd_uri_t prov_uri = {
        .uri      = "/provision",
        .method   = HTTP_POST,
        .handler  = provision_handler,
        .user_ctx = nullptr
    };

    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &assets);
    httpd_register_uri_handler(server, &api_status);
    httpd_register_uri_handler(server, &ws);
    httpd_register_uri_handler(server, &ota);
    httpd_register_uri_handler(server, &prov_uri);

    ESP_LOGI(TAG, "HTTP server started");
    return ESP_OK;
}

void HttpController::stop() {
    if (server) {
        httpd_stop(server);
        server = nullptr;
    }
}

esp_err_t HttpController::send_file(httpd_req_t *req, const char *path) {
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "File not found: %s", path);
        return httpd_resp_send_404(req);
    }

    char buf[512];
    ssize_t r;
    while ((r = read(fd, buf, sizeof(buf))) > 0) {
        esp_err_t err = httpd_resp_send_chunk(req, buf, r);
        if (err != ESP_OK) {
            close(fd);
            return err;
        }
    }
    close(fd);
    return httpd_resp_send_chunk(req, nullptr, 0);
}

esp_err_t HttpController::root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    if (wifi_manager.is_ap_mode())
    {
        return send_file(req, "/httpd/wifiprovision.html");
    }
    return send_file(req, "/httpd/index.html");
}

esp_err_t HttpController::asset_handler(httpd_req_t *req) {
    const char *uri = req->uri; // e.g. "/assets/bootstrap.min.css"
    char path[520];
    snprintf(path, sizeof(path), "/httpd/%s", uri);

    if (strstr(uri, ".css"))
        httpd_resp_set_type(req, "text/css");
    else if (strstr(uri, ".js"))
        httpd_resp_set_type(req, "application/javascript");

    return send_file(req, path);
}

esp_err_t HttpController::provision_handler(httpd_req_t *req) {
    char buf[256];
    int len = httpd_req_recv(req, buf, sizeof(buf)-1);
    if (len <= 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");
    }
    buf[len] = '\0';

    // Very simple JSON parsing (for brevity); in production use a JSON lib
    char ssid[32] = {0};
    char pass[64] = {0};
    sscanf(buf, "{\"ssid\":\"%31[^\"]\",\"pass\":\"%63[^\"]\"}", ssid, pass);

    if (ssid[0] == '\0') {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid SSID");
    }

    wifi_manager.save_credentials(ssid, pass);

    httpd_resp_sendstr(req, "Saved, rebooting...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;
}

esp_err_t HttpController::api_status_handler(httpd_req_t *req) {
    const char *json = R"({"status":"ok","uptime":1234,"heap_free":45678})";
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

// ---------- WebSocket (ESP-IDF 5.5) ----------

esp_err_t HttpController::ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WS handshake complete");
        return ESP_OK;
    }

    httpd_ws_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.type = HTTPD_WS_TYPE_TEXT;

    // First call to get length
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) return ret;

    if (frame.len > 0) {
        std::string payload(frame.len, '\0');
        frame.payload = reinterpret_cast<uint8_t *>(payload.data());
        ret = httpd_ws_recv_frame(req, &frame, frame.len);
        if (ret != ESP_OK) return ret;

        ESP_LOGI(TAG, "WS recv: %.*s", frame.len, payload.c_str());
        // You can parse commands here if you want
    }

    return ESP_OK;
}

void HttpController::ws_broadcast(httpd_handle_t server, int uptime_sec, int heap_free) {
    if (!server) return;

    char msg[128];
    snprintf(msg, sizeof(msg),
             R"({"uptime":%d,"heap_free":%d})",
             uptime_sec, heap_free);

    httpd_ws_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.type = HTTPD_WS_TYPE_TEXT;
    frame.payload = reinterpret_cast<uint8_t *>(msg);
    frame.len = strlen(msg);

    // In ESP-IDF 5.x, we can iterate sockets and check WS state
    size_t max_fds = CONFIG_LWIP_MAX_SOCKETS;
    for (int fd = 0; fd < (int)max_fds; ++fd) {
        if (httpd_ws_get_fd_info(server, fd) == HTTPD_WS_CLIENT_WEBSOCKET) {
            httpd_ws_send_frame_async(server, fd, &frame);
        }
    }
}

void HttpController::broadcast_status(int uptime_sec, int heap_free) {
    ws_broadcast(server, uptime_sec, heap_free);
}

// ---------- OTA upload (ESP-IDF 5.5) ----------

esp_err_t HttpController::ota_upload_handler(httpd_req_t *req) {
    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(nullptr);
    if (!update_partition) {
        ESP_LOGE(TAG, "No OTA partition");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA to partition subtype %d at 0x%lx",
             update_partition->subtype, (unsigned long)update_partition->address);

    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return err;
    }

    int remaining = req->content_len;
    char buf[1024];

    while (remaining > 0) {
        int to_read = remaining > (int)sizeof(buf) ? sizeof(buf) : remaining;
        int r = httpd_req_recv(req, buf, to_read);
        if (r <= 0) {
            ESP_LOGE(TAG, "OTA recv error: %d", r);
            esp_ota_end(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA recv error");
            return ESP_FAIL;
        }

        err = esp_ota_write(ota_handle, buf, r);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            esp_ota_end(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
            return err;
        }

        remaining -= r;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        return err;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA set boot failed");
        return err;
    }

    ESP_LOGI(TAG, "OTA OK, rebooting...");
    httpd_resp_sendstr(req, "OK, rebooting");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;
}
