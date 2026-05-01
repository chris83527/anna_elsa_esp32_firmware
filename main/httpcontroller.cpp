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
#include "maincontroller.h"

static const char* TAG = "HttpController";

HttpController::~HttpController() { stop(); }

esp_err_t HttpController::start()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 10;
    config.server_port = 80;
    config.uri_match_fn = httpd_uri_match_wildcard;

    ESP_ERROR_CHECK(httpd_start(&server, &config));

    httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_handler,
        .user_ctx = this,
        .is_websocket = false,
        .supported_subprotocol = nullptr,

    };

    httpd_uri_t assets = {
        .uri = "/httpd/*",
        .method = HTTP_GET,
        .handler = asset_handler,
        .user_ctx = this
    };

    httpd_uri_t api_status = {
        .uri = "/api/status",
        .method = HTTP_GET,
        .handler = api_status_handler,
        .user_ctx = this
    };

    httpd_uri_t api_status_update = {
        .uri = "/api/status",
        .method = HTTP_POST,
        .handler = api_status_update_handler,
        .user_ctx = this
    };

    httpd_uri_t ws = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = this,
        .is_websocket = true
    };

    httpd_uri_t ota = {
        .uri = "/update",
        .method = HTTP_POST,
        .handler = ota_upload_handler,
        .user_ctx = this
    };

    httpd_uri_t prov_uri = {
        .uri = "/provision",
        .method = HTTP_POST,
        .handler = provision_handler,
        .user_ctx = this
    };

    httpd_uri_t scan_uri = {
        .uri = "/wifi/scan",
        .method = HTTP_GET,
        .handler = wifi_scan_handler,
        .user_ctx = this
    };

    httpd_uri_t captive = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = captive_redirect_handler,
        .user_ctx = this // <‑‑ pass instance pointer
    };

    httpd_uri_t wifi_ws = {
        .uri = "/ws/wifi",
        .method = HTTP_GET,
        .handler = wifi_ws_handler,
        .user_ctx = this,
        .is_websocket = true
    };


    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &assets);
    httpd_register_uri_handler(server, &api_status);
    httpd_register_uri_handler(server, &api_status_update);
    httpd_register_uri_handler(server, &ws);
    httpd_register_uri_handler(server, &ota);
    httpd_register_uri_handler(server, &prov_uri);
    httpd_register_uri_handler(server, &wifi_ws);
    httpd_register_uri_handler(server, &scan_uri);
    httpd_register_uri_handler(server, &captive);

    ESP_LOGI(TAG, "HTTP server started");

    // broadcast data via websocket
    broadcast_status();

    return ESP_OK;
}

void HttpController::stop()
{
    if (server)
    {
        httpd_stop(server);
        server = nullptr;
    }
}

esp_err_t HttpController::send_file(httpd_req_t* req, const char* path)
{
    int fd = open(path, O_RDONLY);
    if (fd < 0)
    {
        ESP_LOGE(TAG, "File not found: %s", path);
        return httpd_resp_send_404(req);
    }

    char buf[512];
    ssize_t r;
    while ((r = read(fd, buf, sizeof(buf))) > 0)
    {
        esp_err_t err = httpd_resp_send_chunk(req, buf, r);
        if (err != ESP_OK)
        {
            close(fd);
            return err;
        }
    }
    close(fd);
    return httpd_resp_send_chunk(req, nullptr, 0);
}

esp_err_t HttpController::root_handler(httpd_req_t* req)
{
    httpd_resp_set_type(req, "text/html");
    return send_file(req, "/httpd/index.html");
}

esp_err_t HttpController::asset_handler(httpd_req_t* req)
{
    const char* uri = req->uri; // e.g. "/httpd/bootstrap.min.css"
    char path[520];
    snprintf(path, sizeof(path), "%s", uri);

    if (strstr(uri, ".css"))
        httpd_resp_set_type(req, "text/css");
    else if (strstr(uri, ".js"))
        httpd_resp_set_type(req, "application/javascript");

    return send_file(req, path);
}

esp_err_t HttpController::captive_redirect_handler(httpd_req_t* req)
{
    // Only active in AP mode
    auto* self = static_cast<HttpController*>(req->user_ctx);

    if (!self->wifi.is_ap_mode())
    {
        return httpd_resp_send_404(req);
    }

    httpd_resp_set_status(req, "302 Temporary Redirect");
    httpd_resp_set_hdr(req, "Location", "/provision");
    return httpd_resp_send(req, nullptr, 0);
}

esp_err_t HttpController::provision_handler(httpd_req_t* req)
{
    auto* self = static_cast<HttpController*>(req->user_ctx);

    char buf[256];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");
    }
    buf[len] = '\0';

    // Very simple JSON parsing (for brevity); in production use a JSON lib
    char ssid[32] = {0};
    char pass[64] = {0};
    sscanf(buf, R"({"ssid":"%31[^"]","pass":"%63[^"]"})", ssid, pass);

    if (ssid[0] == '\0')
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid SSID");
    }

    self->wifi.save_credentials(ssid, pass);

    httpd_resp_sendstr(req, "Saved, rebooting...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;
}

esp_err_t HttpController::api_status_handler(httpd_req_t* req)
{
    auto* self = static_cast<HttpController*>(req->user_ctx);

    uint16_t bank = self->mainController.getPaymentController().getBank();
    uint16_t credit = self->mainController.getPaymentController().getCredit();
    uint16_t gameCount = self->mainController.getPaymentController().getGameCount();
    uint16_t incomeTotal = self->mainController.getPaymentController().getIncomeTotal();
    uint16_t payoutTotal = self->mainController.getPaymentController().getPayoutTotal();
    uint16_t transfer = self->mainController.getPaymentController().getTransfer();
    int volume = self->mainController.getAudioController().getVolume();

    // credits, number of games, payout stats etc.
    std::string json = "{";
    json += ",\"bank\":\"" + std::to_string(bank)+ "\"";
    json += ",\"credit\":\"" + std::to_string(credit)+ "\"";
    json += ",\"gameCount\":\"" + std::to_string(gameCount)+ "\"";
    json += ",\"transfer\":\"" + std::to_string(transfer)+ "\"";
    json += ",\"incomeTotal\":\"" + std::to_string(incomeTotal)+ "\"";
    json += ",\"payoutTotal\":\"" + std::to_string(payoutTotal)+ "\"";
    json += ",\"volume\":\"" + std::to_string(volume) + "\"";
    json += "}";


    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json.c_str(), HTTPD_RESP_USE_STRLEN);
}

esp_err_t HttpController::wifi_scan_handler(httpd_req_t* req)
{
    auto* self = static_cast<HttpController*>(req->user_ctx);

    auto networks = self->wifi.scan_networks();

    std::string json = "[";
    for (size_t i = 0; i < networks.size(); i++)
    {
        const auto& n = networks[i];
        json += "{";
        json += "\"ssid\":\"" + n.ssid + "\",";
        json += "\"rssi\":" + std::to_string(n.rssi) + ",";
        json += "\"auth\":" + std::to_string(n.auth) + ",";
        json += "\"hidden\":" + std::string(n.is_hidden ? "true" : "false");
        json += "}";
        if (i + 1 < networks.size()) json += ",";
    }
    json += "]";

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json.c_str(), json.size());
}

esp_err_t HttpController::api_status_update_handler(httpd_req_t* req)
{
    esp_err_t ret;
    auto* self = static_cast<HttpController*>(req->user_ctx);

    char buf[256];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");
    }
    buf[len] = '\0';

    // Very simple JSON parsing (for brevity); in production use a JSON lib
    char volume[4] = {0};
    sscanf(buf, R"({"volume":"%3[^"]"})", volume);

    self->mainController.getAudioController().setVolume(atoi(volume));

    ret = httpd_resp_set_status(req, HTTPD_200);
    return httpd_resp_send(req, nullptr, 0);
}

// ---------- WebSocket (ESP-IDF 5.5) ----------

esp_err_t HttpController::ws_handler(httpd_req_t* req)
{
    if (req->method == HTTP_GET)
    {
        ESP_LOGI(TAG, "WS handshake complete");
        return ESP_OK;
    }

    httpd_ws_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.type = HTTPD_WS_TYPE_TEXT;

    // First call to get length
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) return ret;

    if (frame.len > 0)
    {
        std::string payload(frame.len, '\0');
        frame.payload = reinterpret_cast<uint8_t*>(payload.data());
        ret = httpd_ws_recv_frame(req, &frame, frame.len);
        if (ret != ESP_OK) return ret;

        ESP_LOGI(TAG, "WS recv: %.*s", frame.len, payload.c_str());
        // You can parse commands here if you want
    }

    return ESP_OK;
}

void HttpController::ws_broadcast(HttpController* httpController, httpd_handle_t server, int rssi)
{
    if (!server) return;

    uint16_t bank = httpController->mainController.getPaymentController().getBank();
    uint16_t credit = httpController->mainController.getPaymentController().getCredit();
    uint16_t gameCount = httpController->mainController.getPaymentController().getGameCount();
    uint16_t incomeTotal = httpController->mainController.getPaymentController().getIncomeTotal();
    uint16_t payoutTotal = httpController->mainController.getPaymentController().getPayoutTotal();
    int volume = httpController->mainController.getAudioController().getVolume();

    // credits, number of games, payout stats etc.
    std::string json = "{";
    json += ",\"bank\":\"" + std::to_string(bank)+ "\"";
    json += ",\"credit\":\"" + std::to_string(credit)+ "\"";
    json += ",\"gameCount\":\"" + std::to_string(gameCount)+ "\"";
    json += ",\"incomeTotal\":\"" + std::to_string(incomeTotal)+ "\"";
    json += ",\"payoutTotal\":\"" + std::to_string(payoutTotal)+ "\"";
    json += ",\"volume\":\"" + std::to_string(volume) + "\"";
    json += "}";

    httpd_ws_frame_t frame{};
    frame.type = HTTPD_WS_TYPE_TEXT;
    frame.payload = reinterpret_cast<uint8_t*>(json.data());
    frame.len = json.size();

    // In ESP-IDF 5.x, we can iterate sockets and check WS state
    size_t max_fds = CONFIG_LWIP_MAX_SOCKETS;
    for (int fd = 0; fd < (int)max_fds; ++fd)
    {
        if (httpd_ws_get_fd_info(server, fd) == HTTPD_WS_CLIENT_WEBSOCKET)
        {
            httpd_ws_send_frame_async(server, fd, &frame);
        }
    }
}

void HttpController::broadcast_status()
{
    ws_broadcast(this, server, wifi.get_rssi());
}

// ---------- OTA upload (ESP-IDF 5.5) ----------

esp_err_t HttpController::ota_upload_handler(httpd_req_t* req)
{
    esp_ota_handle_t ota_handle = 0;
    const esp_partition_t* update_partition = esp_ota_get_next_update_partition(nullptr);
    if (!update_partition)
    {
        ESP_LOGE(TAG, "No OTA partition");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA to partition subtype %d at 0x%lx",
             update_partition->subtype, (unsigned long)update_partition->address);

    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return err;
    }

    int remaining = req->content_len;
    char buf[1024];

    while (remaining > 0)
    {
        int to_read = remaining > (int)sizeof(buf) ? sizeof(buf) : remaining;
        int r = httpd_req_recv(req, buf, to_read);
        if (r <= 0)
        {
            ESP_LOGE(TAG, "OTA recv error: %d", r);
            esp_ota_end(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA recv error");
            return ESP_FAIL;
        }

        err = esp_ota_write(ota_handle, buf, r);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            esp_ota_end(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
            return err;
        }

        remaining -= r;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        return err;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
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

esp_err_t HttpController::wifi_ws_handler(httpd_req_t* req)
{
    auto* self = static_cast<HttpController*>(req->user_ctx);

    if (req->method == HTTP_GET)
    {
        ESP_LOGI("HttpController", "WiFi WS handshake");
        return ESP_OK;
    }

    httpd_ws_frame_t frame{};
    frame.type = HTTPD_WS_TYPE_TEXT;

    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) return ret;

    if (frame.len > 0)
    {
        std::string payload(frame.len, '\0');
        frame.payload = reinterpret_cast<uint8_t*>(payload.data());

        esp_err_t ret = httpd_ws_recv_frame(req, &frame, frame.len);
        if (ret != ESP_OK) return ret;

        // Very simple protocol: {"cmd":"scan"} or {"cmd":"status"}
        if (payload.find("\"scan\"") != std::string::npos)
        {
            auto nets = self->wifi.scan_networks();

            std::string json = R"({"type":"scan","networks":[)";
            for (size_t i = 0; i < nets.size(); i++)
            {
                const auto& n = nets[i];
                json += "{";
                json += "\"ssid\":\"" + n.ssid + "\",";
                json += "\"rssi\":" + std::to_string(n.rssi) + ",";
                json += "\"auth\":" + std::to_string(n.auth) + ",";
                json += "\"hidden\":" + std::string(n.is_hidden ? "true" : "false");
                json += "}";
                if (i + 1 < nets.size()) json += ",";
            }
            json += "]}";

            httpd_ws_frame_t out{};
            out.type = HTTPD_WS_TYPE_TEXT;
            out.payload = (uint8_t*)json.data();
            out.len = json.size();
            return httpd_ws_send_frame(req, &out);
        }

        if (payload.find("\"status\"") != std::string::npos)
        {
            int rssi = self->wifi.get_rssi();
            bool connected = self->wifi.is_connected();

            char buf[128];
            snprintf(buf, sizeof(buf),
                     R"({"type":"status","connected":%s,"rssi":%d})",
                     connected ? "true" : "false", rssi);

            httpd_ws_frame_t out{};
            out.type = HTTPD_WS_TYPE_TEXT;
            out.payload = reinterpret_cast<uint8_t*>(buf);
            out.len = strlen(buf);
            return httpd_ws_send_frame(req, &out);
        }
    }

    return ESP_OK;
}

