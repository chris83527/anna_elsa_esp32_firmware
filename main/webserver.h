#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include "protocol_common.h"

#include <esp_https_server.h>

void init_webserver(const char *base_path);
httpd_handle_t start_webserver(const char *base_path);
void stop_webserver(httpd_handle_t server);
void disconnect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void connect_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data);
const char* get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize);
esp_err_t download_get_handler(httpd_req_t *req);
esp_err_t index_html_get_handler(httpd_req_t *req);
esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename);