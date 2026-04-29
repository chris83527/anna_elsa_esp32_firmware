//
// Created by chris on 29.04.26.
//

#include "dns_server.hpp"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_log.h"

static const char* TAG = "DnsServer";

DnsServer::DnsServer() {}
DnsServer::~DnsServer() { stop(); }

void DnsServer::start(const char* ap_ip) {
    running = true;
    worker = std::thread(&DnsServer::dns_thread, this, ap_ip);
    worker.detach();
}

void DnsServer::stop() {
    running = false;
}

void DnsServer::dns_thread(const char* ap_ip) {
    ESP_LOGI(TAG, "DNS server starting");

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create DNS socket");
        return;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(53);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (sockaddr*)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "DNS bind failed");
        close(sock);
        return;
    }

    uint8_t buffer[512];
    sockaddr_in client;
    socklen_t client_len = sizeof(client);

    uint32_t ip = inet_addr(ap_ip);

    while (running) {
        int len = recvfrom(sock, buffer, sizeof(buffer), 0,
                           (sockaddr*)&client, &client_len);
        if (len <= 0) continue;

        // DNS header: set response flags
        buffer[2] |= 0x80; // QR = response
        buffer[3] |= 0x80; // RA = recursion available
        buffer[7] = 1;     // ANCOUNT = 1

        // Append answer section
        int pos = len;

        // Pointer to domain name in question section
        buffer[pos++] = 0xC0;
        buffer[pos++] = 0x0C;

        // Type A, Class IN
        buffer[pos++] = 0x00;
        buffer[pos++] = 0x01;
        buffer[pos++] = 0x00;
        buffer[pos++] = 0x01;

        // TTL
        buffer[pos++] = 0x00;
        buffer[pos++] = 0x00;
        buffer[pos++] = 0x00;
        buffer[pos++] = 0x3C;

        // RDLENGTH = 4
        buffer[pos++] = 0x00;
        buffer[pos++] = 0x04;

        // IP address
        memcpy(&buffer[pos], &ip, 4);
        pos += 4;

        sendto(sock, buffer, pos, 0, (sockaddr*)&client, client_len);
    }

    close(sock);
    ESP_LOGI(TAG, "DNS server stopped");
}
