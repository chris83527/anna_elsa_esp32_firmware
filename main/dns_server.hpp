//
// Created by chris on 29.04.26.
//

#pragma once

#include <thread>
#include <atomic>

class DnsServer {
public:
    DnsServer();
    ~DnsServer();

    void start(const char* ap_ip = "192.168.4.1");
    void stop();

private:
    void dns_thread(const char* ap_ip);

    std::thread worker;
    std::atomic<bool> running{false};
};

