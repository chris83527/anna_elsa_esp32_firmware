#pragma once

#include "cctalk.hpp"
#include <chrono>
#include "driver/uart.h"
#include "esp_log.h"

class EspIdfCctalkUart : public ICctalkUart {
public:
    EspIdfCctalkUart(uart_port_t uart_num)
        : uart_num_(uart_num) {}

    int write(const uint8_t* data, size_t len,  std::chrono::milliseconds timeout) override {
        // uart_write_bytes is blocking; timeout is handled by driver config
        int res = uart_write_bytes(uart_num_, data, len);
        return res;
    }

    int read(uint8_t* data, size_t len, std::chrono::milliseconds timeout) override {
        int res = uart_read_bytes(uart_num_, data, len, pdMS_TO_TICKS(timeout.count()));
        return res;
    }

private:
    uart_port_t uart_num_;
};
