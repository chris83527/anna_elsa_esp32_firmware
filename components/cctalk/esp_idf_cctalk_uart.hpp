#pragma once

#include "cctalk.hpp"
#include <chrono>
#include "driver/uart.h"
#include "esp_log.h"
#include "rom/uart.h"

#define MAX_BUFFER_SIZE 1024

class EspIdfCctalkUart : public ICctalkUart
{
public:
    explicit EspIdfCctalkUart(uart_port_t uart_num, int txPin, int rxPin)
        : uart_num_(uart_num)
    {
        int intr_alloc_flags = 0;

        uart_config_t uart_config = {
            .baud_rate = 9600,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_APB,
            .flags = {}
        };

#if CONFIG_UART_ISR_IN_IRAM
        intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif
        uart_driver_install(uart_num_, MAX_BUFFER_SIZE, 0, 0, nullptr, intr_alloc_flags);

        ESP_ERROR_CHECK(
            uart_set_pin(uart_num_, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

        // Configure UART parameters
        ESP_ERROR_CHECK(uart_param_config(uart_num_, &uart_config));
    }

    ~EspIdfCctalkUart() override
    {
        uart_driver_delete(uart_num_);
    }

    int write(const uint8_t* data, size_t len, std::chrono::milliseconds timeout) override
    {
        // uart_write_bytes is blocking; timeout is handled by driver config
        int res = uart_write_bytes(uart_num_, data, len);
        uart_wait_tx_done(uart_num_, pdMS_TO_TICKS(timeout.count())); // wait 1s max

        return res;
    }

    int read(uint8_t* data, size_t len, std::chrono::milliseconds timeout) override
    {
        int res = uart_read_bytes(uart_num_, data, len, pdMS_TO_TICKS(timeout.count()));
        return res;
    }

private:
    uart_port_t uart_num_;
};
