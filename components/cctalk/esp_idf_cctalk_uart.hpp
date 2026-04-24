#pragma once

#include "cctalk.hpp"
#include <chrono>
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "rom/uart.h"

#define MAX_BUFFER_SIZE 1024


class EspIdfCctalkUart : public ICctalkUart
{
public:
    explicit EspIdfCctalkUart(const uart_port_t uart_num, const int txPin, const int rxPin)
        : uart_num_(uart_num)
    {
        ESP_LOGI(TAG, "EspIdfCctalkUart constructor");

        int intr_alloc_flags = 0;

        uart_config_t uart_config = {
            .baud_rate = 9600,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_DEFAULT,
            .flags = {}
        };


#if CONFIG_UART_ISR_IN_IRAM
        intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

        ESP_ERROR_CHECK(uart_driver_install(uart_num_, MAX_BUFFER_SIZE * 2, 0, 0, nullptr, intr_alloc_flags));
        // Configure UART parameters
        ESP_ERROR_CHECK(uart_param_config(uart_num_, &uart_config));
        ESP_ERROR_CHECK(
            uart_set_pin(uart_num_, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    }

    ~EspIdfCctalkUart() override
    {
        uart_driver_delete(uart_num_);
    }

    const char* TAG = "cctalk_uart";

    /**
     * Writes data to the UART
     *
     * @param data
     * @param len
     * @param timeout
     * @return
     */
    int write(const uint8_t* data, size_t len, std::chrono::milliseconds timeout) override
    {
        auto timeoutMs = static_cast<uint32_t>(timeout.count());
        //ESP_LOGI(TAG, "UART write called. Timeout set to %lums", timeoutMs);
        //ESP_LOGI(TAG, "%d bytes to write", len);
        // uart_write_bytes is blocking; timeout is handled by driver config
        int res = uart_write_bytes(uart_num_, data, len);
        ESP_LOGI(TAG, "%d bytes written", res);
        esp_err_t err = uart_wait_tx_done(uart_num_, pdMS_TO_TICKS(timeoutMs)); // wait 1s max

        return res;
    }


    /**
     * Reads data from the UART
     *
     * @param data A pointer to an array that will contain the data read from the UART
     * @param len The number of bytes to read from the UART
     * @param timeout The number of milliseconds to wait before giving up
     * @return The total number of bytes read
     */
    int read(uint8_t* data, size_t len, std::chrono::milliseconds timeout) override
    {
        auto timeoutMs = static_cast<uint32_t>(timeout.count());
        return  uart_read_bytes(
                uart_num_,
                data,
                len,
                pdMS_TO_TICKS(timeoutMs)
            );
        //ESP_LOGI(TAG, "UART read called with timeout %lu", timeoutMs);
        int bytesReadOut = 0;

        int64_t start = esp_timer_get_time(); // microseconds

        while (bytesReadOut < len) {

            // Remaining time
            int64_t elapsed = (esp_timer_get_time() - start) / 1000;
            if (elapsed >= timeoutMs) {
                return bytesReadOut;
            }

            uint32_t remainingTimeout =
                timeoutMs - static_cast<uint32_t>(elapsed);

            int chunk = uart_read_bytes(
                uart_num_,
                data + bytesReadOut,
                len- bytesReadOut,
                pdMS_TO_TICKS(remainingTimeout)
            );

            if (chunk > 0)
            {
                //ESP_LOGI(TAG, "chunk read = %d", chunk);
            }

            if (chunk < 0) {
                return -1; // UART error
            }

            if (chunk == 0) {
                // No data this iteration → loop again until timeout
                continue;
            }

            bytesReadOut += chunk;
        }

        return bytesReadOut;
    }

private:
    uart_port_t uart_num_;
};
