//
// Created by chris on 19.04.26.
//

#ifndef ANNA_ELSA_ESP32_I2C_BUS_H
#define ANNA_ELSA_ESP32_I2C_BUS_H

#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#include <cstdint>
#include <mutex>
#include <chrono>

class I2CBus;

class I2CDevice {
public:
    I2CDevice() = default;
    I2CDevice(I2CBus& bus, uint8_t addr_7bit,
              uint32_t scl_speed_hz = 400000);

    I2CDevice(const I2CDevice&) = delete;
    I2CDevice& operator=(const I2CDevice&) = delete;

    I2CDevice(I2CDevice&& other) noexcept;
    I2CDevice& operator=(I2CDevice&& other) noexcept;

    ~I2CDevice();

    esp_err_t write(const uint8_t* data, size_t len,
                    std::chrono::milliseconds timeout = std::chrono::milliseconds(100));

    esp_err_t read(uint8_t* data, size_t len,
                   std::chrono::milliseconds timeout = std::chrono::milliseconds(100));

    esp_err_t write_read(const uint8_t* wdata, size_t wlen,
                         uint8_t* rdata, size_t rlen,
                         std::chrono::milliseconds timeout = std::chrono::milliseconds(100));

    esp_err_t writeRegister(uint8_t reg,
                        uint8_t value,
                        std::chrono::milliseconds timeout = std::chrono::milliseconds(100));

    esp_err_t readRegister(uint8_t reg,
                           uint8_t& out,
                           std::chrono::milliseconds timeout = std::chrono::milliseconds(100));

    esp_err_t readRegister16(uint8_t reg,
                             uint16_t& out,
                             std::chrono::milliseconds timeout = std::chrono::milliseconds(100));

    bool valid() const { return m_dev != nullptr; }

private:
    friend class I2CBus;

    I2CBus*                 m_bus   = nullptr;
    i2c_master_dev_handle_t m_dev   = nullptr;
    std::mutex              m_mutex;
};

class I2CBus {
public:
    I2CBus() = default;
    explicit I2CBus(i2c_port_num_t port);

    I2CBus(const I2CBus&) = delete;
    I2CBus& operator=(const I2CBus&) = delete;

    I2CBus(I2CBus&& other) noexcept;
    I2CBus& operator=(I2CBus&& other) noexcept;

    ~I2CBus();

    esp_err_t init(gpio_num_t sda,
                   gpio_num_t scl,
                   uint32_t clk_source_hz = 0,   // 0 = default
                   bool pullup = true);

    bool valid() const { return m_bus != nullptr; }

    esp_err_t add_device(uint8_t addr_7bit,
                         I2CDevice& out_dev,
                         uint32_t scl_speed_hz = 400000);

    i2c_master_bus_handle_t handle() const { return m_bus; }

private:
    i2c_port_num_t          m_port = I2C_NUM_0;
    i2c_master_bus_handle_t m_bus  = nullptr;
};

#endif //ANNA_ELSA_ESP32_I2C_BUS_H