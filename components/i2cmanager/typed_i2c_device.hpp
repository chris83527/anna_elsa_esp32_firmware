//
// Created by chris on 19.04.26.
//

#pragma once

#ifndef ANNA_ELSA_ESP32_TYPED_I2C_DEVICE_H
#define ANNA_ELSA_ESP32_TYPED_I2C_DEVICE_H

#include "i2c_bus.hpp"
#include <type_traits>
#include <chrono>
#include <cstdint>

class TypedI2CDevice
{
public:
    TypedI2CDevice(I2CBus& bus, uint8_t addr_7bit)
        : m_dev(bus, addr_7bit)
    {
    }

    [[nodiscard]] bool valid() const { return m_dev.valid(); }

    esp_err_t write(const std::vector<uint8_t>& data,
                       std::chrono::milliseconds timeout = std::chrono::milliseconds(100))
    {
        if (data.empty())
            return ESP_ERR_INVALID_ARG;

        return m_dev.write(data.data(), data.size(), timeout);
    }

    esp_err_t read(std::vector<uint8_t>& out,
                      size_t len,
                      std::chrono::milliseconds timeout = std::chrono::milliseconds(100))
    {
        if (len == 0)
            return ESP_ERR_INVALID_ARG;

        out.resize(len);

        return m_dev.read(out.data(), len, timeout);
    }

    esp_err_t writeReg(uint8_t reg, uint8_t data, std::chrono::milliseconds timeout = std::chrono::milliseconds(100))
    {
       return m_dev.writeRegister(reg, data, timeout);
    }

    // -------- Vector Write --------
    esp_err_t writeReg(uint8_t reg,
                       const std::vector<uint8_t>& data,
                       std::chrono::milliseconds timeout = std::chrono::milliseconds(100))
    {
        if (data.empty())
            return ESP_ERR_INVALID_ARG;

        std::vector<uint8_t> buf;
        buf.reserve(1 + data.size());

        buf.push_back(reg);
        buf.insert(buf.end(), data.begin(), data.end());

        return m_dev.write(buf.data(), buf.size(), timeout);
    }

    // -------- Vector Read --------
    esp_err_t readReg(uint8_t reg,
                      std::vector<uint8_t>& out,
                      size_t len,
                      std::chrono::milliseconds timeout = std::chrono::milliseconds(100))
    {
        if (len == 0)
            return ESP_ERR_INVALID_ARG;

        out.resize(len);

        return m_dev.write_read(&reg, 1, out.data(), len, timeout);
    }

protected:
    I2CDevice m_dev;
};


#endif //ANNA_ELSA_ESP32_TYPED_I2C_DEVICE_H
