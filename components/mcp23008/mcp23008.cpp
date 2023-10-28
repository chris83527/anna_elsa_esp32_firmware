/*
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file mcp23008.c
 *
 * ESP-IDF driver for I2C 8 bit GPIO expander MCP23008
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include "mcp23008.h"

#define I2C_FREQ_HZ 1000000 // Max 1MHz for esp-idf, but device supports up to 1.7Mhz

#define REG_IODIR   0x00
#define REG_IPOL    0x01
#define REG_GPINTEN 0x02
#define REG_DEFVAL  0x03
#define REG_INTCON  0x04
#define REG_IOCON   0x05
#define REG_GPPU    0x06
#define REG_INTF    0x07
#define REG_INTCAP  0x08
#define REG_GPIO    0x09
#define REG_OLAT    0x0a

#define BIT_IOCON_INTPOL 1
#define BIT_IOCON_ODR    2
//#define BIT_IOCON_HAEN   3
#define BIT_IOCON_DISSLW 4
#define BIT_IOCON_SREAD  5

static const char *TAG = "mcp23008";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define BV(x) (1 << (x))

MCP23008::MCP23008(const i2c_port_t port = I2C_NUM_0, const uint8_t address = MCP23008_I2C_ADDR_BASE) {
    this->i2c_port = port;
    this->i2c_address = address;
}

MCP23008::~MCP23008() {
    
}

esp_err_t MCP23008::get_int_out_mode(int_out_mode_t& mode) {
    CHECK_ARG(mode);

    uint8_t r;
    CHECK(read_reg(REG_IOCON, r));

    if (r & BV(BIT_IOCON_ODR)) {
        mode = MCP23008_OPEN_DRAIN;
        return ESP_OK;
    }

    mode = r & BV(BIT_IOCON_INTPOL) ? MCP23008_ACTIVE_HIGH : MCP23008_ACTIVE_LOW;
    return ESP_OK;
}

esp_err_t MCP23008::set_int_out_mode(const int_out_mode_t mode) {
    if (mode == MCP23008_OPEN_DRAIN)
        return write_reg_bit(REG_IOCON, true, BIT_IOCON_ODR);

    return write_reg_bit(REG_IOCON, mode == MCP23008_ACTIVE_HIGH, BIT_IOCON_INTPOL);
}

esp_err_t MCP23008::port_get_mode(uint8_t& val) {
    return read_reg(REG_IODIR, val);
}

esp_err_t MCP23008::port_set_mode(uint8_t val) {
    return write_reg(REG_IODIR, val);
}

esp_err_t MCP23008::port_get_pullup(uint8_t& val) {
    return read_reg(REG_GPPU, val);
}

esp_err_t MCP23008::port_set_pullup(uint8_t val) {
    return write_reg(REG_GPPU, val);
}

esp_err_t MCP23008::port_read(uint8_t& val) {
    return read_reg(REG_GPIO, val);
}

esp_err_t MCP23008::port_write(uint8_t val) {
    return write_reg(REG_GPIO, val);
}

esp_err_t MCP23008::get_mode(uint8_t pin, gpio_mode_t& mode) {
    CHECK_ARG(mode && pin < 8);

    bool buf;
    CHECK(read_reg_bit(REG_IODIR, buf, pin));
    mode = buf ? MCP23008_GPIO_INPUT : MCP23008_GPIO_OUTPUT;

    return ESP_OK;
}

esp_err_t MCP23008::set_mode(const uint8_t pin, const gpio_mode_t mode) {
    CHECK_ARG(pin < 8);

    return write_reg_bit(REG_IODIR, mode, pin);
}

esp_err_t MCP23008::MCP23008::get_pullup(const uint8_t pin, bool& enable) {
    CHECK_ARG(pin < 8);

    return read_reg_bit(REG_GPPU, enable, pin);
}

esp_err_t MCP23008::set_pullup(const uint8_t pin, const bool enable) {
    CHECK_ARG(pin < 8);

    return write_reg_bit(REG_GPPU, enable, pin);
}

esp_err_t MCP23008::get_level(const uint8_t pin, uint32_t& val) {
    CHECK_ARG(val && pin < 8);

    bool buf;
    CHECK(read_reg_bit(REG_GPIO, buf, pin));
    val = buf ? 1 : 0;

    return ESP_OK;
}

esp_err_t MCP23008::set_level(const uint8_t pin, const uint32_t val) {
    CHECK_ARG(pin < 8);

    return write_reg_bit(REG_GPIO, val, pin);
}

esp_err_t MCP23008::port_set_interrupt(const uint8_t mask, const gpio_intr_t intr) {

    uint8_t int_en;
    CHECK(read_reg(REG_GPINTEN, int_en));

    if (intr == MCP23008_INT_DISABLED) {
        // disable interrupts
        int_en &= ~mask;
        CHECK(write_reg(REG_GPINTEN, int_en));

        return ESP_OK;
    }

    uint8_t int_con;
    CHECK(read_reg(REG_INTCON, int_con));

    if (intr == MCP23008_INT_ANY_EDGE)
        int_con &= ~mask;
    else {
        int_con |= mask;

        uint8_t int_def;
        CHECK(read_reg(REG_DEFVAL, int_def));
        if (intr == MCP23008_INT_LOW_EDGE)
            int_def |= mask;
        else
            int_def &= ~mask;
        CHECK(write_reg(REG_DEFVAL, int_def));
    }

    CHECK(write_reg(REG_INTCON, int_con));

    // enable interrupts
    int_en |= mask;
    CHECK(write_reg(REG_GPINTEN, int_en));

    return ESP_OK;
}

esp_err_t MCP23008::set_interrupt(uint8_t pin, gpio_intr_t intr) {
    CHECK_ARG(pin < 8);

    return port_set_interrupt(BV(pin), intr);
}

esp_err_t MCP23008::read_reg(const uint8_t reg, uint8_t& val) {
    CHECK_ARG(val);

    return i2c_manager_read(this->i2c_port, this->i2c_address, reg, &val, 1);
}

esp_err_t MCP23008::write_reg(const uint8_t reg, const uint8_t val) {
    return i2c_manager_write(this->i2c_port, this->i2c_address, reg, &val, 1);
}

esp_err_t MCP23008::read_reg_bit(const uint8_t reg, bool& val, const uint8_t bit) {
    CHECK_ARG(val);

    uint8_t buf;

    i2c_manager_read(this->i2c_port, this->i2c_address, reg, &buf, 1);

    val = (buf & BV(bit)) >> bit;

    return ESP_OK;
}

esp_err_t MCP23008::write_reg_bit(const uint8_t reg, const bool val, const uint8_t bit) {
    uint8_t buf;
    i2c_manager_read(this->i2c_port, this->i2c_address, reg, &buf, 1);

    buf = (buf & ~BV(bit)) | (val ? BV(bit) : 0);

    i2c_manager_write(this->i2c_port, this->i2c_address, reg, &buf, 1);

    return ESP_OK;
}
