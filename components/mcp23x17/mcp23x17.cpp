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
 * @file mcp23x17.c
 *
 * ESP-IDF driver for I2C/SPI 16 bit GPIO expanders MCP23017/MCP23S17
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include <string.h>
#include "mcp23x17.h"

static const char *TAG = "mcp23x17";

#define I2C_FREQ_HZ 1000000  // Max 1MHz for esp-idf, but device supports up to 1.7Mhz

#define REG_IODIRA   0x00
#define REG_IODIRB   0x01
#define REG_IPOLA    0x02
#define REG_IPOLB    0x03
#define REG_GPINTENA 0x04
#define REG_GPINTENB 0x05
#define REG_DEFVALA  0x06
#define REG_DEFVALB  0x07
#define REG_INTCONA  0x08
#define REG_INTCONB  0x09
#define REG_IOCON    0x0A
#define REG_GPPUA    0x0C
#define REG_GPPUB    0x0D
#define REG_INTFA    0x0E
#define REG_INTFB    0x0F
#define REG_INTCAPA  0x10
#define REG_INTCAPB  0x11
#define REG_GPIOA    0x12
#define REG_GPIOB    0x13
#define REG_OLATA    0x14
#define REG_OLATB    0x15

#define BIT_IOCON_INTPOL 1
#define BIT_IOCON_ODR    2
#define BIT_IOCON_HAEN   3
#define BIT_IOCON_DISSLW 4
#define BIT_IOCON_SEQOP  5
#define BIT_IOCON_MIRROR 6
#define BIT_IOCON_BANK   7

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define BV(x) (1 << (x))

MCP23x17::MCP23x17(const i2c_port_t port = I2C_NUM_0, const uint8_t address = MCP23X17_ADDR_BASE) {
    this->i2c_port = port;
    this->i2c_address = address;
}

MCP23x17::~MCP23x17() {

}

esp_err_t MCP23x17::get_int_out_mode(int_out_mode_t& mode) {
    CHECK_ARG(mode);

    bool buf;
    CHECK(read_reg_bit_8(REG_IOCON, buf, BIT_IOCON_ODR));
    if (buf) {
        mode = MCP23X17_OPEN_DRAIN;
        return ESP_OK;
    }
    CHECK(read_reg_bit_8(REG_IOCON, buf, BIT_IOCON_INTPOL));
    mode = buf ? MCP23X17_ACTIVE_HIGH : MCP23X17_ACTIVE_LOW;

    return ESP_OK;
}

esp_err_t MCP23x17::set_int_out_mode(const int_out_mode_t mode) {
    if (mode == MCP23X17_OPEN_DRAIN)
        return write_reg_bit_8(REG_IOCON, true, BIT_IOCON_ODR);

    // The INTPOL bit is only functional if the ODR bit is cleared.
    write_reg_bit_8(REG_IOCON, false, BIT_IOCON_ODR);
    return write_reg_bit_8(REG_IOCON, mode == MCP23X17_ACTIVE_HIGH, BIT_IOCON_INTPOL);
}

esp_err_t MCP23x17::port_get_mode(uint16_t& val) {
    return read_reg_16(REG_IODIRA, val);
}

esp_err_t MCP23x17::port_set_mode(const uint16_t val) {
    return write_reg_16(REG_IODIRA, val);
}

esp_err_t MCP23x17::port_get_pullup(uint16_t& val) {
    return read_reg_16(REG_GPPUA, val);
}

esp_err_t MCP23x17::port_set_pullup(const uint16_t val) {
    return write_reg_16(REG_GPPUA, val);
}

esp_err_t MCP23x17::port_read(uint16_t& val) {
    return read_reg_16(REG_GPIOA, val);
}

esp_err_t MCP23x17::port_write(const uint16_t val) {
    return write_reg_16(REG_GPIOA, val);
}

esp_err_t MCP23x17::get_mode(const uint8_t pin, gpio_mode_t& mode) {
    CHECK_ARG(mode);

    bool buf;
    CHECK(read_reg_bit_16(REG_IODIRA, buf, pin));
    mode = buf ? MCP23X17_GPIO_INPUT : MCP23X17_GPIO_OUTPUT;

    return ESP_OK;
}

esp_err_t MCP23x17::set_mode(const uint8_t pin, const gpio_mode_t mode) {
    return write_reg_bit_16(REG_IODIRA, mode, pin);
}

esp_err_t MCP23x17::get_pullup(const uint8_t pin, bool& enable) {
    return read_reg_bit_16(REG_GPPUA, enable, pin);
}

esp_err_t MCP23x17::set_pullup(const uint8_t pin, const bool enable) {
    return write_reg_bit_16(REG_GPPUA, enable, pin);
}

esp_err_t MCP23x17::get_level(uint8_t pin, uint32_t& val) {
    CHECK_ARG(val);

    bool buf;
    CHECK(read_reg_bit_16(REG_GPIOA, buf, pin));
    val = buf ? 1 : 0;

    return ESP_OK;
}

esp_err_t MCP23x17::set_level(const uint8_t pin, const uint32_t val) {
    return write_reg_bit_16(REG_GPIOA, val, pin);
}

esp_err_t MCP23x17::port_set_interrupt(const uint16_t mask, const gpio_intr_t intr) {
    uint16_t int_en;
    CHECK(read_reg_16(REG_GPINTENA, int_en));

    if (intr == MCP23X17_INT_DISABLED) {
        // disable interrupts
        int_en &= ~mask;
        CHECK(write_reg_16(REG_GPINTENA, int_en));

        return ESP_OK;
    }

    uint16_t int_con;
    CHECK(read_reg_16(REG_INTCONA, int_con));

    if (intr == MCP23X17_INT_ANY_EDGE)
        int_con &= ~mask;
    else {
        int_con |= mask;

        uint16_t int_def;
        CHECK(read_reg_16(REG_DEFVALA, int_def));
        if (intr == MCP23X17_INT_LOW_EDGE)
            int_def |= mask;
        else
            int_def &= ~mask;
        CHECK(write_reg_16(REG_DEFVALA, int_def));
    }

    CHECK(write_reg_16(REG_INTCONA, int_con));

    // enable interrupts
    int_en |= mask;
    CHECK(write_reg_16(REG_GPINTENA, int_en));

    return ESP_OK;
}


// ---------------------

esp_err_t MCP23x17::set_interrupt(uint8_t pin, gpio_intr_t intr) {
    return port_set_interrupt(BV(pin), intr);
}

esp_err_t MCP23x17::read_reg_16(const uint8_t reg, uint16_t& val) {
    CHECK_ARG(val);
    uint8_t data[2];

    i2c_manager_read(this->i2c_port, this->i2c_address, reg, data, 2);

    val = (data[ 0 ] << 8 | data[ 1 ]);
    
    return ESP_OK;
}

esp_err_t MCP23x17::write_reg_16(const uint8_t reg, const uint16_t val) {
    uint8_t data[2];
    data[0] = (val >> 8);
    data[1] = (val & 0xff);

    return i2c_manager_write(this->i2c_port, this->i2c_address, reg, data, 2);
}

esp_err_t MCP23x17::write_reg_bit_16(const uint8_t reg, bool val, uint8_t bit) {
    uint8_t data[2];

    i2c_manager_read(this->i2c_port, this->i2c_address, reg, data, 2);

    uint16_t buf16 = (data[ 0 ] << 8 | data[ 1 ]);

    buf16 = (buf16 & ~BV(bit)) | (val ? BV(bit) : 0);
    data[0] = (buf16 >> 8);
    data[1] = (buf16 & 0xff);

    return i2c_manager_write(this->i2c_port, this->i2c_address, reg, data, 2);
}

esp_err_t MCP23x17::read_reg_bit_8(const uint8_t reg, bool& val, uint8_t bit) {
    CHECK_ARG(val);

    uint8_t buf;

    i2c_manager_read(this->i2c_port, this->i2c_address, reg, &buf, 1);

    val = (buf & BV(bit)) >> bit;

    return ESP_OK;
}

esp_err_t MCP23x17::write_reg_bit_8(const uint8_t reg, const bool val, const uint8_t bit) {

    uint8_t buf;

    i2c_manager_read(this->i2c_port, this->i2c_address, reg, &buf, 1);
    buf = (buf & ~BV(bit)) | (val ? BV(bit) : 0);
    i2c_manager_write(this->i2c_port, this->i2c_address, reg, &buf, 1);

    return ESP_OK;
}

esp_err_t MCP23x17::read_reg_bit_16(const uint8_t reg, bool& val, const uint8_t bit) {
    uint16_t buf;

    CHECK(read_reg_16(reg, buf));

    val = (buf & BV(bit)) >> bit;

    return ESP_OK;
}
