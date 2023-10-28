/*
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
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
 * @file ht16k33.h
 *
 * ESP-IDF driver for Holtek HT16K33 I2C LED Matrix driver chip
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __HT16K33_H__
#define __HT16K33_H__

#include <cstddef>
#include <cstdbool>
#include <esp_err.h>
#include <i2c_manager.h>

#define HT16K33_ADDR_BASE 0x70

class HT16K33 {
public:

    HT16K33(const i2c_port_t port, const uint8_t address);
    ~HT16K33();   

    esp_err_t set_digits(uint8_t val);
    esp_err_t display_on();
    esp_err_t display(uint8_t *arr, const uint8_t dp);
    esp_err_t write_digit(const uint8_t pos, const uint8_t val, const uint8_t dp);
    esp_err_t write_value(const char* fmt, const int value);

private:
    esp_err_t write_cmd(const uint8_t cmd);
    esp_err_t write_pos(const uint8_t pos, const uint8_t mask, const bool dp);

    i2c_port_t i2c_port;
    uint8_t i2c_address;
};
#endif

