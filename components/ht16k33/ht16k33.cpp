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
 * @file ht16k33.c
 *
 * ESP-IDF driver for Holtek HT16K33 I2C LED Matrix driver chip
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_log.h>
#include <cstring>
#include <cmath>
#include "i2c_manager.h"

#include "ht16k33.h"

#define HT16K33_ON              0x21  // Commands
#define HT16K33_STANDBY         0x20
#define HT16K33_DISPLAYON       0x81
#define HT16K33_DISPLAYOFF      0x80
#define HT16K33_BLINKON         0x85 // Blink is off (00), 2 Hz (01), 1 Hz (10), or 0.5 Hz (11) for bits (21) 
#define HT16K33_BLINKOFF        0x81
#define HT16K33_DIM             0xE0 | 0x08 // Set dim from 0x00 (1/16th duty ccycle) to 0x0F (16/16 duty cycle)

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define BV(x) (1 << (x))

static const char *TAG = "ht16k33";

#define SEG_A (1<<0)
#define SEG_B (1<<1)    
#define SEG_C (1<<2)
#define SEG_D (1<<3)
#define SEG_E (1<<4)
#define SEG_F (1<<5)    
#define SEG_G (1<<6)
#define SEG_DP (1<<7)

// Arrangement for display
// )
//               a = A6
//             _________
//            |         |
//   f = A1   |  g = A0 | b = A5
//            |_________|
//            |         |
//   e = A2   |         | c = A4
//            |_________|
//               d = A3        
static const uint8_t charmap[] = {

    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,
    SEG_B | SEG_C, // 1 = 1, etc
    SEG_A | SEG_B | SEG_G | SEG_E | SEG_D, // 2        
    SEG_A | SEG_B | SEG_C | SEG_G | SEG_D, // 3
    SEG_F | SEG_G | SEG_B | SEG_C, // 4
    SEG_A | SEG_F | SEG_G | SEG_C | SEG_D, // 5
    SEG_A | SEG_F | SEG_E | SEG_D | SEG_C | SEG_G, // 6
    SEG_A | SEG_B | SEG_C, // 7
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G, // 8
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G, // 9
    SEG_DP, // decimal point
    0b00000000, // blank
    SEG_G, // minus sign
    SEG_A | SEG_B | SEG_C | SEG_E | SEG_F, // A = 13
    SEG_F | SEG_E | SEG_D | SEG_C | SEG_G, // B = 14
    SEG_A | SEG_F | SEG_E | SEG_D, // C = 15
    SEG_B | SEG_C | SEG_D | SEG_E | SEG_G, // D = 16
    SEG_A | SEG_F | SEG_E | SEG_G | SEG_D, // E = 17
    SEG_A | SEG_G | SEG_F | SEG_E, // F = 18
    SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G, // G = 19
    SEG_F | SEG_E | SEG_G | SEG_B | SEG_C, // H = 20
    SEG_B | SEG_C, // I = 21
    0b00001110, // J = 22
    0b00000000, // No K!
    SEG_F | SEG_E | SEG_D, // L = 24
    0b00000000, // No M!
    SEG_E | SEG_F | SEG_A | SEG_B | SEG_C, // N = 26
    SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F, // O = 27
    SEG_E | SEG_F | SEG_A | SEG_B | SEG_G, // P = 28
    0b01100111, // Q = 29
    SEG_E | SEG_F | SEG_A, // r = 30
    SEG_A | SEG_F | SEG_G | SEG_C | SEG_D // S = 31
};

HT16K33::HT16K33(const i2c_port_t port = I2C_NUM_0, const uint8_t address = HT16K33_ADDR_BASE) {
    ESP_LOGD(TAG, "i2c_port: %d, i2c_address: %d", port, address);
    this->i2c_port = port;
    this->i2c_address = address;
}

HT16K33::~HT16K33() {
    
}

esp_err_t HT16K33::display_on() {
    
    HT16K33::write_cmd(HT16K33_ON);
    HT16K33::write_cmd(HT16K33_DISPLAYON);
    HT16K33::write_cmd(HT16K33_DIM);

    return ESP_OK;
}

esp_err_t HT16K33::display(uint8_t *arr, const uint8_t dp) {
    
    HT16K33::write_pos(0, charmap[arr[0]], dp == 0);
    HT16K33::write_pos(1, charmap[arr[1]], dp == 1);
    HT16K33::write_pos(2, charmap[arr[2]], dp == 2);
    HT16K33::write_pos(3, charmap[arr[3]], dp == 3);
    HT16K33::write_pos(4, charmap[arr[4]], dp == 4);

    return ESP_OK;
}

esp_err_t HT16K33::write_digit(const uint8_t pos, const uint8_t val, const uint8_t dp) {
   
    HT16K33::write_pos(pos, charmap[val], dp == pos);

    return ESP_OK;
}

esp_err_t HT16K33::write_value( const char* fmt, const int value) {
    char buf[6];

    sprintf(buf, fmt, value);

    for (int pos = 0; pos < 5; pos++) {
        HT16K33::write_digit((uint8_t) pos, (uint8_t) (buf[pos] - 48), (uint8_t) 2);
    }

    return ESP_OK;
}

esp_err_t HT16K33::write_cmd(const uint8_t cmd) {
    esp_err_t ret = i2c_manager_write(this->i2c_port, this->i2c_address, I2C_NO_REG, &cmd, 1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred in HT16K33::write_cmd writing i2c data");
    }

    return ret;
}

esp_err_t HT16K33::write_pos(const uint8_t pos, const uint8_t mask, const bool dp) {

    uint8_t new_mask = mask;
    if (dp) {
        new_mask |= SEG_DP; // dp
    }

    esp_err_t ret = i2c_manager_write(this->i2c_port, this->i2c_address, pos * 2, &new_mask, 1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred in HT16K33::write_pos writing i2c data");
    }

    return ret;
}
