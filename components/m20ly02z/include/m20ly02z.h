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
 * @file m20ly02z.h
 *
 * ESP-IDF driver for Noritake VFD Display M20LY02Z
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __M20LY02Z_H__
#define __M20LY02Z_H__

#include <stddef.h>
#include <stdbool.h>
#include <driver/gpio.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_M20LY02Z_IFACE_PARALLEL
/**
 * @brief Setup GPIO pins for parallel operation
 *
 * @param latchPin GPIO pin to use for latching data
 * @param porPin GPIO pin for Power on Reset
 * @param d0 GPIO ping for D0 data line
 * @param d1 GPIO ping for D1 data line
 * @param d2 GPIO ping for D2 data line
 * @param d3 GPIO ping for D3 data line
 * @param d4 GPIO ping for D4 data line
 * @param d5 GPIO ping for D5 data line
 * @param d6 GPIO ping for D6 data line
 * @param d7 GPIO ping for D7 data line
 * 
 * @return `ESP_OK` on success
 */
esp_err_t m20ly02z_init(uint8_t latchPin, uint8_t porPin, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);

#else 

/**
 * @brief Setup GPIO pins for serial operation
 *
 * @param latchPin GPIO pin connected to latch pin of display
 * @param porPin GPIO pin connected to power on reset pin of display
 * @param clockPin GPIO pin connected to clock pin of display
 * @param doutPin GPIO Pin connected to Data Out pin of display
 * @return `ESP_OK` on success
 */
esp_err_t m20ly02z_init(gpio_num_t latchPin, gpio_num_t oePin, gpio_num_t clockPin, gpio_num_t doutPin);

#endif

void m20ly02z_clear(void);
void m20ly02z_send_command(const uint8_t command);
void m20ly02z_send_byte(const uint8_t data);


#ifdef __cplusplus
}
#endif

#endif /* __M20LY02Z_H__ */