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
 * @file m20ly02z.c
 *
 * ESP-IDF driver for Noritake VFD Display M20LY02Z
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <stdio.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp32/rom/ets_sys.h>

#include "m20ly02z.h"

static const char *TAG = "m20ly02z";

static gpio_port_t _latchPin;
static gpio_port_t _oePin;
static gpio_port_t _clockPin;
static gpio_port_t _doutPin;

#ifdef CONFIG_M20LY02Z_IFACE_PARALLEL

esp_err_t init(uint8_t latchPin, uint8_t oePin, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{

    return ESP_OK;
}

#else

esp_err_t m20ly02z_init(gpio_num_t latchPin, gpio_num_t oePin, gpio_num_t clockPin, gpio_num_t doutPin)
{
    ESP_LOGI(TAG, "Initialising VFD Display");
    // if (!latchPin || !oePin || !clockPin || !doutPin)
    // {
    //     ESP_LOGE(TAG, "One of the pins was not assigned to a GPIO pin");
    //     return ESP_FAIL;
    // }

    _latchPin = latchPin;
    _oePin = oePin;
    _clockPin = clockPin;
    _doutPin = doutPin;

    gpio_pad_select_gpio(_latchPin);    
    gpio_pad_select_gpio(_oePin);    
    gpio_pad_select_gpio(_clockPin);    
    gpio_pad_select_gpio(_doutPin);    
    
    
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(_latchPin, GPIO_MODE_OUTPUT);
    gpio_set_direction(_oePin, GPIO_MODE_OUTPUT);
    gpio_set_direction(_clockPin, GPIO_MODE_OUTPUT);
    gpio_set_direction(_doutPin, GPIO_MODE_OUTPUT);

    // pull OE down for 200msec
    gpio_set_level(_oePin, 0);
    ets_delay_us(2000);
    gpio_set_level(_oePin, 1);
    
    // initialise brightness, duty cycle etc.
    m20ly02z_send_command(0x07); // high brightness
    m20ly02z_send_command(0x7f); // duty cycle
    m20ly02z_send_command(0x94); // 20 character modules
    m20ly02z_send_command(0x0e); // initialisation complete
    m20ly02z_send_command(0xc0); // move cursor to pos 1
    
    return ESP_OK;
}

void m20ly02z_clear() 
{
    m20ly02z_send_command(0xc0);  // move cursor to pos 1
    for (int i = 0 ; i < 20 ; i++)
    {
        m20ly02z_send_byte(' ');
    }
}

void m20ly02z_send_byte(const uint8_t data) 
{
    gpio_set_level(_latchPin, 0);
    for (int i = 0 ; i < 8 ; i++)
    {
        //gpio_set_level(_doutPin, !!(data & (1 << (7 - i))));
        gpio_set_level(_doutPin, (data & (1 << (7 - i))));
        gpio_set_level(_clockPin, 1);
        ets_delay_us(10);
        gpio_set_level(_clockPin, 0);
        ets_delay_us(10);
    }
    gpio_set_level(_latchPin, 1);
    ets_delay_us(50);
    gpio_set_level(_latchPin, 0);
}

void m20ly02z_send_command(const uint8_t command)
{
    m20ly02z_send_byte(0x01);
    m20ly02z_send_byte(command);
}
#endif