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
 * @file tableau.c
 *
 * Higher level routines for controlling HT16K33 I2C LED Matrix driver chips and ws2812 LEDs
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_err.h>
#include <driver/rmt.h>
#include <driver/gpio.h>
#include <led_strip.h>
#include <stdint.h>
#include <stdbool.h>
#include <esp_log.h>
#include <ht16k33.h>
#include "config.h"

#include "tableau.h"

static const char *TAG = "tableau";

esp_err_t init_tableau() {

    movesDisplay = (ht16k33_t*) malloc(sizeof(ht16k33_t));
    creditDisplay = (ht16k33_t*) malloc(sizeof(ht16k33_t));
    bankDisplay = (ht16k33_t*) malloc(sizeof(ht16k33_t));
    
    if (ht16k33_init_desc(bankDisplay, 0, HT16K33_ADDR_BASE, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not initialise moves display");        
    } else {
        ht16k33_display_on(bankDisplay);
        ht16k33_write_digit(bankDisplay, 0, 8, 0);
        ht16k33_write_digit(bankDisplay, 1, 7, 0);
        ht16k33_write_digit(bankDisplay, 2, 6, 0);
        ht16k33_write_digit(bankDisplay, 3, 5, 0);
        ht16k33_write_digit(bankDisplay, 4, 4, 0);        
    }   

    if (ht16k33_init_desc(creditDisplay, 0, HT16K33_ADDR_BASE + 1, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not initialise moves display");        
    } else {
        ht16k33_display_on(creditDisplay);
        ht16k33_write_digit(creditDisplay, 0, 8, 1);
        ht16k33_write_digit(creditDisplay, 1, 8, 2);
        ht16k33_write_digit(creditDisplay, 2, 8, 3);
        ht16k33_write_digit(creditDisplay, 3, 8, 4);
        ht16k33_write_digit(creditDisplay, 4, 8, 5);        
    }   

    if (ht16k33_init_desc(movesDisplay, 0, HT16K33_ADDR_BASE + 2, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not initialise moves display");        
    } else {
        ht16k33_display_on(movesDisplay);
        ht16k33_write_digit(movesDisplay, 0, 8, 1);
        ht16k33_write_digit(movesDisplay, 1, 8, 2);      
    }   
    
    red = 0;
    green = 0;
    blue = 0;
    hue = 0;
    start_rgb = 0;

    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(LED_GPIO, RMT_TX_CHANNEL);
    // set counter clock to 40MHz
    config.clk_div = 2;

    esp_err_t ret = rmt_config(&config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Unable to configure RMT. Required for LED Tableau");
        return ret;
    }    
    ret = rmt_driver_install(config.channel, 0, 0);
    if (ret != ESP_OK) 
    {
        ESP_LOGE(TAG, "Unable to install RMT driver. Required for LED Tableau");
        return ret;
    }

     // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(LED_COUNT, (led_strip_dev_t)config.channel);
    led_strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!led_strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
        return ESP_FAIL;
    }

    // Clear LED strip (turn off all LEDs)
    led_strip->clear(led_strip, 100);

    return ESP_OK;   
}

void rainbow_chase_task(void *pvParameter) {
    // Show simple rainbow chasing pattern
    ESP_LOGI(TAG, "LED Rainbow Chase Start");
    while (true) {
        for (int i = 0; i < 3; i++) {
            for (int j = i; j < LED_COUNT; j += 3) {
                // Build RGB values
                hue = j * 360 / LED_COUNT + start_rgb;
                led_strip_hsv2rgb(hue, 100, 100, &red, &green, &blue);
                // Write RGB values to strip driver
                ESP_ERROR_CHECK(led_strip->set_pixel(led_strip, j, red, green, blue));
            }
            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(led_strip->refresh(led_strip, 100));
            vTaskDelay(pdMS_TO_TICKS(CHASE_SPEED_MS));
     //       led_strip->clear(led_strip, 50);
     //       vTaskDelay(pdMS_TO_TICKS(CHASE_SPEED_MS));
        }
        start_rgb += 60;
    }
}

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

