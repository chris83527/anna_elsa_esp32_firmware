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
 * @file main.c
 *
 * ESP-IDF driver for Holtek HT16K33 I2C LED Matrix driver chip
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <driver/uart.h>
#include <mdns.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_spiffs.h>
#include <esp_http_server.h>
#include <esp_spiffs.h>
#include <ds3231.h>
#include <cctalk.h>
#include <ht16k33.h>
#include <m20ly02z.h>
#include <mcp23x17.h>


#include "wave.h"
#include "spiffs.h"
#include "tableau.h"
#include "webserver.h"
#include "cctalk.h"

#include "sdkconfig.h"
#include "config.h"

#define CONFIG_FIRMWARE_SERVICE

static const char *TAG = "main";

uint_fast64_t bank;
uint_fast64_t credit;

void check_coin_validator(void *pvParameter) {
    while (1) {
        cctalk_poll();
        vTaskDelay(pdMS_TO_TICKS(CCTALK_POLL_FREQ));    
    }
}

void blink_task(void *pvParameter)
{
    while (1)
    {
        /* Blink off (output low) */
        gpio_set_level(CPU_LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        /* Blink on (output high) */
        gpio_set_level(CPU_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main()
{    

    ESP_ERROR_CHECK(i2cdev_init());

    if (init_tableau() != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to initialise tableau subsystem");
    } else {
        xTaskCreate(&rainbow_chase_task, "rainbow_chase_task", 2048, NULL, 5, NULL);
    }

    if (cctalk_init_desc(UART_NUM_2, CCTALK_GPIO_TX, CCTALK_GPIO_RX) != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to initialise ccTalk subsystem");
    } else {
        xTaskCreate(&check_coin_validator, "check_coin_validator", 2048, NULL, 5, NULL);
    }
        
    init_spiffs();
    
    init_webserver("/spiffs");

    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(CPU_LED_GPIO);    
    
    
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(CPU_LED_GPIO, GPIO_MODE_OUTPUT);
 
    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
    
     if (m20ly02z_init(MD_STROBE, MD_OE, MD_CLK, MD_DATA) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialise VFD display");        
    }
    

   play_audio("/spiffs/kerching.wav");
}