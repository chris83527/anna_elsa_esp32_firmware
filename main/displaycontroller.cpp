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
 * @file tableaucontroller.c
 *
 * Higher level routines for controlling HT16K33 I2C LED Matrix driver chips and ws2812 LEDs
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <driver/rmt.h>
#include <driver/gpio.h>
#include <led_strip.h>
#include <color.h>
#include <stdint.h>
#include <stdbool.h>
#include <string>
#include <esp_log.h>
#include <ht16k33.h>
#include <mcp23x17.h>
#include <m20ly02z.h>


#include "config.h"
#include "sdkconfig.h"

#include "moneycontroller.h"
#include "displaycontroller.h"
#include "maincontroller.h"

using namespace std;

static const char *TAG = "DisplayController";

led_strip_t ledStrip = {

    .type = LED_STRIP_WS2812,
    .gpio = LED_GPIO,
    .channel = RMT_TX_CHANNEL,
    .length = LED_COUNT,
    .buf = NULL,
    .brightness = 255,
};

DisplayController::lamp_data_t *lampData[LED_COUNT + 6];
uint8_t DisplayController::NUDGE_LAMPS[] = {0, LAMP_NUDGE_1, LAMP_NUDGE_2, LAMP_NUDGE_3, LAMP_NUDGE_4, LAMP_NUDGE_5};

esp_err_t DisplayController::initialise() {

    memset(&movesDisplay, 0, sizeof (ht16k33_t));
    memset(&creditDisplay, 0, sizeof (ht16k33_t));
    memset(&bankDisplay, 0, sizeof (ht16k33_t));
    memset(&buttonIO, 0, sizeof (mcp23x17_t));

    if (ht16k33_init_desc(&bankDisplay, 0, HT16K33_ADDR_BASE + 1, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "Could not initialise bank display");
    } else {
        ht16k33_display_on(&bankDisplay);
        ht16k33_write_value(&bankDisplay, "%05d", 87654);
    }

    if (ht16k33_init_desc(&creditDisplay, 0, HT16K33_ADDR_BASE, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "Could not initialise credit display");
    } else {
        ht16k33_display_on(&creditDisplay);
        ht16k33_write_value(&creditDisplay, "%05d", 11223);
    }

    if (ht16k33_init_desc(&movesDisplay, 0, HT16K33_ADDR_BASE + 2, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "Could not initialise moves display");
    } else {
        ht16k33_display_on(&movesDisplay);
        ht16k33_write_value(&movesDisplay, "%02d", 88);
    }

    if (mcp23x17_init_desc(&buttonIO, 0, MCP23X17_ADDR_BASE, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "Could not intialise button interface");
    } else {

    }

    if (m20ly02z_init(MD_STROBE, MD_OE, MD_CLK, MD_DATA) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialise VFD display");
    }

    DisplayController::clearText();

    hue = 0;
    start_rgb = 0;

    led_strip_install();
    ESP_ERROR_CHECK(led_strip_init(&ledStrip));

    DisplayController::resetLampData();

    return ESP_OK;
}

void DisplayController::resetLampData() {
    // initialise lamps
    for (int i = 0; i < LED_COUNT + 6; i++) {
        lampData->blinkspeed.blinkFast = false;
        lampData->blink = false;
        lampData->lampIndex = i;
        lampData->on = false;
        // set lamp colour to white
        lampData->rgb.b = 255;
        lampData->rgb.g = 255;
        lampData->rgb.r = 255;
    }

}

DisplayController::lamp_data_t* DisplayController::getLampData() {
    return lampData;
}

void DisplayController::updateLampData(lamp_data_t lampData, bool performUpdate) {
    // set leds
    for (int i = 0; i < LED_COUNT; i++) {
        ESP_ERROR_CHECK(led_strip_set_pixel(&ledStrip, i, lampData[i].rgb));
    }

    // set button lamps
    for (int i = LED_COUNT + 1; i < LED_COUNT + 7; i++) {
        mcp23x17_port_write(&buttonIO,)
    }
}

uint8_t DisplayController::getButtonStatus() {
    uint16_t val;
    mcp23x17_port_read(&buttonIO, &val);
    return ((val >> 8) & 0xff);
}

void DisplayController::setText(const char *text) {

}

void DisplayController::setText(string &text) {
    m20ly02z_clear();

    string::iterator it;
    for (it = text.begin(); it != text.end(); it++) {
        m20ly02z_send_byte(*it);
    }

}

void DisplayController::clearText() {
    m20ly02z_clear();
}

void DisplayController::displayText() {

}

uint8_t DisplayController::getHue(void) {
    return this->hue;
}

uint8_t DisplayController::getStartRgb(void) {
    return this->start_rgb;
}

void DisplayController::setHue(uint8_t hue) {
    this->hue = hue;
}

void DisplayController::setStartRgb(uint8_t startRgb) {
    this->start_rgb = startRgb;
}

void rainbowChaseTask(DisplayController *displayController) {
    // Show simple rainbow chasing pattern
    ESP_LOGI(TAG, "LED Rainbow Chase Start");
    rgb_t rgb_data;
    while (true) {
        for (int i = 0; i < 3; i++) {
            for (int j = i; j < LED_COUNT; j += 3) {
                // Build RGB values
                displayController.setHue(j * 360 / LED_COUNT + displayController->getStartRgb());
                displayController->ledStripHsv2rgb(displayController.getHue(), 100, 100, &rgb_data.b, &rgb_data.g, &rgb_data.r);
                // Write RGB values to strip driver
                ESP_ERROR_CHECK_WITHOUT_ABORT(led_strip_set_pixel(&ledStrip, j, rgb_data));
            }
            // Flush RGB values to LEDs
            ESP_ERROR_CHECK_WITHOUT_ABORT(led_strip_flush(&ledStrip));
            vTaskDelay(pdMS_TO_TICKS(CHASE_SPEED_MS));
        }
        displayController->setStartRgb(displayController->getStartRgb() += 60);
    }
}

/**
 * @brief Task to refresh the seven segment displays. 
 * 
 * @param pvParameter 
 */
void updateSevenSegDisplaysTask(MainController *mainController) {
    for (;;) {
        uint16_t bank = mainController->getMoneyController()->getBank();
        uint16_t credit = mainController->getMoneyController()->getCredit();
        //ht16k33_write_value(&mainController->getDisplayController().bankDisplay, "%05d", bank);
        //ht16k33_write_value(&creditDisplay, "%05d", credit);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
void DisplayController::ledStripHsv2rgb(uint8_t h, uint8_t s, uint8_t v, uint8_t* r, uint8_t* g, uint8_t* b) {
    h %= 360; // h -> [0,360]
    uint8_t rgb_max = v * 2.55f;
    uint8_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint8_t i = h / 60;
    uint8_t diff = h % 60;

    // RGB adjustment amount by hue
    uint8_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

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
