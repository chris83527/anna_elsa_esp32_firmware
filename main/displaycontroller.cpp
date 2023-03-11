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

#include <stdlib.h>
#include <string>
#include <cstddef>
#include <bitset>
#include <chrono>

#include "../config/sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/gpio.h"

#include "rgb.h"
#include "color.h"
#include "led_strip.h"

#include "config.h"

#include "esp_log.h"
#include "ht16k33.h"
#include "mcp23x17.h"
#include "m20ly02z.h"

#include "moneycontroller.h"
#include "maincontroller.h"
#include "displaycontroller.h"
#include "audiocontroller.h"
#include "game.h"

using namespace std;

static const char *TAG = "DisplayController";
static string vfdText;

const uint8_t DisplayController::NUDGE_LAMPS[] = {
    LAMP_NUDGE_1,
    LAMP_NUDGE_2,
    LAMP_NUDGE_3,
    LAMP_NUDGE_4,
    LAMP_NUDGE_5
};
const uint8_t DisplayController::TRAIL_LAMPS[] = {23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39};
const uint8_t DisplayController::FEATURE_LAMPS[] = {
    LAMP_MATRIX_FREE_SPIN_1_2,
    LAMP_MATRIX_DOUBLE_MONEY_1_3,
    LAMP_MATRIX_SHUFFLE_1_1,
    LAMP_MATRIX_LOSE_2_2,
    LAMP_MATRIX_PALACE_2_3,
    LAMP_MATRIX_PALACE_2_1,
    LAMP_MATRIX_SHUFFLE_3_2,
    LAMP_MATRIX_LOSE_3_3,
    LAMP_MATRIX_FREE_SPIN_3_1,
    LAMP_MATRIX_HI_LO_4_2,
    LAMP_MATRIX_FREE_SPIN_4_3,
    LAMP_MATRIX_PALACE_4_1
};

DisplayController::DisplayController(MainController* mainController) {
    ESP_LOGD(TAG, "Entering constructor");
    this->mainController = mainController;
    ESP_LOGD(TAG, "Leaving constructor");
}

DisplayController::DisplayController(const DisplayController& orig) {

}

esp_err_t DisplayController::initialise() {

    ESP_LOGI(TAG, "Entering DisplayController::initialise()");

    this->buttonStatus = 0;

    memset(&movesDisplay, 0, sizeof (ht16k33_t));
    memset(&creditDisplay, 0, sizeof (ht16k33_t));
    memset(&bankDisplay, 0, sizeof (ht16k33_t));
    memset(&buttonIO, 0, sizeof (mcp23x17_t));
    memset(&ledStrip, 0, sizeof (led_strip_t));    

    ledStrip.is_rgbw = true;
    ledStrip.type = LED_STRIP_WS2812;
    ledStrip.length = LED_COUNT;
    ledStrip.gpio = LED_GPIO;
    ledStrip.buf = NULL;
    ledStrip.brightness = 255;
    ledStrip.channel = RMT_TX_CHANNEL;
    
    
    led_strip_install();

    if (led_strip_init(&this->ledStrip) != ESP_OK) {
        ESP_LOGE(TAG, "WS2812 driver installation failed!");
    } else {
        ESP_LOGD(TAG, "WS2812 driver installation succeeded");
    }

    creditDisplay.cfg.master.clk_speed = I2C_FREQ_HZ;
    creditDisplay.cfg.mode = I2C_MODE_MASTER;
    creditDisplay.cfg.scl_pullup_en = false;
    creditDisplay.cfg.sda_pullup_en = false;
    if (ht16k33_init_desc(&creditDisplay, 0, CREDIT_DISPLAY_ADDRESS, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "Could not initialise credit display");
    } else {
        ht16k33_display_on(&creditDisplay);
        ht16k33_write_value(&creditDisplay, "%05d", 88888);
        ESP_LOGD(TAG, "Credit display initialisation succeeded");
    }

    bankDisplay.cfg.master.clk_speed = I2C_FREQ_HZ;
    bankDisplay.cfg.mode = I2C_MODE_MASTER;
    bankDisplay.cfg.scl_pullup_en = false;
    bankDisplay.cfg.sda_pullup_en = false;
    if (ht16k33_init_desc(&bankDisplay, 0, BANK_DISPLAY_ADDRESS, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "Could not initialise bank display");
    } else {
        ht16k33_display_on(&bankDisplay);
        ht16k33_write_value(&bankDisplay, "%05d", 88888);
        ESP_LOGD(TAG, "Bank display initialisation succeeded");
    }

    movesDisplay.cfg.master.clk_speed = I2C_FREQ_HZ;
    movesDisplay.cfg.mode = I2C_MODE_MASTER;
    movesDisplay.cfg.scl_pullup_en = false;
    movesDisplay.cfg.sda_pullup_en = false;
    if (ht16k33_init_desc(&movesDisplay, 0, MOVES_DISPLAY_ADDRESS, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "Could not initialise moves display");
    } else {
        ht16k33_display_on(&movesDisplay);
        ht16k33_write_value(&movesDisplay, "%02d", 88);
        ESP_LOGD(TAG, "Moves display initialisation succeeded");
    }

    buttonIO.cfg.master.clk_speed = I2C_FREQ_HZ;
    buttonIO.cfg.mode = I2C_MODE_MASTER;
    buttonIO.cfg.scl_pullup_en = false;
    buttonIO.cfg.sda_pullup_en = false;

    if (mcp23x17_init_desc(&buttonIO, BUTTONS_I2C_ADDRESS, 0, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "Could not initialise button interface");
    } else {

        uint16_t portMode = 0x00ff; // PortA input, portB output (0 = output, 1 = input)
        uint16_t pullup = 0x0000; // internal pullup resistors on button pins off - we pull them up in hardware.

        mcp23x17_port_set_mode(&buttonIO, portMode);
        mcp23x17_port_set_pullup(&buttonIO, pullup);

        ESP_LOGI(TAG, "Button interface initialisation succeeded");
    }


    this->resetLampData();

    // Start a new thread to update the lamps
    this->updateLampsThread.reset(new std::thread([this]() {
        updateLampsTask();
    }));

    // Start a thread to update the 7-segment displays
    this->updateSevenSegDisplaysThread.reset(new std::thread([this]() {
        updateSevenSegDisplaysTask();
    }));

    this->testLamps();

    pollButtonStatusThread.reset(new std::thread([this]() {
        pollButtonStatus();
    }));

    ESP_LOGD(TAG, "Exiting DisplayController::initialise()");
    return ESP_OK;
}

void DisplayController::beginAttractMode() {
    attractMode = true;
    this->attractModeThread.reset(new std::thread([this]() {
        attractModeTask();
    }));
}

void DisplayController::stopAttractMode() {
    attractMode = false;
}

void DisplayController::resetLampData() {
    ESP_LOGD(TAG, "Entering resetLampData()");
    // initialise lamps
    for (int i = 0; i < (LED_COUNT + 6); i++) {
        lampData[i].lampState = LampState::off;
        // set lamp colour to white
        lampData[i].rgb.b = 255;
        lampData[i].rgb.g = 255;
        lampData[i].rgb.r = 255;
    }
    ESP_LOGD(TAG, "Exiting resetLampData()");
}

bool DisplayController::isAttractMode() {
    return attractMode;
}

void DisplayController::testLamps() {
    ESP_LOGD(TAG, "Entering testLamps()");
    // initialise lamps
    // switch all LEDs on;
    resetLampData();
    for (int i = 0; i < (LED_COUNT + 6); i++) {
        lampData[i].lampState = LampState::on;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    resetLampData();
    ESP_LOGD(TAG, "Exiting testLamps()");
}

void DisplayController::setMoves(uint8_t value) {
    ESP_LOGD(TAG, "Entering setMoves(%d)", value);
    ht16k33_write_value(getMovesDisplay(), "%02d", value);
    ESP_LOGD(TAG, "Exiting setMoves");
}

LampData* DisplayController::getLampData() {
    return lampData;
}

uint8_t DisplayController::getButtonStatus() {
    ESP_LOGD(TAG, "Exiting getButtonStatus() with value %u", this->buttonStatus);
    return this->buttonStatus;
}

void DisplayController::pollButtonStatus() {
    ESP_LOGD(TAG, "Entering pollButtonStatus()");

    uint16_t val;
    esp_err_t err;

    while (1) {
        err = mcp23x17_port_read(&buttonIO, &val);

        if (err == ESP_OK) {
            this->buttonStatus = (uint8_t) ~(val & 0xff); // Invert because we are pulling low in hardware.
        } else {
            ESP_LOGE(TAG, "An error occurred getting button status");
            this->buttonStatus = 0; // Failsafe
        }

        if ((this->buttonStatus & (1 << BTN_DOOR)) == 0) {
            if (!this->doorOpen) {
                ESP_LOGI(TAG, "Door open!");
            }
            this->doorOpen = true;
        } else {
            if (this->doorOpen) {
                ESP_LOGI(TAG, "Door closed!");
            }
            this->doorOpen = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

}

void DisplayController::displayText(const string& text) {

    // Only update if we need to 
    if (vfdText.compare(text) != 0) {
        m20ly02z_clear();

        for (char str_char : text) {
            m20ly02z_send_byte(str_char);
        }

        vfdText = text;
    }

}

void DisplayController::clearText() {
    m20ly02z_clear();
}

led_strip_t* DisplayController::getLedStrip() {
    return &this->ledStrip;
}

mcp23x17_t* DisplayController::getButtonIO() {
    return &this->buttonIO;
}

ht16k33_t* DisplayController::getBankDisplay() {
    return &this->bankDisplay;
}

ht16k33_t* DisplayController::getCreditDisplay() {
    return &this->creditDisplay;
}

ht16k33_t* DisplayController::getMovesDisplay() {
    return &this->movesDisplay;
}

void DisplayController::attractModeTask() {
    // Show simple rainbow chasing pattern
    ESP_LOGI(TAG, "Animation task started");

    LampData *lampData = this->getLampData();

    this->resetLampData();

    rgb_t rgb_data;
    hsv_t hsv_data;
    uint8_t start_rgb = 0;
    int state = 0;

    // switch all LEDs on;
    for (int i = 0; i < LED_COUNT; i++) {
        lampData[i].lampState = LampState::on;
    }


    while (this->attractMode) {

        for (int i = 0; i < 3; i++) {
            for (int j = i; j < LED_COUNT; j += 3) {

                // Build RGB values
                hsv_data.hue = j * 360 / LED_COUNT + start_rgb;
                hsv_data.sat = 255;
                hsv_data.val = 255;
                rgb_data = hsv2rgb_rainbow(hsv_data);

                // Write RGB values to strip driver
                lampData[j].rgb = rgb_data;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(CHASE_SPEED_MS));
        }
        start_rgb += 60;

        switch (state) {
            case 0:
                this->displayText("       FROZEN       ");
                break;
            case 1:
                this->displayText("      PLAY ME       ");
                break;
            case 2:
                this->displayText("     20CT GAME      ");
                break;
            case 3:
                this->displayText("    INSERT COINS    ");
                break;
        }

        // reset state
        if (state >= 3) {
            state = 0;
        } else {
            if ((start_rgb % 240) == 0) { // only advance every 4 calls
                state++;
            }
        }

    }

    for (int i = 0; i < LED_COUNT; i++) {
        lampData[i].lampState = LampState::off;
    }

}

void DisplayController::updateLampsTask() {
    ESP_LOGI(TAG, "Update Lamps task started");

    uint16_t btnLamps = 0;

    LampData *tmpLampData = this->getLampData();
    led_strip_t *ledStrip = this->getLedStrip();

    for (;;) {

        rgb_t rgb;

        // set leds
        btnLamps = 0;
        for (int i = 0; i < LED_COUNT + 6; i++) {
            if (tmpLampData[i].lampState == LampState::on || tmpLampData[i].lampState == LampState::blinkfast || tmpLampData[i].lampState == LampState::blinkslow) {
                //ESP_LOGD(TAG, "Switching on pixel %d with r: %d, g: %d, b: %d", i, tmpLampData[i].rgb.r, tmpLampData[i].rgb.g, tmpLampData[i].rgb.b);       
                if (i < LED_COUNT) {
                    led_strip_set_pixel(ledStrip, i, tmpLampData[i].rgb);
                } else {
                    // GPB1 and GPB0 are unconnected
                    switch (i) {
                        case LED_COUNT:
                            btnLamps |= (1 << 7); //GPB7 (Start)
                            break;
                        case LED_COUNT + 1:
                            btnLamps |= (1 << 6); //GPB6 (Collect)
                            break;
                        case LED_COUNT + 2:
                            btnLamps |= (1 << 5); // GPB5
                            break;
                        case LED_COUNT + 3:
                            btnLamps |= (1 << 4); // GPB4
                            break;
                        case LED_COUNT + 4:
                            btnLamps |= (1 << 3); // GPB3
                            break;
                        case LED_COUNT + 5:
                            btnLamps |= (1 << 2); // GPB2
                            break;
                    }
                }
            } else {
                if (i < LED_COUNT) {
                    //ESP_LOGD(TAG, "Switching off pixel %d", i);
                    rgb.r = 0;
                    rgb.g = 0;
                    rgb.b = 0;
                    led_strip_set_pixel(ledStrip, i, rgb);
                }
            }
        }
        led_strip_flush(ledStrip);
        mcp23x17_port_write(this->getButtonIO(), btnLamps);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // set leds
        btnLamps = 0;
        for (int i = 0; i < LED_COUNT + 6; i++) {
            if (tmpLampData[i].lampState == LampState::on || tmpLampData[i].lampState == LampState::blinkslow) {
                ESP_LOGD(TAG, "Switching on pixel %d with r: %d, g: %d, b: %d", i, tmpLampData[i].rgb.r, tmpLampData[i].rgb.g, tmpLampData[i].rgb.b);
                if (i < LED_COUNT) {
                    led_strip_set_pixel(ledStrip, i, tmpLampData[i].rgb);
                } else {
                    // GPB1 and GPB0 are unconnected
                    switch (i) {
                        case LED_COUNT:
                            btnLamps |= (1 << 7); //GPB7 (Start)
                            break;
                        case LED_COUNT + 1:
                            btnLamps |= (1 << 6); //GPB6 (Collect)
                            break;
                        case LED_COUNT + 2:
                            btnLamps |= (1 << 5); // GPB5
                            break;
                        case LED_COUNT + 3:
                            btnLamps |= (1 << 4); // GPB4
                            break;
                        case LED_COUNT + 4:
                            btnLamps |= (1 << 3); // GPB3
                            break;
                        case LED_COUNT + 5:
                            btnLamps |= (1 << 2); // GPB2
                            break;
                    }
                }
            } else {
                if (i < LED_COUNT) {
                    ESP_LOGD(TAG, "Switching off pixel %d", i);
                    rgb.r = 0;
                    rgb.g = 0;
                    rgb.b = 0;
                    led_strip_set_pixel(ledStrip, i, rgb);
                }
            }
        }

        led_strip_flush(ledStrip);
        mcp23x17_port_write(this->getButtonIO(), btnLamps);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        btnLamps = 0;
        for (int i = 0; i < LED_COUNT + 6; i++) {
            if (tmpLampData[i].lampState == LampState::on) {
                ESP_LOGD(TAG, "Switching on pixel %d with r: %d, g: %d, b: %d", i, tmpLampData[i].rgb.r, tmpLampData[i].rgb.g, tmpLampData[i].rgb.b);
                if (i < LED_COUNT) {
                    led_strip_set_pixel(ledStrip, i, tmpLampData[i].rgb);
                } else {
                    // GPB1 and GPB0 are unconnected
                    switch (i) {
                        case LED_COUNT:
                            btnLamps |= (1 << 7); //GPB7 (Start)
                            break;
                        case LED_COUNT + 1:
                            btnLamps |= (1 << 6); //GPB6 (Collect)
                            break;
                        case LED_COUNT + 2:
                            btnLamps |= (1 << 5); // GPB5
                            break;
                        case LED_COUNT + 3:
                            btnLamps |= (1 << 4); // GPB4
                            break;
                        case LED_COUNT + 4:
                            btnLamps |= (1 << 3); // GPB3
                            break;
                        case LED_COUNT + 5:
                            btnLamps |= (1 << 2); // GPB2
                            break;
                    }
                }
            } else {
                if (i < LED_COUNT) {
                    //ESP_LOGD(TAG, "Switching off pixel %d", i);
                    rgb.r = 0;
                    rgb.g = 0;
                    rgb.b = 0;
                    led_strip_set_pixel(ledStrip, i, rgb);
                }
            }
        }

        led_strip_flush(ledStrip);
        mcp23x17_port_write(this->getButtonIO(), btnLamps);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        btnLamps = 0;
        for (int i = 0; i < LED_COUNT + 6; i++) {
            if (tmpLampData[i].lampState == LampState::on || tmpLampData[i].lampState == LampState::blinkfast) {
                ESP_LOGD(TAG, "Switching on pixel %d with r: %d, g: %d, b: %d", i, tmpLampData[i].rgb.r, tmpLampData[i].rgb.g, tmpLampData[i].rgb.b);
                if (i < LED_COUNT) {
                    led_strip_set_pixel(ledStrip, i, tmpLampData[i].rgb);
                } else {
                    // GPB1 and GPB0 are unconnected
                    switch (i) {
                        case LED_COUNT:
                            btnLamps |= (1 << 7); //GPB7 (Start)
                            break;
                        case LED_COUNT + 1:
                            btnLamps |= (1 << 6); //GPB6 (Collect)
                            break;
                        case LED_COUNT + 2:
                            btnLamps |= (1 << 5); // GPB5
                            break;
                        case LED_COUNT + 3:
                            btnLamps |= (1 << 4); // GPB4
                            break;
                        case LED_COUNT + 4:
                            btnLamps |= (1 << 3); // GPB3
                            break;
                        case LED_COUNT + 5:
                            btnLamps |= (1 << 2); // GPB2
                            break;
                    }
                }
            } else {
                if (i < LED_COUNT) {
                    //ESP_LOGD(TAG, "Switching off pixel %d", i);
                    rgb.r = 0;
                    rgb.g = 0;
                    rgb.b = 0;
                    led_strip_set_pixel(ledStrip, i, rgb);
                }
            }
        }

        led_strip_flush(ledStrip);
        mcp23x17_port_write(this->getButtonIO(), (btnLamps & 0xff00));

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

/**
 * @brief Task to refresh the seven segment displays. 
 * 
 * @param pvParameter 
 */
void DisplayController::updateSevenSegDisplaysTask() {
    ESP_LOGI(TAG, "Update 7-segment display task started");

    // 
    uint16_t bank = 0;
    uint16_t credit = 0;
    bool initialRun = true;

    for (;;) {

        if (initialRun || (bank != mainController->getMoneyController()->getBank())) {
            bank = mainController->getMoneyController()->getBank();
            ht16k33_write_value(mainController->getDisplayController()->getBankDisplay(), "%05d", bank);
        }

        if (initialRun || (credit != mainController->getMoneyController()->getCredit())) {
            credit = mainController->getMoneyController()->getCredit();
            ht16k33_write_value(mainController->getDisplayController()->getCreditDisplay(), "%05d", credit);
        }

        initialRun = false;

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

}

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
void DisplayController::ledStripHsv2rgb(uint8_t h, uint8_t s, uint8_t v, uint8_t* r, uint8_t* g, uint8_t * b) {
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
