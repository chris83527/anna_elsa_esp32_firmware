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
#include "esp_pthread.h"
#include "esp_debug_helpers.h"
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

std::array<int, 0> const DisplayController::SEGMENTS = {

};

std::array<int, 5> const DisplayController::NUDGE_LAMPS = {
    LAMP_NUDGE_1,
    LAMP_NUDGE_2,
    LAMP_NUDGE_3,
    LAMP_NUDGE_4,
    LAMP_NUDGE_5
};

std::array<int, 17> const DisplayController::TRAIL_LAMPS = {
    LAMP_TRAIL_20_CENT,
    LAMP_TRAIL_40_CENT,
    LAMP_TRAIL_60_CENT,
    LAMP_TRAIL_80_CENT,
    LAMP_TRAIL_ONE_EURO,
    LAMP_TRAIL_ONE_TWENTY,
    LAMP_TRAIL_ONE_FOURTY,
    LAMP_TRAIL_ONE_SIXTY,
    LAMP_TRAIL_ONE_EIGHTY,
    LAMP_TRAIL_TWO_EURO,
    LAMP_TRAIL_TWO_FOURTY,
    LAMP_TRAIL_TWO_EIGHTY,
    LAMP_TRAIL_THREE_FOURTY,
    LAMP_TRAIL_THREE_EIGHTY,
    LAMP_TRAIL_FOUR_TWENTY,
    LAMP_TRAIL_FOUR_SIXTY,
    LAMP_TRAIL_FIVE_EURO,
};

std::array<int, 12> const DisplayController::FEATURE_LAMPS = {
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
    if (ht16k33_init_desc(this->getBankDisplay(), 0, BANK_DISPLAY_ADDRESS, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "Could not initialise bank display");
    } else {
        ht16k33_display_on(this->getBankDisplay());
        ht16k33_write_value(this->getBankDisplay(), "%05d", 88888);
        ESP_LOGD(TAG, "Bank display initialisation succeeded");
    }

    movesDisplay.cfg.master.clk_speed = I2C_FREQ_HZ;
    movesDisplay.cfg.mode = I2C_MODE_MASTER;
    movesDisplay.cfg.scl_pullup_en = false;
    movesDisplay.cfg.sda_pullup_en = false;
    if (ht16k33_init_desc(this->getMovesDisplay(), 0, MOVES_DISPLAY_ADDRESS, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "Could not initialise moves display");
    } else {
        ht16k33_display_on(this->getMovesDisplay());
        ht16k33_write_value(this->getMovesDisplay(), "%02d", 88);
        ESP_LOGD(TAG, "Moves display initialisation succeeded");
    }

    if (mcp23x17_init_desc(this->getButtonIO(), BUTTONS_I2C_ADDRESS, 0, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "Could not initialise button interface");
    } else {

        uint16_t portMode = 0x00ff; // PortA input, portB output (0 = output, 1 = input)
        uint16_t pullup = 0x0000; // internal pullup resistors on button pins off - we pull them up in hardware.

        mcp23x17_port_set_mode(this->getButtonIO(), portMode);
        mcp23x17_port_set_pullup(this->getButtonIO(), pullup);

        ESP_LOGI(TAG, "Button interface initialisation succeeded");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    for (int i = 0 ; i < LED_COUNT + 6 ; i++) {
        lampData[i] = LampData();
    }

    this->resetLampData();

    // Start a new thread to update the lamps
    auto cfg = esp_pthread_get_default_config();
    cfg.thread_name = "BlinkLamps";
    cfg.prio = 3;
    cfg.stack_size = 1024;
    esp_pthread_set_cfg(&cfg);
    this->blinkLampsThread = std::thread([this]() {
        blinkLampsTask();
    });

    cfg = esp_pthread_get_default_config();
    cfg.thread_name = "UpdateLamps";
    cfg.prio = 4;
    cfg.stack_size = 1024;
    esp_pthread_set_cfg(&cfg);
    this->updateLampsThread = std::thread([this]() {
        updateLampsTask();
    });

    cfg = esp_pthread_get_default_config();
    cfg.thread_name = "UpdateSevenSeg";
    cfg.prio = 2;
    cfg.stack_size = 1024;
    esp_pthread_set_cfg(&cfg);
    // Start a thread to update the 7-segment displays
    this->updateSevenSegDisplaysThread = std::thread([this]() {
        updateSevenSegDisplaysTask();
    });

    this->testLamps();

    this->attractMode = false;
    cfg = esp_pthread_get_default_config();
    cfg.thread_name = "AttractMode";
    cfg.prio = 4;
    cfg.stack_size = 4096;
    esp_pthread_set_cfg(&cfg);
    this->attractModeThread = std::thread([this]() {
        attractModeTask();
    });

    ESP_LOGD(TAG, "Exiting DisplayController::initialise()");
    return ESP_OK;
}

void DisplayController::beginAttractMode() {
    this->attractMode = true;

}

void DisplayController::stopAttractMode() {
    this->attractMode = false;
}

void DisplayController::resetLampData() {

    ESP_LOGD(TAG, "Entering resetLampData()");
    // initialise lamps
    for (int i = 0; i < (LED_COUNT + 6); i++) {
        lampData[i].lampState = LampState::off;
        // set lamp colour to white
        lampData[i].rgb.r = 255;
        lampData[i].rgb.g = 255;
        lampData[i].rgb.b = 255;
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
    return &this->lampData[0];
}

uint8_t DisplayController::getButtonStatus() {
    //ESP_LOGD(TAG, "Exiting getButtonStatus() with value %u", this->buttonStatus);
    return this->buttonStatus;
}

uint8_t DisplayController::waitForButton(uint8_t mask) {
    uint8_t btnStatus = mainController->getDisplayController()->getButtonStatus();
    // loop waiting for button press.
    while ((btnStatus & mask) == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(75));
        btnStatus = mainController->getDisplayController()->getButtonStatus();
    }

    return btnStatus;
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

    ESP_LOGI(TAG, "Attract mode thread started");

    while (1) {
        if (this->isAttractMode()) {

            if (this->isAttractMode()) this->displayText("       FROZEN       ");
            if (this->isAttractMode()) this->resetLampData();

            if (this->isAttractMode()) this->rainbowEffect();

            if (this->isAttractMode()) std::this_thread::sleep_for(std::chrono::seconds(10));

            if (this->isAttractMode()) resetLampData();

            if (this->isAttractMode()) this->displayText("      PLAY ME       ");
            if (this->isAttractMode()) this->chaseEffect();

            if (this->isAttractMode()) std::this_thread::sleep_for(std::chrono::seconds(5));

            if (this->isAttractMode()) resetLampData();

            if (this->isAttractMode()) this->displayText("     20CT GAME      ");

            if (this->isAttractMode()) std::this_thread::sleep_for(std::chrono::seconds(5));

            if (this->isAttractMode()) this->displayText("    INSERT COINS    ");

            if (this->isAttractMode()) std::this_thread::sleep_for(std::chrono::seconds(5));

        } else {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

void DisplayController::chaseEffect() {
    // Red trail effect           

    //int trailElements = (sizeof (TRAIL_LAMPS) / sizeof (TRAIL_LAMPS[0]));
    int trailElements = 17;
    for (int j = trailElements; j > 0; j--) {

        resetLampData();

        for (int i = 0; i < (j + 3); i++) {

            if (!this->attractMode) {
                return;
            }

            ESP_LOGI(TAG, "j = %d, i = %d, lamp = %d, trailElements = %d", j, i, i < j ? TRAIL_LAMPS.at(i) : TRAIL_LAMPS.at(16), trailElements);

            if (i < j) {
                lampData.at(TRAIL_LAMPS.at(i)).rgb = rgb_from_code(0x00ff0000);
                lampData.at(TRAIL_LAMPS.at(i)).lampState = LampState::on;

                if (i > 0) {
                    lampData.at(TRAIL_LAMPS.at(i - 1)).rgb = rgb_from_code(0x00c00000);
                    lampData.at(TRAIL_LAMPS.at(i - 1)).lampState = LampState::on;
                }
                if (i > 1) {
                    lampData.at(TRAIL_LAMPS.at(i - 2)).rgb = rgb_from_code(0x00810000);
                    lampData.at(TRAIL_LAMPS.at(i - 2)).lampState = LampState::on;
                }
                if (i > 2) {
                    lampData.at(TRAIL_LAMPS.at(i - 3)).rgb = rgb_from_code(0x00420000);
                    lampData.at(TRAIL_LAMPS.at(i - 3)).lampState = LampState::on;
                }
            }

            // tail catches up                    
            if (j > 4) {
                lampData.at(TRAIL_LAMPS.at(i - 4)).lampState = LampState::off;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(CHASE_SPEED_MS));
        }
    }
}

void DisplayController::rainbowEffect() {
    hsv_t hsv_data;
    uint8_t start_rgb = 0;

    // Rainbow effect (10 repeats)
    for (int k = 0; k < 10; k++) {
        for (int i = 0; i < 3; i++) {
            for (int j = i; j < LED_COUNT; j += 3) {

                if (!this->attractMode) {
                    return;
                }

                // Build RGB values
                hsv_data.hue = j * 360 / LED_COUNT + start_rgb;
                hsv_data.sat = 255;
                hsv_data.val = 255;

                // Write RGB values to strip driver
                lampData[j].rgb = hsv2rgb_rainbow(hsv_data);
                lampData[j].lampState = LampState::on;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(CHASE_SPEED_MS));
        }
        start_rgb += 60;
    }
}

void DisplayController::blinkLampsTask() {

    for (;;) {

        for (int i = 0; i < LED_COUNT + 6; i++) {

            if (this->lampData[i].lampState == LampState::on || this->lampData[i].lampState == LampState::blinkfast || this->lampData[i].lampState == LampState::blinkslow) {
                this->lampData[i].activeRgb = this->lampData[i].rgb;
            } else {
                // Set active rgb value to 0 (off or black)
                this->lampData[i].activeRgb.blue = 0;
                this->lampData[i].activeRgb.green = 0;
                this->lampData[i].activeRgb.red = 0;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        for (int i = 0; i < LED_COUNT + 6; i++) {
            if (this->lampData[i].lampState == LampState::on || this->lampData[i].lampState == LampState::blinkslow) {
                this->lampData[i].activeRgb = this->lampData[i].rgb;
            } else {
                // Set active rgb value to 0 (off or black)
                this->lampData[i].activeRgb.blue = 0;
                this->lampData[i].activeRgb.green = 0;
                this->lampData[i].activeRgb.red = 0;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        for (int i = 0; i < LED_COUNT + 6; i++) {
            if (this->lampData[i].lampState == LampState::on) {
                this->lampData[i].activeRgb = this->lampData[i].rgb;
            } else {
                // Set active rgb value to 0 (off or black)
                this->lampData[i].activeRgb.blue = 0;
                this->lampData[i].activeRgb.green = 0;
                this->lampData[i].activeRgb.red = 0;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        for (int i = 0; i < LED_COUNT + 6; i++) {
            if (this->lampData[i].lampState == LampState::on || this->lampData[i].lampState == LampState::blinkfast) {
                this->lampData[i].activeRgb = this->lampData[i].rgb;
            } else {
                // Set active rgb value to 0 (off or black)
                this->lampData[i].activeRgb.blue = 0;
                this->lampData[i].activeRgb.green = 0;
                this->lampData[i].activeRgb.red = 0;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }
}

void DisplayController::updateLampsTask() {
    ESP_LOGI(TAG, "Update Lamps task started");

    err_t err;
    uint16_t buttonVal;

    for (;;) {

        err = mcp23x17_port_read(&buttonIO, &buttonVal);

        if (err == ESP_OK) {
            this->buttonStatus = (uint8_t) ~(buttonVal & 0xff); // Invert because we are pulling low in hardware.

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

        } else {
            ESP_LOGE(TAG, "An error occurred getting button status");
            esp_backtrace_print(100);
        }

        // set leds
        buttonVal = 0;
        for (int i = 0; i < LED_COUNT + 6; i++) {

            //ESP_LOGD(TAG, "Switching on pixel %d with r: %d, g: %d, b: %d", i, tmpLampData[i].rgb.r, tmpLampData[i].rgb.g, tmpLampData[i].rgb.b);       
            if (i < LED_COUNT) {
                led_strip_set_pixel(&ledStrip, i, this->lampData[i].activeRgb);
            } else {
                // GPB1 and GPB0 are unconnected
                // RGB value must be 255 for non-led lamps
                if (this->lampData[i].activeRgb.red == 255 && this->lampData[i].activeRgb.green == 255 && this->lampData[i].activeRgb.blue == 255) {
                    switch (i) {
                        case LED_COUNT:
                            buttonVal |= (1 << 15); //GPB7 (Start)
                            break;
                        case LED_COUNT + 1:
                            buttonVal |= (1 << 14); //GPB6 (Collect)
                            break;
                        case LED_COUNT + 2:
                            buttonVal |= (1 << 13); // GPB5
                            break;
                        case LED_COUNT + 3:
                            buttonVal |= (1 << 12); // GPB4
                            break;
                        case LED_COUNT + 4:
                            buttonVal |= (1 << 11); // GPB3
                            break;
                        case LED_COUNT + 5:
                            buttonVal |= (1 << 10); // GPB2
                            break;
                    }
                }
            }
        }

        led_strip_flush(&ledStrip);
        mcp23x17_port_write(this->getButtonIO(), buttonVal);

        std::this_thread::sleep_for(std::chrono::milliseconds(25));

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
