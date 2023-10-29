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
#include <vector>

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

std::array<int, 0> DisplayController::SEGMENTS;
std::array<int, DisplayController::NUDGE_LAMPS_LENGTH> DisplayController::NUDGE_LAMPS;
std::array<int, DisplayController::TRAIL_LAMPS_LENGTH> DisplayController::TRAIL_LAMPS;
std::array<int, DisplayController::FEATURE_LAMPS_LENGTH> DisplayController::FEATURE_LAMPS;

LampData::LampData() {
    this->setLampState(LampState::off);
    this->setRgb(rgb_from_values(255, 255, 255));
    this->setActiveRgb(rgb_from_values(255, 255, 255));
}

rgb_t LampData::getActiveRgb() {
    return this->activeRgb;
}

void LampData::setActiveRgb(rgb_t rgb) {
    this->activeRgb = rgb;
}

rgb_t LampData::getRgb() {
    return this->rgb;
}

void LampData::setRgb(rgb_t rgb) {
    this->rgb = rgb;
}

LampState LampData::getLampState() {
    return this->lampState;
}

void LampData::setLampState(LampState lampState) {
    this->lampState = lampState;
}

DisplayController::DisplayController(MainController* mainController) {

    ESP_LOGD(TAG, "Entering constructor");
    this->mainController = mainController;

    DisplayController::NUDGE_LAMPS[0] = LAMP_NUDGE_1;
    DisplayController::NUDGE_LAMPS[1] = LAMP_NUDGE_2;
    DisplayController::NUDGE_LAMPS[2] = LAMP_NUDGE_3;
    DisplayController::NUDGE_LAMPS[3] = LAMP_NUDGE_4;
    DisplayController::NUDGE_LAMPS[4] = LAMP_NUDGE_5;

    DisplayController::TRAIL_LAMPS[0] = LAMP_TRAIL_20_CENT;
    DisplayController::TRAIL_LAMPS[1] = LAMP_TRAIL_40_CENT;
    DisplayController::TRAIL_LAMPS[2] = LAMP_TRAIL_60_CENT;
    DisplayController::TRAIL_LAMPS[3] = LAMP_TRAIL_80_CENT;
    DisplayController::TRAIL_LAMPS[4] = LAMP_TRAIL_ONE_EURO;
    DisplayController::TRAIL_LAMPS[5] = LAMP_TRAIL_ONE_TWENTY;
    DisplayController::TRAIL_LAMPS[6] = LAMP_TRAIL_ONE_FOURTY;
    DisplayController::TRAIL_LAMPS[7] = LAMP_TRAIL_ONE_SIXTY;
    DisplayController::TRAIL_LAMPS[8] = LAMP_TRAIL_ONE_EIGHTY;
    DisplayController::TRAIL_LAMPS[9] = LAMP_TRAIL_TWO_EURO;
    DisplayController::TRAIL_LAMPS[10] = LAMP_TRAIL_TWO_FOURTY;
    DisplayController::TRAIL_LAMPS[11] = LAMP_TRAIL_TWO_EIGHTY;
    DisplayController::TRAIL_LAMPS[12] = LAMP_TRAIL_THREE_FOURTY;
    DisplayController::TRAIL_LAMPS[13] = LAMP_TRAIL_THREE_EIGHTY;
    DisplayController::TRAIL_LAMPS[14] = LAMP_TRAIL_FOUR_TWENTY;
    DisplayController::TRAIL_LAMPS[15] = LAMP_TRAIL_FOUR_SIXTY;
    DisplayController::TRAIL_LAMPS[16] = LAMP_TRAIL_FIVE_EURO;

    DisplayController::FEATURE_LAMPS[0] = LAMP_MATRIX_FREE_SPIN_1_2;
    DisplayController::FEATURE_LAMPS[1] = LAMP_MATRIX_DOUBLE_MONEY_1_3;
    DisplayController::FEATURE_LAMPS[2] = LAMP_MATRIX_SHUFFLE_1_1;
    DisplayController::FEATURE_LAMPS[3] = LAMP_MATRIX_LOSE_2_2;
    DisplayController::FEATURE_LAMPS[4] = LAMP_MATRIX_PALACE_2_3;
    DisplayController::FEATURE_LAMPS[5] = LAMP_MATRIX_PALACE_2_1;
    DisplayController::FEATURE_LAMPS[6] = LAMP_MATRIX_SHUFFLE_3_2;
    DisplayController::FEATURE_LAMPS[7] = LAMP_MATRIX_LOSE_3_3;
    DisplayController::FEATURE_LAMPS[8] = LAMP_MATRIX_FREE_SPIN_3_1;
    DisplayController::FEATURE_LAMPS[9] = LAMP_MATRIX_HI_LO_4_2;
    DisplayController::FEATURE_LAMPS[10] = LAMP_MATRIX_FREE_SPIN_4_3;
    DisplayController::FEATURE_LAMPS[11] = LAMP_MATRIX_PALACE_4_1;

    movesDisplay = new HT16K33(I2C_NUM_0, MOVES_DISPLAY_ADDRESS);
    creditDisplay = new HT16K33(I2C_NUM_0, CREDIT_DISPLAY_ADDRESS);
    bankDisplay = new HT16K33(I2C_NUM_0, BANK_DISPLAY_ADDRESS);
    buttonIO = new MCP23x17(I2C_NUM_0, BUTTONS_I2C_ADDRESS);

    
    ESP_LOGD(TAG, "Leaving constructor");
}

esp_err_t DisplayController::initialise() {

    ESP_LOGI(TAG, "Entering DisplayController::initialise()");

    this->buttonStatus = 0;
    
    memset(&ledStrip, 0, sizeof (led_strip_t));

    //ledStrip.is_rgbw = true;
    ledStrip.type = LED_STRIP_WS2812;
    ledStrip.length = LED_COUNT;
    ledStrip.gpio = LED_GPIO;
    //ledStrip.buf = NULL;
    ledStrip.brightness = 255;
    ledStrip.channel = RMT_TX_CHANNEL;

    led_strip_install();

    if (led_strip_init(&this->ledStrip) != ESP_OK) {
        ESP_LOGE(TAG, "WS2812 driver installation failed!");
    } else {
        ESP_LOGD(TAG, "WS2812 driver installation succeeded");
    }

    creditDisplay->display_on();
    creditDisplay->write_value("%05d", 88888);
    ESP_LOGD(TAG, "Credit display initialisation succeeded");

    bankDisplay->display_on();
    bankDisplay->write_value("%05d", 88888);
    ESP_LOGD(TAG, "Bank display initialisation succeeded");

    movesDisplay->display_on();
    movesDisplay->write_value("%05d", 88);
    ESP_LOGD(TAG, "Moves display initialisation succeeded");

    uint16_t portMode = 0x00ff; // PortA input, portB output (0 = output, 1 = input)
    uint16_t pullup = 0x0000; // internal pullup resistors on button pins off - we pull them up in hardware.

    buttonIO->port_set_mode(portMode);
    buttonIO->port_set_pullup(pullup);
    ESP_LOGI(TAG, "Button interface initialisation succeeded");

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    for (int i = 0; i < LED_COUNT + 6; i++) {
        lampData[i] = LampData();
    }

    this->resetLampData();

    // Start a new thread to update the lamps
    auto cfg = esp_pthread_get_default_config();
    cfg.thread_name = "BlinkLamps";
    cfg.prio = 2;
    cfg.stack_size = 1024;
    esp_pthread_set_cfg(&cfg);
    this->blinkLampsThread = std::thread([this]() {
        blinkLampsTask();
    });
    this->blinkLampsThread.detach();

    cfg = esp_pthread_get_default_config();
    cfg.thread_name = "UpdateLamps";
    cfg.prio = 2;
    cfg.stack_size = 1024;
    esp_pthread_set_cfg(&cfg);
    this->updateLampsThread = std::thread([this]() {
        updateLampsTask();
    });
    this->updateLampsThread.detach();

    cfg = esp_pthread_get_default_config();
    cfg.thread_name = "UpdateSevenSeg";
    cfg.prio = 1;
    cfg.stack_size = 1024;
    esp_pthread_set_cfg(&cfg);
    // Start a thread to update the 7-segment displays
    this->updateSevenSegDisplaysThread = std::thread([this]() {
        updateSevenSegDisplaysTask();
    });
    this->updateSevenSegDisplaysThread.detach();

    this->testLamps();

    this->attractMode = false;
    cfg = esp_pthread_get_default_config();
    cfg.thread_name = "AttractMode";
    cfg.prio = 1;
    cfg.stack_size = 4096;
    esp_pthread_set_cfg(&cfg);
    this->attractModeThread = std::thread([this]() {
        attractModeTask();
    });
    this->attractModeThread.detach();

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
        lampData.at(i).setLampState(LampState::off);
        // set lamp colour to white
        lampData.at(i).setRgb(rgb_from_values(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS)); // changed from 255 to try and prevent voltage drop browning out vfd display
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
        lampData.at(i).setLampState(LampState::on);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    resetLampData();
    ESP_LOGD(TAG, "Exiting testLamps()");
}

void DisplayController::setMoves(uint8_t value) {

    ESP_LOGD(TAG, "Entering setMoves(%d)", value);
    movesDisplay->write_value("%02d", value);
    ESP_LOGD(TAG, "Exiting setMoves");
}

std::array<LampData, LED_COUNT + 6 > & DisplayController::getLampData() {

    return this->lampData;
}

uint8_t DisplayController::getButtonStatus() {
    //ESP_LOGD(TAG, "Exiting getButtonStatus() with value %u", this->buttonStatus);

    return this->buttonStatus;
}

uint8_t DisplayController::waitForButton(uint8_t mask) {
    uint8_t btnStatus = getButtonStatus();
    // loop waiting for button press.
    while ((btnStatus & mask) == 0) {

        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        btnStatus = getButtonStatus();
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

MCP23x17* DisplayController::getButtonIO() {

    return this->buttonIO;
}

HT16K33* DisplayController::getBankDisplay() {

    return this->bankDisplay;
}

HT16K33* DisplayController::getCreditDisplay() {

    return this->creditDisplay;
}

HT16K33* DisplayController::getMovesDisplay() {

    return this->movesDisplay;
}

void DisplayController::attractModeTask() {

    ESP_LOGI(TAG, "Attract mode thread started");

    while (1) {
        if (this->isAttractMode()) {

            if (this->isAttractMode()) this->displayText("       FROZEN       ");
            if (this->isAttractMode()) this->resetLampData();

            if (this->isAttractMode()) this->rainbowEffect();

            if (this->isAttractMode()) std::this_thread::sleep_for(std::chrono::seconds(1));

            if (this->isAttractMode()) resetLampData();

            if (this->isAttractMode()) this->displayText("      PLAY ME       ");
            if (this->isAttractMode()) this->chaseEffect();

            if (this->isAttractMode()) std::this_thread::sleep_for(std::chrono::seconds(1));


            if (this->isAttractMode()) this->displayText("     20CT GAME      ");
            if (this->isAttractMode()) this->fadeInOutEffect();


            if (this->isAttractMode()) std::this_thread::sleep_for(std::chrono::seconds(5));

            if (this->isAttractMode()) this->displayText("    INSERT COINS    ");

            if (this->isAttractMode()) std::this_thread::sleep_for(std::chrono::seconds(5));

        } else {

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

void DisplayController::fadeInOutEffect() {

    resetLampData();

    // Trail lamps fade in
    for (int i = 255; i >= 0; i -= 2) {
        for (int j = 0; j < TRAIL_LAMPS.size(); j++) {
            if (!this->attractMode) {
                return;
            }
            lampData[TRAIL_LAMPS[j]].setRgb(rgb_fade_light(rgb_from_values(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS), i));
            lampData[TRAIL_LAMPS[j]].setLampState(LampState::on);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // Nudge lamps fade in
    for (int i = 255; i >= 0; i -= 2) {
        for (int j = 0; j < NUDGE_LAMPS.size(); j++) {
            if (!this->attractMode) {
                return;
            }
            lampData[NUDGE_LAMPS[j]].setRgb(rgb_fade_light(rgb_from_values(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS), i));
            lampData[NUDGE_LAMPS[j]].setLampState(LampState::on);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // Trail lamps fade out
    for (int i = 0; i < 256; i++) {
        for (int j = 0; j < TRAIL_LAMPS.size(); j++) {
            if (!this->attractMode) {
                return;
            }
            lampData[TRAIL_LAMPS[j]].setRgb(rgb_fade_light(rgb_from_values(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS), i));
            lampData[TRAIL_LAMPS[j]].setLampState(LampState::on);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }


    // Feature lamps fade in
    for (int i = 255; i >= 0; i -= 2) {
        for (int j = 0; j < FEATURE_LAMPS.size(); j++) {
            if (!this->attractMode) {
                return;
            }
            lampData[FEATURE_LAMPS[j]].setRgb(rgb_fade_light(rgb_from_values(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS), i));
            lampData[FEATURE_LAMPS[j]].setLampState(LampState::on);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }


    // Nudge lamps fade out
    for (int i = 0; i < 256; i++) {
        for (int j = 0; j < NUDGE_LAMPS.size(); j++) {
            if (!this->attractMode) {
                return;
            }
            lampData[NUDGE_LAMPS[j]].setRgb(rgb_fade_light(rgb_from_values(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS), i));
            lampData[NUDGE_LAMPS[j]].setLampState(LampState::on);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }


    // Feature lamps fade out
    for (int i = 0; i < 256; i++) {
        for (int j = 0; j < FEATURE_LAMPS.size(); j++) {
            if (!this->attractMode) {
                return;
            }
            lampData[FEATURE_LAMPS[j]].setRgb(rgb_fade_light(rgb_from_values(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS), i));
            lampData[FEATURE_LAMPS[j]].setLampState(LampState::on);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

}

void DisplayController::chaseEffect() {
    // Red trail effect           
    resetLampData();

    for (int j = 0; j < 5; j++) {

        // Trail
        for (int i = 0; i < TRAIL_LAMPS.size(); i++) {
            if (!this->attractMode) {
                return;
            }
            lampData.at(TRAIL_LAMPS.at(i)).setRgb(rgb_from_values(0, MAX_BRIGHTNESS, MAX_BRIGHTNESS));
            lampData.at(TRAIL_LAMPS.at(i)).setLampState(LampState::on);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));

        }

        for (int i = 0; i < TRAIL_LAMPS.size(); i++) {
            if (!this->attractMode) {
                return;
            }
            lampData.at(TRAIL_LAMPS.at(i)).setLampState(LampState::off);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
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
                lampData.at(j).setRgb(hsv2rgb_rainbow(hsv_data));
                lampData.at(j).setLampState(LampState::on);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(CHASE_SPEED_MS));
        }
        start_rgb += 60;
    }

    // Trail lamps fade out
    for (int i = 255; i >= 0; i -= 2) {
        for (int j = 0; j < LED_COUNT; j++) {
            lampData.at(j).setRgb(rgb_fade_light(lampData.at(j).getRgb(), i));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    resetLampData();
}

void DisplayController::blinkLampsTask() {

    for (;;) {

        for (int i = 0; i < (LED_COUNT + 6); i++) {

            if (getLampData().at(i).getLampState() == LampState::on || getLampData().at(i).getLampState() == LampState::blinkfast || getLampData().at(i).getLampState() == LampState::blinkslow) {
                if (i < LED_COUNT) {
                    getLampData().at(i).setActiveRgb(getLampData().at(i).getRgb());
                } else {
                    // non- RGB button lamps
                    getLampData().at(i).setActiveRgb(rgb_from_values(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS));
                }
            } else {
                // Set active rgb value to 0 (off or black)
                getLampData().at(i).setActiveRgb(rgb_from_values(0, 0, 0));
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        for (int i = 0; i < (LED_COUNT + 6); i++) {
            if (getLampData().at(i).getLampState() == LampState::on || getLampData().at(i).getLampState() == LampState::blinkslow) {
                if (i < LED_COUNT) {
                    getLampData().at(i).setActiveRgb(getLampData().at(i).getRgb());
                } else {
                    // non- RGB button lamps
                    getLampData().at(i).setActiveRgb(rgb_from_values(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS));
                }
            } else {
                // Set active rgb value to 0 (off or black)
                getLampData().at(i).setActiveRgb(rgb_from_values(0, 0, 0));
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        for (int i = 0; i < (LED_COUNT + 6); i++) {
            if (getLampData().at(i).getLampState() == LampState::on) {
                if (i < LED_COUNT) {
                    getLampData().at(i).setActiveRgb(getLampData().at(i).getRgb());
                } else {
                    // non- RGB button lamps
                    getLampData().at(i).setActiveRgb(rgb_from_values(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS));
                }
            } else {
                // Set active rgb value to 0 (off or black)
                getLampData().at(i).setActiveRgb(rgb_from_values(0, 0, 0));
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        for (int i = 0; i < (LED_COUNT + 6); i++) {
            if (getLampData().at(i).getLampState() == LampState::on || getLampData().at(i).getLampState() == LampState::blinkfast) {
                if (i < LED_COUNT) {
                    getLampData().at(i).setActiveRgb(getLampData().at(i).getRgb());
                } else {
                    // non- RGB button lamps
                    getLampData().at(i).setActiveRgb(rgb_from_values(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS));
                }
            } else {
                // Set active rgb value to 0 (off or black)
                getLampData().at(i).setActiveRgb(rgb_from_values(0, 0, 0));
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

        err = buttonIO->port_read(buttonVal);

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
            this-> buttonStatus = 0;
            ESP_LOGE(TAG, "An error occurred getting button status");
            //esp_backtrace_print
        }

        // set leds
        buttonVal = 0;
        for (int i = 0; i < (LED_COUNT + 6); i++) {

            //ESP_LOGD(TAG, "Switching on pixel %d with r: %d, g: %d, b: %d", i, tmpLampData[i].rgb.r, tmpLampData[i].rgb.g, tmpLampData[i].rgb.b);       
            if (i < LED_COUNT) {
                led_strip_set_pixel(&ledStrip, i, getLampData().at(i).getActiveRgb());
            } else {
                // GPB1 and GPB0 are unconnected
                // activeRgb value must be have have at least one channel (r, g or b) with a positive value to light
                if (getLampData().at(i).getActiveRgb().r > 0 || getLampData().at(i).getActiveRgb().g > 0 || getLampData().at(i).getActiveRgb().b > 0) {
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
        buttonIO->port_write(buttonVal);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

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
            bankDisplay->write_value("%05d", bank);
        }

        if (initialRun || (credit != mainController->getMoneyController()->getCredit())) {
            credit = mainController->getMoneyController()->getCredit();
            creditDisplay->write_value("%05d", credit);
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
