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

#include "../config/sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/gpio.h"

#include "rgb.h"
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

    memset(&movesDisplay, 0, sizeof (ht16k33_t));
    memset(&creditDisplay, 0, sizeof (ht16k33_t));
    memset(&bankDisplay, 0, sizeof (ht16k33_t));
    memset(&buttonIO, 0, sizeof (mcp23x17_t));

    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(LED_GPIO, RMT_TX_CHANNEL);
    config.clk_div = 2;
    if (rmt_config(&config) != ESP_OK) {
        ESP_LOGE(TAG, "Could not initialise RMT");
    } else {
        ESP_LOGD(TAG, "RMT initialised");
        rmt_driver_install(config.channel, 0, 0);
    }

    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(LED_COUNT, (led_strip_dev_t) config.channel);
    led_strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!led_strip) {
        ESP_LOGE(TAG, "WS2812 driver installation failed!");
    } else {
        ESP_LOGD(TAG, "WS2812 driver installation succeeded");
    }

    if (ht16k33_init_desc(&bankDisplay, 0, HT16K33_ADDR_BASE + 1, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "Could not initialise bank display");
    } else {
        ht16k33_display_on(&bankDisplay);
        ht16k33_write_value(&bankDisplay, "%05d", 88888);
        ESP_LOGD(TAG, "Bank display initialisation succeeded");
    }

    if (ht16k33_init_desc(&creditDisplay, 0, HT16K33_ADDR_BASE, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "Could not initialise credit display");
    } else {
        ht16k33_display_on(&creditDisplay);
        ht16k33_write_value(&creditDisplay, "%05d", 88888);
        ESP_LOGD(TAG, "Credit display initialisation succeeded");
    }

    if (ht16k33_init_desc(&movesDisplay, 0, HT16K33_ADDR_BASE + 2, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "Could not initialise moves display");
    } else {
        ht16k33_display_on(&movesDisplay);
        ht16k33_write_value(&movesDisplay, "%02d", 88);
        ESP_LOGD(TAG, "Moves display initialisation succeeded");
    }

    buttonIO.cfg.master.clk_speed = I2C_FREQ_HZ;
    buttonIO.cfg.mode = I2C_MODE_MASTER;

    if (mcp23x17_init_desc(&buttonIO, 0, MCP23X17_ADDR_BASE + 7, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "Could not initialise button interface");
    } else {

        uint16_t portMode = 0x00ff; // PortA input, portB output (0 = output, 1 = input)
        uint16_t pullup = 0x00ff; // internal pullup resistors.

        mcp23x17_port_set_mode(&buttonIO, portMode);
        mcp23x17_port_set_pullup(&buttonIO, pullup);

        ESP_LOGI(TAG, "Button interface initialisation succeeded");
    }

    if (m20ly02z_init(MD_STROBE, MD_OE, MD_CLK, MD_DATA) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialise VFD display");
    }

    DisplayController::clearText();

    hue = 0;
    start_rgb = 0;    

    DisplayController::resetLampData(true);

    xTaskCreate(&updateLampsTask, "update lamps", 2048, this, 5, NULL);
    xTaskCreate(&updateSevenSegDisplaysTask, "update_7seg_displays", 2048, mainController, 5, NULL);    

    DisplayController::testLamps();    

    ESP_LOGD(TAG, "Exiting DisplayController::initialise()");
    return ESP_OK;
}

void DisplayController::beginAnimation() {
    xTaskCreate(&rainbowChaseTask, "animation", 2048, mainController, 5, NULL);
}

void DisplayController::resetLampData(bool performUpdate) {
    ESP_LOGI(TAG, "Entering resetLampData()");
    // initialise lamps
    for (int i = 0; i < (LED_COUNT + 6); i++) {
        lampData[i].lampState = LampState::off;
        // set lamp colour to white
        lampData[i].rgb.b = 255;
        lampData[i].rgb.g = 255;
        lampData[i].rgb.r = 255;
    }
    ESP_LOGI(TAG, "Exiting resetLampData()");
}

void DisplayController::testLamps() {
    ESP_LOGI(TAG, "Entering testLamps()");
    // initialise lamps
    for (int i = 0; i < (LED_COUNT + 6); i++) {
        lampData[i].lampState = LampState::on;
        // set lamp colour to white
        lampData[i].rgb.b = 255;
        lampData[i].rgb.g = 255;
        lampData[i].rgb.r = 255;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "Exiting testLamps()");
}

LampData* DisplayController::getLampData() {
    return lampData;
}

uint8_t DisplayController::getButtonStatus() {
    ESP_LOGD(TAG, "Entering getButtonStatus()");

    uint16_t val;
    uint8_t result;
    
    if (mcp23x17_port_read(&buttonIO, &val) == ESP_OK) {
        result = (uint8_t) ~(val & 0xff);
    } else {
        result = 0; // Failsafe
    }    

    ESP_LOGD(TAG, "Exiting getButtonStatus() with value %u", result);
    return result;
}

void DisplayController::setText(const string& text) {
    m20ly02z_clear();

    for (char str_char : text) {
        m20ly02z_send_byte(str_char);
    }

}

void DisplayController::clearText() {
    m20ly02z_clear();
}

void DisplayController::displayText() {

}

led_strip_t* DisplayController::getLedStrip() {
    return this->led_strip;
}

mcp23x17_t* DisplayController::getButtonIO() {
    return &this->buttonIO;
}

ht16k33_t* DisplayController::getBankDisplay(void) {
    return &this->bankDisplay;
}

ht16k33_t* DisplayController::getCreditDisplay(void) {
    return &this->creditDisplay;
}

ht16k33_t* DisplayController::getMovesDisplay(void) {
    return &this->movesDisplay;
}

void DisplayController::rainbowChaseTask(void *pvParameters) {
    MainController *mainController = reinterpret_cast<MainController *> (pvParameters);
    // Show simple rainbow chasing pattern
    ESP_LOGI(TAG, "Animation task started");

    DisplayController *displayController = mainController->getDisplayController();
    LampData *lampData = displayController->getLampData();
    displayController->resetLampData(true);

    rgb_t rgb_data;
    uint8_t hue;
    uint8_t start_rgb = 0;

    // switch all led's on;
    for (int i = 0; i < LED_COUNT; i++) {
        lampData[i].lampState = LampState::on;
    }
   
    while (!mainController->getGame()->isGameInProgress()) {

        for (int i = 0; i < 3; i++) {
            for (int j = i; j < LED_COUNT; j += 3) {

                // Build RGB values
                hue = j * 360 / LED_COUNT + start_rgb;
                mainController->getDisplayController()->ledStripHsv2rgb(hue, 100, 100, &rgb_data.b, &rgb_data.g, &rgb_data.r);
                // Write RGB values to strip driver
                lampData[j].rgb = rgb_data;            
            }
            
            vTaskDelay(pdMS_TO_TICKS(CHASE_SPEED_MS));
        }
        start_rgb += 60;

    }

    // TODO: rename this method and add following
    //    if ((!game->isGameInProgress()) && ((unsigned long) (currentMillis - msgUpdatePreviousMillis) >= msgUpdateRefreshInterval)) {
    //        switch (state) {
    //            case 0:
    //                getDisplayController()->setText("       FROZEN       ");
    //                break;
    //            case 1:
    //                getDisplayController()->setText("      PLAY ME       ");
    //                break;
    //            case 2:
    //                getDisplayController()->setText("     20CT GAME      ");
    //                break;
    //            case 3:
    //                getDisplayController()->setText("    INSERT COINS    ");
    //                break;
    //        }
    //
    //        // reset state
    //        if (state >= 3) {
    //            state = 0;
    //        } else {
    //            state++;
    //        }
    //
    //
    //        msgUpdatePreviousMillis = millis();
    //    }
    vTaskDelete(NULL);
}


void DisplayController::updateLampsTask(void *pvParameters) {
    ESP_LOGI(TAG, "Update Lamps task started");

    DisplayController *displayController = reinterpret_cast<DisplayController *> (pvParameters);

    uint16_t btnLamps = 0;

    LampData *tmpLampData = displayController->getLampData();
    led_strip_t *ledStrip = displayController->getLedStrip();

    for (;;) {

        // set leds
        for (int i = 0; i < LED_COUNT; i++) {            
            if (tmpLampData[i].lampState == LampState::on || tmpLampData[i].lampState == LampState::blinkfast || tmpLampData[i].lampState == LampState::blinkslow) {            
                ESP_LOGD(TAG, "Switching on pixel %d with r: %d, g: %d, b: %d", i, tmpLampData[i].rgb.r, tmpLampData[i].rgb.g, tmpLampData[i].rgb.b);
                ledStrip->set_pixel(ledStrip, i, tmpLampData[i].rgb.r, tmpLampData[i].rgb.g, tmpLampData[i].rgb.b);                
            } else {
                ESP_LOGD(TAG, "Switching off pixel %d", i);
                ledStrip->clear(ledStrip, i);
            }
        }
        ledStrip->refresh(ledStrip, 100);
        
        btnLamps = 0;
        // set button lamps
        for (int i = 0; i < 6; i++) {
            if (tmpLampData[i + LED_COUNT].lampState == LampState::on || tmpLampData[i + LED_COUNT].lampState == LampState::blinkfast || tmpLampData[i + LED_COUNT].lampState == LampState::blinkslow) {
                btnLamps |= (1 << (i + 8));
            }
        }        
        mcp23x17_port_write(displayController->getButtonIO(), btnLamps);


        vTaskDelay(pdMS_TO_TICKS(50));

        // set leds
        for (int i = 0; i < LED_COUNT; i++) {            
            if (tmpLampData[i].lampState == LampState::on || tmpLampData[i].lampState == LampState::blinkslow) {
                ESP_LOGD(TAG, "Switching on pixel %d with r: %d, g: %d, b: %d", i, tmpLampData[i].rgb.r, tmpLampData[i].rgb.g, tmpLampData[i].rgb.b);
                ledStrip->set_pixel(ledStrip, i, tmpLampData[i].rgb.r, tmpLampData[i].rgb.g, tmpLampData[i].rgb.b);               
            } else {
                ESP_LOGD(TAG, "Switching off pixel %d", i);
                ledStrip->clear(ledStrip, i);
            }
        }
        ledStrip->refresh(ledStrip, 100);

        btnLamps = 0;
        // set button lamps
        for (int i = 0; i < 6; i++) {
            if (tmpLampData[i + LED_COUNT].lampState == LampState::on || tmpLampData[i + LED_COUNT].lampState == LampState::blinkslow) {
                btnLamps |= (1 << (i + 8));
            }
        }
        
        mcp23x17_port_write(displayController->getButtonIO(), btnLamps);


        vTaskDelay(pdMS_TO_TICKS(50));

        // set leds
        for (int i = 0; i < LED_COUNT; i++) {            
            if (tmpLampData[i].lampState == LampState::on) {
                ESP_LOGD(TAG, "Switching on pixel %d with r: %d, g: %d, b: %d", i, tmpLampData[i].rgb.r, tmpLampData[i].rgb.g, tmpLampData[i].rgb.b);
                ledStrip->set_pixel(ledStrip, i, tmpLampData[i].rgb.r, tmpLampData[i].rgb.g, tmpLampData[i].rgb.b);                
            } else {
                ESP_LOGD(TAG, "Switching off pixel %d", i);
                ledStrip->clear(ledStrip, i);
            }
        }
        ledStrip->refresh(ledStrip, 100);
        
        btnLamps = 0;
        // set button lamps
        for (int i = 0; i < 6; i++) {
            if (tmpLampData[i + LED_COUNT].lampState == LampState::on) {
                btnLamps |= (1 << (i + 8));
            }
        }      
        mcp23x17_port_write(displayController->getButtonIO(), btnLamps);


        vTaskDelay(pdMS_TO_TICKS(50));

        // set leds
        for (int i = 0; i < LED_COUNT; i++) {            
            if (tmpLampData[i].lampState == LampState::on || tmpLampData[i].lampState == LampState::blinkfast) {
                ESP_LOGD(TAG, "Switching on pixel %d with r: %d, g: %d, b: %d", i, tmpLampData[i].rgb.r, tmpLampData[i].rgb.g, tmpLampData[i].rgb.b);
                ledStrip->set_pixel(ledStrip, i, tmpLampData[i].rgb.r, tmpLampData[i].rgb.g, tmpLampData[i].rgb.b);                
            } else {
                ESP_LOGD(TAG, "Switching off pixel %d", i);
                ledStrip->clear(ledStrip, i);
            }
        }
        ledStrip->refresh(ledStrip, 100);

        btnLamps = 0;
        // set button lamps
        for (int i = 0; i < 6; i++) {
            if (tmpLampData[i + LED_COUNT].lampState == LampState::on || tmpLampData[i + LED_COUNT].lampState == LampState::blinkfast) {
                btnLamps |= (1 << (i + 8));
            }
        }        
        mcp23x17_port_write(displayController->getButtonIO(), (btnLamps & 0xff00));

        vTaskDelay(pdMS_TO_TICKS(50));

    }

}

/**
 * @brief Task to refresh the seven segment displays. 
 * 
 * @param pvParameter 
 */
void DisplayController::updateSevenSegDisplaysTask(void *pvParameters) {
    ESP_LOGI(TAG, "Update 7-segment display task started");
    MainController *mainController = reinterpret_cast<MainController *> (pvParameters);
    for (;;) {

        uint16_t bank = mainController->getMoneyController()->getBank();
        uint16_t credit = mainController->getMoneyController()->getCredit();

        ht16k33_write_value(mainController->getDisplayController()->getBankDisplay(), "%05d", bank);
        ht16k33_write_value(mainController->getDisplayController()->getCreditDisplay(), "%05d", credit);

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
