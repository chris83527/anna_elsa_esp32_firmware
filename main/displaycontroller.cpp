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
 *    may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file displaycontroller.cpp
 *
 * Higher level routines for controlling HT16K33 I2C LED Matrix driver chips and
 * ws2812 LEDs
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <bitset>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <string>
#include <vector>

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_debug_helpers.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_pthread.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"

#include "ht16k33.h"
#include "m20ly02z.h"
#include "mcp23x17.h"

#include "audiocontroller.h"
#include "displaycontroller.h"
#include "game.h"
#include "moneycontroller.h"


using namespace std;

static const char* TAG = "DisplayController";
static string vfdText;

DisplayController::DisplayController(MoneyController& moneyController,
                                     I2CManager& i2cmgr)
    : movesDisplay(HT16K33(i2cmgr, MOVES_DISPLAY_ADDRESS)),
      creditDisplay(HT16K33(i2cmgr, CREDIT_DISPLAY_ADDRESS)),
      bankDisplay(HT16K33(i2cmgr, BANK_DISPLAY_ADDRESS)),
      buttonIO(MCP23x17(i2cmgr, BUTTONS_I2C_ADDRESS)),
      moneyController(moneyController),
      oledController(OledController(i2cmgr, 0x3c)), i2cManager(i2cmgr)
{
    ESP_LOGD(TAG, "Entering constructor");

    // Perform this here so we have debug output
    oledController.initialise();

    ESP_LOGD(TAG, "Leaving constructor");
}

DisplayController::~DisplayController() = default;

esp_err_t DisplayController::initialise()
{
    ESP_LOGI(TAG, "Entering DisplayController::initialise()");

    this->buttonStatus = 0;

    if (ws28xx_init(LED_GPIO, WS2812B, LED_COUNT, &ws2812_buffer) != ESP_OK)
    {
        ESP_LOGE(TAG, "WS2812 driver installation failed!");
    }
    else
    {
        ESP_LOGD(TAG, "WS2812 driver installation succeeded");
    }

    creditDisplay.display_on();
    creditDisplay.write_value("%05d", 88888);
    ESP_LOGD(TAG, "Credit display initialisation succeeded");

    bankDisplay.display_on();
    bankDisplay.write_value("%05d", 88888);
    ESP_LOGD(TAG, "Bank display initialisation succeeded");

    movesDisplay.display_on();
    movesDisplay.write_value("%05d", 88);
    ESP_LOGD(TAG, "Moves display initialisation succeeded");

    buttonIO.setGPIOAInputOutputMode(0xff); // PORT A (switches) - input
    buttonIO.setGPIOBInputOutputMode(0x00); // PORT B (button lamps) - output
    buttonIO.setGPIOAPullup(0x00); // GPIOA Pullups off
    buttonIO.setGPIOBPullup(0x00); // GPIOB Pullups off
    buttonIO.setGPIOAInputPolarity(
        0xff); // Invert polarity (bit refelcts the opposite logic state of the
    // input pin)
    ESP_LOGI(TAG, "Button interface initialisation succeeded");

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    this->resetLampData();

    // Start a new thread to update the lamps
    auto cfg = esp_pthread_get_default_config();
    cfg.thread_name = "BlinkLamps";
    cfg.prio = 2;
    cfg.stack_size = 2048;
    cfg.pin_to_core = 1;
    esp_pthread_set_cfg(&cfg);
    this->blinkLampsThread = std::thread([&]() { blinkLampsTask(); });
    this->blinkLampsThread.detach();

    cfg = esp_pthread_get_default_config();
    cfg.thread_name = "UpdateLamps";
    cfg.prio = 2;
    cfg.stack_size = 4096;
    cfg.pin_to_core = 1;
    esp_pthread_set_cfg(&cfg);
    this->updateLampsThread = std::thread([&]() { updateLampsTask(); });
    this->updateLampsThread.detach();

    cfg = esp_pthread_get_default_config();
    cfg.thread_name = "UpdateSevenSeg";
    cfg.prio = 1;
    cfg.pin_to_core = 1;
    cfg.stack_size = 4096;
    esp_pthread_set_cfg(&cfg);
    // Start a thread to update the 7-segment displays
    this->updateSevenSegDisplaysThread =
        std::thread([this]() { updateSevenSegDisplaysTask(); });
    this->updateSevenSegDisplaysThread.detach();

    this->testLamps();

    this->attractMode = false;
    cfg = esp_pthread_get_default_config();
    cfg.thread_name = "AttractMode";
    cfg.prio = 1;
    cfg.pin_to_core = 1;
    cfg.stack_size = 4096;
    esp_pthread_set_cfg(&cfg);
    this->attractModeThread = std::thread([&]() { attractModeTask(); });
    this->attractModeThread.detach();

    ESP_LOGD(TAG, "Exiting DisplayController::initialise()");

    return ESP_OK;
}

void DisplayController::beginAttractMode() { this->attractMode = true; }

void DisplayController::stopAttractMode() { this->attractMode = false; }

void DisplayController::resetLampData()
{
    ESP_LOGD(TAG, "Entering resetLampData()");
    // initialise lamps
    for (int i = 0; i < (LED_COUNT + 6); i++)
    {
        lampData.at(i).lampState = LampState::off;
        // set lamp colour to white
        lampData.at(i).rgb = rgbFromValues(
            MAX_BRIGHTNESS, MAX_BRIGHTNESS,
            MAX_BRIGHTNESS); // changed from 255 to try and prevent voltage drop
        // browning out vfd display
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ESP_LOGD(TAG, "Exiting resetLampData()");
}

void DisplayController::scrollOledText(const std::string& text)
{
    oledController.scrollText(text);
}

void DisplayController::clearOledDisplay()
{
    oledController.clearDisplay();
}

void DisplayController::displayOledText(const std::string& text, int lineNumber,
                                        bool invert)
{
    oledController.displayText(text, lineNumber, invert);
}

bool DisplayController::isAttractMode() const { return attractMode; }

void DisplayController::testLamps()
{
    ESP_LOGD(TAG, "Entering testLamps()");
    // initialise lamps
    // switch all LEDs on;
    resetLampData();
    for (int i = 0; i < (LED_COUNT + 6); i++)
    {
        lampData.at(i).lampState = LampState::on;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    resetLampData();
    ESP_LOGD(TAG, "Exiting testLamps()");
}

void DisplayController::setMoves(uint8_t value)
{
    ESP_LOGD(TAG, "Entering setMoves(%d)", value);
    movesDisplay.write_value("%02d", value);
    ESP_LOGD(TAG, "Exiting setMoves");
}

std::array<DisplayController::lamp_data_t, LED_COUNT + 6>&
DisplayController::getLampData()
{
    return this->lampData;
}

uint8_t DisplayController::getButtonStatus()
{
    esp_err_t err = buttonIO.readGPIOA(this->buttonStatus);

    if (err == ESP_OK)
    {
        if ((this->buttonStatus & (1 << BTN_DOOR)) == 0)
        {
            if (!this->doorOpen)
            {
                ESP_LOGI(TAG, "Door open!");
            }
            this->doorOpen = true;
        }
        else
        {
            if (this->doorOpen)
            {
                ESP_LOGI(TAG, "Door closed!");
            }
            this->doorOpen = false;
        }
    }
    else
    {
        this->buttonStatus = 0;
        ESP_LOGE(TAG, "An error occurred getting button status");
        // esp_backtrace_print
    }

    return this->buttonStatus;
}

uint8_t DisplayController::waitForButton(uint8_t mask) const
{
    // loop waiting for button press.
    while ((mask & this->buttonStatus) == 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return this->buttonStatus;
}

void DisplayController::displayVFDText(const string& text)
{
    // Only update if we need to
    if (vfdText != text)
    {
        m20ly02z_clear();

        for (char str_char : text)
        {
            m20ly02z_send_byte(str_char);
        }

        vfdText = text;
    }
}

void DisplayController::clearText() { m20ly02z_clear(); }

MCP23x17& DisplayController::getButtonIO() { return this->buttonIO; }

HT16K33& DisplayController::getBankDisplay() { return this->bankDisplay; }

HT16K33& DisplayController::getCreditDisplay() { return this->creditDisplay; }

HT16K33& DisplayController::getMovesDisplay() { return this->movesDisplay; }

void DisplayController::attractModeTask()
{
    ESP_LOGI(TAG, "Attract mode thread started");

    while (true)
    {
        if (this->isAttractMode())
        {
            if (this->isAttractMode())
            {
                displayVFDText("  WOODS AMUSEMENTS  ");
            }
            std::this_thread::sleep_for(std::chrono::seconds(5));

            if (this->isAttractMode())
            {
                displayVFDText("      PRESENTS      ");
            }
            std::this_thread::sleep_for(std::chrono::seconds(5));

            if (this->isAttractMode())
            {
                resetLampData();
                displayVFDText("       FROZEN       ");
                this->chaseEffect();
            }

            std::this_thread::sleep_for(std::chrono::seconds(5));

            if (this->isAttractMode())
            {
                displayVFDText("     20CT GAME      ");
                this->fadeInOutEffect();
            }

            std::this_thread::sleep_for(std::chrono::seconds(5));

            if (this->isAttractMode())
                displayVFDText("    INSERT COINS    ");

            if (this->isAttractMode())
                std::this_thread::sleep_for(std::chrono::seconds(5));
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

void DisplayController::fadeInOutEffect()
{
    resetLampData();

    // Trail lamps fade in
    for (int i = 0; i < MAX_BRIGHTNESS; i += 2)
    {
        for (int j : TRAIL_LAMPS)
        {
            if (!isAttractMode())
            {
                return;
            }
            lampData[j].rgb = rgbFromValues(i, i, i);
            lampData[j].lampState = LampState::on;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // Nudge lamps fade in
    for (int i = 0; i < MAX_BRIGHTNESS; i += 2)
    {
        for (int j : NUDGE_LAMPS)
        {
            if (!isAttractMode())
            {
                return;
            }
            lampData[j].rgb = rgbFromValues(i, i, i);
            lampData[j].lampState = LampState::on;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // Trail lamps fade out
    for (int i = MAX_BRIGHTNESS; i > 0; i -= 2)
    {
        for (int j : TRAIL_LAMPS)
        {
            if (!isAttractMode())
            {
                return;
            }
            lampData[j].rgb = rgbFromValues(i, i, i);
            lampData[j].lampState = LampState::on;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // Feature lamps fade in
    for (int i = 0; i < MAX_BRIGHTNESS; i += 2)
    {
        for (int j : FEATURE_LAMPS)
        {
            if (!isAttractMode())
            {
                return;
            }
            lampData[j].rgb = rgbFromValues(i, i, i);
            lampData[j].lampState = LampState::on;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // Nudge lamps fade out
    for (int i = MAX_BRIGHTNESS; i > 0; i -= 2)
    {
        for (int j : NUDGE_LAMPS)
        {
            if (!isAttractMode())
            {
                return;
            }
            lampData[j].rgb = rgbFromValues(i, i, i);
            lampData[j].lampState = LampState::on;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    // Feature lamps fade out
    for (int i = MAX_BRIGHTNESS; i > 0; i -= 2)
    {
        for (int j : FEATURE_LAMPS)
        {
            if (!isAttractMode())
            {
                return;
            }
            lampData[j].rgb = rgbFromValues(i, i, i);
            lampData[j].lampState = (LampState::on);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

void DisplayController::chaseEffect()
{
    // Red trail effect
    resetLampData();

    for (int j = 0; j < 5; j++)
    {
        // Trail
        for (int i : TRAIL_LAMPS)
        {
            if (this->attractMode != true)
            {
                return;
            }
            lampData.at(i).rgb =
                rgbFromValues(0, MAX_BRIGHTNESS, MAX_BRIGHTNESS);
            lampData.at(i).lampState = LampState::on;
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }

        for (int i : TRAIL_LAMPS)
        {
            if (this->attractMode != true)
            {
                return;
            }
            lampData.at(i).lampState = (LampState::off);
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
    }
}

void DisplayController::rainbowEffect()
{
    uint8_t start_rgb = 0;

    // Rainbow effect (10 repeats)
    for (int k = 0; k < 10; k++)
    {
        for (int i = 0; i < 3; i++)
        {
            for (int j = i; j < LED_COUNT; j += 3)
            {
                if (!this->attractMode)
                {
                    return;
                }

                // Build RGB values
                uint8_t hue = j * 360 / LED_COUNT + start_rgb;
                uint8_t sat = 255;
                uint8_t val = 255;

                // Write RGB values to strip driver
                CRGB rgbData = {};
                hsvToRgbRainbow(hue, sat, val, rgbData);
                lampData.at(j).rgb = rgbData;
                lampData.at(j).lampState = LampState::on;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(CHASE_SPEED_MS));
        }
        start_rgb += 60;
    }

    resetLampData();
}

void DisplayController::blinkLampsTask()
{
    for (;;)
    {
        ESP_LOGD(TAG, "blink lamp loop");

        for (int i = 0; i < (LED_COUNT + 6); i++)
        {
            if (this->lampData.at(i).lampState == LampState::on ||
                this->lampData.at(i).lampState == LampState::blinkfast ||
                this->lampData.at(i).lampState == LampState::blinkslow)
            {
                if (i < LED_COUNT)
                {
                    this->lampData.at(i).activeRgb = this->lampData.at(i).rgb;
                }
                else
                {
                    // non- RGB button lamps
                    this->lampData.at(i).activeRgb =
                        rgbFromValues(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS);
                }
            }
            else
            {
                // Set active rgb value to 0 (off or black)
                this->lampData.at(i).activeRgb = rgbFromValues(0, 0, 0);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        for (int i = 0; i < (LED_COUNT + 6); i++)
        {
            if (this->lampData.at(i).lampState == LampState::on ||
                this->lampData.at(i).lampState == LampState::blinkslow)
            {
                if (i < LED_COUNT)
                {
                    this->lampData.at(i).activeRgb = this->lampData.at(i).rgb;
                }
                else
                {
                    // non- RGB button lamps
                    this->lampData.at(i).activeRgb =
                        rgbFromValues(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS);
                }
            }
            else
            {
                // Set active rgb value to 0 (off or black)
                this->lampData.at(i).activeRgb = rgbFromValues(0, 0, 0);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        for (int i = 0; i < (LED_COUNT + 6); i++)
        {
            if (this->lampData.at(i).lampState == LampState::on)
            {
                if (i < LED_COUNT)
                {
                    this->lampData.at(i).activeRgb = this->lampData.at(i).rgb;
                }
                else
                {
                    // non- RGB button lamps
                    this->lampData.at(i).activeRgb =
                        rgbFromValues(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS);
                }
            }
            else
            {
                // Set active rgb value to 0 (off or black)
                this->lampData.at(i).activeRgb = rgbFromValues(0, 0, 0);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        for (int i = 0; i < (LED_COUNT + 6); i++)
        {
            if (this->lampData.at(i).lampState == LampState::on ||
                this->lampData.at(i).lampState == LampState::blinkfast)
            {
                if (i < LED_COUNT)
                {
                    this->lampData.at(i).activeRgb = this->lampData.at(i).rgb;
                }
                else
                {
                    // non- RGB button lamps
                    this->lampData.at(i).activeRgb =
                        rgbFromValues(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS);
                }
            }
            else
            {
                // Set active rgb value to 0 (off or black)
                this->lampData.at(i).activeRgb = rgbFromValues(0, 0, 0);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void DisplayController::updateLampsTask()
{
    ESP_LOGI(TAG, "Update Lamps task started");

    //esp_err_t err;

    uint8_t lampVal = 0;

    for (;;)
    {
        ESP_LOGD(TAG, "update lamp loop");

        // set leds
        lampVal = 0;
        for (int i = 0; i < (LED_COUNT + 6); i++)
        {
            // ESP_LOGD(TAG, "Switching on pixel %d with r: %d, g: %d, b: %d", i,
            // tmpLampData[i].rgb.r, tmpLampData[i].rgb.g, tmpLampData[i].rgb.b);
            if (i < LED_COUNT)
            {
                CRGB ledRGB = this->lampData.at(i).activeRgb;
                //led_strip_set_pixel(ledStripHandle, i, ledRGB.r, ledRGB.g, ledRGB.b);
                ws2812_buffer[i] = ledRGB;
            }
            else
            {
                // GPB1 and GPB0 are unconnected
                // activeRgb value must be have have at least one channel (r, g or b)
                // with a positive value to light
                if (this->lampData.at(i).activeRgb.r > 0 ||
                    this->lampData.at(i).activeRgb.g > 0 ||
                    this->lampData.at(i).activeRgb.b > 0)
                {
                    switch (i)
                    {
                    case LED_COUNT:
                        lampVal |= (1 << 7); // GPB7 (Start)
                        break;
                    case LED_COUNT + 1:
                        lampVal |= (1 << 6); // GPB6 (Collect)
                        break;
                    case LED_COUNT + 2:
                        lampVal |= (1 << 5); // GPB5
                        break;
                    case LED_COUNT + 3:
                        lampVal |= (1 << 4); // GPB4
                        break;
                    case LED_COUNT + 4:
                        lampVal |= (1 << 3); // GPB3
                        break;
                    case LED_COUNT + 5:
                        lampVal |= (1 << 2); // GPB2

                        break;
                    default:
                        break;
                    }
                }
            }
        }

        ws28xx_update();
        buttonIO.writeGPIOB(lampVal);
        getButtonStatus();

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

/**
 * @brief Task to refresh the seven segment displays.
 *
 */
void DisplayController::updateSevenSegDisplaysTask()
{
    ESP_LOGI(TAG, "Update 7-segment display task started");

    //
    uint16_t bank = 0;
    uint16_t credit = 0;
    bool initialRun = true;

    for (;;)
    {
        ESP_LOGD(TAG, "7 seg display loop");

        if (initialRun || (bank != this->moneyController.getBank()))
        {
            bank = this->moneyController.getBank();
            bankDisplay.write_value("%05d", bank);
        }

        if (initialRun || (credit != this->moneyController.getCredit()))
        {
            credit = this->moneyController.getCredit();
            creditDisplay.write_value("%05d", credit);
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
void DisplayController::hsvToRgb(uint8_t h, uint8_t s, uint8_t v,
                                 CRGB& rgb)
{
    h %= 360; // h -> [0,360]
    uint8_t rgb_max = v * 2.55f;
    uint8_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint8_t i = h / 60;
    uint8_t diff = h % 60;

    // RGB adjustment amount by hue
    uint8_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i)
    {
    case 0:
        rgb.r = rgb_max;
        rgb.g = rgb_min + rgb_adj;
        rgb.b = rgb_min;
        break;
    case 1:
        rgb.r = rgb_max - rgb_adj;
        rgb.g = rgb_max;
        rgb.b = rgb_min;
        break;
    case 2:
        rgb.r = rgb_min;
        rgb.g = rgb_max;
        rgb.b = rgb_min + rgb_adj;
        break;
    case 3:
        rgb.r = rgb_min;
        rgb.g = rgb_max - rgb_adj;
        rgb.b = rgb_max;
        break;
    case 4:
        rgb.r = rgb_min + rgb_adj;
        rgb.g = rgb_min;
        rgb.b = rgb_max;
        break;
    default:
        rgb.r = rgb_max;
        rgb.g = rgb_min;
        rgb.b = rgb_max - rgb_adj;
        break;
    }
}

CRGB DisplayController::rgbFromValues(uint8_t red, uint8_t green, uint8_t blue)
{
    CRGB result;
    result.red = red;
    result.green = green;
    result.blue = blue;
    return result;
}

void DisplayController::hsvToRgbRainbow(uint8_t h, uint8_t s, uint8_t v,
                                        CRGB& rgb)
{
}
