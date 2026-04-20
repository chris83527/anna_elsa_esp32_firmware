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
#include <string>
#include <vector>

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_pthread.h"
#include "freertos/FreeRTOS.h"

#include "config.h"

#include "ht16k33.h"
#include "m20ly02z.h"
#include "mcp23x17.h"

#include "displaycontroller.h"
#include "game.h"
#include "paymentcontroller.h"

#include "ws2812b.hpp"
#include "lib8tion/random8.h"


static const char* TAG = "DisplayController";
static std::string vfdText;

DisplayController::DisplayController(PaymentController& paymentController,
                                     I2CBus& i2c_bus) : movesDisplay(i2c_bus, MOVES_DISPLAY_ADDRESS),
                                                        creditDisplay(i2c_bus, CREDIT_DISPLAY_ADDRESS),
                                                        bankDisplay(i2c_bus, BANK_DISPLAY_ADDRESS),
                                                        buttonIO(i2c_bus, BUTTONS_I2C_ADDRESS),
                                                        oledController(i2c_bus, 0x3c),
                                                        strip(LED_GPIO, LED_COUNT), currentBlending(),
                                                        paymentController(paymentController), buttonStatus(0),
                                                        doorOpen(false)
{
    ESP_LOGD(TAG, "Entering constructor");

    // Perform this here so we have debug output
    oledController.initialise();

    attractMode = false;

    ESP_LOGD(TAG, "Leaving constructor");
}

DisplayController::~DisplayController() = default;

esp_err_t DisplayController::initialise()
{
    ESP_LOGI(TAG, "Entering DisplayController::initialise()");

    buttonStatus = 0;

    currentPalette = RainbowColors_p;
    currentBlending = LINEARBLEND;


    strip.init();

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
    buttonIO.setGPIOAInputPolarity(0xff); // Invert polarity (bit reflects the opposite logic state of the
    // input pin)
    ESP_LOGI(TAG, "Button interface initialisation succeeded");

    resetLampData();

    // Blink
    esp_timer_create_args_t blinkLampsTimerArgs = {
        .callback = &blinkLampsCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "BLink Lamps Timer",
        .skip_unhandled_events = true,
    };
    esp_timer_handle_t blinkLampsTimerHandler;
    ESP_ERROR_CHECK(esp_timer_create(&blinkLampsTimerArgs, &blinkLampsTimerHandler));
    ESP_ERROR_CHECK(esp_timer_start_periodic(blinkLampsTimerHandler, 100000)); // 100ms

    // Set up a timer to update the lamps every 5 seconds
    esp_timer_create_args_t updateLampsTimerArgs = {
        .callback = &updateLampsCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "Update Lamps Timer",
        .skip_unhandled_events = true,
    };
    esp_timer_handle_t updateLampsTimerHandler;
    ESP_ERROR_CHECK(esp_timer_create(&updateLampsTimerArgs, &updateLampsTimerHandler));
    ESP_ERROR_CHECK(esp_timer_start_periodic(updateLampsTimerHandler, 50000)); // 30ms

    testLamps();

    attractMode = false;
    auto cfg = esp_pthread_get_default_config();
    cfg.thread_name = "AttractMode";
    esp_pthread_set_cfg(&cfg);
    attractModeThread = std::thread([&]() { attractModeTask(); });
    attractModeThread.detach();

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
        lampData[i].lampState = LampState::off;
        // set lamp colour to white
        lampData[i].rgb = CRGB(
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

bool DisplayController::isAttractMode() { return attractMode; }

void DisplayController::testLamps()
{
    ESP_LOGD(TAG, "Entering testLamps()");
    // initialise lamps
    // switch all LEDs on;
    resetLampData();
    for (int i = 0; i < (LED_COUNT); i++)
    {
        lampData[i].lampState = LampState::on;
        lampData[i].rgb = CRGB(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS);
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
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
    esp_err_t err = buttonIO.readGPIOA(buttonStatus);

    if (err == ESP_OK)
    {
        if ((buttonStatus & (1 << BTN_DOOR)) == 0)
        {
            if (!doorOpen)
            {
                ESP_LOGI(TAG, "Door open!");
            }
            doorOpen = true;
        }
        else
        {
            if (doorOpen)
            {
                ESP_LOGI(TAG, "Door closed!");
            }
            doorOpen = false;
        }
    }
    else
    {
        buttonStatus = 0;
        ESP_LOGE(TAG, "An error occurred getting button status");
        // esp_backtrace_print
    }

    return buttonStatus;
}

uint8_t DisplayController::waitForButton(const uint8_t mask)
{
    // loop waiting for button press.
    while ((mask & buttonStatus) == 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return buttonStatus;
}

void DisplayController::displayVFDText(const std::string& text)
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

MCP23x17& DisplayController::getButtonIO() { return buttonIO; }

HT16K33& DisplayController::getBankDisplay() { return bankDisplay; }

HT16K33& DisplayController::getCreditDisplay() { return creditDisplay; }

HT16K33& DisplayController::getMovesDisplay() { return movesDisplay; }

void DisplayController::attractModeTask()
{
    ESP_LOGI(TAG, "Attract mode thread started");
    static uint8_t startIndex = 0;
    while (true)
    {
        if (isAttractMode())
        {
            ChangePalettePeriodically();

            startIndex++; /* motion speed */
            FillLEDsFromPaletteColors(startIndex);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            trail();
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
            lampData[j].rgb = CRGB(i, i, i);
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
            lampData[j].rgb = CRGB(i, i, i);
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
            lampData[j].rgb = CRGB(i, i, i);
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
            lampData[j].rgb = CRGB(i, i, i);
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
            lampData[j].rgb = CRGB(i, i, i);
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
            lampData[j].rgb = CRGB(i, i, i);
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
            if (attractMode != true)
            {
                return;
            }
            lampData[i].rgb =
                CRGB(0, MAX_BRIGHTNESS, MAX_BRIGHTNESS);
            lampData[i].lampState = LampState::on;
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }

        for (int i : TRAIL_LAMPS)
        {
            if (attractMode != true)
            {
                return;
            }
            lampData[i].lampState = (LampState::off);
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    }
}

void DisplayController::blinkLampsCallback(void* param)
{
    DisplayController* displayController = static_cast<DisplayController*>(param);

    static int state = 0;
    ESP_LOGD(TAG, "blink lamp timer called");
    switch (state)
    {
    case 0:
        for (int i = 0; i < (LED_COUNT + 6); i++)
        {
            if (displayController->lampData[i].lampState == LampState::on ||
                displayController->lampData[i].lampState == LampState::blinkfast ||
                displayController->lampData[i].lampState == LampState::blinkslow)
            {
                if (i < LED_COUNT)
                {
                    displayController->lampData[i].activeRgb = displayController->lampData[i].rgb;
                }
                else
                {
                    // non- RGB button lamps
                    displayController->lampData[i].activeRgb =
                        CRGB(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS);
                }
            }
            else
            {
                // Set active rgb value to 0 (off or black)
                displayController->lampData[i].activeRgb = CRGB(0, 0, 0);
            }
        }
        break;
    case 1:
        for (int i = 0; i < (LED_COUNT + 6); i++)
        {
            if (displayController->lampData[i].lampState == LampState::on ||
                displayController->lampData[i].lampState == LampState::blinkslow)
            {
                if (i < LED_COUNT)
                {
                    displayController->lampData[i].activeRgb = displayController->lampData[i].rgb;
                }
                else
                {
                    // non- RGB button lamps
                    displayController->lampData[i].activeRgb =
                        CRGB(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS);
                }
            }
            else
            {
                // Set active rgb value to 0 (off or black)
                displayController->lampData[i].activeRgb = CRGB(0, 0, 0);
            }
        }
        break;
    case 2:
        for (int i = 0; i < LED_COUNT + 6; i++)
        {
            if (displayController->lampData[i].lampState == LampState::on)
            {
                if (i < LED_COUNT)
                {
                    displayController->lampData[i].activeRgb = displayController->lampData[i].rgb;
                }
                else
                {
                    // non- RGB button lamps
                    displayController->lampData[i].activeRgb =
                        CRGB(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS);
                }
            }
            else
            {
                // Set active rgb value to 0 (off or black)
                displayController->lampData[i].activeRgb = CRGB(0, 0, 0);
            }
        }
        break;
    case 3:
        for (int i = 0; i < (LED_COUNT + 6); i++)
        {
            if (displayController->lampData[i].lampState == LampState::on ||
                displayController->lampData[i].lampState == LampState::blinkfast)
            {
                if (i < LED_COUNT)
                {
                    displayController->lampData[i].activeRgb = displayController->lampData[i].rgb;
                }
                else
                {
                    // non- RGB button lamps
                    displayController->lampData[i].activeRgb =
                        CRGB(MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS);
                }
            }
            else
            {
                // Set active rgb value to 0 (off or black)
                displayController->lampData[i].activeRgb = CRGB(0, 0, 0);
            }
        }
        break;
    }

    // if we have completed all four states (0-3).. reset state to zero and start again
    if (state == 4)
    {
        state = 0;
    }
}

void DisplayController::updateLampsCallback(void* param)
{
    ESP_LOGD(TAG, "Update Lamps Callback called");

    DisplayController* displayController = static_cast<DisplayController*>(param);

    static uint8_t lampVal = 0;
    static uint16_t bank = 0;
    static uint16_t credit = 0;
    static bool initialRun = true;

    // set leds
    lampVal = 0;
    for (int i = 0; i < LED_COUNT + 6; i++)
    {
        if (i < LED_COUNT)
        {
            displayController->strip.set_pixel(i, displayController->lampData[i].activeRgb);
        }
        else
        {
            // GPB1 and GPB0 are unconnected
            // activeRgb value must be have have at least one channel (r, g or b)
            // with a positive value to light
            if ((displayController->lampData[i].activeRgb.r > 0 ||
                    displayController->lampData[i].activeRgb.g > 0 ||
                    displayController->lampData[i].activeRgb.b > 0) &&
                displayController->lampData[i].lampState == LampState::on)
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
                }
            }
        }
    }

    displayController->strip.show();
    displayController->buttonIO.writeGPIOB(lampVal);
    displayController->getButtonStatus();

    if (initialRun || bank != displayController->paymentController.getBank())
    {
        bank = displayController->paymentController.getBank();
        displayController->bankDisplay.write_value("%05d", bank);
    }

    if (initialRun || credit != displayController->paymentController.getCredit())
    {
        credit = displayController->paymentController.getCredit();
        displayController->creditDisplay.write_value("%05d", credit);
    }

    initialRun = false;
}

void DisplayController::FillLEDsFromPaletteColors(uint8_t colorIndex)
{
    uint8_t brightness = MAX_BRIGHTNESS;

    for (int i = 0; i < LED_COUNT; ++i)
    {
        lampData[i].lampState = LampState::on;
        lampData[i].rgb = colorFromPalette(currentPalette, colorIndex, brightness, currentBlending);
        lampData[i].activeRgb = lampData[i].rgb;
        colorIndex += 3;
    }
}

void DisplayController::ChangePalettePeriodically()
{
    uint8_t secondHand = (millis() / 1000) % 60;
    static uint8_t lastSecond = 99;

    if (lastSecond != secondHand)
    {
        lastSecond = secondHand;
        if (secondHand == 0)
        {
            displayVFDText("  WOODS AMUSEMENTS  ");
            currentPalette = RainbowColors_p;
            currentBlending = LINEARBLEND;
        }
        if (secondHand == 10)
        {
            displayVFDText("      PRESENTS      ");
            currentPalette = RainbowStripeColors_p;
            currentBlending = NOBLEND;
        }
        if (secondHand == 15)
        {
            displayVFDText("       FROZEN       ");
            currentPalette = RainbowColors_p;
            currentBlending = LINEARBLEND;
        }
        if (secondHand == 20)
        {
            SetupPurpleAndGreenPalette();
            currentBlending = LINEARBLEND;
        }
        if (secondHand == 25)
        {
            displayVFDText("     20CT GAME      ");

            SetupTotallyRandomPalette();
            currentBlending = LINEARBLEND;
        }
        if (secondHand == 30)
        {
            displayVFDText("    INSERT COINS    ");
            SetupBlackAndWhiteStripedPalette();
            currentBlending = NOBLEND;
        }
        if (secondHand == 35)
        {
            SetupBlackAndWhiteStripedPalette();
            currentBlending = LINEARBLEND;
        }
        if (secondHand == 40)
        {
            currentPalette = CloudColors_p;
            currentBlending = LINEARBLEND;
        }
        if (secondHand == 45)
        {
            currentPalette = PartyColors_p;
            currentBlending = LINEARBLEND;
        }
        if (secondHand == 50)
        {
            currentPalette = myRedWhiteBluePalette_p;
            currentBlending = NOBLEND;
        }
        if (secondHand == 55)
        {
            currentPalette = myRedWhiteBluePalette_p;
            currentBlending = LINEARBLEND;
        }
    }
}

// This function fills the palette with totally random colors.
void DisplayController::SetupTotallyRandomPalette()
{
    for (int i = 0; i < 16; ++i)
    {
        currentPalette.colors[i] = hsv2rgb(CHSV(frand::random8(), 255, frand::random8()));
    }
}

// This function sets up a palette of black and white stripes,
// using code.  Since the palette is effectively an array of
// sixteen CRGB colors, the various fill_* functions can be used
// to set them up.
void DisplayController::SetupBlackAndWhiteStripedPalette()
{
    // 'black out' all 16 palette entries...
    for (int i = 0; i < 16; i++)
    {
        currentPalette.colors[i] = Colour::Black;
    }

    // and set every fourth one to white.
    currentPalette.colors[0] = Colour::White;
    currentPalette.colors[4] = Colour::White;
    currentPalette.colors[8] = Colour::White;
    currentPalette.colors[12] = Colour::White;
}

// This function sets up a palette of purple and green stripes.
void DisplayController::SetupPurpleAndGreenPalette()
{
    CRGB purple = hsv2rgb(CHSV(CHSV::HUE_PURPLE, 255, 255));
    CRGB green = hsv2rgb(CHSV(CHSV::HUE_GREEN, 255, 255));
    CRGB black = Colour::Black;

    currentPalette = CRGBPalette16({
        green, green, black, black,
        purple, purple, black, black,
        green, green, black, black,
        purple, purple, black, black
    });
}

const CRGBPalette16 myRedWhiteBluePalette_p =
{
    Colour::Red,
    Colour::Gray, // 'white' is too bright compared to red and blue
    Colour::Blue,
    Colour::Black,

    Colour::Red,
    Colour::Gray,
    Colour::Blue,
    Colour::Black,

    Colour::Red,
    Colour::Red,
    Colour::Gray,
    Colour::Gray,
    Colour::Blue,
    Colour::Blue,
    Colour::Black,
    Colour::Black
};

void DisplayController::trail()
{
    uint8_t hue = 0;

    for (int j = 0; j < 5; j++)
    {
        strip.fadeToBlackBy(20); // fade old pixels
        strip.set_pixel(0, hsv2rgb(CHSV(hue, 255, 255)));
        strip.show();

        hue += 4;

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

uint32_t DisplayController::millis()
{
    return esp_timer_get_time() / 1000;
}
