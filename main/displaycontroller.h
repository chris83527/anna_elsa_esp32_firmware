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
 * @file displaycontroller.h
 *
 * Definitions and methods for lamps/buttons/displays
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#pragma once

#ifndef DISPLAY_CONTROLLER_H
#define DISPLAY_CONTROLLER_H

#include <array>
#include <string>
#include <thread>

#include "config.h"
#include "ht16k33.h"
//#include "led_strip.h"
#include "FastLED.h"
#include "FX.h"
#include "mcp23x17.h"
#include "moneycontroller.h"
#include "oledcontroller.h"

#define CHASE_SPEED_MS (100)

enum class LampState { off, blinkslow, blinkfast, on };

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p FL_PROGMEM;

class DisplayController
{
public:
    DisplayController(MoneyController& moneyController, I2CManager& i2cmgr);
    ~DisplayController();

    struct lamp_data_t
    {
        CRGB rgb = rgbFromValues(255, 255, 255);
        CRGB activeRgb = rgbFromValues(255, 255, 255);
        LampState lampState = LampState::off;
    };

    esp_err_t initialise();

    void setMoves(uint8_t value);

    void resetLampData();

    std::array<lamp_data_t, LED_COUNT + 6>& getLampData();

    static CRGB rgbFromValues(uint8_t red, uint8_t green, uint8_t blue);
    void FillLEDsFromPaletteColors(uint8_t colorIndex);
    void ChangePalettePeriodically();
    void SetupTotallyRandomPalette();
    void SetupBlackAndWhiteStripedPalette();
    void SetupPurpleAndGreenPalette();
    static void hsvToRgb(uint8_t hue, uint8_t saturation, uint8_t value, CRGB& rgb);
    static void hsvToRgbRainbow(uint8_t hue, uint8_t saturation, uint8_t value, CRGB& rgb);


    static void clearText();
    static void displayVFDText(const std::string& text);
    void scrollOledText(const std::string& text);
    void clearOledDisplay();
    void displayOledText(const std::string& text, int lineNumber, bool invert);
    [[nodiscard]] bool isAttractMode() const;

    uint8_t getButtonStatus();
    [[nodiscard]] uint8_t waitForButton(uint8_t mask) const;

    void beginAttractMode();
    void stopAttractMode();

    HT16K33& getBankDisplay();
    HT16K33& getCreditDisplay();
    HT16K33& getMovesDisplay();

    //led_strip_handle_t& getLedStripHandle();
    MCP23x17& getButtonIO();

public:
    // WS2128B LEDs
    static constexpr int REEL_LAMP_L1 = 0;
    static constexpr int REEL_LAMP_L2 = 1;
    static constexpr int REEL_LAMP_L3 = 2;
    static constexpr int REEL_LAMP_C1 = 3;
    static constexpr int REEL_LAMP_C2 = 4;
    static constexpr int REEL_LAMP_C3 = 5;
    static constexpr int REEL_LAMP_R1 = 6;
    static constexpr int REEL_LAMP_R2 = 7;
    static constexpr int REEL_LAMP_R3 = 8;

    static constexpr int LAMP_NUDGE_5 = 9;
    static constexpr int LAMP_NUDGE_4 = 10;
    static constexpr int LAMP_NUDGE_3 = 11;
    static constexpr int LAMP_NUDGE_2 = 12;
    static constexpr int LAMP_NUDGE_1 = 13;

    static constexpr int LAMP_PRIZE_PALACE = 14;
    static constexpr int LAMP_PRIZE_ANNA = 15;
    static constexpr int LAMP_PRIZE_ELSA = 16;
    static constexpr int LAMP_PRIZE_CHRISTOPH = 17;
    static constexpr int LAMP_PRIZE_SVEN = 18;
    static constexpr int LAMP_PRIZE_OLAF_3 = 19;
    static constexpr int LAMP_PRIZE_OLAF_ANY = 20;
    static constexpr int LAMP_PRIZE_HANS = 21;
    static constexpr int LAMP_PRIZE_20_CENT = 22;
    static constexpr int LAMP_PRIZE_40_CENT = 23;
    static constexpr int LAMP_PRIZE_80_CENT = 24;
    static constexpr int LAMP_PRIZE_120_CENT = 25;
    static constexpr int LAMP_PRIZE_160_CENT = 26;
    static constexpr int LAMP_PRIZE_200_CENT = 27;
    static constexpr int LAMP_PRIZE_300_CENT = 28;
    static constexpr int LAMP_PRIZE_400_CENT = 29;

    static constexpr int LAMP_HI = 30;
    static constexpr int LAMP_lO = 31;

    static constexpr int LAMP_MATRIX_SHUFFLE_1_1 = 32;
    static constexpr int LAMP_MATRIX_FREE_SPIN_1_2 = 33;
    static constexpr int LAMP_MATRIX_DOUBLE_MONEY_1_3 = 34;
    static constexpr int LAMP_MATRIX_PALACE_2_3 = 35;
    static constexpr int LAMP_MATRIX_LOSE_2_2 = 36;
    static constexpr int LAMP_MATRIX_PALACE_2_1 = 37;
    static constexpr int LAMP_MATRIX_FREE_SPIN_3_1 = 38;
    static constexpr int LAMP_MATRIX_SHUFFLE_3_2 = 39;
    static constexpr int LAMP_MATRIX_LOSE_3_3 = 40;
    static constexpr int LAMP_MATRIX_HI_LO_4_2 = 42;
    static constexpr int LAMP_MATRIX_FREE_SPIN_4_3 = 41;
    static constexpr int LAMP_MATRIX_PALACE_4_1 = 43;

    static constexpr int LAMP_TRAIL_20_CENT = 44;
    static constexpr int LAMP_TRAIL_40_CENT = 45;
    static constexpr int LAMP_TRAIL_60_CENT = 46;
    static constexpr int LAMP_TRAIL_80_CENT = 47;
    static constexpr int LAMP_TRAIL_ONE_EURO = 48;
    static constexpr int LAMP_TRAIL_ONE_TWENTY = 49;
    static constexpr int LAMP_TRAIL_ONE_FOURTY = 50;
    static constexpr int LAMP_TRAIL_ONE_SIXTY = 51;
    static constexpr int LAMP_TRAIL_ONE_EIGHTY = 52;
    static constexpr int LAMP_TRAIL_TWO_EURO = 53;
    static constexpr int LAMP_TRAIL_TWO_FOURTY = 54;
    static constexpr int LAMP_TRAIL_TWO_EIGHTY = 55;
    static constexpr int LAMP_TRAIL_THREE_FOURTY = 56;
    static constexpr int LAMP_TRAIL_THREE_EIGHTY = 57;
    static constexpr int LAMP_TRAIL_FOUR_TWENTY = 58;
    static constexpr int LAMP_TRAIL_FOUR_SIXTY = 59;
    static constexpr int LAMP_TRAIL_FIVE_EURO = 60;

    static constexpr int LMP_START = 61;
    static constexpr int LMP_COLLECT = 62;
    static constexpr int LMP_HOLD_LO = 63;
    static constexpr int LMP_HOLD = 64;
    static constexpr int LMP_HOLD_HI = 65;
    static constexpr int LMP_TRANSFER = 66;

    static constexpr int NUDGE_LAMPS_LENGTH = 5;
    static constexpr int FEATURE_LAMPS_LENGTH = 12;
    static constexpr int TRAIL_LAMPS_LENGTH = 17;
    static constexpr int PRIZE_LADDER_LENGTH = 8;

    // static constexpr std::array<int, 0> SEGMENTS;

    static constexpr std::array<int, NUDGE_LAMPS_LENGTH> NUDGE_LAMPS = {
        LAMP_NUDGE_1, LAMP_NUDGE_2, LAMP_NUDGE_3, LAMP_NUDGE_4, LAMP_NUDGE_5,
    };

    static constexpr std::array<int, TRAIL_LAMPS_LENGTH> TRAIL_LAMPS = {
        LAMP_TRAIL_20_CENT, LAMP_TRAIL_40_CENT, LAMP_TRAIL_60_CENT,
        LAMP_TRAIL_80_CENT, LAMP_TRAIL_ONE_EURO, LAMP_TRAIL_ONE_TWENTY,
        LAMP_TRAIL_ONE_FOURTY, LAMP_TRAIL_ONE_SIXTY, LAMP_TRAIL_ONE_EIGHTY,
        LAMP_TRAIL_TWO_EURO, LAMP_TRAIL_TWO_FOURTY, LAMP_TRAIL_TWO_EIGHTY,
        LAMP_TRAIL_THREE_FOURTY, LAMP_TRAIL_THREE_EIGHTY, LAMP_TRAIL_FOUR_TWENTY,
        LAMP_TRAIL_FOUR_SIXTY, LAMP_TRAIL_FIVE_EURO,
    };

    static constexpr std::array<int, FEATURE_LAMPS_LENGTH> FEATURE_LAMPS = {
        LAMP_MATRIX_FREE_SPIN_1_2, LAMP_MATRIX_DOUBLE_MONEY_1_3,
        LAMP_MATRIX_SHUFFLE_1_1, LAMP_MATRIX_LOSE_2_2,
        LAMP_MATRIX_PALACE_2_3, LAMP_MATRIX_PALACE_2_1,
        LAMP_MATRIX_SHUFFLE_3_2, LAMP_MATRIX_LOSE_3_3,
        LAMP_MATRIX_FREE_SPIN_3_1, LAMP_MATRIX_HI_LO_4_2,
        LAMP_MATRIX_FREE_SPIN_4_3, LAMP_MATRIX_PALACE_4_1,
    };

    // Prize ladder
    static constexpr std::array<std::array<int, 2>, 8> PRIZE_LADDER_LAMPS = {
        std::array{LAMP_PRIZE_HANS, LAMP_PRIZE_20_CENT},
        std::array{LAMP_PRIZE_OLAF_3, LAMP_PRIZE_80_CENT},
        std::array{LAMP_PRIZE_OLAF_ANY, LAMP_PRIZE_40_CENT},
        std::array{LAMP_PRIZE_SVEN, LAMP_PRIZE_120_CENT},
        std::array{LAMP_PRIZE_CHRISTOPH, LAMP_PRIZE_160_CENT},
        std::array{LAMP_PRIZE_ELSA, LAMP_PRIZE_200_CENT},
        std::array{LAMP_PRIZE_ANNA, LAMP_PRIZE_300_CENT},
        std::array{LAMP_PRIZE_PALACE, LAMP_PRIZE_400_CENT},
    };

    static constexpr int MAX_BRIGHTNESS = 100;

    /*

  Row/Column	0               1               2               3 4	5
  6	7	8           9       10          11      12  13	14	15 0
  Shuffle (t)     Palace (top)	Free Spin (t)	Palace (top left)
  -	-	-	-	20ct        40ct	60ct        80ct -   -
  -	- 1           Dbl Money (b)	Palace (bottom)	Lose (b) Free Spin
  (bottom)	-	-	-	-	€1          €1,20 €1,40 €1,60 -
  -	-	- 2           Free Spin (c)	Lose (centre)	Shuffle (c) Hi /
  Lo (centre)	-	-	-	-	€1,80       €2,00 €2,40 €2,80 -
  -	-	- 3           -               -               Nudge 5 Nudge 4 -
  -	-	-	Nudge 3     Nudge 2	Nudge 1     -       -   -
  -	- 4           -               -               -               - -
  -	-	-	-           -       -           -       -   - -	- 5 - -
  -               -                   -	-	-	-	- -       - - -
  -	-	- 6           -               Start           Cancel/Collect
  -                   -	-	-	-	Hold / Hi	Hold
  Hold / Lo	Xfer	-	-	-	- 7           -               -
  -               -                   -	-	-	-	- -       - - -
  -	-	-
     */

protected:
private:
    [[noreturn]] void attractModeTask();
    [[noreturn]] void updateSevenSegDisplaysTask();
    [[noreturn]] void updateLampsTask();
    [[noreturn]] void blinkLampsTask();

    void rainbowEffect();
    void chaseEffect();
    void fadeInOutEffect();

private:
    HT16K33 movesDisplay;
    HT16K33 creditDisplay;
    HT16K33 bankDisplay;

    MCP23x17 buttonIO;

    CRGB ws2812_buffer[LED_COUNT];
    CRGBPalette16 currentPalette;
    TBlendType    currentBlending;


    WS2812FX ws2812fx;

    MoneyController& moneyController;
    OledController oledController;

    uint8_t buttonStatus{};
    bool doorOpen{};

    std::array<lamp_data_t, LED_COUNT + 6> lampData;

    void testLamps();

    uint8_t keyStatus{};

    bool attractMode = false;

    // Arrangement for display
    // )
    //               a = A6
    //             _________
    //            |         |
    //   f = A1   |  g = A0 | b = A5
    //            |_________|
    //            |         |
    //   e = A2   |         | c = A4
    //            |_________|
    //               d = A3

    std::thread attractModeThread;
    std::thread updateSevenSegDisplaysThread;
    std::thread updateLampsThread;
    std::thread blinkLampsThread;

    I2CManager& i2cManager;
};

#endif
