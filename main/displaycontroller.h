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
 * @file displaycontroller.h
 *
 * Definitions and methods for lamps/buttons/displays
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __DISPLAYCONTROLLER_H__
#define __DISPLAYCONTROLLER_H__

#include <stdbool.h>
#include <stdint.h>
#include <string>
#include <array>
#include <cstddef>
#include <bitset>
#include <thread>

#include "freertos/task.h"
#include "esp_pthread.h"

#include "led_strip.h"
#include "driver/rmt.h"
#include "mcp23x17.h"
#include "m20ly02z.h"
#include "ht16k33.h"
#include "color.h"
#include "config.h"


#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define CHASE_SPEED_MS (100)

class MainController;

enum class LampState {
    off, blinkslow, blinkfast, on
};

class LampData {
public:
    rgb_t rgb;
    rgb_t activeRgb;
    LampState lampState;
private:

};

class DisplayController {
public:
    DisplayController(MainController *mainController);
    DisplayController(const DisplayController &orig);

    esp_err_t initialise(void);

    void setMoves(uint8_t value);

    void resetLampData();

    std::array<LampData, LED_COUNT + 6 > getLampData(void);

    void clearText(void);
    void displayText(const std::string &text);

    bool isAttractMode();

    uint8_t getButtonStatus(void);
    uint8_t waitForButton(uint8_t mask);

    void test();
    void test2(bool pauseBetweenLamps);

    void ledStripHsv2rgb(uint8_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b);

    void beginAttractMode(void);
    void stopAttractMode(void);

    ht16k33_t* getBankDisplay(void);
    ht16k33_t* getCreditDisplay(void);
    ht16k33_t* getMovesDisplay(void);

    led_strip_t* getLedStrip(void);
    mcp23x17_t* getButtonIO(void);


    static const int NUDGE_LAMPS_LENGTH = 5;
    static const int FEATURE_LAMPS_LENGTH = 12;
    static const int TRAIL_LAMPS_LENGTH = 17;

    // WS2128B LEDs
    static const int REEL_LAMP_L1 = 0;
    static const int REEL_LAMP_L2 = 1;
    static const int REEL_LAMP_L3 = 2;
    static const int REEL_LAMP_C1 = 3;
    static const int REEL_LAMP_C2 = 4;
    static const int REEL_LAMP_C3 = 5;
    static const int REEL_LAMP_R1 = 6;
    static const int REEL_LAMP_R2 = 7;
    static const int REEL_LAMP_R3 = 8;

    static const int LAMP_NUDGE_5 = 9;
    static const int LAMP_NUDGE_4 = 10;
    static const int LAMP_NUDGE_3 = 11;
    static const int LAMP_NUDGE_2 = 12;
    static const int LAMP_NUDGE_1 = 13;

    static const int LAMP_PRIZE_PALACE = 14;
    static const int LAMP_PRIZE_ANNA = 15;
    static const int LAMP_PRIZE_ELSA = 16;
    static const int LAMP_PRIZE_CHRISTOPH = 17;
    static const int LAMP_PRIZE_SVEN = 18;
    static const int LAMP_PRIZE_OLAF_3 = 19;
    static const int LAMP_PRIZE_OLAF_ANY = 20;
    static const int LAMP_PRIZE_HANS = 21;
    static const int LAMP_PRIZE_20_CENT = 22;
    static const int LAMP_PRIZE_40_CENT = 23;
    static const int LAMP_PRIZE_80_CENT = 24;
    static const int LAMP_PRIZE_120_CENT = 25;
    static const int LAMP_PRIZE_160_CENT = 26;
    static const int LAMP_PRIZE_200_CENT = 27;
    static const int LAMP_PRIZE_300_CENT = 28;
    static const int LAMP_PRIZE_400_CENT = 29;

    static const int LAMP_HI = 30;
    static const int LAMP_lO = 31;

    static const int LAMP_MATRIX_SHUFFLE_1_1 = 32;
    static const int LAMP_MATRIX_FREE_SPIN_1_2 = 33;
    static const int LAMP_MATRIX_DOUBLE_MONEY_1_3 = 34;
    static const int LAMP_MATRIX_PALACE_2_3 = 35;
    static const int LAMP_MATRIX_LOSE_2_2 = 36;
    static const int LAMP_MATRIX_PALACE_2_1 = 37;
    static const int LAMP_MATRIX_FREE_SPIN_3_1 = 38;
    static const int LAMP_MATRIX_SHUFFLE_3_2 = 39;
    static const int LAMP_MATRIX_LOSE_3_3 = 40;
    static const int LAMP_MATRIX_HI_LO_4_2 = 42;
    static const int LAMP_MATRIX_FREE_SPIN_4_3 = 41;
    static const int LAMP_MATRIX_PALACE_4_1 = 43;

    static const int LAMP_TRAIL_20_CENT = 44;
    static const int LAMP_TRAIL_40_CENT = 45;
    static const int LAMP_TRAIL_60_CENT = 46;
    static const int LAMP_TRAIL_80_CENT = 47;
    static const int LAMP_TRAIL_ONE_EURO = 48;
    static const int LAMP_TRAIL_ONE_TWENTY = 49;
    static const int LAMP_TRAIL_ONE_FOURTY = 50;
    static const int LAMP_TRAIL_ONE_SIXTY = 51;
    static const int LAMP_TRAIL_ONE_EIGHTY = 52;
    static const int LAMP_TRAIL_TWO_EURO = 53;
    static const int LAMP_TRAIL_TWO_FOURTY = 54;
    static const int LAMP_TRAIL_TWO_EIGHTY = 55;
    static const int LAMP_TRAIL_THREE_FOURTY = 56;
    static const int LAMP_TRAIL_THREE_EIGHTY = 57;
    static const int LAMP_TRAIL_FOUR_TWENTY = 58;
    static const int LAMP_TRAIL_FOUR_SIXTY = 59;
    static const int LAMP_TRAIL_FIVE_EURO = 60;

    static const int LMP_START = 61;
    static const int LMP_COLLECT = 62;
    static const int LMP_HOLD_LO = 63;
    static const int LMP_HOLD = 64;
    static const int LMP_HOLD_HI = 65;
    static const int LMP_TRANSFER = 66;

    static std::array<int, 0> SEGMENTS;
    static std::array<int, NUDGE_LAMPS_LENGTH> NUDGE_LAMPS;
    static std::array<int, TRAIL_LAMPS_LENGTH> TRAIL_LAMPS;
    static std::array<int, FEATURE_LAMPS_LENGTH> FEATURE_LAMPS;




    /*

Row/Column	0               1               2               3                   4	5	6	7	8           9       10          11      12  13	14	15
0           Shuffle (t)     Palace (top)	Free Spin (t)	Palace (top left)	-	-	-	-	20ct        40ct	60ct        80ct    -   -	-	-
1           Dbl Money (b)	Palace (bottom)	Lose (b)        Free Spin (bottom)	-	-	-	-	€1          €1,20	€1,40       €1,60   -   -	-	-
2           Free Spin (c)	Lose (centre)	Shuffle (c)     Hi / Lo (centre)	-	-	-	-	€1,80       €2,00	€2,40       €2,80   -   -	-	-
3           -               -               Nudge 5         Nudge 4             -	-	-	-	Nudge 3     Nudge 2	Nudge 1     -       -   -	-	-
4           -               -               -               -                   -	-	-	-	-           -       -           -       -   -	-	-
5           -               -               -               -                   -	-	-	-	-           -       -           -       -   -	-	-
6           -               Start           Cancel/Collect	-                   -	-	-	-	Hold / Hi	Hold	Hold / Lo	Xfer	-	-	-	-
7           -               -               -               -                   -	-	-	-	-           -       -           -       -   -	-	-
     */


protected:
private:
    led_strip_t ledStrip;

    ht16k33_t movesDisplay;
    ht16k33_t creditDisplay;
    ht16k33_t bankDisplay;

    mcp23x17_t buttonIO;

    uint8_t buttonStatus;
    bool doorOpen;

    std::array<LampData, LED_COUNT + 6 > lampData;

    MainController *mainController;

    void testLamps(void);

    uint8_t keyStatus;

    //uint32_t lampDataNew[LED_COUNT + 6];

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
    void attractModeTask(void);
    std::thread updateSevenSegDisplaysThread;
    void updateSevenSegDisplaysTask(void);
    std::thread updateLampsThread;
    void updateLampsTask(void);
    std::thread blinkLampsThread;
    void blinkLampsTask(void);

    void rainbowEffect();
    void chaseEffect();
    void fadeInOutEffect();
};



#endif // __DISPLAYCONTROLLER_H__