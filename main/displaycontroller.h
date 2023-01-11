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
#include <cstddef>
#include <bitset>

#include "freertos/task.h"

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


enum class LampState {off, blinkslow, blinkfast, on};   

class LampData {
public:
    rgb_t rgb;
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

    LampData* getLampData(void);
        
    void clearText(void);
    void displayText(const std::string &text);
    
    bool isAttractMode();           

    uint8_t getButtonStatus(void);

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
    
    TaskHandle_t attractModeTaskHandle;

    const static uint8_t NUDGE_LAMPS_LENGTH = 6;
    const static uint8_t FEATURE_LAMPS_LENGTH = 12;
    const static uint8_t TRAIL_LAMPS_LENGTH = 17;

    const static uint8_t NUDGE_LAMPS[NUDGE_LAMPS_LENGTH];
    const static uint8_t SEGMENTS[];
    const static uint8_t FEATURE_LAMPS[FEATURE_LAMPS_LENGTH];
    const static uint8_t TRAIL_LAMPS[TRAIL_LAMPS_LENGTH];

    

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

    LampData lampData[LED_COUNT + 6];       

    MainController *mainController;

    void testLamps(void);
    uint8_t keyStatus;

    uint32_t lampDataNew[LED_COUNT + 6];

    bool attractMode;             
 
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



    static void attractModeTask(void *pvParameters);
    static void updateSevenSegDisplaysTask(void *pvParameters);
    static void updateLampsTask(void *pvParameters);
};



#endif // __DISPLAYCONTROLLER_H__