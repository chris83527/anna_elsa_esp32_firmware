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
 * @file reelcontroller.h
 *
 * Definitions for reel controller board
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#pragma once

#ifndef __REELS_H__
#define __REELS_H__

#include <chrono>

#include "driver/ledc.h"
#include "pca9629a.h"

#include "audiocontroller.h"
#include "displaycontroller.h"

#define REEL_LEFT (1 << 0)
#define REEL_CENTRE (1 << 1)
#define REEL_RIGHT (1 << 2)

#define GPIO_MOTOR_A_PLUS (1 << 0)
#define GPIO_MOTOR_A_MINUS (1 << 1)
#define GPIO_MOTOR_B_MINUS (1 << 2)
#define GPIO_MOTOR_B_PLUS (1 << 3)
#define GPIO_PHOTO_INTERRUPTER 4

#define STATUS_OK 0
#define STATUS_ERR_REEL_OPTIC 1
#define STATUS_INITIAL 255

#define STEPS_PER_STOP 8 // half-step drive

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY_QUARTER (256)        // Set duty to 25% ((2**10)-1) * 25%
#define LEDC_DUTY_FULL (1023) // Set duty to 100%. ((2 ** 10) - 1) * 100%  = 8191

#define LEDC_FREQUENCY (60)   // Frequency in Hertz. Set frequency to 200Hz

class MainController;

class ReelController
{
public:
    ReelController(MainController* mainContoller, I2CBus& bus);
    ~ReelController();

    struct reel_stop_info_t
    {
        uint8_t leftStop = 0;
        uint8_t centreStop = 0;
        uint8_t rightStop = 0;
    };

    bool reelLeftInitOk{};
    bool reelCentreInitOk{};
    bool reelRightInitOk{};

    bool initialise();

    void spin(uint8_t leftSymbol, uint8_t midStop,
              uint8_t rightSymbol);
    void nudge(uint8_t leftStop, uint8_t midStop,
               uint8_t rightStop);
    void shuffle(uint8_t leftStop, uint8_t midStop,
                 uint8_t rightStop);

    [[nodiscard]] reel_stop_info_t getReelStopInfo() const;

    [[nodiscard]] bool isCommandInProgress() const;

    void calibrate();
    void test();

private:
    const int MAX_STOPS = 25; // total number of stops (i.e. symbols)

    reel_stop_info_t reelStopInfo;

    uint8_t status{};
    bool commandInProgress{};

    MainController* mainController;

    pca9629a::Driver leftReel;
    pca9629a::Driver centreReel;
    pca9629a::Driver rightReel;

};

#endif /* __REELS_H__ */
