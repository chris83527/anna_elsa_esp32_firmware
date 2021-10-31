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
 * @file reelcontroller.h
 *
 * Definitions for reel controller board
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __REELS_H__
#define __REELS_H__

#include <driver/gpio.h>
#include <driver/ledc.h>

#include "maincontroller.h"

#define I2C_FREQ_HZ 100000

#define GPIO_MOTOR_PWM_EN GPIO_NUM_23

#define REEL_LEFT (1 << 0)
#define REEL_CENTRE (1 << 1)
#define REEL_RIGHT (1 << 2)

#define GPIO_MOTOR_1 0
#define GPIO_MOTOR_2 1
#define GPIO_MOTOR_3 2
#define GPIO_MOTOR_4 3
#define GPIO_PHOTO_INTERRUPTER 4

#define STATUS_OK 0
#define STATUS_ERR_REEL_OPTIC 1

class ReelController {
public:
    ReelController(MainController *mainController);
    ReelController(const ReelController &orig);

    typedef struct {
        int stop; // 1 - 25
        int status;
        uint8_t step_data;
        int step; // 0 - 3 - index into stepper output configuration
        bool photo_interrupter_set;
    } reel_status_data_t;

    typedef enum {
        Clockwise = 0,
        CounterClockwise,
    } direction_t;

    esp_err_t initialise(void);

    void move(int reels, direction_t dir_left, direction_t dir_centre, direction_t dir_right);

    void spin(const uint8_t leftStops, const uint8_t midStops, const uint8_t rightStops);
    void nudge(const uint8_t leftStops, const uint8_t midStops, const uint8_t rightStops);
    void shuffle(const uint8_t leftStops, const uint8_t midStops, const uint8_t rightStops);

    void hold(void);
    bool isHeld(void);

    void getReelPositions(uint8_t & lefPos, uint8_t & centrePos, uint8_t & rightPos);
    reel_status_data_t getReelStatus(uint8_t reel);

    bool isCommandInProgress(void);
    uint8_t getStatus(void);

private:
    bool _isHeld = false;
    int _position;

    uint8_t _reelPosLeft = 0;
    uint8_t _reelPosCentre = 0;
    uint8_t _reelPosRight = 0;

    int _leftState = 0;
    int _midState = 0;
    int _rightState = 0;

    const int MAX_STATES = 4;
    const int MAX_STEPS = 25;

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer;
    ledc_channel_config_t ledc_channel;

    reel_status_data_t reel_status_data_left;
    reel_status_data_t reel_status_data_centre;
    reel_status_data_t reel_status_data_right;

    uint8_t status;
    bool commandInProgress;

    MainController* mainController;

    void spinToZero(void);
};

#endif /* __WAVE_H__ */
