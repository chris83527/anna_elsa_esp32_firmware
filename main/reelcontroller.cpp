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
 * 3. Neither the name of the copyright holder nor the names of its contributors
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
 * @file reelcontroller.cpp
 *
 * Higher level routines for controlling MCP23008 based reel driver board
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <chrono>
#include <cstdio>
#include <cstring>
#include <bitset>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "hal/ledc_types.h"
#include "pca9629a.h"

#define FASTLED_ESP32_I2S
#define I2S_DEVICE 1
#define FASTLED_ESP32_I2S_NUM_DMA_BUFFERS 4

#include "FastLED.h"
#include "lib8tion.h"

#include "audiocontroller.h"
#include "config.h"
#include "displaycontroller.h"
#include "esp_pthread.h"
#include "game.h"
#include "reelcontroller.h"

static const char* TAG = "ReelController";

bool reelLeftInitOk;
bool reelCentreInitOk;
bool reelRightInitOk;

ReelController::ReelController(AudioController& audioController,
                               DisplayController& displayController,
                               I2CManager& i2cmgr)
    : audioController(audioController),
      displayController(displayController),
      leftReel(PCA9629A(i2cmgr, REEL_LEFT_I2C_ADDRESS)),
      centreReel(PCA9629A(i2cmgr, REEL_CENTRE_I2C_ADDRESS)),
      rightReel(PCA9629A(i2cmgr, REEL_RIGHT_I2C_ADDRESS))
{
    ESP_LOGD(TAG, "Entering constructor");

    ESP_LOGD(TAG, "Leaving constructor");
}

ReelController::~ReelController() = default;

bool ReelController::isCommandInProgress() const { return commandInProgress; }

ReelController::reel_stop_info_t ReelController::getReelStopInfo() const
{
    return reelStopInfo;
}

bool ReelController::initialise()
{
    ESP_LOGI(TAG, "ReelController::initialise() called");

    // MOTOR_EN is on a GPIO
    esp_rom_gpio_pad_select_gpio(GPIO_MOTOR_EN);
    // Set the GPIO as a push/pull output
    gpio_set_direction(GPIO_MOTOR_EN, GPIO_MODE_OUTPUT);

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 5000Hz
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };

    ledc_channel_config_t ledc_channel = {
        .gpio_num = GPIO_MOTOR_EN,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE,
        .flags = {},
    };
    // Prepare and then apply the LEDC PWM timer configuration
    if (ledc_timer_config(&ledc_timer) != ESP_OK)
    {
        ESP_LOGE(TAG, "An error occurred initialising PWM subsystem for reels "
                 "(timer config)");
        return false;
    }

    // Prepare and then apply the LEDC PWM channel configuration
    if (ledc_channel_config(&ledc_channel) != ESP_OK)
    {
        ESP_LOGE(TAG, "An error occurred initialising PWM subsystem for reels "
                 "(channel config)");
        return false;
    }

    ledc_bind_channel_timer(LEDC_MODE, LEDC_CHANNEL, LEDC_TIMER);

    reelLeftInitOk = false;
    reelCentreInitOk = false;
    reelRightInitOk = false;

    this->leftReel.initialise();
    this->centreReel.initialise();
    this->rightReel.initialise();

    // Switch on
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FULL);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    // return ESP_OK; // DEBUG

    this->leftReel.start(PCA9629A::Direction::CW, 75, 1); // 3x complete turn
    this->centreReel.start(PCA9629A::Direction::CW, 50, 1); // 2x complete turn
    this->rightReel.start(PCA9629A::Direction::CW, 25, 1); // 1x complete turn

    for (int i = 0; i < 100; i++)
    {
        if (leftReel.isStopped() && centreReel.isStopped() &&
            rightReel.isStopped())
        {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    this->leftReel.home(PCA9629A::Direction::CW); // return to home
    this->centreReel.home(PCA9629A::Direction::CW); // return to home
    this->rightReel.home(PCA9629A::Direction::CW); // return to home

    // Wait for reels to stop (max 10 seconds)
    for (int i = 0; i < 20; i++)
    {
        if (leftReel.isStopped() && centreReel.isStopped() &&
            rightReel.isStopped())
        {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_QUARTER);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    // calibrate();
    // test();

    return true;
}

void ReelController::spin(const uint8_t leftStop, const uint8_t centreStop,
                          const uint8_t rightStop)
{
    this->commandInProgress = true;

    ESP_LOGI(TAG, "spin called: left stop: %d, centre stop: %d, right stop: %d",
             leftStop, centreStop, rightStop);

    if (leftStop > 0)
        this->reelStopInfo.leftStop = leftStop;
    if (centreStop > 0)
        this->reelStopInfo.centreStop = centreStop;
    if (rightStop > 0)
        this->reelStopInfo.rightStop = rightStop;

    uint8_t leftSymbolId = Game::symbolsLeftReel[this->reelStopInfo.leftStop - 1];
    uint8_t centreSymbolId =
        Game::symbolsCentreReel[this->reelStopInfo.centreStop - 1];
    uint8_t rightSymbolId =
        Game::symbolsRightReel[this->reelStopInfo.rightStop - 1];

    printf("Calculated reel positions: %s - %s - %s",
           Game::symbolMap[leftSymbolId].c_str(),
           Game::symbolMap[centreSymbolId].c_str(),
           Game::symbolMap[rightSymbolId].c_str());

    int leftSteps = (((this->reelStopInfo.leftStop - 1) + 75) * STEPS_PER_STOP);
    int centreSteps =
        (((this->reelStopInfo.centreStop - 1) + 50) * STEPS_PER_STOP);
    int rightSteps = (((this->reelStopInfo.rightStop - 1) + 25) * STEPS_PER_STOP);

    // Switch on
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FULL);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    if (leftStop > 0)
    {
        // Check if reel is held
        leftReel.startAfterHome(PCA9629A::Direction::CW, leftSteps, 1);
    }

    if (centreStop > 0)
    {
        // Check if reel is held
        centreReel.startAfterHome(PCA9629A::Direction::CW, centreSteps, 1);
    }

    if (rightStop > 0)
    {
        rightReel.startAfterHome(PCA9629A::Direction::CW, rightSteps, 1);
    }

    // if reel is held (i.e. stop number is 0) then don't play the reel stop sound for that reel
    bool leftPlayAudio = leftStop > 0;
    bool centrePlayAudio = centreStop > 0;
    bool rightPlayAudio = rightStop > 0;

    // Loop waiting for reels to stop
    bool leftFinished = leftReel.isStopped();
    bool centreFinished = centreReel.isStopped();
    bool rightFinished = rightReel.isStopped();

    int count = 0;

    for (;;)
    {
        if (leftFinished && leftPlayAudio)
        {
            this->audioController.playAudioFile(Sounds::SND_REEL_STOP);
            leftPlayAudio = false;
        }

        if (centreFinished && centrePlayAudio)
        {
            this->audioController.playAudioFile(Sounds::SND_REEL_STOP);
            centrePlayAudio = false;
        }

        if (rightFinished && rightPlayAudio)
        {
            this->audioController.playAudioFile(Sounds::SND_REEL_STOP);
            rightPlayAudio = false;
        }

        if (leftFinished && centreFinished && rightFinished)
        {
            break;
        }

        // Update the moves value - just a bit of decoration here really
        if (count == 0)
        {
            this->displayController.setMoves(random8(13));
        }
        else if (count == 10)
        {
            count = 0;
        }
        count++;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (!leftFinished)
        {
            leftFinished = leftReel.isStopped();
        }
        if (!centreFinished)
        {
            centreFinished = centreReel.isStopped();
        }
        if (!rightFinished)
        {
            rightFinished = rightReel.isStopped();
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Switch off
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_QUARTER);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    this->commandInProgress = false;
}

void ReelController::shuffle(const uint8_t leftStop, const uint8_t centreStop,
                             const uint8_t rightStop)
{
    this->commandInProgress = true;

    ESP_LOGI(TAG, "spin called: left stop: %d, centre stop: %d, right stop: %d",
             leftStop, centreStop, rightStop);

    if (leftStop > 0)
        this->reelStopInfo.leftStop = leftStop;
    if (centreStop > 0)
        this->reelStopInfo.centreStop = centreStop;
    if (rightStop > 0)
        this->reelStopInfo.rightStop = rightStop;

    uint8_t leftSymbolId = Game::symbolsLeftReel[this->reelStopInfo.leftStop - 1];
    uint8_t centreSymbolId =
        Game::symbolsCentreReel[this->reelStopInfo.centreStop - 1];
    uint8_t rightSymbolId =
        Game::symbolsRightReel[this->reelStopInfo.rightStop - 1];

    printf("Calculated reel positions: %s - %s - %s",
           Game::symbolMap[leftSymbolId].c_str(),
           Game::symbolMap[centreSymbolId].c_str(),
           Game::symbolMap[rightSymbolId].c_str());

    int leftSteps = (((this->reelStopInfo.leftStop - 1) + 75) * STEPS_PER_STOP);
    int centreSteps =
        (((this->reelStopInfo.centreStop - 1) + 50) * STEPS_PER_STOP);
    int rightSteps = (((this->reelStopInfo.rightStop - 1) + 25) * STEPS_PER_STOP);

    // Switch on
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FULL);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    if (leftStop > 0)
    {
        // Check if reel is held
        leftReel.startAfterHome(PCA9629A::Direction::CW, leftSteps, 1);
    }

    if (centreStop > 0)
    {
        // Check if reel is held
        centreReel.startAfterHome(PCA9629A::Direction::CCW, centreSteps, 1);
    }

    if (rightStop > 0)
    {
        rightReel.startAfterHome(PCA9629A::Direction::CW, rightSteps, 1);
    }

    bool leftPlayAudio = leftStop > 0;
    bool centrePlayAudio = centreStop > 0;
    bool rightPlayAudio = rightStop > 0;

    // Loop waiting for reels to stop
    bool leftFinished = leftReel.isStopped();
    bool centreFinished = centreReel.isStopped();
    bool rightFinished = rightReel.isStopped();

    int count = 0;

    for (;;)
    {
        if (leftFinished && leftPlayAudio)
        {
            this->audioController.playAudioFile(Sounds::SND_REEL_STOP);
            leftPlayAudio = false;
        }

        if (centreFinished && centrePlayAudio)
        {
            this->audioController.playAudioFile(Sounds::SND_REEL_STOP);
            centrePlayAudio = false;
        }

        if (rightFinished && rightPlayAudio)
        {
            this->audioController.playAudioFile(Sounds::SND_REEL_STOP);
            rightPlayAudio = false;
        }

        if (leftFinished && centreFinished && rightFinished)
        {
            break;
        }

        // Update the moves value - just a bit of decoration here really
        if (count == 0)
        {
            this->displayController.setMoves(random8(13));
        }
        else if (count == 10)
        {
            count = 0;
        }
        count++;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (!leftFinished)
        {
            leftFinished = leftReel.isStopped();
        }
        if (!centreFinished)
        {
            centreFinished = centreReel.isStopped();
        }
        if (!rightFinished)
        {
            rightFinished = rightReel.isStopped();
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Switch off
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_QUARTER);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    this->commandInProgress = false;
}

void ReelController::nudge(const uint8_t leftStops, const uint8_t centreStops,
                           const uint8_t rightStops)
{
    this->commandInProgress = true;

    ESP_LOGI(TAG,
             "nudge() called: leftStops: %d, centreStops: %d, rightStops: %d",
             leftStops, centreStops, rightStops);

    this->reelStopInfo.leftStop += leftStops;
    this->reelStopInfo.centreStop += centreStops;
    this->reelStopInfo.rightStop += rightStops;

    uint8_t leftSymbolId = Game::symbolsLeftReel[this->reelStopInfo.leftStop - 1];
    uint8_t centreSymbolId =
        Game::symbolsCentreReel[this->reelStopInfo.centreStop - 1];
    uint8_t rightSymbolId =
        Game::symbolsRightReel[this->reelStopInfo.rightStop - 1];

    printf("Calculated reel positions: %s - %s - %s",
           Game::symbolMap[leftSymbolId].c_str(),
           Game::symbolMap[centreSymbolId].c_str(),
           Game::symbolMap[rightSymbolId].c_str());

    int leftSteps = leftStops * STEPS_PER_STOP;
    int centreSteps = centreStops * STEPS_PER_STOP;
    int rightSteps = rightStops * STEPS_PER_STOP;

    // Switch on
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FULL);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    if (leftStops > 0)
    {
        // Check if reel is held
        leftReel.start(PCA9629A::Direction::CW, leftSteps, 1);
    }

    if (centreStops > 0)
    {
        // Check if reel is held
        centreReel.start(PCA9629A::Direction::CW, centreSteps, 1);
    }

    if (rightStops > 0)
    {
        rightReel.start(PCA9629A::Direction::CW, rightSteps, 1);
    }

    bool leftPlayAudio = leftStops > 0;
    bool centrePlayAudio = centreStops > 0;
    bool rightPlayAudio = rightStops > 0;

    // Loop waiting for reels to stop
    bool leftFinished = leftReel.isStopped();
    bool centreFinished = centreReel.isStopped();
    bool rightFinished = rightReel.isStopped();

    int count = 0;

    for (;;)
    {
        if (leftFinished && leftPlayAudio)
        {
            this->audioController.playAudioFile(Sounds::SND_REEL_STOP);
            leftPlayAudio = false;
        }

        if (centreFinished && centrePlayAudio)
        {
            this->audioController.playAudioFile(Sounds::SND_REEL_STOP);
            centrePlayAudio = false;
        }

        if (rightFinished && rightPlayAudio)
        {
            this->audioController.playAudioFile(Sounds::SND_REEL_STOP);
            rightPlayAudio = false;
        }

        if (leftFinished && centreFinished && rightFinished)
        {
            break;
        }

        // Update the moves value - just a bit of decoration here really
        if (count == 0)
        {
            this->displayController.setMoves(random8(13));
        }
        else if (count == 10)
        {
            count = 0;
        }
        count++;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (!leftFinished)
        {
            leftFinished = leftReel.isStopped();
        }
        if (!centreFinished)
        {
            centreFinished = centreReel.isStopped();
        }
        if (!rightFinished)
        {
            rightFinished = rightReel.isStopped();
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Switch off
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_QUARTER);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    this->commandInProgress = false;
}

void ReelController::calibrate()
{
    ESP_LOGI(TAG, "Entering calibration mode");
    int leftCwCorrection = 0;
    int leftCcwCorrection = 0;
    int centreCwCorrection = 0;
    int centreCcwCorrection = 0;
    int rightCwCorrection = 0;
    int rightCcwCorrection = 0;

    // Switch on
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FULL);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    displayController.displayVFDText("LEFT CW: 00");
    this->leftReel.home(PCA9629A::Direction::CW);
    std::bitset<8> btnStatus = 0;
    while (!btnStatus.test(BTN_START))
    {
        if (btnStatus.test(BTN_HOLD_HI))
        {
            leftCwCorrection--;
            displayController.displayVFDText(
                std::string("LEFT CW: ").append(std::to_string(leftCwCorrection)));
            leftReel.start(PCA9629A::Direction::CCW, 1, 1);
        }
        else if (btnStatus.test(BTN_HOLD_LO))
        {
            leftCwCorrection++;
            displayController.displayVFDText(
                std::string("LEFT CW: ").append(std::to_string(leftCwCorrection)));
            leftReel.start(PCA9629A::Direction::CW, 1, 1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(75));
        btnStatus = this->displayController.getButtonStatus();
    }

    displayController.displayVFDText("LEFT CCW: 00");
    this->leftReel.home(PCA9629A::Direction::CCW);
    btnStatus = 0;
    while (!btnStatus.test(BTN_START))
    {
        if (btnStatus.test(BTN_HOLD_HI))
        {
            leftCcwCorrection--;
            displayController.displayVFDText(
                std::string("LEFT CCW: ").append(std::to_string(leftCcwCorrection)));
            leftReel.start(PCA9629A::Direction::CW, 1, 1);
        }
        else if (btnStatus.test(BTN_HOLD_LO))
        {
            leftCcwCorrection++;
            displayController.displayVFDText(
                std::string("LEFT CCW: ").append(std::to_string(leftCcwCorrection)));
            leftReel.start(PCA9629A::Direction::CCW, 1, 1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(75));
        btnStatus = this->displayController.getButtonStatus();
    }

    displayController.displayVFDText("CENTRE CW: 00");
    this->rightReel.home(PCA9629A::Direction::CW);
    btnStatus = 0;
    while (!btnStatus.test(BTN_START))
    {
        if (btnStatus.test(BTN_HOLD_HI))
        {
            centreCwCorrection--;
            displayController.displayVFDText(
                std::string("CENTRE CW: ")
                .append(std::to_string(centreCwCorrection)));
            centreReel.start(PCA9629A::Direction::CCW, 1, 1);
        }
        else if (btnStatus.test(BTN_HOLD_LO))
        {
            centreCwCorrection++;
            displayController.displayVFDText(
                std::string("CENTRE CW: ")
                .append(std::to_string(centreCwCorrection)));
            centreReel.start(PCA9629A::Direction::CW, 1, 1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(75));
        btnStatus = this->displayController.getButtonStatus();
    }

    displayController.displayVFDText("CENTRE CCW: 00");
    this->rightReel.home(PCA9629A::Direction::CCW);
    btnStatus = 0;
    while (!btnStatus.test(BTN_START))
    {
        if (btnStatus.test(BTN_HOLD_HI))
        {
            centreCcwCorrection--;
            displayController.displayVFDText(
                std::string("CENTRE CCW: ")
                .append(std::to_string(rightCcwCorrection)));
            centreReel.start(PCA9629A::Direction::CW, 1, 1);
        }
        else if (btnStatus.test(BTN_HOLD_LO))
        {
            centreCcwCorrection++;
            displayController.displayVFDText(
                std::string("CENTRE CCW: ")
                .append(std::to_string(rightCcwCorrection)));
            centreReel.start(PCA9629A::Direction::CCW, 1, 1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(75));
        btnStatus = this->displayController.getButtonStatus();
    }

    displayController.displayVFDText("RIGHT CW: 00");
    this->rightReel.home(PCA9629A::Direction::CW);
    btnStatus = 0;
    while (!btnStatus.test(BTN_START))
    {
        if (btnStatus.test(BTN_HOLD_HI))
        {
            rightCwCorrection--;
            displayController.displayVFDText(
                std::string("RIGHT CW: ").append(std::to_string(rightCwCorrection)));
            rightReel.start(PCA9629A::Direction::CCW, 1, 1);
        }
        else if (btnStatus.test(BTN_HOLD_LO))
        {
            rightCwCorrection++;
            displayController.displayVFDText(
                std::string("RIGHT CW: ").append(std::to_string(rightCwCorrection)));
            rightReel.start(PCA9629A::Direction::CW, 1, 1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(75));
        btnStatus = this->displayController.getButtonStatus();
    }

    displayController.displayVFDText("RIGHTCCW: 00");
    this->rightReel.home(PCA9629A::Direction::CCW);
    btnStatus = 0;
    while (!btnStatus.test(BTN_START))
    {
        if (btnStatus.test(BTN_HOLD_HI))
        {
            rightCcwCorrection--;
            displayController.displayVFDText(
                std::string("RIGHT CCW: ")
                .append(std::to_string(rightCcwCorrection)));
            rightReel.start(PCA9629A::Direction::CW, 1, 1);
        }
        else if (btnStatus.test(BTN_HOLD_LO))
        {
            rightCcwCorrection++;
            displayController.displayVFDText(
                std::string("RIGHT CCW: ")
                .append(std::to_string(rightCcwCorrection)));
            rightReel.start(PCA9629A::Direction::CCW, 1, 1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(75));
        btnStatus = this->displayController.getButtonStatus();
    }

    // Switch off
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_QUARTER);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void ReelController::test()
{
    ESP_LOGI(TAG, "Entering test mode");

    for (int i = 0; i < 25; i++)
    {
        uint8_t leftSymbolId = Game::symbolsLeftReel[i];
        uint8_t centreSymbolId = Game::symbolsCentreReel[i];
        uint8_t rightSymbolId = Game::symbolsRightReel[i];

        ESP_LOGI(TAG, "Calculated reel positions: %s - %s - %s",
                 Game::symbolMap[leftSymbolId].c_str(),
                 Game::symbolMap[centreSymbolId].c_str(),
                 Game::symbolMap[rightSymbolId].c_str());

        // Switch on
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FULL);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

        uint8_t leftSteps = i * STEPS_PER_STOP;
        uint8_t centreSteps = i * STEPS_PER_STOP;
        uint8_t rightSteps = i * STEPS_PER_STOP;

        auto leftReelThread = std::thread([this, leftSteps]()
        {
            leftReel.startAfterHome(PCA9629A::Direction::CW, leftSteps, 1);
        });
        auto centreReelThread = std::thread([this, centreSteps]()
        {
            centreReel.startAfterHome(PCA9629A::Direction::CW, centreSteps, 1);
        });
        auto rightReelThread = std::thread([this, rightSteps]()
        {
            rightReel.startAfterHome(PCA9629A::Direction::CW, rightSteps, 1);
        });
        leftReelThread.join();
        centreReelThread.join();
        rightReelThread.join();

        while (!leftReel.isStopped() || !centreReel.isStopped() ||
            !rightReel.isStopped())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }

        this->displayController.waitForButton(BTN_START_MASK_BIT);

        // Switch off
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_QUARTER);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }
}
