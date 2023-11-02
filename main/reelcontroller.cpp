
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
 * @file reelcontroller.cpp
 *
 * Higher level routines for controlling MCP23008 based reel driver board
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <cstring>
#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pca9629a.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#include "config.h"
#include "reelcontroller.h"
#include "audiocontroller.h"
#include "esp_pthread.h"
#include "displaycontroller.h"
#include "game.h"

static const char *TAG = "ReelController";

bool reelLeftInitOk;
bool reelCentreInitOk;
bool reelRightInitOk;

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY_QUARTER (127)        // Set duty to 12,5% 
#define LEDC_DUTY_FULL (1023)           // Set duty to 100%.((2 ** 10) - 1)  = 1023
#define LEDC_FREQUENCY (50)            // Frequency in Hertz. Set frequency at 100Hz

ReelController::ReelController(MainController *mainController) {
    ESP_LOGD(TAG, "Entering constructor");
    this->mainController = mainController;
    ESP_LOGD(TAG, "Leaving constructor");
}

ReelController::ReelController(const ReelController &orig) {
}

bool ReelController::isCommandInProgress() {
    return commandInProgress;
}

ReelController::reel_stop_info_t ReelController::getReelStopInfo() {
    return reelStopInfo;
}

bool ReelController::initialise() {

    ESP_LOGI(TAG, "ReelController::initialise() called");

    // MOTOR_EN is on a GPIO
    gpio_pad_select_gpio(GPIO_MOTOR_EN);
    // Set the GPIO as a push/pull output
    gpio_set_direction(GPIO_MOTOR_EN, GPIO_MODE_OUTPUT);

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel.gpio_num = GPIO_MOTOR_EN;
    ledc_channel.speed_mode = LEDC_MODE;
    ledc_channel.channel = LEDC_CHANNEL;
    ledc_channel.intr_type = LEDC_INTR_DISABLE;
    ledc_channel.timer_sel = LEDC_TIMER;
    ledc_channel.duty = 0;
    ledc_channel.hpoint = 0;

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer.speed_mode = LEDC_MODE;
    ledc_timer.timer_num = LEDC_TIMER;
    ledc_timer.duty_resolution = LEDC_DUTY_RES;
    ledc_timer.freq_hz = LEDC_FREQUENCY; // Set output frequency at 100Hz
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;

    if (ledc_channel_config(&ledc_channel) != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred initialising PWM subsystem for reels (channel config)");
        return ESP_FAIL;
    }

    if (ledc_timer_config(&ledc_timer) != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred initialising PWM subsystem for reels (timer config)");
        return ESP_FAIL;
    }

    reelLeftInitOk = false;
    reelCentreInitOk = false;
    reelRightInitOk = false;

    leftReel = new PCA9629A(I2C_NUM_0, REEL_LEFT_I2C_ADDRESS);
    centreReel = new PCA9629A(I2C_NUM_0, REEL_CENTRE_I2C_ADDRESS);
    rightReel = new PCA9629A(I2C_NUM_0, REEL_RIGHT_I2C_ADDRESS);

    this->leftReel->initialise();
    this->centreReel->initialise();
    this->rightReel->initialise();

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FULL);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    this->leftReel->home(PCA9629A::Direction::CW); // return to home
    this->centreReel->home(PCA9629A::Direction::CW); // return to home
    this->rightReel->home(PCA9629A::Direction::CW); // return to home

    // Wait for reels to stop
    while (!leftReel->isStopped() || !centreReel->isStopped() || !rightReel->isStopped()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_QUARTER);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    //calibrate();
    //test();

    return true;
}

void ReelController::spin(const uint8_t leftStop, const uint8_t centreStop, const uint8_t rightStop) {

    ESP_LOGI(TAG, "spin called: left stop: %d, centre stop: %d, right stop: %d", leftStop, centreStop, rightStop);

    uint8_t leftSymbolId = mainController->getGame()->symbolsLeftReel[leftStop - 1];
    uint8_t centreSymbolId = mainController->getGame()->symbolsCentreReel[centreStop - 1];
    uint8_t rightSymbolId = mainController->getGame()->symbolsRightReel[rightStop - 1];

    ESP_LOGI(TAG, "Calculated reel positions: %s - %s - %s", mainController->getGame()->symbolMap[leftSymbolId].c_str(), mainController->getGame()->symbolMap[centreSymbolId].c_str(), mainController->getGame()->symbolMap[rightSymbolId].c_str());

    this->commandInProgress = true;
    this->reelStopInfo.leftStop = leftStop;
    this->reelStopInfo.centreStop = centreStop;
    this->reelStopInfo.rightStop = rightStop;

    int leftSteps = (((this->reelStopInfo.leftStop - 1) + 75) * STEPS_PER_STOP);
    int centreSteps = (((this->reelStopInfo.centreStop - 1) + 50) * STEPS_PER_STOP);
    int rightSteps = (((this->reelStopInfo.rightStop - 1) + 25) * STEPS_PER_STOP);

    // Switch on
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FULL);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    auto cfg = esp_pthread_get_default_config();
    cfg.thread_name = "LeftReelThread";
    cfg.prio = 1;
    cfg.stack_size = 1024;
    esp_pthread_set_cfg(&cfg);

    auto leftReelThread = std::thread([this, &leftSteps]() {
        leftReel->startAfterHome(PCA9629A::Direction::CW, leftSteps, 1);
    });
    cfg.thread_name = "CentreReelThread";
    cfg.prio = 1;
    cfg.stack_size = 1024;
    esp_pthread_set_cfg(&cfg);
    auto centreReelThread = std::thread([this, &centreSteps]() {
        centreReel->startAfterHome(PCA9629A::Direction::CW, centreSteps, 1);
    });
    cfg.thread_name = "RightReelThread";
    cfg.prio = 1;
    cfg.stack_size = 1024;
    esp_pthread_set_cfg(&cfg);
    auto rightReelThread = std::thread([this, &rightSteps]() {
        rightReel->startAfterHome(PCA9629A::Direction::CW, rightSteps, 1);
    });

    leftReelThread.join();
    centreReelThread.join();
    rightReelThread.join();

    
    bool leftPlayAudio = true;
    bool centrePlayAudio = true;
    bool rightPlayAudio = true;
    
    // Loop waiting for reels to stop    
    bool leftFinished = leftReel->isStopped();
    bool centreFinished = centreReel->isStopped();
    bool rightFinished = rightReel->isStopped();
    
    while (!leftFinished || !centreFinished || !rightFinished) {

        if (leftFinished && leftPlayAudio) {
            this->mainController->getAudioController()->playAudioFile(Sounds::SND_REEL_STOP);            
            leftPlayAudio = false;
        }

        if (centreFinished && centrePlayAudio) {
            this->mainController->getAudioController()->playAudioFile(Sounds::SND_REEL_STOP);            
            centrePlayAudio = false;
        }

        if (rightFinished && rightPlayAudio) {
            this->mainController->getAudioController()->playAudioFile(Sounds::SND_REEL_STOP);            
            rightPlayAudio = false;
        }

        uint8_t moves = random8_to(13);
        this->mainController->getDisplayController()->setMoves(moves);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        leftFinished = leftReel->isStopped();
        centreFinished = centreReel->isStopped();
        rightFinished = rightReel->isStopped();

    }

    // Switch off
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_QUARTER);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    this->commandInProgress = false;
}

void ReelController::shuffle(const uint8_t leftStop, const uint8_t centreStop, const uint8_t rightStop) {
    ESP_LOGI(TAG, "shuffle() called: leftStop: %d, centreStop: %d, rightStop: %d", leftStop, centreStop, rightStop);

    uint8_t leftSymbolId = mainController->getGame()->symbolsLeftReel[leftStop - 1];
    uint8_t centreSymbolId = mainController->getGame()->symbolsCentreReel[centreStop - 1];
    uint8_t rightSymbolId = mainController->getGame()->symbolsRightReel[rightStop - 1];

    ESP_LOGI(TAG, "Calculated reel positions: %s - %s - %s", mainController->getGame()->symbolMap[leftSymbolId].c_str(), mainController->getGame()->symbolMap[centreSymbolId].c_str(), mainController->getGame()->symbolMap[rightSymbolId].c_str());

    this->commandInProgress = true;
    this->reelStopInfo.leftStop = leftStop;
    this->reelStopInfo.centreStop = centreStop;
    this->reelStopInfo.rightStop = rightStop;

    int leftSteps = (((this->reelStopInfo.leftStop - 1) + 75) * STEPS_PER_STOP);
    int centreSteps = (((this->reelStopInfo.centreStop - 1) + 50) * STEPS_PER_STOP);
    int rightSteps = (((this->reelStopInfo.rightStop - 1) + 25) * STEPS_PER_STOP);

    // Switch on
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FULL);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    gpio_set_level(GPIO_MOTOR_EN, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    auto leftReelThread = std::thread([this, &leftSteps]() {
        leftReel->startAfterHome(PCA9629A::Direction::CW, leftSteps, 1);
    });
    auto centreReelThread = std::thread([this, &centreSteps]() {
        centreReel->startAfterHome(PCA9629A::Direction::CCW, centreSteps, 1);
    });
    auto rightReelThread = std::thread([this, &rightSteps]() {
        rightReel->startAfterHome(PCA9629A::Direction::CW, rightSteps, 1);
    });
    leftReelThread.join();
    centreReelThread.join();
    rightReelThread.join();

    // Loop waiting for reels to stop    
    bool leftFinished = false;
    bool centreFinished = false;
    bool rightFinished = false;

    while (!leftReel->isStopped() || !centreReel->isStopped() || !rightReel->isStopped()) {

        if (leftReel->isStopped() && !leftFinished) {
            this->mainController->getAudioController()->playAudioFile(Sounds::SND_REEL_STOP);
            leftFinished = true;
        }

        if (centreReel->isStopped() && !centreFinished) {
            this->mainController->getAudioController()->playAudioFile(Sounds::SND_REEL_STOP);
            centreFinished = true;
        }

        if (rightReel->isStopped() && !rightFinished) {
            this->mainController->getAudioController()->playAudioFile(Sounds::SND_REEL_STOP);
            rightFinished = true;
        }

        uint8_t moves = random8_to(13);
        this->mainController->getDisplayController()->setMoves(moves);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Switch off    
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_QUARTER);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    this->commandInProgress = false;
}

void ReelController::nudge(const uint8_t leftStops, const uint8_t centreStops, const uint8_t rightStops) {

    this->commandInProgress = true;
    ESP_LOGI(TAG, "nudge() called: leftStops: %d, centreStops: %d, rightStops: %d", leftStops, centreStops, rightStops);

    this->reelStopInfo.leftStop += leftStops;
    this->reelStopInfo.centreStop += centreStops;
    this->reelStopInfo.rightStop += rightStops;

    uint8_t leftSymbolId = mainController->getGame()->symbolsLeftReel[this->reelStopInfo.leftStop - 1];
    uint8_t centreSymbolId = mainController->getGame()->symbolsCentreReel[this->reelStopInfo.centreStop - 1];
    uint8_t rightSymbolId = mainController->getGame()->symbolsRightReel[this->reelStopInfo.rightStop - 1];

    ESP_LOGD(TAG, "Calculated reel positions: %s - %s - %s", mainController->getGame()->symbolMap[leftSymbolId].c_str(), mainController->getGame()->symbolMap[centreSymbolId].c_str(), mainController->getGame()->symbolMap[rightSymbolId].c_str());

    int leftSteps = leftStops * STEPS_PER_STOP;
    int centreSteps = centreStops * STEPS_PER_STOP;
    int rightSteps = rightStops * STEPS_PER_STOP;

    ESP_LOGD(TAG, "nudge: leftSteps: %d, centreSteps: %d, rightSteps: %d", leftSteps, centreSteps, rightSteps);

    // Switch on
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FULL);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    auto leftReelThread = std::thread([this, &leftSteps]() {
        leftReel->start(PCA9629A::Direction::CW, leftSteps, 1);
    });

    auto centreReelThread = std::thread([this, &centreSteps]() {
        centreReel->start(PCA9629A::Direction::CW, centreSteps, 1);
    });
    auto rightReelThread = std::thread([this, &rightSteps]() {
        rightReel->start(PCA9629A::Direction::CW, rightSteps, 1);
    });
    leftReelThread.join();
    centreReelThread.join();
    rightReelThread.join();

    // Loop waiting for reels to stop    
    bool leftFinished = false;
    bool centreFinished = false;
    bool rightFinished = false;
    while (!leftReel->isStopped() || !centreReel->isStopped() || !rightReel->isStopped()) {

        if (leftReel->isStopped() && !leftFinished) {
            this->mainController->getAudioController()->playAudioFile(Sounds::SND_REEL_STOP);
            leftFinished = true;
        }

        if (centreReel->isStopped() && !centreFinished) {
            this->mainController->getAudioController()->playAudioFile(Sounds::SND_REEL_STOP);
            centreFinished = true;
        }

        if (rightReel->isStopped() && !rightFinished) {
            this->mainController->getAudioController()->playAudioFile(Sounds::SND_REEL_STOP);
            rightFinished = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Switch off
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_QUARTER);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    this->commandInProgress = false;
}

void ReelController::calibrate() {
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

    this->mainController->getDisplayController()->displayText("LEFT CW: 00");
    this->leftReel->home(PCA9629A::Direction::CW);
    std::bitset<8> btnStatus = 0;
    while (!btnStatus.test(BTN_START)) {
        if (btnStatus.test(BTN_HOLD_HI)) {
            leftCwCorrection--;
            this->mainController->getDisplayController()->displayText(std::string("LEFT CW: ").append(std::to_string(leftCwCorrection)));
            leftReel->start(PCA9629A::Direction::CCW, 1, 1);
        } else if (btnStatus.test(BTN_HOLD_LO)) {
            leftCwCorrection++;
            this->mainController->getDisplayController()->displayText(std::string("LEFT CW: ").append(std::to_string(leftCwCorrection)));
            leftReel->start(PCA9629A::Direction::CW, 1, 1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(75));
        btnStatus = mainController->getDisplayController()->getButtonStatus();
    }

    this->mainController->getDisplayController()->displayText("LEFT CCW: 00");
    this->leftReel->home(PCA9629A::Direction::CCW);
    btnStatus = 0;
    while (!btnStatus.test(BTN_START)) {
        if (btnStatus.test(BTN_HOLD_HI)) {
            leftCcwCorrection--;
            this->mainController->getDisplayController()->displayText(std::string("LEFT CCW: ").append(std::to_string(leftCcwCorrection)));
            leftReel->start(PCA9629A::Direction::CW, 1, 1);
        } else if (btnStatus.test(BTN_HOLD_LO)) {
            leftCcwCorrection++;
            this->mainController->getDisplayController()->displayText(std::string("LEFT CCW: ").append(std::to_string(leftCcwCorrection)));
            leftReel->start(PCA9629A::Direction::CCW, 1, 1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(75));
        btnStatus = mainController->getDisplayController()->getButtonStatus();
    }

    this->mainController->getDisplayController()->displayText("CENTRE CW: 00");
    this->rightReel->home(PCA9629A::Direction::CW);
    btnStatus = 0;
    while (!btnStatus.test(BTN_START)) {
        if (btnStatus.test(BTN_HOLD_HI)) {
            centreCwCorrection--;
            this->mainController->getDisplayController()->displayText(std::string("CENTRE CW: ").append(std::to_string(centreCwCorrection)));
            centreReel->start(PCA9629A::Direction::CCW, 1, 1);
        } else if (btnStatus.test(BTN_HOLD_LO)) {
            centreCwCorrection++;
            this->mainController->getDisplayController()->displayText(std::string("CENTRE CW: ").append(std::to_string(centreCwCorrection)));
            centreReel->start(PCA9629A::Direction::CW, 1, 1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(75));
        btnStatus = mainController->getDisplayController()->getButtonStatus();
    }

    this->mainController->getDisplayController()->displayText("CENTRE CCW: 00");
    this->rightReel->home(PCA9629A::Direction::CCW);
    btnStatus = 0;
    while (!btnStatus.test(BTN_START)) {
        if (btnStatus.test(BTN_HOLD_HI)) {
            centreCcwCorrection--;
            this->mainController->getDisplayController()->displayText(std::string("CENTRE CCW: ").append(std::to_string(rightCcwCorrection)));
            centreReel->start(PCA9629A::Direction::CW, 1, 1);
        } else if (btnStatus.test(BTN_HOLD_LO)) {
            centreCcwCorrection++;
            this->mainController->getDisplayController()->displayText(std::string("CENTRE CCW: ").append(std::to_string(rightCcwCorrection)));
            centreReel->start(PCA9629A::Direction::CCW, 1, 1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(75));
        btnStatus = mainController->getDisplayController()->getButtonStatus();
    }

    this->mainController->getDisplayController()->displayText("RIGHT CW: 00");
    this->rightReel->home(PCA9629A::Direction::CW);
    btnStatus = 0;
    while (!btnStatus.test(BTN_START)) {
        if (btnStatus.test(BTN_HOLD_HI)) {
            rightCwCorrection--;
            this->mainController->getDisplayController()->displayText(std::string("RIGHT CW: ").append(std::to_string(rightCwCorrection)));
            rightReel->start(PCA9629A::Direction::CCW, 1, 1);
        } else if (btnStatus.test(BTN_HOLD_LO)) {
            rightCwCorrection++;
            this->mainController->getDisplayController()->displayText(std::string("RIGHT CW: ").append(std::to_string(rightCwCorrection)));
            rightReel->start(PCA9629A::Direction::CW, 1, 1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(75));
        btnStatus = mainController->getDisplayController()->getButtonStatus();
    }

    this->mainController->getDisplayController()->displayText("RIGHTCCW: 00");
    this->rightReel->home(PCA9629A::Direction::CCW);
    btnStatus = 0;
    while (!btnStatus.test(BTN_START)) {
        if (btnStatus.test(BTN_HOLD_HI)) {
            rightCcwCorrection--;
            this->mainController->getDisplayController()->displayText(std::string("RIGHT CCW: ").append(std::to_string(rightCcwCorrection)));
            rightReel->start(PCA9629A::Direction::CW, 1, 1);
        } else if (btnStatus.test(BTN_HOLD_LO)) {
            rightCcwCorrection++;
            this->mainController->getDisplayController()->displayText(std::string("RIGHT CCW: ").append(std::to_string(rightCcwCorrection)));
            rightReel->start(PCA9629A::Direction::CCW, 1, 1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(75));
        btnStatus = mainController->getDisplayController()->getButtonStatus();
    }

    // Switch off
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_QUARTER);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

}

void ReelController::test() {
    ESP_LOGI(TAG, "Entering test mode");

    for (int i = 0; i < 25; i++) {

        uint8_t leftSymbolId = mainController->getGame()->symbolsLeftReel[i];
        uint8_t centreSymbolId = mainController->getGame()->symbolsCentreReel[i];
        uint8_t rightSymbolId = mainController->getGame()->symbolsRightReel[i];

        ESP_LOGI(TAG, "Calculated reel positions: %s - %s - %s", mainController->getGame()->symbolMap[leftSymbolId].c_str(), mainController->getGame()->symbolMap[centreSymbolId].c_str(), mainController->getGame()->symbolMap[rightSymbolId].c_str());

        // Switch on
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FULL);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

        uint8_t leftSteps = i * STEPS_PER_STOP;
        uint8_t centreSteps = i * STEPS_PER_STOP;
        uint8_t rightSteps = i * STEPS_PER_STOP;

        auto leftReelThread = std::thread([this, leftSteps]() {
            leftReel->startAfterHome(PCA9629A::Direction::CW, leftSteps, 1);
        });
        auto centreReelThread = std::thread([this, centreSteps]() {
            centreReel->startAfterHome(PCA9629A::Direction::CW, centreSteps, 1);
        });
        auto rightReelThread = std::thread([this, rightSteps]() {
            rightReel->startAfterHome(PCA9629A::Direction::CW, rightSteps, 1);
        });
        leftReelThread.join();
        centreReelThread.join();
        rightReelThread.join();

        while (!leftReel->isStopped() || !centreReel->isStopped() || !rightReel->isStopped()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }

        this->mainController->getDisplayController()->waitForButton(BTN_START_MASK_BIT);

        // Switch off
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_QUARTER);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }
}