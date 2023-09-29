
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
#include <string.h>

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

static const char *TAG = "ReelController";

bool reelLeftInitOk;
bool reelCentreInitOk;
bool reelRightInitOk;

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

    reelLeftInitOk = false;
    reelCentreInitOk = false;
    reelRightInitOk = false;                    

    // MOTOR_EN is on a GPIO
    gpio_pad_select_gpio(GPIO_MOTOR_EN);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(GPIO_MOTOR_EN, GPIO_MODE_OUTPUT);
    /* Switch off to start */
    gpio_set_level(GPIO_MOTOR_EN, 1);
    
    leftReel = new PCA9629A(0, GPIO_I2C_SDA, GPIO_I2C_SCL, REEL_LEFT_I2C_ADDRESS, I2C_FREQ_HZ);
    centreReel = new PCA9629A(0, GPIO_I2C_SDA, GPIO_I2C_SCL, REEL_CENTRE_I2C_ADDRESS, I2C_FREQ_HZ);
    rightReel = new PCA9629A(0, GPIO_I2C_SDA, GPIO_I2C_SCL, REEL_RIGHT_I2C_ADDRESS, I2C_FREQ_HZ);
    
    this->leftReel->initialise();
    this->centreReel->initialise();
    this->rightReel->initialise();
    
    this->leftReel->home(PCA9629A::Direction::CW); // return to home
    this->centreReel->home(PCA9629A::Direction::CCW); // return to home
    this->rightReel->home(PCA9629A::Direction::CW); // return to home
    
    return true;
}

void ReelController::spin(const uint8_t leftStop, const uint8_t centreStop, const uint8_t rightStop) {

    ESP_LOGI(TAG, "spin called: left stop: %d, centre stop: %d, right stop: %d", leftStop, centreStop, rightStop);
    
    this->commandInProgress = true;
    this->reelStopInfo.leftStop = leftStop;
    this->reelStopInfo.centreStop = centreStop;
    this->reelStopInfo.rightStop = rightStop;
      
    
    int leftSteps = ((this->reelStopInfo.leftStop + 75) * STEPS_PER_STOP);
    int centreSteps = ((this->reelStopInfo.centreStop + 50) * STEPS_PER_STOP);
    int rightSteps = ((this->reelStopInfo.rightStop + 25) * STEPS_PER_STOP);
       
   leftReel->home(PCA9629A::Direction::CW);
    centreReel->home(PCA9629A::Direction::CW);
    rightReel->home(PCA9629A::Direction::CW);
    
    // Loop waiting for reels to home
    while (!leftReel->isStopped() || !centreReel->isStopped() || !rightReel->isStopped()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    leftReel->start(PCA9629A::Direction::CW, leftSteps, 0);
    centreReel->start(PCA9629A::Direction::CW, centreSteps, 0);
    rightReel->start(PCA9629A::Direction::CW, rightSteps, 0);
    
    // Loop waiting for reels to stop    
    bool leftFinished = false;
    bool centreFinished = false;
    bool rightFinished = false;
    while (!leftReel->isStopped() || !centreReel->isStopped() || !rightReel->isStopped()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        
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
        
    }
    
    this->commandInProgress = false;
}

void ReelController::shuffle(const uint8_t leftStop, const uint8_t centreStop, const uint8_t rightStop) {
    ESP_LOGI(TAG, "shuffle() called: leftStop: %d, centreStop: %d, rightStop: %d", leftStop, centreStop, rightStop);
    
    this->commandInProgress = true;
    this->reelStopInfo.leftStop = leftStop;
    this->reelStopInfo.centreStop = centreStop;
    this->reelStopInfo.rightStop = rightStop;

    int leftSteps = ((this->reelStopInfo.leftStop + 75) * STEPS_PER_STOP);
    int centreSteps = ((this->reelStopInfo.centreStop + 50) * STEPS_PER_STOP);
    int rightSteps = ((this->reelStopInfo.rightStop + 25) * STEPS_PER_STOP);
       
       
    leftReel->home(PCA9629A::Direction::CW);
    centreReel->home(PCA9629A::Direction::CCW);
    rightReel->home(PCA9629A::Direction::CW);
    
    // Loop waiting for reels to home
    while (!leftReel->isStopped() || !centreReel->isStopped() || !rightReel->isStopped()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    leftReel->start(PCA9629A::Direction::CW, leftSteps, 0);
    centreReel->start(PCA9629A::Direction::CCW, centreSteps, 0);
    rightReel->start(PCA9629A::Direction::CW, rightSteps, 0);
    
    // Loop waiting for reels to stop    
    bool leftFinished = false;
    bool centreFinished = false;
    bool rightFinished = false;
    while (!leftReel->isStopped() || !centreReel->isStopped() || !rightReel->isStopped()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        
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
        
    }            
    
    this->commandInProgress = false;
}

void ReelController::nudge(const uint8_t leftStops, const uint8_t centreStops, const uint8_t rightStops) {

    this->commandInProgress = true;
    ESP_LOGI(TAG, "nudge() called: leftStops: %d, centreStops: %d, rightStops: %d", leftStops, centreStops, rightStops);

    this->reelStopInfo.leftStop += leftStops;
    this->reelStopInfo.centreStop += centreStops;
    this->reelStopInfo.rightStop += rightStops;

    int leftSteps = leftStops * STEPS_PER_STOP;
    int centreSteps = centreStops * STEPS_PER_STOP;
    int rightSteps = rightStops * STEPS_PER_STOP;
    
    ESP_LOGI(TAG, "nudge: leftSteps: %d, centreSteps: %d, rightSteps: %d", leftSteps, centreSteps, rightSteps);

    leftReel->start(PCA9629A::Direction::CW, leftSteps, 0);
    centreReel->start(PCA9629A::Direction::CW, centreSteps, 0);
    rightReel->start(PCA9629A::Direction::CW, rightSteps, 0);
    
    // Loop waiting for reels to stop    
    bool leftFinished = false;
    bool centreFinished = false;
    bool rightFinished = false;
    while (!leftReel->isStopped() || !centreReel->isStopped() || !rightReel->isStopped()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        
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
        
    }     

    this->commandInProgress = false;
}
