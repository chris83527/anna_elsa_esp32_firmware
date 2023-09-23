
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
    ESP_LOGD(TAG, "initialise() called");

    reelLeftInitOk = false;
    reelCentreInitOk = false;
    reelRightInitOk = false;                    

    this->leftReel.startWithHome(PCA9629A::Direction::CW, 0, 0); // return to home
    this->centreReel.startWithHome(PCA9629A::Direction::CW, 0, 0); // return to home
    this->rightReel.startWithHome(PCA9629A::Direction::CW, 0, 0); // return to home
    
    return true;
}

void ReelController::spin(const uint8_t leftStop, const uint8_t centreStop, const uint8_t rightStop) {

    this->commandInProgress = true;
    this->reelStopInfo.leftStop = leftStop;
    this->reelStopInfo.centreStop = centreStop;
    this->reelStopInfo.rightStop = rightStop;
      
    
    int leftSteps = ((this->reelStopInfo.leftStop + 75) * STEPS_PER_STOP);
    int centreSteps = ((this->reelStopInfo.centreStop + 50) * STEPS_PER_STOP);
    int rightSteps = ((this->reelStopInfo.rightStop + 25) * STEPS_PER_STOP);
       
    leftReel.startWithHome(PCA9629A::Direction::CW, leftSteps, 0);
    centreReel.startWithHome(PCA9629A::Direction::CW, centreSteps, 0);
    rightReel.startWithHome(PCA9629A::Direction::CW, rightSteps, 0);
    
    // TODO - Poll registers to see when motors have finished
    
    this->commandInProgress = false;
}

void ReelController::shuffle(const uint8_t leftStop, const uint8_t centreStop, const uint8_t rightStop) {

    this->commandInProgress = true;
    this->reelStopInfo.leftStop = leftStop;
    this->reelStopInfo.centreStop = centreStop;
    this->reelStopInfo.rightStop = rightStop;

    int leftSteps = ((this->reelStopInfo.leftStop + 75) * STEPS_PER_STOP);
    int centreSteps = ((this->reelStopInfo.centreStop + 50) * STEPS_PER_STOP);
    int rightSteps = ((this->reelStopInfo.rightStop + 25) * STEPS_PER_STOP);
       
    leftReel.startWithHome(PCA9629A::Direction::CW, leftSteps, 0);
    centreReel.startWithHome(PCA9629A::Direction::CCW, centreSteps, 0);
    rightReel.startWithHome(PCA9629A::Direction::CW, rightSteps, 0);
    
    // TODO - Poll registers to see when motors have finished
    
    this->commandInProgress = false;
}

void ReelController::nudge(const uint8_t leftStops, const uint8_t centreStops, const uint8_t rightStops) {

    this->commandInProgress = true;
    //ESP_LOGD(TAG, "nudge() called: leftStops: %d, midStops: %d, rightStops: %d", leftStops, midStops, rightStops);

    this->reelStopInfo.leftStop += leftStops;
    this->reelStopInfo.centreStop += centreStops;
    this->reelStopInfo.rightStop += rightStops;

    int leftSteps = leftStops * STEPS_PER_STOP;
    int centreSteps = centreStops * STEPS_PER_STOP;
    int rightSteps = rightStops * STEPS_PER_STOP;

    leftReel.start(PCA9629A::Direction::CW, leftSteps, 0);
    centreReel.start(PCA9629A::Direction::CW, centreSteps, 0);
    rightReel.start(PCA9629A::Direction::CW, rightSteps, 0);

    this->commandInProgress = false;
}
