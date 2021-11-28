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

/* 
 * File:   Game.cpp
 * Author: chris
 *
 * Created on January 14, 2018, 3:02 PM
 * Updated for ESP-IDF October 1st, 2021, 11:26 AM
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <string>
#include <color.h>

#include "config.h"
#include "displaycontroller.h"
#include "audiocontroller.h"
#include "reelcontroller.h"
#include "maincontroller.h"
#include "lib8tion/random8.h"


#include "game.h"
#include "moneycontroller.h"

using namespace std;

static const char *TAG = "Game";

Game::Game(MainController *mainController) {

    ESP_LOGI(TAG, "Entering constructor");

    this->mainController = mainController;
    this->isInProgress = false;

    ESP_LOGI(TAG, "Leaving constructor");
}

Game::Game(const Game &orig) {
}

void Game::initialise() {
    this->lampData = mainController->getDisplayController()->getLampData();
}

void Game::start() {

    ESP_LOGI(TAG, "Beginning game");

    mainController->getDisplayController()->resetLampData(true);
    mainController->getAudioController()->stopPlaying();

    this->isInProgress = true;

    uint8_t nudges = random8_to(6); // 0 - 5    
    bool hold = offerHold();

    lampData[REEL_LAMP_L2].lampState = LampState::on;
    lampData[REEL_LAMP_C2].lampState = LampState::on;
    lampData[REEL_LAMP_R2].lampState = LampState::on;

    if (hold) {
        lampData[LMP_START].lampState = LampState::blinkslow;
        lampData[LMP_HOLD_LO].lampState = LampState::blinkslow;
        lampData[LMP_HOLD].lampState = LampState::blinkslow;
        lampData[LMP_HOLD_HI].lampState = LampState::blinkslow;
    } else {
        lampData[LMP_START].lampState = LampState::blinkslow;
    }    

    mainController->getDisplayController()->setText("PRESS START TO BEGIN");
    mainController->getDisplayController()->displayText();

    
    // loop waiting for button press.
    uint8_t btnStatus = mainController->getDisplayController()->getButtonStatus();
    while (!(btnStatus & BTN_START)) {
        btnStatus = this->mainController->getDisplayController()->getButtonStatus();
        vTaskDelay(pdMS_TO_TICKS(50));

        holdLeft = false;
        holdCentre = false;
        holdRight = false;

        if (hold) {
            if (btnStatus & BTN_HOLD) {
                holdCentre = true;
                lampData[LMP_HOLD].lampState = LampState::on;
            } else if (btnStatus & BTN_HOLD_HI) {
                holdRight = true;
                lampData[LMP_HOLD_HI].lampState = LampState::on;
            } else if (btnStatus & BTN_HOLD_LO) {
                holdLeft = true;
                lampData[LMP_HOLD_LO].lampState = LampState::on;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // let the processor do something else
    }
    mainController->getMoneyController()->incrementGameCount();
    mainController->getMoneyController()->removeFromCredit(20);
    mainController->getAudioController()->playAudioFileSync(Sounds::SND_NOW_THATS_ICE);

    spinReels();

    if (isWinningLine()) {
        transferOrGamble();
    } else if (nudges > 0) {
        playNudges(nudges);
    } else {
        mainController->getAudioController()->playAudioFile(Sounds::SND_LOSE);
    }

    lampData[LMP_START].lampState = LampState::off;
    lampData[LMP_COLLECT].lampState = LampState::off;
    lampData[LMP_HOLD_HI].lampState = LampState::off;
    lampData[LMP_HOLD].lampState = LampState::off;
    lampData[LMP_HOLD_LO].lampState = LampState::off;
    lampData[LMP_TRANSFER].lampState = LampState::off;

    // payout
    if ((mainController->getMoneyController()->getBank() > 0) && (mainController->getMoneyController()->getCredit() < 20)) {
        collectOrContinue();
    }

    Game::isInProgress = false;
    ESP_LOGI(TAG, "Exiting game");
}

void Game::spinReels() {

    ESP_LOGI(TAG, "Entering spinReels()");

    //uint8_t reelStopLeft = holdLeft ? 0 : random8_to(26);
    //uint8_t reelStopCentre = holdCentre ? 0 : random8_to(26);
    //uint8_t reelStopRight = holdRight ? 0 : random8_to(26);
    
    uint8_t reelStopLeft = random8_to(26);
    uint8_t reelStopCentre = random8_to(26);
    uint8_t reelStopRight = random8_to(26);
    
    lampData[LMP_START].lampState = LampState::off;

    this->mainController->getDisplayController()->setText("    LET IT GO!!     ");
    this->mainController->getDisplayController()->displayText();

    mainController->getReelController()->spin(reelStopLeft, reelStopCentre, reelStopRight);

    while (mainController->getReelController()->isCommandInProgress()) {
        vTaskDelay(pdMS_TO_TICKS(50));
        this->moves = random8_to(13);
        //this->mainController->getDisplayController()->setMoves(this->moves); // TODO: fix this
    }

    ESP_LOGI(TAG, "Exiting spinReels()");
}

void Game::shuffleReels() {
    uint8_t reelStopLeft = random8_to(26);
    uint8_t reelStopCentre = random8_to(26);
    uint8_t reelStopRight = random8_to(26);

    mainController->getMoneyController()->removeFromCredit(20);
    lampData[LMP_START].lampState = LampState::off;

    this->mainController->getDisplayController()->setText("    LET IT GO!!     ");
    this->mainController->getDisplayController()->displayText();

    mainController->getReelController()->shuffle(reelStopLeft, reelStopCentre, reelStopRight);

    while (mainController->getReelController()->isCommandInProgress()) {
        vTaskDelay(pdMS_TO_TICKS(50));
        this->moves = random8_to(12);
        //this->mainController->getDisplayController()->setMoves(this->moves);   // TODO: fix this
    }
}

void Game::playNudges(int nudges) {
    ESP_LOGI(TAG, "Entering playNudges(%d)", nudges);

    std::string nudgeText = "        NUDGE        ";

    mainController->getDisplayController()->setText(nudgeText);
    mainController->getDisplayController()->displayText();

    mainController->getDisplayController()->resetLampData(false);
    for (int i = 0; i < nudges; i++) {
        lampData[DisplayController::NUDGE_LAMPS[i]].lampState = LampState::on;
        lampData[DisplayController::NUDGE_LAMPS[i]].rgb.r = 0;
        lampData[DisplayController::NUDGE_LAMPS[i]].rgb.g = 0;
        lampData[DisplayController::NUDGE_LAMPS[i]].rgb.b = 255;
    }
    lampData[DisplayController::NUDGE_LAMPS[nudges]].lampState = LampState::blinkfast;

    bool win;
    
    while (nudges > 0) {
        
        win = isWinningLine();

        lampData[LMP_HOLD].lampState = LampState::blinkfast;
        lampData[LMP_HOLD_HI].lampState = LampState::blinkfast;
        lampData[LMP_HOLD_LO].lampState = LampState::blinkfast;

        if (win) {           
            break;
        }

        uint8_t btnStatus = mainController->getDisplayController()->getButtonStatus();

        // loop waiting for button press.
        while ((!win && (!(btnStatus & (BTN_HOLD_LO | BTN_HOLD | BTN_HOLD_HI)))) || (win && (!(btnStatus & (BTN_TRANSFER | BTN_HOLD_LO | BTN_HOLD | BTN_HOLD_HI | BTN_START))))) {
            btnStatus = mainController->getDisplayController()->getButtonStatus();
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (win && (btnStatus & BTN_TRANSFER)) {            
            mainController->getMoneyController()->addToBank(mainController->getMoneyController()->getTransfer());
            mainController->getMoneyController()->setTransfer(0);
            mainController->getAudioController()->playAudioFile(Sounds::SND_KERCHING);
            return;
        } else if (win && (btnStatus & BTN_START)) {
            playFeatureMatrix();
            return;
        }

        nudges--;

        mainController->getDisplayController()->setText(nudgeText);
        mainController->getDisplayController()->displayText();

        mainController->getDisplayController()->resetLampData(false);
        for (int i = 0; i < nudges; i++) {
            lampData[DisplayController::NUDGE_LAMPS[i]].lampState = LampState::on;
            lampData[DisplayController::NUDGE_LAMPS[i]].rgb.r = 0;
            lampData[DisplayController::NUDGE_LAMPS[i]].rgb.g = 0;
            lampData[DisplayController::NUDGE_LAMPS[i]].rgb.b = 255;
        }
        lampData[nudges].lampState = LampState::blinkfast;

        lampData[LMP_HOLD].lampState = LampState::blinkfast;
        lampData[LMP_HOLD_LO].lampState = LampState::blinkfast;
        lampData[LMP_HOLD_HI].lampState = LampState::blinkfast;


        if ((btnStatus & BTN_HOLD_LO) == BTN_HOLD_LO) {            
            mainController->getReelController()->nudge(1, 0, 0);
        } else if ((btnStatus & BTN_HOLD) == BTN_HOLD) {            
            mainController->getReelController()->nudge(0, 1, 0);
        } else if ((btnStatus & BTN_HOLD_HI) == BTN_HOLD_HI) {            
            mainController->getReelController()->nudge(0, 0, 1);
        }

        // wait for reel controller to finish command
        while (mainController->getReelController()->isCommandInProgress()) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    mainController->getDisplayController()->resetLampData(true);

    if (isWinningLine()) {
        transferOrGamble();
    } else {
        mainController->getAudioController()->playAudioFile(Sounds::SND_LOSE);
    }

    ESP_LOGI(TAG, "Exiting nudges()");
}

bool Game::offerHold() {
    ESP_LOGI(TAG, "Entering offerHold()");

    uint8_t leftPos;
    uint8_t centrePos;
    uint8_t rightPos;

    uint8_t hold = random8_to(2); //0 or 1

    mainController->getReelController()->getReelPositions(leftPos, centrePos, rightPos);

    uint8_t leftSymbolId = symbolsLeftReel[leftPos];
    uint8_t centreSymbolId = symbolsCentreReel[centrePos];
    uint8_t rightSymbolId = symbolsRightReel[rightPos];
    
    bool result = (((leftSymbolId == centreSymbolId) || (leftSymbolId == rightSymbolId) || (centreSymbolId == rightSymbolId)) && hold > 0); 
    
    ESP_LOGI(TAG, "Exiting offerHold(). Returning %s", result ? "true" : "false");
    
    return result;
}

void Game::transferOrGamble() {
    ESP_LOGI(TAG, "Entering transferOrGamble()");

    lampData[LMP_TRANSFER].lampState = LampState::blinkfast;
    lampData[LMP_START].lampState = LampState::blinkslow;

    uint8_t btnStatus = mainController->getDisplayController()->getButtonStatus();

    // loop waiting for button press.
    while (!(btnStatus & (BTN_TRANSFER | BTN_START))) {
        btnStatus = mainController->getDisplayController()->getButtonStatus();
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    lampData[LMP_START].lampState = LampState::off;
    lampData[LMP_COLLECT].lampState = LampState::off;
    lampData[LMP_TRANSFER].lampState = LampState::off;
    lampData[LMP_HOLD_LO].lampState = LampState::off;
    lampData[LMP_HOLD].lampState = LampState::off;
    lampData[LMP_HOLD_HI].lampState = LampState::off;


    if (btnStatus & BTN_TRANSFER) {
        mainController->getMoneyController()->moveTransferToBank();
        mainController->getAudioController()->playAudioFile(Sounds::SND_KERCHING);
    } else if (btnStatus & BTN_START) {
        playFeatureMatrix();
    }

    ESP_LOGI(TAG, "Exiting transferOrGamble()");
}

void Game::collectOrContinue() {

    ESP_LOGI(TAG, "Entering collectOrContinue()");

    lampData[LMP_START].lampState = LampState::blinkfast;
    lampData[LMP_COLLECT].lampState = LampState::blinkslow;

    uint8_t btnStatus = mainController->getDisplayController()->getButtonStatus();

    // loop waiting for button press.
    while (!(btnStatus & (BTN_COLLECT | BTN_START))) {
        btnStatus = mainController->getDisplayController()->getButtonStatus();
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    lampData[LMP_START].lampState = LampState::off;
    lampData[LMP_COLLECT].lampState = LampState::off;

    if ((btnStatus & BTN_COLLECT)) {
        mainController->payout();
    } else if ((btnStatus & BTN_START)) {
        mainController->getMoneyController()->moveBankToCredit();
    }

    ESP_LOGI(TAG, "Exiting collectOrContinue()");
}

bool Game::isWinningLine() {
    ESP_LOGI(TAG, "Entering isWinningLine()");
    uint8_t leftPos;
    uint8_t centrePos;
    uint8_t rightPos;

    bool isWin = false;

    mainController->getReelController()->getReelPositions(leftPos, centrePos, rightPos);

    uint8_t leftSymbolId = symbolsLeftReel[leftPos];
    uint8_t centreSymbolId = symbolsCentreReel[centrePos];
    uint8_t rightSymbolId = symbolsRightReel[rightPos];

    ESP_LOGI(TAG, "Reel positions: %s - %s - %s", Game::symbolMap[leftSymbolId].c_str(), Game::symbolMap[centreSymbolId].c_str(), Game::symbolMap[rightSymbolId].c_str());

    for (uint8_t i = 0; i < 7; i++) {

        if (((leftSymbolId == winningCombinations[i].leftSymbolId) || (winningCombinations[i].leftSymbolId == 255)) &&
                ((centreSymbolId == winningCombinations[i].centreSymbolId) || (winningCombinations[i].centreSymbolId == 255)) &&
                ((rightSymbolId == winningCombinations[i].rightSymbolId) || (winningCombinations[i].rightSymbolId == 255))) {
            mainController->getMoneyController()->setTransfer(this->winningCombinations[i].amount);
            isWin = true;
            break;
        }
    }

    ESP_LOGI(TAG, "Exiting isWinningLine()");
    return isWin;
}

void Game::playFeatureMatrix() {
    uint8_t featureIndex = 0;
    mainController->getAudioController()->playAudioFile(Sounds::SND_LET_IT_GO);
    lampData = mainController->getDisplayController()->getLampData();
    lampData[LMP_START].lampState = LampState::blinkslow;

    // loop waiting for button press.
    uint8_t btnStatus = mainController->getDisplayController()->getButtonStatus();
    while ((btnStatus & (BTN_START)) == 0) {
        featureIndex = random8_to(13); // number of features

        lampData[DisplayController::FEATURE_LAMPS[featureIndex]].lampState = LampState::on;

        btnStatus = mainController->getDisplayController()->getButtonStatus();

        // TODO: do something here.

        lampData[DisplayController::FEATURE_LAMPS[featureIndex]].lampState = LampState::off;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Feature has been chosen, let's continue...
    switch (featureIndex) {
            // String featureMap[12] = {"Free Spin", "Double Money", "Shuffle", "Lose", "Palace", "Palace", "Shuffle", "Lose", "Free Spin", "Hi/Lo", "Free Spin", "Palace"};
        case 0:
        case 8:
        case 10:
            // Free Spin
            playFreeSpin();
            break;
        case 1:
            // Double Money
            break;
        case 2:
        case 6:
            // Shuffle
            playShuffle();
            break;
        case 3:
        case 7:
            // Lose
            break;
        case 4:
        case 5:
        case 11:
            // Palace
            playTrail();
            break;
        case 9:
            // Hi/Lo
            playHiLo();
            break;
    }
}

void Game::playTrail() {
    uint8_t index = 0;
    while (PRIZE_TRAIL_PRIZES[index] < mainController->getMoneyController()->getTransfer() && index < PRIZE_TRAIL_PRIZES_LENGTH) {
        index++;
    }

    //mainController->getDisplayController()->lampsOn(mainController->getDisplayController()->TRAIL_LAMPS[index], false);
}

void Game::playHiLo() {
}

void Game::playShuffle() {
    uint8_t btnStatus = mainController->getDisplayController()->getButtonStatus();

    mainController->getDisplayController()->setText("      SHUFFLE!      ");
    mainController->getDisplayController()->displayText();
    // loop waiting for button press.
    while ((btnStatus & BTN_START) != BTN_START) {
        btnStatus = this->mainController->getDisplayController()->getButtonStatus();
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    mainController->getAudioController()->playAudioFile(Sounds::SND_NOW_THATS_ICE);    

    shuffleReels();

    if (isWinningLine()) {
        transferOrGamble();
    } else {
        mainController->getAudioController()->playAudioFile(Sounds::SND_LOSE);
    }

    lampData = mainController->getDisplayController()->getLampData();
    lampData[LMP_START].lampState = LampState::off;
    lampData[LMP_COLLECT].lampState = LampState::off;
    lampData[LMP_HOLD_LO].lampState = LampState::off;
    lampData[LMP_HOLD].lampState = LampState::off;
    lampData[LMP_HOLD_HI].lampState = LampState::off;
}

void Game::playFreeSpin() {
    uint8_t btnStatus = mainController->getDisplayController()->getButtonStatus();

    mainController->getDisplayController()->setText("     FREE SPIN!     ");
    mainController->getDisplayController()->displayText();
    // loop waiting for button press.
    while ((btnStatus & BTN_START) != BTN_START) {
        btnStatus = this->mainController->getDisplayController()->getButtonStatus();
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    mainController->getAudioController()->playAudioFile(Sounds::SND_NOW_THATS_ICE);

    spinReels();

    if (isWinningLine()) {
        transferOrGamble();
    } else {
        mainController->getAudioController()->playAudioFile(Sounds::SND_LOSE);
    }

    lampData = mainController->getDisplayController()->getLampData();
    lampData[LMP_START].lampState = LampState::off;
    lampData[LMP_COLLECT].lampState = LampState::off;
    lampData[LMP_TRANSFER].lampState = LampState::off;
    lampData[LMP_HOLD_LO].lampState = LampState::off;
    lampData[LMP_HOLD].lampState = LampState::off;
    lampData[LMP_HOLD_HI].lampState = LampState::off;
}

bool Game::isGameInProgress() {
    return this->isInProgress;
}
