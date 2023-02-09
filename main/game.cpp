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
#include <cstring>
#include <cstddef>
#include <bitset>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "color.h"

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

    random16_add_entropy(esp_random() >> 16);
    random16_add_entropy(esp_random() >> 16);
    
    mainController->getDisplayController()->resetLampData();
    if (mainController->getDisplayController()->isAttractMode()) {
        mainController->getDisplayController()->stopAttractMode();
    }
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

    mainController->getDisplayController()->displayText("PRESS START TO BEGIN");

    // loop waiting for button press.
    std::bitset<8> btnStatus = mainController->getDisplayController()->getButtonStatus();
    while (!btnStatus.test(BTN_START)) {
        btnStatus = this->mainController->getDisplayController()->getButtonStatus();
        vTaskDelay(pdMS_TO_TICKS(100)); // let the processor do something else

        holdLeft = false;
        holdCentre = false;
        holdRight = false;

        if (hold) {
            if (btnStatus.test(BTN_HOLD)) {
                holdCentre = true;
                lampData[LMP_HOLD].lampState = LampState::on;
                lampData[LMP_COLLECT].lampState = LampState::blinkslow;
            } else if (btnStatus.test(BTN_HOLD_HI)) {
                holdLeft = true;
                lampData[LMP_HOLD_HI].lampState = LampState::on;
                lampData[LMP_COLLECT].lampState = LampState::blinkslow;
            } else if (btnStatus.test(BTN_HOLD_LO)) {
                holdRight = true;
                lampData[LMP_HOLD_LO].lampState = LampState::on;
                lampData[LMP_COLLECT].lampState = LampState::blinkslow;
            }

            if (btnStatus.test(BTN_COLLECT)) { // Cancel
                lampData[LMP_COLLECT].lampState = LampState::off;
                lampData[LMP_HOLD_LO].lampState = LampState::blinkslow;
                lampData[LMP_HOLD].lampState = LampState::blinkslow;
                lampData[LMP_HOLD_HI].lampState = LampState::blinkslow;
                holdLeft = false;
                holdCentre = false;
                holdRight = false;
            }
        }

    }
    
    // Switch off hold lights for reels that are not held
    if (!holdLeft) lampData[LMP_HOLD_HI].lampState = LampState::off;
    if (!holdCentre) lampData[LMP_HOLD].lampState = LampState::off;
    if (!holdRight) lampData[LMP_HOLD_LO].lampState = LampState::off;
    lampData[LMP_COLLECT].lampState = LampState::off;
    lampData[LMP_START].lampState = LampState::off;
    
    mainController->getMoneyController()->incrementGameCount();
    mainController->getMoneyController()->removeFromCredit(20);
    mainController->getAudioController()->playAudioFileSync(Sounds::SND_NOW_THATS_ICE);

    spinReels(holdLeft, holdCentre, holdRight);

    if (isWinningLine()) {
        transferOrGamble();
    } else if (nudges > 0) {
        playNudges(nudges);
    } else {
        ESP_LOGI(TAG, "Returning from game to main loop");
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

void Game::spinReels(bool holdLeft, bool holdCentre, bool holdRight) {

    ESP_LOGI(TAG, "Entering spinReels()");

    uint8_t reelStopLeft;
    uint8_t reelStopCentre;
    uint8_t reelStopRight;
    
    mainController->getReelController()->getReelPositions(reelStopLeft, reelStopCentre, reelStopRight);
    
    if (!holdLeft) reelStopLeft = random8_to(26);
    if (!holdCentre) reelStopCentre = random8_to(26);
    if (!holdRight) reelStopRight = random8_to(26);
    
    lampData[LMP_START].lampState = LampState::off;

    this->mainController->getDisplayController()->displayText("    LET IT GO!!     ");

    mainController->getReelController()->spin(reelStopLeft, reelStopCentre, reelStopRight);

    while (mainController->getReelController()->isCommandInProgress()) {
        vTaskDelay(pdMS_TO_TICKS(50));
        this->moves = random8_to(13);
        this->mainController->getDisplayController()->setMoves(this->moves); // TODO: fix this
    }

    ESP_LOGI(TAG, "Exiting spinReels()");
}

void Game::shuffleReels() {
    uint8_t reelStopLeft = random8_to(26);
    uint8_t reelStopCentre = random8_to(26);
    uint8_t reelStopRight = random8_to(26);

    mainController->getMoneyController()->removeFromCredit(20);
    lampData[LMP_START].lampState = LampState::off;

    this->mainController->getDisplayController()->displayText("    LET IT GO!!     ");

    mainController->getReelController()->shuffle(reelStopLeft, reelStopCentre, reelStopRight);

    while (mainController->getReelController()->isCommandInProgress()) {
        vTaskDelay(pdMS_TO_TICKS(50));
        this->moves = random8_to(12);
        this->mainController->getDisplayController()->setMoves(this->moves);   // TODO: fix this
    }
}

void Game::playNudges(int nudges) {
    ESP_LOGI(TAG, "Entering playNudges(%d)", nudges);

    std::string nudgeText = "        NUDGE        ";

    mainController->getDisplayController()->displayText(nudgeText);

    bool win = false;

    while (nudges > 0) {

        mainController->getDisplayController()->resetLampData();

        // have to take off 1, because array is zero-indexed
        lampData[DisplayController::NUDGE_LAMPS[nudges - 1]].rgb.r = 255;
        lampData[DisplayController::NUDGE_LAMPS[nudges - 1]].rgb.g = 255;
        lampData[DisplayController::NUDGE_LAMPS[nudges - 1]].rgb.b = 255;
        lampData[DisplayController::NUDGE_LAMPS[nudges - 1]].lampState = LampState::blinkfast;

        if (nudges > 1) {
            for (int i = 0; i < (nudges - 1); i++) {
                lampData[DisplayController::NUDGE_LAMPS[i]].lampState = LampState::on;
                lampData[DisplayController::NUDGE_LAMPS[i]].rgb.r = 0;
                lampData[DisplayController::NUDGE_LAMPS[i]].rgb.g = 0;
                lampData[DisplayController::NUDGE_LAMPS[i]].rgb.b = 255;
            }
        }

        lampData[LMP_HOLD].lampState = LampState::blinkfast;
        lampData[LMP_HOLD_HI].lampState = LampState::blinkfast;
        lampData[LMP_HOLD_LO].lampState = LampState::blinkfast;

        std::bitset<8> btnStatus = mainController->getDisplayController()->getButtonStatus();

        // loop waiting for button press.
        while (

                (!btnStatus.test(BTN_HOLD_LO)) &&
                (!btnStatus.test(BTN_HOLD)) &&
                (!btnStatus.test(BTN_HOLD_HI))) {

            btnStatus = mainController->getDisplayController()->getButtonStatus();
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (btnStatus.test(BTN_HOLD_LO)) {
            mainController->getReelController()->nudge(1, 0, 0);
        } else if (btnStatus.test(BTN_HOLD)) {
            mainController->getReelController()->nudge(0, 1, 0);
        } else if (btnStatus.test(BTN_HOLD_HI)) {
            mainController->getReelController()->nudge(0, 0, 1);
        }

        // wait for reel controller to finish command
        while (mainController->getReelController()->isCommandInProgress()) {
            vTaskDelay(pdMS_TO_TICKS(75));
        }

        win = isWinningLine();

        if (win) {
            transferOrGamble();
            return;
        } else {
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        nudges--;

    }

    mainController->getDisplayController()->resetLampData();

    if (isWinningLine()) {
        transferOrGamble();
    } else {
        mainController->getAudioController()->playAudioFileSync(Sounds::SND_LOSE);
    }

    ESP_LOGD(TAG, "Exiting nudges()");
}

bool Game::offerHold() {
    ESP_LOGD(TAG, "Entering offerHold()");

    uint8_t leftPos;
    uint8_t centrePos;
    uint8_t rightPos;

    uint8_t hold = random8_to(2); //0 or 1

    mainController->getReelController()->getReelPositions(leftPos, centrePos, rightPos);

    uint8_t leftSymbolId = symbolsLeftReel[leftPos];
    uint8_t centreSymbolId = symbolsCentreReel[centrePos];
    uint8_t rightSymbolId = symbolsRightReel[rightPos];

    bool result = (((leftSymbolId == centreSymbolId) || (leftSymbolId == rightSymbolId) || (centreSymbolId == rightSymbolId)) && hold > 0);

    ESP_LOGD(TAG, "Exiting offerHold(). Returning %s", result ? "true" : "false");

    return result;
}

void Game::transferOrGamble() {
    ESP_LOGD(TAG, "Entering transferOrGamble()");

    lampData[LMP_TRANSFER].lampState = LampState::blinkfast;
    lampData[LMP_START].lampState = LampState::blinkslow;

    std::bitset<8> btnStatus = mainController->getDisplayController()->getButtonStatus();

    // loop waiting for button press.
    while (!btnStatus.test(BTN_TRANSFER) && !btnStatus.test(BTN_START)) {
        btnStatus = mainController->getDisplayController()->getButtonStatus();
        vTaskDelay(pdMS_TO_TICKS(75));
    }

    lampData[LMP_START].lampState = LampState::off;
    lampData[LMP_COLLECT].lampState = LampState::off;
    lampData[LMP_TRANSFER].lampState = LampState::off;
    lampData[LMP_HOLD_LO].lampState = LampState::off;
    lampData[LMP_HOLD].lampState = LampState::off;
    lampData[LMP_HOLD_HI].lampState = LampState::off;


    if (btnStatus.test(BTN_TRANSFER)) {
        mainController->getMoneyController()->moveTransferToBank();
        mainController->getAudioController()->playAudioFileSync(Sounds::SND_KERCHING);
    } else if (btnStatus.test(BTN_START)) {
        playFeatureMatrix();
    }

    ESP_LOGD(TAG, "Exiting transferOrGamble()");
}

void Game::collectOrContinue() {

    ESP_LOGD(TAG, "Entering collectOrContinue()");

    lampData[LMP_START].lampState = LampState::blinkfast;
    lampData[LMP_COLLECT].lampState = LampState::blinkslow;

    std::bitset<8> btnStatus = mainController->getDisplayController()->getButtonStatus();

    // loop waiting for button press.
    while (!btnStatus.test(BTN_COLLECT) && !btnStatus.test(BTN_START)) {
        btnStatus = mainController->getDisplayController()->getButtonStatus();
        vTaskDelay(pdMS_TO_TICKS(75));
    }

    lampData[LMP_START].lampState = LampState::off;
    lampData[LMP_COLLECT].lampState = LampState::off;

    if (btnStatus.test(BTN_COLLECT)) {
        // TODO: Sort this out when cctalk finished
        //mainController->payout();
    } else if (btnStatus.test(BTN_START)) {
        mainController->getMoneyController()->moveBankToCredit();
    }

    ESP_LOGD(TAG, "Exiting collectOrContinue()");
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

    for (int i = 0; i < 7; i++) {

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
    std::bitset<8> btnStatus = mainController->getDisplayController()->getButtonStatus();
    while (!btnStatus.test(BTN_START)) {
        featureIndex = random8_to(13); // number of features

        lampData[DisplayController::FEATURE_LAMPS[featureIndex]].lampState = LampState::on;

        btnStatus = mainController->getDisplayController()->getButtonStatus();

        // TODO: do something here.

        lampData[DisplayController::FEATURE_LAMPS[featureIndex]].lampState = LampState::off;
        vTaskDelay(pdMS_TO_TICKS(100));
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
    std::bitset<8> btnStatus = mainController->getDisplayController()->getButtonStatus();

    mainController->getDisplayController()->displayText("      SHUFFLE!      ");

    // loop waiting for button press.
    while (!btnStatus.test(BTN_START)) {
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
    std::bitset<8> btnStatus = mainController->getDisplayController()->getButtonStatus();

    mainController->getDisplayController()->displayText("     FREE SPIN!     ");

    // loop waiting for button press.
    while (!btnStatus.test(BTN_START)) {
        btnStatus = this->mainController->getDisplayController()->getButtonStatus();
        vTaskDelay(pdMS_TO_TICKS(75));
    }

    mainController->getAudioController()->playAudioFile(Sounds::SND_NOW_THATS_ICE);

    spinReels(false, false, false); // no holds

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
