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

}

void Game::start() {

    ESP_LOGI(TAG, "Beginning game");

    this->isInProgress = true;

    random16_add_entropy(esp_random() >> 16);
    random16_add_entropy(esp_random() >> 16);

    mainController->getDisplayController()->resetLampData();
    mainController->getDisplayController()->stopAttractMode();
    mainController->getAudioController()->stopPlaying();

    uint8_t nudges = random8_to(6); // 0 - 5    
    bool hold = offerHold();

    mainController->getDisplayController()->getLampData().at(DisplayController::REEL_LAMP_L2).setLampState(LampState::on);
    mainController->getDisplayController()->getLampData().at(DisplayController::REEL_LAMP_C2).setLampState(LampState::on);
    mainController->getDisplayController()->getLampData().at(DisplayController::REEL_LAMP_R2).setLampState(LampState::on);

    if (hold) {
        mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD_LO).setLampState(LampState::blinkslow);
        mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD).setLampState(LampState::blinkslow);
        mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD_HI).setLampState(LampState::blinkslow);
    }

    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_START).setLampState(LampState::blinkslow);
    mainController->getDisplayController()->displayText("PRESS START TO BEGIN");

    // loop waiting for button press.
    holdLeft = false;
    holdCentre = false;
    holdRight = false;

    std::bitset<8> btnStatus = mainController->getDisplayController()->getButtonStatus();
    while (!btnStatus.test(BTN_START)) {

        if (hold) {
            if (btnStatus.test(BTN_HOLD)) {
                holdCentre = true;
                mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD).setLampState(LampState::on);
                mainController->getAudioController()->playAudioFile(Sounds::SND_REEL_STOP);
                mainController->getDisplayController()->getLampData().at(DisplayController::LMP_COLLECT).setLampState(LampState::blinkslow);
            } else if (btnStatus.test(BTN_HOLD_HI)) {
                holdLeft = true;
                mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD_HI).setLampState(LampState::on);
                mainController->getAudioController()->playAudioFile(Sounds::SND_REEL_STOP);
                mainController->getDisplayController()->getLampData().at(DisplayController::LMP_COLLECT).setLampState(LampState::blinkslow);
            } else if (btnStatus.test(BTN_HOLD_LO)) {
                holdRight = true;
                mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD_LO).setLampState(LampState::on);
                mainController->getAudioController()->playAudioFile(Sounds::SND_REEL_STOP);
                mainController->getDisplayController()->getLampData().at(DisplayController::LMP_COLLECT).setLampState(LampState::blinkslow);
            }

            if (btnStatus.test(BTN_COLLECT)) { // Cancel
                mainController->getDisplayController()->getLampData().at(DisplayController::LMP_COLLECT).setLampState(LampState::off);
                mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD_LO).setLampState(LampState::blinkslow);
                mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD).setLampState(LampState::blinkslow);
                mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD_HI).setLampState(LampState::blinkslow);
                holdLeft = false;
                holdCentre = false;
                holdRight = false;
            }
        }

        btnStatus = this->mainController->getDisplayController()->getButtonStatus();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Switch off hold lights for reels that are not held
    if (!holdLeft) mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD_HI).setLampState(LampState::off);
    if (!holdCentre) mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD).setLampState(LampState::off);
    if (!holdRight) mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD_LO).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_COLLECT).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_START).setLampState(LampState::off);

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

    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_START).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_COLLECT).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD_HI).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD_LO).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_TRANSFER).setLampState(LampState::off);

    // payout
    if ((mainController->getMoneyController()->getBank() > 0) && (mainController->getMoneyController()->getCredit() < 20)) {
        collectOrContinue();
    }

    Game::isInProgress = false;

    ESP_LOGI(TAG, "Exiting game");
}

void Game::spinReels(bool holdLeft, bool holdCentre, bool holdRight) {

    ESP_LOGI(TAG, "Entering spinReels()");

    uint8_t reelStopLeft = mainController->getReelController()->getReelStopInfo().leftStop;
    uint8_t reelStopCentre = mainController->getReelController()->getReelStopInfo().centreStop;
    uint8_t reelStopRight = mainController->getReelController()->getReelStopInfo().rightStop; 

    reelStopLeft = holdLeft ? 0 : random8_to(26);
    reelStopCentre = holdCentre ? 0 : random8_to(26);
    reelStopRight = holdRight ? 0 : random8_to(26);

    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_START).setLampState(LampState::off);

    this->mainController->getDisplayController()->displayText("    LET IT GO!!     ");

    mainController->getReelController()->spin(reelStopLeft, reelStopCentre, reelStopRight);

    // FIXME: this is never called, because spin waits for the reels to stop so the loop never executes
    while (mainController->getReelController()->isCommandInProgress()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
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
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_START).setLampState(LampState::off);

    this->mainController->getDisplayController()->displayText("    LET IT GO!!     ");

    mainController->getReelController()->shuffle(reelStopLeft, reelStopCentre, reelStopRight);

    // FIXME: this is never called, because spin waits for the reels to stop so the loop never executes
    while (mainController->getReelController()->isCommandInProgress()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        this->moves = random8_to(12);
        this->mainController->getDisplayController()->setMoves(this->moves); // TODO: fix this
    }
}

void Game::playNudges(int nudges) {
    ESP_LOGI(TAG, "Entering playNudges(%d)", nudges);

    std::string nudgeText = "        NUDGE        ";

    mainController->getDisplayController()->displayText(nudgeText);

    while (nudges > 0) {

        mainController->getDisplayController()->resetLampData();

        // have to take off 1, because array is zero-indexed        
        mainController->getDisplayController()->getLampData().at(DisplayController::NUDGE_LAMPS.at(nudges - 1)).setRgb(rgb_from_values(255, 255, 255));
        mainController->getDisplayController()->getLampData().at(DisplayController::NUDGE_LAMPS.at(nudges - 1)).setLampState(LampState::blinkfast);

        if (nudges > 1) {
            for (int i = 0; i < (nudges - 1); i++) {
                mainController->getDisplayController()->getLampData().at(DisplayController::NUDGE_LAMPS.at(i)).setLampState(LampState::on);
                mainController->getDisplayController()->getLampData().at(DisplayController::NUDGE_LAMPS.at(nudges - 1)).setRgb(rgb_from_values(0, 0, 255));
            }
        }

        mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD).setLampState(LampState::blinkfast);
        mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD_HI).setLampState(LampState::blinkfast);
        mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD_LO).setLampState(LampState::blinkfast);

        uint8_t btnStatus = mainController->getDisplayController()->waitForButton(BTN_HOLD_LO_MASK_BIT | BTN_HOLD_MASK_BIT | BTN_HOLD_HI_MASK_BIT);

        if ((btnStatus & BTN_HOLD_LO_MASK_BIT) == BTN_HOLD_LO_MASK_BIT) {
            mainController->getReelController()->nudge(0, 0, 1);
        } else if ((btnStatus & BTN_HOLD_MASK_BIT) == BTN_HOLD_MASK_BIT) {
            mainController->getReelController()->nudge(0, 1, 0);
        } else if ((btnStatus & BTN_HOLD_HI_MASK_BIT) == BTN_HOLD_HI_MASK_BIT) {
            mainController->getReelController()->nudge(1, 0, 0);
        }

        // wait for reel controller to finish command
        while (mainController->getReelController()->isCommandInProgress()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }

        if (isWinningLine()) {
            transferOrGamble();
            return;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
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

    leftPos = mainController->getReelController()->getReelStopInfo().leftStop;
    centrePos = mainController->getReelController()->getReelStopInfo().centreStop;
    rightPos = mainController->getReelController()->getReelStopInfo().rightStop;

    uint8_t leftSymbolId = symbolsLeftReel[leftPos];
    uint8_t centreSymbolId = symbolsCentreReel[centrePos];
    uint8_t rightSymbolId = symbolsRightReel[rightPos];

    bool result = (((leftSymbolId == centreSymbolId) || (leftSymbolId == rightSymbolId) || (centreSymbolId == rightSymbolId)) && hold > 0);

    ESP_LOGD(TAG, "Exiting offerHold(). Returning %s", result ? "true" : "false");

    return result;
}

void Game::transferOrGamble() {
    ESP_LOGD(TAG, "Entering transferOrGamble()");

    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_TRANSFER).setLampState(LampState::blinkfast);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_START).setLampState(LampState::blinkslow);

    std::bitset<8> btnStatus = mainController->getDisplayController()->getButtonStatus();

    // loop waiting for button press.
    while (!btnStatus.test(BTN_TRANSFER) && !btnStatus.test(BTN_START)) {
        btnStatus = mainController->getDisplayController()->getButtonStatus();
        std::this_thread::sleep_for(std::chrono::milliseconds(75));
    }

    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_START).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_COLLECT).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_TRANSFER).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD_LO).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD_HI).setLampState(LampState::off);


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

    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_START).setLampState(LampState::blinkfast);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_COLLECT).setLampState(LampState::blinkslow);

    std::bitset<8> btnStatus = mainController->getDisplayController()->getButtonStatus();

    // loop waiting for button press.
    while (!btnStatus.test(BTN_COLLECT) && !btnStatus.test(BTN_START)) {
        btnStatus = mainController->getDisplayController()->getButtonStatus();
        std::this_thread::sleep_for(std::chrono::milliseconds(75));
    }

    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_START).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_COLLECT).setLampState(LampState::off);

    if (btnStatus.test(BTN_COLLECT)) {
        ESP_LOGI(TAG, "Calling payout...");

        this->mainController->getCCTalkController()->hopper.dispenseCoins((this->mainController->getMoneyController()->getBank() / 20), [&](const std::string & error_msg) {
            // TODO: Check status and see how many coins were returned and remove these from bank. For now we will just set bank to 0 (and presume all coins were paid out)
            if (error_msg.size() > 0) {
                ESP_LOGE(TAG, "An error occurred during payout: %s", error_msg.c_str());
            } else {
                this->mainController->getMoneyController()->payout(this->mainController->getMoneyController()->getBank());
            }
        });
    } else if (btnStatus.test(BTN_START)) {
        mainController->getMoneyController()->moveBankToCredit();
    }

    ESP_LOGD(TAG, "Exiting collectOrContinue()");
}

bool Game::isWinningLine() {
    ESP_LOGI(TAG, "Entering isWinningLine()");
    
    bool isWin = false;

    uint8_t leftPos = mainController->getReelController()->getReelStopInfo().leftStop;
    uint8_t centrePos = mainController->getReelController()->getReelStopInfo().centreStop;
    uint8_t rightPos = mainController->getReelController()->getReelStopInfo().rightStop;

    uint8_t leftSymbolId = symbolsLeftReel[leftPos];
    uint8_t centreSymbolId = symbolsCentreReel[centrePos];
    uint8_t rightSymbolId = symbolsRightReel[rightPos];

    //ESP_LOGI(TAG, "Reel positions: %s - %s - %s", this->symbolMap[leftSymbolId].c_str(), this->symbolMap[centreSymbolId].c_str(), this->symbolMap[rightSymbolId].c_str());

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
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_START).setLampState(LampState::blinkslow);

    // loop waiting for button press.
    std::bitset<8> btnStatus = mainController->getDisplayController()->getButtonStatus();
    while (!btnStatus.test(BTN_START)) {
        featureIndex = random8_to(13); // number of features

        mainController->getDisplayController()->getLampData().at(DisplayController::FEATURE_LAMPS.at(featureIndex)).setLampState(LampState::on);

        btnStatus = mainController->getDisplayController()->getButtonStatus();

        // TODO: do something here.

        mainController->getDisplayController()->getLampData().at(DisplayController::FEATURE_LAMPS.at(featureIndex)).setLampState(LampState::off);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    mainController->getAudioController()->playAudioFile(Sounds::SND_NOW_THATS_ICE);

    shuffleReels();

    if (isWinningLine()) {
        transferOrGamble();
    } else {
        mainController->getAudioController()->playAudioFile(Sounds::SND_LOSE);
    }

    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_START).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_COLLECT).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD_LO).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD_HI).setLampState(LampState::off);
}

void Game::playFreeSpin() {
    std::bitset<8> btnStatus = mainController->getDisplayController()->getButtonStatus();

    mainController->getDisplayController()->displayText("     FREE SPIN!     ");

    // loop waiting for button press.
    while (!btnStatus.test(BTN_START)) {
        btnStatus = this->mainController->getDisplayController()->getButtonStatus();
        std::this_thread::sleep_for(std::chrono::milliseconds(75));
    }

    mainController->getAudioController()->playAudioFile(Sounds::SND_NOW_THATS_ICE);

    spinReels(false, false, false); // no holds

    if (isWinningLine()) {
        transferOrGamble();
    } else {
        mainController->getAudioController()->playAudioFile(Sounds::SND_LOSE);
    }

    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_START).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_COLLECT).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_TRANSFER).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD_LO).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD).setLampState(LampState::off);
    mainController->getDisplayController()->getLampData().at(DisplayController::LMP_HOLD_HI).setLampState(LampState::off);
}

bool Game::isGameInProgress() {
    return this->isInProgress;
}
