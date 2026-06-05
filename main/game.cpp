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

/*
 * File:   Game.cpp
 * Author: chris
 *
 * Created on January 14, 2018, 3:02 PM
 * Updated for ESP-IDF October 1st, 2021, 11:26 AM
 */
#include <array>
#include <bitset>

#include <string>
#include "esp_random.h"
#include "freertos/FreeRTOS.h"

#include "lib8tion/random8.h"

#include "game.h"

using namespace std;

static const char* TAG = "Game";

Game::Game(DisplayController& displayController,
           AudioController& audioController, PaymentController& paymentController,
           ReelController& reelController)
    : mainController(nullptr),
      displayController(displayController),
      audioController(audioController),
      paymentController(paymentController), reelController(reelController), moves(0)
{
    ESP_LOGI(TAG, "Entering constructor");

    this->isInProgress = false;

    ESP_LOGI(TAG, "Leaving constructor");
}

Game::~Game()
= default;

void Game::initialise()
{
}

void Game::start()
{
    ESP_LOGI(TAG, "Beginning game");

    this->isInProgress = true;

    this->displayController.stopAttractMode();
    this->displayController.resetLampData();
    this->audioController.stopPlaying();

    playNormalSpin();

    // payout
    if ((this->paymentController.getBank() > 0) &&
        (this->paymentController.getCredit() < 20))
    {
        collectOrContinue();
    }

    isInProgress = false;

    ESP_LOGI(TAG, "Exiting game");
}

void Game::spinReels(bool holdLeft, bool holdCentre, bool holdRight) const
{
    ESP_LOGI(TAG, "Entering spinReels()");

    uint8_t reelStopLeft = holdLeft ? 0 : frand::random8(1, 25);
    uint8_t reelStopCentre = holdCentre ? 0 : frand::random8(1, 25);
    uint8_t reelStopRight = holdRight ? 0 : frand::random8(1, 25);

    this->displayController.getLampData()
        .at(DisplayController::LMP_START)
        .lampState = LampState::off;

    displayController.displayVFDText("    LET IT GO!!     ");
    this->audioController.playAudioFileAsync(Sounds::SND_LET_IT_GO);

    this->reelController.spin(reelStopLeft, reelStopCentre, reelStopRight);

    ESP_LOGI(TAG, "Exiting spinReels()");
}

void Game::shuffleReels(bool holdLeft, bool holdCentre, bool holdRight) const
{
    ESP_LOGI(TAG, "Entering shuffleReels()");

    uint8_t reelStopLeft = holdLeft ? 0 : frand::random8(1, 25);
    uint8_t reelStopCentre = holdCentre ? 0 : frand::random8(1, 25);
    uint8_t reelStopRight = holdRight ? 0 : frand::random8(1, 25);

    this->displayController.getLampData()
        .at(DisplayController::LMP_START)
        .lampState = LampState::off;

    displayController.displayVFDText("    LET IT GO!!     ");
    this->audioController.playAudioFileAsync(Sounds::SND_LET_IT_GO);

    this->reelController.shuffle(reelStopLeft, reelStopCentre, reelStopRight);

    ESP_LOGI(TAG, "Exiting shuffleReels()");
}

void Game::playNormalSpin()
{
    uint8_t nudges = frand::random8(5); // 0 - 5
    bool hold = offerHold();

    this->displayController.getLampData()
        .at(DisplayController::REEL_LAMP_L2)
        .lampState = LampState::on;
    this->displayController.getLampData()
        .at(DisplayController::REEL_LAMP_C2)
        .lampState = LampState::on;
    this->displayController.getLampData()
        .at(DisplayController::REEL_LAMP_R2)
        .lampState = LampState::on;

    if (hold)
    {
        this->displayController.getLampData()
            .at(DisplayController::LMP_HOLD_LO)
            .lampState = LampState::blinkslow;
        this->displayController.getLampData()
            .at(DisplayController::LMP_HOLD)
            .lampState = LampState::blinkslow;
        this->displayController.getLampData()
            .at(DisplayController::LMP_HOLD_HI)
            .lampState = LampState::blinkslow;
    }

    this->displayController.getLampData()
        .at(DisplayController::LMP_START)
        .lampState = LampState::blinkslow;
    displayController.displayVFDText("PRESS START TO BEGIN");

    // loop waiting for button press.
    bool holdLeft = false;
    bool holdCentre = false;
    bool holdRight = false;

    std::bitset<8> btnStatus = this->displayController.getButtonStatus();
    while (!btnStatus.test(BTN_START))
    {
        if (hold)
        {
            if (btnStatus.test(BTN_HOLD))
            {
                holdCentre = true;
                this->displayController.getLampData()
                    .at(DisplayController::LMP_HOLD)
                    .lampState = LampState::on;
                this->audioController.playAudioFile(Sounds::SND_REEL_STOP);
                this->displayController.getLampData()
                    .at(DisplayController::LMP_COLLECT)
                    .lampState = LampState::blinkslow;
            }
            else if (btnStatus.test(BTN_HOLD_HI))
            {
                holdLeft = true;
                this->displayController.getLampData()
                    .at(DisplayController::LMP_HOLD_HI)
                    .lampState = LampState::on;
                this->audioController.playAudioFile(Sounds::SND_REEL_STOP);
                this->displayController.getLampData()
                    .at(DisplayController::LMP_COLLECT)
                    .lampState = LampState::blinkslow;
            }
            else if (btnStatus.test(BTN_HOLD_LO))
            {
                holdRight = true;
                this->displayController.getLampData()
                    .at(DisplayController::LMP_HOLD_LO)
                    .lampState = LampState::on;
                this->audioController.playAudioFile(Sounds::SND_REEL_STOP);
                this->displayController.getLampData()
                    .at(DisplayController::LMP_COLLECT)
                    .lampState = LampState::blinkslow;
            }

            if (btnStatus.test(BTN_COLLECT))
            {
                // Cancel
                this->displayController.getLampData()
                    .at(DisplayController::LMP_COLLECT)
                    .lampState = LampState::off;
                this->displayController.getLampData()
                    .at(DisplayController::LMP_HOLD_LO)
                    .lampState = LampState::blinkslow;
                this->displayController.getLampData()
                    .at(DisplayController::LMP_HOLD)
                    .lampState = LampState::blinkslow;
                this->displayController.getLampData()
                    .at(DisplayController::LMP_HOLD_HI)
                    .lampState = LampState::blinkslow;
                holdLeft = false;
                holdCentre = false;
                holdRight = false;
            }
        }

        btnStatus = this->displayController.getButtonStatus();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Switch off hold lights for reels that are not held
    if (!holdLeft)
        this->displayController.getLampData()
            .at(DisplayController::LMP_HOLD_HI)
            .lampState = LampState::off;
    if (!holdCentre)
        this->displayController.getLampData()
            .at(DisplayController::LMP_HOLD)
            .lampState = LampState::off;
    if (!holdRight)
        this->displayController.getLampData()
            .at(DisplayController::LMP_HOLD_LO)
            .lampState = LampState::off;
    this->displayController.getLampData()
        .at(DisplayController::LMP_COLLECT)
        .lampState = LampState::off;
    this->displayController.getLampData()
        .at(DisplayController::LMP_START)
        .lampState = LampState::off;

    this->paymentController.incrementGameCount();
    this->paymentController.removeFromCredit(20);

    spinReels(holdLeft, holdCentre, holdRight);

    if (isWinningLine())
    {
        transferOrGamble();
    }
    else if (nudges > 0)
    {
        playNudges(nudges);
    }
    else
    {
        ESP_LOGI(TAG, "Returning from game to main loop");
        this->audioController.playAudioFile(
            Sounds::SND_WONT_GET_AWAY_WITH_THIS);
    }
}

void Game::playNudges(int nudges)
{
    ESP_LOGI(TAG, "Entering playNudges(%d)", nudges);

    std::string nudgeText = "        NUDGE        ";

    this->audioController.playAudioFileAsync(Sounds::SND_THEYRE_TROLLS);

    displayController.displayVFDText(nudgeText);

    while (nudges > 0)
    {
        this->displayController.resetLampData();

        // have to take off 1, because array is zero-indexed
        this->displayController.getLampData()
            .at(DisplayController::NUDGE_LAMPS.at(nudges - 1))
            .rgb = CRGB(255, 255, 255);
        this->displayController.getLampData()
            .at(DisplayController::NUDGE_LAMPS.at(nudges - 1))
            .lampState = LampState::blinkfast;

        if (nudges > 1)
        {
            for (int i = 0; i < (nudges - 1); i++)
            {
                this->displayController.getLampData()
                    .at(DisplayController::NUDGE_LAMPS.at(i))
                    .rgb = CRGB(255, 255, 255);
                this->displayController.getLampData()
                    .at(DisplayController::NUDGE_LAMPS.at(i))
                    .lampState = LampState::on;
                this->displayController.getLampData()
                    .at(DisplayController::NUDGE_LAMPS.at(nudges - 1))
                    .rgb = CRGB(0, 0, 255);
                this->displayController.getLampData()
                    .at(DisplayController::NUDGE_LAMPS.at(nudges - 1))
                    .lampState = LampState::blinkfast;
            }
        }

        this->displayController.getLampData()
            .at(DisplayController::LMP_HOLD)
            .lampState = LampState::blinkfast;
        this->displayController.getLampData()
            .at(DisplayController::LMP_HOLD_HI)
            .lampState = LampState::blinkfast;
        this->displayController.getLampData()
            .at(DisplayController::LMP_HOLD_LO)
            .lampState = LampState::blinkfast;

        uint8_t btnStatus = this->displayController.waitForButton(
            BTN_HOLD_LO_MASK_BIT | BTN_HOLD_MASK_BIT | BTN_HOLD_HI_MASK_BIT);

        if ((btnStatus & BTN_HOLD_LO_MASK_BIT) == BTN_HOLD_LO_MASK_BIT)
        {
            this->reelController.nudge(0, 0, 1);
        }
        else if ((btnStatus & BTN_HOLD_MASK_BIT) == BTN_HOLD_MASK_BIT)
        {
            this->reelController.nudge(0, 1, 0);
        }
        else if ((btnStatus & BTN_HOLD_HI_MASK_BIT) == BTN_HOLD_HI_MASK_BIT)
        {
            this->reelController.nudge(1, 0, 0);
        }

        // wait for reel controller to finish command
        while (this->reelController.isCommandInProgress())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }

        if (isWinningLine())
        {
            transferOrGamble();
            return;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }

        nudges--;
    }

    this->displayController.resetLampData();

    if (isWinningLine())
    {
        transferOrGamble();
    }
    else
    {
        this->audioController.playAudioFile(Sounds::SND_WONT_GET_AWAY_WITH_THIS);
    }

    ESP_LOGD(TAG, "Exiting nudges()");
}

bool Game::offerHold() const
{
    ESP_LOGD(TAG, "Entering offerHold()");

    uint8_t leftPos;
    uint8_t centrePos;
    uint8_t rightPos;

    uint8_t hold = (frand::random8(10) == 1); // 1 in 10 chance

    leftPos = this->reelController.getReelStopInfo().leftStop;
    centrePos = this->reelController.getReelStopInfo().centreStop;
    rightPos = this->reelController.getReelStopInfo().rightStop;

    uint8_t leftSymbolId = symbolsLeftReel[leftPos];
    uint8_t centreSymbolId = symbolsCentreReel[centrePos];
    uint8_t rightSymbolId = symbolsRightReel[rightPos];

    bool result =
    (((leftSymbolId == centreSymbolId) || (leftSymbolId == rightSymbolId) ||
            (centreSymbolId == rightSymbolId)) &&
        !(((leftSymbolId == centreSymbolId) && (leftSymbolId == rightSymbolId) &&
            (centreSymbolId == rightSymbolId))) &&
        hold > 0);

    ESP_LOGD(TAG, "Exiting offerHold(). Returning %s", result ? "true" : "false");

    return result;
}

void Game::transferOrGamble()
{
    ESP_LOGD(TAG, "Entering transferOrGamble()");

    displayController.displayVFDText(" TRANSFER OR GAMBLE ");

    this->audioController.playAudioFileAsync(Sounds::SND_NOW_THATS_ICE);

    this->displayController.getLampData()
        .at(DisplayController::LMP_TRANSFER)
        .lampState = LampState::blinkfast;
    this->displayController.getLampData()
        .at(DisplayController::LMP_START)
        .lampState = LampState::blinkslow;

    uint8_t btnStatus = displayController.waitForButton(BTN_TRANSFER_MASK_BIT | BTN_START_MASK_BIT);

    this->displayController.getLampData()
        .at(DisplayController::LMP_START)
        .lampState = LampState::off;
    this->displayController.getLampData()
        .at(DisplayController::LMP_COLLECT)
        .lampState = LampState::off;
    this->displayController.getLampData()
        .at(DisplayController::LMP_TRANSFER)
        .lampState = LampState::off;
    this->displayController.getLampData()
        .at(DisplayController::LMP_HOLD_LO)
        .lampState = LampState::off;
    this->displayController.getLampData()
        .at(DisplayController::LMP_HOLD)
        .lampState = LampState::off;
    this->displayController.getLampData()
        .at(DisplayController::LMP_HOLD_HI)
        .lampState = LampState::off;

    if ((btnStatus & BTN_TRANSFER_MASK_BIT) == BTN_TRANSFER_MASK_BIT)
    {
        this->paymentController.moveTransferToBank();
        this->audioController.playAudioFile(Sounds::SND_KERCHING);
    }
    else if ((btnStatus & BTN_START_MASK_BIT) == BTN_START_MASK_BIT)
    {
        this->paymentController.moveTransferToBank();
        this->audioController.playAudioFile(Sounds::SND_KERCHING);
        playFeatureMatrix();
    }

    ESP_LOGD(TAG, "Exiting transferOrGamble()");
}

void Game::collectOrContinue() const
{
    ESP_LOGD(TAG, "Entering collectOrContinue()");
    displayController.displayVFDText("COLLECT OR CONTINUE");

    this->displayController.getLampData()
        .at(DisplayController::LMP_START)
        .lampState = LampState::blinkfast;
    this->displayController.getLampData()
        .at(DisplayController::LMP_COLLECT)
        .lampState = LampState::blinkslow;

    // loop waiting for button press.
    uint8_t btnStatus = displayController.waitForButton(BTN_COLLECT_MASK_BIT | BTN_START_MASK_BIT);

    this->displayController.getLampData()
        .at(DisplayController::LMP_START)
        .lampState = LampState::off;
    this->displayController.getLampData()
        .at(DisplayController::LMP_COLLECT)
        .lampState = LampState::off;

    if ((btnStatus & BTN_COLLECT_MASK_BIT) == BTN_COLLECT_MASK_BIT)
    {
        ESP_LOGI(TAG, "Calling payout...");

        this->paymentController.payoutBank();
    }
    else if ((btnStatus & BTN_START_MASK_BIT) == BTN_START_MASK_BIT)
    {
        this->paymentController.moveBankToCredit();
    }

    ESP_LOGD(TAG, "Exiting collectOrContinue()");
}

bool Game::isWinningLine() const
{
    ESP_LOGI(TAG, "Entering isWinningLine()");

    bool isWin = false;

    uint8_t leftPos = this->reelController.getReelStopInfo().leftStop;
    uint8_t centrePos = this->reelController.getReelStopInfo().centreStop;
    uint8_t rightPos = this->reelController.getReelStopInfo().rightStop;

    uint8_t leftSymbolId = symbolsLeftReel[leftPos - 1];
    uint8_t centreSymbolId = symbolsCentreReel[centrePos - 1];
    uint8_t rightSymbolId = symbolsRightReel[rightPos - 1];

    //    ESP_LOGI(TAG, "Reel positions: %s - %s - %s",
    //    this->symbolMap[leftSymbolId].c_str(),
    //    this->symbolMap[centreSymbolId].c_str(),
    //    this->symbolMap[rightSymbolId].c_str());

    for (int i = 0; i < 7; i++)
    {
        if (((leftSymbolId == winningCombinations[i].leftSymbolId) ||
                (winningCombinations[i].leftSymbolId == 255)) &&
            ((centreSymbolId == winningCombinations[i].centreSymbolId) ||
                (winningCombinations[i].centreSymbolId == 255)) &&
            ((rightSymbolId == winningCombinations[i].rightSymbolId) ||
                (winningCombinations[i].rightSymbolId == 255)))
        {
            this->paymentController.setTransfer(winningCombinations[i].amount);
            isWin = true;
            break;
        }
    }

    ESP_LOGI(TAG, "Exiting isWinningLine()");
    return isWin;
}

void Game::playFeatureMatrix()
{
    this->displayController.displayVFDText("   FEATURE MATRIX   ");
    uint8_t featureIndex = 0;
    this->audioController.playAudioFileAsync(Sounds::SND_COLDER_BY_THE_MINUTE);

    this->displayController.resetLampData();

    this->displayController.getLampData()
        .at(DisplayController::LMP_START)
        .lampState = LampState::blinkslow;

    // loop waiting for button press.
    std::bitset<8> btnStatus = this->displayController.getButtonStatus();
    while (!btnStatus.test(BTN_START))
    {
        featureIndex = frand::random8(12); // number of features

        displayController.getLampData().at(DisplayController::FEATURE_LAMPS.at(0)).lampState= LampState::off;
        displayController.getLampData().at(DisplayController::FEATURE_LAMPS.at(1)).lampState= LampState::off;
        displayController.getLampData().at(DisplayController::FEATURE_LAMPS.at(2)).lampState= LampState::off;
        displayController.getLampData().at(DisplayController::FEATURE_LAMPS.at(3)).lampState= LampState::off;
        displayController.getLampData().at(DisplayController::FEATURE_LAMPS.at(4)).lampState= LampState::off;
        displayController.getLampData().at(DisplayController::FEATURE_LAMPS.at(5)).lampState= LampState::off;
        displayController.getLampData().at(DisplayController::FEATURE_LAMPS.at(6)).lampState= LampState::off;
        displayController.getLampData().at(DisplayController::FEATURE_LAMPS.at(7)).lampState= LampState::off;
        displayController.getLampData().at(DisplayController::FEATURE_LAMPS.at(8)).lampState= LampState::off;
        displayController.getLampData().at(DisplayController::FEATURE_LAMPS.at(9)).lampState= LampState::off;
        displayController.getLampData().at(DisplayController::FEATURE_LAMPS.at(10)).lampState= LampState::off;
        displayController.getLampData().at(DisplayController::FEATURE_LAMPS.at(11)).lampState= LampState::off;

        this->displayController.getLampData()
            .at(DisplayController::FEATURE_LAMPS.at(featureIndex))
            .lampState = LampState::on;
        this->displayController.getLampData()
            .at(DisplayController::FEATURE_LAMPS.at(featureIndex))
            .activeRgb = Colour::White;

        btnStatus = this->displayController.getButtonStatus();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    this->displayController.getLampData()
            .at(DisplayController::LMP_START)
            .lampState = LampState::off;

    this->displayController.getLampData()
        .at(DisplayController::FEATURE_LAMPS.at(featureIndex))
        .lampState = LampState::off;

    // Feature has been chosen, let's continue...
    switch (featureIndex)
    {
    // String featureMap[12] = {"Free Spin", "Double Money", "Shuffle",
    // "Lose", "Palace", "Palace", "Shuffle", "Lose", "Free Spin", "Hi/Lo",
    // "Free Spin", "Palace"};
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
        this->paymentController.removeFromBank(this->paymentController.getTransfer()); // lose the current win
        this->audioController.playAudioFile(Sounds::SND_WONT_GET_AWAY_WITH_THIS);
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
    default:
        break;
    }

    this->displayController.resetLampData();
    this->displayController.clearText();
}

void Game::playTrail()
{

    displayController.displayVFDText("       TRAIL!       ");
    uint8_t index = 0;
    while (PRIZE_TRAIL_PRIZES[index] < this->paymentController.getTransfer() &&
        index < PRIZE_TRAIL_PRIZES_LENGTH)
    {
        index++;
    }

    // this->displayController.TRAIL_LAMPS[index],
    // false);
    this_thread::sleep_for(std::chrono::seconds(5)); // just for now
}

void Game::playHiLo()
{
    displayController.displayVFDText("      SHUFFLE!      ");

    this_thread::sleep_for(std::chrono::seconds(5)); // just for now

}

void Game::playShuffle()
{
    displayController.displayVFDText("      SHUFFLE!      ");

    // loop waiting for button press.
    bool holdLeft = false;
    bool holdCentre = false;
    bool holdRight = false;

    bool hold = offerHold();

    std::bitset<8> btnStatus = this->displayController.getButtonStatus();
    while (!btnStatus.test(BTN_START))
    {
        if (hold)
        {
            if (btnStatus.test(BTN_HOLD))
            {
                holdCentre = true;
                this->displayController.getLampData()
                    .at(DisplayController::LMP_HOLD)
                    .lampState = LampState::on;
                this->audioController.playAudioFile(Sounds::SND_REEL_STOP);
                this->displayController.getLampData()
                    .at(DisplayController::LMP_COLLECT)
                    .lampState = LampState::blinkslow;
            }
            else if (btnStatus.test(BTN_HOLD_HI))
            {
                holdLeft = true;
                this->displayController.getLampData()
                    .at(DisplayController::LMP_HOLD_HI)
                    .lampState = LampState::on;
                this->audioController.playAudioFile(Sounds::SND_REEL_STOP);
                this->displayController.getLampData()
                    .at(DisplayController::LMP_COLLECT)
                    .lampState = LampState::blinkslow;
            }
            else if (btnStatus.test(BTN_HOLD_LO))
            {
                holdRight = true;
                this->displayController.getLampData()
                    .at(DisplayController::LMP_HOLD_LO)
                    .lampState = LampState::on;
                this->audioController.playAudioFile(Sounds::SND_REEL_STOP);
                this->displayController.getLampData()
                    .at(DisplayController::LMP_COLLECT)
                    .lampState = LampState::blinkslow;
            }

            if (btnStatus.test(BTN_COLLECT))
            {
                // Cancel
                this->displayController.getLampData()
                    .at(DisplayController::LMP_COLLECT)
                    .lampState = LampState::off;
                this->displayController.getLampData()
                    .at(DisplayController::LMP_HOLD_LO)
                    .lampState = LampState::blinkslow;
                this->displayController.getLampData()
                    .at(DisplayController::LMP_HOLD)
                    .lampState = LampState::blinkslow;
                this->displayController.getLampData()
                    .at(DisplayController::LMP_HOLD_HI)
                    .lampState = LampState::blinkslow;
                holdLeft = false;
                holdCentre = false;
                holdRight = false;
            }
        }

        btnStatus = this->displayController.getButtonStatus();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Switch off hold lights for reels that are not held
    if (!holdLeft)
        this->displayController.getLampData()
            .at(DisplayController::LMP_HOLD_HI)
            .lampState = LampState::off;
    if (!holdCentre)
        this->displayController.getLampData()
            .at(DisplayController::LMP_HOLD)
            .lampState = LampState::off;
    if (!holdRight)
        this->displayController.getLampData()
            .at(DisplayController::LMP_HOLD_LO)
            .lampState = LampState::off;
    this->displayController.getLampData()
        .at(DisplayController::LMP_COLLECT)
        .lampState = LampState::off;
    this->displayController.getLampData()
        .at(DisplayController::LMP_START)
        .lampState = LampState::off;

    this->paymentController.incrementGameCount();
    this->paymentController.removeFromCredit(20);
    this->audioController.playAudioFileAsync(Sounds::SND_LET_IT_GO);

    shuffleReels(holdLeft, holdCentre, holdRight);

    if (isWinningLine())
    {
        transferOrGamble();
    }
    else
    {
        this->audioController.playAudioFile(Sounds::SND_WONT_GET_AWAY_WITH_THIS);
    }

    this->displayController.getLampData()
        .at(DisplayController::LMP_START)
        .lampState = LampState::off;
    this->displayController.getLampData()
        .at(DisplayController::LMP_COLLECT)
        .lampState = LampState::off;
    this->displayController.getLampData()
        .at(DisplayController::LMP_HOLD_LO)
        .lampState = LampState::off;
    this->displayController.getLampData()
        .at(DisplayController::LMP_HOLD)
        .lampState = LampState::off;
    this->displayController.getLampData()
        .at(DisplayController::LMP_HOLD_HI)
        .lampState = LampState::off;
}

void Game::playFreeSpin()
{
    displayController.displayVFDText("     FREE SPIN!     ");

    // loop waiting for button press.
    displayController.waitForButton(BTN_START_MASK_BIT);

    spinReels(false, false, false); // no holds

    if (isWinningLine())
    {
        transferOrGamble();
    }
    else
    {
        this->audioController.playAudioFile(Sounds::SND_WONT_GET_AWAY_WITH_THIS);
    }

    this->displayController.getLampData()
        .at(DisplayController::LMP_START)
        .lampState = LampState::off;
    this->displayController.getLampData()
        .at(DisplayController::LMP_COLLECT)
        .lampState = LampState::off;
    this->displayController.getLampData()
        .at(DisplayController::LMP_TRANSFER)
        .lampState = LampState::off;
    this->displayController.getLampData()
        .at(DisplayController::LMP_HOLD_LO)
        .lampState = LampState::off;
    this->displayController.getLampData()
        .at(DisplayController::LMP_HOLD)
        .lampState = LampState::off;
    this->displayController.getLampData()
        .at(DisplayController::LMP_HOLD_HI)
        .lampState = LampState::off;
}

bool Game::isGameInProgress() const { return this->isInProgress; }
