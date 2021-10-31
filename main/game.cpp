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
#include "entropy.h"


#include "game.h"

using namespace std;

Game::Game(MainController *mainController)
{
    mainController->getDisplayController()->resetLampData(); // switch everything off and set to defined state
    
    this->lampData = mainController->getDisplayController()->getLampData();
    this->mainController = mainController;
    this->isInProgress = false;
}

Game::Game(const Game &orig)
{
}

void Game::start()
{
    mainController->incrementGameCounter();

    this->isInProgress = true;

    // while (!Entropy.available())
    // {
    //     mainController->refreshStatus();
    // }

    uint32_t nudges = Entropy.random(6); // 0 - 5
    uint32_t hold = Entropy.random(2);    //0 or 1
    
    lampData[REEL_LAMP_L2].on = true;
    lampData[REEL_LAMP_C2].on = true;
    lampData[REEL_LAMP_R2].on = true;

    mainController->getDisplayController()->updateLampData(lampData, true);
    
        
    // TODO: implement stopPlaying method
    //mainController->getAudioController()->stopPlaying(); // in case any other music is currently playing.

    if (hold > 0)
    {
        lampData[LMP_START]->blinkspeed.blinkSlow = true;
        lampData[LMP_START]->blink = true;
        lampData[LMP_START]->on = true;
        lampData[LMP_HOLD_LO]->blinkspeed.blinkSlow = true;
        lampData[LMP_HOLD_LO]->blink = true;
        lampData[LMP_HOLD_LO]->on = true;
        lampData[LMP_HOLD]->blinkspeed.blinkSlow = true;
        lampData[LMP_HOLD]->blink = true;
        lampData[LMP_HOLD]->on = true;
        lampData[LMP_HOLD_HI]->blinkspeed.blinkSlow = true;
        lampData[LMP_HOLD_HI]->blink = true;
        lampData[LMP_HOLD_HI]->on = true;
                                
        mainController->getDisplayController()->updateLampData(lampData, true);
    }
    else
    {
        lampData[LMP_START]->blinkspeed.blinkSlow = true;
        lampData[LMP_START]->blink = true;
        lampData[LMP_START]->on = true;
        mainController->getDisplayController()->updateLampData(lampData, true);
    }

    uint8_t btnStatus = mainController->getDisplayController()->getButtonStatus();

    mainController->getDisplayController()->setText("PRESS START TO BEGIN");
    mainController->getDisplayController()->displayText();

    // loop waiting for button press.
    while (!(btnStatus & BTN_START))
    {
        btnStatus = this->mainController->getDisplayController()->getButtonStatus();
        mainController->refreshStatus();

        holdLeft = false;
        holdCentre = false;
        holdRight = false;

        if (hold)
        {
            if (btnStatus & BTN_HOLD)
            {
                holdCentre = true;
                lampData[LMP_HOLD]->blink = false;
                
                mainController->getDisplayController()->updateLampData(lampData, true);
            }
            else if (btnStatus & BTN_HOLD_HI)
            {
                holdRight = true;
                lampData[LMP_HOLD_HI]->blink = false;
                
                mainController->getDisplayController()->updateLampData(lampData, true);
            }
            else if (btnStatus & BTN_HOLD_LO)
            {
                holdLeft = true;
                lampData[LMP_HOLD_LO]->blink = false;
                
                mainController->getDisplayController()->updateLampData(lampData, true);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // let the processor do something else
    }

    mainController->getAudioController()->playAudioFile(SND_NOW_THATS_ICE);

    spinReels();

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
        mainController->getAudioController()->playAudioFile(SND_LOSE);
    }

        
    lampData[LMP_START].blink = false;
    lampData[LMP_START].on = false;    
    lampData[LMP_COLLECT].blink = false;
    lampData[LMP_COLLECT].on = false;
    lampData[LMP_HOLD_HI].blink = false;
    lampData[LMP_HOLD_HI].on = false;
    lampData[LMP_HOLD].blink = false;
    lampData[LMP_HOLD].on = false;
    lampData[LMP_HOLD_LO].blink = false;
    lampData[LMP_HOLD_LO].on = false;
    lampData[LMP_TRANSFER].blink = false;
    lampData[LMP_TRANSFER].on = false;
    
    mainController->getDisplayController()->updateLampData(lampData, true);
    
    // payout
    if ((mainController->getBank() > 0) && (mainController->getCredit() < 20))
    {
        collectOrContinue();
    }

    Game::isInProgress = false;
}

void Game::spinReels()
{
    uint32_t reelStopLeft = holdLeft ? 0 : Entropy.random(26);
    uint32_t reelStopCentre = holdCentre ? 0 : Entropy.random(26);
    uint32_t reelStopRight = holdRight ? 0 : Entropy.random(26);

    mainController->setCredit(mainController->getCredit() - 20);
    lampData[LMP_START].on = false;
    mainController->getDisplayController()->updateLampData(lampData, true);

    this->mainController->getDisplayController()->setText("    LET IT GO!!     ");
    this->mainController->getDisplayController()->displayText();

    mainController->getReelController()->spin(reelStopLeft, reelStopCentre, reelStopRight);

    while (mainController->getReelController()->isCommandInProgress())
    {
        this->mainController->refreshStatus();
        this->moves = Entropy.random(13);
        this->mainController->getDisplayController()->setMoves(this->moves);
    }
}

void Game::shuffleReels()
{
    uint32_t reelStopLeft = Entropy.random(26);
    uint32_t reelStopCentre = Entropy.random(26);
    uint32_t reelStopRight = Entropy.random(26);

    mainController->setCredit(mainController->getCredit() - 20);
    lampData[LMP_START].on = false;
    mainController->getDisplayController()->updateLampData(lampData, true);

    this->mainController->getDisplayController()->setText("    LET IT GO!!     ");
    this->mainController->getDisplayController()->displayText();

    mainController->getReelController()->shuffle(reelStopLeft, reelStopCentre, reelStopRight);

    while (mainController->getReelController()->isCommandInProgress())
    {
        this->mainController->refreshStatus();
        this->moves = Entropy.random(12);
        this->mainController->getDisplayController()->setMoves(this->moves);
    }
}

void Game::playNudges(int nudges)
{

    uint8_t leftPos;
    uint8_t centrePos;
    uint8_t rightPos;

    std::string nudgeText = "        NUDGE        ";

    mainController->getDisplayController()->setText(nudgeText);
    mainController->getDisplayController()->displayText();

    lampData[LMP_START].blink = false;
    lampData[LMP_START].on = false;    
    lampData[LMP_COLLECT].blink = false;
    lampData[LMP_COLLECT].on = false;
    lampData[LMP_HOLD_HI].blink = false;
    lampData[LMP_HOLD_HI].on = false;
    lampData[LMP_HOLD].blink = false;
    lampData[LMP_HOLD].on = false;
    lampData[LMP_HOLD_LO].blink = false;
    lampData[LMP_HOLD_LO].on = false;
    lampData[LMP_TRANSFER].blink = false;
    lampData[LMP_TRANSFER].on = false;
    
    lampData[NUDGE_LAMPS[nudges]]->blinkspeed.blinkFast = true;
    lampData[NUDGE_LAMPS[nudges]]->blink = true;
    lampData[NUDGE_LAMPS[nudges]]->on = true;
    
    mainController->getDisplayController()->updateLampData(lampData, true);
    
    for (int i = 1; i <= nudges; i++) {
        lampData[NUDGE_LAMPS[i]]->blinkspeed.blinkFast = true;
        lampData[NUDGE_LAMPS[i]]->blink = true;
        lampData[NUDGE_LAMPS[i]]->on = true;
    }

    //mainController->getDisplayController()->lampsOn(NUDGES_COL, lmpData, false);

    mainController->getDisplayController()->clearAllData(false);
    for (int i = 0; i < nudges; i++)
    {
        mainController->getDisplayController()->setLampRGBData(&mainController->getDisplayController()->NUDGE_LAMPS[i], BLINK_MASK_NONE | RGB_BLUE, false);
    }
    mainController->getDisplayController()->setLampRGBData(&mainController->getDisplayController()->NUDGE_LAMPS[nudges], BLINK_MASK_FAST | RGB_WHITE, true);

    while (nudges > 0)
    {

        mainController->getDisplayController()->lampsBlinkFast(BTN_COL, LMP_HOLD | LMP_HOLD_HI | LMP_HOLD_LO, false);

        if (isWinningLine())
        {
            mainController->getDisplayController()->lampsBlinkFast(BTN_COL, (mainController->getDisplayController()->getShortBlinkData(BTN_COL) | LMP_TRANSFER), false);
            mainController->getDisplayController()->lampsBlinkSlow(BTN_COL, (mainController->getDisplayController()->getShortBlinkData(BTN_COL) | LMP_START), false);
            mainController->getDisplayController()->lampsOn(BTN_COL, (mainController->getDisplayController()->getLampData(BTN_COL) | LMP_TRANSFER | LMP_START), true);
        }
        else
        {
            mainController->getDisplayController()->lampsOn(BTN_COL, LMP_HOLD_LO | LMP_HOLD | LMP_HOLD_HI, true);
        }

        uint8_t btnStatus = mainController->getDisplayController()->getButtonStatus();

        // loop waiting for button press.
        while ((!isWinningLine() && (!(btnStatus & (BTN_HOLD_LO | BTN_HOLD | BTN_HOLD_HI)))) || (isWinningLine() && (!(btnStatus & (BTN_TRANSFER | BTN_HOLD_LO | BTN_HOLD | BTN_HOLD_HI | BTN_START)))))
        {
            btnStatus = mainController->getDisplayController()->getButtonStatus();
            mainController->refreshStatus();
        }

        if (isWinningLine() && (btnStatus & BTN_TRANSFER))
        {
            mainController->addToBank(mainController->getTransfer());
            mainController->setTransfer(0);
            mainController->getAudioController()->playAudioFile("kerching.wav");
            return;
        }
        else if (isWinningLine() && (btnStatus & BTN_START))
        {
            playFeatureMatrix();
            return;
        }

        nudges--;

        mainController->getDisplayController()->setText(nudgeText);
        mainController->getDisplayController()->displayText();

        mainController->getDisplayController()->clearAllData(false);

        for (int i = 0; i < nudges; i++)
        {
            mainController->getDisplayController()->setLampRGBData(&mainController->getDisplayController()->NUDGE_LAMPS[i], BLINK_MASK_NONE | RGB_BLUE, false);
        }
        mainController->getDisplayController()->setLampRGBData(&mainController->getDisplayController()->NUDGE_LAMPS[nudges], BLINK_MASK_FAST | RGB_WHITE, true);

        //mainController->getDisplayController()->lampsOn(NUDGES_COL, lmpData, false);
        mainController->getDisplayController()->lampsBlinkFast(BTN_COL, LMP_HOLD | LMP_HOLD_HI | LMP_HOLD_LO, false);
        mainController->getDisplayController()->lampsOn(BTN_COL, LMP_HOLD | LMP_HOLD_HI | LMP_HOLD_LO, true);

        if ((btnStatus & BTN_HOLD_LO) == BTN_HOLD_LO)
        {
            // switch off lamps and reset blink status
            mainController->getDisplayController()->lampsBlinkFast(BTN_COL, 0x0000, false);
            mainController->getDisplayController()->lampsOn(BTN_COL, 0x0000, true); // all button lamps off
            mainController->getReelController()->getReelPositions(leftPos, centrePos, rightPos);
            mainController->getReelController()->nudge(1, 0, 0);
        }
        else if ((btnStatus & BTN_HOLD) == BTN_HOLD)
        {
            // switch off lamps and reset blink status
            mainController->getDisplayController()->lampsBlinkFast(BTN_COL, 0x0000, false);
            mainController->getDisplayController()->lampsOn(BTN_COL, 0x0000, true); // all button lamps off
            mainController->getReelController()->getReelPositions(leftPos, centrePos, rightPos);
            mainController->getReelController()->nudge(0, 1, 0);
        }
        else if ((btnStatus & BTN_HOLD_HI) == BTN_HOLD_HI)
        {
            // switch off lamps and reset blink status
            mainController->getDisplayController()->clsetLampsBlinkFast(BTN_COL, 0x0000, false);
            mainController->getDisplayController()->setAllLampsOff(true); // all button lamps off
            mainController->getReelController()->getReelPositions(leftPos, centrePos, rightPos);
            mainController->getReelController()->nudge(0, 0, 1);
        }

        // wait for reel controller to finish command
        while (mainController->getReelController()->isCommandInProgress())
        {
            mainController->refreshStatus();
        }
    }

    mainController->getDisplayController()->clearAllData(true);
    if (isWinningLine())
    {
        transferOrGamble();
    }
    else
    {
        mainController->getAudioController()->playAudioFile(SND_LOSE);
    }
}

void Game::checkHoldPossibilities()
{

    uint8_t leftPos;
    uint8_t centrePos;
    uint8_t rightPos;

    mainController->getReelController()->getReelPositions(leftPos, centrePos, rightPos);

    uint8_t leftSymbolId = symbolsLeftReel[leftPos];
    uint8_t centreSymbolId = symbolsCentreReel[centrePos];
    uint8_t rightSymbolId = symbolsRightReel[rightPos];

    // TODO: work out algorithm for which symbols to hold random? two of the same symbols?
}

void Game::transferOrGamble()
{

    mainController->getDisplayController()->lampsBlinkFast(BTN_COL, LMP_TRANSFER, false);
    mainController->getDisplayController()->lampsBlinkSlow(BTN_COL, LMP_START, false);
    mainController->getDisplayController()->lampsOn(BTN_COL, LMP_START | LMP_TRANSFER, true);

    uint8_t btnStatus = mainController->getDisplayController()->getButtonStatus();

    // loop waiting for button press.
    while (!(btnStatus & (BTN_TRANSFER | BTN_START)))
    {
        btnStatus = mainController->getDisplayController()->getButtonStatus();
        mainController->refreshStatus();
    }

    mainController->getDisplayController()->lampsBlinkFast(BTN_COL, 0x0000, false);
    mainController->getDisplayController()->lampsBlinkSlow(BTN_COL, 0x0000, false);
    mainController->getDisplayController()->lampsOn(BTN_COL, 0x0000, true); // all button lamps off

    if (btnStatus & BTN_TRANSFER)
    {
        mainController->addToBank(mainController->getTransfer());
        mainController->setTransfer(0);
        mainController->getAudioController()->playAudioFile("kerching.wav");
    }
    else if (btnStatus & BTN_START)
    {
        playFeatureMatrix();
    }
}

void Game::collectOrContinue()
{
    mainController->getDisplayController()->lampsBlinkFast(BTN_COL, LMP_START, false);
    mainController->getDisplayController()->lampsBlinkSlow(BTN_COL, LMP_COLLECT, false);
    mainController->getDisplayController()->lampsOn(BTN_COL, LMP_START | LMP_COLLECT, true);

    uint8_t btnStatus = mainController->getDisplayController()->getButtonStatus();

    // loop waiting for button press.
    while (!(btnStatus & (BTN_COLLECT | BTN_START)))
    {
        btnStatus = mainController->getDisplayController()->getButtonStatus();
        mainController->refreshStatus();
    }

    mainController->getDisplayController()->lampsBlinkFast(BTN_COL, 0x0000, false);
    mainController->getDisplayController()->lampsBlinkSlow(BTN_COL, 0x0000, false);
    mainController->getDisplayController()->lampsOn(BTN_COL, 0x0000, true); // all button lamps off

    if ((btnStatus & BTN_COLLECT))
    {
        mainController->payout();
    }
    else if ((btnStatus & BTN_START))
    {
        mainController->addToCredit(mainController->getBank());
        mainController->setBank(0);
    }
}

bool Game::isWinningLine()
{
    uint8_t leftPos;
    uint8_t centrePos;
    uint8_t rightPos;

    bool isWin = false;

    mainController->getReelController()->getReelPositions(leftPos, centrePos, rightPos);

    uint8_t leftSymbolId = symbolsLeftReel[leftPos];
    uint8_t centreSymbolId = symbolsCentreReel[centrePos];
    uint8_t rightSymbolId = symbolsRightReel[rightPos];

    //for (uint8_t i = 0; i < (sizeof (uint8_t) / sizeof (*_winningCombinations)); i++) {
    for (uint8_t i = 0; i < 7; i++)
    {

        if (((leftSymbolId == winningCombinations[i].leftSymbolId) || (winningCombinations[i].leftSymbolId == 255)) &&
            ((centreSymbolId == winningCombinations[i].centreSymbolId) || (winningCombinations[i].centreSymbolId == 255)) &&
            ((rightSymbolId == winningCombinations[i].rightSymbolId) || (winningCombinations[i].rightSymbolId == 255)))
        {
            mainController->setTransfer(this->winningCombinations[i].amount);
            isWin = true;
            break;
        }
    }

    return isWin;
}

void Game::playFeatureMatrix()
{
    uint32_t featureIndex = 0;
    mainController->getAudioController()->playAudioFile(SND_LET_IT_GO);

    mainController->getDisplayController()->lampsBlinkFast(LMP_START, false);
    mainController->getDisplayController()->lampsOn(LMP_START, true);

    // loop waiting for button press.
    uint8_t btnStatus = mainController->getDisplayController()->getButtonStatus();
    while ((btnStatus & (BTN_START)) == 0)
    {
        featureIndex = Entropy.random(13);                                                                      // number of features
        // uint32_t lampData = pgm_read_dword(&mainController->getDisplayController()->FEATURE_LAMPS[featureIndex]);
        mainController->getDisplayController()->setLampRGBData(LMP_TRAIL_BASE + featureIndex, RGB_WHITE, true); // switch on feature LED
        mainController->refreshStatus();
        btnStatus = mainController->getDisplayController()->getButtonStatus();
        mainController->getDisplayController()->setLampRGBData(LMP_TRAIL_BASE + featureIndex, RGB_BLACK, true); // switch feature LED back off
    }

    // Feature has been chosen, let's continue...
    switch (featureIndex)
    {
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

void Game::playTrail()
{
    uint8_t index = 0;
    while (PRIZE_TRAIL_PRIZES[index] < mainController->getTransfer() && index < PRIZE_TRAIL_PRIZES_LENGTH)
    {
        index++;
    }

    //mainController->getDisplayController()->lampsOn(mainController->getDisplayController()->TRAIL_LAMPS[index], false);
}

void Game::playHiLo()
{
}

void Game::playShuffle()
{
    uint8_t btnStatus = mainController->getDisplayController()->getButtonStatus();

    mainController->getDisplayController()->setText("      SHUFFLE!      ");
    mainController->getDisplayController()->displayText();
    // loop waiting for button press.
    while ((btnStatus & BTN_START) != BTN_START)
    {
        btnStatus = this->mainController->getDisplayController()->getButtonStatus();
        mainController->refreshStatus();
    }

    mainController->getAudioController()->playAudioFile(SND_NOW_THATS_ICE);

    uint32_t nudges = Entropy.random(6);

    shuffleReels();

    if (isWinningLine())
    {
        transferOrGamble();
    }
    else
    {
        mainController->getAudioController()->playAudioFile(SND_LOSE);
    }

    mainController->getDisplayController()->lampsBlinkFast(BTN_COL, 0x0000, false);
    mainController->getDisplayController()->lampsBlinkSlow(BTN_COL, 0x0000, false);
    mainController->getDisplayController()->lampsOn(BTN_COL, 0x0000, true);
}

void Game::playFreeSpin()
{
    uint8_t btnStatus = mainController->getDisplayController()->getButtonStatus();

    mainController->getDisplayController()->setText(string("     FREE SPIN!     "));
    mainController->getDisplayController()->displayText();
    // loop waiting for button press.
    while ((btnStatus & BTN_START) != BTN_START)
    {
        btnStatus = this->mainController->getDisplayController()->getButtonStatus();
        mainController->refreshStatus();
    }

    mainController->getAudioController()->playAudioFile(SND_NOW_THATS_ICE);

    spinReels();

    if (isWinningLine())
    {
        transferOrGamble();
    }
    else
    {
        mainController->getAudioController()->playAudioFile(SND_LOSE);
    }

    mainController->getDisplayController()->lampsBlinkFast(BTN_COL, 0x0000, false);
    mainController->getDisplayController()->lampsBlinkSlow(BTN_COL, 0x0000, false);
    mainController->getDisplayController()->lampsOn(BTN_COL, 0x0000, true);
}

bool Game::isGameInProgress()
{
    return this->isInProgress;
}
