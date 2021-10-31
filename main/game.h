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
 * File:   Game.h
 * Author: chris
 *
 * Created on January 14, 2018, 3:02 PM
 */

#ifndef GAME_H
#define GAME_H

#include <string>

#include "reelcontroller.h"
#include "displaycontroller.h"


class SerialMonitorController;
class MainController; // forward declaration

class Game {
public:
    Game(MainController *mainController);
    Game(const Game& orig);    
    void start(void);
    bool isGameInProgress();
    void playNudges(int nudges);

    // 0 = Hans
    // 1 = Olaf
    // 2 = Sven
    // 3 = Anna
    // 4 = Kristof
    // 5 = Elsa
    // 6 = Palace
    // Map symbol IDs to symbol names (for debugging only)
    std::string symbolMap[7] = {"Hans", "Olaf", "Sven", "Anna", "Kristof", "Elsa", "Palace"};
    std::string featureMap[12] = {"Free Spin", "Double Money", "Shuffle", "Lose", "Palace", "Palace", "Shuffle", "Lose", "Free Spin", "Hi/Lo", "Free Spin", "Palace"};

    // Map symbols to physical stops on reels (25 stops on a reel)
    uint8_t symbolsLeftReel[25]   = {0, 1, 0, 0, 3, 2, 4, 0, 1, 1, 2, 4, 5, 2, 4, 2, 1, 4, 5, 1, 3, 4, 6, 2, 1};
    uint8_t symbolsCentreReel[25] = {2, 0, 1, 5, 0, 2, 0, 4, 1, 5, 2, 4, 1, 2, 4, 4, 1, 2, 5, 2, 4, 6, 1, 3, 0};
    uint8_t symbolsRightReel[25]  = {4, 4, 1, 1, 2, 0, 0, 1, 2, 4, 6, 1, 5, 0, 2, 4, 6, 1, 2, 0, 3, 2, 5, 4, 1};

private:

    MainController* mainController;

    DisplayController::lamp_data_t *lampData;
    
    bool isInProgress;

    bool holdLeft;
    bool holdCentre;
    bool holdRight;

    void collectOrContinue();
    void transferOrGamble();
    void checkHoldPossibilities();
    void playFeatureMatrix();
    void playTrail();
    void playHiLo();
    void playShuffle();
    void playFreeSpin();
    bool isWinningLine();
    void spinReels();
    void shuffleReels();

    bool holdEnabled;

    struct PositionValueMapping {
        uint8_t valueLeft;
        uint8_t valueCentre;
        uint8_t valueRight;
    };

    struct WinningLine {
        uint8_t leftSymbolId;
        uint8_t centreSymbolId;
        uint8_t rightSymbolId;
        int amount;
        bool extendedGame;
        bool freeSpin;
    };

    WinningLine winningCombinations[8] = {
        {0, 0, 0, 20, false, false},
        {1, 1, 1, 80, true, false},
        {1, 1, 255, 40, false, false}, // Olaf, Olaf, Any
        {2, 2, 2, 120, true, false},
        {3, 3, 3, 300, false, false},
        {4, 4, 4, 160, false, true},
        {5, 5, 5, 200, true, false},
        {6, 6, 6, 400, true, false}
    };
    
    uint16_t PRIZE_TRAIL_PRIZES[17] {
        20,40,60,80,100,120,140,160,180,200,240,280,340,380,420,460,500
    };
    uint8_t PRIZE_TRAIL_PRIZES_LENGTH = sizeof(PRIZE_TRAIL_PRIZES) / 16;
    
    uint8_t moves;
};

#endif /* GAME_H */

