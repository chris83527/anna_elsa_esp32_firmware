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
 * @file moneycontroller.h
 *
 * Routines for adding to bank, adding credit etc.
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __MONEYCONTROLLER_H__
#define __MONEYCONTROLLER_H__

#include <stdint.h>


class MainController;

class Payment {
public:
    void addTenCent(void);
    void addTwentyCent(void);
    void addFiftyCent(void);
    void addOneEuro(void);
    void addTwoEuro(void);
    
    uint16_t getTenCent(void);
    uint16_t getTwentyCent(void);
    uint16_t getFiftyCent(void);
    uint16_t getOneEuro(void);
    uint16_t getTwoEuro(void);
    void clear(void);
private:
    uint16_t tenCentIn;
    uint16_t twentyCentIn;
    uint16_t fiftyCentIn;
    uint16_t oneEuroIn;
    uint16_t twoEuroIn;
};

class MoneyController {
public:
    MoneyController(MainController* mainController);
    ~MoneyController();

    void initialise(void);

    void addToCredit(Payment &payment);
    void addToCredit(uint16_t value);
    void addToBank(const uint16_t value);
    void setTransfer(const uint16_t value);
    void incrementGameCount(void);
    void removeFromCredit(const uint16_t value);
    void removeFromBank(const uint16_t value);
    void moveBankToCredit(void);
    void moveTransferToBank(void);
    //void removeFromTransfer(const int value);
    uint16_t getCredit(void);
    uint16_t getBank(void);
    uint16_t getTransfer(void);
    uint16_t getGameCount(void);

    uint16_t getPayoutTotal(void);
    uint16_t getIncomeTotal(void);

    void setPayoutInProgress(bool inProgress);
    bool isPayoutInProgress(void);
    void resetCounters(void);

private:

    void loadValuesFromStorage(void);

    uint16_t credit;
    uint16_t bank;
    uint16_t transfer;
    uint16_t gamecount;
    uint16_t payoutTotal;
    uint16_t incomeTotal;
    uint16_t tenCentIn;
    uint16_t twentyCentIn;
    uint16_t fiftyCentIn;
    uint16_t oneEuroIn;
    uint16_t twoEuroIn;

    const char* NVS_KEY_CREDIT = "credit";
    const char* NVS_KEY_BANK = "bank";
    const char* NVS_KEY_TRANSFER = "transfer";
    const char* NVS_KEY_GAME_COUNT = "gameCount";
    const char* NVS_KEY_PAYOUT_TOTAL = "payoutTotal";
    const char* NVS_KEY_INCOME_TOTAL = "incomeTotal";
    const char* NVS_KEY_TEN_CENT_IN = "tenCentIn";
    const char* NVS_KEY_TWENTY_CENT_IN = "twentyCentIn";
    const char* NVS_KEY_FIFTY_CENT_IN = "fiftyCentIn";
    const char* NVS_KEY_ONE_EURO_IN = "oneEuroIn";
    const char* NVS_KEY_TWO_EURO_IN = "twoEuroIn";

    MainController* mainController;

    bool payoutInProgress;
};

#endif
