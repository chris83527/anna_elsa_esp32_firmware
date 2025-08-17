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

/**
 * @file moneycontroller.cpp
 *
 * Routines for adding to bank, adding credit etc.
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "cctalkcontroller.h"
#include "esp_log.h"
#include "nvs.h"

#include "NvsController.h"
#include "moneycontroller.h"

static const char* TAG = "MoneyController";

MoneyController::MoneyController(NvsController& nvsCtrlr,
                                 CCTalkController& cctalkController)
    : credit(0), bank(0), transfer(0), gamecount(0), payoutTotal(0), incomeTotal(0), tenCentIn(0), twentyCentIn(0),
      fiftyCentIn(0),
      oneEuroIn(0),
      twoEuroIn(0),
      nvsController(nvsCtrlr),
      cctalkController(cctalkController)
{
    ESP_LOGI(TAG, "Entering constructor");
    ESP_LOGI(TAG, "Leaving constructor");

    this->payoutInProgress = false;
}

MoneyController::~MoneyController()
= default;

void MoneyController::initialise()
{
    ESP_LOGI(TAG, "initialise() called");
    loadValuesFromStorage();
}

/**
 * @brief Load credit and bank values from persistent storage. Used to recover
 * credit and bank values after poweroff
 *
 */
void MoneyController::loadValuesFromStorage()
{
    this->credit = this->nvsController.readValueFromNVS(NVS_KEY_CREDIT.c_str());
    this->bank = this->nvsController.readValueFromNVS(NVS_KEY_BANK.c_str());
    this->gamecount =
        this->nvsController.readValueFromNVS(NVS_KEY_GAME_COUNT.c_str());
    this->transfer =
        this->nvsController.readValueFromNVS(NVS_KEY_TRANSFER.c_str());
    this->tenCentIn =
        this->nvsController.readValueFromNVS(NVS_KEY_TEN_CENT_IN.c_str());
    this->twentyCentIn =
        this->nvsController.readValueFromNVS(NVS_KEY_TWENTY_CENT_IN.c_str());
    this->fiftyCentIn =
        this->nvsController.readValueFromNVS(NVS_KEY_FIFTY_CENT_IN.c_str());
    this->oneEuroIn =
        this->nvsController.readValueFromNVS(NVS_KEY_ONE_EURO_IN.c_str());
    this->twoEuroIn =
        this->nvsController.readValueFromNVS(NVS_KEY_TWO_EURO_IN.c_str());
    this->incomeTotal =
        this->nvsController.readValueFromNVS(NVS_KEY_INCOME_TOTAL.c_str());
    this->payoutTotal =
        this->nvsController.readValueFromNVS(NVS_KEY_PAYOUT_TOTAL.c_str());
}

/**
 * @brief Add the given amount to the credit
 *
 * @param value The amount to be added to the player's total credit
 */
void MoneyController::addToCredit(Payment& payment)
{
    addToCredit(payment.getTenCent() * 10);
    this->nvsController.writeValueToNVS(NVS_KEY_TEN_CENT_IN.c_str(),
                                        this->tenCentIn + payment.getTenCent());
    addToCredit(payment.getTwentyCent() * 20);
    this->nvsController.writeValueToNVS(NVS_KEY_TWENTY_CENT_IN.c_str(),
                                        this->twentyCentIn +
                                        payment.getTwentyCent());
    addToCredit(payment.getFiftyCent() * 50);
    this->nvsController.writeValueToNVS(NVS_KEY_FIFTY_CENT_IN.c_str(),
                                        this->fiftyCentIn +
                                        payment.getFiftyCent());
    addToCredit(payment.getOneEuro() * 100);
    this->nvsController.writeValueToNVS(NVS_KEY_ONE_EURO_IN.c_str(),
                                        this->oneEuroIn + payment.getOneEuro());
    addToCredit(payment.getTwoEuro() * 200);
    this->nvsController.writeValueToNVS(NVS_KEY_TWO_EURO_IN.c_str(),
                                        this->twoEuroIn + payment.getTwoEuro());

    this->nvsController.writeValueToNVS(NVS_KEY_CREDIT.c_str(), credit);
}

void MoneyController::addToCredit(uint16_t value)
{
    this->credit += value;
    this->incomeTotal += value;
    this->nvsController.writeValueToNVS(NVS_KEY_CREDIT.c_str(), credit);
    this->nvsController.writeValueToNVS(NVS_KEY_INCOME_TOTAL.c_str(),
                                        this->incomeTotal);
}

/**
 * @brief Add the given amount to the bank
 *
 * @param value The amount to be added to the player's bank
 */
void MoneyController::addToBank(const uint16_t value)
{
    this->bank += value;
    this->nvsController.writeValueToNVS(NVS_KEY_BANK.c_str(), bank);
}

/**
 * @brief Remove the given amount from the player's credit (e.g. when booking a
 * game)
 *
 * @param value The amount to be removed to the player's total credit
 */
void MoneyController::removeFromCredit(const uint16_t value)
{
    // check for negative values
    if ((credit - value) >= 0)
    {
        credit -= value;
        this->nvsController.writeValueToNVS(NVS_KEY_CREDIT.c_str(), credit);
    }
}

void MoneyController::payoutBank()
{
    cctalkController.dispenseCoins(
        (getBank() / 20), [&](const std::string& error_msg)
        {
            // TODO: Check status and see how many coins were returned and
            // remove these from bank. For now we will just set bank to 0 (and
            // presume all coins were paid out)
            if (!error_msg.empty())
            {
                ESP_LOGE(TAG, "An error occurred during payout: %s",
                         error_msg.c_str());
            }
            else
            {
                this->payoutTotal += getBank();
                this->removeFromBank(getBank());
                this->nvsController.writeValueToNVS(NVS_KEY_PAYOUT_TOTAL.c_str(),
                                                    this->payoutTotal);
            }
        });
}

/**
 * @brief Remove the given amount from the player's bank
 *
 * @param value
 */
void MoneyController::removeFromBank(const uint16_t value)
{
    // check for negative values
    if ((bank - value) >= 0)
    {
        bank -= value;
        this->nvsController.writeValueToNVS(NVS_KEY_BANK.c_str(), bank);
    }
}

/**
 * @brief Get the Bank object
 *
 * @return int
 */
uint16_t MoneyController::getBank() const { return bank; }

uint16_t MoneyController::getGameCount() const { return gamecount; }

uint16_t MoneyController::getPayoutTotal() { return payoutTotal; }

uint16_t MoneyController::getIncomeTotal() const { return incomeTotal; }

void MoneyController::incrementGameCount()
{
    this->gamecount += 1;
    this->nvsController.writeValueToNVS(NVS_KEY_GAME_COUNT.c_str(), gamecount);
}

bool MoneyController::isPayoutInProgress() { return this->payoutInProgress; }

void MoneyController::setPayoutInProgress(bool payoutInProgress)
{
    this->payoutInProgress = payoutInProgress;
}

void MoneyController::moveBankToCredit()
{
    MoneyController::addToCredit(this->bank);
    MoneyController::removeFromBank(this->bank);
}

void MoneyController::moveTransferToBank()
{
    MoneyController::addToBank(this->transfer);
    MoneyController::setTransfer(0);
}

/*
 * @brief Get the Credit object
 *
 * @return int
 */
uint16_t MoneyController::getCredit() { return credit; }

void MoneyController::setTransfer(uint16_t amount)
{
    this->transfer = amount;
    nvsController.writeValueToNVS(NVS_KEY_TRANSFER.c_str(), transfer);
}

uint16_t MoneyController::getTransfer() { return transfer; }

void Payment::clear()
{
    Payment::tenCentIn = 0;
    Payment::twentyCentIn = 0;
    Payment::fiftyCentIn = 0;
    Payment::oneEuroIn = 0;
    Payment::twoEuroIn = 0;
}

void Payment::addTenCent() { Payment::tenCentIn++; }

uint16_t Payment::getTenCent() { return Payment::tenCentIn; }

void Payment::addTwentyCent() { Payment::twentyCentIn++; }

uint16_t Payment::getTwentyCent() { return Payment::twentyCentIn; }

void Payment::addFiftyCent() { Payment::fiftyCentIn++; }

uint16_t Payment::getFiftyCent() { return Payment::fiftyCentIn; }

void Payment::addOneEuro() { Payment::oneEuroIn++; }

uint16_t Payment::getOneEuro() { return Payment::oneEuroIn; }

void Payment::addTwoEuro() { Payment::twoEuroIn++; }

uint16_t Payment::getTwoEuro() { return Payment::twoEuroIn; }
