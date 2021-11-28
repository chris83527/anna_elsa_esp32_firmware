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
 * @file moneycontroller.c
 *
 * Routines for adding to bank, adding credit etc.
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <stdint.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

#include "maincontroller.h"
#include "moneycontroller.h"

static const char *TAG = "MoneyController";

MoneyController::MoneyController(MainController* mainController) {
    ESP_LOGI(TAG, "Entering constructor");
    ESP_LOGI(TAG, "Leaving constructor");
    this->mainController = mainController;
}

void MoneyController::initialise() {
    ESP_LOGI(TAG, "initialise() called");
    loadValuesFromStorage();
}

/**
 * @brief Load credit and bank values from persistent storage. Used to recover credit and bank values after poweroff
 * 
 */
void MoneyController::loadValuesFromStorage() {
    credit = this->mainController->readValueFromNVS(NVS_KEY_CREDIT);
    bank = this->mainController->readValueFromNVS(NVS_KEY_BANK);
    gamecount = this->mainController->readValueFromNVS(NVS_KEY_GAME_COUNT);
    transfer = this->mainController->readValueFromNVS(NVS_KEY_TRANSFER);
}

/**
 * @brief Add the given amount to the credit
 * 
 * @param value The amount to be added to the player's total credit
 */
void MoneyController::addToCredit(Payment payment) {
    addToCredit(payment.getTenCent() * 10);
    this->mainController->writeValueToNVS(NVS_KEY_TEN_CENT_IN, payment.getTenCent());
    addToCredit(payment.getTwentyCent() * 20);
    this->mainController->writeValueToNVS(NVS_KEY_TWENTY_CENT_IN, payment.getTwentyCent());
    addToCredit(payment.getFiftyCent() * 50);
    this->mainController->writeValueToNVS(NVS_KEY_FIFTY_CENT_IN, payment.getFiftyCent());
    addToCredit(payment.getOneEuro() * 100);
    this->mainController->writeValueToNVS(NVS_KEY_ONE_EURO_IN, payment.getOneEuro());
    addToCredit(payment.getTwoEuro() * 200);
    this->mainController->writeValueToNVS(NVS_KEY_TWO_EURO_IN, payment.getTwoEuro());
    
    this->mainController->writeValueToNVS(NVS_KEY_CREDIT, credit);
}

void MoneyController::addToCredit(uint16_t value) {
    this->credit += value;
    this->mainController->writeValueToNVS(NVS_KEY_CREDIT, credit);
}

/**
 * @brief Add the given amount to the bank
 * 
 * @param value The amount to be added to the player's bank
 */
void MoneyController::addToBank(const uint16_t value) {
    this->bank += value;
    this->mainController->writeValueToNVS(NVS_KEY_BANK, bank);
}

/**
 * @brief Remove the given amount from the player's credit (e.g. when booking a game)
 * 
 * @param value The amount to be removed to the player's total credit
 */
void MoneyController::removeFromCredit(const uint16_t value) {
    // check for negative values
    if ((credit - value) >= 0) {
        credit -= value;
        this->mainController->writeValueToNVS(NVS_KEY_CREDIT, credit);
    }
}

/**
 * @brief Remove the given amount from the player's bank 
 * 
 * @param value 
 */
void MoneyController::removeFromBank(const uint16_t value) {
    // check for negative values
    if ((bank - value) >= 0) {
        bank -= value;
        this->mainController->writeValueToNVS(NVS_KEY_BANK, bank);
    }
}

/**
 * @brief Get the Bank object
 * 
 * @return int
 */
uint16_t MoneyController::getBank() {
    return bank;
}

void MoneyController::incrementGameCount() {
    this->gamecount += 1;
    this->mainController->writeValueToNVS(NVS_KEY_GAME_COUNT, gamecount);
}

bool MoneyController::isPayoutInProgress() {
    return this->payoutInProgress;
}

void MoneyController::setPayoutInProgress(bool payoutInProgress) {
    this->payoutInProgress = payoutInProgress;
}

void MoneyController::moveBankToCredit() {
    MoneyController::addToCredit(this->bank);
    MoneyController::removeFromBank(this->bank);
}

void MoneyController::moveTransferToBank() {
    MoneyController::addToBank(this->transfer);
    MoneyController::setTransfer(0);
}

/*
 * @brief Get the Credit object
 * 
 * @return int 
 */
uint16_t MoneyController::getCredit() {
    return credit;
}

void MoneyController::setTransfer(uint16_t amount) {
    this->transfer = amount;
    mainController->writeValueToNVS(NVS_KEY_TRANSFER, transfer);
}

uint16_t MoneyController::getTransfer() {
    return transfer;
}


void Payment::addTenCent() {
    Payment::tenCentIn++;
}

uint16_t Payment::getTenCent() {
    return Payment::tenCentIn;
}

void Payment::addTwentyCent() {
    Payment::twentyCentIn++;
}

uint16_t Payment::getTwentyCent() {
    return Payment::twentyCentIn;
}

void Payment::addFiftyCent() {
    Payment::fiftyCentIn++;
}

uint16_t Payment::getFiftyCent() {
    return Payment::fiftyCentIn;
}

void Payment::addOneEuro() {
    Payment::oneEuroIn++;
}

uint16_t Payment::getOneEuro() {
    return Payment::oneEuroIn;
}

void Payment::addTwoEuro() {
    Payment::twoEuroIn++;
}

uint16_t Payment::getTwoEuro() {
    return Payment::twoEuroIn;
}
