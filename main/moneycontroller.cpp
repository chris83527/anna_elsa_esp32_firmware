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
#include <esp_system.h>
#include <nvs_flash.h>
#include <nvs.h>

#include "moneycontroller.h"

static const char *TAG = "MoneyController";

MoneyController::MoneyController() {

    loadValuesFromStorage();
}

/**
 * @brief Load credit and bank values from persistent storage. Used to recover credit and bank values after poweroff
 * 
 */
void MoneyController::loadValuesFromStorage() {
    credit = 0;
    bank = 0;
}

/**
 * @brief Add the given amount to the credit
 * 
 * @param value The amount to be added to the player's total credit
 */
void MoneyController::addToCredit(const uint16_t value) {
    credit += value;
}

/**
 * @brief Add the given amount to the bank
 * 
 * @param value The amount to be added to the player's bank
 */
void MoneyController::addToBank(const uint16_t value) {
    bank += value;
}

/**
 * @brief Remove the given amount from the player's credit (e.g. when booking a game)
 * 
 * @param value The amount to be removed to the player's total credit
 */
void MoneyController::removeFromCredit(const uint16_t value) {
    // TODO: check for negative values
    credit -= value;
}

/**
 * @brief Remove the given amount from the player's bank 
 * 
 * @param value 
 */
uint16_t MoneyController::removeFromBank(const uint16_t value) {
    // TODO: check for negative values
    bank -= value;
}

/**
 * @brief 
 * 
 * @param value 
 */
uint16_t MoneyController::removeFromTransfer(const uint16_t value) {
    // TODO: check for negative values
    transfer -= value;
}

/**
 * @brief Get the Bank object
 * 
 * @return int
 */
uint16_t MoneyController::getBank() {
    return bank;
}

/*
 * @brief Get the Credit object
 * 
 * @return int 
 */
uint16_t MoneyController::getCredit() {
    return credit;
}

uint16_t MoneyController::getTransfer() {
    return transfer;
}
