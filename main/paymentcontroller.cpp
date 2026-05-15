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
 * @file paymentcontroller.cpp
 *
 * Routines for adding to bank, adding credit etc.
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "cctalk.hpp"
#include "esp_log.h"

#include "paymentcontroller.h"
#include "maincontroller.h"

static const char* TAG = "PaymentController";

PaymentController::PaymentController(MainController* mainController, std::unique_ptr<ICctalkUart> uart) :
    credit(0), bank(0),
    transfer(0), gamecount(0),
    payoutTotal(0), incomeTotal(0), tenCentIn(0),
    twentyCentIn(0),
    fiftyCentIn(0),
    oneEuroIn(0),
    twoEuroIn(0),
    mainController(mainController),
    uart_(std::move(uart))
{
    ESP_LOGI(TAG, "Entering constructor");

    bus_ = std::make_unique<CctalkBus>(*uart_, CCTALK_HOST_ADDRESS);

    // Create high-level façade
    facade_ = std::make_unique<CctalkDeviceFacade>(
        *bus_, CCTALK_HOST_ADDRESS, CCTALK_COIN_ACCEPTOR_ADDRESS, CCTALK_HOPPER_ADDRESS
    );

    // Create worker threads
    acceptorThread_ = std::make_unique<CoinAcceptorThread>(*facade_, eventQueue_);
    hopperThread_ = std::make_unique<HopperThread>(*facade_, eventQueue_);

    setEventHandler([this](const CctalkEvent& evt) { this->onEvent(evt); });

    this->payoutInProgress = false;
    ESP_LOGI(TAG, "Leaving constructor");
}

PaymentController::~PaymentController()
{
    stop();
}

void PaymentController::setEventHandler(EventHandler handler)
{
    handler_ = std::move(handler);
}

void PaymentController::start()
{
    ESP_LOGI(TAG, "start() called");

    if (running_) return;
    running_ = true;

    loadValuesFromStorage();

    std::string out;
    facade_->getManufacturer(CCTALK_COIN_ACCEPTOR_ADDRESS, out);
    ESP_LOGI(TAG, "Manufacturer Id: %s", out.c_str());
    facade_->getSerialNumber(CCTALK_COIN_ACCEPTOR_ADDRESS, out);
    ESP_LOGI(TAG, "Serial Number: %s", out.c_str());
    facade_->getCategoryId(CCTALK_COIN_ACCEPTOR_ADDRESS, out);
    ESP_LOGI(TAG, "Category ID: %s", out.c_str());
    facade_->getBuildCode(CCTALK_COIN_ACCEPTOR_ADDRESS, out);
    ESP_LOGI(TAG, "Build code: %s", out.c_str());
    facade_->getSoftwareRevision(CCTALK_COIN_ACCEPTOR_ADDRESS, out);
    ESP_LOGI(TAG, "Software Revision: %s", out.c_str());
    facade_->requestCommsRevision(CCTALK_COIN_ACCEPTOR_ADDRESS, out);
    ESP_LOGI(TAG, "Comms revision: %s", out.c_str());


    facade_->getManufacturer(CCTALK_HOPPER_ADDRESS, out);
    ESP_LOGI(TAG, "Manufacturer Id: %s", out.c_str());
    facade_->getSerialNumber(CCTALK_HOPPER_ADDRESS, out);
    ESP_LOGI(TAG, "Serial Number: %s", out.c_str());
    facade_->getCategoryId(CCTALK_HOPPER_ADDRESS, out);
    ESP_LOGI(TAG, "Category ID: %s", out.c_str());
    facade_->getBuildCode(CCTALK_HOPPER_ADDRESS, out);
    ESP_LOGI(TAG, "Build code: %s", out.c_str());
    facade_->getSoftwareRevision(CCTALK_HOPPER_ADDRESS, out);
    ESP_LOGI(TAG, "Software Revision: %s", out.c_str());
    facade_->requestCommsRevision(CCTALK_HOPPER_ADDRESS, out);
    ESP_LOGI(TAG, "Comms revision: %s", out.c_str());


    facade_->resetDevice(CCTALK_COIN_ACCEPTOR_ADDRESS);
    facade_->resetDevice(CCTALK_HOPPER_ADDRESS);
    // adapter slot D, cctalk sort chute 1
    facade_->modifyDefaultSorterPath(1);
    // 5ct  (Kasse - rejected anyway)
    facade_->modifySorterPaths(1, 1);
    // 10ct (Kasse, adapter slot D, cctalk sort chute 1)
    facade_->modifySorterPaths(2, 1);
    // 20ct (Hopper, adapter slot C, cctalk sort chute 2)
    facade_->modifySorterPaths(3, 2);
    // 50ct (Kasse, adapter slot D, cctalk sort chute 1)
    facade_->modifySorterPaths(4, 1);
    // 1eur (Kasse, adapter slot D, cctalk sort chute 1)
    facade_->modifySorterPaths(5, 1);
    // 2eur (Kasse, adapter slot D, cctalk sort chute 1)
    facade_->modifySorterPaths(6, 1);
    // the coin validator automatically sends coins to cash box - stop this.
    facade_->modifySorterOverrideStatus(255);
    // Allow all coins except 5ct
    facade_->setInhibitMask(254, 0);

    facade_->enableHopper();

    acceptorThread_->start();
    //hopperThread_->start();

    dispatcherThread_ = std::make_unique<EventDispatcherThread>(
        eventQueue_,
        handler_
            ? handler_
            : [](const CctalkEvent&)
            {
                ESP_LOGI(TAG, "Dummy handler called!");
            }
    );

    dispatcherThread_->start();
}

void PaymentController::stop()
{
    if (!running_) return;
    running_ = false;

    acceptorThread_->stop();
    //hopperThread_->stop();

    if (dispatcherThread_)
    {
        dispatcherThread_->stop();
    }
}

/**
 * @brief Load credit and bank values from persistent storage. Used to recover
 * credit and bank values after poweroff
 *
 */
void PaymentController::loadValuesFromStorage()
{
    resetCounters();
    this->credit = this->mainController->getNvsController().readValueFromNVS(NVS_KEY_CREDIT.c_str());
    this->bank = this->mainController->getNvsController().readValueFromNVS(NVS_KEY_BANK.c_str());
    this->gamecount =
        this->mainController->getNvsController().readValueFromNVS(NVS_KEY_GAME_COUNT.c_str());
    this->transfer =
        this->mainController->getNvsController().readValueFromNVS(NVS_KEY_TRANSFER.c_str());
    this->tenCentIn =
        this->mainController->getNvsController().readValueFromNVS(NVS_KEY_TEN_CENT_IN.c_str());
    this->twentyCentIn =
        this->mainController->getNvsController().readValueFromNVS(NVS_KEY_TWENTY_CENT_IN.c_str());
    this->fiftyCentIn =
        this->mainController->getNvsController().readValueFromNVS(NVS_KEY_FIFTY_CENT_IN.c_str());
    this->oneEuroIn =
        this->mainController->getNvsController().readValueFromNVS(NVS_KEY_ONE_EURO_IN.c_str());
    this->twoEuroIn =
        this->mainController->getNvsController().readValueFromNVS(NVS_KEY_TWO_EURO_IN.c_str());
    this->incomeTotal =
        this->mainController->getNvsController().readValueFromNVS(NVS_KEY_INCOME_TOTAL.c_str());
    this->payoutTotal =
        this->mainController->getNvsController().readValueFromNVS(NVS_KEY_PAYOUT_TOTAL.c_str());
}

/**
 * @brief Add the given amount to the credit
 *
 * @param payment The amount to be added to the player's total credit
 */
void PaymentController::addToCredit(const Payment& payment)
{
    addToCredit(payment.getTenCent() * 10);
    this->mainController->getNvsController().writeValueToNVS(NVS_KEY_TEN_CENT_IN.c_str(),
                                                             this->tenCentIn + payment.getTenCent());
    addToCredit(payment.getTwentyCent() * 20);
    this->mainController->getNvsController().writeValueToNVS(NVS_KEY_TWENTY_CENT_IN.c_str(),
                                                             this->twentyCentIn +
                                                             payment.getTwentyCent());
    addToCredit(payment.getFiftyCent() * 50);
    this->mainController->getNvsController().writeValueToNVS(NVS_KEY_FIFTY_CENT_IN.c_str(),
                                                             this->fiftyCentIn +
                                                             payment.getFiftyCent());
    addToCredit(payment.getOneEuro() * 100);
    this->mainController->getNvsController().writeValueToNVS(NVS_KEY_ONE_EURO_IN.c_str(),
                                                             this->oneEuroIn + payment.getOneEuro());
    addToCredit(payment.getTwoEuro() * 200);
    this->mainController->getNvsController().writeValueToNVS(NVS_KEY_TWO_EURO_IN.c_str(),
                                                             this->twoEuroIn + payment.getTwoEuro());

    this->mainController->getNvsController().writeValueToNVS(NVS_KEY_CREDIT.c_str(), credit);
}

void PaymentController::addToCredit(uint16_t value)
{
    this->credit += value;
    this->incomeTotal += value;
    this->mainController->getNvsController().writeValueToNVS(NVS_KEY_CREDIT.c_str(), credit);
    this->mainController->getNvsController().writeValueToNVS(NVS_KEY_INCOME_TOTAL.c_str(),
                                                             this->incomeTotal);
}

/**
 * @brief Add the given amount to the bank
 *
 * @param value The amount to be added to the player's bank
 */
void PaymentController::addToBank(const uint16_t value)
{
    this->bank += value;
    this->mainController->getNvsController().writeValueToNVS(NVS_KEY_BANK.c_str(), bank);
}

/**
 * @brief Remove the given amount from the player's credit (e.g. when booking a
 * game)
 *
 * @param value The amount to be removed to the player's total credit
 */
void PaymentController::removeFromCredit(const uint16_t value)
{
    // check for negative values
    if ((credit - value) >= 0)
    {
        credit -= value;
        this->mainController->getNvsController().writeValueToNVS(NVS_KEY_CREDIT.c_str(), credit);
    }
}

void PaymentController::payoutBank()
{
    ESP_LOGI(TAG, "payoutBank called. Bank total: %d", this->bank);
    payoutInProgress.store(true);

    ESP_LOGI(TAG, "Testing hopper status");
    TestHopperStatus testHopperStatus{};
    CctalkError err = facade_->testHopper(testHopperStatus);

    if (err != CctalkError::OK || testHopperStatus.statusRegister1 != 0)
    {
        ESP_LOGE(TAG, "Hopper status not ok: %d", testHopperStatus.statusRegister1);
        if ((testHopperStatus.statusRegister1 | HopperStatusRegister1Bits::OK) != HopperStatusRegister1Bits::OK)
        {
            ESP_LOGI(TAG, "Hopper status register 1: %d", testHopperStatus.statusRegister1);
            ESP_LOGI(TAG, "Resetting hopper");
            err = facade_->resetDevice(facade_->getHopperAddress());
            lastPayoutStatus = {}; // reset lastPayout status too
        }

        ESP_LOGI(TAG, "Re-testing hopper status");
        TestHopperStatus testHopperStatus{};
        CctalkError err = facade_->testHopper(testHopperStatus);
        if ((testHopperStatus.statusRegister1 | HopperStatusRegister1Bits::OK) != HopperStatusRegister1Bits::OK)
        {
            ESP_LOGI(TAG, "Hopper status register 1: %d", testHopperStatus.statusRegister1);
            ESP_LOGI(TAG, "Giving up. Please check hopper");
            return;
        }
    }

    ESP_LOGI(TAG, "Enabling hopper");
    err = facade_->enableHopper();
    if (err == CctalkError::OK)
    {
        cctalk::hopper::RspHopperCipherKey out{};
        CctalkError err = facade_->requestCipherKey(out);

        if (err == CctalkError::OK && out.cipher.size() == 8)
        {
            ESP_LOGI(TAG, "Got cipher key: %d %d %d %d %d %d %d %d", out.cipher[0], out.cipher[1], out.cipher[2],
                     out.cipher[3], out.cipher[4], out.cipher[5], out.cipher[6], out.cipher[7]);
            ESP_LOGI(TAG, "Calling hopperPayout with cipher and %d coins", static_cast<uint8_t>(getBank() / 20));
            err = facade_->hopperPayout(out.cipher, static_cast<uint8_t>(getBank() / 20));
            if (err == CctalkError::OK)
            {
                HopperPayoutStatus payoutStatus{};
                do
                {
                    ESP_LOGI(TAG, "Polling hopper status");
                    err = facade_->getHopperStatus(payoutStatus);
                    if (err == CctalkError::OK)
                    {
                        if (payoutStatus.dispenseCount > 0 && payoutStatus.dispenseCount > lastPayoutStatus.
                            dispenseCount)
                        {
                            ESP_LOGI(TAG, "Hopper dispenseCount changed from %d to %d",
                                     lastPayoutStatus.dispenseCount, payoutStatus.dispenseCount);
                            ESP_LOGI(TAG, "Hopper coinsRemaining changed from %d to %d",
                                     lastPayoutStatus.coinsRemaining, payoutStatus.coinsRemaining);
                            ESP_LOGI(TAG, "Hopper coinsPaid changed from %d to %d", lastPayoutStatus.coinsPaid,
                                     payoutStatus.coinsPaid);
                            ESP_LOGI(TAG, "Hopper coinsUnpaid changed from %d to %d", lastPayoutStatus.coinsUnpaid,
                                     payoutStatus.coinsUnpaid);

                            removeFromBank(lastPayoutStatus.coinsPaid * 20);
                        }
                    }
                    else
                    {
                        ESP_LOGE(TAG, "An error occurred getting hopper payout status: %d", err);
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                // If the payout operation completes successfully or abnormally, the coinsRemaining counter will be set to 0.
                while (payoutStatus.coinsRemaining != 0);

                lastPayoutStatus = payoutStatus;

                // Give an error if we have unpaid coins
                if (lastPayoutStatus.coinsUnpaid > 0)
                {
                    ESP_LOGE(TAG, "Hopper has unpaid coins!");
                }

                // The Host software should always check the coinsRemaining counter if it has become 0. If it has become 0 and the
                // coins unpaid counter is non-zero, then the dispense procedure has been aborted before all coins were
                // dispensed. Check with the Hopper Test command if the hopper has timed-out (due to jams or empty).
                ESP_LOGI(TAG, "Rechecking hopper status after payout");
                err = facade_->testHopper(testHopperStatus);
                if ((testHopperStatus.statusRegister1 | static_cast<uint8_t>(HopperStatusRegister1Bits::OK)) !=
                    static_cast<uint8_t>(HopperStatusRegister1Bits::OK))
                {
                    ESP_LOGI(TAG, "Hopper status register 1: %d", testHopperStatus.statusRegister1);
                    ESP_LOGI(TAG, "Resetting hopper");
                    err = facade_->resetDevice(facade_->getHopperAddress());
                    lastPayoutStatus = {}; // reset lastPayout status too
                }
                else
                {
                    ESP_LOGI(TAG, "Hopper status ok. No unpaid coins");
                }
            }
        }
    }

    payoutInProgress.store(false);
}

/**
 * @brief Remove the given amount from the player's bank
 *
 * @param value
 */
void PaymentController::removeFromBank(const uint16_t value)
{
    // check for negative values
    if ((bank - value) >= 0)
    {
        bank -= value;
        this->mainController->getNvsController().writeValueToNVS(NVS_KEY_BANK.c_str(), bank);
    }
}

void PaymentController::resetCounters()
{
    credit = 0;
    bank = 0;
    transfer = 0;
    gamecount = 0;
    payoutTotal = 0;
    incomeTotal = 0;
    tenCentIn = 0;
    twentyCentIn = 0;
    fiftyCentIn = 0;
    oneEuroIn = 0;
    twoEuroIn = 0;
}


/**
 * @brief Get the Bank object
 *
 * @return int
 */
uint16_t PaymentController::getBank() const { return bank; }

uint16_t PaymentController::getGameCount() const { return gamecount; }

uint16_t PaymentController::getPayoutTotal() const { return payoutTotal; }

uint16_t PaymentController::getIncomeTotal() const { return incomeTotal; }

void PaymentController::incrementGameCount()
{
    this->gamecount += 1;
    this->mainController->getNvsController().writeValueToNVS(NVS_KEY_GAME_COUNT.c_str(), gamecount);
}

bool PaymentController::isPayoutInProgress() const { return this->payoutInProgress; }

void PaymentController::setPayoutInProgress(bool payoutInProgress)
{
    this->payoutInProgress = payoutInProgress;
}

void PaymentController::moveBankToCredit()
{
    addToCredit(this->bank);
    removeFromBank(this->bank);
}

void PaymentController::moveTransferToBank()
{
    addToBank(this->transfer);
    setTransfer(0);
}

/*
 * @brief Get the Credit object
 *
 * @return int
 */
uint16_t PaymentController::getCredit() const { return credit; }

void PaymentController::setTransfer(uint16_t amount)
{
    this->transfer = amount;
    this->mainController->getNvsController().writeValueToNVS(NVS_KEY_TRANSFER.c_str(), transfer);
}

uint16_t PaymentController::getTransfer() const { return transfer; }

void PaymentController::onEvent(const CctalkEvent& evt)
{
    ESP_LOGI(TAG, "onEvent called!");
    switch (evt.type)
    {
    case CctalkEventType::CoinAccepted:
        ESP_LOGI(TAG, "Coin accepted. Coin Id: %d, Coin value: %d", evt.coin.coin_id, evt.coin.coin_value);
        this->addToCredit(COIN_VALUES[evt.coin.coin_id]);
        // easy way - just look up in our table instead of from the validator
        this->mainController->getHttpController().broadcast_status();
        break;

    case CctalkEventType::HopperPayoutStatusChanged:
        break;
    }
}

void Payment::clear()
{
    tenCentIn = 0;
    twentyCentIn = 0;
    fiftyCentIn = 0;
    oneEuroIn = 0;
    twoEuroIn = 0;
}

void Payment::addTenCent()
{
    tenCentIn++;
}

uint16_t Payment::getTenCent() const
{
    return tenCentIn;
}

void Payment::addTwentyCent()
{
    twentyCentIn++;
}

uint16_t Payment::getTwentyCent() const
{
    return twentyCentIn;
}

void Payment::addFiftyCent()
{
    fiftyCentIn++;
}

uint16_t Payment::getFiftyCent() const
{
    return fiftyCentIn;
}

void Payment::addOneEuro()
{
    oneEuroIn++;
}

uint16_t Payment::getOneEuro() const
{
    return oneEuroIn;
}

void Payment::addTwoEuro()
{
    twoEuroIn++;
}

uint16_t Payment::getTwoEuro() const
{
    return twoEuroIn;
}
