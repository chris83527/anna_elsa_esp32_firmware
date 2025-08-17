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

#include <string>

#include "NvsController.h"
#include "cctalkcontroller.h"

class Payment {
public:
  void addTenCent();
  void addTwentyCent();
  void addFiftyCent();
  void addOneEuro();
  void addTwoEuro();

  uint16_t getTenCent();
  uint16_t getTwentyCent();
  uint16_t getFiftyCent();
  uint16_t getOneEuro();
  uint16_t getTwoEuro();
  void clear();
  void payoutBank();

private:
  uint16_t tenCentIn = 0;
  uint16_t twentyCentIn = 0;
  uint16_t fiftyCentIn = 0;
  uint16_t oneEuroIn = 0;
  uint16_t twoEuroIn = 0;
};

class MoneyController {
public:
  MoneyController(NvsController &nvsController,
                  CCTalkController &cctalkController);
  ~MoneyController();

  void initialise();

  void addToCredit(Payment &payment);
  void addToCredit(uint16_t value);
  void addToBank(uint16_t value);
  void setTransfer(uint16_t value);
  void incrementGameCount();
  void removeFromCredit(uint16_t value);
  void removeFromBank(uint16_t value);
  void moveBankToCredit();
  void moveTransferToBank();
  void payoutBank();
  // void removeFromTransfer(const int value);
  uint16_t getCredit();
  [[nodiscard]] uint16_t getBank() const;
  uint16_t getTransfer();
  [[nodiscard]] uint16_t getGameCount() const;

  uint16_t getPayoutTotal();
  [[nodiscard]] uint16_t getIncomeTotal() const;

  void setPayoutInProgress(bool inProgress);
  bool isPayoutInProgress();
  void resetCounters();

private:
  void loadValuesFromStorage();

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

  NvsController &nvsController;
  CCTalkController &cctalkController;

  static constexpr std::string NVS_KEY_CREDIT = "credit";
  static constexpr std::string NVS_KEY_BANK = "bank";
  static constexpr std::string NVS_KEY_TRANSFER = "transfer";
  static constexpr std::string NVS_KEY_GAME_COUNT = "gameCount";
  static constexpr std::string NVS_KEY_PAYOUT_TOTAL = "payoutTotal";
  static constexpr std::string NVS_KEY_INCOME_TOTAL = "incomeTotal";
  static constexpr std::string NVS_KEY_TEN_CENT_IN = "tenCentIn";
  static constexpr std::string NVS_KEY_TWENTY_CENT_IN = "twentyCentIn";
  static constexpr std::string NVS_KEY_FIFTY_CENT_IN = "fiftyCentIn";
  static constexpr std::string NVS_KEY_ONE_EURO_IN = "oneEuroIn";
  static constexpr std::string NVS_KEY_TWO_EURO_IN = "twoEuroIn";

  bool payoutInProgress;
};

#endif
