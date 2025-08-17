/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   CCTalkController.h
 * Author: chris
 *
 * Created on October 14, 2018, 4:10 PM
 */

#ifndef CCTALKCONTROLLER_H
#define CCTALKCONTROLLER_H

#include <string>

#include "cctalk_device.h"
#include "cctalk_link_controller.h"
#include "coin_acceptor_device.h"
#include "coin_hopper_device.h"

class MainController;

class CCTalkController {
public:
  CCTalkController();
  virtual ~CCTalkController();

  void setCreditAcceptedCallback(
      esp32cc::CoinAcceptorDevice::CreditAcceptedFunc creditAcceptedCallback);

  esp_err_t initialise();

  void dispenseCoins(
      int numberOfCoins,
      std::function<void(const std::string &error_msg)> finish_callback);

public:
  static constexpr uint8_t COIN_VALUES[] = {0, 5, 10, 20, 50, 100, 200};

  static constexpr unsigned long VALIDATOR_POLL_INTERVAL = 250;
  static constexpr unsigned long HOPPER_STATUS_POLL_INTERVAL = 100;

  static constexpr uint8_t CCTALK_HOST = 1;
  static constexpr uint8_t CCTALK_COIN_VALIDATOR = 2;
  static constexpr uint8_t CCTALK_HOPPER = 3;

private:
  esp32cc::CctalkLinkController cctalkLinkController;
  esp32cc::CoinHopperDevice hopper;
  esp32cc::CoinAcceptorDevice coinAcceptor;
};

#endif /* CCTALKCONTROLLER_H */
