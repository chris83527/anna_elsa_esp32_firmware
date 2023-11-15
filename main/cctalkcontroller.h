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
#include <vector>

#include <stdint.h>

#include "cctalk_enums.h"
#include "cctalk_link_controller.h"
#include "coin_acceptor_device.h"
#include "coin_hopper_device.h"

#define CCTALK_HOST (1)
#define CCTALK_HOPPER (3)
#define CCTALK_COIN_VALIDATOR (2)

class MainController;

class CCTalkController {
public:
    CCTalkController();
    CCTalkController(const CCTalkController& orig);
    virtual ~CCTalkController();

    void setCreditAcceptedCallback(esp32cc::CoinAcceptorDevice::CreditAcceptedFunc creditAcceptedCallback);
    
    esp_err_t initialise(void);        

    const static unsigned long VALIDATOR_POLL_INTERVAL = 250;
    const static unsigned long HOPPER_STATUS_POLL_INTERVAL = 100;
    
    // These should be private
    esp32cc::CoinHopperDevice hopper;
    esp32cc::CoinAcceptorDevice coinAcceptor;
private:

    esp32cc::CctalkLinkController* cctalkLinkController;
       
};

const static uint8_t COIN_VALUES[] = {0, 5, 10, 20, 50, 100, 200};

#endif /* CCTALKCONTROLLER_H */

