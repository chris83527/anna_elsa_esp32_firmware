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

#include <stdint.h>

#include "cctalk_link_controller.h"


class CCTalkController {
public:
    CCTalkController(MainController *mainController);    
    CCTalkController(const CCTalkController& orig);
    virtual ~CCTalkController();
    
    esp_err_t initialise(void);
    
//    void resetDevice(const uint8_t address, CctalkResponse &response);
//    void requestManufacturerId(const uint8_t address, CctalkResponse &response);
//    void requestProductCode(const uint8_t address, CctalkResponse &response);
//    void requestBuildCode(const uint8_t address, CctalkResponse &response);
//    void requestSerialNumber(const uint8_t address, CctalkResponse &response);
//    void requestSoftwareRevision(const uint8_t address, CctalkResponse &response);
//    void requestCommsRevision(const uint8_t address, CctalkResponse &response);
//    void pollCredit(const uint8_t address, CctalkResponse &response);
//    void pollHopperStatus(const uint8_t address, CctalkResponse &response);
//    void modifyInhibitStatus(const uint8_t address, const uint8_t enable1, const uint8_t enable2, CctalkResponse &response);
//    void modifyMasterInhibitStatus(const uint8_t address, const uint8_t enable, CctalkResponse &response);
//    void modifyDefaultSorterPath(const uint8_t address, const uint8_t defaultChute, CctalkResponse &response);
//    void modifySorterPath(const uint8_t address, const uint8_t path, const uint8_t chute, CctalkResponse &response);
//    void enableHopper(const uint8_t address, CctalkResponse &response);
//    void readOptoStates(const uint8_t address, CctalkResponse &response);
//    void dispenseCoins(const uint8_t address, uint8_t numCoins, CctalkResponse &response);
//    void requestCipherKey(const uint8_t address, CctalkResponse &response);
//    void testHopper(const uint8_t address, CctalkResponse &response);
//    void testSolenoids(const uint8_t address, CctalkResponse &response); 
//    void requestPayoutHighLowStatus(const uint8_t address, CctalkResponse &response);
    
    const static uint8_t COIN_VALUES[];
          
    const static unsigned long VALIDATOR_POLL_INTERVAL = 100;    
    const static unsigned long HOPPER_STATUS_POLL_INTERVAL = 400;   
    
    //void sendRequest(Cctalk::Headers header, uint8_t destination, std::vector<uint8_t>& data, CctalkResponse &response);       
    
    
private:    
    
    esp32cc::CctalkLinkController linkController;
    //esp32cc::CoinAcceptorDevice coinAcceptor;
    //esp32cc::CoinHopperDevice hopper;
    
    MainController * mainController;
    
    //int getState();
        
};



#endif /* CCTALKCONTROLLER_H */

