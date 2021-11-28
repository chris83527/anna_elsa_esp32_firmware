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

#include "cctalk.h"

class CCTalkController {
public:
    CCTalkController(MainController *mainController);    
    CCTalkController(const CCTalkController& orig);
    virtual ~CCTalkController();
    
    esp_err_t initialise(void);
    
    cctalk_response_t* resetDevice(const uint8_t address);
    std::string requestManufacturerId(const uint8_t address);
    std::string requestProductCode(const uint8_t address);
    std::string requestBuildCode(const uint8_t address);
    std::string requestSerialNumber(const uint8_t addres);
    std::string requestSoftwareRevision(const uint8_t address);
    std::string requestCommsRevision(const uint8_t address);
    cctalk_response_t* pollCredit(const uint8_t address);
    cctalk_response_t* pollHopperStatus(const uint8_t address);
    cctalk_response_t* modifyInhibitStatus(const uint8_t address, const uint8_t enable1, const uint8_t enable2);
    cctalk_response_t* modifyMasterInhibitStatus(const uint8_t address, const uint8_t enable);
    cctalk_response_t* modifyDefaultSorterPath(const uint8_t address, const uint8_t defaultChute);
    cctalk_response_t* modifySorterPath(const uint8_t address, const uint8_t path, const uint8_t chute);
    cctalk_response_t* enableHopper(const uint8_t address);
    cctalk_response_t* readOptoStates(const uint8_t address);
    cctalk_response_t* dispenseCoins(const uint8_t address, uint8_t numCoins);
    cctalk_response_t* requestCipherKey(const uint8_t address);
    cctalk_response_t* testHopper(const uint8_t address);
    cctalk_response_t* testSolenoids(const uint8_t address); 
    cctalk_response_t* requestPayoutHighLowStatus(const uint8_t address);
    
    const static uint8_t COIN_VALUES[];
          
    const static unsigned long VALIDATOR_POLL_INTERVAL = 100;    
    const static unsigned long HOPPER_STATUS_POLL_INTERVAL = 200;   
    
    cctalk_response_t* sendRequest(cctalk_header_e header, const uint8_t address, uint8_t *data, uint8_t length);
    
    std::string getStringResponse(cctalk_response_t *response);
    
private:
    cctalk_device_t *cctalkDevice;
    MainController * mainController;
    
    int getState();
      
    
};



#endif /* CCTALKCONTROLLER_H */

