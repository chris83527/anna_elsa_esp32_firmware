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
#include <stdlib.h>
#include <stdint.h>

#include "cctalk.h"

class CCTalkController {
public:
    CCTalkController(MainController *mainController);    
    CCTalkController(const CCTalkController& orig);
    virtual ~CCTalkController();
    
    cctalk_response_t* resetDevice(const uint8_t address, uint8_t *data, uint8_t length);
    cctalk_response_t* requestManufacturerId(const uint8_t address, uint8_t *data, uint8_t length);
    cctalk_response_t* requestProductCode(const uint8_t address, uint8_t *data, uint8_t length);
    cctalk_response_t* requestBuildCode(const uint8_t address, uint8_t *data, uint8_t length);
    cctalk_response_t* requestSerialNumber(const uint8_t address, uint8_t *data, uint8_t length);
    cctalk_response_t* requestSoftwareRevision(const uint8_t address, uint8_t *data, uint8_t length);
    cctalk_response_t* requestCommsRevision(const uint8_t address, uint8_t *data, uint8_t length);
    cctalk_response_t* pollCredit(const uint8_t address, uint8_t *data, uint8_t length);
    cctalk_response_t* pollHopperStatus(const uint8_t address, uint8_t *data, uint8_t length);
    cctalk_response_t* modifyInhibitStatus(const uint8_t address, const uint8_t enable1, const uint8_t enable2, uint8_t *data, uint8_t length);
    cctalk_response_t* modifyMasterInhibitStatus(const uint8_t address, const uint8_t enable, uint8_t *data, uint8_t length);
    cctalk_response_t* modifyDefaultSorterPath(const uint8_t address, const uint8_t defaultChute, uint8_t *data, uint8_t length);
    cctalk_response_t* modifySorterPath(const uint8_t address, const uint8_t path, const uint8_t chute, uint8_t *data, uint8_t length);
    cctalk_response_t* enableHopper(const uint8_t address, uint8_t *data, uint8_t length);
    cctalk_response_t* readOptoStates(const uint8_t address, uint8_t *data, uint8_t length);
    cctalk_response_t* dispenseCoins(const uint8_t address, uint8_t numCoins, uint8_t *data, uint8_t length);
    cctalk_response_t* requestCipherKey(const uint8_t address, uint8_t *data, uint8_t length);
    cctalk_response_t* testHopper(const uint8_t address, uint8_t *data, uint8_t length);
    cctalk_response_t* testSolenoids(const uint8_t address, uint8_t *data, uint8_t length);
        
    void begin();    
    
    cctalk_response_t* sendRequest(const uint8_t header, const uint8_t address);    
    
    std::string getStringResponse();    
    
private:
    cctalk_device_t *cctalkDevice;
    MainController * mainController;
    
    int getState();
    
    const uint8_t COIN_VALUES[7] = {0, 5, 10, 20, 50, 100, 200};
};

#endif /* CCTALKCONTROLLER_H */

