/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   CCTalkController.cpp
 * Author: chris
 * 
 * Created on October 14, 2018, 4:10 PM
 */
#include <stdlib.h>
#include <string>

#include "config.h"
#include "cctalk.h"
#include "cctalkheaders.h"

#include "maincontroller.h"
#include "cctalkcontroller.h"


static const char *TAG = "CCTalkController";

CCTalkController::CCTalkController(MainController *mainController) {    
    this->mainController = mainController;
    this->cctalkDevice = cctalk_init_device(UART_NUM_0, CCTALK_GPIO_TX, CCTALK_GPIO_RX, CCTALK_HOST);
}

CCTalkController::CCTalkController(const CCTalkController& orig) {
}

CCTalkController::~CCTalkController() {
}

void CCTalkController::begin() {
    uint8_t data[255];
    uint8_t length;
    resetDevice(CCTALK_COIN_VALIDATOR, data, length);
    resetDevice(CCTALK_HOPPER, data, length);

    //mainController->getTerm()->println(F("Coin Validator"));
    requestManufacturerId(CCTALK_COIN_VALIDATOR, data, length);
    //mainController->getTerm()->println(getStringResponse(data, length));
    requestBuildCode(CCTALK_COIN_VALIDATOR, data, length);
    //mainController->getTerm()->println(getStringResponse(data, length));
    requestProductCode(CCTALK_COIN_VALIDATOR, data, length);
    //mainController->getTerm()->println(getStringResponse(data, length));

    modifySorterPath(CCTALK_COIN_VALIDATOR, 1, 1, data, length); // 5ct  (Kasse - rejected anyway)
    modifySorterPath(CCTALK_COIN_VALIDATOR, 2, 1, data, length); // 10ct (Kasse, adapter slot D, cctalk sort chute 1)
    modifySorterPath(CCTALK_COIN_VALIDATOR, 3, 2, data, length); // 20ct (Hopper, adapter slot C, cctalk sort chute 2)
    modifySorterPath(CCTALK_COIN_VALIDATOR, 4, 1, data, length); // 50ct (Kasse, adapter slot D, cctalk sort chute 1)
    modifySorterPath(CCTALK_COIN_VALIDATOR, 5, 1, data, length); // 1eur (Kasse, adapter slot D, cctalk sort chute 1)
    modifySorterPath(CCTALK_COIN_VALIDATOR, 6, 1, data, length); // 2eur (Kasse, adapter slot D, cctalk sort chute 1)

    //    modifyDefaultSorterPath(COIN_VALIDATOR, 1, data, length); // adapter slot D, cctalk sort chute 1
    modifyInhibitStatus(CCTALK_COIN_VALIDATOR, 254, 0, data, length); // Allow all coins except 5ct
    modifyMasterInhibitStatus(CCTALK_COIN_VALIDATOR, 1, data, length); // Enable coin validator

    //mainController->getTerm()->println(F("Hopper"));
    requestManufacturerId(CCTALK_HOPPER, data, length);
    //mainController->getTerm()->println(getStringResponse(data, length));
    requestBuildCode(CCTALK_HOPPER, data, length);
    //mainController->getTerm()->println(getStringResponse(data, length));
    requestProductCode(CCTALK_HOPPER, data, length);
    //mainController->getTerm()->println(getStringResponse(data, length));

}

cctalk_response_t* CCTalkController::resetDevice(const uint8_t address, uint8_t *data, uint8_t length) {
    cctalk_request_t* request = (cctalk_request_t*)calloc(1, sizeof(cctalk_request_t));    
    request->destination = address;
    request->additionalData = data;
    request->additionalDataSize = length;
        
    cctalk_response_t response = cctalk_send_request(&cctalkDevice, &request);
    
    free(request);
   
    return response;
}

cctalk_response_t* CCTalkController::requestManufacturerId(const uint8_t address, uint8_t data[], uint8_t length) {

    CCTalk_Request* request = new CCTalk_Request(address, request_manufacturer_id, 0, 0);
    cctalkDevice->sendMessage(request);
    delete request;

    while (cctalkDevice->getState() < CCTalk_Device::RX_State::RXcomplete) {
        cctalkDevice->poll();
    }

    return cctalkDevice->getResponse(data, length);

}

cctalk_response_t* CCTalkController::requestBuildCode(const uint8_t address, uint8_t data[], uint8_t length) {

    CCTalk_Request* request = new CCTalk_Request(address, request_build_code, 0, 0);
    cctalkDevice->sendMessage(request);
    delete request;

    while (cctalkDevice->getState() < CCTalk_Device::RX_State::RXcomplete) {
        cctalkDevice->poll();
    }

    return cctalkDevice->getResponse(data, length);
}

cctalk_response_t* CCTalkController::requestSerialNumber(const uint8_t address, uint8_t data[], uint8_t length) {

    CCTalk_Request* request = new CCTalk_Request(address, request_serial_number, 0, 0);
    cctalkDevice->sendMessage(request);
    delete request;

    while (cctalkDevice->getState() < CCTalk_Device::RX_State::RXcomplete) {
        cctalkDevice->poll();
    }

    return cctalkDevice->getResponse(data, length);
}

cctalk_response_t* CCTalkController::requestProductCode(const uint8_t address, uint8_t data[], uint8_t length) {

    CCTalk_Request* request = new CCTalk_Request(address, request_product_code, 0, 0);
    cctalkDevice->sendMessage(request);
    delete request;

    while (cctalkDevice->getState() < CCTalk_Device::RX_State::RXcomplete) {
        cctalkDevice->poll();
    }

    return cctalkDevice->getResponse(data, length);
}

cctalk_response_t* CCTalkController::pollCredit(const uint8_t address, uint8_t data[], uint8_t length) {
    mainController->getTerm()->print("Poll Credit: ");
    CCTalk_Request* request = new CCTalk_Request(address, read_buffered_credit_or_error_codes, 0, 0);
    cctalkDevice->sendMessage(request);
    delete request;

    while (cctalkDevice->getState() < CCTalk_Device::RX_State::RXcomplete) {
        cctalkDevice->poll();
    }

    return cctalkDevice->getResponse(data, length);
}

cctalk_response_t* CCTalkController::testHopper(const uint8_t address, uint8_t data[], uint8_t length) {
    mainController->getTerm()->print("Test Hopper: ");
    CCTalk_Request* request = new CCTalk_Request(address, test_hopper, 0, 0);
    cctalkDevice->sendMessage(request);
    delete request;

    while (cctalkDevice->getState() < CCTalk_Device::RX_State::RXcomplete) {
        cctalkDevice->poll();
    }

    return cctalkDevice->getResponse(data, length);
}

cctalk_response_t* CCTalkController::pollHopperStatus(const uint8_t address, uint8_t data[], uint8_t length) {

    CCTalk_Request* request = new CCTalk_Request(address, request_hopper_status, 0, 0);
    cctalkDevice->sendMessage(request);
    delete request;

    while (cctalkDevice->getState() < CCTalk_Device::RX_State::RXcomplete) {
        cctalkDevice->poll();
    }

    return cctalkDevice->getResponse(data, length);
}

cctalk_response_t* CCTalkController::modifyDefaultSorterPath(const uint8_t address, const uint8_t defaultChute, uint8_t data[], uint8_t length) {

    uint8_t tmpData[1];
    tmpData[0] = defaultChute;
    CCTalk_Request* request = new CCTalk_Request(address, modify_default_sorter_path, tmpData, (uint8_t) 1);
    cctalkDevice->sendMessage(request);
    delete request;

    while (cctalkDevice->getState() < CCTalk_Device::RX_State::RXcomplete) {
        cctalkDevice->poll();
    }

    return cctalkDevice->getResponse(data, length);
}

cctalk_response_t* CCTalkController::modifySorterPath(const uint8_t address, const uint8_t path, const uint8_t chute, uint8_t data[], uint8_t length) {

    uint8_t tmpData[2];
    tmpData[0] = path;
    tmpData[1] = chute;
    CCTalk_Request* request = new CCTalk_Request(address, modify_sorter_paths, tmpData, (uint8_t) 2);
    cctalkDevice->sendMessage(request);
    delete request;

    while (cctalkDevice->getState() < CCTalk_Device::RX_State::RXcomplete) {
        cctalkDevice->poll();
    }

    return cctalkDevice->getResponse(data, length);
}

cctalk_response_t* CCTalkController::modifyInhibitStatus(const uint8_t address, const uint8_t enable1, const uint8_t enable2, uint8_t data[], uint8_t length) {


    uint8_t tmpData[2];
    tmpData[0] = enable1;
    tmpData[1] = enable2;
    CCTalk_Request* request = new CCTalk_Request(address, modify_inhibit_status, tmpData, (uint8_t) 2);
    cctalkDevice->sendMessage(request);
    delete request;

    while (cctalkDevice->getState() < CCTalk_Device::RX_State::RXcomplete) {
        cctalkDevice->poll();
    }

    return cctalkDevice->getResponse(data, length);
}

cctalk_response_t* CCTalkController::modifyMasterInhibitStatus(const uint8_t address, const uint8_t enable, uint8_t data[], uint8_t length) {

    uint8_t tmpData[1];
    tmpData[0] = enable;

    CCTalk_Request* request = new CCTalk_Request(address, modify_master_inhibit_status, tmpData, (uint8_t) 1);
    cctalkDevice->sendMessage(request);
    delete request;

    while (cctalkDevice->getState() < CCTalk_Device::RX_State::RXcomplete) {
        cctalkDevice->poll();
    }

    return cctalkDevice->getResponse(data, length);
}

cctalk_response_t* CCTalkController::enableHopper(const uint8_t address, uint8_t data[], uint8_t length) {

    uint8_t tmpData[1];
    tmpData[0] = 165;
    CCTalk_Request* request = new CCTalk_Request(address, enable_hopper, tmpData, 1);
    cctalkDevice->sendMessage(request);
    delete request;

    while (cctalkDevice->getState() < CCTalk_Device::RX_State::RXcomplete) {
        cctalkDevice->poll();
    }

    return cctalkDevice->getResponse(data, length);
}

cctalk_response_t* CCTalkController::dispenseCoins(const uint8_t address, const uint8_t numCoins, uint8_t data[], uint8_t length) {

    uint8_t tmpData[9];
    uint8_t tmpLength;


    // cipher key should be 8 bytes long. 
    bool result = requestCipherKey(address, tmpData, tmpLength);

    
    ESP_LOGI(TAG, "Cipher Key: %03d %03d %03d %03d %03d %03d %03d %03d", tmpData[0], tmpData[1], tmpData[2], tmpData[3], tmpData[4], tmpData[5], tmpData[6], tmpData[7]);    
    ESP_LOGI(TAG, "Checking response");    
    ESP_LOGI(TAG, "Dispensing %02d coin(s).", numCoins);
    
    // In the 9th byte we have the number of coins.
    tmpData[8] = numCoins;

    
    CCTalkController::sendRequest(address, dispense_hopper_coins, tmpData, (uint8_t) 9);
    
    //evice->sendMessage(request);
    

    while (cctalkDevice->getState() < CCTalk_Device::RX_State::RXcomplete) {
        cctalkDevice->poll();
    }


    return cctalkDevice->getResponse(data, length);
}

cctalk_response_t* CCTalkController::requestCipherKey(const uint8_t address, uint8_t data[], uint8_t length) {
    ESP_LOGI("Cipher key requested");

    CCTalk_Request* request = new CCTalk_Request(address, request_cipher_key, 0, 0);
    cctalkDevice->sendMessage(request);
    delete request;

    while (cctalkDevice->getState() < CCTalk_Device::RX_State::RXcomplete) {
        cctalkDevice->poll();
    }

    return cctalkDevice->getResponse(data, length);
}

/*
 * @brief Sends a request to the given ccTalk device
 * 
 *  
 */
cctalk_response_t* CCTalkController::sendRequest(const char * header, const uint8_t address, uint8_t *data, uint8_t length) {
    cctalk_request_t *request;
    cctalk_response_t *response;
    request = (cctalk_request_t*)calloc(1, sizeof(cctalk_request_t));    
    
    request->destination = address;
    request->header = DISPENSE_HOPPER_COINS;
    request->additionalData = data;
    request->additionalDataSize = length;
    
    response = cctalk_send_request(&this->cctalkDevice, &request);
    
    free(request); // free up the memory
    
    // TODO: send request 
    return response;
}

//uint8_t CCTalk_Device::getBinaryData() {
//    String data = "";
//
//    while (this->getState() < CCTalk_Device::RXcomplete) {
//        this->poll();
//    }
//
//    if (this->getState() == CCTalk_Device::RXcomplete) {
//        CCTalk_Response* response = this->readResponse();
//
//        if (response->getDataFieldLength() > 0) {
//            for (int i = 0; i < response->getDataFieldLength(); i++) {
//                data.concat((char) response->getData()[i]);
//            }
//        }
//        delete response;
//    }
//
//    return data;
//}

std::string CCTalkController::getStringResponse(const uint8_t data[], const uint8_t length) {

    char tmpData[256];

    // sanity check
    if (length > 255) {
        return "";
    }
    uint8_t i = 0;
    for (i = 0; i < length; i++) {
        tmpData[i] = data[i];
    }
    tmpData[i++] = 0;

    return String(tmpData);
}

