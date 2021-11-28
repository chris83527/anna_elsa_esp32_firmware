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
#include <cstdlib>
#include <string>

#include "esp_log.h"

#include "config.h"
#include "cctalk.h"
#include "cctalkheaders.h"

#include "maincontroller.h"

#include "cctalkcontroller.h"

using namespace std;

static const char *TAG = "CCTalkController";

const uint8_t CCTalkController::COIN_VALUES[] = {0, 5, 10, 20, 50, 100, 200};

CCTalkController::CCTalkController(MainController *mainController) {
    ESP_LOGD(TAG, "Entering constructor");
    this->mainController = mainController;
    ESP_LOGD(TAG, "Leaving constructor");
}

CCTalkController::CCTalkController(const CCTalkController& orig) {
}

CCTalkController::~CCTalkController() {
}

esp_err_t CCTalkController::initialise() {
    ESP_LOGD(TAG, "CCTalkController::initialise called");
    cctalkDevice = cctalk_init_device(UART_NUM_1, CCTALK_GPIO_TX, CCTALK_GPIO_RX, CCTALK_HOST);

    if (cctalkDevice == nullptr) {
        ESP_LOGE(TAG, "Initialising cctalk device failed.");
        return ESP_FAIL;
    }
    
    string strResponse;

    resetDevice(CCTALK_COIN_VALIDATOR);
    resetDevice(CCTALK_HOPPER);

    //mainController->getTerm()->println(F("Coin Validator"));
    strResponse = requestManufacturerId(CCTALK_COIN_VALIDATOR);
    ESP_LOGI(TAG, "Coin validator manufacturer Id: %s", strResponse.c_str());    
    strResponse = requestBuildCode(CCTALK_COIN_VALIDATOR);
    ESP_LOGI(TAG, "Build Code: %s", strResponse.c_str());    
    strResponse = requestProductCode(CCTALK_COIN_VALIDATOR);
    ESP_LOGI(TAG, "Product Code: %s", strResponse.c_str());    

    modifySorterPath(CCTALK_COIN_VALIDATOR, 1, 1); // 5ct  (Kasse - rejected anyway)
    modifySorterPath(CCTALK_COIN_VALIDATOR, 2, 1); // 10ct (Kasse, adapter slot D, cctalk sort chute 1)
    modifySorterPath(CCTALK_COIN_VALIDATOR, 3, 2); // 20ct (Hopper, adapter slot C, cctalk sort chute 2)
    modifySorterPath(CCTALK_COIN_VALIDATOR, 4, 1); // 50ct (Kasse, adapter slot D, cctalk sort chute 1)
    modifySorterPath(CCTALK_COIN_VALIDATOR, 5, 1); // 1eur (Kasse, adapter slot D, cctalk sort chute 1)
    modifySorterPath(CCTALK_COIN_VALIDATOR, 6, 1); // 2eur (Kasse, adapter slot D, cctalk sort chute 1)

    modifyDefaultSorterPath(CCTALK_COIN_VALIDATOR, 1); // adapter slot D, cctalk sort chute 1
    modifyInhibitStatus(CCTALK_COIN_VALIDATOR, 254, 0); // Allow all coins except 5ct
    modifyMasterInhibitStatus(CCTALK_COIN_VALIDATOR, 1); // Enable coin validator

    
    strResponse = requestManufacturerId(CCTALK_HOPPER);    
    ESP_LOGI(TAG, "Hopper manufacturer Id: %s", strResponse.c_str());    
    strResponse = requestBuildCode(CCTALK_HOPPER);    
    ESP_LOGI(TAG, "Build Code: %s", strResponse.c_str());    
    strResponse = requestProductCode(CCTALK_HOPPER);    
    ESP_LOGI(TAG, "Product Code: %s", strResponse.c_str());  

    return ESP_OK;
}

cctalk_response_t* CCTalkController::resetDevice(const uint8_t address) {
    cctalk_response_t* response = sendRequest(RESET_DEVICE, address, NULL, 0);
    return response;
}

string CCTalkController::requestManufacturerId(const uint8_t address) {
    cctalk_response_t* response = sendRequest(REQUEST_MANUFACTURER_ID, address, NULL, 0);
    return getStringResponse(response);
}

string CCTalkController::requestBuildCode(const uint8_t address) {
    cctalk_response_t* response = sendRequest(REQUEST_BUILD_CODE, address, NULL, 0);
    return getStringResponse(response);
}

string CCTalkController::requestSerialNumber(const uint8_t address) {
    cctalk_response_t* response = sendRequest(REQUEST_SERIAL_NUMBER, address, NULL, 0);
    return getStringResponse(response);
}

string CCTalkController::requestProductCode(const uint8_t address) {
    cctalk_response_t* response = sendRequest(REQUEST_PRODUCT_CODE, address, NULL, 0);
    return getStringResponse(response);
}

cctalk_response_t* CCTalkController::pollCredit(const uint8_t address) {

    //TODO maybe do something more complicated here with a structure
    return sendRequest(READ_BUFFERED_CREDIT_OR_ERROR_CODES, address, NULL, 0);
}

cctalk_response_t* CCTalkController::testHopper(const uint8_t address) {
    return sendRequest(TEST_HOPPER, address, NULL, 0);
}

cctalk_response_t* CCTalkController::pollHopperStatus(const uint8_t address) {
    return sendRequest(REQUEST_HOPPER_STATUS, address, NULL, 0);
}

cctalk_response_t* CCTalkController::modifyDefaultSorterPath(const uint8_t address, const uint8_t defaultChute) {

    uint8_t tmpData[1];
    tmpData[0] = defaultChute;
    return sendRequest(MODIFY_DEFAULT_SORTER_PATH, address, tmpData, (uint8_t) 1);
}

cctalk_response_t* CCTalkController::modifySorterPath(const uint8_t address, const uint8_t path, const uint8_t chute) {

    uint8_t tmpData[2];
    tmpData[0] = path;
    tmpData[1] = chute;

    return sendRequest(MODIFY_SORTER_PATHS, address, tmpData, (uint8_t) 2);

}

cctalk_response_t* CCTalkController::modifyInhibitStatus(const uint8_t address, const uint8_t enable1, const uint8_t enable2) {
    uint8_t tmpData[2];
    tmpData[0] = enable1;
    tmpData[1] = enable2;
    return sendRequest(MODIFY_INHIBIT_STATUS, address, tmpData, (uint8_t) 2);
}

cctalk_response_t* CCTalkController::modifyMasterInhibitStatus(const uint8_t address, const uint8_t enable) {
    uint8_t tmpData[1];
    tmpData[0] = enable;
    return sendRequest(MODIFY_MASTER_INHIBIT_STATUS, address, tmpData, (uint8_t) 1);
}

cctalk_response_t* CCTalkController::enableHopper(const uint8_t address) {
    uint8_t tmpData[1];
    tmpData[0] = 165;
    return sendRequest(ENABLE_HOPPER, address, tmpData, (uint8_t) 1);
}

cctalk_response_t* CCTalkController::dispenseCoins(const uint8_t address, const uint8_t numCoins) {
    
    uint8_t tmpData[9];

    // cipher key should be 8 bytes long. 
    cctalk_response_t *result = requestCipherKey(address);

    // copy 8-byte cipher key to tmpData to be passed to dispense coins. 9th byte is the number of coins to dispense.
    for (int i = 0; i < 8; i++) {
        tmpData[i] = result->additionalData[i];
    }

    ESP_LOGI(TAG, "Cipher Key: %03d %03d %03d %03d %03d %03d %03d %03d", tmpData[0], tmpData[1], tmpData[2], tmpData[3], tmpData[4], tmpData[5], tmpData[6], tmpData[7]);
    ESP_LOGI(TAG, "Checking response");
    ESP_LOGI(TAG, "Dispensing %02d coin(s).", numCoins);

    // In the 9th byte we have the number of coins.
    tmpData[8] = numCoins;


    return sendRequest(DISPENSE_HOPPER_COINS, address, tmpData, (uint8_t) 9);
}

cctalk_response_t* CCTalkController::requestCipherKey(const uint8_t address) {
    ESP_LOGD(TAG, "Cipher key requested");

    return sendRequest(REQUEST_CIPHER_KEY, address, NULL, 0);
}

cctalk_response_t* CCTalkController::requestPayoutHighLowStatus(const uint8_t address) {
    ESP_LOGD(TAG, "Request payout high/low status called");

    return sendRequest(REQUEST_PAYOUT_HIGHLOW_STATUS, address, NULL, 0);
}

/*
 * @brief Sends a request to the given ccTalk device
 * 
 *  
 */
cctalk_response_t* CCTalkController::sendRequest(cctalk_header_e header, const uint8_t address, uint8_t *data, uint8_t length) {

    ESP_LOGD(TAG, "sendRequest called with header %d to address %d", header, address);

    cctalk_request_t* request = (cctalk_request_t*) calloc(1, sizeof (cctalk_request_t));
    request->source = CCTALK_HOST;
    request->destination = address;
    request->header = header;
    if (data != NULL) {
        for (int i = 0; i < length; i++) {
            request->additionalData[i] = data[i];
        }
        request->additionalDataSize = length;
    } else {
        request->additionalDataSize = 0;
    }

    cctalk_response_t *response = cctalk_send_request(this->cctalkDevice, request);

    free(request); // free up the memory

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

std::string CCTalkController::getStringResponse(cctalk_response_t *response) {

    std::string retval;

    if (response) {
        retval.append(reinterpret_cast<const char *> (response->additionalData), response->additionalDataSize);
    }

    return retval;
}

