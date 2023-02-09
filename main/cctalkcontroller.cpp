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
#include "esp_log.h"
#include "esp_err.h"

#include "config.h"

#include "maincontroller.h"

#include "cctalkcontroller.h"
#include "oledcontroller.h"

using namespace std;

static const char *TAG = "CCTALK_CONTROLLER";

const uint8_t CCTalkController::COIN_VALUES[] = {0, 5, 10, 20, 50, 100, 200};

//Cctalk cctalk(UART_NUM_1, CCTALK_GPIO_TX, CCTALK_GPIO_RX);

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

//    this->coinAcceptor.initialise([ = ](const std::string & errorMessage) {
//        this->mainController->getOledController()->scrollText(errorMessage);
//    });
//    
    
//    resetDevice(CCTALK_COIN_VALIDATOR, response);
//    resetDevice(CCTALK_HOPPER, response);
//
//    requestManufacturerId(CCTALK_COIN_VALIDATOR, response);
//    ESP_LOGI(TAG, "Coin validator manufacturer Id: %s", response.asStringResponse().c_str());
//    mainController->getOledController()->scrollText("Manufacturer Id:");
//    mainController->getOledController()->scrollText(response.asStringResponse());
//    requestBuildCode(CCTALK_COIN_VALIDATOR, response);
//    ESP_LOGI(TAG, "Build Code: %s", response.asStringResponse().c_str());
//    mainController->getOledController()->scrollText("Build code: ");
//    mainController->getOledController()->scrollText(response.asStringResponse());
//    requestProductCode(CCTALK_COIN_VALIDATOR, response);
//    ESP_LOGI(TAG, "Product Code: %s", response.asStringResponse().c_str());
//    mainController->getOledController()->scrollText("Product code: ");
//    mainController->getOledController()->scrollText(response.asStringResponse());
//
//    modifySorterPath(CCTALK_COIN_VALIDATOR, 1, 1, response); // 5ct  (Kasse - rejected anyway)
//    modifySorterPath(CCTALK_COIN_VALIDATOR, 2, 1, response); // 10ct (Kasse, adapter slot D, cctalk sort chute 1)
//    modifySorterPath(CCTALK_COIN_VALIDATOR, 3, 2, response); // 20ct (Hopper, adapter slot C, cctalk sort chute 2)
//    modifySorterPath(CCTALK_COIN_VALIDATOR, 4, 1, response); // 50ct (Kasse, adapter slot D, cctalk sort chute 1)
//    modifySorterPath(CCTALK_COIN_VALIDATOR, 5, 1, response); // 1eur (Kasse, adapter slot D, cctalk sort chute 1)
//    modifySorterPath(CCTALK_COIN_VALIDATOR, 6, 1, response); // 2eur (Kasse, adapter slot D, cctalk sort chute 1)
//
//    modifyDefaultSorterPath(CCTALK_COIN_VALIDATOR, 1, response); // adapter slot D, cctalk sort chute 1
//    modifyInhibitStatus(CCTALK_COIN_VALIDATOR, 254, 0, response); // Allow all coins except 5ct
//    modifyMasterInhibitStatus(CCTALK_COIN_VALIDATOR, 1, response); // Enable coin validator
//
//    requestManufacturerId(CCTALK_HOPPER, response);
//    ESP_LOGI(TAG, "Hopper manufacturer Id: %s", response.asStringResponse().c_str());
//    requestBuildCode(CCTALK_HOPPER, response);
//    ESP_LOGI(TAG, "Build Code: %s", response.asStringResponse().c_str());
//    requestProductCode(CCTALK_HOPPER, response);
//    ESP_LOGI(TAG, "Product Code: %s", response.asStringResponse().c_str());

    //dispenseCoins(CCTALK_HOPPER, 1, response);

    return ESP_OK;
}

//void CCTalkController::resetDevice(const uint8_t destination, CctalkResponse &response) {
//    std::vector<uint8_t> additionalData;
//    sendRequest(Cctalk::Headers::RESET_DEVICE, destination, additionalData, response);
//}
//
//void CCTalkController::requestManufacturerId(const uint8_t destination, CctalkResponse &response) {
//    std::vector<uint8_t> additionalData;
//    sendRequest(Cctalk::Headers::REQUEST_MANUFACTURER_ID, destination, additionalData, response);
//}
//
//void CCTalkController::requestBuildCode(const uint8_t destination, CctalkResponse &response) {
//    std::vector<uint8_t> additionalData;
//    sendRequest(Cctalk::Headers::REQUEST_BUILD_CODE, destination, additionalData, response);
//}
//
//void CCTalkController::requestSerialNumber(const uint8_t destination, CctalkResponse &response) {
//    std::vector<uint8_t> additionalData;
//    sendRequest(Cctalk::Headers::REQUEST_SERIAL_NUMBER, destination, additionalData, response);
//}
//
//void CCTalkController::requestProductCode(const uint8_t destination, CctalkResponse &response) {
//    std::vector<uint8_t> additionalData;
//    sendRequest(Cctalk::Headers::REQUEST_PRODUCT_CODE, destination, additionalData, response);
//}
//
//void CCTalkController::pollCredit(const uint8_t destination, CctalkResponse &response) {
//    std::vector<uint8_t> additionalData;
//    sendRequest(Cctalk::Headers::READ_BUFFERED_CREDIT_OR_ERROR_CODES, destination, additionalData, response);
//}
//
//void CCTalkController::testHopper(const uint8_t destination, CctalkResponse &response) {
//    std::vector<uint8_t> additionalData;
//    sendRequest(Cctalk::Headers::TEST_HOPPER, destination, additionalData, response);
//}
//
//void CCTalkController::pollHopperStatus(const uint8_t destination, CctalkResponse &response) {
//    std::vector<uint8_t> additionalData;
//    sendRequest(Cctalk::Headers::REQUEST_HOPPER_STATUS, destination, additionalData, response);
//}
//
//void CCTalkController::modifyDefaultSorterPath(const uint8_t destination, const uint8_t defaultChute, CctalkResponse &response) {
//
//    std::vector<uint8_t> additionalData;
//    additionalData.push_back(defaultChute);
//
//    sendRequest(Cctalk::Headers::MODIFY_DEFAULT_SORTER_PATH, destination, additionalData, response);
//}
//
//void CCTalkController::modifySorterPath(const uint8_t destination, const uint8_t path, const uint8_t chute, CctalkResponse &response) {
//
//    std::vector<uint8_t> additionalData;
//
//    additionalData.push_back(path);
//    additionalData.push_back(chute);
//
//    sendRequest(Cctalk::Headers::MODIFY_SORTER_PATHS, destination, additionalData, response);
//
//}
//
//void CCTalkController::modifyInhibitStatus(const uint8_t destination, const uint8_t enable1, const uint8_t enable2, CctalkResponse &response) {
//    std::vector<uint8_t> additionalData;
//
//    additionalData.push_back(enable1);
//    additionalData.push_back(enable2);
//
//    sendRequest(Cctalk::Headers::MODIFY_INHIBIT_STATUS, destination, additionalData, response);
//}
//
//void CCTalkController::modifyMasterInhibitStatus(const uint8_t destination, const uint8_t enable, CctalkResponse &response) {
//    std::vector<uint8_t> additionalData;
//    additionalData.push_back(enable);
//
//    sendRequest(Cctalk::Headers::MODIFY_MASTER_INHIBIT_STATUS, destination, additionalData, response);
//}
//
//void CCTalkController::enableHopper(const uint8_t destination, CctalkResponse &response) {
//    std::vector<uint8_t> additionalData;
//
//    additionalData.push_back(165);
//
//    sendRequest(Cctalk::Headers::ENABLE_HOPPER, destination, additionalData, response);
//}
//
//
//void CCTalkController::dispenseCoins(const uint8_t destination, const uint8_t numCoins, CctalkResponse &response) {
//
//    CctalkResponse tmpResponse;
//
//    testHopper(destination, tmpResponse);
//    if (!tmpResponse.isValidResponse()) {
//        ESP_LOGE(TAG, "Test hopper command failed");
//        return;
//    }
//
//    // Prepare for payout
//    enableHopper(destination, tmpResponse);
//    if (!tmpResponse.isValidResponse()) {
//        ESP_LOGE(TAG, "Enable hopper command failed");
//        return;
//    }
//
//    // cipher key should be 8 bytes long.         
//    requestCipherKey(destination, tmpResponse);
//    if (!tmpResponse.isValidResponse()) {
//        ESP_LOGE(TAG, "Request cipher key command failed");
//        return;
//    }
//
//    std::vector<uint8_t> additionalData = tmpResponse.getAdditionalData();
//
//    if (additionalData.size() != 8) {
//        ESP_LOGE(TAG, "Additional data size was %d, expecting 8", additionalData.size());
//        return;
//    }
//
//    ESP_LOGI(TAG, "Cipher Key: %03d %03d %03d %03d %03d %03d %03d %03d", additionalData.at(0), additionalData.at(1), additionalData.at(2), additionalData.at(3), additionalData.at(4), additionalData.at(5), additionalData.at(6), additionalData.at(7));
//    ESP_LOGI(TAG, "Checking response");
//    ESP_LOGI(TAG, "Dispensing %02d coin(s).", numCoins);
//
//    // In the 9th byte we have the number of coins.
//    additionalData.push_back(numCoins);
//
//    sendRequest(Cctalk::Headers::DISPENSE_HOPPER_COINS, destination, additionalData, response);
//
//}
//
//void CCTalkController::requestCipherKey(const uint8_t destination, CctalkResponse &response) {
//    ESP_LOGD(TAG, "Cipher key requested");
//
//    std::vector<uint8_t> additionalData;
//
//    sendRequest(Cctalk::Headers::REQUEST_CIPHER_KEY, destination, additionalData, response);
//}
//
//void CCTalkController::requestPayoutHighLowStatus(const uint8_t destination, CctalkResponse &response) {
//    ESP_LOGD(TAG, "Request payout high/low status called");
//
//    std::vector<uint8_t> additionalData;
//
//    sendRequest(Cctalk::Headers::REQUEST_PAYOUT_HIGHLOW_STATUS, destination, additionalData, response);
//}
//
///*
// * @brief Sends a request to the given ccTalk device
// * 
// *  
// */
//void CCTalkController::sendRequest(Cctalk::Headers header, uint8_t destination, std::vector<uint8_t> &data, CctalkResponse &response) {
//
//    CctalkRequest request;
//    request.initialise();
//    request.setSource(CCTALK_HOST);
//    request.setDestination(destination);
//    request.setHeader(header);
//    request.setAdditionalData(data);
//    
//    response.initialise();
//
//    ESP_LOGD(TAG, "sendRequest called with header %d to destination %d", request.getHeader(), request.getDestination());
//
//    try {
//        cctalk.sendRequest(request, response);
//    } catch (CctalkException& e) {
//        ESP_LOGD(TAG, "An exception occurred sending cctalk request: %s", e.what());
//    }
//}


