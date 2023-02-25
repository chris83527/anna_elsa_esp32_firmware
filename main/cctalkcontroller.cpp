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

#include "oledcontroller.h"

#include "cctalkcontroller.h"

static const char *TAG = "CCTALK_CONTROLLER";

CCTalkController::CCTalkController() {
    ESP_LOGD(TAG, "Entering constructor");

    ESP_LOGD(TAG, "Leaving constructor");
}

CCTalkController::CCTalkController(const CCTalkController& orig) {
}

CCTalkController::~CCTalkController() {
}

esp_err_t CCTalkController::initialise() {
    ESP_LOGD(TAG, "CCTalkController::initialise called");
    cctalkLinkController = new esp32cc::CctalkLinkController();

    cctalkLinkController->initialise(CCTALK_UART, CCTALK_GPIO_TX, CCTALK_GPIO_RX, false, false);

//    this->hopper.initialise(this->cctalkLinkController, CCTALK_HOPPER, [ = ](const std::string & error_msg){
//        if (error_msg.size() > 0) {
//            ESP_LOGE(TAG, "An error occurred initialising the hopper: %s", error_msg.c_str());
//        }
//    });

    this->coinAcceptor.initialise(this->cctalkLinkController, CCTALK_COIN_VALIDATOR, [ = ](const std::string & error_msg){
        if (error_msg.size() > 0) {
            ESP_LOGE(TAG, "An error occurred initialising the coin acceptor: %s", error_msg.c_str());
        }
    });

    this->coinAcceptor.modifySorterPath(1, 1, [&](const std::string & error_msg) {

    });
    this->coinAcceptor.modifySorterPath(2, 1, [&](const std::string & error_msg) {

    });
    this->coinAcceptor.modifySorterPath(3, 2, [&](const std::string & error_msg) {

    });
    this->coinAcceptor.modifySorterPath(4, 1, [&](const std::string & error_msg) {

    });
    this->coinAcceptor.modifySorterPath(5, 1, [&](const std::string & error_msg) {

    });
    this->coinAcceptor.modifySorterPath(6, 1, [&](const std::string & error_msg) {

    });
    this->coinAcceptor.modifyDefaultSorterPath(1, [&](const std::string & error_msg) {

    });

    esp32cc::CcDeviceState acceptingState = esp32cc::CcDeviceState::NormalAccepting;
    this->coinAcceptor.requestSwitchDeviceState(acceptingState, [&]([[maybe_unused]] const std::string & error_msg) {
        if (error_msg.size() > 0) {
            ESP_LOGE(TAG, "An error occurred switching to accept state: %s", error_msg.c_str());
        }
    });


    return ESP_OK;
}

void CCTalkController::setCreditAcceptedCallback(esp32cc::CoinAcceptorDevice::CreditAcceptedFunc creditAcceptedCallback) {
    this->coinAcceptor.setCreditAcceptedCallback(creditAcceptedCallback);
}

/**
 esp_err_t CCTalkController::initialise() {
    ESP_LOGD(TAG, "CCTalkController::initialise called");

    cctalk.initialise();

    CctalkResponse response;

    resetDevice(CCTALK_COIN_VALIDATOR, response);
    resetDevice(CCTALK_HOPPER, response);

    requestManufacturerId(CCTALK_COIN_VALIDATOR, response);
    ESP_LOGI(TAG, "Coin validator manufacturer Id: %s", response.asStringResponse().c_str());
    mainController->getOledController()->scrollText(std::string("Manufacturer Id:"));
    mainController->getOledController()->scrollText(std::string(response.asStringResponse()));
    requestBuildCode(CCTALK_COIN_VALIDATOR, response);
    ESP_LOGI(TAG, "Build Code: %s", response.asStringResponse().c_str());
    mainController->getOledController()->scrollText(std::string("Build code: "));
    mainController->getOledController()->scrollText(std::string(response.asStringResponse()));
    requestProductCode(CCTALK_COIN_VALIDATOR, response);
    ESP_LOGI(TAG, "Product Code: %s", response.asStringResponse().c_str());
    mainController->getOledController()->scrollText(std::string("Product code: "));
    mainController->getOledController()->scrollText(std::string(response.asStringResponse()));

    modifySorterPath(CCTALK_COIN_VALIDATOR, 1, 1, response); // 5ct  (Kasse - rejected anyway)
    modifySorterPath(CCTALK_COIN_VALIDATOR, 2, 1, response); // 10ct (Kasse, adapter slot D, cctalk sort chute 1)
    modifySorterPath(CCTALK_COIN_VALIDATOR, 3, 2, response); // 20ct (Hopper, adapter slot C, cctalk sort chute 2)
    modifySorterPath(CCTALK_COIN_VALIDATOR, 4, 1, response); // 50ct (Kasse, adapter slot D, cctalk sort chute 1)
    modifySorterPath(CCTALK_COIN_VALIDATOR, 5, 1, response); // 1eur (Kasse, adapter slot D, cctalk sort chute 1)
    modifySorterPath(CCTALK_COIN_VALIDATOR, 6, 1, response); // 2eur (Kasse, adapter slot D, cctalk sort chute 1)

    modifyDefaultSorterPath(CCTALK_COIN_VALIDATOR, 1, response); // adapter slot D, cctalk sort chute 1
    modifyInhibitStatus(CCTALK_COIN_VALIDATOR, 254, 0, response); // Allow all coins except 5ct
    modifyMasterInhibitStatus(CCTALK_COIN_VALIDATOR, 1, response); // Enable coin validator

    requestManufacturerId(CCTALK_HOPPER, response);
    ESP_LOGI(TAG, "Hopper manufacturer Id: %s", response.asStringResponse().c_str());
    requestBuildCode(CCTALK_HOPPER, response);
    ESP_LOGI(TAG, "Build Code: %s", response.asStringResponse().c_str());
    requestProductCode(CCTALK_HOPPER, response);
    ESP_LOGI(TAG, "Product Code: %s", response.asStringResponse().c_str());

    //dispenseCoins(CCTALK_HOPPER, 1, response);

    return ESP_OK;
}
 */
