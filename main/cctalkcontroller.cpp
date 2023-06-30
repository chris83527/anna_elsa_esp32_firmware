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

    cctalkLinkController.initialise(CCTALK_UART, CCTALK_GPIO_TX, CCTALK_GPIO_RX, false, false);

    this->hopper.initialise(&this->cctalkLinkController, CCTALK_HOPPER, [ = ](const std::string & error_msg){
        if (error_msg.size() > 0) {
            ESP_LOGE(TAG, "An error occurred initialising the hopper: %s", error_msg.c_str());
        }
    });

    this->coinAcceptor.initialise(&this->cctalkLinkController, CCTALK_COIN_VALIDATOR, [ = ](const std::string & error_msg){
        if (error_msg.size() > 0) {
            ESP_LOGE(TAG, "An error occurred initialising the coin acceptor: %s", error_msg.c_str());
        }
    });

    this->hopper.requestResetDevice([ = ](const std::string & error_msg){
        if (error_msg.size() > 0) {
            ESP_LOGE(TAG, "%s", error_msg.c_str());
        }
    });

    this->coinAcceptor.requestResetDevice([ = ](const std::string & error_msg){
        if (error_msg.size() > 0) {
            ESP_LOGE(TAG, "%s", error_msg.c_str());
        }
    });

    // adapter slot D, cctalk sort chute 1
    this->coinAcceptor.modifyDefaultSorterPath(1, [&](const std::string & error_msg) {
        if (error_msg.size() > 0) {
            ESP_LOGE(TAG, "%s", error_msg.c_str());
        }
    });

    // 5ct  (Kasse - rejected anyway)
    this->coinAcceptor.modifySorterPath(1, 1, [&](const std::string & error_msg) {
        if (error_msg.size() > 0) {
            ESP_LOGE(TAG, "%s", error_msg.c_str());
        }
    });

    // 10ct (Kasse, adapter slot D, cctalk sort chute 1)
    this->coinAcceptor.modifySorterPath(2, 1, [&](const std::string & error_msg) {
        if (error_msg.size() > 0) {
            ESP_LOGE(TAG, "%s", error_msg.c_str());
        }
    });

    // 20ct (Hopper, adapter slot C, cctalk sort chute 2)
    this->coinAcceptor.modifySorterPath(3, 2, [&](const std::string & error_msg) {
        if (error_msg.size() > 0) {
            ESP_LOGE(TAG, "%s", error_msg.c_str());
        }
    });

    // 50ct (Kasse, adapter slot D, cctalk sort chute 1)
    this->coinAcceptor.modifySorterPath(4, 1, [&](const std::string & error_msg) {
        if (error_msg.size() > 0) {
            ESP_LOGE(TAG, "%s", error_msg.c_str());
        }
    });

    // 1eur (Kasse, adapter slot D, cctalk sort chute 1)
    this->coinAcceptor.modifySorterPath(5, 1, [&](const std::string & error_msg) {
        if (error_msg.size() > 0) {
            ESP_LOGE(TAG, "%s", error_msg.c_str());
        }
    });

    // 2eur (Kasse, adapter slot D, cctalk sort chute 1)
    this->coinAcceptor.modifySorterPath(6, 1, [&](const std::string & error_msg) {
        if (error_msg.size() > 0) {
            ESP_LOGE(TAG, "%s", error_msg.c_str());
        }
    });

    // the coin validator automatically sends coins to cash box - stop this.
    this->coinAcceptor.modifySorterOverrideStatus(255, []&](const std::string & error_msg) {
        if (error_msg.size() > 0) {
            ESP_LOGE(TAG, "%s", error_msg.c_str());
        }
    });
    
    
    // Allow all coins except 5ct
    this->coinAcceptor.modifyInhibitStatus(254, 0, [&](const std::string & error_msg) {
        if (error_msg.size() > 0) {
            ESP_LOGE(TAG, "%s", error_msg.c_str());
        }
    });

    esp32cc::CcDeviceState acceptingState = esp32cc::CcDeviceState::NormalAccepting;
    this->coinAcceptor.requestSwitchDeviceState(acceptingState, [&]([[maybe_unused]] const std::string & error_msg) {
        if (error_msg.size() > 0) {
            ESP_LOGE(TAG, "An error occurred switching to accept state: %s", error_msg.c_str());
        }
    });

    return ESP_OK;
}

/**
 * @brief The callback to be executed if a credit event is received
 * 
 * @param creditAcceptedCallback
 */
void CCTalkController::setCreditAcceptedCallback(esp32cc::CoinAcceptorDevice::CreditAcceptedFunc creditAcceptedCallback) {
    this->coinAcceptor.setCreditAcceptedCallback(creditAcceptedCallback);
}