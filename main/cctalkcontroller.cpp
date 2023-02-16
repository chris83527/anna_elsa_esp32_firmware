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
    
    cctalkLinkController.initialise(CCTALK_UART, CCTALK_GPIO_TX, CCTALK_GPIO_RX, false, false);
    
    this->hopper.initialise(this->cctalkLinkController, 1, [=](const std::string& error_msg) {
        ESP_LOGE(TAG, "An error occurred initialising the hopper: %s", error_msg.c_str());
    }); 
    
    this->coinAcceptor.initialise(this->cctalkLinkController, 3, [=](const std::string& error_msg) {
        ESP_LOGE(TAG, "An error occurred initialising the coin acceptor: %s", error_msg.c_str());
    });   
                   
    return ESP_OK;
}



