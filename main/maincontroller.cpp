/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   MainController.cpp
 * Author: chris
 *
 * Created on July 28, 2018, 6:33 PM
 */
#include <string>
#include <stdlib.h>
#include <ctime>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "ds3231.h"
#include "esp_wifi.h"


#include "config.h"
#include "reelcontroller.h"
#include "displaycontroller.h"
#include "cctalkcontroller.h"
#include "game.h"
#include "audiocontroller.h"
#include "moneycontroller.h"
#include "maincontroller.h"
#include "webserver.h"
#include "spiffs.h"
#include "wificontroller.h"
//#include "errors.h"

static const char *TAG = "MainController";

void blinkCPUStatusLEDTask(void *pvParameter) {
    while (1) {
        /* Blink off (output low) */
        gpio_set_level(CPU_LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        /* Blink on (output high) */
        gpio_set_level(CPU_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void MainController::cctalkStatusCheckTask(void *pvParameter) {
    ESP_LOGD(TAG, "Hopper status check task started.");
    MainController *mainController = reinterpret_cast<MainController *> (pvParameter);
    CCTalkController *cctalkController = mainController->getCCTalkController();
    MoneyController *moneyController = mainController->getMoneyController();
    
    int validatorEventCounter = 0;

    CctalkResponse response;

    for (;;) {        

        if (!moneyController->isPayoutInProgress()) {

            // now switch to polling for credit
            cctalkController->pollCredit(CCTALK_COIN_VALIDATOR, response);

            if (response.isValidResponse() && response.getAdditionalData().size() > 0) {
                uint8_t tmpValidatorEventCounter = response.getAdditionalData().at(0);

                if (tmpValidatorEventCounter > validatorEventCounter) {
                    ESP_LOGD(TAG, "Current Coin validator event count: %d, Previous count: %d ", response.getAdditionalData().at(0), validatorEventCounter);

                    int numCoinEventsToProcess = tmpValidatorEventCounter - validatorEventCounter;

                    Payment payment;
                    payment.clear();
                    for (int i = 1; i <= numCoinEventsToProcess; i += 2) {
                        ESP_LOGD(TAG, "Coin value for event %d: %d ", i, CCTalkController::COIN_VALUES[response.getAdditionalData().at(i)]);
                        switch (response.getAdditionalData().at(i)) {
                            case 2:
                                payment.addTenCent();
                                break;
                            case 3:
                                payment.addTwentyCent();
                                break;
                            case 4:
                                payment.addFiftyCent();
                                break;
                            case 5:
                                payment.addOneEuro();
                                break;
                            case 6:
                                payment.addTwoEuro();
                                break;
                        }
                    }
                    moneyController->addToCredit(payment);
                }
                validatorEventCounter = tmpValidatorEventCounter;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(75));
    }
}

MainController::MainController() {
    ESP_LOGD(TAG, "Entering constructor");
    ESP_LOGD(TAG, "Leaving constructor");
}

MainController::MainController(const MainController& orig) {
}

void MainController::start() {
    ESP_LOGD(TAG, "start() called");

    this->cctalkController = new CCTalkController(this);
    this->audioController = new AudioController();
    this->displayController = new DisplayController(this);
    this->reelController = new ReelController(this);
    this->moneyController = new MoneyController(this);
    this->game = new Game(this);
    this->wifiController = new WifiController();

    esp_err_t err;

    wifiController->initialiseWifi();
    
    ESP_LOGD(TAG, "Calling i2cdev_init()");
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2cdev_init());

    
    // Initialize NVS
    ESP_LOGD(TAG, "Setting up NVS");
    // Initialize NVS
    err = nvs_flash_init_partition(NVS_PARTITION_SETTINGS);
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase_partition(NVS_PARTITION_SETTINGS));
        err = nvs_flash_init_partition(NVS_PARTITION_SETTINGS);
    }
    ESP_ERROR_CHECK(err);

    nvs_handle = nvs::open_nvs_handle_from_partition(NVS_PARTITION_SETTINGS, NVS_PARTITION_SETTINGS, NVS_READWRITE, &err);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    } else {
        ESP_LOGD(TAG, "NVS opened ok.");
    }

    // initialise ds3231 RTC

    err = ds3231_init_desc(&ds3231, 0, GPIO_I2C_SDA, GPIO_I2C_SCL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error initialising RTC!");
    } else {
        ESP_LOGI(TAG, "RTC initialised ok");
    }
          
    init_spiffs();
    init_webserver("/spiffs");

    // intialise audio subsystem    
    audioController->initialise();
    
    if (displayController->initialise() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialise tableau subsystem");
    } else {
        ESP_LOGD(TAG, "Display controller initialisation ok.");
    }
    
    moneyController->initialise();

    if (reelController->initialise() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialise reel controller subsystem");
    } else {
        ESP_LOGD(TAG, "Reel controller initialisation ok.");
    }     
    
    xTaskCreate(&blinkCPUStatusLEDTask, "cpu_status_led_blink", 2048, this, 1, NULL);

    if (cctalkController->initialise() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialise ccTalk subsystem");
    } else {
        ESP_LOGD(TAG, "Starting validator and hopper status polling");
        xTaskCreate(&cctalkStatusCheckTask, "check_hopper_status", 10000, this, 14, NULL);
    }
    
    //audioController->playAudioFile(Sounds::SND_LET_IT_GO);

    game->initialise();
    this->displayController->beginAttractMode();

    for (;;) {
        if (!(game->isGameInProgress()) && (this->moneyController->getCredit() >= 20)) {
            ESP_LOGD(TAG, "Starting game...");
            this->game->start();            
        } else {
            if (!getDisplayController()->isAttractMode()) {
                getDisplayController()->beginAttractMode();
            }
        }
        
        

        vTaskDelay(pdMS_TO_TICKS(200));
    }

}

void MainController::payout() {    
    
    ESP_LOGI(TAG, "Entering %s", __func__);
    moneyController->setPayoutInProgress(true);
    
    uint8_t numCoins = (this->moneyController->getBank() / 20);
    ESP_LOGI(TAG, "Paying out %d coins", numCoins);

    CctalkResponse response;
    cctalkController->dispenseCoins(CCTALK_HOPPER, numCoins, response);
    

    while (moneyController->isPayoutInProgress()) {
        cctalkController->pollHopperStatus(CCTALK_HOPPER, response);
        ESP_LOGI(TAG, "Polling hopper status");
        if (response.isValidResponse() && response.getAdditionalData().size() >= 4) {            
            uint8_t tmpHopperEventCounter = response.getAdditionalData().at(0);
            uint8_t coinsRemaining = response.getAdditionalData().at(1);
            uint8_t coinsPaid = response.getAdditionalData().at(2);            
            uint8_t coinsUnpaid = response.getAdditionalData().at(3);
            ESP_LOGI(TAG, "Event counter: %d, Coins remaining: %d, Coins paid: %d, Coins unpaid: %d", tmpHopperEventCounter, coinsRemaining, coinsPaid, coinsUnpaid);

            if (tmpHopperEventCounter > hopperEventCounter) { 
                // all coins have been paid, or some could not be paid
                if (coinsRemaining == 0) {
                    ESP_LOGI(TAG, "Coins paid: %d", coinsPaid);
                    hopperEventCounter = tmpHopperEventCounter;
                    moneyController->setPayoutInProgress(false);
                    moneyController->removeFromBank(coinsPaid * 20);
                } else if (coinsUnpaid > 0) {
                    ESP_LOGE(TAG, "Coins unpaid: %d", coinsUnpaid);
                    moneyController->setPayoutInProgress(false);
                    // TODO: Error here (probably hopper was empty)
                    //processHopperErrors();
                } else {
                    ESP_LOGI(TAG, "Coins remaining: %d", coinsRemaining);                    
                }
            } 
            
        } else {
            ESP_LOGE(TAG, "Invalid response received when polling hopper status");
            moneyController->setPayoutInProgress(false);
                // TODO: We got a negative response polling hopper status. This means we should find out what went wrong
            cctalkController->testHopper(CCTALK_HOPPER, response);

        }
        //TODO: Unpaid coins
        vTaskDelay(75);
    }

    ESP_LOGI(TAG, "Exiting %s", __func__);
}

//void MainController::dumpEEPROMValues() {
//    this->getSerialMonitorController()->clearScreenAndDrawBorder();
//    term->println("EEPROM dump");
//    char buf[255];
//    snprintf(buf, sizeof (buf), "Credit: %05u", this->eeprom_data.credit);
//    term->println(buf);
//    snprintf(buf, sizeof (buf), "Bank: %05u", this->eeprom_data.bank);
//    term->println(buf);
//    snprintf(buf, sizeof (buf), "Games Played (total): %05lu", this->eeprom_data.numGamesPlayed);
//    term->println(buf);
//    snprintf(buf, sizeof (buf), "10ct in: %05lu", this->eeprom_data.in10ct);
//    term->println(buf);
//    snprintf(buf, sizeof (buf), "20ct in: %05lu", this->eeprom_data.in20ct);
//    term->println(buf);
//    snprintf(buf, sizeof (buf), "50ct in: %05lu", this->eeprom_data.in50ct);
//    term->println(buf);
//    snprintf(buf, sizeof (buf), "1Eur in: %05lu", this->eeprom_data.in1eur);
//    term->println(buf);
//    snprintf(buf, sizeof (buf), "2Eur in: %05lu", this->eeprom_data.in2eur);
//    term->println(buf);
//    snprintf(buf, sizeof (buf), "Coins out: %05lu", this->eeprom_data.coinsOut);
//    term->println(buf);
//    snprintf(buf, sizeof (buf), "Volume: %02u", this->eeprom_data.volume);
//    term->println(buf);
//
//}

void MainController::setDateTime() {
    tm time;
    time.tm_hour = 0;
    time.tm_min = 0;
    time.tm_sec = 0;
    time.tm_isdst = true;
    time.tm_mon = 11;
    time.tm_year = 2021;
    time.tm_mday = 27;

    ds3231_set_time(&ds3231, &time);

    //    serialMonitorController->clearScreenAndDrawBorder();
    //    term->position(4, 2);
    //    term->println(F("Please enter the current date/time (yyyymmddHHMMSS"));
    //    String dateString;
    //
    //    for (int i = 0; i < 14; i++) {
    //        char c = term->read();
    //        while (c == -1) {
    //            c = term->read();
    //        }
    //        term->write(c);
    //        dateString.concat(c);
    //    }
    //
    //    int year = dateString.substring(0, 4).toInt();
    //    int month = dateString.substring(4, 6).toInt();
    //    int day = dateString.substring(6, 8).toInt();
    //    int hours = dateString.substring(8, 10).toInt();
    //    int minutes = dateString.substring(10, 12).toInt();
    //    int seconds = dateString.substring(12, 14).toInt();
    //
    //    setTime(hours, minutes, seconds, day, month, year);
    //    time_t t = now();
    //    RTC.set(t);
}

//void MainController::printDate() {
//    ESP_LOGI(TAG, )
//}
////
//std::string MainController::getDateTime() {
//    tm time; 
//    std::string dateTimeString;
//    ds3231_get_time(&ds3231, &time);
//    
//    dateTimeStringappend(time.tm_hour).append(":").append()
//}

AudioController* MainController::getAudioController() {
    return audioController;
}

ReelController* MainController::getReelController() {
    return reelController;
}

CCTalkController* MainController::getCCTalkController() {
    return cctalkController;
}

DisplayController* MainController::getDisplayController() {
    return displayController;
}

MoneyController* MainController::getMoneyController() {
    return moneyController;
}

uint8_t MainController::getVolume() {
    return volume;
}

Game* MainController::getGame() {
    return game;
}

/*
 * if low indicator not set, then hopper level low
 * if high indicator set, then hopper full and we must modify the sorter path, so that the 20ct coins are diverted to cash box
 */
void MainController::checkHopperLevel() {

    CctalkResponse response;
    cctalkController->requestPayoutHighLowStatus(CCTALK_HOPPER, response);


    if (response.isValidResponse() && (response.getAdditionalData().at(0) >> 1) & 1U) { // high mark
        cctalkController->modifySorterPath(CCTALK_COIN_VALIDATOR, 3, 1, response); // 20ct send to cash box
    } else {
        cctalkController->modifySorterPath(CCTALK_COIN_VALIDATOR, 3, 2, response); // 20ct (Hopper, adapter slot C, cctalk sort chute 2)
    }

    //    if (response.isValidResponse() && (response.getAdditionalData().at(0) >> 0) & 1U) {
    //        // hopper low / maybe empty
    //        //error(HOPPER_LOW);
    //    }
}

void MainController::processHopperErrors() {

}

void MainController::error(int errorCode) {
    //    displayController->clearText();
    //    //displayController->setText(errors[errorCode].errorMsg);
    //
    //    if (errors[errorCode].attendantRequired) {
    //        // loop with blinking lights
    //        while (true) {
    //
    //        }
    //    }
}

void MainController::writeValueToNVS(const char * key, uint16_t value) {

    esp_err_t err;

    // Write
    ESP_LOGI(TAG, "Updating %s in NVS ... ", key);

    err = nvs_handle->set_item<uint16_t>(key, value);
    switch (err) {
        case ESP_OK:
            ESP_LOGI(TAG, "Done");
            break;
        default:
            ESP_LOGE(TAG, "Failed!");
    }

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    ESP_LOGD(TAG, "Committing updates in NVS ... ");
    err = nvs_handle->commit();

    switch (err) {
        case ESP_OK:
            ESP_LOGI(TAG, "Commit Done");
            break;
        default:
            ESP_LOGE(TAG, "Coimmit Failed!");
    }




}

uint16_t MainController::readValueFromNVS(const char * key) {

    esp_err_t err;

    // Read
    ESP_LOGI(TAG, "Reading %s from NVS ... ", key);

    uint16_t value = 0; // value will default to 0, if not set yet in NVS
    err = nvs_handle->get_item<uint16_t>(key, value);
    switch (err) {
        case ESP_OK:
            ESP_LOGI(TAG, "Done");
            ESP_LOGI(TAG, "%s = %d", key, value);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGE(TAG, "The value for %s is not initialized yet! Initialising now to 0", key);
            writeValueToNVS(key, 0);
            break;
        default:
            ESP_LOGE(TAG, "Error reading %s!", esp_err_to_name(err));
    }


    return value;
}

