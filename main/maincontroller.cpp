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
#define __STDC_FORMAT_MACROS

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <string>
#include <inttypes.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <nvs.h>


#include "config.h"
#include "reelcontroller.h"
#include "displaycontroller.h"
#include "cctalk.h"
#include "game.h"
#include "audiocontroller.h"
#include "moneycontroller.h"
#include "maincontroller.h"
#include "webserver.h"
//#include "errors.h"

static const char *TAG = "MainController";
    
MainController::MainController() {

    //CCTalk_Device *cctalkDevice = new CCTalk_Device(this, &Serial1, (uint8_t) 1);
    this->cctalkController = new CCTalkController(this, cctalkDevice);
    this->audioController = new AudioController(this);
    this->displayController = new DisplayController(this);
    this->reelController = new ReelController(this);
    this->game = new Game(this);


}

MainController::MainController(const MainController& orig) {
}

void MainController::start() {

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2cdev_init());

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // intialise audio subsystem
    audioController->initialise();
    moneyController->initialise();

    init_spiffs();
    init_webserver("/spiffs");

    if (cctalk_init_desc(CCTALK_GPIO_TX, CCTALK_GPIO_RX) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialise ccTalk subsystem");
    } else {
        xTaskCreate(&check_coin_validator, "check_coin_validator", 2048, NULL, 5, NULL);
    }

    if (init_reels() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialise reel controller subsystem");
    } else {
    }




    if (displayController->initialise() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialise tableau subsystem");
    } else {
        xTaskCreate(&updateSevenSegDisplaysTask, "update_7seg_displays", 2048, &mainController, 5, NULL);
        xTaskCreate(&rainbowChaseTask, "rainbow_chase_task", 2048, NULL, 5, NULL);
    }

    validatorEventCounter = 0;
    hopperEventCounter = 0;
    letitgoCountdown = 0;

    // set the value to something sensible if the value hasn't yet been set in EEPROM
    // if (this->eeprom_data.volume < 0 || this->eeprom_data.volume > 20) {
    //     this->eeprom_data.volume = 10;
    // }
    // this->audioController->setVolume(eeprom_data.volume, eeprom_data.volume);    

    audioController->playAudioFile(SND_LET_IT_GO);

    startPolling = true;
}

void MainController::refreshStatus() {

    uint8_t data[256];
    uint8_t length;

    this->currentMillis = millis(); // grab current time

    if (this->startPolling) {
        this->cctalkController->poll();
    }

    if (!(game->isGameInProgress()) && (this->moneyController->getCredit() >= 20)) {
        ESP_LOGD(TAG, "Starting game...");
        this->game->start();
    }


    // check reel status and play "reel stop" sound effect
    uint8_t reelStatus = reelController->getStatus();
    if ((oldReelStatus != reelStatus) && (reelStatus & REEL_STATUS_OK_LEFT || reelStatus & REEL_STATUS_OK_CENTRE || reelStatus & REEL_STATUS_OK_RIGHT)) {
        oldReelStatus = reelStatus;
        this->audioController->playAudioFile(AUDIO_REEL_STOP);
    }

    if ((unsigned long) (currentMillis - serialReadPreviousMillis) >= serialReadInterval) {
        serialReadPreviousMillis = millis();
    }

    if (((unsigned long) (currentMillis - coinCheckPreviousMillis) >= coinRefreshInterval) && !payoutInProgress) {
        // now switch to polling for credit

        bool result = cctalkController->pollCredit(CCTalk_Device::COIN_VALIDATOR, data, length);
        // make sure we have the current cipher key from the hopper
        if (result && (data[0] > validatorEventCounter)) {

            uint8_t tmpCounter = data[0];
            uint8_t numEvents = tmpCounter - validatorEventCounter;

            for (int i = 1; i <= numEvents; i += 2) {
                this->addToCredit(cctalkController->COIN_VALUES[data[i]]);
                switch (data[i]) {
                    case 2:
                        this->eeprom_data.in10ct++;
                        break;
                    case 3:
                        this->eeprom_data.in20ct++;
                        break;
                    case 4:
                        this->eeprom_data.in50ct++;
                        break;
                    case 5:
                        this->eeprom_data.in1eur++;
                        break;
                    case 6:
                        this->eeprom_data.in2eur++;
                        break;
                }

            }
            validatorEventCounter = tmpCounter;

        }
        coinCheckPreviousMillis = millis();
    }

    if ((unsigned long) (currentMillis - hopperStatusPollPreviousMillis) >= hopperStatusPollInterval) {

        if (!payoutInProgress) {
            if (cctalkController->testHopper(CCTalk_Device::HOPPER, data, length)) {

            }

            checkHopperLevel();

            hopperStatusPollPreviousMillis = millis();

        } else if (payoutInProgress) {

            bool result = cctalkController->pollHopperStatus(CCTalk_Device::HOPPER, data, length);

            if (result) {
                uint8_t tmpEventCounter = data[0];
                uint8_t coinsRemaining = data[1];
                uint8_t coinsPaid = data[2];
                //uint8_t coinsUnpaid = data[3];

                if (tmpEventCounter > hopperEventCounter) {
                    // all coins have been paid, or some could not be paid
                    if (coinsRemaining == 0) {
                        hopperEventCounter = tmpEventCounter;
                        payoutInProgress = false;
                        bank -= coinsPaid * 20;
                        this->eeprom_data.coinsOut += coinsPaid;
                    } else {
                        processHopperErrors();
                    }
                } else {
                    payoutInProgress = false;
                    // TODO: We got a negative response polling hopper status. This means we should find out what went wrong
                    cctalkController->testHopper(CCTalk_Device::HOPPER, data, length);

                }
            }
        }

        hopperStatusPollPreviousMillis = millis();
    }

    if ((!game->isGameInProgress()) && ((unsigned long) (currentMillis - msgUpdatePreviousMillis) >= msgUpdateRefreshInterval)) {
        switch (state) {
            case 0:
                getDisplayController()->setText("       FROZEN       ");
                break;
            case 1:
                getDisplayController()->setText("      PLAY ME       ");
                break;
            case 2:
                getDisplayController()->setText("     20CT GAME      ");
                break;
            case 3:
                getDisplayController()->setText("    INSERT COINS    ");
                break;
        }

        // reset state
        if (state >= 3) {
            state = 0;
        } else {
            state++;
        }


        msgUpdatePreviousMillis = millis();
    }

    if ((!game->isGameInProgress()) && ((unsigned long) (currentMillis - tableauPreviousMillis) >= tableauRefreshInterval)) {


        displayController->clearAllData(false);
        for (int i = 0; i < MAX_LEDS; i++) {
            displayController->setLampRGBData(i, pgm_read_dword(&DisplayController::ANIMATION_IMAGES[animationStage][i]), false);
        }
        displayController->updateData();


        if (++animationStage >= DisplayController::ANIMATION_IMAGES_LENGTH) {
            animationStage = 0;
        }

        if (letitgoCountdown++ > 20) {
            letitgoCountdown = 0;
            audioController->startPlayingFile("letitgo.ogg");
        }

        tableauPreviousMillis = millis();
    }

}

void MainController::payout() {
    uint8_t data[255];
    uint8_t length;

    uint8_t numCoins = this->bank / 20;

    // Prepare for payout
    cctalkController->enableHopper(CCTalk_Device::HOPPER, data, length);

    if (cctalkController->dispenseCoins(CCTalk_Device::HOPPER, numCoins, data, length)) {
        this->payoutInProgress = true;
    }

}

void MainController::incrementGameCounter() {
    this->eeprom_data.numGamesPlayed++;
}

void MainController::dumpEEPROMValues() {
    this->getSerialMonitorController()->clearScreenAndDrawBorder();
    term->println("EEPROM dump");
    char buf[255];
    snprintf(buf, sizeof (buf), "Credit: %05u", this->eeprom_data.credit);
    term->println(buf);
    snprintf(buf, sizeof (buf), "Bank: %05u", this->eeprom_data.bank);
    term->println(buf);
    snprintf(buf, sizeof (buf), "Games Played (total): %05lu", this->eeprom_data.numGamesPlayed);
    term->println(buf);
    snprintf(buf, sizeof (buf), "10ct in: %05lu", this->eeprom_data.in10ct);
    term->println(buf);
    snprintf(buf, sizeof (buf), "20ct in: %05lu", this->eeprom_data.in20ct);
    term->println(buf);
    snprintf(buf, sizeof (buf), "50ct in: %05lu", this->eeprom_data.in50ct);
    term->println(buf);
    snprintf(buf, sizeof (buf), "1Eur in: %05lu", this->eeprom_data.in1eur);
    term->println(buf);
    snprintf(buf, sizeof (buf), "2Eur in: %05lu", this->eeprom_data.in2eur);
    term->println(buf);
    snprintf(buf, sizeof (buf), "Coins out: %05lu", this->eeprom_data.coinsOut);
    term->println(buf);
    snprintf(buf, sizeof (buf), "Volume: %02u", this->eeprom_data.volume);
    term->println(buf);

}

void MainController::setDateTime() {
    serialMonitorController->clearScreenAndDrawBorder();
    term->position(4, 2);
    term->println(F("Please enter the current date/time (yyyymmddHHMMSS"));
    String dateString;

    for (int i = 0; i < 14; i++) {
        char c = term->read();
        while (c == -1) {
            c = term->read();
        }
        term->write(c);
        dateString.concat(c);
    }

    int year = dateString.substring(0, 4).toInt();
    int month = dateString.substring(4, 6).toInt();
    int day = dateString.substring(6, 8).toInt();
    int hours = dateString.substring(8, 10).toInt();
    int minutes = dateString.substring(10, 12).toInt();
    int seconds = dateString.substring(12, 14).toInt();

    setTime(hours, minutes, seconds, day, month, year);
    time_t t = now();
    RTC.set(t);
}

void MainController::printDate(Stream *stream) {
    // digital clock display of the time

    char buf[20];
    snprintf(buf, sizeof (buf), "%02d-%02d-%04d %02d:%02d:%02d", day(), month(), year(), hour(), minute(), second());
    stream->println(buf);
}

time_t MainController::getDateTime() {
    return RTC.get();
}

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
    uint8_t data[2];
    uint8_t length;

    bool result = cctalkController->requestPayoutHighLowStatus(CCTalk_Device::HOPPER, data, length);

    if (result) {
        if ((data[0] >> 1) & 1U) { // high mark
            cctalkController->modifySorterPath(CCTalk_Device::COIN_VALIDATOR, 3, 1, data, length); // 20ct send to cash box
        } else {
            cctalkController->modifySorterPath(CCTalk_Device::COIN_VALIDATOR, 3, 2, data, length); // 20ct (Hopper, adapter slot C, cctalk sort chute 2)
        }
        if ((data[0] >> 0) & 1U) {
            // hopper low / maybe empty
            error(HOPPER_LOW);
        }
    }


}

void MainController::processHopperErrors() {

}

void MainController::error(int errorCode) {
    displayController->clearText();
    //displayController->setText(errors[errorCode].errorMsg);

    if (errors[errorCode].attendantRequired) {
        // loop with blinking lights
        while (true) {

        }
    }
}

void MainController::writeValueToNVS(const char * key, int value) {
    err = nvs_open("nvs", NVS_READWRITE, &nvs_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    } else {
        ESP_LOGI("Done");


        // Write
        ESP_LOGI(TAG, "Updating %s in NVS ... ", key);

        err = nvs_set_i32(nvs_handle, key, value);
        switch (err) {
            case ESP_OK:
                ESP_LOGI("Done");
                break;
            default:
                ESP_LOGE(TAG, "Failed!");
        }

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        ESP_LOGI(TAG, "Committing updates in NVS ... ");

        err = nvs_commit(my_handle);

        switch (err) {
            case ESP_OK:
                ESP_LOGI("Done");
                break;
            default:
                ESP_LOGE(TAG, "Failed!");
        }

    }
    // Close
    nvs_close(nvs_handle);

}

int MainController::readValueFromNVS(const char * key) {
    err = nvs_open("nvs", NVS_READONLY, &nvs_handle);

    int value = 0;
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    } else {
        ESP_LOGI("Done");
 // Read
        ESP_LOGI(TAG, "Reading %s from NVS ... ", key);
        int32_t value = 0; // value will default to 0, if not set yet in NVS
        err = nvs_get_i32(nvs_handle, key, &value);
        switch (err) {
            case ESP_OK:
                ESP_LOGI(TAG, "Done");
                ESP_LOGI(TAG, "%s = %d", key, restart_counter);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGE(TAG, "The value is not initialized yet!");
                break;
            default :
                ESP_LOGE(TAG,"Error reading %s!", esp_err_to_name(err));
        }
    }
    
    // Close
    nvs_close(nvs_handle);
    
    return value;
}

void blinkTask(void *pvParameter) {
    while (1) {
        /* Blink off (output low) */
        gpio_set_level(CPU_LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        /* Blink on (output high) */
        gpio_set_level(CPU_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}