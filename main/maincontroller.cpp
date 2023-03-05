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
#include <functional>
#include <chrono>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_system.h>
#include <ds3231.h>
#include <esp_ota_ops.h>


#include "config.h"
#include "spiffs.h"
#include "reelcontroller.h"
#include "displaycontroller.h"
#include "cctalkcontroller.h"
#include "game.h"
#include "audiocontroller.h"
#include "moneycontroller.h"
#include "maincontroller.h"
//#include "webserver.h"

#include "esp_littlefs.h"
#include "oledcontroller.h"
#include "wifi.h"
//#include "errors.h"

static const char *TAG = "MainController";
static int blinkDelay = 250;

MainController::MainController() {
    ESP_LOGD(TAG, "Entering constructor");
    ESP_LOGD(TAG, "Leaving constructor");
}

MainController::MainController(const MainController& orig) {
}

void MainController::start() {
    ESP_LOGD(TAG, "start() called");

    this->audioController.reset(new AudioController());
    this->displayController.reset(new DisplayController(this));
    this->reelController.reset(new ReelController(this));
    this->oledController.reset(new oledcontroller());
    this->moneyController.reset(new MoneyController(this));
    this->game.reset(new Game(this));
    this->cctalkController.reset(new CCTalkController());                          
    this->httpController.reset(new HttpController());      

    // CPU LED is on a GPIO
    gpio_pad_select_gpio(CPU_LED_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(CPU_LED_GPIO, GPIO_MODE_OUTPUT);
    /* Switch off to start */
    gpio_set_level(CPU_LED_GPIO, 0);

    esp_err_t err;

    esp_event_loop_create_default();

    if (m20ly02z_init(MD_STROBE, MD_OE, MD_CLK, MD_DATA) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialise VFD display");
    } else {
        this->displayController->displayText("INITIALISING");
    }
    
    this->blinkCPUStatusLEDThread.reset(new std::thread([this]() {
        blinkCPUStatusLEDTask();
    }));
    //xTaskCreate(&blinkCPUStatusLEDTask, "cpu_status_led_blink", 2048, this, 1, NULL);


    ESP_LOGD(TAG, "Calling i2cdev_init()");
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2cdev_init());
    //i2c_set_timeout(I2C_NUM_0, 400000);

    // start outputting to status oled
    oledController->initialise();

    // Initialize NVS
    ESP_LOGD(TAG, "Setting up NVS");
    oledController->scrollText("Init NVS");

    // Initialize NVS
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

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
        oledController->scrollText("  -> failed");
    } else {
        ESP_LOGD(TAG, "NVS opened ok.");
        oledController->scrollText("  -> ok");
    }

    // Initialise WiFi
    oledController->scrollText("Init WiFi");
    

    // initialise ds3231 RTC
    oledController->scrollText("Init RTC");
    memset(&ds3231, 0, sizeof (i2c_dev_t));

    err = ds3231_init_desc(&ds3231, 0, GPIO_I2C_SDA, GPIO_I2C_SCL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error initialising RTC!");
        oledController->scrollText("  -> failed");
    } else {
        //this->setDateTime(); // Debug only
        ESP_LOGI(TAG, "RTC initialised ok");
        oledController->scrollText("  -> ok");
    }

    oledController->scrollText("Init LittleFS");

    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/httpd",
        .partition_label = "httpd",
        .format_if_mount_failed = false,
        .dont_mount = false,
    };

    // Use settings defined above to initialize and mount LittleFS filesystem.
    // Note: esp_vfs_littlefs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_littlefs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find LittleFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
        return;
    }
    
    oledController->scrollText("Init Webserver");
    this->httpController->initialise(80, "/httpd", "INNUENDO", "woodsamusements");

    /* Mark current app as valid */
    const esp_partition_t *partition = esp_ota_get_running_partition();
    printf("Currently running partition: %s\r\n", partition->label);

    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(partition, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }


    // intialise audio subsystem    
    oledController->scrollText("Init Audio");
    audioController->initialise();

    esp_err_t res;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    printf("00:         ");
    for (uint8_t i = 3; i < 0x78; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
        i2c_master_stop(cmd);

        res = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10));
        if (i % 16 == 0)
            printf("\n%2x:", i);
        if (res == 0)
            printf(" %2x", i);
        else
            printf(" --");
        i2c_cmd_link_delete(cmd);
    }
    printf("\n\n");

    oledController->scrollText("Init Display");
    if (displayController->initialise() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialise tableau subsystem");
        oledController->scrollText("  -> failed");
    } else {
        ESP_LOGD(TAG, "Display controller initialisation ok.");
        oledController->scrollText("  -> ok");
    }

    //oledController->scrollText("Init NVRAM");
    moneyController->initialise();

    oledController->scrollText("Init reels");
    if (reelController->initialise() != ESP_OK) {
        oledController->scrollText("  -> failed");
        ESP_LOGE(TAG, "Failed to initialise reel controller subsystem");
    } else {
        oledController->scrollText("  -> ok");
        ESP_LOGD(TAG, "Reel controller initialisation ok.");
    }

    oledController->scrollText("Init cctalk");
    cctalkController->setCreditAcceptedCallback([&](uint8_t coin_id, const esp32cc::CcIdentifier & identifier) {
        ESP_LOGI(TAG, "Credit accepted: Coin id: %d, Identifier: %s", coin_id, identifier.id_string.c_str());
        moneyController->addToCredit(COIN_VALUES[coin_id]);
    });


    if (cctalkController->initialise() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialise ccTalk subsystem");
        oledController->scrollText("  -> failed");
    } else {
        oledController->scrollText("  -> ok");
    }

    blinkDelay = 1000;

    oledController->scrollText("Init game");
    game->initialise();
    this->displayController->beginAttractMode();

    updateStatisticsThread.reset(new std::thread([this]() {
        updateStatisticsDisplayTask();
    }));    

    
    for (;;) {
        if (!(game->isGameInProgress()) && (this->moneyController->getCredit() >= 20)) {
            ESP_LOGD(TAG, "Starting game...");
            this->game->start();
        } else {
            if (!getDisplayController()->isAttractMode()) {
                getDisplayController()->beginAttractMode();
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

}

void MainController::setDateTime() {
    tm time;
    time.tm_hour = 12;
    time.tm_min = 31;
    time.tm_sec = 0;
    time.tm_isdst = true;
    time.tm_mon = 01;
    time.tm_year = (2023 - 1900); // tm_year = number of years since 1900
    time.tm_mday = 24;

    ds3231_set_time(&ds3231, &time);    
}

std::shared_ptr<AudioController> MainController::getAudioController() {
    return audioController;
}

std::shared_ptr<ReelController> MainController::getReelController() {
    return reelController;
}

std::shared_ptr<CCTalkController> MainController::getCCTalkController() {
    return cctalkController;
}

std::shared_ptr<DisplayController> MainController::getDisplayController() {
    return displayController;
}

std::shared_ptr<MoneyController> MainController::getMoneyController() {
    return moneyController;
}

std::shared_ptr<oledcontroller> MainController::getOledController() {
    return oledController;
}

uint8_t MainController::getVolume() {
    return volume;
}

std::shared_ptr<Game> MainController::getGame() {
    return game;
}

i2c_dev_t* MainController::getDs3231() {
    return &this->ds3231;
}

void MainController::error(int errorCode) {
    //    displayController->clearText();
    //    //displayController->scrollText(errors[errorCode].errorMsg);
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
    ESP_LOGD(TAG, "Updating %s in NVS ... ", key);

    err = nvs_handle->set_item<uint16_t>(key, value);
    switch (err) {
        case ESP_OK:
            ESP_LOGD(TAG, "Done");
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
            ESP_LOGD(TAG, "Commit Done");
            break;
        default:
            ESP_LOGE(TAG, "Commit Failed!");
    }

}

uint16_t MainController::readValueFromNVS(const char * key) {

    esp_err_t err;

    // Read
    ESP_LOGD(TAG, "Reading %s from NVS ... ", key);

    uint16_t value = 0; // value will default to 0, if not set yet in NVS
    err = nvs_handle->get_item<uint16_t>(key, value);
    switch (err) {
        case ESP_OK:
            ESP_LOGD(TAG, "Done");
            ESP_LOGD(TAG, "%s = %d", key, value);
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

//std::shared_ptr<WIFI::Wifi> MainController::getWifiController() {
//    return wifi;
//}


void MainController::blinkCPUStatusLEDTask() {
    while (1) {
        /* Blink off (output low) */
        gpio_set_level(CPU_LED_GPIO, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(blinkDelay));
        /* Blink on (output high) */
        gpio_set_level(CPU_LED_GPIO, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(blinkDelay));
    }
}

void MainController::updateStatisticsDisplayTask() {

    tm time;
    std::string dateString;

    this->oledController->clearDisplay();

    char buf[21];
    esp_err_t ret;

    while (1) {

        ret = ds3231_get_time(&this->ds3231, &time);

        if (ret == ESP_OK) {

            uint16_t years = time.tm_year + 1900;
            std::sprintf(buf, "%02d-%02d-%04d %02d:%02d", time.tm_mday, time.tm_mon, years, time.tm_hour, time.tm_min);

            dateString.clear();
            dateString.append(buf);
            oledController->displayText(dateString, 0, true);

            std::sprintf(buf, "Games    : %05d", this->moneyController->getGameCount());
            oledController->displayText(buf, 2, false);
            std::sprintf(buf, "Total in : %05d", this->moneyController->getIncomeTotal());
            oledController->displayText(buf, 3, false);
            std::sprintf(buf, "Total out: %05d", this->moneyController->getPayoutTotal());
            oledController->displayText(buf, 4, false);
            std::sprintf(buf, "Credit   : %05d", this->moneyController->getCredit());
            oledController->displayText(buf, 5, false);
            std::sprintf(buf, "Bank     : %05d", this->moneyController->getBank());
            oledController->displayText(buf, 6, false);
        } else {
            ESP_LOGW(TAG, "Couldn't read time from RTC!");
        }

        std::this_thread::sleep_for(std::chrono::seconds(1)); // 1 second
    }
}
