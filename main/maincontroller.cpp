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
#include <chrono>
#include <ctime>
#include <functional>
#include <string>

#include "ds3231.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

#include "audiocontroller.h"
#include "cctalkcontroller.h"
#include "config.h"
#include "displaycontroller.h"
#include "esp_event.h"
#include "game.h"
#include "maincontroller.h"
#include "moneycontroller.h"
#include "reelcontroller.h"

#include "esp_littlefs.h"
#include "esp_pthread.h"
#include "m20ly02z.h"

// #include "errors.h"

static const char* TAG = "MainController";
static int blinkDelay = 250000; // µs, not ms

MainController::MainController()
{
    ESP_LOGD(TAG, "Entering constructor");

    ESP_LOGD(TAG, "Leaving constructor");
}

MainController::~MainController() = default;

void MainController::start()
{
    ESP_LOGD(TAG, "start() called");

    // CPU LED is on a GPIO
    esp_rom_gpio_pad_select_gpio(CPU_LED_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(CPU_LED_GPIO, GPIO_MODE_OUTPUT);
    /* Switch off to start */
    gpio_set_level(CPU_LED_GPIO, 0);

    esp_event_loop_create_default();

    if (m20ly02z_init(MD_STROBE, MD_OE, MD_CLK, MD_DATA) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialise VFD display");
    }
    else
    {
        displayController.displayVFDText("INITIALISING 01");
    }

    // Set up a timer to blink the CPU LED
    constexpr esp_timer_create_args_t my_timer_args = {
        .callback = &blinkCPUStatusLEDCallback,
        .arg = nullptr,
		.dispatch_method = ESP_TIMER_TASK,
        .name = "CPU LED Timer",
        .skip_unhandled_events = true
    };
    esp_timer_handle_t timer_handler;
    ESP_ERROR_CHECK(esp_timer_create(&my_timer_args, &timer_handler));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, blinkDelay));


    displayController.displayVFDText("INITIALISING 02");
    // Initialize NVS
    ESP_LOGD(TAG, "Setting up NVS");
    displayController.scrollOledText("Init NVS");
    esp_err_t err = nvsController.initialise();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        displayController.scrollOledText("  -> failed");
    }
    else
    {
        ESP_LOGD(TAG, "NVS opened ok.");
        displayController.scrollOledText("  -> ok");
    }


    // Initialise WiFi
    displayController.displayVFDText("INITIALISING 03");
    displayController.scrollOledText("Init WiFi");

    // initialise ds3231 RTC
    displayController.displayVFDText("INITIALISING 04");
    displayController.scrollOledText("Init RTC");

    setDateTime(); // Debug

    //    if (err != ESP_OK) {
    //        ESP_LOGE(TAG, "Error initialising RTC!");
    //        oledController->scrollOledText("  -> failed");
    //    } else {
    //        //this->setDateTime(); // Debug only
    ESP_LOGI(TAG, "RTC initialised ok");
    displayController.scrollOledText("  -> ok");
    //}

    displayController.displayVFDText("INITIALISING 05");
    displayController.scrollOledText("Init LittleFS");
    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/httpd",
        .partition_label = "httpd",
        .partition = nullptr,
        .format_if_mount_failed = false,
        .read_only = false,
        .dont_mount = false,
        .grow_on_mount = false,
    };

    // Use settings defined above to initialize and mount LittleFS filesystem.
    // Note: esp_vfs_littlefs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_littlefs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find LittleFS partition");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
    }

    displayController.displayVFDText("INITIALISING 06");
    displayController.scrollOledText("Init Webserver");
    // this->httpController->initialise(80, "/httpd", "INNUENDO",
    // "woodsamusements");

    /* Mark current app as valid */
    const esp_partition_t* partition = esp_ota_get_running_partition();
    printf("Currently running partition: %s\r\n", partition->label);

    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(partition, &ota_state) == ESP_OK)
    {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
        {
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }

    // initialise audio subsystem
    displayController.displayVFDText("INITIALISING 07");
    displayController.scrollOledText("Init Audio");
    this->audioController.initialise();

    displayController.displayVFDText("INITIALISING 08");
    displayController.scrollOledText("Init Display");
    if (displayController.initialise() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialise tableau subsystem");
        displayController.scrollOledText("  -> failed");
    }
    else
    {
        ESP_LOGD(TAG, "Display controller initialisation ok.");
        displayController.scrollOledText("  -> ok");
    }

    displayController.displayVFDText("INITIALISING 09");
    displayController.scrollOledText("Load stats");
    moneyController.initialise();

    displayController.displayVFDText("INITIALISING 0A");
    displayController.scrollOledText("Init cctalk");
    cctalkController.setCreditAcceptedCallback(
        [&](uint8_t coin_id, const esp32cc::CcIdentifier& identifier)
        {
            ESP_LOGI(TAG, "Credit accepted: Coin id: %d, Identifier: %s", coin_id,
                     identifier.id_string.c_str());
            moneyController.addToCredit(CCTalkController::COIN_VALUES[coin_id]);
        });

    displayController.displayVFDText("INITIALISING 0B");
    if (cctalkController.initialise() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialise ccTalk subsystem");
        displayController.scrollOledText("  -> failed");
    }
    else
    {
        displayController.scrollOledText("  -> ok");
    }

    displayController.displayVFDText("INITIALISING 0C");
    displayController.scrollOledText("Init reels");
    if (!reelController.initialise())
    {
        displayController.scrollOledText("  -> failed");
        ESP_LOGE(TAG, "Failed to initialise reel controller subsystem");
    }
    else
    {
        displayController.scrollOledText("  -> ok");
        ESP_LOGD(TAG, "Reel controller initialisation ok.");
    }

    blinkDelay = 1000000;
	ESP_ERROR_CHECK(esp_timer_stop(timer_handler));
	ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, 5000000));

    displayController.displayVFDText("INITIALISING 0D");
    displayController.scrollOledText("Init game");
    this->game.initialise();

    displayController.displayVFDText("INITIALISING 0E");

    // Set up a timer to update the statistics every 5 seconds
    constexpr esp_timer_create_args_t updateStatisticsTimerArgs = {
        .callback = &updateStatisticsDisplayCallback,
		.arg = nullptr,
		.dispatch_method = ESP_TIMER_TASK,
        .name = "Update Statistics Timer",
		.skip_unhandled_events = true
	};
    esp_timer_handle_t updateStatisticsTimerHandler;
    ESP_ERROR_CHECK(esp_timer_create(&updateStatisticsTimerArgs, &updateStatisticsTimerHandler));
    ESP_ERROR_CHECK(esp_timer_start_periodic(updateStatisticsTimerHandler, 5000000));

    displayController.displayVFDText("                    ");

    for (;;)
    {
        if ((!this->game.isGameInProgress()) &&
            (this->moneyController.getCredit() >= 20))
        {
            ESP_LOGD(TAG, "Starting game...");
            this->game.start();
        }
        else
        {
            if (!getDisplayController().isAttractMode())
            {
                getDisplayController().beginAttractMode();
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void MainController::setDateTime()
{
    tm time{};
    time.tm_hour = 12;
    time.tm_min = 31;
    time.tm_sec = 0;
    time.tm_isdst = true;
    time.tm_mon = 01;
    time.tm_year = (2023 - 1900); // tm_year = number of years since 1900
    time.tm_mday = 24;

    ds3231.set_time(&time);
}

AudioController& MainController::getAudioController()
{
    return audioController;
}

ReelController& MainController::getReelController() { return reelController; }

CCTalkController& MainController::getCCTalkController()
{
    return cctalkController;
}

DisplayController& MainController::getDisplayController()
{
    return displayController;
}

MoneyController& MainController::getMoneyController()
{
    return moneyController;
}

Game& MainController::getGame() { return game; }

DS3231& MainController::getDs3231() { return this->ds3231; }

void MainController::error(int errorCode)
{
    //    displayController->clearText();
    //    //displayController->scrollOledText(errors[errorCode].errorMsg);
    //
    //    if (errors[errorCode].attendantRequired) {
    //        // loop with blinking lights
    //        while (true) {
    //
    //        }
    //    }
}

void MainController::blinkCPUStatusLEDCallback(void *param)
{
    static bool on;
    on = !on;
    gpio_set_level(CPU_LED_GPIO, on);
}

void MainController::updateStatisticsDisplayCallback(void *param)
{
    MainController* mainController = static_cast<MainController*>(param);

    tm time{};
    std::string dateString;

    mainController->displayController.clearOledDisplay();

    char buf[21];
    esp_err_t ret;

    ESP_LOGD(TAG, "Updating statics loop");
    ret = mainController->ds3231.get_time(time);

    if (ret == ESP_OK)
    {
        uint16_t years = time.tm_year + 1900;
        std::sprintf(buf, "%02d-%02d-%04d %02d:%02d", time.tm_mday, time.tm_mon,
                     years, time.tm_hour, time.tm_min);

        dateString.clear();
        dateString.append(buf);
        mainController->displayController.displayOledText(dateString, 0, true);

        std::sprintf(buf, "Games    : %05d",
                     mainController->moneyController.getGameCount());
        mainController->displayController.displayOledText(buf, 2, false);
        std::sprintf(buf, "Total in : %05d",
                     mainController->moneyController.getIncomeTotal());
        mainController->displayController.displayOledText(buf, 3, false);
        std::sprintf(buf, "Total out: %05d",
                     mainController->moneyController.getPayoutTotal());
        mainController->displayController.displayOledText(buf, 4, false);
        std::sprintf(buf, "Credit   : %05d", mainController->moneyController.getCredit());
        mainController->displayController.displayOledText(buf, 5, false);
        std::sprintf(buf, "Bank     : %05d", mainController->moneyController.getBank());
        mainController->displayController.displayOledText(buf, 6, false);
    }
    else
    {
        ESP_LOGW(TAG, "Couldn't read time from RTC!");
    }

}
