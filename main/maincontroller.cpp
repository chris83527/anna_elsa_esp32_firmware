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
#include <stdlib.h>
#include <string>

#include <ds3231.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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
#include "oledcontroller.h"

// #include "errors.h"

static const char *TAG = "MainController";
static int blinkDelay = 250;

MainController::MainController()
    : i2c_manager(I2CManager(I2C_NUM_0, GPIO_NUM_22, GPIO_NUM_21)),
      nvsController(NvsController()),
      moneyController(nvsController, cctalkController),
      displayController(DisplayController(moneyController, i2c_manager)),
      audioController(i2c_manager),
      reelController(audioController, displayController, i2c_manager),
      game(Game(displayController, audioController, moneyController,
                reelController)),
      ds3231(DS3231(i2c_manager, DS3231_ADDR)) {
  ESP_LOGD(TAG, "Entering constructor");

  ESP_LOGD(TAG, "Leaving constructor");
}

void MainController::start() {
  ESP_LOGD(TAG, "start() called");

  // CPU LED is on a GPIO
  esp_rom_gpio_pad_select_gpio(CPU_LED_GPIO);
  /* Set the GPIO as a push/pull output */
  gpio_set_direction(CPU_LED_GPIO, GPIO_MODE_OUTPUT);
  /* Switch off to start */
  gpio_set_level(CPU_LED_GPIO, 0);

  esp_event_loop_create_default();

  if (m20ly02z_init(MD_STROBE, MD_OE, MD_CLK, MD_DATA) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialise VFD display");
  } else {
    this->displayController.displayVFDText("INITIALISING 01");
  }

  auto cfg = esp_pthread_get_default_config();
  cfg.thread_name = "BlinkRunLED";
  cfg.prio = 1;
  cfg.stack_size = 2048;
  cfg.inherit_cfg = false;
  esp_pthread_set_cfg(&cfg);
  this->blinkCPUStatusLEDThread =
      std::thread([&]() { blinkCPUStatusLEDTask(); });
  this->blinkCPUStatusLEDThread.detach();

  this->displayController.displayVFDText("INITIALISING 02");
  // Initialize NVS
  ESP_LOGD(TAG, "Setting up NVS");
  this->displayController.scrollOledText("Init NVS");
  esp_err_t err = this->nvsController.initialise();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    this->displayController.scrollOledText("  -> failed");
  } else {
    ESP_LOGD(TAG, "NVS opened ok.");
    this->displayController.scrollOledText("  -> ok");
  }

  // Initialise WiFi
  this->displayController.displayVFDText("INITIALISING 03");
  this->displayController.scrollOledText("Init WiFi");

  // initialise ds3231 RTC
  this->displayController.displayVFDText("INITIALISING 04");
  this->displayController.scrollOledText("Init RTC");

  setDateTime(); // Debug

  //    if (err != ESP_OK) {
  //        ESP_LOGE(TAG, "Error initialising RTC!");
  //        oledController->scrollOledText("  -> failed");
  //    } else {
  //        //this->setDateTime(); // Debug only
  ESP_LOGI(TAG, "RTC initialised ok");
  this->displayController.scrollOledText("  -> ok");
  //}

  this->displayController.displayVFDText("INITIALISING 05");
  this->displayController.scrollOledText("Init LittleFS");
  esp_vfs_littlefs_conf_t conf = {
      .base_path = "/httpd",
      .partition_label = "httpd",
      .partition = NULL,
      .format_if_mount_failed = false,
      .read_only = false,
      .dont_mount = false,
      .grow_on_mount = false,
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
  }

  this->displayController.displayVFDText("INITIALISING 06");
  this->displayController.scrollOledText("Init Webserver");
  // this->httpController->initialise(80, "/httpd", "INNUENDO",
  // "woodsamusements");

  /* Mark current app as valid */
  const esp_partition_t *partition = esp_ota_get_running_partition();
  printf("Currently running partition: %s\r\n", partition->label);

  esp_ota_img_states_t ota_state;
  if (esp_ota_get_state_partition(partition, &ota_state) == ESP_OK) {
    if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
      esp_ota_mark_app_valid_cancel_rollback();
    }
  }

  // initialise audio subsystem
  this->displayController.displayVFDText("INITIALISING 07");
  this->displayController.scrollOledText("Init Audio");
  this->audioController.initialise();

  this->displayController.displayVFDText("INITIALISING 08");
  this->displayController.scrollOledText("Init Display");
  if (this->displayController.initialise() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialise tableau subsystem");
    this->displayController.scrollOledText("  -> failed");
  } else {
    ESP_LOGD(TAG, "Display controller initialisation ok.");
    this->displayController.scrollOledText("  -> ok");
  }

  this->displayController.displayVFDText("INITIALISING 09");
  this->displayController.scrollOledText("Load stats");
  moneyController.initialise();

  this->displayController.displayVFDText("INITIALISING 0A");
  this->displayController.scrollOledText("Init cctalk");
  cctalkController.setCreditAcceptedCallback(
      [&](uint8_t coin_id, const esp32cc::CcIdentifier &identifier) {
        ESP_LOGI(TAG, "Credit accepted: Coin id: %d, Identifier: %s", coin_id,
                 identifier.id_string.c_str());
        moneyController.addToCredit(CCTalkController::COIN_VALUES[coin_id]);
      });

  this->displayController.displayVFDText("INITIALISING 0B");
  if (cctalkController.initialise() != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialise ccTalk subsystem");
    this->displayController.scrollOledText("  -> failed");
  } else {
    this->displayController.scrollOledText("  -> ok");
  }

  this->displayController.displayVFDText("INITIALISING 0C");
  this->displayController.scrollOledText("Init reels");
  if (!reelController.initialise()) {
    this->displayController.scrollOledText("  -> failed");
    ESP_LOGE(TAG, "Failed to initialise reel controller subsystem");
  } else {
    this->displayController.scrollOledText("  -> ok");
    ESP_LOGD(TAG, "Reel controller initialisation ok.");
  }

  blinkDelay = 1000;

  this->displayController.displayVFDText("INITIALISING 0D");
  this->displayController.scrollOledText("Init game");
  this->game.initialise();

  this->displayController.displayVFDText("INITIALISING 0E");
  cfg = esp_pthread_get_default_config();
  cfg.thread_name = "UpdateStatistics";
  cfg.prio = 1;
  cfg.stack_size = 4096;
  esp_pthread_set_cfg(&cfg);
  this->updateStatisticsThread =
      std::thread([this]() { updateStatisticsDisplayTask(); });
  this->updateStatisticsThread.detach();

  this->displayController.displayVFDText("                    ");

  for (;;) {
    if ((!this->game.isGameInProgress()) &&
        (this->moneyController.getCredit() >= 20)) {
      ESP_LOGD(TAG, "Starting game...");
      this->game.start();
    } else {
      if (!getDisplayController().isAttractMode()) {
        getDisplayController().beginAttractMode();
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

  ds3231.set_time(&time);
}

AudioController &MainController::getAudioController() {
  return audioController;
}

ReelController &MainController::getReelController() { return reelController; }

CCTalkController &MainController::getCCTalkController() {
  return cctalkController;
}

DisplayController &MainController::getDisplayController() {
  return displayController;
}

MoneyController &MainController::getMoneyController() {
  return moneyController;
}

Game &MainController::getGame() { return game; }

DS3231 &MainController::getDs3231() { return this->ds3231; }

void MainController::error(int errorCode) {
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

  this->displayController.clearOledDisplay();

  char buf[21];
  esp_err_t ret;

  while (1) {
    ESP_LOGD(TAG, "Updating statics loop");
    ret = ds3231.get_time(time);

    if (ret == ESP_OK) {

      uint16_t years = time.tm_year + 1900;
      std::sprintf(buf, "%02d-%02d-%04d %02d:%02d", time.tm_mday, time.tm_mon,
                   years, time.tm_hour, time.tm_min);

      dateString.clear();
      dateString.append(buf);
      this->displayController.displayOledText(dateString, 0, true);

      std::sprintf(buf, "Games    : %05d",
                   this->moneyController.getGameCount());
      this->displayController.displayOledText(buf, 2, false);
      std::sprintf(buf, "Total in : %05d",
                   this->moneyController.getIncomeTotal());
      this->displayController.displayOledText(buf, 3, false);
      std::sprintf(buf, "Total out: %05d",
                   this->moneyController.getPayoutTotal());
      this->displayController.displayOledText(buf, 4, false);
      std::sprintf(buf, "Credit   : %05d", this->moneyController.getCredit());
      this->displayController.displayOledText(buf, 5, false);
      std::sprintf(buf, "Bank     : %05d", this->moneyController.getBank());
      this->displayController.displayOledText(buf, 6, false);
    } else {
      ESP_LOGW(TAG, "Couldn't read time from RTC!");
    }

    std::this_thread::sleep_for(std::chrono::seconds(5)); // 5 seconds
  }
}
