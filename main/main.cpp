/*
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file main.cpp
 *
 * ESP-IDF driver for Holtek HT16K33 I2C LED Matrix driver chip
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "esp_err.h"
#include "esp_log.h"
#include "esp_pthread.h"

#include "maincontroller.h"

#include "config.h"

using namespace std;

// #define CONFIG_FIRMWARE_SERVICE

static const char *TAG = "main";

extern "C" void app_main() {
  ESP_LOGI(TAG, "app_main() called");

  // See https://github.com/espressif/esp-idf/issues/3544
  esp_pthread_cfg_t cfg;
  if (esp_pthread_get_cfg(&cfg) != ESP_OK) {
    cfg = esp_pthread_get_default_config();
  }

  //    cfg.prio = 1;
  // cfg.inherit_cfg = true;
  cfg.thread_name = "testStack";
//  cfg.pin_to_core = 1;
  cfg.stack_size = (9216);
  if (esp_pthread_set_cfg(&cfg) != ESP_OK) {
    printf("esp_pthread_set_cfg failed\n");
    abort();
  };

  esp_log_level_set("gpio", ESP_LOG_WARN);
  esp_log_level_set("ReelController", ESP_LOG_INFO);
  esp_log_level_set("m20ly02z", ESP_LOG_WARN);
  esp_log_level_set("i2cmanager", ESP_LOG_WARN);
  esp_log_level_set("i2c.master", ESP_LOG_WARN);
  esp_log_level_set("ssd1306", ESP_LOG_INFO);
  esp_log_level_set("cctalkDevice", ESP_LOG_INFO);
  esp_log_level_set("cctalk_link_controller", ESP_LOG_INFO);
  esp_log_level_set("serial_worker", ESP_LOG_INFO);
  esp_log_level_set("AudioController", ESP_LOG_INFO);
  esp_log_level_set("DisplayController", ESP_LOG_INFO);
  esp_log_level_set("MainController", ESP_LOG_INFO);
  esp_log_level_set("CCTALK_CONTROLLER", ESP_LOG_INFO);

  MainController mainController;
  mainController.start();
}
