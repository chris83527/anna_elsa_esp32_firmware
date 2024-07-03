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
 * @file audiocontroller.c
 *
 * Definitions for playing audio files
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <cstring>
#include <fstream>
#include <ios>
#include <iostream>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <streambuf>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "driver/i2s_common.h"
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "hal/i2s_types.h"

#include "audiocontroller.h"
#include "config.h"

static const char *TAG = "AudioController";

AudioController::AudioController(I2CManager &i2cmgr) : i2cManager{i2cmgr} {}

AudioController::~AudioController() {}

void AudioController::initialise() {

  ESP_LOGI(TAG, "Enter initialise()");

  ESP_LOGI(TAG, "Initialising board and codecs");

  init_spiffs();

  ESP_LOGD(TAG, "Power ON CODEC with GPIO %d", TAS5731M_PDWN_GPIO);

  esp_rom_gpio_pad_select_gpio(TAS5731M_RST_GPIO);
  esp_rom_gpio_pad_select_gpio(TAS5731M_PDWN_GPIO);

  gpio_set_direction(TAS5731M_RST_GPIO, GPIO_MODE_OUTPUT);
  gpio_set_direction(TAS5731M_PDWN_GPIO, GPIO_MODE_OUTPUT);

  uint32_t reg_val = REG_READ(PIN_CTRL);
  ESP_LOGD(TAG, "PIN_CTRL before: %lu", reg_val);
  REG_WRITE(PIN_CTRL, 0xFFFFFFF0);
  reg_val = REG_READ(PIN_CTRL);
  ESP_LOGD(TAG, "PIN_CTRL after: %lu", reg_val);
  PIN_FUNC_SELECT(GPIO_PIN_REG_0, 1); // GPIO0 as CLK_OUT1

  // See TI TAS5731M Datasheet page 63
  gpio_set_level(TAS5731M_RST_GPIO, 0); // Drive /RESET = 0
  gpio_set_level(TAS5731M_PDWN_GPIO, 0);
  vTaskDelay(pdMS_TO_TICKS(200));
  gpio_set_level(TAS5731M_PDWN_GPIO, 1);
  vTaskDelay(pdMS_TO_TICKS(200));
  gpio_set_level(TAS5731M_RST_GPIO, 1);
  vTaskDelay(pdMS_TO_TICKS(500));

  i2cInit();
  i2sInit();

  setVolume(40); // TODO: get this volume from NVRAM

  playAudioFile(Sounds::SND_STARTUP);
}

esp_err_t AudioController::i2cInit() {
  this->deviceConfig.device_address = TAS5731M_I2C_ADDRESS;
  this->deviceConfig.scl_speed_hz = 100000;
  this->deviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;

  i2cManager.addDevice(deviceConfig, deviceHandle);

  std::vector<uint8_t> data = std::vector<uint8_t>(4);

  data[0] = 0x00;
  // init sequence
  i2cManager.writeRegister(this->deviceHandle, 0x1b, data, 1);
  vTaskDelay(pdMS_TO_TICKS(50));
  data[0] = 0x03;
  i2cManager.writeRegister(this->deviceHandle, 0x04, data, 1);
  data[0] = 0x00;
  i2cManager.writeRegister(this->deviceHandle, 0x06, data, 1);
  data[0] = 0x30;
  i2cManager.writeRegister(this->deviceHandle, 0x0a, data, 1);
  i2cManager.writeRegister(this->deviceHandle, 0x09, data, 1);
  i2cManager.writeRegister(this->deviceHandle, 0x08, data, 1);
  data[0] = 0x54;
  i2cManager.writeRegister(this->deviceHandle, 0x14, data, 1);
  data[0] = 0xac;
  i2cManager.writeRegister(this->deviceHandle, 0x13, data, 1);
  data[0] = 0x54;
  i2cManager.writeRegister(this->deviceHandle, 0x12, data, 1);
  data[0] = 0xac;
  i2cManager.writeRegister(this->deviceHandle, 0x11, data, 1);
  data[0] = 0x91;
  i2cManager.writeRegister(this->deviceHandle, 0x0e, data);
  data[0] = 0x00;
  data[1] = 0x01;
  data[2] = 0x77;
  data[3] = 0x72;
  i2cManager.writeRegister(this->deviceHandle, 0x20, data, 4);
  data[0] = 0x02;
  i2cManager.writeRegister(this->deviceHandle, 0x10, data, 1);
  data[0] = 0x00;
  i2cManager.writeRegister(this->deviceHandle, 0x0b, data, 1);
  data[0] = 0x02;
  i2cManager.writeRegister(this->deviceHandle, 0x10, data, 1);
  i2cManager.writeRegister(this->deviceHandle, 0x1c, data, 1);
  data[0] = 0x30;
  i2cManager.writeRegister(this->deviceHandle, 0x19, data, 1);
  data[0] = 0x01;
  data[1] = 0x02;
  data[2] = 0x13;
  data[3] = 0x45;
  i2cManager.writeRegister(this->deviceHandle, 0x25, data, 4);
  data[0] = 0xff;
  i2cManager.writeRegister(this->deviceHandle, 0x07, data, 1);
  data[0] = 0x00;
  i2cManager.writeRegister(this->deviceHandle, 0x05, data, 1);
  data[0] = 0x60;
  i2cManager.writeRegister(this->deviceHandle, 0x07, data, 1);

  // Read error status register

  i2cManager.readRegister(this->deviceHandle, 0x02, data, 1);

  if (data[0] & 2) {
    ESP_LOGW(TAG, "Overcurrent, overtemperature or undervoltage errors");
  }

  if (data[0] & 4) {
    ESP_LOGW(TAG, "Clip indicator");
  }

  if (data[0] & 8) {
    ESP_LOGW(TAG, "Frame slip");
  }

  if (data[0] & 16) {
    ESP_LOGW(TAG, "LRCLK error");
  }

  if (data[0] & 32) {
    ESP_LOGW(TAG, "SCLK error");
  }

  if (data[0] & 64) {
    ESP_LOGW(TAG, "PLL autolock error");
  }

  if (data[0] & 128) {
    ESP_LOGW(TAG, "MCLK error");
  }

  return ESP_OK;
}

esp_err_t AudioController::i2sInit() {
  this->channelConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&channelConfig, &channelHandle, NULL);

  this->i2sConfig = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                      I2S_SLOT_MODE_STEREO),
      .gpio_cfg =
          {
              // refer to configuration.h for pin setup
              .mclk = AUDIO_MCLK,
              .bclk = AUDIO_SCLK,
              .ws = AUDIO_LRCLK,
              .dout = AUDIO_DOUT,
              .din = GPIO_NUM_NC,
              .invert_flags =
                  {
                      .mclk_inv = false,
                      .bclk_inv = false,
                      .ws_inv = false,
                  },
          },
  };

  return i2s_channel_init_std_mode(channelHandle, &i2sConfig);
}

void AudioController::playAudioFile(const char *filepath) {
  stopPlaying();

  std::string uri = "/spiffs/";
  uri = uri.append(filepath);

  ESP_LOGI(TAG, "Playing %s", uri.c_str());

  std::ifstream file(uri.c_str(),
                     std::ios::in | std::ios::binary | std::ios::ate);
  char audiodatafer[(AUDIO_BUFFER)];

  if (!file.is_open()) {
    ESP_LOGE(TAG, "Failed to open file");
    // return ESP_ERR_INVALID_ARG;
    return;
  }

  // skip the header...
  file.seekg(44, std::ios::beg);

  size_t bytes_written = 0;

  i2s_channel_enable(this->channelHandle);

  this->playing = true;

  while (file.good() && this->playing) {
    file.read(audiodatafer, AUDIO_BUFFER);
    std::streamsize bytesRead = file.gcount();
    i2s_channel_write(this->channelHandle, audiodatafer,
                      bytesRead * sizeof(char), &bytes_written, portMAX_DELAY);

    ESP_LOGV(TAG, "Bytes read: %d", bytesRead);
  }

  i2s_channel_disable(this->channelHandle);
}

void AudioController::playAudioFileAsync(const char *filepath) {
  auto cfg = esp_pthread_get_default_config();
  cfg.thread_name = "PlayAudioAsync";
  cfg.prio = 5;
  cfg.stack_size = 4192;
  esp_pthread_set_cfg(&cfg);
  std::thread([&]() { playAudioFile(filepath); }).detach();
}

void AudioController::setVolume(int volume) {

  int vol_idx = 0;

  if (vol < TAS5731M_VOLUME_MIN) {
    vol = TAS5731M_VOLUME_MIN;
  }
  if (vol > TAS5731M_VOLUME_MAX) {
    vol = TAS5731M_VOLUME_MAX;
  }
  vol_idx = vol / 5;

  std::vector<uint8_t> data;

  data.push_back(tas5731m_volume[vol_idx]);
  this->i2cManager.writeRegister(this->deviceHandle, MASTER_VOL_REG_ADDR, data);
  ESP_LOGI(TAG, "volume = 0x%x",
           data[1]); // the value is at index 1, because writeRegister places
                     // the register address at index 0
}

void AudioController::getVolume(int &volume) {

  /// FIXME: Got the digit volume is not right.
  std::vector<uint8_t> data;

  this->i2cManager.readRegister(this->deviceHandle, MASTER_VOL_REG_ADDR, data,
                                1);
  // TAS5731M_ASSERT(ret, "Failed to get volume", ESP_FAIL);
  int i;
  for (i = 0; i < sizeof(tas5731m_volume); i++) {
    if (data[0] >= tas5731m_volume[i])
      break;
  }
  ESP_LOGI(TAG, "Volume is %d", i * 5);
  volume = 5 * i;
}

void AudioController::stopPlaying() { this->playing = false; }

bool AudioController::isPlaying() { return this->playing; }

uint8_t AudioController::getErrors() {

  //    // check for errors
  //    uint8_t err_data[1] = {0};
  //    uint8_t reg = 0x02;
  //
  //
  //    I2C_DEV_TAKE_MUTEX(&i2c_dev);
  //    I2C_DEV_CHECK(&i2c_dev, i2c_dev_read_reg(&i2c_dev, reg, err_data, 1));
  //    I2C_DEV_GIVE_MUTEX(&i2c_dev);
  //
  //    ESP_LOG_dataFER_HEX(TAG, err_data, 1);
  //
  //    return err_data[0];
  return (uint8_t)0;
}
