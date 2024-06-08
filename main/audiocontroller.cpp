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
#include <ios>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <streambuf>
#include <vector>
#include <thread>

#include "driver/i2s_common.h"
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "hal/i2s_types.h"

#include "audiocontroller.h"
#include "config.h"

static const char *TAG = "AudioController";

AudioController::AudioController(I2CManager& i2cmgr) : i2cManager(i2cmgr) {
	
}

AudioController::~AudioController() {}

void AudioController::initialise() {

  ESP_LOGI(TAG, "Enter initialise()");

  ESP_LOGI(TAG, "Initialising board and codecs");

  init_spiffs();

  i2cInit();
  i2sInit();
  
}

esp_err_t AudioController::i2cInit() {
	this->deviceConfig.device_address = TAS5731M_I2C_ADDRESS;
    this->deviceConfig.scl_speed_hz = 100000;
    this->deviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;
		
	i2cManager.addDevice(deviceConfig, deviceHandle);
	
	uint8_t buf[10];
    buf[0] = 0x00;
    // init sequence
    i2cManager.writeRegister(this->deviceHandle, 0x1b, buf, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    buf[0] = 0x03;
    i2cManager.writeRegister(this->deviceHandle,0x04, buf, 1);
    buf[0] = 0x00;
    i2cManager.writeRegister(this->deviceHandle,0x06, buf, 1);
    buf[0] = 0x30;
    i2cManager.writeRegister(this->deviceHandle,0x0a, buf, 1);
    i2cManager.writeRegister(this->deviceHandle,0x09, buf, 1);
    i2cManager.writeRegister(this->deviceHandle,0x08, buf, 1);
    buf[0] = 0x54;
    i2cManager.writeRegister(this->deviceHandle,0x14, buf, 1);
    buf[0] = 0xac;
    i2cManager.writeRegister(this->deviceHandle,0x13, buf, 1);
    buf[0] = 0x54;
    i2cManager.writeRegister(this->deviceHandle,0x12, buf, 1);
    buf[0] = 0xac;
    i2cManager.writeRegister(this->deviceHandle,0x11, buf, 1);
    buf[0] = 0x91;
    i2cManager.writeRegister(this->deviceHandle,0x0e, buf, 1);
    buf[0] = 0x00;
    buf[1] = 0x01;
    buf[2] = 0x77;
    buf[3] = 0x72;
    i2cManager.writeRegister(this->deviceHandle,0x20, buf, 4);
    buf[0] = 0x02;
    i2cManager.writeRegister(this->deviceHandle,0x10, buf, 1);
    buf[0] = 0x00;
    i2cManager.writeRegister(this->deviceHandle,0x0b, buf, 1);
    buf[0] = 0x02;
    i2cManager.writeRegister(this->deviceHandle,0x10, buf, 1);
    i2cManager.writeRegister(this->deviceHandle,0x1c, buf, 1);
    buf[0] = 0x30;
    i2cManager.writeRegister(this->deviceHandle,0x19, buf, 1);
    buf[0] = 0x01;
    buf[1] = 0x02;
    buf[2] = 0x13;
    buf[3] = 0x45;
    i2cManager.writeRegister(this->deviceHandle,0x25, buf, 4);
    buf[0] = 0xff;
    i2cManager.writeRegister(this->deviceHandle,0x07, buf, 1);
    buf[0] = 0x00;
    i2cManager.writeRegister(this->deviceHandle,0x05, buf, 1);
    buf[0] = 0x60;
    i2cManager.writeRegister(this->deviceHandle,0x07, buf, 1);
    
    // Read error status register    
    i2cManager.writeRegister(this->deviceHandle,0x02, buf, 1);   

    if (buf[0] & 2) {
        ESP_LOGW(TAG, "Overcurrent, overtemperature or undervoltage errors");
    }

    if (buf[0] & 4) {
        ESP_LOGW(TAG, "Clip indicator");
    }

    if (buf[0] & 8) {
        ESP_LOGW(TAG, "Frame slip");
    }

    if (buf[0] & 16) {
        ESP_LOGW(TAG, "LRCLK error");
    }

    if (buf[0] & 32) {
        ESP_LOGW(TAG, "SCLK error");
    }

    if (buf[0] & 64) {
        ESP_LOGW(TAG, "PLL autolock error");
    }

    if (buf[0] & 128) {
        ESP_LOGW(TAG, "MCLK error");
    }


  return ESP_OK;

}

esp_err_t AudioController::i2sInit() {
	this->channelConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

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
  
  std::ifstream file (uri.c_str(), std::ios::in|std::ios::binary|std::ios::ate);
  char audioBuffer[(AUDIO_BUFFER)];
  
  if (!file.is_open()) {
    ESP_LOGE(TAG, "Failed to open file");
    //return ESP_ERR_INVALID_ARG;
    return;
  }

  // skip the header...  
  file.seekg(44, std::ios::beg);
      
  size_t bytes_written = 0;
  
  i2s_channel_enable(this->channelHandle);
  
  while (file.good()) {
    file.read(audioBuffer, AUDIO_BUFFER);
    std::streamsize bytesRead = file.gcount();
	i2s_channel_write(this->channelHandle, audioBuffer, bytesRead * sizeof(char), &bytes_written, portMAX_DELAY);
    
    ESP_LOGV(TAG, "Bytes read: %d", bytesRead);
  }

  i2s_channel_disable(this->channelHandle);  
}

void AudioController::playAudioFileAsync(const char *filepath) {
	std::thread([&]() {
		playAudioFile(filepath);
	});
  
}

void AudioController::setVolume(int volume) {
  
}

void AudioController::stopPlaying() {
  
}

bool AudioController::isPlaying() {
  // TODO - find a method that will enable us to find out if the playing has
  // finished
  return true;
}

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
  //    ESP_LOG_BUFFER_HEX(TAG, err_data, 1);
  //
  //    return err_data[0];
  return (uint8_t)0;
}