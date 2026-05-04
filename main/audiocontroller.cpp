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
 * @file audiocontroller.cpp
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
#include <streambuf>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "spiffs.h"

#include "config.h"
#include "audiocontroller.h"
#include "maincontroller.h"

static const char* TAG = "AudioController";

AudioController::AudioController(I2CBus& bus, MainController* mainController) : tas5731m(bus, TAS5731M_I2C_ADDRESS, TAS5731M_RST_GPIO, TAS5731M_PDWN_GPIO, AUDIO_MCLK, AUDIO_SCLK, AUDIO_LRCLK, AUDIO_DOUT), mainController(mainController)
{
}


void AudioController::initialise()
{
    ESP_LOGI(TAG, "Enter initialise()");

    ESP_LOGI(TAG, "Initialising board and codecs");

    init_spiffs();

    tas5731m.initialise();

    setVolume(mainController->getNvsController().readValueFromNVS("volume")); // Get this volume from NVRAM

    playAudioFile(Sounds::SND_STARTUP);
}

void AudioController::playAudioFile(const char* filepath)
{
    stopPlaying();

    std::string uri = "/spiffs/";
    uri = uri.append(filepath);

    ESP_LOGI(TAG, "Playing %s", uri.c_str());

    std::ifstream file(uri.c_str(),
                       std::ios::in | std::ios::binary | std::ios::ate);
    char audioData[(AUDIO_BUFFER)];

    if (!file.is_open())
    {
        ESP_LOGE(TAG, "Failed to open file");
        // return ESP_ERR_INVALID_ARG;
        return;
    }

    // skip the header...
    file.seekg(44, std::ios::beg);

    tas5731m.enableChannel();

    this->playing = true;

    while (file.good())
    {
        file.read(audioData, AUDIO_BUFFER);
        std::streamsize bytesRead = file.gcount();
        tas5731m.writeAudioData(audioData, bytesRead);

        ESP_LOGV(TAG, "Bytes read: %d", bytesRead);
    }

    tas5731m.disableChannel();
}

void AudioController::playAudioFileAsync(const char* filepath)
{
    /*
    auto cfg = esp_pthread_get_default_config();
    cfg.thread_name = "PlayAudioAsync";
    cfg.prio = 5;
    cfg.stack_size = 4192;
    esp_pthread_set_cfg(&cfg);
    */
    std::thread([&]() { playAudioFile(filepath); }).detach();
}

void AudioController::setVolume(int volume)
{
    tas5731m.setVolume(volume);

    // publish the change via Websocket
    mainController->getNvsController().writeValueToNVS("volume", volume);
    mainController->getHttpController().broadcast_status();
}

void AudioController::mute()
{
    tas5731m.setMute(true);
}

void AudioController::unmute()
{
    tas5731m.setMute(false);
}

int AudioController::getVolume()
{
    return tas5731m.getVolume();
}

void AudioController::stopPlaying() { this->playing = false; }

bool AudioController::isPlaying() const { return this->playing; }
