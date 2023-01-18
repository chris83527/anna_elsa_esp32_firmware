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
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <cstring>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "driver/i2s.h"
#include "spiffs_stream.h"
#include "i2s_stream.h"
#include "ogg_decoder.h"
#include "board.h"
#include "audio_def.h"
#include "esp_audio.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "audiocontroller.h"
#include "config.h"

static const char *TAG = "AudioController";

AudioController::AudioController() {

}

AudioController::AudioController(const AudioController& orig) {

}

AudioController::~AudioController() {
}

void AudioController::initialise() {

    ESP_LOGI(TAG, "Enter initialise()");

    ESP_LOGI(TAG, "Initialising board and codecs");
    I2S_NUM = I2S_NUM_0; // i2s port number

    cfg = DEFAULT_ESP_AUDIO_CONFIG();
    cfg.in_stream_buf_size = 10 * 1024;
    cfg.out_stream_buf_size = 10 * 1024;

    board_handle = audio_board_init();

    if (board_handle != NULL) {

        esp_err_t ret = audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);

        if (ret == ESP_OK) {
            cfg.vol_handle = board_handle->audio_hal;
            cfg.task_prio = 20;
            cfg.vol_set = (audio_volume_set) audio_hal_set_volume;
            cfg.vol_get = (audio_volume_get) audio_hal_get_volume;
            cfg.prefer_type = ESP_AUDIO_PREFER_MEM;
            cfg.cb_func = NULL;
            cfg.cb_ctx = NULL;


            player = esp_audio_create(&cfg);

            if (player != NULL) {
                // Create readers and add to esp_audio
                ESP_LOGI(TAG, "Creating spiffs stream reader...");

                spiffs_reader = SPIFFS_STREAM_CFG_DEFAULT();
                spiffs_reader.type = AUDIO_STREAM_READER;

                esp_audio_input_stream_add(player, spiffs_stream_init(&spiffs_reader));

                // Add decoders and encoders to esp_audio
                ESP_LOGI(TAG, "Creating ogg vorbis decoder...");
                ogg_dec_cfg = DEFAULT_OGG_DECODER_CONFIG();
                esp_audio_codec_lib_add(player, AUDIO_CODEC_TYPE_DECODER, ogg_decoder_init(&ogg_dec_cfg));

                // Add i2s writer
                // Create writers and add to esp_audio
                ESP_LOGI(TAG, "Creating i2s writer...");
                i2s_writer = I2S_STREAM_CFG_DEFAULT();
                i2s_writer.i2s_config.fixed_mclk = 1;
                i2s_writer.i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
                esp_audio_output_stream_add(player, i2s_stream_init(&i2s_writer));

                // Set default volume
                ESP_LOGI(TAG, "Setting volume to 45...");
                //esp_audio_vol_set(player, 80);

                playAudioFileSync(Sounds::SND_KERCHING);

                ESP_LOGI(TAG, "esp_audio instance is:%p", player);
            }
        }
    }
}

void AudioController::playAudioFile(const char* filepath) {
    std::string uri = "spiffs://";
    uri = uri.append(filepath);
    ESP_LOGI(TAG, "Playing %s", uri.c_str());
    esp_audio_play(player, AUDIO_CODEC_TYPE_DECODER, uri.c_str(), 0);
}

void AudioController::playAudioFileSync(const char* filepath) {
    std::string uri = "spiffs://";
    uri = uri.append(filepath);
    ESP_LOGI(TAG, "Playing %s", uri.c_str());
    esp_audio_sync_play(player, uri.c_str(), 0);
}

void AudioController::setVolume(int volume) {
    esp_audio_vol_set(player, volume);
}

void AudioController::stopPlaying() {
    esp_audio_stop(player, TERMINATION_TYPE_NOW);
}

bool AudioController::isPlaying() {
    // TODO - find a method that will enable us to find out if the playing has finished
    return true;
}