/*
 * The MIT License
 *
 * Copyright 2021 chris.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "driver/gpio.h"
#include "audio_mem.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "board.h"



static const char *TAG = "ANNA_ELSA_AUDIO_BOARD";

static audio_board_handle_t board_handle = 0;


audio_board_handle_t audio_board_init(void) {
    ESP_LOGD(TAG, "audio_board_init called");
    if (board_handle) {
        ESP_LOGW(TAG, "The board has already been initialized!");
        return board_handle;
    }
    board_handle = (audio_board_handle_t) audio_calloc(1, sizeof (struct audio_board_handle));
    AUDIO_MEM_CHECK(TAG, board_handle, return NULL);  
    
    board_handle->audio_hal = audio_board_codec_init();
    
    return board_handle;
}

audio_hal_handle_t audio_board_codec_init(void) {
    ESP_LOGD(TAG, "audio_board_codec_init called");
    audio_hal_codec_config_t audio_codec_cfg = AUDIO_CODEC_DEFAULT_CONFIG();
    audio_hal_handle_t codec_hal = audio_hal_init(&audio_codec_cfg, &AUDIO_CODEC_TAS5731M_DEFAULT_HANDLE);
    AUDIO_NULL_CHECK(TAG, codec_hal, return NULL);
    return codec_hal;

}

display_service_handle_t audio_board_led_init(void) {
    return NULL;
}

esp_err_t audio_board_sdcard_init(esp_periph_set_handle_t set, periph_sdcard_mode_t mode) {
    return ESP_OK;
}

esp_err_t audio_board_key_init(esp_periph_set_handle_t set) {
    return ESP_OK;
}

audio_board_handle_t audio_board_get_handle(void) {
    return board_handle;
}

esp_err_t audio_board_deinit(audio_board_handle_t audio_board) {
    esp_err_t ret = ESP_OK;
    ret = audio_hal_deinit(audio_board->audio_hal);
    audio_free(audio_board);
    board_handle = NULL;
    return ret;
}

