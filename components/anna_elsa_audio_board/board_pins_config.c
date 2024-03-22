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

#include <string.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "board.h"
#include "audio_error.h"
#include "audio_mem.h"
#include "board_pins_config.h"

#define GPIO_I2C_SDA GPIO_NUM_21
#define GPIO_I2C_SCL GPIO_NUM_22

static const char *TAG = "ANNA_ELSA_AUDIO_BOARD_V1_0";

esp_err_t get_i2c_pins(i2c_port_t port, i2c_master_bus_config_t *i2c_config)

{    
    AUDIO_NULL_CHECK(TAG, i2c_config, return ESP_FAIL);
    if (port == I2C_NUM_0) {
        i2c_config->sda_io_num = GPIO_I2C_SDA;
        i2c_config->scl_io_num = GPIO_I2C_SCL;
    } else {
        i2c_config->sda_io_num = -1;
        i2c_config->scl_io_num = -1;
        ESP_LOGE(TAG, "i2c port %d is not supported", port);
        return ESP_FAIL;
    }
    return ESP_OK;
}


esp_err_t get_i2s_pins(i2s_port_t port, i2s_std_config_t *i2s_std_config) {
    AUDIO_NULL_CHECK(TAG, i2s_std_config, return ESP_FAIL);
   
    i2s_std_config->gpio_cfg.bclk = GPIO_NUM_26;
    i2s_std_config->gpio_cfg.ws = GPIO_NUM_25;
    i2s_std_config->gpio_cfg.dout = GPIO_NUM_27;        

    return ESP_OK;
}

esp_err_t i2s_mclk_gpio_select(i2s_port_t i2s_num, gpio_num_t gpio_num)
{
    
//  if (i2s_num >= I2S_STD_NUM_MAX) {
//        ESP_LOGE(TAG, "Does not support i2s number(%d)", i2s_num);
//        return ESP_ERR_INVALID_ARG;
//    }
    ESP_LOGD(TAG, "I2S%d, MCLK output by GPIO%d", i2s_num, gpio_num);
    return ESP_OK;
}

// sdcard

int8_t get_sdcard_intr_gpio(void)
{
    return -1;
}

int8_t get_sdcard_open_file_num_max(void)
{
    return -1;
}

// input-output pins

int8_t get_pa_enable_gpio(void)
{
    return GPIO_NUM_2;
}


// adc button

int8_t get_adc_detect_gpio(void)
{
    return -1;
}

int8_t get_input_rec_id(void)
{
    return -1;
}

int8_t get_input_mode_id(void)
{
    return -1;
}

int8_t get_input_play_id(void)
{
    return -1;
}

int8_t get_input_set_id(void)
{
    return -1;
}

int8_t get_input_volup_id(void)
{
    return -1;
}

int8_t get_input_voldown_id(void)
{
    return -1;
}


// board related

int8_t get_reset_codec_gpio(void)
{
    return -1;
}

int8_t get_reset_board_gpio(void)
{
    return -1;
}
