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

#include "board.h"
#include "esp_log.h"
#include "tas5731m.h"
#include "tas5731m_reg_cfg.h"

static const char *TAG = "TAS5731M";

//#define TAS5731M_ADDRESS 0x34 // ASEL pulled low
//#define TAS5731M_ADDRESS 0x36 // ASEL pulled high
#define TAS5731M_PDWN_GPIO get_pa_enable_gpio()
#define TAS5731M_RST_GPIO GPIO_NUM_14
#define TAS5731M_VOLUME_MAX 100
#define TAS5731M_VOLUME_MIN 100

#define TAS5731M_I2C_ADDRESS 0x1a

#define TAS5731M_ASSERT(a, format, b, ...) \
    if ((a) != 0) { \
        ESP_LOGE(TAG, format, ##__VA_ARGS__); \
        return b;\
    }

esp_err_t tas5731m_ctrl(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state);
esp_err_t tas5731m_config_iface(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t *iface);


/*
 * Operate function of PA
 */
audio_hal_func_t AUDIO_CODEC_TAS5731M_DEFAULT_HANDLE = {
    .audio_codec_initialize = tas5731m_init,
    .audio_codec_deinitialize = tas5731m_deinit,
    .audio_codec_ctrl = tas5731m_ctrl,
    .audio_codec_config_iface = tas5731m_config_iface,
    .audio_codec_set_mute = tas5731m_set_mute,
    .audio_codec_set_volume = tas5731m_set_volume,
    .audio_codec_get_volume = tas5731m_get_volume,
    .audio_hal_lock = NULL,
    .handle = NULL,
};

static esp_err_t tas5731m_transmit_registers() {

    esp_err_t ret = ESP_OK;
    
    uint8_t buf[10];
    buf[0] = 0x00;
    // init sequence
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x1b, &buf[0], 1);
    vTaskDelay(50 / portTICK_RATE_MS);
    buf[0] = 0x03;
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x04, &buf[0], 1);
    buf[0] = 0x00;
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x06, &buf[0], 1);
    buf[0] = 0x30;
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x0a, &buf[0], 1);
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x09, &buf[0], 1);
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x08, &buf[0], 1);
    buf[0] = 0x54;
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x14, &buf[0], 1);
    buf[0] = 0xac;
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x13, &buf[0], 1);
    buf[0] = 0x54;
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x12, &buf[0], 1);
    buf[0] = 0xac;
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x11, &buf[0], 1);
    buf[0] = 0x91;
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x0e, &buf[0], 1);
    buf[0] = 0x00;
    buf[1] = 0x01;
    buf[2] = 0x77;
    buf[3] = 0x72;
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x20, &buf[0], 4);
    buf[0] = 0x02;
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x10, &buf[0], 1);
    buf[0] = 0x00;
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x0b, &buf[0], 1);
    buf[0] = 0x02;
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x10, &buf[0], 1);
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x1c, &buf[0], 1);
    buf[0] = 0x30;
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x19, &buf[0], 1);
    buf[0] = 0x01;
    buf[1] = 0x02;
    buf[2] = 0x13;
    buf[3] = 0x45;
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x25, &buf[0], 4);
    buf[0] = 0xff;
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x07, &buf[0], 1);
    buf[0] = 0x00;
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x05, &buf[0], 1);
    buf[0] = 0x60;
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, 0x07, &buf[0], 1);
    
    // Read error status register    
    i2c_manager_read(I2C_NUM_0, TAS5731M_I2C_ADDRESS,  0x02, &buf[0], 1);   

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


    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fail to load configuration to TAS5731M");
        return ESP_FAIL;
    }

    return ret;
}

esp_err_t tas5731m_init(audio_hal_codec_config_t *codec_cfg) {
    esp_err_t ret = ESP_OK;
    ESP_LOGD(TAG, "Power ON CODEC with GPIO %d", TAS5731M_PDWN_GPIO);

    gpio_pad_select_gpio(TAS5731M_RST_GPIO);
    gpio_pad_select_gpio(TAS5731M_PDWN_GPIO);

    gpio_set_direction(TAS5731M_RST_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(TAS5731M_PDWN_GPIO, GPIO_MODE_OUTPUT);

    uint32_t reg_val = REG_READ(PIN_CTRL);
    ESP_LOGD(TAG, "PIN_CTRL before:%x", reg_val);
    REG_WRITE(PIN_CTRL, 0xFFFFFFF0);
    reg_val = REG_READ(PIN_CTRL);
    ESP_LOGD(TAG, "PIN_CTRL after:%x", reg_val);
    PIN_FUNC_SELECT(GPIO_PIN_REG_0, 1); //GPIO0 as CLK_OUT1

    // See TI TAS5731M Datasheet page 63
    gpio_set_level(TAS5731M_RST_GPIO, 0); // Drive /RESET = 0
    gpio_set_level(TAS5731M_PDWN_GPIO, 0);
    vTaskDelay(200 / portTICK_RATE_MS);
    gpio_set_level(TAS5731M_PDWN_GPIO, 1);
    vTaskDelay(200 / portTICK_RATE_MS);
    gpio_set_level(TAS5731M_RST_GPIO, 1);
    vTaskDelay(500 / portTICK_RATE_MS);
    
    ret |= tas5731m_transmit_registers();

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "failed to transmit register");
        return ESP_FAIL;
    }

    TAS5731M_ASSERT(ret, "Fail to initialise TAS5731M PA", ESP_FAIL);
    return ret;
}

esp_err_t tas5731m_set_volume(int vol) {
    esp_err_t ret = ESP_OK;

    int vol_idx = 0;

    if (vol < TAS5731M_VOLUME_MIN) {
        vol = TAS5731M_VOLUME_MIN;
    }
    if (vol > TAS5731M_VOLUME_MAX) {
        vol = TAS5731M_VOLUME_MAX;
    }
    vol_idx = vol / 5;

    uint8_t cmd[2] = {0, 0};

    cmd[0] = MASTER_VOL_REG_ADDR;
    cmd[1] = tas5731m_volume[vol_idx];
    i2c_manager_write(I2C_NUM_0, TAS5731M_I2C_ADDRESS, cmd[0], &cmd[1], 1);
    ESP_LOGW(TAG, "volume = 0x%x", cmd[1]);
    return ret;
}

esp_err_t tas5731m_get_volume(int *value) {
    esp_err_t ret = ESP_OK;
    /// FIXME: Got the digit volume is not right.
    uint8_t cmd[2] = {MASTER_VOL_REG_ADDR, 0x00};
    //ret = i2c_bus_read_bytes(i2c_handler, TAS5731M_ADDRESS, &cmd[0], 1, &cmd[1], 1);
    ret = i2c_manager_read(I2C_NUM_0, TAS5731M_I2C_ADDRESS, cmd[0], &cmd[1], 1);
    TAS5731M_ASSERT(ret, "Fail to get volume", ESP_FAIL);
    int i;
    for (i = 0; i < sizeof (tas5731m_volume); i++) {
        if (cmd[1] >= tas5731m_volume[i])
            break;
    }
    ESP_LOGD(TAG, "Volume is %d", i * 5);
    *value = 5 * i;
    return ret;
}

esp_err_t tas5731m_set_mute(bool enable) {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[2] = {MASTER_VOL_REG_ADDR, 0x00};
    // ret |= i2c_bus_read_bytes(i2c_handler, TAS5731M_ADDRESS, &cmd[0], 1, &cmd[1], 1);

    if (enable) {
        cmd[1] |= 0x8;
    } else {
        cmd[1] &= (~0x08);
    }
    //  ret |= i2c_bus_write_bytes(i2c_handler, TAS5731M_ADDRESS, &cmd[0], 1, &cmd[1], 1);

    TAS5731M_ASSERT(ret, "Fail to set mute", ESP_FAIL);
    return ret;
}

esp_err_t tas5731m_get_mute(int *value) {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[2] = {MASTER_VOL_REG_ADDR, 0x00};
    //  ret |= i2c_bus_read_bytes(i2c_handler, TAS5731M_ADDRESS, &cmd[0], 1, &cmd[1], 1);

    TAS5731M_ASSERT(ret, "Fail to get mute", ESP_FAIL);
    *value = (cmd[1] & 0x08) >> 4;
    ESP_LOGD(TAG, "Get mute value: 0x%x", *value);
    return ret;
}

esp_err_t tas5731m_deinit(void) {
    // TODO
    return ESP_OK;
}

esp_err_t tas5731m_ctrl(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state) {
    // TODO
    return ESP_OK;
}

esp_err_t tas5731m_config_iface(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t *iface) {
    //TODO
    return ESP_OK;
}
