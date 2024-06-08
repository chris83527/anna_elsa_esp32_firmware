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
#include <inttypes.h>

#include "soc/io_mux_reg.h"

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
audio_hal_func_t TAS5731M::getHandle() {
	audio_hal_func_t handle = {0};
	handle.audio_codec_initialize = init;
	handle.audio_codec_deinitialize = this->deinit;
	handle.audio_codec_ctrl = this->ctrl;
	handle.audio_codec_config_iface = this->deviceConfig;
	handle.audio_codec_set_mute = this->setMute;
	handle.audio_codec_set_volume = this->setVolume;
	handle.audio_codec_get_volume = this->getVolume;
	handle.audio_hal_lock = NULL;
	handle.handle = NULL;
	
	return handle;
}

esp_err_t TAS5731M::transmitRegisters() {

    esp_err_t ret = ESP_OK;
    
    uint8_t buf[10];
    buf[0] = 0x00;
    // init sequence
    i2c_manager.writeRegister(this->deviceHandle, 0x1b, buf, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    buf[0] = 0x03;
    i2c_manager.writeRegister(this->deviceHandle,0x04, buf, 1);
    buf[0] = 0x00;
    i2c_manager.writeRegister(this->deviceHandle,0x06, buf, 1);
    buf[0] = 0x30;
    i2c_manager.writeRegister(this->deviceHandle,0x0a, buf, 1);
    i2c_manager.writeRegister(this->deviceHandle,0x09, buf, 1);
    i2c_manager.writeRegister(this->deviceHandle,0x08, buf, 1);
    buf[0] = 0x54;
    i2c_manager.writeRegister(this->deviceHandle,0x14, buf, 1);
    buf[0] = 0xac;
    i2c_manager.writeRegister(this->deviceHandle,0x13, buf, 1);
    buf[0] = 0x54;
    i2c_manager.writeRegister(this->deviceHandle,0x12, buf, 1);
    buf[0] = 0xac;
    i2c_manager.writeRegister(this->deviceHandle,0x11, buf, 1);
    buf[0] = 0x91;
    i2c_manager.writeRegister(this->deviceHandle,0x0e, buf, 1);
    buf[0] = 0x00;
    buf[1] = 0x01;
    buf[2] = 0x77;
    buf[3] = 0x72;
    i2c_manager.writeRegister(this->deviceHandle,0x20, buf, 4);
    buf[0] = 0x02;
    i2c_manager.writeRegister(this->deviceHandle,0x10, buf, 1);
    buf[0] = 0x00;
    i2c_manager.writeRegister(this->deviceHandle,0x0b, buf, 1);
    buf[0] = 0x02;
    i2c_manager.writeRegister(this->deviceHandle,0x10, buf, 1);
    i2c_manager.writeRegister(this->deviceHandle,0x1c, buf, 1);
    buf[0] = 0x30;
    i2c_manager.writeRegister(this->deviceHandle,0x19, buf, 1);
    buf[0] = 0x01;
    buf[1] = 0x02;
    buf[2] = 0x13;
    buf[3] = 0x45;
    i2c_manager.writeRegister(this->deviceHandle,0x25, buf, 4);
    buf[0] = 0xff;
    i2c_manager.writeRegister(this->deviceHandle,0x07, buf, 1);
    buf[0] = 0x00;
    i2c_manager.writeRegister(this->deviceHandle,0x05, buf, 1);
    buf[0] = 0x60;
    i2c_manager.writeRegister(this->deviceHandle,0x07, buf, 1);
    
    // Read error status register    
    i2c_manager.writeRegister(this->deviceHandle,0x02, buf, 1);   

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

esp_err_t TAS5731M::init(audio_hal_codec_config_t& codec_cfg) {
    esp_err_t ret = ESP_OK;
    ESP_LOGD(TAG, "Power ON CODEC with GPIO %d", TAS5731M_PDWN_GPIO);

    esp_rom_gpio_pad_select_gpio(TAS5731M_RST_GPIO);
    esp_rom_gpio_pad_select_gpio(TAS5731M_PDWN_GPIO);

    gpio_set_direction(TAS5731M_RST_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(TAS5731M_PDWN_GPIO, GPIO_MODE_OUTPUT);

    uint32_t reg_val = REG_READ(PIN_CTRL);
    ESP_LOGD(TAG, "PIN_CTRL before: %d", reg_val);
    REG_WRITE(PIN_CTRL, 0xFFFFFFF0);
    reg_val = REG_READ(PIN_CTRL);
    ESP_LOGD(TAG, "PIN_CTRL after: %d", reg_val);
    PIN_FUNC_SELECT(GPIO_PIN_REG_0, 1); //GPIO0 as CLK_OUT1

    // See TI TAS5731M Datasheet page 63
    gpio_set_level(TAS5731M_RST_GPIO, 0); // Drive /RESET = 0
    gpio_set_level(TAS5731M_PDWN_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(TAS5731M_PDWN_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(TAS5731M_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ret |= transmitRegisters();

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "failed to transmit register");
        return ESP_FAIL;
    }

    TAS5731M_ASSERT(ret, "Fail to initialise TAS5731M PA", ESP_FAIL);
    return ret;
}

esp_err_t TAS5731M::setVolume(int vol) {
    esp_err_t ret = ESP_OK;

    int vol_idx = 0;

    if (vol < TAS5731M_VOLUME_MIN) {
        vol = TAS5731M_VOLUME_MIN;
    }
    if (vol > TAS5731M_VOLUME_MAX) {
        vol = TAS5731M_VOLUME_MAX;
    }
    vol_idx = vol / 5;

    uint8_t cmd[1] = {0};
   
    cmd[0] = tas5731m_volume[vol_idx];
    this->i2c_manager.writeRegister(this->deviceHandle, MASTER_VOL_REG_ADDR, cmd, 1);
    ESP_LOGW(TAG, "volume = 0x%x", cmd[0]);
    return ret;
}

esp_err_t TAS5731M::getVolume(int& value) {
    esp_err_t ret = ESP_OK;
    /// FIXME: Got the digit volume is not right.
    uint8_t cmd[1] = {0x00};
    
    ret = this->i2c_manager.readRegister(this->deviceHandle, MASTER_VOL_REG_ADDR, cmd, 1);
    TAS5731M_ASSERT(ret, "Fail to get volume", ESP_FAIL);
    int i;
    for (i = 0; i < sizeof (tas5731m_volume); i++) {
        if (cmd[0] >= tas5731m_volume[i])
            break;
    }
    ESP_LOGD(TAG, "Volume is %d", i * 5);
    value = 5 * i;
    return ret;
}

esp_err_t TAS5731M::setMute(bool enable) {
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

esp_err_t TAS5731M::getMute(int *value) {
    esp_err_t ret = ESP_OK;
    uint8_t cmd[2] = {MASTER_VOL_REG_ADDR, 0x00};
    //  ret |= i2c_bus_read_bytes(i2c_handler, TAS5731M_ADDRESS, &cmd[0], 1, &cmd[1], 1);

    TAS5731M_ASSERT(ret, "Fail to get mute", ESP_FAIL);
    *value = (cmd[1] & 0x08) >> 4;
    ESP_LOGD(TAG, "Get mute value: 0x%x", *value);
    return ret;
}

esp_err_t TAS5731M::deinit(void) {
    // TODO
    return ESP_OK;
}

esp_err_t TAS5731M::ctrl(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state) {
    // TODO
    return ESP_OK;
}

esp_err_t TAS5731M::configureInterface(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t *iface) {
    //TODO
    return ESP_OK;
}
