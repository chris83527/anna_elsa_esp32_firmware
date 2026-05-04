//
// Created by chris on 03.05.26.
//


#include <thread>
#include <chrono>
#include <vector>

#include <soc/io_mux_reg.h>
#include <soc/soc.h>

#include "driver/gpio.h"

#include "tas5731m.hpp"

#include <portmacro.h>

#include "tas5731m_regs.hpp"

esp_err_t TAS5731M::initialise()
{
    ESP_LOGD(TAG, "Power ON CODEC with GPIO %d", powerDownPin);

    esp_rom_gpio_pad_select_gpio(resetPin);
    esp_rom_gpio_pad_select_gpio(powerDownPin);

    gpio_set_direction(resetPin, GPIO_MODE_OUTPUT);
    gpio_set_direction(powerDownPin, GPIO_MODE_OUTPUT);

    uint32_t reg_val = REG_READ(PIN_CTRL);
    ESP_LOGD(TAG, "PIN_CTRL before: %" PRIu32 "", reg_val);
    REG_WRITE(PIN_CTRL, 0xFFFFFFF0);
    reg_val = REG_READ(PIN_CTRL);
    ESP_LOGD(TAG, "PIN_CTRL after: %" PRIu32 "", reg_val);
    PIN_FUNC_SELECT(GPIO_PIN_REG_0, 1); // GPIO0 as CLK_OUT1

    // See TI TAS5731M Datasheet page 63
    gpio_set_level(resetPin, 0); // Drive /RESET = 0
    gpio_set_level(powerDownPin, 1);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
    gpio_set_level(resetPin, 1);
    std::this_thread::sleep_for(std::chrono::microseconds(10));

    i2cInit();
    i2sInit();

    return ESP_OK;
}

esp_err_t TAS5731M::i2cInit()
{
    // init sequence (page 63, TAS5731M datasheet)
    // Trim oscillator (write 0x00 to register 0x1B) and wait at least 50 ms.
    writeReg(TAS5731M_REGISTERS::OSCILLATOR_TRIM_REGISTER, 0x00);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    writeReg(TAS5731M_REGISTERS::SERIAL_DATA_INTERFACE_REGISTER, 0x03);
    writeReg(TAS5731M_REGISTERS::SOFT_MUTE_REGISTER, 0x00);

    writeReg(TAS5731M_REGISTERS::CHANNEL_1_VOL, 0x30);
    writeReg(TAS5731M_REGISTERS::CHANNEL_2_VOL, 0x30);
    writeReg(TAS5731M_REGISTERS::CHANNEL_3_VOL, 0x30);

    writeReg(TAS5731M_REGISTERS::IC_DELAY_CHANNEL_1, 0xac);
    writeReg(TAS5731M_REGISTERS::IC_DELAY_CHANNEL_2, 0x54);
    writeReg(TAS5731M_REGISTERS::IC_DELAY_CHANNEL_3, 0xac);
    writeReg(TAS5731M_REGISTERS::IC_DELAY_CHANNEL_4, 0x54);

    writeReg(TAS5731M_REGISTERS::VOLUME_CONFIGURATION_REGISTER, 0x91);

    std::vector<uint8_t> data;
    data.push_back(0x00);
    data.push_back(0x01);
    data.push_back(0x77);
    data.push_back(0x72);
    writeReg(TAS5731M_REGISTERS::INPUT_MUX_REGISTER, data);

    writeReg(TAS5731M_REGISTERS::MODULATION_LIMIT_REGISTER, 0x02);
    writeReg(TAS5731M_REGISTERS::BKND_ERR_REGISTER, 0x02);
    writeReg(TAS5731M_REGISTERS::PWM_CHANNEL_SHUTDOWN_GROUP_REGISTER, 0x30);

    data.clear();
    data.push_back(0x01);
    data.push_back(0x02);
    data.push_back(0x13);
    data.push_back(0x45);
    writeReg(0x25, data);

    writeReg(TAS5731M_REGISTERS::MASTER_VOLUME, 0xff); // Mute

    writeReg(TAS5731M_REGISTERS::SYSTEM_CONTROL_REGISTER_2, 0x00);

    return ESP_OK;
}

esp_err_t TAS5731M::i2sInit()
{
    this->channelConfig = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_new_channel(&channelConfig, &channelHandle, nullptr);

    this->i2sConfig = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg =
        {
            // refer to configuration.h for pin setup
            .mclk = mclkPin,
            .bclk = sclkPin,
            .ws = lrckPin,
            .dout = dataPin,
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

void TAS5731M::setVolume(int volume)
{
    int vol_idx = 0;

    if (volume < TAS5731M_VOLUME_MIN)
    {
        volume = TAS5731M_VOLUME_MIN;
    }
    if (volume > TAS5731M_VOLUME_MAX)
    {
        volume = TAS5731M_VOLUME_MAX;
    }
    vol_idx = volume / 5;

    currentVolume = tas5731m_volume[vol_idx];
    writeReg(TAS5731M_REGISTERS::MASTER_VOLUME, currentVolume);
    ESP_LOGI(TAG, "Set volume to 0x%x", currentVolume);
}

int TAS5731M::getVolume()
{
    /// FIXME: Got the digit volume is not right.
    std::vector<uint8_t> data;
    data.resize(1);
    readReg(TAS5731M_REGISTERS::MASTER_VOLUME, data, 1);
    // TAS5731M_ASSERT(ret, "Failed to get volume", ESP_FAIL);
    int i;
    for (i = 0; i < sizeof(tas5731m_volume); i++)
    {
        if (data[0] >= tas5731m_volume[i])
            break;
    }
    ESP_LOGI(TAG, "Volume is %d", i * 5);
    return 5 * i;
}

esp_err_t TAS5731M::setMute(bool mute)
{
    if (mute)
    {
        return writeReg(TAS5731M_REGISTERS::MASTER_VOLUME, 0xff);
    }
    else
    {
        return writeReg(TAS5731M_REGISTERS::MASTER_VOLUME, currentVolume);
    }
}

esp_err_t TAS5731M::enableChannel()
{
    return i2s_channel_enable(channelHandle);
}

esp_err_t TAS5731M::disableChannel()
{
    return i2s_channel_disable(channelHandle);
}

esp_err_t TAS5731M::writeAudioData(char* audioData, int bytesRead)
{
    size_t bytes_written = 0;
    return i2s_channel_write(this->channelHandle, audioData,
                             bytesRead * sizeof(char), &bytes_written, portMAX_DELAY);
}

esp_err_t TAS5731M::readErrorRegister()
{
    // Read error status register
    std::vector<uint8_t> data(1);
    esp_err_t err = readReg(0x02, data, 1);
    if (err == ESP_OK)
    {
        if (data[0] & 2)
            ESP_LOGW(TAG, "Overcurrent, overtemperature or undervoltage errors");
        if (data[0] & 4)
            ESP_LOGW(TAG, "Clip indicator");
        if (data[0] & 8)
            ESP_LOGW(TAG, "Frame slip");
        if (data[0] & 16)
            ESP_LOGW(TAG, "LRCLK error");
        if (data[0] & 32)
            ESP_LOGW(TAG, "SCLK error");
        if (data[0] & 64)
            ESP_LOGW(TAG, "PLL autolock error");
        if (data[0] & 128)
            ESP_LOGW(TAG, "MCLK error");
    }
    return err;
}
