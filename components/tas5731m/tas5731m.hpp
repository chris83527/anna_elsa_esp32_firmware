/*
* The MIT License
 *
 * Copyright 2023 chris.
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

/*
 * File:   tas5731m.h
 * Author: chris
 *
 * Created on May 2, 2026, 6:48 PM
 */
#ifndef TAS5731M_H
#define TAS5731M_H


#include "driver/i2s_common.h"
#include "driver/i2s_std.h"
#include "esp_err.h"
#include "esp_log.h"

#include "typed_i2c_device.hpp"


class TAS5731M : public TypedI2CDevice
{
public:
    TAS5731M(I2CBus& i2c_bus, uint8_t address, gpio_num_t resetPin, gpio_num_t powerDownPin, gpio_num_t mclkPin, gpio_num_t sclkPin, gpio_num_t lrckPin, gpio_num_t dataPin) :
        TypedI2CDevice(i2c_bus, address), resetPin(resetPin), powerDownPin(powerDownPin), mclkPin(mclkPin), sclkPin(sclkPin), lrckPin(lrckPin), dataPin(dataPin)
    {
    }

    ~TAS5731M() = default;

    esp_err_t initialise();
    void setVolume(int volume);
    int getVolume();
    esp_err_t enableChannel();
    esp_err_t disableChannel();
    esp_err_t setMute(bool mute);
    esp_err_t writeAudioData(char* audioData, int bytes);
    esp_err_t readErrorRegister();

private:
    const char* TAG = "TAS5731M";



    esp_err_t i2sInit();
    esp_err_t i2cInit();

    i2s_chan_handle_t channelHandle{};
    i2s_chan_config_t channelConfig{};
    i2s_std_config_t i2sConfig{};

    static constexpr int TAS5731M_I2C_ADDRESS = 0x1a;

    static constexpr int TAS5731M_VOLUME_MAX = 100;
    static constexpr int TAS5731M_VOLUME_MIN = 5;

    gpio_num_t resetPin;
    gpio_num_t powerDownPin;
    gpio_num_t mclkPin;
    gpio_num_t sclkPin;
    gpio_num_t lrckPin;
    gpio_num_t dataPin;

    static constexpr uint8_t tas5731m_volume[] = {
        0xff, 0x9f, 0x8f, 0x7f, 0x6f, 0x5f, 0x5c, 0x5a, 0x58, 0x54, 0x50,
        0x4c, 0x4a, 0x48, 0x44, 0x40, 0x3d, 0x3b, 0x39, 0x37, 0x35
    };

    uint8_t currentVolume;
};

#endif //TAS5731M_H
