#pragma once

#ifndef __WS2812B_H__
#define __WS2812B_H__

#include "led_strip.h"
//#include "driver/rmt_tx.h"
#include "driver/i2s_common.h"
#include "driver/i2s_std.h"
#include "esp_err.h"
#include <vector>

#include "palette.hpp"

#define PIXEL_SIZE 12
#define SAMPLE_RATE (93750)
#define ZERO_BUFFER 48
#define I2S_NUM I2S_NUM_1

class WS2812B {
public:
    WS2812B(gpio_num_t gpio, uint16_t led_count);
    ~WS2812B();

    esp_err_t init();
    esp_err_t set_pixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
    esp_err_t set_all(uint8_t r, uint8_t g, uint8_t b);
    esp_err_t set_pixel(uint16_t index, const CRGB &c);
    esp_err_t set_all(const CRGB &c);
    esp_err_t fill_palette(const CRGBPalette16 &pal,
                       uint8_t start_index,
                       uint8_t step,
                       TBlendType blend = LINEARBLEND);
    esp_err_t fill_solid(const CRGB &color);
    esp_err_t fill_rainbow(uint8_t initial_hue, uint8_t delta_hue);
    esp_err_t fadeToBlackBy(uint8_t amount);
    esp_err_t nscale8(uint8_t scale);
    esp_err_t clear();
    esp_err_t show();

    uint16_t size() const { return led_count_; }

private:
    gpio_num_t gpio_;
    uint16_t led_count_;

    static constexpr uint16_t bitpatterns[4] = {0x88, 0x8e, 0xe8, 0xee};
    std::vector<uint8_t> out_buffer_; // [LED_NUMBER * PIXEL_SIZE] = {0};
    std::vector<uint8_t> off_buffer_; // [ZERO_BUFFER] = {0};
    std::vector<uint8_t> buffer_;   // GRB order
    uint16_t size_buffer_;

    i2s_chan_handle_t channelHandle{};
    i2s_chan_config_t channelConfig{};
    i2s_std_config_t i2sConfig{};
};

#endif