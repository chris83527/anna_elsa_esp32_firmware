#pragma once

#include "led_strip.h"
#include "led_strip_rmt.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include <cstdint>
#include <vector>

#include "crgb.h"
#include "palette.h"

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
    esp_err_t fill_solid(CRGBPalette16 &pal, uint8_t count, const CRGB &color);
    esp_err_t fill_rainbow(uint8_t initial_hue, uint8_t delta_hue);
    esp_err_t fadeToBlackBy(uint8_t amount);
    esp_err_t nscale8(uint8_t scale);
    esp_err_t clear();
    esp_err_t show();

    uint16_t size() const { return led_count_; }

private:
    gpio_num_t gpio_;
    uint16_t led_count_;

    led_strip_handle_t strip_ = nullptr;
    std::vector<uint8_t> buffer_;   // GRB order
};