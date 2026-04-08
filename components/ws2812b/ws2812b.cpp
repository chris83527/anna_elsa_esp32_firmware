#include "ws2812b.hpp"

#include <bits/this_thread_sleep.h>

#include "esp_err.h"
#include "esp_log.h"

static const char* TAG = "WS2812B";

WS2812B::WS2812B(gpio_num_t gpio, uint16_t led_count)
    : gpio_(gpio),
      led_count_(led_count),
      out_buffer_(led_count * PIXEL_SIZE, 0),
      off_buffer_(led_count * PIXEL_SIZE, 0),
      buffer_(led_count * 3, 0),
      size_buffer_(led_count * PIXEL_SIZE)
{
}

WS2812B::~WS2812B()
{
    //i2s_channel_driver_uninstall(I2S_NUM);
}

esp_err_t WS2812B::init()
{
    /*
    led_strip_config_t strip_config = {
        .strip_gpio_num = gpio_,
        .max_leds = led_count_,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false,
        },

    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10'000'000, // 10 MHz recommended
        .mem_block_symbols = 64,
        .flags = {
          .with_dma = 0
        },
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &strip_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LED strip: %s", esp_err_to_name(err));
        return err;
    }
    */

    this->i2sConfig = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg =
        {
            // refer to configuration.h for pin setup
            .mclk = GPIO_NUM_NC,
            .bclk = GPIO_NUM_NC,
            .ws = GPIO_NUM_NC,
            .dout = gpio_,
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

esp_err_t WS2812B::set_pixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= led_count_)
    {
        return ESP_ERR_INVALID_ARG;
    }

    buffer_[index * 3 + 0] = g;
    buffer_[index * 3 + 1] = r;
    buffer_[index * 3 + 2] = b;

    return ESP_OK;
}

esp_err_t WS2812B::set_all(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint16_t i = 0; i < led_count_; i++)
    {
        set_pixel(i, r, g, b);
    }
    return ESP_OK;
}

esp_err_t WS2812B::clear()
{
    std::fill(buffer_.begin(), buffer_.end(), 0);
    return show();
}

/*
esp_err_t WS2812B::show() {
    if (!strip_) return ESP_ERR_INVALID_STATE;

    esp_err_t err = led_strip_refresh(strip_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Refresh failed: %s", esp_err_to_name(err));
        return err;
    }

    // Push pixel data
    for (uint16_t i = 0; i < led_count_; i++) {
        uint8_t g = buffer_[i * 3 + 0];
        uint8_t r = buffer_[i * 3 + 1];
        uint8_t b = buffer_[i * 3 + 2];
        led_strip_set_pixel(strip_, i, r, g, b);
    }

    return led_strip_refresh(strip_);
}
*/

esp_err_t WS2812B::show()
{
    size_t bytes_written = 0;
    for (uint16_t i = 0; i < led_count_; i++)
    {
        int loc = i * PIXEL_SIZE;
        uint8_t g = buffer_[i * 3 + 0];
        uint8_t r = buffer_[i * 3 + 1];
        uint8_t b = buffer_[i * 3 + 2];

        out_buffer_[loc] = bitpatterns[g >> 6 & 0x03];
        out_buffer_[loc + 1] = bitpatterns[g >> 4 & 0x03];
        out_buffer_[loc + 2] = bitpatterns[g >> 2 & 0x03];
        out_buffer_[loc + 3] = bitpatterns[g & 0x03];

        out_buffer_[loc + 4] = bitpatterns[r >> 6 & 0x03];
        out_buffer_[loc + 5] = bitpatterns[r >> 4 & 0x03];
        out_buffer_[loc + 6] = bitpatterns[r >> 2 & 0x03];
        out_buffer_[loc + 7] = bitpatterns[r & 0x03];

        out_buffer_[loc + 8] = bitpatterns[b >> 6 & 0x03];
        out_buffer_[loc + 9] = bitpatterns[b >> 4 & 0x03];
        out_buffer_[loc + 10] = bitpatterns[b >> 2 & 0x03];
        out_buffer_[loc + 11] = bitpatterns[b & 0x03];
    }

    i2s_channel_enable(this->channelHandle);
    i2s_channel_write(this->channelHandle, out_buffer_.data(),
                      size_buffer_, &bytes_written, portMAX_DELAY);
    i2s_channel_write(this->channelHandle, off_buffer_.data(),
                      ZERO_BUFFER, &bytes_written, portMAX_DELAY);
    i2s_channel_disable(this->channelHandle);

    return ESP_OK;
}

esp_err_t WS2812B::set_pixel(uint16_t index, const CRGB& c)
{
    return set_pixel(index, c.r, c.g, c.b);
}

esp_err_t WS2812B::set_all(const CRGB& c)
{
    for (uint16_t i = 0; i < led_count_; i++)
    {
        set_pixel(i, c);
    }
    return ESP_OK;
}

esp_err_t WS2812B::fill_palette(const CRGBPalette16& pal,
                                uint8_t start,
                                uint8_t step,
                                TBlendType blend)
{
    uint8_t index = start;

    for (uint16_t i = 0; i < led_count_; i++)
    {
        CRGB c = pal.getColor(index, blend);
        set_pixel(i, c);
        index += step;
    }

    return ESP_OK;
}

esp_err_t WS2812B::fill_solid(const CRGB& color)
{
    for (uint16_t i = 0; i < led_count_; i++)
    {
        set_pixel(i, color);
    }
    return ESP_OK;
}

esp_err_t WS2812B::fill_rainbow(uint8_t initial_hue, uint8_t delta_hue)
{
    uint8_t hue = initial_hue;

    for (uint16_t i = 0; i < led_count_; i++)
    {
        CHSV hsv(hue, 255, 255);
        CRGB rgb = hsv2rgb(hsv);
        set_pixel(i, rgb);
        hue += delta_hue;
    }

    return ESP_OK;
}

esp_err_t WS2812B::fadeToBlackBy(uint8_t amount)
{
    uint8_t scale = 255 - amount;

    for (uint16_t i = 0; i < led_count_; i++)
    {
        uint8_t g = buffer_[i * 3 + 0];
        uint8_t r = buffer_[i * 3 + 1];
        uint8_t b = buffer_[i * 3 + 2];

        r = (static_cast<uint16_t>(r) * scale) >> 8;
        g = (static_cast<uint16_t>(g) * scale) >> 8;
        b = (static_cast<uint16_t>(b) * scale) >> 8;

        buffer_[i * 3 + 0] = g;
        buffer_[i * 3 + 1] = r;
        buffer_[i * 3 + 2] = b;
    }

    return ESP_OK;
}

esp_err_t WS2812B::nscale8(uint8_t scale)
{
    for (uint16_t i = 0; i < led_count_; i++)
    {
        uint8_t g = buffer_[i * 3 + 0];
        uint8_t r = buffer_[i * 3 + 1];
        uint8_t b = buffer_[i * 3 + 2];

        r = (static_cast<uint16_t>(r) * scale) >> 8;
        g = (static_cast<uint16_t>(g) * scale) >> 8;
        b = (static_cast<uint16_t>(b) * scale) >> 8;

        buffer_[i * 3 + 0] = g;
        buffer_[i * 3 + 1] = r;
        buffer_[i * 3 + 2] = b;
    }

    return ESP_OK;
}
