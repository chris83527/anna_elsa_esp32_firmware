#pragma once

struct CRGB
{
    uint8_t r, g, b;

    CRGB() : r(0), g(0), b(0)
    {
    }

    CRGB(uint8_t r_, uint8_t g_, uint8_t b_) : r(r_), g(g_), b(b_)
    {
    }

    void nscale8(uint8_t scale)
    {
        r = (uint16_t(r) * scale) >> 8;
        g = (uint16_t(g) * scale) >> 8;
        b = (uint16_t(b) * scale) >> 8;
    }

    void fadeToBlackBy(uint8_t amount)
    {
        uint8_t scale = 255 - amount;
        nscale8(scale);
    }

    // --- Basic Colors ---
    static const CRGB Black;
    static const CRGB White;
    static const CRGB Red;
    static const CRGB Green;
    static const CRGB Blue;
    static const CRGB Yellow;
    static const CRGB Cyan;
    static const CRGB Magenta;
    static const CRGB Orange;
    static const CRGB Purple;
    static const CRGB Pink;

    // --- Grayscale ---
    static const CRGB VeryDarkGray;
    static const CRGB DarkGray;
    static const CRGB Gray;
    static const CRGB LightGray;
    static const CRGB VeryLightGray;


    // --- Extended Colors (FastLED style) ---
    static const CRGB Aqua;
    static const CRGB Aquamarine;
    static const CRGB Azure;
    static const CRGB Beige;
    static const CRGB Coral;
    static const CRGB Crimson;
    static const CRGB Gold;
    static const CRGB Goldenrod;
    static const CRGB HotPink;
    static const CRGB Indigo;
    static const CRGB Lavender;
    static const CRGB Lime;
    static const CRGB Maroon;
    static const CRGB Navy;
    static const CRGB Olive;
    static const CRGB OrangeRed;
    static const CRGB Orchid;
    static const CRGB Salmon;
    static const CRGB SeaGreen;
    static const CRGB SkyBlue;
    static const CRGB SlateBlue;
    static const CRGB SpringGreen;
    static const CRGB Teal;
    static const CRGB Tomato;
    static const CRGB Turquoise;
    static const CRGB Violet;
    static const CRGB Wheat;
};

// Basic
const CRGB CRGB::Black = CRGB(0, 0, 0);
const CRGB CRGB::White = CRGB(255, 255, 255);
const CRGB CRGB::Red = CRGB(255, 0, 0);
const CRGB CRGB::Green = CRGB(0, 255, 0);
const CRGB CRGB::Blue = CRGB(0, 0, 255);
const CRGB CRGB::Yellow = CRGB(255, 255, 0);
const CRGB CRGB::Cyan = CRGB(0, 255, 255);
const CRGB CRGB::Magenta = CRGB(255, 0, 255);
const CRGB CRGB::Orange = CRGB(255, 165, 0);
const CRGB CRGB::Purple = CRGB(128, 0, 128);
const CRGB CRGB::Pink = CRGB(255, 105, 180);

// Greyscale
const CRGB CRGB::VeryDarkGray = CRGB(64, 64, 64);
const CRGB CRGB::DarkGray = CRGB(96, 96, 96);
const CRGB CRGB::Gray = CRGB(128, 128, 128);
const CRGB CRGB::LightGray = CRGB(192, 192, 192);
const CRGB CRGB::VeryLightGray = CRGB(224, 224, 224);


// Extended
const CRGB CRGB::Aqua = CRGB(0, 255, 255);
const CRGB CRGB::Aquamarine = CRGB(127, 255, 212);
const CRGB CRGB::Azure = CRGB(240, 255, 255);
const CRGB CRGB::Beige = CRGB(245, 245, 220);
const CRGB CRGB::Coral = CRGB(255, 127, 80);
const CRGB CRGB::Crimson = CRGB(220, 20, 60);
const CRGB CRGB::Gold = CRGB(255, 215, 0);
const CRGB CRGB::Goldenrod = CRGB(218, 165, 32);
const CRGB CRGB::HotPink = CRGB(255, 105, 180);
const CRGB CRGB::Indigo = CRGB(75, 0, 130);
const CRGB CRGB::Lavender = CRGB(230, 230, 250);
const CRGB CRGB::Lime = CRGB(0, 255, 0);
const CRGB CRGB::Maroon = CRGB(128, 0, 0);
const CRGB CRGB::Navy = CRGB(0, 0, 128);
const CRGB CRGB::Olive = CRGB(128, 128, 0);
const CRGB CRGB::OrangeRed = CRGB(255, 69, 0);
const CRGB CRGB::Orchid = CRGB(218, 112, 214);
const CRGB CRGB::Salmon = CRGB(250, 128, 114);
const CRGB CRGB::SeaGreen = CRGB(46, 139, 87);
const CRGB CRGB::SkyBlue = CRGB(135, 206, 235);
const CRGB CRGB::SlateBlue = CRGB(106, 90, 205);
const CRGB CRGB::SpringGreen = CRGB(0, 255, 127);
const CRGB CRGB::Teal = CRGB(0, 128, 128);
const CRGB CRGB::Tomato = CRGB(255, 99, 71);
const CRGB CRGB::Turquoise = CRGB(64, 224, 208);
const CRGB CRGB::Violet = CRGB(238, 130, 238);
const CRGB CRGB::Wheat = CRGB(245, 222, 179);

struct CHSV
{
    uint8_t h, s, v;

    CHSV() : h(0), s(0), v(0)
    {
    }

    CHSV(uint8_t h_, uint8_t s_, uint8_t v_) : h(h_), s(s_), v(v_)
    {
    }

    // FastLED hue constants (0–255)
    static const uint8_t HUE_RED = 0;
    static const uint8_t HUE_ORANGE = 32;
    static const uint8_t HUE_YELLOW = 64;
    static const uint8_t HUE_GREEN = 96;
    static const uint8_t HUE_AQUA = 128;
    static const uint8_t HUE_BLUE = 160;
    static const uint8_t HUE_PURPLE = 192;
    static const uint8_t HUE_PINK = 224;



};

// Simple HSV → RGB conversion (FastLED‑style)
inline CRGB hsv2rgb(const CHSV& hsv)
{
    uint8_t h = hsv.h;
    uint8_t s = hsv.s;
    uint8_t v = hsv.v;

    if (s == 0) return CRGB(v, v, v);

    uint8_t region = h / 43;
    uint8_t remainder = (h - (region * 43)) * 6;

    uint8_t p = (v * (255 - s)) >> 8;
    uint8_t q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    uint8_t t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
    case 0: return CRGB(v, t, p);
    case 1: return CRGB(q, v, p);
    case 2: return CRGB(p, v, t);
    case 3: return CRGB(p, q, v);
    case 4: return CRGB(t, p, v);
    default: return CRGB(v, p, q);
    }
}

inline CRGB ColorWheel(uint8_t hue)
{
    // FastLED-style HSV wheel:
    // hue 0–255 → full rainbow
    CHSV hsv(hue, 255, 255);
    return hsv2rgb(hsv);
}