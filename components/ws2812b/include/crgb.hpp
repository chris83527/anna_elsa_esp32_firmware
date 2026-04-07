#pragma once

#ifndef __CRGB_H__
#define __CRGB_H__

struct CRGB
{
    uint8_t r, g, b;

    CRGB() : r(0), g(0), b(0)
    {
    }

    CRGB(uint8_t r_, uint8_t g_, uint8_t b_) : r(r_), g(g_), b(b_)
    {
    }

};


// Basic
namespace Colour
{
    static const CRGB Black = CRGB(0, 0, 0);
    static const CRGB White = CRGB(255, 255, 255);
    static const CRGB Red = CRGB(255, 0, 0);
    static const CRGB Green = CRGB(0, 255, 0);
    static const CRGB Blue = CRGB(0, 0, 255);
    static const CRGB Yellow = CRGB(255, 255, 0);
    static const CRGB Cyan = CRGB(0, 255, 255);
    static const CRGB Magenta = CRGB(255, 0, 255);
    static const CRGB Orange = CRGB(255, 165, 0);
    static const CRGB Purple = CRGB(128, 0, 128);
    static const CRGB Pink = CRGB(255, 105, 180);

    // Greyscale
    static const CRGB VeryDarkGray = CRGB(64, 64, 64);
    static const CRGB DarkGray = CRGB(96, 96, 96);
    static const CRGB Gray = CRGB(128, 128, 128);
    static const CRGB LightGray = CRGB(192, 192, 192);
    static const CRGB VeryLightGray = CRGB(224, 224, 224);


    // Extended
    static const CRGB Aqua = CRGB(0, 255, 255);
    static const CRGB Aquamarine = CRGB(127, 255, 212);
    static const CRGB Azure = CRGB(240, 255, 255);
    static const CRGB Beige = CRGB(245, 245, 220);
    static const CRGB Coral = CRGB(255, 127, 80);
    static const CRGB Crimson = CRGB(220, 20, 60);
    static const CRGB Gold = CRGB(255, 215, 0);
    static const CRGB Goldenrod = CRGB(218, 165, 32);
    static const CRGB HotPink = CRGB(255, 105, 180);
    static const CRGB Indigo = CRGB(75, 0, 130);
    static const CRGB Lavender = CRGB(230, 230, 250);
    static const CRGB Lime = CRGB(0, 255, 0);
    static const CRGB Maroon = CRGB(128, 0, 0);
    static const CRGB Navy = CRGB(0, 0, 128);
    static const CRGB Olive = CRGB(128, 128, 0);
    static const CRGB OrangeRed = CRGB(255, 69, 0);
    static const CRGB Orchid = CRGB(218, 112, 214);
    static const CRGB Salmon = CRGB(250, 128, 114);
    static const CRGB SeaGreen = CRGB(46, 139, 87);
    static const CRGB SkyBlue = CRGB(135, 206, 235);
    static const CRGB SlateBlue = CRGB(106, 90, 205);
    static const CRGB SpringGreen = CRGB(0, 255, 127);
    static const CRGB Teal = CRGB(0, 128, 128);
    static const CRGB Tomato = CRGB(255, 99, 71);
    static const CRGB Turquoise = CRGB(64, 224, 208);
    static const CRGB Violet = CRGB(238, 130, 238);
    static const CRGB Wheat = CRGB(245, 222, 179);
}

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

#endif