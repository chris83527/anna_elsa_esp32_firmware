#pragma once
#include "crgb.h"

enum TBlendType : uint8_t
{
    NOBLEND = 0,
    LINEARBLEND = 1
};

struct CRGBPalette16 {
    CRGB colors[16];

    CRGBPalette16() {
        for (auto &c : colors) c = CRGB::Black;
    }

    CRGBPalette16(std::initializer_list<CRGB> list) {
        int i = 0;
        for (auto &c : list) {
            if (i < 16) colors[i++] = c;
        }
        for (; i < 16; i++) colors[i] = CRGB::Black;
    }

    CRGB getColor(uint8_t index, TBlendType blend = LINEARBLEND) const {
        uint8_t idx1 = index >> 4;       // 0–15
        uint8_t idx2 = (idx1 + 1) & 0x0F;
        uint8_t frac = index & 0x0F;     // 0–15

        const CRGB &c1 = colors[idx1];
        const CRGB &c2 = colors[idx2];

        if (blend == NOBLEND) {
            return c1;
        }

        // Linear blend
        uint8_t r = c1.r + ((c2.r - c1.r) * frac) / 16;
        uint8_t g = c1.g + ((c2.g - c1.g) * frac) / 16;
        uint8_t b = c1.b + ((c2.b - c1.b) * frac) / 16;

        return CRGB(r, g, b);
    }
};

// Built‑in palettes
static const CRGBPalette16 RainbowColors_p = {
    CRGB(255,0,0), CRGB(255,127,0), CRGB(255,255,0), CRGB(0,255,0),
    CRGB(0,255,255), CRGB(0,0,255), CRGB(139,0,255), CRGB(255,0,255),
    CRGB(255,0,0), CRGB(255,127,0), CRGB(255,255,0), CRGB(0,255,0),
    CRGB(0,255,255), CRGB(0,0,255), CRGB(139,0,255), CRGB(255,0,255)
};

static const CRGBPalette16 PartyColors_p = {
    CRGB(255,0,255), CRGB(0,255,255), CRGB(255,255,0), CRGB(0,255,0),
    CRGB(0,0,255), CRGB(255,0,0), CRGB(255,127,0), CRGB(127,0,255),
    CRGB(255,0,255), CRGB(0,255,255), CRGB(255,255,0), CRGB(0,255,0),
    CRGB(0,0,255), CRGB(255,0,0), CRGB(255,127,0), CRGB(127,0,255)
};

static const CRGBPalette16 CloudColors_p = {
    CRGB(255, 255, 255),   // White
    CRGB(224, 224, 255),   // Soft blue‑white
    CRGB(192, 192, 255),   // Light lavender‑blue
    CRGB(160, 160, 255),   // Pale periwinkle
    CRGB(128, 128, 255),   // Soft blue
    CRGB(160, 160, 255),   // Pale periwinkle
    CRGB(192, 192, 255),   // Light lavender‑blue
    CRGB(224, 224, 255),   // Soft blue‑white
    CRGB(255, 255, 255),   // White
    CRGB(224, 255, 255),   // Soft aqua‑white
    CRGB(192, 255, 255),   // Light aqua
    CRGB(160, 255, 255),   // Pale cyan
    CRGB(128, 255, 255),   // Soft cyan
    CRGB(160, 255, 255),   // Pale cyan
    CRGB(192, 255, 255),   // Light aqua
    CRGB(224, 255, 255)    // Soft aqua‑white
};

static const CRGBPalette16 RainbowStripeColors_p = {
    CRGB(255,   0,   0),   // Red
    CRGB(255,   0,   0),   // Red (repeat for stripe)
    CRGB(255, 127,   0),   // Orange
    CRGB(255, 127,   0),   // Orange
    CRGB(255, 255,   0),   // Yellow
    CRGB(255, 255,   0),   // Yellow
    CRGB(0,   255,   0),   // Green
    CRGB(0,   255,   0),   // Green
    CRGB(0,   255, 255),   // Cyan
    CRGB(0,   255, 255),   // Cyan
    CRGB(0,     0, 255),   // Blue
    CRGB(0,     0, 255),   // Blue
    CRGB(255,   0, 255),   // Magenta
    CRGB(255,   0, 255),   // Magenta
    CRGB(255,   0,   0),   // Red (wrap)
    CRGB(255,   0,   0)    // Red (wrap)
};

inline CRGB colorFromPalette(
    const CRGBPalette16 &pal,
    uint8_t index,
    uint8_t brightness = 255,
    TBlendType blend = LINEARBLEND)
{
    // Get blended or unblended color
    CRGB c = pal.getColor(index, blend);

    // Apply brightness scaling (FastLED nscale8)
    if (brightness != 255) {
        c.r = (uint16_t(c.r) * brightness) >> 8;
        c.g = (uint16_t(c.g) * brightness) >> 8;
        c.b = (uint16_t(c.b) * brightness) >> 8;
    }

    return c;
}
