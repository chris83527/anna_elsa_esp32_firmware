// random8.h
#pragma once

#include <cstdint>
#include "esp_random.h"

// FastLED‑style random helpers backed by ESP32 hardware RNG
namespace frand {

    // Return full 8‑bit random value (0–255)
    inline uint8_t random8() {
        return static_cast<uint8_t>(esp_random() & 0xFF);
    }

    // Return 0..max‑1
    inline uint8_t random8(uint8_t max) {
        // Multiply 8‑bit random by max, keep high byte
        return uint8_t((uint16_t(random8()) * max) >> 8);
    }

    // Return min..max‑1
    inline uint8_t random8(uint8_t min, uint8_t max) {
        return min + random8(uint8_t(max - min));
    }

    // 16‑bit versions
    inline uint16_t random16() {
        return static_cast<uint16_t>(esp_random() & 0xFFFF);
    }

    inline uint16_t random16(uint16_t max) {
        return uint16_t((uint32_t(random16()) * max) >> 16);
    }

    inline uint16_t random16(uint16_t min, uint16_t max) {
        return min + random16(uint16_t(max - min));
    }
}
