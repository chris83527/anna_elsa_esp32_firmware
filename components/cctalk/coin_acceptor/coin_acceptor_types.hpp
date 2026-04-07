//
// Created by chris on 06.04.26.
//
#pragma once
#include <cstdint>

enum class CoinRouting : uint8_t {
    Rejected = 0,
    Accepted = 1
};

struct CoinEvent {
    uint8_t event_counter;
    uint8_t channel;
    uint16_t coin_value;     // parsed from channel table
    CoinRouting routing;
};

struct CoinChannelInfo {
    uint8_t channel;
    uint16_t value;   // e.g., 10 = 10 cents
};

