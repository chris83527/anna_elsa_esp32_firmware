//
// Created by chris on 06.04.26.
//
#pragma once
#include <map>
#include <cstdint>

enum class CoinRouting : uint8_t {
    Rejected = 0,
    Accepted = 1,
    Unknown = 2,
};

inline std::string
ccCoinRejectionTypeGetDisplayableName(CoinRouting type)
{
    static std::map<CoinRouting, std::string> name_map = {
        {CoinRouting::Rejected, "Rejected"},
        {CoinRouting::Accepted, "Accepted"},
        {CoinRouting::Unknown, "Unknown"},
    };

    try
    {
        return name_map.at(type);
    }
    catch (const std::exception& e)
    {
        return "Unknown";
    }
}

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


