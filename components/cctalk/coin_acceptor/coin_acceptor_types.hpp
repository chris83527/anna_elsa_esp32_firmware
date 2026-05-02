//
// Created by chris on 06.04.26.
//
#pragma once
#include <map>
#include <cstdint>

enum class CoinRouting : uint8_t
{
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

struct CoinEvent
{
    uint8_t channel;
    uint8_t coin_id;
    uint16_t coin_value;
    CoinRouting routing;
};

struct CoinChannelInfo
{
    uint8_t channel;
    uint16_t value; // e.g., 10 = 10 cents
};

/*
     *  Get coin values according to coin code (cctalk spec Appendix 3 (2.1))
     */
inline uint64_t ccCoinValueCodeGetValue(const std::string& three_char_code, uint8_t& decimal_places)
{
    // coin code -> (coin_value, decimal_places).
    static std::map<std::string, std::pair<uint64_t, uint8_t>> value_map = {
        {
            "5m0",
            {5, 3}
        }, // 0.005
        {
            "10m",
            {1, 2}
        }, // 0.01
        {
            ".01",
            {1, 2}
        }, // 0.01
        {
            "20m",
            {2, 2}
        }, // 0.02
        {
            ".02",
            {2, 2}
        }, // 0.02
        {
            "25m",
            {25, 3}
        }, // 0.025
        {
            "50m",
            {5, 2}
        }, // 0.05
        {
            ".05",
            {5, 2}
        }, // 0.05
        {
            ".10",
            {1, 1}
        }, // 0.10
        {
            ".20",
            {2, 1}
        }, // 0.20
        {
            ".25",
            {25, 2}
        }, // 0.25
        {
            ".50",
            {5, 1}
        }, // 0.50
        {
            "001",
            {1, 0}
        }, // 1
        {
            "002",
            {1, 0}
        }, // 2
        {
            "2.5",
            {25, 1}
        }, // 2.5
        {
            "005",
            {5, 0}
        }, // 5
        {
            "010",
            {10, 0}
        }, // 10
        {
            "020",
            {20, 0}
        }, // 20
        {
            "025",
            {25, 0}
        }, // 25
        {
            "050",
            {50, 0}
        }, // 50
        {
            "100",
            {100, 0}
        }, // 100
        {
            "200",
            {200, 0}
        }, // 200
        {
            "250",
            {250, 0}
        }, // 250
        {
            "500",
            {500, 0}
        }, // 500
        {
            "1K0",
            {1000, 0}
        }, // 1 000
        {
            "2K0",
            {2000, 0}
        }, // 2 000
        {
            "2K5",
            {2500, 0}
        }, // 2 500
        {
            "5K0",
            {5000, 0}
        }, // 5 000
        {
            "10K",
            {10000, 0}
        }, // 10 000
        {
            "20K",
            {20000, 0}
        }, // 20 000
        {
            "25K",
            {25000, 0}
        }, // 25 000
        {
            "50K",
            {50000, 0}
        }, // 50 000
        {
            "M10",
            {100000, 0}
        }, // 100 000
        {
            "M20",
            {200000, 0}
        }, // 200 000
        {
            "M25",
            {250000, 0}
        }, // 250 000
        {
            "M50",
            {500000, 0}
        }, // 500 000
        {
            "1M0",
            {1000000, 0}
        }, // 1 000 000
        {
            "2M0",
            {2000000, 0}
        }, // 2 000 000
        {
            "2M5",
            {2500000, 0}
        }, // 2 500 000
        {
            "5M0",
            {5000000, 0}
        }, // 5 000 000
        {
            "10M",
            {10000000, 0}
        }, // 10 000 000
        {
            "20M",
            {20000000, 0}
        }, // 20 000 000
        {
            "25M",
            {25000000, 0}
        }, // 25 000 000
        {
            "50M",
            {50000000, 0}
        }, // 50 000 000
        {
            "G10",
            {100000000, 0}
        }, // 100 000 000
    };

    std::pair<uint64_t, uint8_t> value;
    try
    {
        value = value_map.at(three_char_code);
    }
    catch (const std::exception& e)
    {
        value = std::pair<uint64_t, uint8_t>(0, 0);
    }

    decimal_places = value.second;

    return value.first;
}

// Country scaling data, as returned by GetCountryScalingFactor command.
struct CcCountryScalingData
{
    /// Scaling factor from country scaling data.
    /// The bill identifier values should be
    /// multiplied by this to get cents.
    uint16_t scaling_factor = 1;

    /// Decimal places from country scaling data. 2 for USD (10^2 cents in USD)
    uint8_t decimal_places = 0;

    /// If country code is unsupported, this returns false.
    [[nodiscard]] bool isValid() const
    {
        return scaling_factor != 0 || decimal_places != 0;
    }
};


/// ccTalk coin / bill identifier, as returned by GetBillId and GetCoinId commands.

struct CcIdentifier
{
    /// std::map requirement
    CcIdentifier() = default;

    // Parse ID string and store the results
    explicit CcIdentifier(std::string arg_id_string) : id_string(std::move(arg_id_string))
    {
        if (arg_id_string.size() == 7)
        {
            // Bills
            country = arg_id_string.substr(0, 2);
            issue_code = arg_id_string.at(arg_id_string.size() - 1);

            value_code = std::stoull(arg_id_string.substr(2, 4), nullptr, 10);
        }
        else if (arg_id_string.size() == 6)
        {
            // Coins
            country = arg_id_string.substr(0, 2);
            issue_code = arg_id_string.at(arg_id_string.size() - 1);
            value_code = ccCoinValueCodeGetValue(arg_id_string.substr(2, 3), coin_decimals);
        }
        else
        {
            //DBG_ASSERT(0);
        }
    }

    /// Set country scaling data for bills and coins. For bills this can be retrieved
    /// from the device, while for coins it has to be predefined by us.

    void setCountryScalingData(CcCountryScalingData data)
    {
        country_scaling_data = data;
    }

    /// Get coin / bill value. The returned value should be divided by \c 10^divisor
    /// to get a value in country currency (e.g. USD).
    /// For coin acceptors the divisor is always 1.

    uint64_t getValue(uint64_t& divisor) const
    {
        divisor = country_scaling_data.decimal_places + coin_decimals;
        return value_code * country_scaling_data.scaling_factor;
    }


    std::string id_string;
    ///< Bill / coin identifier, e.g. "GE0005A" for the first ("A") issue of Georgian 5 lari (value code 0005).
    std::string country; ///< Country identifier, e.g. "GE".
    char issue_code = 0; ///< Issue code (A, B, C, ...), to differentiate various issues of the same-value coin.
    uint64_t value_code = 0; ///< Value code (before country scaling, if it's a bill).
    uint8_t coin_decimals = 0; ///< Value code should be divided by 10^coin_decimals to get the real value.

    CcCountryScalingData country_scaling_data; ///< Coin / bill scaling data
};

