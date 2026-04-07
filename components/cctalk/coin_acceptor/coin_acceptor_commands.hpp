#pragma once
#include <algorithm>
#include "cctalk.hpp"
#include "cctalk_headers.hpp"
#include "coin_acceptor_types.hpp"

struct ReqGetCategoryId {};
struct RspGetCategoryId {
    std::string category;
};

inline CctalkError cctalk_get_category_id(CctalkBus& bus,
                                          std::uint8_t host,
                                          std::uint8_t device,
                                          RspGetCategoryId& out,
                                          std::chrono::milliseconds timeout)
{
    CctalkFrame req;
    req.destination = device;
    req.source      = host;
    req.header      = static_cast<std::uint8_t>(HDR_REQUEST_EQUIPMENT_CATEGORY_ID);
    req.data.clear();

    CctalkFrame resp;
    auto err = bus.sendAndReceive(req, resp, timeout);
    if (err != CctalkError::OK) return err;

    out.category.assign(resp.data.begin(), resp.data.end());
    return CctalkError::OK;
}

// Channel values (header 184)
struct ReqGetChannelValues {};
struct RspGetChannelValues {
    std::vector<CoinChannelInfo> channels;
};

inline CctalkError cctalk_get_channel_values(CctalkBus& bus,
                                             std::uint8_t host,
                                             std::uint8_t device,
                                             RspGetChannelValues& out,
                                             std::chrono::milliseconds timeout)
{
    CctalkFrame req;
    req.destination = device;
    req.source      = host;
    req.header      = static_cast<std::uint8_t>(HDR_REQUEST_COIN_ID);
    req.data.clear();

    CctalkFrame resp;
    auto err = bus.sendAndReceive(req, resp, timeout);
    if (err != CctalkError::OK) return err;

    out.channels.clear();
    // Example interpretation: [channel, value_byte] pairs
    for (std::size_t i = 0; i + 1 < resp.data.size(); i += 2) {
        CoinChannelInfo info;
        info.channel = resp.data[i];
        info.value   = static_cast<std::uint16_t>(resp.data[i+1]) * 5; // device-specific scaling
        out.channels.push_back(info);
    }
    return CctalkError::OK;
}

// Modify inhibit status (header 231)
struct ReqSetInhibitMask {
    std::uint8_t low_mask;   // channels 1–8
    std::uint8_t high_mask;  // channels 9–16
};

inline CctalkError cctalk_set_inhibit_mask(CctalkBus& bus,
                                           std::uint8_t host,
                                           std::uint8_t device,
                                           const ReqSetInhibitMask& reqMask,
                                           std::chrono::milliseconds timeout)
{
    CctalkFrame req;
    req.destination = device;
    req.source      = host;
    req.header      = static_cast<std::uint8_t>(HDR_MODIFY_INHIBIT_STATUS);
    req.data = { reqMask.low_mask, reqMask.high_mask };

    return bus.send(req, timeout);
}

// Buffered credit events (header 159)
struct ReqReadBufferedCreditEvents {};
struct RspReadBufferedCreditEvents {
    std::vector<CoinEvent> events;
};

inline CctalkError cctalk_read_buffered_credit_events(
    CctalkBus& bus,
    std::uint8_t host,
    std::uint8_t device,
    const std::vector<CoinChannelInfo>& channel_table,
    RspReadBufferedCreditEvents& out,
    std::chrono::milliseconds timeout)
{
    CctalkFrame req;
    req.destination = device;
    req.source      = host;
    req.header      = static_cast<std::uint8_t>(HDR_READ_BUFFERED_CREDIT_OR_ERROR_CODES);
    req.data.clear();

    CctalkFrame resp;
    auto err = bus.sendAndReceive(req, resp, timeout);
    if (err != CctalkError::OK) return err;

    if (resp.data.size() % 2 != 0) return CctalkError::MalformedFrame;

    out.events.clear();
    for (std::size_t i = 0; i < resp.data.size(); i += 2) {
        CoinEvent ev;
        ev.event_counter = resp.data[i];
        ev.channel       = resp.data[i+1];

        auto it = std::find_if(channel_table.begin(), channel_table.end(),
                               [&](const CoinChannelInfo& c){ return c.channel == ev.channel; });
        ev.coin_value = (it != channel_table.end()) ? it->value : 0;
        ev.routing    = (ev.channel > 0) ? CoinRouting::Accepted : CoinRouting::Rejected;

        out.events.push_back(ev);
    }
    return CctalkError::OK;
}
