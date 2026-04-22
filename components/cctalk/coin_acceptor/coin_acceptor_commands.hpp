#pragma once
#include <algorithm>
#include "cctalk.hpp"
#include "cctalk_headers.hpp"
#include "coin_acceptor_types.hpp"


namespace cctalk::coin_validator
{
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
        req.header      = static_cast<std::uint8_t>(CctalkHeader::RequestCoinId);
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
        req.header      = static_cast<std::uint8_t>(CctalkHeader::ModifyInhibitStatus);
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
        req.header      = static_cast<std::uint8_t>(CctalkHeader::ReadBufferedCreditOrErrorCodes);
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

            auto it = std::ranges::find_if(channel_table.begin(), channel_table.end(),
                                   [&](const CoinChannelInfo& c){ return c.channel == ev.channel; });
            ev.coin_value = (it != channel_table.end()) ? it->value : 0;
            ev.routing    = (ev.channel > 0) ? CoinRouting::Accepted : CoinRouting::Rejected;

            out.events.push_back(ev);
        }
        return CctalkError::OK;
    }

    // Modify sorter override status (header 222)
    struct ReqModifySorterOverrideStatus {
        std::uint8_t overrideStatus;   // channels 1–8
    };

    inline CctalkError cctalk_modify_sorter_override_status(CctalkBus& bus,
                                               std::uint8_t host,
                                               std::uint8_t device,
                                               const ReqModifySorterOverrideStatus& reqModifySorterOverrideStatus,
                                               std::chrono::milliseconds timeout)
    {
        CctalkFrame req;
        req.destination = device;
        req.source      = host;
        req.header      = static_cast<std::uint8_t>(CctalkHeader::ModifySorterOverrideStatus);
        req.data = { reqModifySorterOverrideStatus.overrideStatus };

        return bus.send(req, timeout);
    }

    // Modify default sorter path (header 189)
    struct ReqModifyDefaultSorterPath {
        std::uint8_t path;
    };

    inline CctalkError cctalk_modify_default_sorter_path(CctalkBus& bus,
                                               std::uint8_t host,
                                               std::uint8_t device,
                                               const ReqModifyDefaultSorterPath& reqModifyDefaultSorterPath,
                                               std::chrono::milliseconds timeout)
    {
        CctalkFrame req;
        req.destination = device;
        req.source      = host;
        req.header      = static_cast<std::uint8_t>(CctalkHeader::ModifyDefaultSorterPath);
        req.data = { reqModifyDefaultSorterPath.path };

        return bus.send(req, timeout);
    }

    // Modify modify sorter paths (header 210)
    struct ReqModifySorterPaths {
        std::uint8_t coin_id;
        std::uint8_t path;
    };

    inline CctalkError cctalk_modify_sorter_paths(CctalkBus& bus,
                                               std::uint8_t host,
                                               std::uint8_t device,
                                               const ReqModifySorterPaths& reqModifySorterPaths,
                                               std::chrono::milliseconds timeout)
    {
        CctalkFrame req;
        req.destination = device;
        req.source      = host;
        req.header      = static_cast<std::uint8_t>(CctalkHeader::ModifySorterPaths);
        req.data = { reqModifySorterPaths.coin_id, reqModifySorterPaths.path };

        return bus.send(req, timeout);
    }



}
