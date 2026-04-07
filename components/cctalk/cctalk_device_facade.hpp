#pragma once

#include <chrono>
#include <vector>
#include <string>

#include "cctalk.hpp"
#include "cctalk_error.hpp"
#include "cctalk_headers.hpp"

#include "cctalk_event_dispatcher_thread.hpp"
#include "cctalk_event_queue.hpp"

#include "coin_acceptor_types.hpp"
#include "hopper_types.hpp"

#include "coin_acceptor_commands.hpp"
#include "hopper_commands.hpp"

//
// High-level façade for ccTalk devices.
// This class hides all protocol details and exposes a clean API
// for your application logic.
//
class CctalkDeviceFacade {
public:
    CctalkDeviceFacade(CctalkBus& bus,
                       std::uint8_t hostAddr,
                       std::uint8_t coinAcceptorAddr,
                       std::uint8_t hopperAddr)
        : bus_(bus),
          host_(hostAddr),
          coin_(coinAcceptorAddr),
          hopper_(hopperAddr)
    {}

    //
    // --- Generic Device Info ---
    //

    CctalkError getCategoryId(std::string& out,
                              std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        RspGetCategoryId rsp;
        auto err = cctalk_get_category_id(bus_, host_, coin_, rsp, timeout);
        if (err == CctalkError::OK) {
            out = rsp.category;
        }
        return err;
    }

    CctalkError getSerialNumber(std::string& out,
                                std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        CctalkFrame req;
        req.destination = coin_;
        req.source      = host_;
        req.header      = static_cast<std::uint8_t>(HDR_REQUEST_SERIAL_NUMBER);

        CctalkFrame resp;
        auto err = bus_.sendAndReceive(req, resp, timeout);
        if (err == CctalkError::OK) {
            out.assign(resp.data.begin(), resp.data.end());
        }
        return err;
    }

    CctalkError getSoftwareRevision(std::string& out,
                                    std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        CctalkFrame req;
        req.destination = coin_;
        req.source      = host_;
        req.header      = static_cast<std::uint8_t>(HDR_REQUEST_SOFTWARE_REVISION);

        CctalkFrame resp;
        auto err = bus_.sendAndReceive(req, resp, timeout);
        if (err == CctalkError::OK) {
            out.assign(resp.data.begin(), resp.data.end());
        }
        return err;
    }

    //
    // --- Coin Acceptor ---
    //

    CctalkError enableAllChannels(std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        ReqSetInhibitMask mask{0xFF, 0xFF};
        return cctalk_set_inhibit_mask(bus_, host_, coin_, mask, timeout);
    }

    CctalkError disableAllChannels(std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        ReqSetInhibitMask mask{0x00, 0x00};
        return cctalk_set_inhibit_mask(bus_, host_, coin_, mask, timeout);
    }

    CctalkError getChannelValues(std::vector<CoinChannelInfo>& out,
                                 std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        RspGetChannelValues rsp;
        auto err = cctalk_get_channel_values(bus_, host_, coin_, rsp, timeout);
        if (err == CctalkError::OK) {
            out = rsp.channels;
        }
        return err;
    }

    CctalkError readBufferedCreditEvents(
        const std::vector<CoinChannelInfo>& channelTable,
        std::vector<CoinEvent>& out,
        std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        RspReadBufferedCreditEvents rsp;
        auto err = cctalk_read_buffered_credit_events(
            bus_, host_, coin_, channelTable, rsp, timeout);

        if (err == CctalkError::OK) {
            out = rsp.events;
        }
        return err;
    }

    //
    // --- Hopper ---
    //

    CctalkError hopperPayout(std::uint8_t coins,
                             std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        ReqHopperPayout req{coins};
        return cctalk_hopper_payout(bus_, host_, hopper_, req, timeout);
    }

    CctalkError getHopperStatus(HopperStatus& out,
                                std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        RspHopperStatus rsp;
        auto err = cctalk_hopper_get_status(bus_, host_, hopper_, rsp, timeout);
        if (err == CctalkError::OK) {
            out = rsp.status;
        }
        return err;
    }

    CctalkError getHopperLevel(std::uint8_t& out,
                               std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        RspHopperLevel rsp;
        auto err = cctalk_hopper_get_level(bus_, host_, hopper_, rsp, timeout);
        if (err == CctalkError::OK) {
            out = rsp.level;
        }
        return err;
    }

private:
    CctalkBus& bus_;
    std::uint8_t host_;
    std::uint8_t coin_;
    std::uint8_t hopper_;
};
