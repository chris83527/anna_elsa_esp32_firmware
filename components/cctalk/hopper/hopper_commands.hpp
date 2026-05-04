#pragma once
#include "cctalk.hpp"
#include "cctalk_headers.hpp"
#include "hopper_types.hpp"

namespace cctalk::hopper
{
    // Pay money out (header 164)
    struct ReqHopperPayout
    {
        uint8_t cipherKey[8];
        std::uint8_t coins;
    };

    inline CctalkError cctalk_hopper_payout(CctalkBus& bus,
                                            std::uint8_t host,
                                            std::uint8_t device,
                                            const ReqHopperPayout& reqPay,
                                            std::chrono::milliseconds timeout)
    {
        CctalkFrame req;
        req.destination = device;
        req.source = host;
        req.header = static_cast<std::uint8_t>(CctalkHeader::PayMoneyOut);
        req.data.assign(std::begin(reqPay.cipherKey), std::end(reqPay.cipherKey));
        req.data.push_back(reqPay.coins);

        return bus.send(req, timeout);
    }

    // Hopper status (header 166)
    struct ReqHopperStatus
    {
    };

    struct RspHopperStatus
    {
        HopperStatus status;
    };


    inline CctalkError cctalk_hopper_get_status(CctalkBus& bus,
                                                std::uint8_t host,
                                                std::uint8_t device,
                                                RspHopperStatus& out,
                                                std::chrono::milliseconds timeout)
    {
        CctalkFrame req;
        req.destination = device;
        req.source = host;
        req.header = static_cast<std::uint8_t>(CctalkHeader::RequestHopperStatus);
        req.data.clear();

        CctalkFrame resp;
        auto err = bus.sendAndReceive(req, resp, timeout);
        if (err != CctalkError::OK) return err;
        if (resp.data.empty()) return CctalkError::MalformedFrame;

        std::uint8_t s = resp.data[0];
        out.status.raw_status = s;
        out.status.empty = (s & 0x01) != 0;
        out.status.jammed = (s & 0x02) != 0;
        out.status.motor_running = (s & 0x04) != 0;
        out.status.motor_timeout = (s & 0x08) != 0;
        out.status.sensor_blocked = (s & 0x10) != 0;

        return CctalkError::OK;
    }

    // Hopper coin value / level (header 184, reused)
    struct ReqHopperLevel
    {
    };

    struct RspHopperLevel
    {
        std::uint8_t level;
    };

    inline CctalkError cctalk_hopper_get_level(CctalkBus& bus,
                                               std::uint8_t host,
                                               std::uint8_t device,
                                               RspHopperLevel& out,
                                               std::chrono::milliseconds timeout)
    {
        CctalkFrame req;
        req.destination = device;
        req.source = host;
        req.header = static_cast<std::uint8_t>(CctalkHeader::RequestHopperCoinValue);
        req.data.clear();

        CctalkFrame resp;
        auto err = bus.sendAndReceive(req, resp, timeout);
        if (err != CctalkError::OK) return err;
        if (resp.data.empty()) return CctalkError::MalformedFrame;

        out.level = resp.data[0];
        return CctalkError::OK;
    }

    struct RspHopperCipherKey
    {
        std::uint8_t cipher[8];
    };

    inline CctalkError cctalk_hopper_request_cipher_key(CctalkBus& bus,
                                                        std::uint8_t host,
                                                        std::uint8_t device,
                                                        RspHopperCipherKey& out,
                                                        std::chrono::milliseconds timeout)
    {
        CctalkFrame req;
        req.destination = device;
        req.source = host;
        req.header = static_cast<std::uint8_t>(CctalkHeader::RequestCipherKey);
        req.data.clear();

        CctalkFrame resp;
        auto err = bus.sendAndReceive(req, resp, timeout);
        if (err != CctalkError::OK) return err;
        if (resp.data.empty()) return CctalkError::MalformedFrame;

        out.cipher[0] = resp.data[0];
        out.cipher[1] = resp.data[1];
        out.cipher[2] = resp.data[2];
        out.cipher[3] = resp.data[3];
        out.cipher[4] = resp.data[4];
        out.cipher[5] = resp.data[5];
        out.cipher[6] = resp.data[6];
        out.cipher[7] = resp.data[7];

        return CctalkError::OK;
    }
}
