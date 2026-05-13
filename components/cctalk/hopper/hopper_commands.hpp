#pragma once
#include "cctalk.hpp"
#include "cctalk_headers.hpp"
#include "hopper_types.hpp"

namespace cctalk::hopper
{

    inline CctalkError cctalk_enable_hopper(CctalkBus& bus, std::uint8_t host, std::uint8_t device, std::chrono::milliseconds timeout)
    {
        CctalkFrame req;
        req.destination = device;
        req.source = host;
        req.header = CctalkHeaders::EnableHopper;
        req.data.clear();

        req.data.push_back(165); // always send this byte

        return bus.send(req, timeout);
    }

    // Pay money out (header 164)
    struct ReqHopperPayout
    {
        std::vector<uint8_t> cipherKey;
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
        req.header = CctalkHeaders::PayMoneyOut;
        req.data.clear();
        req.data.assign(reqPay.cipherKey.begin(), reqPay.cipherKey.end());
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
        req.header = CctalkHeaders::RequestHopperStatus;
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
        req.header = CctalkHeaders::RequestPayoutHighLowStatus;
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
        std::vector<uint8_t> cipher;
    };
    inline CctalkError cctalk_hopper_request_cipher_key(CctalkBus& bus,
                                                        std::uint8_t host,
                                                        std::uint8_t device,
                                                        RspHopperCipherKey& out,
                                                        std::chrono::milliseconds timeout)
    {

        const char* TAG = "hopper_commands";

        CctalkFrame req;
        req.destination = device;
        req.source = host;
        req.header = CctalkHeaders::RequestCipherKey;
        req.data.clear();

        CctalkFrame resp;
        auto err = bus.sendAndReceive(req, resp, timeout);
        if (err != CctalkError::OK)
        {
            ESP_LOGE(TAG, "Received error from bus.sendAndReceive: %d", err);
            return err;
        }
        if (resp.data.empty())
        {
            ESP_LOGE(TAG, "Received MalformedFrame error");
            return CctalkError::MalformedFrame;
        }
        if (resp.data.size() != 8)
        {
            ESP_LOGE(TAG, "Did not receive 8 cipher bytes");
            return CctalkError::MalformedFrame;
        }

        out.cipher = resp.data;

        return CctalkError::OK;
    }
}
