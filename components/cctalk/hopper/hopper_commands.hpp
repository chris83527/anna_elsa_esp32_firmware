#pragma once
#include "cctalk.hpp"
#include "cctalk_headers.hpp"
#include "hopper_types.hpp"

namespace cctalk::hopper
{
    /**
     * @brief Header 164 - Enable hopper
     *
     * Header 164 - Enable hopper
    * Transmitted data : [ enable code ]
    * Received data : ACK
    * [ enable code ]
    * 165 - enable hopper payout
    * not 165 - disable hopper payout
    * This command must be used to enable a hopper before paying out coins.
    * The value 165 is ‘A5’ in hex and ‘10100101’ in binary.
     *
     * @param bus The cctalk bus object used to send commands to the UART
     * @param host The ID of the host (usually 1)
     * @param device The cctalk device (coin validator, hopper)
     * @param timeout Operation timeout in milliseconds
     * @return A CctalkError structure containing further information pertaining to problems
     */
    inline CctalkError cctalk_enable_hopper(CctalkBus& bus, std::uint8_t host, std::uint8_t device,
                                            std::chrono::milliseconds timeout)
    {
        CctalkFrame req;
        req.destination = device;
        req.source = host;
        req.header = CctalkHeaders::EnableHopper;
        req.data.clear();

        req.data.push_back(165); // always send this byte (enable). Any other value means disable

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
    struct ReqPayoutHopperStatus
    {
    };

    struct RspPayoutHopperStatus
    {
        HopperPayoutStatus status;
    };


    inline CctalkError cctalk_hopper_get_status(CctalkBus& bus,
                                                std::uint8_t host,
                                                std::uint8_t device,
                                                RspPayoutHopperStatus& out,
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

        out.status.eventCounter = resp.data[0];
        out.status.coinsPaid = resp.data[1];
        out.status.coinsRemaining = resp.data[2];
        out.status.coinsUnpaid = resp.data[3];

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


    struct ReqTestHopper
    {
    };
    struct RspTestHopper
    {
        TestHopperStatus status;
    };

    /**
     * @brief Header 163 - Test hopper
     *
     *
     *
     * @param bus
     * @param host
     * @param device
     * @param out
     * @param timeout
     * @return
     */
    inline CctalkError cctalk_test_hopper(CctalkBus& bus,
                                          std::uint8_t host,
                                          std::uint8_t device,
                                          RspTestHopper& out,
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

        // SCH1
        out.status.statusRegister1 = resp.data[0];

        // SCH2
        if (resp.data.size() == 2)
        {
            out.status.statusRegister2 = resp.data[1];
        }

        // SCH3
        if (resp.data.size() == 3)
        {
            out.status.statusRegister3 = resp.data[2];
        }

        return CctalkError::OK;
    }
}
