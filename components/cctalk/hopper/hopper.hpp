#pragma once

#include "cctalk.hpp"
#include "cctalk_devices.hpp"
#include "hopper_types.hpp"

class CctalkHopper
{
public:
    CctalkHopper(CctalkBus& bus, uint8_t host_addr, uint8_t device_addr)
        : bus_(bus), host_addr_(host_addr), device_addr_(device_addr)
    {
    }

    // Simple payout: header 164 (Pay money out)
    // data[0] = number of coins to pay
    CctalkError payout(uint8_t coins, std::chrono::milliseconds timeout)
    {
        std::vector<uint8_t> data = {coins};
        auto err = cctalk_send_simple(bus_, host_addr_, device_addr_,
                                      164, data, nullptr, timeout);

        if (err != CctalkError::OK) return err;

        return CctalkError::OK;
    }

    CctalkError getStatus(HopperStatus& out, std::chrono::milliseconds timeout)
    {
        CctalkFrame resp;
        auto err = cctalk_send_simple(bus_, host_addr_, device_addr_, 166, {}, &resp, timeout);
        if (err != CctalkError::OK) return err;

        if (resp.data.empty()) return CctalkError::MalformedFrame;

        uint8_t s = resp.data[0];
        out.raw_status = s;
        out.empty = s & 0x01;
        out.jammed = s & 0x02;
        out.motor_running = s & 0x04;
        out.motor_timeout = s & 0x08;
        out.sensor_blocked = s & 0x10;

        return CctalkError::OK;
    }


    // Example: read hopper level (header 184: Request hopper coin value / level)
    CctalkError getLevel(uint8_t& level, std::chrono::milliseconds timeout)
    {
        CctalkFrame resp;
        auto err = cctalk_send_simple(bus_, host_addr_, device_addr_,
                                      184, {}, &resp, timeout);

        if (err != CctalkError::OK) return err;

        if (resp.data.empty()) return CctalkError::MalformedFrame;

        level = resp.data[0];

        return CctalkError::OK;
    }

private:
    CctalkBus& bus_;
    uint8_t host_addr_;
    uint8_t device_addr_;
};
