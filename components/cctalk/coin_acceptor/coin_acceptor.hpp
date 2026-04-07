#pragma once

#include "cctalk.hpp"
#include "cctalk_devices.hpp"
#include "coin_acceptor_types.hpp"

class CctalkCoinAcceptor {
public:
    CctalkCoinAcceptor(CctalkBus& bus, uint8_t host_addr, uint8_t device_addr)
        : bus_(bus), host_addr_(host_addr), device_addr_(device_addr) {}

    // Read equipment category ID (header 245)
    CctalkError getCategoryId(std::string& out, std::chrono::milliseconds timeout) {
        CctalkFrame resp;
        auto err = cctalk_send_simple(bus_, host_addr_, device_addr_,
                                     245, {}, &resp, timeout);
        if (err != CctalkError::OK) return err;

        out.assign(resp.data.begin(), resp.data.end());

        return CctalkError::OK;
    }

    // Enable all channels (header 231: Modify inhibit status)
    // data: 2 bytes, bitmask of channels 1–8 and 9–16
    CctalkError enableAllChannels(std::chrono::milliseconds timeout) {
        std::vector<uint8_t> data = {0xFF, 0xFF};
        return cctalk_send_simple(bus_, host_addr_, device_addr_,
                                  231, data, nullptr, timeout);
    }

    // Disable all channels
    CctalkError disableAllChannels(std::chrono::milliseconds timeout) {
        std::vector<uint8_t> data = {0x00, 0x00};
        return cctalk_send_simple(bus_, host_addr_, device_addr_,
                                  231, data, nullptr, timeout);
    }

    CctalkError readBufferedCreditEvents(
    const std::vector<CoinChannelInfo>& channel_table,
    std::vector<CoinEvent>& events,
    std::chrono::milliseconds timeout)
    {
        CctalkFrame resp;
        auto err = cctalk_send_simple(bus_, host_addr_, device_addr_, 159, {}, &resp, timeout);
        if (err != CctalkError::OK) return err;

        if (resp.data.size() % 2 != 0)
            return CctalkError::MalformedFrame;

        events.clear();
        for (size_t i = 0; i < resp.data.size(); i += 2) {
            CoinEvent ev;
            ev.event_counter = resp.data[i];
            ev.channel       = resp.data[i+1];

            // Lookup coin value
            auto it = std::find_if(channel_table.begin(), channel_table.end(),
                                   [&](auto& c){ return c.channel == ev.channel; });
            ev.coin_value = (it != channel_table.end()) ? it->value : 0;

            // Routing: channels > 0 = accepted, 0 = rejected
            ev.routing = (ev.channel > 0) ? CoinRouting::Accepted : CoinRouting::Rejected;

            events.push_back(ev);
        }

        return CctalkError::OK;
    }

    CctalkError getChannelValues(std::vector<CoinChannelInfo>& out, std::chrono::milliseconds timeout) {
        CctalkFrame resp;
        auto err = cctalk_send_simple(bus_, host_addr_, device_addr_, 184, {}, &resp, timeout);
        if (err != CctalkError::OK) return err;

        out.clear();
        for (size_t i = 0; i + 1 < resp.data.size(); i += 2) {
            CoinChannelInfo info;
            info.channel = resp.data[i];
            info.value   = resp.data[i+1] * 5;  // example: device-specific scaling
            out.push_back(info);
        }
        return CctalkError::OK;
    }


private:
    CctalkBus& bus_;
    uint8_t host_addr_;
    uint8_t device_addr_;
};
