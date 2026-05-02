#pragma once
#include <algorithm>
#include "cctalk.hpp"
#include "cctalk_headers.hpp"
#include "coin_acceptor_types.hpp"


namespace cctalk::coin_validator
{
    constexpr const char* TAG = "coin_validator";

    // Channel values (header 184)
    struct ReqGetChannelValues
    {
    };

    struct RspGetChannelValues
    {
        std::vector<CoinChannelInfo> channels;
    };

    inline CctalkError cctalk_get_channel_values(CctalkBus& bus,
                                                 std::uint8_t host,
                                                 std::uint8_t device,
                                                 CcCountryScalingData& countryScalingData,
                                                 RspGetChannelValues& out,
                                                 std::chrono::milliseconds timeout)
    {
        CctalkFrame req;
        req.destination = device;
        req.source = host;
        req.header = static_cast<std::uint8_t>(CctalkHeader::RequestCoinId);
        req.data.clear();

        CctalkFrame resp;
        auto err = bus.sendAndReceive(req, resp, timeout);
        if (err != CctalkError::OK) return err;

        out.channels.clear();

        uint8_t maxPositions = 6; // Coin validator has up to 6 channels

        for (uint8_t pos = 1 ; pos <= maxPositions ; ++pos)
        {
            // Fetch coin / bill ID at position pos.
            req.data.push_back(pos);
            auto err = bus.sendAndReceive(req, resp, timeout);
            if (err != CctalkError::OK) return err;

            // Decode the data.
            // 6 dots mean empty by convention, but we've seen all-null too.
            std::string decodedData(resp.data.begin(), resp.data.end());
            if (decodedData.size() != 0 && decodedData != "......" && decodedData.at(0) != 0)
            {
                CcIdentifier identifier(decodedData);
                identifier.setCountryScalingData(countryScalingData);
                ESP_LOGD(TAG, "Adding coin identifier %s to position %d in shared_identifiers", identifier.id_string.c_str(), pos);
                uint64_t divisor = 1;
                CoinChannelInfo coinChannelInfo{};
                coinChannelInfo.channel = pos;
                coinChannelInfo.value = identifier.getValue(divisor);
                out.channels.push_back(coinChannelInfo);
            }
        }

        return CctalkError::OK;
    }

    // Modify inhibit status (header 231)
    struct ReqSetInhibitMask
    {
        std::uint8_t low_mask; // channels 1–8
        std::uint8_t high_mask; // channels 9–16
    };

    inline CctalkError cctalk_set_inhibit_mask(CctalkBus& bus,
                                               std::uint8_t host,
                                               std::uint8_t device,
                                               const ReqSetInhibitMask& reqMask,
                                               std::chrono::milliseconds timeout)
    {
        CctalkFrame req;
        req.destination = device;
        req.source = host;
        req.header = static_cast<std::uint8_t>(CctalkHeader::ModifyInhibitStatus);
        req.data = {reqMask.low_mask, reqMask.high_mask};

        return bus.send(req, timeout);
    }

    // Buffered credit events (header 159)
    struct ReqReadBufferedCreditEvents
    {
    };

    struct RspReadBufferedCreditEvents
    {
        uint8_t currentEventNumber;
        std::vector<CoinEvent> events;
    };


    /**
     * @brief Read Buffered Credit Events
     *
     * Coin acceptors use ReadBufferedCredit command.
     * Bill validators use ReadBufferedBillEvents command.
     * Both commands return data in approximately the same format.
     *
     * The response format is: [event_counter] [result 1A] [result 1B] [result 2A] [result 2B] ... [result 5B].
     * There are usually 5 events. 1A/1B is the newest one.
     * diff (event_counter, last_event_counter) indicates the number of results that are new. If > 5, it means data was lost.
     * [event_counter] == 0 means power-up or reset condition.
     * Note that [event_counter] wraps from 255 to 1, not 0.
     * [result A]: If in 1-255 range, it's credit (coin/bill position). If 0, see error code in [result B].
     * [result B]: If A is 0, B is error code, see CcCoinAcceptorEventCode / CcBillValidatorErrorCode.
     * If A is credit, B is sorter path (0 unsupported, 1-8 path number).
     *
     * @param bus
     * @param host The ID of the host device (usually 1)
     * @param device The ID of the slave device (2 for coin validator, 3 for hopper)
     * @param channel_table
     * @param out
     * @param timeout
     * @return
     */
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
        req.source = host;
        req.header = static_cast<std::uint8_t>(CctalkHeader::ReadBufferedCreditOrErrorCodes);
        req.data.clear();

        CctalkFrame resp;
        auto err = bus.sendAndReceive(req, resp, timeout);
        if (err != CctalkError::OK)
        {
            ESP_LOGE(TAG, "Got an error sending request");
            return err;
        }

        // Should receive 11 bytes
        if (resp.data.size() % 2 != 1)
        {

            ESP_LOGE(TAG, "Got a malformed frame. Received %d data bytes", resp.data.size());
            return CctalkError::MalformedFrame;
        }

        out.events.clear();

        out.currentEventNumber = resp.data[0];

        for (std::size_t i = 1; i < resp.data.size(); i += 2)
        {
            CoinEvent ev{};
            ev.coin_id = resp.data[i];
            ev.channel = resp.data[i + 1];

            ESP_LOGD(TAG, "Coin ID: %d. Error/Channel: %d", ev.coin_id, ev.channel);

            auto it = std::ranges::find_if(channel_table.begin(), channel_table.end(),
                                           [&](const CoinChannelInfo& c) { return c.channel == ev.channel; });
            ev.coin_value = (it != channel_table.end()) ? it->value : 0;
            ev.routing = (ev.channel > 0) ? CoinRouting::Accepted : CoinRouting::Rejected;

            out.events.push_back(ev);
        }
        return CctalkError::OK;
    }

    // Modify sorter override status (header 222)
    struct ReqModifySorterOverrideStatus
    {
        std::uint8_t overrideStatus; // channels 1–8
    };

    inline CctalkError cctalk_modify_sorter_override_status(CctalkBus& bus,
                                                            std::uint8_t host,
                                                            std::uint8_t device,
                                                            const ReqModifySorterOverrideStatus&
                                                            reqModifySorterOverrideStatus,
                                                            std::chrono::milliseconds timeout)
    {
        CctalkFrame req;
        req.destination = device;
        req.source = host;
        req.header = static_cast<std::uint8_t>(CctalkHeader::ModifySorterOverrideStatus);
        req.data = {reqModifySorterOverrideStatus.overrideStatus};

        return bus.send(req, timeout);
    }

    // Modify default sorter path (header 189)
    struct ReqModifyDefaultSorterPath
    {
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
        req.source = host;
        req.header = static_cast<std::uint8_t>(CctalkHeader::ModifyDefaultSorterPath);
        req.data = {reqModifyDefaultSorterPath.path};

        return bus.send(req, timeout);
    }

    // Modify modify sorter paths (header 210)
    struct ReqModifySorterPaths
    {
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
        req.source = host;
        req.header = static_cast<std::uint8_t>(CctalkHeader::ModifySorterPaths);
        req.data = {reqModifySorterPaths.coin_id, reqModifySorterPaths.path};

        return bus.send(req, timeout);
    }
}
