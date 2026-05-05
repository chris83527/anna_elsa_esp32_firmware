#pragma once

#include <chrono>
#include <vector>
#include <string>

#include "esp_log.h"

#include "cctalk.hpp"
#include "cctalk_error.hpp"
#include "cctalk_headers.hpp"

#include "cctalk_event_dispatcher_thread.hpp"
#include "cctalk_event_queue.hpp"

#include "coin_acceptor_types.hpp"
#include "hopper_types.hpp"

#include "coin_acceptor_commands.hpp"
#include "hopper_commands.hpp"
#include "cctalk_general_commands.hpp"

//
// High-level façade for ccTalk devices.
// This class hides all protocol details and exposes a clean API
// for your application logic.
//
class CctalkDeviceFacade
{
public:
    CctalkDeviceFacade(CctalkBus& bus,
                       std::uint8_t hostAddr,
                       std::uint8_t coinAcceptorAddr,
                       std::uint8_t hopperAddr)
        : bus_(bus),
          host_(hostAddr),
          coin_(coinAcceptorAddr),
          hopper_(hopperAddr)
    {
    }

    //
    // --- Generic Device Info ---
    //

    CctalkError resetDevice(std::uint8_t destination,
                            std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        return cctalk::general::cctalk_device_reset(bus_, host_, destination, timeout);
    }

    CctalkError getCategoryId(uint8_t destination, std::string& out,
                              std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        cctalk::general::RspGetCategoryId rsp;
        auto err = cctalk_get_category_id(bus_, host_, destination, rsp, timeout);
        if (err == CctalkError::OK)
        {
            out = rsp.category;
        }
        return err;
    }

    CctalkError getSerialNumber(uint8_t destination, std::string& out,
                                std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        CctalkFrame req;
        req.destination = destination;
        req.source = host_;
        req.header = static_cast<std::uint8_t>(CctalkHeaders::RequestSerialNumber);

        CctalkFrame resp;
        auto err = bus_.sendAndReceive(req, resp, timeout);
        if (err == CctalkError::OK)
        {
            decodeSerialNumber(resp.data, out);
        }
        return err;
    }

    CctalkError getSoftwareRevision(uint8_t destination, std::string& out,
                                    std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        CctalkFrame req;
        req.destination = destination;
        req.source = host_;
        req.header = static_cast<std::uint8_t>(CctalkHeaders::RequestSoftwareRevision);

        CctalkFrame resp;
        auto err = bus_.sendAndReceive(req, resp, timeout);
        if (err == CctalkError::OK)
        {
            out.assign(resp.data.begin(), resp.data.end());
        }
        return err;
    }

    CctalkError getBuildCode(uint8_t destination, std::string& out,
                             std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        CctalkFrame req;
        req.destination = destination;
        req.source = host_;
        req.header = static_cast<std::uint8_t>(CctalkHeaders::RequestBuildCode);

        CctalkFrame resp;
        auto err = bus_.sendAndReceive(req, resp, timeout);
        if (err == CctalkError::OK)
        {
            out.assign(resp.data.begin(), resp.data.end());
        }
        return err;
    }

    CctalkError getManufacturer(uint8_t destination, std::string& out,
                                std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        CctalkFrame req;
        req.destination = destination;
        req.source = host_;
        req.header = static_cast<std::uint8_t>(CctalkHeaders::RequestManufacturerId);

        CctalkFrame resp;
        auto err = bus_.sendAndReceive(req, resp, timeout);
        if (err == CctalkError::OK)
        {
            out.assign(resp.data.begin(), resp.data.end());
        }
        return err;
    }

    CctalkError requestCommsRevision(uint8_t destination, std::string& out,
                                     std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        CctalkFrame req;
        req.destination = destination;
        req.source = host_;
        req.header = static_cast<std::uint8_t>(CctalkHeaders::RequestCommsRevision);

        CctalkFrame resp;
        auto err = bus_.sendAndReceive(req, resp, timeout);
        if (err == CctalkError::OK)
        {
            toHex(resp.data, out);
        }
        return err;
    }

    //
    // --- Coin Acceptor ---
    //

    // Set individual inhibit status on all bills / coins. The specification
    // says that this is not needed for coin acceptors, but the practice shows
    // it is.
    CctalkError enableAllChannels(std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        cctalk::coin_validator::ReqSetInhibitMask mask{0xFF, 0xFF};
        return cctalk::coin_validator::cctalk_set_inhibit_mask(bus_, host_, coin_, mask, timeout);
    }

    CctalkError disableAllChannels(std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        cctalk::coin_validator::ReqSetInhibitMask mask{0x00, 0x00};
        return cctalk::coin_validator::cctalk_set_inhibit_mask(bus_, host_, coin_, mask, timeout);
    }

    CctalkError modifySorterOverrideStatus(uint8_t overrideStatus,
                                           std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        cctalk::coin_validator::ReqModifySorterOverrideStatus status{overrideStatus};
        return cctalk::coin_validator::cctalk_modify_sorter_override_status(bus_, host_, coin_, status, timeout);
    }

    /**
    * Modify inhibit status. specify which coins should be accepted or rejected. Example: 254 = coin ID 1 will be rejected and all others accepted
    *
    * @param lowMask Bit mask to specify which coins should be accepted or rejected. Example: 254 = coin ID 1 will be rejected and all others accepted
    * @param high Mask Bit mask to specify which coins should be accepted or rejected. Example: 254 = coin ID 1 will be rejected and all others accepted
    * @param timeout
    * @return An error if an error occurred, otherwise OK
    */
    CctalkError setInhibitMask(uint8_t lowMask, uint8_t highMask,
                               std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        cctalk::coin_validator::ReqSetInhibitMask mask{lowMask, highMask};
        return cctalk::coin_validator::cctalk_set_inhibit_mask(bus_, host_, coin_, mask, timeout);
    }

    /**
     * Modify the default sorter path of the coin validator
     *
     * @param coinId The ID of the coin whose path is to be defined
     * @param path The path to be used by default when a coin of the given ID is inserted
     * @param timeout
     * @return An error if an error occurred, otherwise OK
     */
    CctalkError modifySorterPaths(uint8_t coinId, uint8_t path,
                                  std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        cctalk::coin_validator::ReqModifySorterPaths req{coinId, path};
        return cctalk::coin_validator::cctalk_modify_sorter_paths(bus_, host_, coin_, req, timeout);
    }

    /**
     * Modify the default sorter path of the coin validator
     *
     * @param path The path to be used by default when a coin is inserted and accepted
     * @param timeout
     * @return An error if an error occurred, otherwise OK
     */
    CctalkError modifyDefaultSorterPath(uint8_t path,
                                        std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        cctalk::coin_validator::ReqModifyDefaultSorterPath req{path};
        return cctalk::coin_validator::cctalk_modify_default_sorter_path(bus_, host_, coin_, req, timeout);
    }

    CctalkError getChannelValues(std::vector<CoinChannelInfo>& out,
                                 std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        cctalk::coin_validator::RspGetChannelValues rsp;
        CcCountryScalingData countryScalingData{};
        auto err = cctalk::coin_validator::cctalk_get_channel_values(bus_, host_, coin_, countryScalingData, rsp, timeout);
        if (err == CctalkError::OK)
        {
            out = rsp.channels;
        }
        return err;
    }

    CctalkError readBufferedCreditEvents(
        const std::vector<CoinChannelInfo>& channelTable,
        std::vector<CoinEvent>& out,
        std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        // TODO: implement country scaling data. At the moment,
        //CcCountryScalingData scalingData;
        //cctalk::coin_validator:: req{channelTable};


        cctalk::coin_validator::RspReadBufferedCreditEvents rsp;
        auto err = cctalk_read_buffered_credit_events(
            bus_, host_, coin_, channelTable, rsp, timeout);

        if (err == CctalkError::OK)
        {
            ESP_LOGD(TAG, "Processing event number %d. Last processed event: %d", rsp.currentEventNumber, lastEventNumber);

            // When the device is first booted, the event log contains all zeroes.
            if (lastEventNumber == 0 && rsp.currentEventNumber == 0)
            {
                return CctalkError::OK;
            }

            // If the event counter suddenly drops to 0, this means that the device was--
            // probably reset. Probable loss of credits.
            if (lastEventNumber != 0 && rsp.currentEventNumber == 0)
            {
                ESP_LOGW(TAG, "The device appears to have been reset, possible loss of credit.");
                lastEventNumber = 0;
                return CctalkError::OK;
            }


            // If the event counters are equal, there are no new events.
            if (this->lastEventNumber == rsp.currentEventNumber)
            {
                // nothing
                return CctalkError::OK;
            }

            // If true, it means that we're processing the events that are in the device.
            // We should not give credit in this case, since that was probably processed
            // during previous application run.
            /*
            const bool processing_app_startup_events = (this->lastEventNumber == 0);
            if (processing_app_startup_events && rsp.currentEventNumber != 0)
            {
                ESP_LOGD(TAG, "last event number: %d, current event number: %d", this->lastEventNumber, rsp.currentEventNumber);
                ESP_LOGE(TAG, "Detected device that was up (and generating events) before the host startup; ignoring \"credit accepted\" events.");
                // just set lastEventNumber to current EventNumber
                this->lastEventNumber = rsp.currentEventNumber;
            }
            */


            int newEventCount = static_cast<int>(rsp.currentEventNumber) - static_cast<int>(this->lastEventNumber);
            if (newEventCount < 0)
            {
                newEventCount += 255;
            }

            // Any more than 5 events means that events have been lost
            if (newEventCount > 5)
            {
                ESP_LOGW(TAG, "Event counter difference %d is greater than 5. Credits probably lost.", newEventCount);
            }

            // don't output the whole list of returned events, only the number of new events otherwise
            // the credits will exponentially increase
            if (rsp.currentEventNumber > lastEventNumber) {
                out.assign(rsp.events.begin(), rsp.events.begin() + newEventCount);
                ESP_LOGD(TAG, "Found %d new event(s); processing from oldest to newest.", out.size());
            }

            this->lastEventNumber = rsp.currentEventNumber;
        }

        return err;
    }

    //
    // --- Hopper ---
    //

    CctalkError enableHopper(std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        return cctalk::hopper::cctalk_enable_hopper(bus_, host_, hopper_, timeout);
    }

    CctalkError requestCipherKey(cctalk::hopper::RspHopperCipherKey& out,
                                 std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        CctalkError ret =  cctalk::hopper::cctalk_hopper_request_cipher_key(bus_, host_, hopper_, out, timeout);

        if (ret == CctalkError::OK)
        {
            ESP_LOGI(TAG, "requestCipherKey called. Response: %3d %3d %3d %3d %3d %3d %3d %3d", out.cipher[0], out.cipher[1], out.cipher[2], out.cipher[3], out.cipher[4], out.cipher[5], out.cipher[6], out.cipher[7]);
        } else
        {
            ESP_LOGE(TAG, "Got non-ok return code: %d", ret);
        }
        return ret;
    }

    CctalkError hopperPayout(std::uint8_t coins, std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        cctalk::hopper::RspHopperCipherKey out{};
        CctalkError err = requestCipherKey(out);

        if (err == CctalkError::OK)
        {
            ESP_LOGI(TAG, "Calling request hopper payout for %d coins", coins);
            cctalk::hopper::ReqHopperPayout req{
                out.cipher,
                   coins
            };
            return cctalk_hopper_payout(bus_, host_, hopper_, req, timeout);
        }

        return err;
    }

    CctalkError getHopperStatus(HopperStatus& out,
                                std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        cctalk::hopper::RspHopperStatus rsp;
        auto err = cctalk_hopper_get_status(bus_, host_, hopper_, rsp, timeout);
        if (err == CctalkError::OK)
        {
            out = rsp.status;
        }
        return err;
    }

    CctalkError getHopperLevel(std::uint8_t& out,
                               std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        cctalk::hopper::RspHopperLevel rsp;
        auto err = cctalk_hopper_get_level(bus_, host_, hopper_, rsp, timeout);
        if (err == CctalkError::OK)
        {
            out = rsp.level;
        }
        return err;
    }

private:
    CctalkBus& bus_;
    std::uint8_t host_;
    std::uint8_t coin_;
    std::uint8_t hopper_;

    const char* TAG = "cctalk_device_facade";

    uint8_t lastEventNumber = 0;

    void toHex(const std::vector<uint8_t>& v, std::string& out)
    {
        static constexpr char hex[] = "0123456789ABCDEF";
        out.reserve(v.size() * 2);
        for (uint8_t b : v)
        {
            out.push_back(hex[b >> 4]);
            out.push_back(hex[b & 0x0F]);
        }
    }

    void decodeSerialNumber(const std::vector<uint8_t>& responseData, std::string& out)
    {
        if (responseData.size() == 3)
        {
            uint32_t serialNumber = 0;
            serialNumber = responseData.at(0) << 16 | responseData.at(1) << 8 | responseData.at(2);
            out = std::to_string(serialNumber);
        }
    }
};
