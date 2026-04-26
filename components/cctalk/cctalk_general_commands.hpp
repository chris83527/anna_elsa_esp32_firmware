//
// Created by chris on 22.04.26.
//

#pragma once

#include "cctalk.hpp"
#include "cctalk_headers.hpp"

#include <string>

namespace cctalk::general {

    // Reset Device (header 1)
    inline CctalkError cctalk_device_reset(CctalkBus& bus,
                                                   std::uint8_t host,
                                                   std::uint8_t device,
                                                   std::chrono::milliseconds timeout)
    {
        CctalkFrame req;
        req.destination = device;
        req.source      = host;
        req.header      = static_cast<std::uint8_t>(CctalkHeader::ResetDevice);
        req.data = {};

        return bus.send(req, timeout);
    }


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
        req.header      = static_cast<std::uint8_t>(CctalkHeader::RequestEquipmentCategoryId);
        req.data.clear();

        CctalkFrame resp;
        auto err = bus.sendAndReceive(req, resp, timeout);
        if (err != CctalkError::OK) return err;

        out.category.assign(resp.data.begin(), resp.data.end());
        return CctalkError::OK;
    }


}