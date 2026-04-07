#pragma once

#include "cctalk.hpp"

inline CctalkError cctalk_send_simple(CctalkBus& bus,
                               uint8_t host_addr,
                               uint8_t device_addr,
                               uint8_t header,
                               const std::vector<uint8_t>& data,
                               CctalkFrame* response, std::chrono::milliseconds timeout)
{
    CctalkFrame req;
    req.destination = device_addr;
    req.source = host_addr;
    req.header = header;
    req.data = data;

    if (response) {
        return bus.sendAndReceive(req, *response, timeout);
    } else {
        return bus.send(req, timeout);
    }
}
