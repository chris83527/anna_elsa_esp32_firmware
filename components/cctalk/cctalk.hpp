#pragma once

#include <cstdint>
#include <vector>
#include <mutex>
#include <chrono>
#include "cctalk.hpp"
#include "cctalk_error.hpp"

class ICctalkUart {
public:
    virtual ~ICctalkUart() = default;

    virtual int write(const uint8_t* data,
                      std::size_t len,
                      std::chrono::milliseconds timeout) = 0;

    virtual int read(uint8_t* data,
                     std::size_t len,
                     std::chrono::milliseconds timeout) = 0;
};

struct CctalkFrame {
    uint8_t destination = 0;
    uint8_t data_length = 0;
    uint8_t source = 1;
    uint8_t header = 0;
    std::vector<uint8_t> data;
    uint8_t checksum = 0;

    void computeChecksum() {
        uint16_t sum = 0;
        sum += destination;
        sum += data_length;
        sum += source;
        sum += header;
        for (auto b : data) sum += b;
        checksum = static_cast<uint8_t>(-static_cast<int8_t>(sum & 0xFF));
    }

    bool validateChecksum() const {
        uint16_t sum = 0;
        sum += destination;
        sum += data_length;
        sum += source;
        sum += header;
        for (auto b : data) sum += b;
        sum += checksum;
        return (sum & 0xFF) == 0;
    }
};

class CctalkBus {
public:
    explicit CctalkBus(ICctalkUart& uart, uint8_t host_address = 1)
        : uart_(uart), host_address_(host_address) {}

    CctalkError send(const CctalkFrame& frame,
                     std::chrono::milliseconds timeout);

    CctalkError readFrame(CctalkFrame& frame,
                          std::chrono::milliseconds timeout);

    CctalkError sendAndReceive(const CctalkFrame& request,
                               CctalkFrame& response,
                               std::chrono::milliseconds timeout);

private:
    ICctalkUart& uart_;
    uint8_t host_address_;
    std::mutex mutex_;

    CctalkError writeFrameLocked(const CctalkFrame& frame,
                                 std::chrono::milliseconds timeout);

    CctalkError readFrameLocked(CctalkFrame& frame,
                                std::chrono::milliseconds timeout);
};
