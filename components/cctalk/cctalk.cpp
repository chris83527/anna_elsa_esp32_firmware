// cctalk.cpp
#include <vector>

#include "cctalk.hpp"
#include "cctalk_device_facade.hpp"
#include "esp_log.h"

const char* TAG = "CctalkBus";

CctalkError CctalkBus::writeFrameLocked(const CctalkFrame& frame,
                                        std::chrono::milliseconds timeout)
{
    CctalkFrame tmp = frame;
    tmp.data_length = static_cast<uint8_t>(tmp.data.size());
    tmp.computeChecksum();

    uint8_t totalSize = (5 + tmp.data.size());
    std::vector<uint8_t> buf;
    buf.reserve(totalSize);

    buf.push_back(tmp.destination);
    buf.push_back(tmp.data_length);
    buf.push_back(tmp.source);
    buf.push_back(tmp.header);
    buf.insert(buf.end(), tmp.data.begin(), tmp.data.end());
    buf.push_back(tmp.checksum);

    int written = uart_.write(buf.data(), buf.size(), timeout);
    if (written < 0)
    {
        ESP_LOGE(TAG, "Failed to send request: %d", CctalkError::UartError);
        return CctalkError::UartError;
    }
    ESP_LOGI(TAG, "%d bytes of %d written", written, buf.size());
    if (written != totalSize)
    {
        ESP_LOGE(TAG, "Failed to send request: %d", CctalkError::Timeout);
        return CctalkError::Timeout;
    }

    // Read back the echo and ignore it
    std::vector<uint8_t> readBuf;
    readBuf.reserve(totalSize);
    int bytesRead = uart_.read(readBuf.data(), totalSize, timeout);
    // These should be equal because it is an echo of the data sent
    if (readBuf != buf)
    {
        ESP_LOGE(TAG, "Failed to read loopback data. Wrote %d bytes, but received %d bytes ", written, bytesRead);
        return CctalkError::MalformedFrame;
    }

    return CctalkError::OK;
}

CctalkError CctalkBus::readFrameLocked(CctalkFrame& frame,
                                       std::chrono::milliseconds timeout)
{
    uint8_t header_bytes[4];
    int r = uart_.read(header_bytes, 4, timeout);
    if (r < 0) return CctalkError::UartError;
    if (r != 4) return CctalkError::Timeout;

    frame.destination = header_bytes[0];
    frame.data_length = header_bytes[1];
    frame.source = header_bytes[2];
    frame.header = header_bytes[3];

    uint8_t to_read = frame.data_length + 1; // The additional byte (+1) is for the checksum
    std::vector<uint8_t> tail(to_read);
    r = uart_.read(tail.data(), to_read, timeout);
    ESP_LOGI(TAG, "%d bytes read", r);
    if (r < 0) return CctalkError::UartError;
    if (r != to_read) return CctalkError::Timeout;

    frame.data.assign(tail.begin(), tail.begin() + frame.data_length);
    frame.checksum = tail.back();

    if (!frame.validateChecksum()) return CctalkError::BadChecksum;
    return CctalkError::OK;
}

CctalkError CctalkBus::send(const CctalkFrame& frame,
                            std::chrono::milliseconds timeout)
{
    std::lock_guard lock(mutex_);
    return writeFrameLocked(frame, timeout);
}

CctalkError CctalkBus::readFrame(CctalkFrame& frame,
                                 std::chrono::milliseconds timeout)
{
    std::lock_guard lock(mutex_);
    return readFrameLocked(frame, timeout);
}

CctalkError CctalkBus::sendAndReceive(const CctalkFrame& request,
                                      CctalkFrame& response,
                                      std::chrono::milliseconds timeout)
{
    std::lock_guard lock(mutex_);

    auto err = writeFrameLocked(request, timeout);
    if (err != CctalkError::OK) return err;

    err = readFrameLocked(response, timeout);
    if (err != CctalkError::OK) return err;

    if (response.source != request.destination)
        return CctalkError::UnexpectedSource;
    if (response.destination != host_address_)
        return CctalkError::UnexpectedDestination;

    return CctalkError::OK;
}
