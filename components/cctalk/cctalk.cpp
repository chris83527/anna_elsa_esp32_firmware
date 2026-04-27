// cctalk.cpp
#include <vector>
#include <iostream>
#include <iterator>

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

    ESP_LOGD(TAG, "WRITE: dest: %d, data_length: %d, source: %d, header: %d", tmp.destination, tmp.data_length, tmp.source,
             tmp.header);


    int written = uart_.write(buf.data(), buf.size(), timeout);
    if (written < 0)
    {
        ESP_LOGE(TAG, "Failed to send request: %d", CctalkError::UartError);
        return CctalkError::UartError;
    }
    ESP_LOGD(TAG, "%d bytes of %d written", written, buf.size());
    if (written != totalSize)
    {
        ESP_LOGE(TAG, "Failed to send request: %d", CctalkError::Timeout);
        return CctalkError::Timeout;
    }


    // Read back the echo and ignore it
    std::vector<uint8_t> readBuf;
    readBuf.reserve(buf.size());
    int bytesRead = uart_.read(readBuf.data(), buf.size(), timeout);

    if (written != bytesRead)
    {
        ESP_LOGW(TAG, "Failed to read loopback data. Wrote %d bytes, but received %d bytes", written, bytesRead);
        return CctalkError::MalformedFrame;
    }

    // These should be equal because it is an echo of the data sent
    ESP_LOGD(TAG, "Data sent (1st 4 bytes): %d, %d, %d, %d. Data received (1st 4 bytes) %d, %d, %d, %d", buf[0], buf[1], buf[2], buf[3], readBuf[0], readBuf[1], readBuf[2], readBuf[3]);

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

    ESP_LOGD(TAG, "READ: dest: %d, data_length: %d, source: %d, header: %d", frame.destination, frame.data_length,
             frame.source, frame.header);

    uint8_t to_read = frame.data_length + 1; // The additional byte (+1) is for the checksum
    std::vector<uint8_t> tail(to_read);
    r = uart_.read(tail.data(), to_read, timeout);
    ESP_LOGD(TAG, "%d bytes read", r);
    if (r < 0) return CctalkError::UartError;
    if (r != to_read) return CctalkError::Timeout;

    frame.data.assign(tail.begin(), tail.begin() + frame.data_length);
    frame.checksum = tail.back();

    if (!frame.validateChecksum())
    {
        ESP_LOGE(TAG, "Invalid checksum");
        return CctalkError::BadChecksum;
    }
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

    // Write request
    auto err = writeFrameLocked(request, timeout);
    if (err != CctalkError::OK)
    {
        ESP_LOGE(TAG, "An error occurred calling writeFrameLocked.");
        return err;
    }

    // Read response
    err = readFrameLocked(response, timeout);
    if (err != CctalkError::OK)
    {
        ESP_LOGE(TAG, "An error occurred calling readFrameLocked.");
        return err;
    }

    if (response.source != request.destination)
    {
        ESP_LOGE(TAG, "Unexpected source: Expected %d, got %d", request.destination, response.source);
        return CctalkError::UnexpectedSource;
    }

    if (response.destination != host_address_)
    {
        ESP_LOGE(TAG, "Unexpected destination: Expected %d, got %d", host_address_, response.destination);
        return CctalkError::UnexpectedDestination;
    }

    if (response.header != 0)
    {
        ESP_LOGE(TAG, "Incorrect header: Expected 0 (ACK), got %d", response.header);
        return CctalkError::InvalidHeader;
    }

    // Debug
    //std::ranges::copy(response.data, std::ostream_iterator<char>(std::cout, ""));

    return CctalkError::OK;
}
