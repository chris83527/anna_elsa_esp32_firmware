// cctalk.cpp
#include "cctalk.hpp"
#include "cctalk_device_facade.hpp"

CctalkError CctalkBus::writeFrameLocked(const CctalkFrame& frame,
                                        std::chrono::milliseconds timeout) {
    CctalkFrame tmp = frame;
    tmp.data_length = static_cast<uint8_t>(tmp.data.size());
    tmp.computeChecksum();

    std::vector<uint8_t> buf;
    buf.reserve(5 + tmp.data.size());
    buf.push_back(tmp.destination);
    buf.push_back(tmp.data_length);
    buf.push_back(tmp.source);
    buf.push_back(tmp.header);
    buf.insert(buf.end(), tmp.data.begin(), tmp.data.end());
    buf.push_back(tmp.checksum);

    int written = uart_.write(buf.data(), buf.size(), timeout);
    if (written < 0) return CctalkError::UartError;
    if (static_cast<std::size_t>(written) != buf.size()) return CctalkError::Timeout;
    return CctalkError::OK;
}

CctalkError CctalkBus::readFrameLocked(CctalkFrame& frame,
                                       std::chrono::milliseconds timeout) {
    uint8_t header_bytes[4];
    int r = uart_.read(header_bytes, 4, timeout);
    if (r < 0) return CctalkError::UartError;
    if (r != 4) return CctalkError::Timeout;

    frame.destination = header_bytes[0];
    frame.data_length = header_bytes[1];
    frame.source      = header_bytes[2];
    frame.header      = header_bytes[3];

    std::size_t to_read = frame.data_length + 1;
    std::vector<uint8_t> tail(to_read);
    r = uart_.read(tail.data(), to_read, timeout);
    if (r < 0) return CctalkError::UartError;
    if (static_cast<std::size_t>(r) != to_read) return CctalkError::Timeout;

    frame.data.assign(tail.begin(), tail.begin() + frame.data_length);
    frame.checksum = tail.back();

    if (!frame.validateChecksum()) return CctalkError::BadChecksum;
    return CctalkError::OK;
}

CctalkError CctalkBus::send(const CctalkFrame& frame,
                            std::chrono::milliseconds timeout) {
    std::lock_guard<std::mutex> lock(mutex_);
    return writeFrameLocked(frame, timeout);
}

CctalkError CctalkBus::readFrame(CctalkFrame& frame,
                                 std::chrono::milliseconds timeout) {
    std::lock_guard<std::mutex> lock(mutex_);
    return readFrameLocked(frame, timeout);
}

CctalkError CctalkBus::sendAndReceive(const CctalkFrame& request,
                                      CctalkFrame& response,
                                      std::chrono::milliseconds timeout) {
    std::lock_guard<std::mutex> lock(mutex_);

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
