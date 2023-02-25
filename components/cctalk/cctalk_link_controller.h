/*
 * The MIT License
 *
 * Copyright 2023 chris.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/* 
 * File:   cctalk_link_controller.h
 * Author: chris
 *
 * Created on February 3, 2023, 7:16 PM
 */

#ifndef CCTALK_LINK_CONTROLLER_H
#define CCTALK_LINK_CONTROLLER_H

#include <functional>
#include <string>
#include <mutex>
#include <vector>
#include <atomic>
#include <thread>

#include "esp_log.h"

#include "serial_worker.h"


namespace esp32cc {    

    class CctalkLinkController {
    public:


        /// void callback()
        using ResponseAckFunc = std::function<void()>;

        /// void callback(const std::vector<uint8_t>& command_data)
        using ResponseGenericReplyFunc = std::function<void(const std::vector<uint8_t>& command_data)>;

        /// void callback(uint8_t command, const std::vector<uint8_t>& command_data)
        using ResponseWithCommandFunc = std::function<void(uint8_t command, const std::vector<uint8_t>& command_data)>;


        CctalkLinkController();
        virtual ~CctalkLinkController();

        /// Set logging options (though logMessage() signal). Call before opening the device.
        void setLoggingOptions(bool showFullResponse, bool showSerialRequest, bool showSerialResponse, bool showCctalkRequest, bool showCctalkResponse);

        esp_err_t initialise(const uart_port_t uartNumber, const int txPin, const int rxPin, bool isChecksum16bit, bool isDesEncrypted);

        /// Send request to serial port.
        /// The returned value is request ID which can be used to identify which
        /// response comes from which request.
        uint64_t ccRequest(CcHeader command, uint8_t deviceAddress, std::vector<uint8_t>& additionalData, int responseTimeoutMsec, std::function<void(const std::string& error_msg, const std::vector<uint8_t>& command_data) > callbackFunction);

        /// Handle generic serial response and emit ccResponse
        void onResponseReceive(const uint64_t request_id, const std::vector<uint8_t>& response_data) const;

    private:
        
        void openPort(const uart_port_t uartNumber, const int txPin, const int rxPin);

        /// Close the serial port.
        void closePort();

        std::function<void(const std::string& error_msg, const std::vector<uint8_t>& command_data) > executeOnReturnCallback;

        uint8_t deviceAddress = 0x00; // The slave device we are currently talking to
        uint8_t controllerAddress = 0x01; ///< Controller address. 1 means "Master". There is no reason to change this.

        bool isChecksum16bit = false; /// If true, use 16-bit CRC checksum. Otherwise use simple 8-bit checksum. The device must be set to the same value.
        bool isDesEncrypted = false; ///< If true, use DES encryption. The device must be set to the same value. NOTE: Unsupported.

        uint64_t requestNumber = 0; ///< Request number. This is used to identify which response came from which request.

        bool showCctalkRequest = true;
        bool showCctalkResponse = true;

        TaskHandle_t workerTaskHandle;
        SerialWorker* serialWorker;

        uart_port_t uartNumber = 1;

        bool isPortOpen = false;
    };
}

#endif /* CCTALK_LINK_CONTROLLER_H */

