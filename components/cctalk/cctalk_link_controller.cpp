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

#include <memory>
#include <iomanip>
#include <sstream>
#include <functional>
#include <thread>
#include <chrono>



#include <esp_log.h>

#include "cctalk_link_controller.h"
#include "serial_worker.h"

static const char* TAG = "cctalk_link_controller";

//static const std::atomic<bool> requestInProgress{false};
static bool requestInProgress = false;

namespace esp32cc {

    CctalkLinkController::CctalkLinkController() {

    }

    CctalkLinkController::~CctalkLinkController() {

    }

    esp_err_t CctalkLinkController::initialise(const uart_port_t uartNumber, const int txPin, const int rxPin, bool isChecksum16bit, bool isDesEncrypted) {

        this->serialWorker.setOnResponseReceiveCallback([this](const uint64_t requestId, const std::vector<uint8_t> responseData) {
            this->onResponseReceive(requestId, responseData);
        });

        this->controllerAddress = controllerAddress;
        this->isChecksum16bit = isChecksum16bit;
        this->isDesEncrypted = isDesEncrypted;
        this->uartNumber = uartNumber;

        openPort(uartNumber, txPin, rxPin);

        if (isPortOpen) {
            return ESP_OK;
        } else {
            return ESP_FAIL;
        }
    }

    /**
     * Open the serial port
     * 
     * @param uartNumber The ESP32 UART number (default 1)
     * @param txPin The ESP32 GPIO pin used for TX
     * @param rxPin The ESP32 GPIO pin used for RX
     */
    void CctalkLinkController::openPort(const uart_port_t uartNumber, const int txPin, const int rxPin) {

        if (!isPortOpen) {
            this->uartNumber = uartNumber;
            this->isPortOpen = serialWorker.openPort(uartNumber, txPin, rxPin);
        }
    }

    void CctalkLinkController::closePort() {
        this->isPortOpen = serialWorker.closePort();

    }

    /**
     * Set various logging options
     * 
     * @param showFullResponse
     * @param showSerialRequest
     * @param showSerialResponse
     * @param showCctalkRequest
     * @param showCctalkResponse
     */
    void CctalkLinkController::setLoggingOptions(bool showFullResponse, bool showSerialRequest, bool showSerialResponse, bool showCctalkRequest, bool showCctalkResponse) {
        //serial_worker_->setLoggingOptions(show_full_response, show_serial_request, show_serial_response);
        this->showCctalkRequest = showCctalkRequest;
        this->showCctalkResponse = showCctalkResponse;
    }

    /**
     * Send a ccTalk command to a device on the bus
     * 
     * @param command The ccTalk command to send to the device
     * @param 
     * @param data Additional data to be sent if the command requires it
     * @param responseTimeoutMsec The number of milliseconds to wait 
     * @return The requestId corresponding to this request
     */
    uint64_t CctalkLinkController::ccRequest(CcHeader command, uint8_t deviceAddress, std::vector<uint8_t>& data, int responseTimeoutMsec, std::function<void(const std::string& error_msg, const std::vector<uint8_t>& command_data) > callbackFunction) {

        ESP_LOGD(TAG, "Checking no existing request in process");
        while (requestInProgress) {
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }

        requestInProgress = true;

        if (callbackFunction != nullptr) {
            ESP_LOGD(TAG, "Setting callback function");
            try {
                executeOnReturnCallback = callbackFunction;
            } catch (const std::exception& e) {
                ESP_LOGE(TAG, "An exception occurred assigning callback: %s", e.what());
            }
        } else {
            ESP_LOGE(TAG, "executeOnReturn: callbackFunction was null");
        }

        ESP_LOGD(TAG, "Sending request %s", ccHeaderGetDisplayableName(command).c_str());

        this->deviceAddress = deviceAddress;
        ESP_LOGD(TAG, "this->deviceAddress = %d, deviceAddress = %d", this->deviceAddress, deviceAddress);

        if (data.size() > 255) {
            ESP_LOGE(TAG, "Size of additional data too large! Aborting request.");
            requestInProgress = false;
            return 0;
        }

        if (this->isDesEncrypted) {
            ESP_LOGE(TAG, "ccTalk encryption requested, unsupported! Aborting request.");
            requestInProgress = false;
            return 0;
        }

        if (this->isChecksum16bit) {
            // TODO support this
            ESP_LOGE(TAG, "ccTalk 16-bit CRC checksums requested, unsupported! Aborting request.");
            requestInProgress = false;
            return 0;
        }

    //    if (this->showCctalkRequest) {
            std::string formatted_data;
            for (uint8_t tmpData : data) {
                std::stringstream stream;
                stream << "0x" << std::hex << tmpData;
                formatted_data.append(stream.str());
            }
            ESP_LOGD(TAG, "> ccTalk request: %s, address: %d, data: %s", ccHeaderGetDisplayableName(command).c_str(), (int(deviceAddress)), (data.size() == 0 ? "(empty)" : formatted_data.c_str()));
      //  }

        // Build the data structure
        std::vector<uint8_t> requestData;
        //requestData.resize(data.size());
        requestData.push_back(uint8_t(deviceAddress));
        requestData.push_back(uint8_t(data.size()));
        requestData.push_back(uint8_t(this->controllerAddress));
        requestData.push_back(uint8_t(command));
        requestData.insert(requestData.end(), data.begin(), data.end());

        uint8_t checksum = 0;
        for (uint8_t dataByte : requestData) {
            checksum = static_cast<uint8_t> (checksum + dataByte);
        }
        checksum = static_cast<uint8_t> (256 - checksum);
        requestData.push_back(uint8_t(checksum));

        uint64_t requestId = ++this->requestNumber;

        const int writeTimeoutMsec = 500 + requestData.size() * 2; // should be more than enough at 9600 baud.

        ESP_LOGD(TAG, "Sending request");

        this->serialWorker.sendRequest(requestId, requestData, writeTimeoutMsec, responseTimeoutMsec);

        return requestId;
    }

    /**
     * Called when a response is received on the UART
     * 
     * @param request_id
     * @param response_data
     */
    void CctalkLinkController::onResponseReceive(const uint64_t request_id, const std::vector<uint8_t>& responseData) const {

        if (responseData.size() < 5) {
            ESP_LOGE(TAG, "ccTalk response size too small (%d bytes).", responseData.size());
            // TODO The command should be retried.            
            requestInProgress = false;
            return;
        }

        uint8_t destinationAddress = responseData.at(0);
        uint8_t dataSize = responseData.at(1);
        uint8_t sourceAddress = responseData.at(2);
        uint8_t command = responseData.at(3);

        std::vector<uint8_t> responseDataNoLocalEcho = {responseData.begin() + 4, responseData.end() - 1};
        //uint8_t received_checksum = responseData.at(responseData.end() - 1);

       // if (this->showCctalkResponse) {
            std::string formatted_data;
            if (responseDataNoLocalEcho.size() == 0) {
                formatted_data = "(empty)";
            } else {
                for (uint8_t data : responseDataNoLocalEcho) {
                    std::stringstream stream;
                    stream << data;
                    formatted_data.append(stream.str());
                }
            }
            // Don't print response_id, it interferes with identical message hiding.
            ESP_LOGD(TAG, "ccTalk response from address %d, data: %s", int(sourceAddress), formatted_data.c_str());
        //}


        // Format error
        if (responseData.size() != 5 + dataSize) {
            ESP_LOGE(TAG, "Invalid ccTalk response: size (%d bytes).", responseData.size());
            // TODO The command should be retried.      
            requestInProgress = false;
            return;
        }

        // Checksum error        
        uint8_t checksum = 0;
        for (char c : responseData) {
            checksum = static_cast<uint8_t> (checksum + c);
        }

        // The sum of all bytes must be 0.
        if (checksum != 0) {
            ESP_LOGE(TAG, "Invalid ccTalk response checksum.");
            // TODO The command should be retried.            
            requestInProgress = false;
            return;
        }

        // We should be the only destination. In multi-host networks this should be
        // ignored, but not here.
        if (destinationAddress != 0x01) {
            ESP_LOGE(TAG, "Invalid ccTalk response. Destination address %d.", int(destinationAddress));
            requestInProgress = false;
            return;
        }

        // We should be the only destination. In multi-host networks this should be
        // ignored, but not here.
        if (int(sourceAddress) != int(this->deviceAddress)) {
            ESP_LOGE(TAG, "Invalid ccTalk response. Source address %d, expected %d.", int(sourceAddress), int(this->deviceAddress));
            requestInProgress = false;
            return;
        }



        // Every reply must have the command field set to 0.
        if (command != static_cast<decltype(command)> (CcHeader::Ack)) {
            ESP_LOGE(TAG, "Invalid ccTalk response %lld from address %d: Command is %d, expected 0.", request_id, int(sourceAddress), int(command));
            requestInProgress = false;
            return;
        }


        requestInProgress = false;

        if (this->executeOnReturnCallback != nullptr) {
            std::string data;
            this->executeOnReturnCallback(data, responseDataNoLocalEcho);
        } else {
            ESP_LOGE(TAG, "Callback pointer was nullptr");
        }

    }
}