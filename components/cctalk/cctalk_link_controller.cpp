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

#include <esp_log.h>

#include "cctalk_link_controller.h"
#include "serial_worker.h"

static const char* TAG = "cctalk_link_controller";

namespace esp32cc {

    CctalkLinkController::CctalkLinkController() {
        // start rx and uart tasks
        this->serialWorker = new SerialWorker(this);
    }

    CctalkLinkController::~CctalkLinkController() {
        // stop rx and uart tasks
        delete(serialWorker);
    }

    void CctalkLinkController::openPort(const std::function<void(const std::string& error_msg)>& finish_callback) {
        serialWorker->openPort(this->uartNumber, this->txPin, this->rxPin);
    }

    void CctalkLinkController::closePort() {
        serialWorker->closePort();
    }

    /**
     * Set mandatory options for ccTalk communication
     * 
     * @param uartNumber The ESP32 UART number (default 1)
     * @param txPin The ESP32 GPIO pin used for TX
     * @param rxPin The ESP32 GPIO pin used for RX
     * @param deviceAddress
     * @param isChecksum16bit Specify whether we are using 16bit checksums
     * @param isDesEncrypted Specify whether we are using DES encryption
     */
    void CctalkLinkController::setCcTalkOptions(const uart_port_t uartNumber, const int txPin, const int rxPin, uint8_t deviceAddress, bool isChecksum16bit, bool isDesEncrypted) {
        //port_device = port_device;    
        this->uartNumber = uartNumber;
        this->deviceAddress = deviceAddress;
        this->isChecksum16bit = isChecksum16bit;
        this->isDesEncrypted = isDesEncrypted;
        this->txPin = txPin;
        this->rxPin = rxPin;
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
     * @param data Additional data to be sent if the command requires it
     * @param responseTimeoutMsec The number of milliseconds to wait 
     * @return The requestId corresponding to this request
     */
    uint64_t CctalkLinkController::ccRequest(CcHeader command, std::vector<uint8_t>& data, int responseTimeoutMsec, const std::function<void(uint64_t request_id, const std::vector<uint8_t>& command_data)>& callbackFunction) {

        sendMutex.lock();

        if (data.size() > 255) {
            ESP_LOGE(TAG, "Size of additional data too large! Aborting request.");
            sendMutex.unlock();
            return 0;
        }

        if (this->isDesEncrypted) {
            ESP_LOGE(TAG, "ccTalk encryption requested, unsupported! Aborting request.");
            return 0;
        }

        if (this->isChecksum16bit) {
            // TODO support this
            ESP_LOGE(TAG, "ccTalk 16-bit CRC checksums requested, unsupported! Aborting request.");
            return 0;
        }

        if (this->showCctalkRequest) { 
            std::string formatted_data;
            for (uint8_t tmpData : data) {
                std::stringstream stream;
                stream << "0x" << std::hex << tmpData;
                formatted_data.append(stream.str());
            }                       
            ESP_LOGE(TAG, "> ccTalk request: %s, address: %d, data: %s", ccHeaderGetDisplayableName(command).c_str(), (int(this->deviceAddress)), (data.size() == 0 ? "(empty)" : formatted_data.c_str()));
        }

        // Set the UART receive timeout
        esp_err_t xErr = uart_set_rx_timeout(uartNumber, pdMS_TO_TICKS(responseTimeoutMsec));
        CCTALK_PORT_CHECK((xErr == ESP_OK), false, "cctalk set rx timeout failure, uart_set_rx_timeout() returned (0x%x).", xErr);

        uart_set_always_rx_timeout(uartNumber, true);

        // Build the data structure
        std::vector<uint8_t> requestData;
        requestData.reserve(data.size() + 5);
        requestData.push_back(uint8_t(this->deviceAddress));
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

        this->serialWorker->sendRequest(requestId, requestData, writeTimeoutMsec, responseTimeoutMsec);

        return requestId;
    }

    /**
     * Called when a response is received on the UART
     * 
     * @param request_id
     * @param response_data
     */
    void CctalkLinkController::onResponseReceive(uint64_t request_id, std::vector<uint8_t>& responseData) {
        if (responseData.size() < 5) {
            ESP_LOGE(TAG, "ccTalk response #%llu size too small (%d bytes).", request_id, responseData.size());
            // TODO The command should be retried.
            return;
        }

        uint8_t destinationAddress = responseData.at(0);
        //uint8_t dataSize = responseData.at(1);
        uint8_t sourceAddress = responseData.at(2);
        uint8_t command = responseData.at(3);

        std::vector<uint8_t> commandData = {responseData.begin() + 4, responseData.end()};
        //uint8_t received_checksum = responseData.at(responseData.end() - 1);

        // Format error
        if (responseData.size() != 5 + commandData.size()) {
            ESP_LOGE(TAG, "Invalid ccTalk response #%llu size (%d bytes).", request_id, responseData.size());
            // TODO The command should be retried.
            return;
        }

        // Checksum error        
        uint8_t checksum = 0;
        for (char c : responseData) {
            checksum = static_cast<uint8_t> (checksum + c);
        }

        // The sum of all bytes must be 0.
        if (checksum != 0) {
            ESP_LOGE(TAG, "Invalid ccTalk response #%llu checksum.", request_id);
            // TODO The command should be retried.
            return;
        }

        // We should be the only destination. In multi-host networks this should be
        // ignored, but not here.
        if (destinationAddress != 0x01) {
            ESP_LOGE(TAG, "Invalid ccTalk response #%llu destination address %d.", request_id, int(destinationAddress));
            return;
        }

        // We should be the only destination. In multi-host networks this should be
        // ignored, but not here.
        if (this->deviceAddress != 0 && sourceAddress != this->deviceAddress) {
            ESP_LOGE(TAG, "Invalid ccTalk response #%llu source address %d, expected %d.", request_id, int(sourceAddress), int(this->deviceAddress));
            return;
        }

        std::string formatted_data;
        if (commandData.size() == 0) {
            formatted_data = "(empty)";
        } else {
            for (uint8_t data : commandData) {
                std::stringstream stream;
                stream << "0x" << std::hex << data;
                formatted_data.append(stream.str());
            }
        }


        // Every reply must have the command field set to 0.
        if (command != static_cast<decltype(command)> (CcHeader::Ack)) {
            ESP_LOGE(TAG, "Invalid ccTalk response %llu from address %d: Command is %d, expected 0.", request_id, int(sourceAddress), int(command));
            return;
        }


        if (this->showCctalkResponse) {
            // Don't print response_id, it interferes with identical message hiding.
            ESP_LOGD(TAG, "ccTalk response from address %d, data: %s", int(sourceAddress), formatted_data.c_str());
        }

        this->callbackFunction(request_id, commandData);

    }
}