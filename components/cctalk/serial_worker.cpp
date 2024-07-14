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

#include <chrono>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "esp_log.h"

#include "cctalk_link_controller.h"
#include "serial_worker.h"

static const char *TAG = "serial_worker";

// std::mutex sendMutex;

namespace esp32cc {

SerialWorker::SerialWorker(uart_port_t uartNumber, int txPin, int rxPin)
    : m_uartNumber(uartNumber), m_txPin(txPin), m_rxPin(rxPin) {

  ESP_LOGD(
      TAG,
      "Entering SerialWorker constructor: uartNumber %d, txPin %d, rxPin %d",
      uartNumber, txPin, rxPin);

  openPort();

  ESP_LOGD(TAG, "Leaving SerialWorker constructor");
}

SerialWorker::~SerialWorker() {}

bool SerialWorker::openPort() {

  esp_err_t xErr = ESP_OK;
  if (!this->portOpen) {

    uart_config_t xUartConfig = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };

    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    // Set UART config
    xErr = uart_driver_install(this->m_uartNumber, MAX_BUFFER_SIZE,
                               MAX_BUFFER_SIZE, 10,
                               &this->cctalkUartQueueHandle, intr_alloc_flags);

    CCTALK_PORT_CHECK(
        (xErr == ESP_OK), false,
        "cctalk serial driver failure, uart_driver_install() returned (0x%x).",
        xErr);

    xErr = uart_set_pin(this->m_uartNumber, this->m_txPin, this->m_rxPin,
                        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    CCTALK_PORT_CHECK((xErr == ESP_OK), false,
                      "cctalk config failure, uart_set_pin() returned (0x%x).",
                      xErr);

    xErr = uart_param_config(this->m_uartNumber, &xUartConfig);
    CCTALK_PORT_CHECK(
        (xErr == ESP_OK), false,
        "cctalk config failure, uart_param_config() returned (0x%x).", xErr);

    ESP_LOGD(TAG, "%s Init serial.", __func__);

    this->portOpen = true;
  }

  return portOpen;
}

bool SerialWorker::closePort() {
  esp_err_t xErr = uart_driver_delete(this->m_uartNumber);
  this->portOpen = false;
  return xErr != ESP_OK;
}

bool SerialWorker::isRequestInProgress() { return this->requestInProgress; }

void SerialWorker::sendRequest(const uint64_t requestId,
                               const std::vector<uint8_t> &requestData,
                               const int writeTimeoutMsec,
                               const int responseTimeoutMsec) {

  // ESP_LOGD(TAG, "sendRequest called. Request id %d size (start): %d",
  // int(requestId), requestData.size());
  this->requestInProgress = true;
  this->requestId = requestId;
  this->responseTimeoutMsec = responseTimeoutMsec;

  // Set the UART receive timeout
  ESP_LOGD(TAG, "Using UART %d", this->getUartNumber());
  ESP_LOGD(TAG, "Setting RX Timeout %d msec", responseTimeoutMsec);
  // uart_set_rx_timeout(this->getUartNumber(),
  // pdMS_TO_TICKS(responseTimeoutMsec)); CCTALK_PORT_CHECK((xErr == ESP_OK),
  // false, "cctalk set rx timeout failure, uart_set_rx_timeout() returned
  // (0x%x).", xErr);

  // uart_set_always_rx_timeout(this->getUartNumber(), true);

  // xQueueReset(this->cctalkUartQueueHandle);
  uart_flush(this->getUartNumber());
  uart_write_bytes(this->getUartNumber(), requestData.data(),
                   requestData.size());
  uart_wait_tx_done(this->getUartNumber(), pdMS_TO_TICKS(1000)); // wait 1s max

  ESP_LOGD(TAG, "Send complete. Waiting for response");

  std::this_thread::sleep_for(std::chrono::milliseconds(60));

  std::vector<uint8_t> receivedData;

  bool receiveComplete = false;
  timer.startTimer(responseTimeoutMsec);
  int bytesRead = 0;

  int length = 0;

  while (!receiveComplete) {

    if (timer.isReady()) {
      ESP_LOGD(TAG, "Timer hit");
      uart_flush(this->getUartNumber());
      this->onResponseReceiveCallback(this->getRequestId(),
                                      std::vector<uint8_t>());
      break; // receive complete false
    }

    // Read received data and send it to cctalk stack
    ESP_ERROR_CHECK(
        uart_get_buffered_data_len(this->getUartNumber(), (size_t *)&length));
    ESP_LOGD(TAG, "Allegedly available bytes on UART %d: %d",
             this->getUartNumber(), length);

    if (length > 0) {
      receivedData.resize(receivedData.size() + length);
      bytesRead =
          uart_read_bytes(this->getUartNumber(), receivedData.data(), length,
                          pdMS_TO_TICKS(this->getResponseTimeoutMsec()));
    } else {
      receiveComplete = true;
      ESP_LOGD(TAG, "No more data available.");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(60));
  }

  if (receiveComplete) {

    uart_flush(this->getUartNumber());

    this->requestInProgress = false;

    if (receivedData.size() <= requestData.size()) {
      // this shouldn't be possible as we have local echo
      ESP_LOGE(TAG,
               "Received data bytes (%d) was less than or equal to request "
               "data bytes (%d). Is device connected?",
               receivedData.size(), requestData.size());
      uart_flush(this->getUartNumber());
    } else {
      ESP_LOGD(TAG,
               "Read %d bytes - response size %d (with local echo). Executing "
               "callback",
               bytesRead, receivedData.size());
      ESP_LOGD(TAG, "Response size: %d",
               (receivedData.size() - requestData.size()));

      if (this->onResponseReceiveCallback == nullptr) {
        ESP_LOGE(TAG, "this->onResponseReceiveCallback is invalid");
      } else {
        if (receivedData.size() > 5) {
          this->onResponseReceiveCallback(
              this->getRequestId(),
              std::vector<uint8_t>(receivedData.begin() + requestData.size(),
                                   receivedData.end()));
        } else {
          this->onResponseReceiveCallback(this->getRequestId(),
                                          std::vector<uint8_t>());
        }
      }
    }

  } else {
    uart_flush(this->getUartNumber());
  }
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
}

void SerialWorker::setOnResponseReceiveCallback(
    std::function<void(const uint64_t requestId,
                       const std::vector<uint8_t> &responseData)> const
        &callback) {

  ESP_LOGD(TAG, "setOnResponseReceiveCallback called");

  if (callback == nullptr) {
    ESP_LOGE(TAG, "An invalid callback was passed to "
                  "setOnResponseReceiveCallback. Using default callback");
    this->onResponseReceiveCallback =
        [&](const uint64_t requestId,
            const std::vector<uint8_t> &responseData) {
          ESP_LOGD(TAG, "Executing dummy onResponseReceiveCalback");
        };
  } else {
    this->onResponseReceiveCallback = callback;
  }
}

uart_port_t SerialWorker::getUartNumber() { return this->m_uartNumber; }

uint64_t SerialWorker::getRequestId() { return this->requestId; }

int SerialWorker::getResponseTimeoutMsec() { return this->responseTimeoutMsec; }

unsigned long Timer::millis() { return (esp_timer_get_time() / 1000); }

void Timer::startTimer(int tdelay) {
  unsigned long temp;
  temp = millis();
  target = temp + tdelay;
}

bool Timer::isReady() {

  if (millis() >= target) {
    return true;
  } else {
    return false;
  }
}
} // namespace esp32cc
