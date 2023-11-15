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
 * File:   serial_worker.h
 * Author: chris
 *
 * Created on February 4, 2023, 11:36 AM
 */

#ifndef SERIAL_WORKER_H
#define SERIAL_WORKER_H

#include <mutex>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "esp32/rom/uart.h"

#include "cctalk_enums.h"

#define CCTALK_QUEUE_LENGTH                 (255)

#define CCTALK_SERIAL_TASK_PRIO             (15)
#define CCTALK_SERIAL_TASK_STACK_SIZE       (4096)
#define CCTALK_SERIAL_TIMEOUT               (200) // 3.5*8 = 28 ticks, TOUT=3 -> ~24..33 ticks
#define CCTALK_PORT_SERIAL_ISR_FLAG         ESP_INTR_FLAG_IRAM

#define MAX_BUFFER_SIZE 1024

#define CCTALK_PORT_CHECK(a, ret_val, str, ...) \
    if (!(a)) { \
        ESP_LOGE(TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return ret_val; \
    }

namespace esp32cc {

    class Timer {
    public:
        void startTimer(int tdelay); // start the countdown with tdelay in milliseconds
        bool isReady(void); // return true if timer expired
        unsigned long millis();
    private:
        unsigned long target;
    };

    class SerialWorker {
    public:
        SerialWorker();
        virtual ~SerialWorker();

        void setOnResponseReceiveCallback(std::function<void(const uint64_t requestId, const std::vector<uint8_t>& responseData) > callback);
        void setLoggingOptions(bool showFullResponse, bool showSerialRequest, bool showSerialResponse);
        bool openPort(uart_port_t uartNumber, int txPin, int rxPin);
        bool closePort();
        void sendRequest(const uint64_t requestId, const std::vector<uint8_t>& requestData, const int writeTimeoutMsec, const int responseTimeoutMsec);

        uart_port_t getUartNumber();
        uint64_t getRequestId();
        int getResponseTimeoutMsec();

    protected:

    private:

        volatile std::function<void(const uint64_t requestId, const std::vector<uint8_t>& responseData) > onResponseReceiveCallback;
        uint64_t requestId;
        int responseTimeoutMsec;

        // A queue to handle UART event.
        QueueHandle_t cctalkUartQueueHandle;

        uart_port_t uartNumber;
        int txPin;
        int rxPin;

        Timer timer;

        bool portOpen = false;        
    };
}

#endif /* SERIAL_WORKER_H */

