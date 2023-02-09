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

#define MAX_BUFFER_SIZE 4096

#define CCTALK_PORT_CHECK(a, ret_val, str, ...) \
    if (!(a)) { \
        ESP_LOGE(TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return ret_val; \
    }

namespace esp32cc {   
    
    class CctalkLinkController;
    
    class SerialWorker {
    public:
        SerialWorker(const CctalkLinkController* linkController);
        virtual ~SerialWorker();

        void setLoggingOptions(bool showFullResponse, bool showSerialRequest, bool showSerialResponse);
        bool openPort(uart_port_t uartNumber, int txPin, int rxPin);
        void closePort();
        void sendRequest(uint64_t requestId, std::vector<uint8_t>& requestData, int writeTimeoutMsec, int responseTimeoutMsec);

    protected:

    private:
        const CctalkLinkController* linkController;

        static uint64_t requestId;
        int responseTimeoutMsec;

        // A queue to handle UART event.
        QueueHandle_t cctalkUartQueue;
        TaskHandle_t cctalkTaskHandle;

        uart_port_t uartNumber;
        int txPin;
        int rxPin;

        static void uartReceiveTask(void* pvParameters);
        //static void uartQueueTask(void* pvParameters);        
    };

}

#endif /* SERIAL_WORKER_H */

