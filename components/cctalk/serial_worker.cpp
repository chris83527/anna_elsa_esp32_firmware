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

#include <functional>
#include <vector>

#include "esp_log.h"

#include "serial_worker.h"
#include "cctalk_link_controller.h"

static const char* TAG = "serial_worker";

namespace esp32cc {

    SerialWorker::SerialWorker(const CctalkLinkController* linkController) {
        this->linkController = linkController;
    }

    SerialWorker::~SerialWorker() {

    }

    bool SerialWorker::openPort(uart_port_t uartNumber, int txPin, int rxPin) {
        esp_err_t xErr = ESP_OK;

        this->uartNumber = uartNumber;
        this->txPin = txPin;
        this->rxPin = rxPin;

        uart_config_t xUartConfig = {
            .baud_rate = 9600,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .source_clk = UART_SCLK_APB,                        
        };

        //mutex = xSemaphoreCreateMutex();

        // Set UART config   
        xErr = uart_driver_install(uartNumber, MAX_BUFFER_SIZE, MAX_BUFFER_SIZE, CCTALK_QUEUE_LENGTH, &this->cctalkUartQueue, CCTALK_PORT_SERIAL_ISR_FLAG);
        CCTALK_PORT_CHECK((xErr == ESP_OK), false, "cctalk serial driver failure, uart_driver_install() returned (0x%x).", xErr);

        xErr = uart_set_pin(uartNumber, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        CCTALK_PORT_CHECK((xErr == ESP_OK), false, "cctalk config failure, uart_set_pin() returned (0x%x).", xErr);

        xErr = uart_param_config(uartNumber, &xUartConfig);
        CCTALK_PORT_CHECK((xErr == ESP_OK), false, "cctalk config failure, uart_param_config() returned (0x%x).", xErr);


        // Create a task to handle UART events
        BaseType_t xStatus = xTaskCreatePinnedToCore(&uartReceiveTask, "uart_queue_task",
                CCTALK_SERIAL_TASK_STACK_SIZE,
                this, CCTALK_SERIAL_TASK_PRIO,
                &this->cctalkTaskHandle, 0);

        if (xStatus != pdPASS) {
            vTaskDelete(this->cctalkTaskHandle);
            // Force exit from function with failure
            CCTALK_PORT_CHECK(false, false, "cctalk stack serial task creation error. xTaskCreate() returned (0x%x).", xStatus);
        } else {
            vTaskSuspend(this->cctalkTaskHandle); // Suspend serial task while stack is not started    
        }

        ESP_LOGD(TAG, "%s Init serial.", __func__);
        
        return true;
    }

    void SerialWorker::closePort() {
        if (this->cctalkTaskHandle != NULL) {
            vTaskDelete(this->cctalkTaskHandle);
        }
        uart_driver_delete(this->uartNumber);
    }

    void SerialWorker::sendRequest(uint64_t requestId, std::vector<uint8_t>& requestData, int writeTimeoutMsec, int responseTimeoutMsec) {

        this->requestId = requestId;
        this->responseTimeoutMsec = responseTimeoutMsec;

        uart_flush_input(uartNumber);
        vTaskDelay(1);
        uart_write_bytes(uartNumber, requestData.data(), requestData.size());

        // Waits while UART sending the packet
        esp_err_t xTxStatus = uart_wait_tx_done(uartNumber, pdMS_TO_TICKS(writeTimeoutMsec));
    }

    /*
     * UART receive task. 
     */
    void SerialWorker::uartReceiveTask(void* pvParameters) {
        SerialWorker *serialWorker = reinterpret_cast<SerialWorker *> (pvParameters);

        uart_event_t xEvent;        

        while (1) {
            if (xQueueReceive(serialWorker->cctalkUartQueue, (void*) &xEvent, (portTickType) portMAX_DELAY) == pdPASS) {
                ESP_LOGD(TAG, "cctalk_UART[%d] event:", serialWorker->uartNumber);

                switch (xEvent.type) {
                        //Event of UART receiving data
                    case UART_DATA:
                    case UART_BUFFER_FULL:
                    {
                        ESP_LOGD(TAG, "Data event, len: %d.", xEvent.size);

                        // Read received data and send it to cctalk stack
                        std::vector<uint8_t> receivedData;
                        receivedData.reserve(xEvent.size); // make sure the vector 
                        int bytesRead = uart_read_bytes(serialWorker->uartNumber, receivedData.data(), xEvent.size, pdMS_TO_TICKS(serialWorker->responseTimeoutMsec));

                        ESP_LOGD(TAG, "%d bytes read. Expecting %d bytes.", bytesRead, xEvent.size);

                        if (receivedData.size() != xEvent.size) {
                            ESP_LOGD(TAG, "Leaving uartReceiveTask");
                            break;
                        }

                        serialWorker->linkController->onResponseReceive(serialWorker->requestId, receivedData);
                        
                        break;
                        //Event of HW FIFO overflow detected
                    }
                    case UART_FIFO_OVF:
                    {
                        ESP_LOGD(TAG, "Hardware FIFO overflow.");
                        xQueueReset(serialWorker->cctalkUartQueue);
                        break;
                    }
                        //Event of UART RX break detected
                    case UART_BREAK:
                    {
                        ESP_LOGD(TAG, "UART RX break.");
                        break;
                    }
                        //Event of UART parity check error
                    case UART_PARITY_ERR:
                    {
                        ESP_LOGD(TAG, "UART parity error.");
                        break;
                    }
                        //Event of UART frame error
                    case UART_FRAME_ERR:
                    {
                        ESP_LOGD(TAG, "UART frame error.");
                        break;
                    }
                    default:
                    {
                        ESP_LOGD(TAG, "UART event type: %d.", xEvent.type);
                        break;
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(25));
        }
        vTaskDelete(NULL);
    }
}