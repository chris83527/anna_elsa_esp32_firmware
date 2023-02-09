/*
 * The MIT License
 *
 * Copyright 2021 Chris Woods.
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

#include <string>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "soc/dport_access.h"

#include "esp_log.h"
#include "driver/uart.h"
#include "esp32/rom/uart.h"

#include "sdkconfig.h"

#include "cctalk.h"
#include "CctalkRequest.h"
#include "CctalkResponse.h"

static const char *TAG = "CCTALK";

Cctalk::Cctalk() {

}

Cctalk::Cctalk(const uart_port_t uartNumber, const int txPin, const int rxPin) {
    ESP_LOGD(TAG, "Cctalk constructor called with uartNumber %d, txPin %d, rxPin %d", uartNumber, txPin, rxPin);
    this->uartNumber = uartNumber;
    this->txPin = txPin;
    this->rxPin = rxPin;
}

Cctalk::Cctalk(const Cctalk &orig) {

}

Cctalk::~Cctalk() {

}

unsigned long Timer::millis() {
    return (esp_timer_get_time() / 1000);
}

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

void Cctalk::vCctalkSerialEnable(bool bRxEnable) {
    ESP_LOGD(TAG, "Entering %s", __func__);
    
    if (bRxEnable) {
        ESP_LOGD(TAG, "Enabling serial receive");
    } else {
        ESP_LOGD(TAG, "Disabling serial receive");
    }

    if (bRxEnable) {
        bRxStateEnabled = true;
        vTaskResume(xCctalkTaskHandle); // Resume receiver task
    } else {
        vTaskSuspend(xCctalkTaskHandle); // Block receiver task
        bRxStateEnabled = false;
    }
    
    ESP_LOGD(TAG, "Exiting %s", __func__);
}

bool Cctalk::cctalkByteReceivedCallback() {

    ESP_LOGD(TAG, "%s called.", __func__);
    uint8_t receivedByte;

    int bytesRead = uart_read_bytes(this->uartNumber, &receivedByte, 1, CCTALK_SERIAL_RX_TOUT_TICKS);

    ESP_LOGD(TAG, "%d bytes read. Expecting 1 byte.", bytesRead);

    if (bytesRead != 1) {
        ESP_LOGD(TAG, "Leaving pxCctalkCBByteReceived");
        return false;
    }

    ESP_LOGD(TAG, "rxState: %d, rxIndex: %d, RX_ByteCount: %d, Received byte: %03d", rxState, rxIndex, rxByteCount, receivedByte);
    switch (rxState) {
        
        case RX_LOOPBACK:
        {
            rxBuffer[rxIndex] = receivedByte;
            rxByteCount++;
            if (rxBuffer[rxIndex] != txBuffer[rxIndex]) { // error, wrong loopback
                rxState = RX_ERROR;
                error = CCTALK_ERR_WRONG_LOOPBACK;
                ESP_LOGE(TAG, "Wrong loopback bytes");
                vCctalkSerialEnable(false);
                break;
            } else {
                ESP_LOGD(TAG, "RX_Buffer[%d] == TX_Buffer[%d]. Loopback byte ok.", rxIndex, rxIndex);
            }

            if (rxByteCount == txByteCount) { // loopback ready, prepare for answer
                ESP_LOGD(TAG, "rxByteCount (%d) matches txByteCount (%d). Loopback complete. Switching to response message processing", rxByteCount, txByteCount);  
                rxIndex = 0;
                rxByteCount = 0;
                rxState = RX_ANSWER;
            } else {
                ESP_LOGD(TAG, "rxByteCount (%d) does not match txByteCount (%d). Loopback not yet complete", rxByteCount, txByteCount);
            }

            break;
        }

        case RX_ANSWER:
        {

            rxBuffer[rxByteCount] = receivedByte;

            // skip source and destination check leaving them for later
            if ((rxByteCount >= 1) && (rxBuffer[1] > MAX_DATA_LENGTH)) { // check message length
                rxState = RX_ERROR;
                error = CCTALK_ERR_MSG_LENGTH;
                break;
            }
           
            if ((rxByteCount >= 1 ) && rxByteCount == (rxBuffer[1] + 4)) { // message not complete                
                rxState = RX_COMPLETE;
                break;                
            }
            
            rxByteCount++;

            break;
        }

        case RX_COMPLETE:
        {
            //do nothing, stay here until the main program does something
            // just ignore all other bytes received if a complete correct answer was received
            // they will be handled when going to Idle state                 
            break;
        }

        default:
            break;

    }

    rxIndex++;

    return !error;

}

int Cctalk::usCctalkSerialRxPoll(size_t xEventSize) {
    bool xReadStatus = true;
    int usCnt = 0;

    if (bRxStateEnabled) {
        while (xReadStatus && (usCnt++ < xEventSize)) {
            // Call the cctalk stack callback function and let it fill the stack buffers.
            xReadStatus = cctalkByteReceivedCallback(); // callback to receive FSM
        }

        // The buffer is transferred into cctalk stack and is not needed here any more
        uart_flush_input(uartNumber);
        ESP_LOGD(TAG, "Received data: %d bytes in buffer", (uint32_t) (usCnt - 1));

    } else {
        ESP_LOGW(TAG, "%s: bRxState disabled but junk data (%d bytes) received. ", __func__, xEventSize);
    }

    return usCnt - 1;
}



// UART receive event task

void Cctalk::vUartTask(void* pvParameters) {
    uart_event_t xEvent;
    Cctalk *cctalk = reinterpret_cast<Cctalk *> (pvParameters);

    int usResult = 0;
    for (;;) {
        if (xQueueReceive(cctalk->xCctalkUartQueue, (void*) &xEvent, (portTickType) portMAX_DELAY) == pdPASS) {
            ESP_LOGD(TAG, "cctalk_UART[%d] event:", cctalk->uartNumber);
            switch (xEvent.type) {
                    //Event of UART receiving data
                case UART_DATA:
                case UART_BUFFER_FULL:
                    ESP_LOGD(TAG, "Data event, len: %d.", xEvent.size);
                    // This flag set in the event means that no more
                    // data received during configured timeout and UART TOUT feature is triggered
                    //if (xEvent.timeout_flag) {
                        // Read received data and send it to cctalk stack
                        usResult = cctalk->usCctalkSerialRxPoll(xEvent.size);
                        ESP_LOGD(TAG, "Processed: %d bytes", usResult);
                        // Block receiver task until data is not processed
                        //vTaskSuspend(NULL);
                    //}
                    break;
                    //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGD(TAG, "Hardware FIFO overflow.");
                    xQueueReset(cctalk->xCctalkUartQueue);
                    break;
                    //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGD(TAG, "UART RX break.");
                    break;
                    //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGD(TAG, "UART parity error.");
                    break;
                    //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGD(TAG, "UART frame error.");
                    break;
                default:
                    ESP_LOGD(TAG, "UART event type: %d.", xEvent.type);
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    vTaskDelete(NULL);
}

bool Cctalk::initialise() {

    rxByteCount = 0;
    txByteCount = 0;

    esp_err_t xErr = ESP_OK;

    uart_config_t xUartConfig = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,        
        .source_clk = UART_SCLK_APB,
    };

    mutex = xSemaphoreCreateMutex();

    // Set UART config   
    xErr = uart_driver_install(uartNumber, MAX_BUFFER_SIZE, MAX_BUFFER_SIZE, CCTALK_QUEUE_LENGTH , &xCctalkUartQueue, CCTALK_PORT_SERIAL_ISR_FLAG);
    CCTALK_PORT_CHECK((xErr == ESP_OK), false, "cctalk serial driver failure, uart_driver_install() returned (0x%x).", xErr);
    
    xErr = uart_set_pin(uartNumber, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    CCTALK_PORT_CHECK((xErr == ESP_OK), false, "cctalk config failure, uart_set_pin() returned (0x%x).", xErr);

    xErr = uart_param_config(uartNumber, &xUartConfig);
    CCTALK_PORT_CHECK((xErr == ESP_OK), false, "cctalk config failure, uart_param_config() returned (0x%x).", xErr);
    
    uart_set_rx_timeout(uartNumber, CCTALK_SERIAL_RX_TOUT_TICKS);
    CCTALK_PORT_CHECK((xErr == ESP_OK), false, "cctalk set rx timeout failure, uart_set_rx_timeout() returned (0x%x).", xErr);

    uart_set_always_rx_timeout(uartNumber, true);

    // Create a task to handle UART events
    BaseType_t xStatus = xTaskCreatePinnedToCore(vUartTask, "uart_queue_task",
            CCTALK_SERIAL_TASK_STACK_SIZE,
            this, CCTALK_SERIAL_TASK_PRIO,
            &xCctalkTaskHandle, 0);

    if (xStatus != pdPASS) {
        vTaskDelete(xCctalkTaskHandle);
        // Force exit from function with failure
        CCTALK_PORT_CHECK(false, false, "cctalk stack serial task creation error. xTaskCreate() returned (0x%x).", xStatus);
    } else {
        vTaskSuspend(xCctalkTaskHandle); // Suspend serial task while stack is not started    
    }

    ESP_LOGD(TAG, "%s Init serial.", __func__);

    return true;
}

void Cctalk::startReceive(CctalkResponse &response) {
    ESP_LOGD(TAG, "Entering %s", __func__);

    rxState = States::RX_LOOPBACK;
    rxIndex = 0; // reset the index
    rxByteCount = 0;
    response.setError(0);       
    
    ESP_LOGD(TAG, "Setting serial enable to true");
    vCctalkSerialEnable(true); // Enable reception of bytes on UART

    ESP_LOGD(TAG, "Starting timeout timer");
    timer.startTimer(ANSWER_TIMEOUT);
    
    // wait until the message is complete or we have an error
    while (!(rxState == States::RX_ERROR) && !(rxState == States::RX_COMPLETE) && !timer.isReady()) {
        vTaskDelay(pdMS_TO_TICKS(75));
    }
    
    ESP_LOGD(TAG, "rxState: %d, %s", rxState, timer.isReady() ? "true" : "false");
    if (rxState == States::RX_ERROR) {
        response.setError(error);
    } else if (rxState != States::RX_COMPLETE || timer.isReady()) {        
        response.setError(CCTALK_ERR_ANSWER_TIMEOUT);        
    }

    vCctalkSerialEnable(false); // Disable reception of bytes on UART
    ESP_LOGD(TAG, "Setting serial enable to false");    
    
    ESP_LOGD(TAG, "Leaving %s. rxState: %d", __func__, rxState);
}

Cctalk::Errors Cctalk::sendRequest(CctalkRequest &request, CctalkResponse &response) {
    ESP_LOGD(TAG, "Entering %s", __func__);

    // initialise the state machine to known state
    txByteCount = 0;
    rxByteCount = 0;
    
    memset(txBuffer,0,MAX_BUFFER_SIZE);
    memset(rxBuffer,0,MAX_BUFFER_SIZE);
    
    rxIndex = 0;    
    rxState = Cctalk::RX_IDLE;    
    response.initialise();
    
    if (!xSemaphoreTake(mutex, pdMS_TO_TICKS(200))) {
        ESP_LOGD(TAG, "Could not take device mutex");
        throw CctalkException("Could not take device mutex");
    }

    // disable reception of serial data
    this->vCctalkSerialEnable(false);        
    
    uint8_t checksum = request.getChecksum();
   
    ESP_LOGD(TAG, "Building request -> destination: %03d, additionalDataSize: %03d, source: %03d, header: %03d, checksum: %03d", request.getDestination(), request.getAdditionalData().size(), request.getSource(), request.getHeader(), checksum);

    int index = 0;
    txBuffer[index] = request.getDestination();
    txBuffer[++index] = request.getAdditionalData().size();
    txBuffer[++index] = request.getSource(); // source
    txBuffer[++index] = request.getHeader();
    
    for (uint8_t dataByte : request.getAdditionalData()) {
        txBuffer[++index] = dataByte;
    }

    ESP_LOGD(TAG, "Checksum = %d", checksum);
    txBuffer[++index] = checksum;

    txByteCount = index + 1;    
    ESP_LOGD(TAG, "Writing cctalk message to uart");    
    ESP_LOGD(TAG, "Sending Data -> destination: %03d, additionalDataSize: %03d, source: %03d, header: %03d, checksum: %03d", txBuffer[0], txBuffer[1], txBuffer[2], txBuffer[3], txBuffer[index]);    
    
    xQueueReset(xCctalkUartQueue);
    uart_flush_input(uartNumber);
    vTaskDelay(10);
    uart_write_bytes(uartNumber, &txBuffer, txByteCount);

    // Waits while UART sending the packet
    esp_err_t xTxStatus = uart_wait_tx_done(uartNumber, CCTALK_SERIAL_TX_TOUT_TICKS);
    if (xTxStatus != ESP_OK) {

        // Give back mutex
        if (!xSemaphoreGive(mutex)) {
            ESP_LOGE(TAG, "Could not give device mutex");
            throw CctalkException("Could not return device mutex");
        }

        throw CctalkException("ccTalk serial sent buffer failure.");
    }

    startReceive(response);

    if (response.getError()) {
        ESP_LOGE(TAG, "Error - Invalid or incomplete response received from cctalk device. State: %d. Error: %d", rxState, response.getError());

        // Give back mutex
        if (!xSemaphoreGive(mutex)) {
            ESP_LOGE(TAG, "Could not give device mutex");
            throw CctalkException("Could not return device mutex");
        }

        throw CctalkException("An invalid cctalk response was returned");
    }    
    
    if (rxBuffer[3] != CCTALK_ACK) {

        // Give back mutex
        if (!xSemaphoreGive(mutex)) {
            ESP_LOGE(TAG, "Could not give device mutex");
            throw CctalkException("Could not return device mutex");
        }

        ESP_LOGE(TAG, "A non-ACK response was received: %d", rxBuffer[3]);
        throw CctalkException("A non-ACK response was received");
    }

    ESP_LOGD(TAG, "Building response");    
    
    // Process RX Bytes to response    
    response.setDestination(rxBuffer[0]);
    response.setAdditionalDataLength(rxBuffer[1]);
    response.setSource(rxBuffer[2]);
    response.setHeader(rxBuffer[3]);   
    response.getAdditionalData().clear();
    for (int i = 0; i < response.getAdditionalDataLength(); i++) {
        ESP_LOGD(TAG, "Adding byte to additionalData: %03d", rxBuffer[i+4]);
        response.getAdditionalData().push_back(rxBuffer[i + 4]);
    }    
    response.setChecksum(rxBuffer[4 + response.getAdditionalDataLength()]);
    ESP_LOGD(TAG, "Response checksum: %03d", response.getChecksum());
    
    if (!response.isValidResponse()) {
        
        vTaskDelay(pdMS_TO_TICKS(500)); // wait for devices to reset
        
        // Give back mutex
        if (!xSemaphoreGive(mutex)) {
            ESP_LOGE(TAG, "Could not give device mutex");
            throw CctalkException("Could not return device mutex");
        }

        throw CctalkException("Response was not valid (Checksum error)");
    }

    if (!xSemaphoreGive(mutex)) {
        ESP_LOGE(TAG, "Could not give device mutex");
        return CCTALK_ERR_GIVE_MUTEX;
    }

    ESP_LOGD(TAG, "Exiting cctalk_send_request()");
    
    rxState = Cctalk::RX_IDLE; 
    
    return CCTALK_ERR_OK;
}