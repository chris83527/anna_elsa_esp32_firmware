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

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "sdkconfig.h"

#include "cctalkheaders.h"
#include "cctalk.h"

static const char *TAG = "cctalk";

const unsigned char MAX_DATA_LENGTH = 58; // the maximum data field length
const unsigned long INTER_BYTE_TIMEOUT = 75; // the timeout (in ms) for next byte in the same message 
const unsigned long ANSWER_TIMEOUT = 100; // the answer timeout (in ms) , it might be longer for some commands such as eeprom writing

static QueueHandle_t uart_queue;

static cctalk_device_t* cctalk_device;
static cctalk_response_t cctalk_response;

static int64_t target;

void startReceive() {
    cctalk_device->rxstate = RX_LOOPBACK;
    cctalk_device->hasError = false;
    cctalk_device->error = 0;
    cctalk_device->RX_ByteCount = 0;
}

int64_t millis() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}

void startTimer(int tdelay, rx_state_e nextState) {
    int64_t temp;
    temp = millis();
    target = temp + tdelay;
}

bool isTimerReady() {
    int64_t temp;
    temp = millis();
    if (target - temp > 86400000) // one day
        return true;
    else
        return false;
}

static void uart_event_task(void *pvParameters) {
    ESP_LOGD(TAG, "uart_event_task called.");
    uart_event_t event;

    uint8_t* dtmp = (uint8_t*) calloc(MAX_BUFFER_SIZE, sizeof (uint8_t));

    static int responseBeginIndex = 0;

    for (;;) {

        if (xQueueReceive(uart_queue, (void *) &event, (portTickType) portMAX_DELAY)) {
            bzero(dtmp, MAX_BUFFER_SIZE);

            ESP_LOGD(TAG, "uart[%d] interrupt event", cctalk_device->uart_num);

            ESP_LOGD(TAG, "responseBeginIndex: %d", responseBeginIndex);

            if (cctalk_device->rxstate == RX_IDLE) {

                if (event.type == UART_DATA) {
                    ESP_LOGW(TAG, "Erroneous data received on ccTalk bus. Ignoring");
                    uart_flush_input(cctalk_device->uart_num);
                    xQueueReset(uart_queue);
                }

            }

            // copy the bytes received to the RX_Buffer
            if (event.type == UART_DATA) {
                ESP_LOGD(TAG, "Reading %d received bytes into RX_Buffer with rxstate %d", event.size, cctalk_device->rxstate);
                uart_read_bytes(cctalk_device->uart_num, dtmp, event.size, portMAX_DELAY);

                // copy bytes from dtmp to RX_Buffer (this may not be the first time we are called, so we add RX_ByteCount to i)
                for (int i = 0; i < event.size; i++) {
                    cctalk_device->RX_Buffer[responseBeginIndex + i] = dtmp[i];
                    cctalk_device->RX_ByteCount++;
                }
            }

            // because of the open collector circuit, we get the data that we sent back on the data line. We need to check the values are the same (no line errors) and discard this, before proceeding to the actual answer received.
            if (!cctalk_device->hasError && cctalk_device->rxstate == RX_LOOPBACK) {

                if (event.type == UART_DATA) {
                    ESP_LOGD(TAG, "Processing %d bytes of UART data in RX_LOOPBACK state. Request bytes sent: %d. Response bytes received (total): %d", event.size, cctalk_device->TX_ByteCount, cctalk_device->RX_ByteCount);
                    if (cctalk_device->RX_ByteCount < cctalk_device->TX_ByteCount) {
                        ESP_LOGD(TAG, "RX_ByteCount < TX_ByteCount, waiting for next event to deliver more bytes. Remaining in RX_LOOPBACK state.");
                        // we haven't received everything yet
                        //break;                        
                    } else {
                        ESP_LOGD(TAG, "RXByteCount (%d) >= TX_ByteCount (%d). Comparing loopback bytes", cctalk_device->RX_ByteCount, cctalk_device->TX_ByteCount);
                        for (int i = 0; i < cctalk_device->TX_ByteCount; i++) {
                            ESP_LOGD(TAG, "Loopback: RX byte: %d, TX byte: %d", cctalk_device->RX_Buffer[responseBeginIndex + i], cctalk_device->TX_Buffer[responseBeginIndex + i]);
                            if (cctalk_device->RX_Buffer[responseBeginIndex + i] != cctalk_device->TX_Buffer[responseBeginIndex + i]) { // error, wrong loopback
                                cctalk_device->hasError = true;
                                cctalk_device->error = RXERR_WRONG_LOOPBACK;
                            }
                        }

                        if (!cctalk_device->hasError) {
                            ESP_LOGD(TAG, "Loopback ok, switching to RX_ANSWER state with %d answer bytes.", cctalk_device->RX_ByteCount - cctalk_device->TX_ByteCount);
                            cctalk_device->rxstate = RX_ANSWER;
                            responseBeginIndex = cctalk_device->TX_ByteCount;
                        } else {
                            ESP_LOGD(TAG, "Wrong loopback bytes");
                            responseBeginIndex = 0;
                        }

                    }


                } else if (event.timeout_flag) {
                    ESP_LOGE(TAG, "Received UART timeout interrupt. Setting error to RXERR_NO_LOOPBACK");
                    cctalk_device->hasError = true;
                    cctalk_device->error = RXERR_NO_LOOPBACK;
                    uart_flush_input(cctalk_device->uart_num);
                    xQueueReset(uart_queue);
                }

            }

            if (!cctalk_device->hasError && cctalk_device->rxstate == RX_ANSWER) {

                if (event.type == UART_DATA) {
                    uint8_t answerBytes = cctalk_device->RX_ByteCount - cctalk_device->TX_ByteCount;
                    ESP_LOGD(TAG, "Processing %d bytes of UART data in RX_ANSWER state", answerBytes);
                    // skip source and destination check leaving them for later
                    if ((answerBytes >= 1) && (cctalk_device->RX_Buffer[responseBeginIndex + 1] > MAX_DATA_LENGTH)) { // check message length
                        ESP_LOGE(TAG, "additionalDataSize (%d) is greater then MAX_DATA_LENGTH (%d). Aborting.", cctalk_device->RX_Buffer[responseBeginIndex + 1], MAX_DATA_LENGTH);
                        cctalk_device->error = RXERR_MSG_LENGTH;
                        cctalk_device->hasError = true;
                    }

                    if (answerBytes == (cctalk_device->RX_Buffer[responseBeginIndex + 1] + 5) && !cctalk_device->hasError) { // message complete (including checksum)
                        ESP_LOGD(TAG, "Response message complete. Comparing checksum.");

                        uint8_t checksum = 0;

                        for (int i = 0; i < answerBytes; i++) {
                            ESP_LOGD(TAG, "Answer: RX byte: %d", cctalk_device->RX_Buffer[responseBeginIndex + i]);
                            checksum -= cctalk_device->RX_Buffer[responseBeginIndex + i];
                        }

                        if (checksum != 0) { // checksum error
                            ESP_LOGE(TAG, "Checksum failed. Received %d, should be 0", checksum);
                            cctalk_device->error = RXERR_CHECKSUM_FAILED;
                            cctalk_device->hasError = true;
                        } else { // successful                        
                            ESP_LOGD(TAG, "Checksum correct. Switching to state RX_COMPLETE");
                            cctalk_device->rxstate = RX_COMPLETE;
                            cctalk_device->error = 0;
                            cctalk_device->hasError = false;
                        }
                    }

                } else if (event.timeout_flag) {
                    ESP_LOGE(TAG, "Received UART timeout interrupt. Setting state to RXERR_ANSWER_TIMEOUT");
                    responseBeginIndex = 0;
                    cctalk_device->hasError = true;
                    cctalk_device->error = RXERR_ANSWER_TIMEOUT;
                    uart_flush_input(cctalk_device->uart_num);
                    xQueueReset(uart_queue);
                }

            }

            if (cctalk_device->hasError) { // all errors here
                // nothing can be done here we only keep an eye on the break length                
                uart_flush_input(cctalk_device->uart_num);
                xQueueReset(uart_queue);
            }

            responseBeginIndex = 0;

            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

cctalk_device_t* cctalk_init_device(uart_port_t uart_num, int tx_pin, int rx_pin, uint8_t source) {
    ESP_LOGD(TAG, "Initialising device on port %d using tx pin %d and rx pin %d with source address %d", uart_num, tx_pin, rx_pin, source);


    cctalk_device = (cctalk_device_t*) calloc(1, sizeof (cctalk_device_t));

    cctalk_device->uart_config.baud_rate = 9600;
    cctalk_device->uart_config.data_bits = UART_DATA_8_BITS;
    cctalk_device->uart_config.stop_bits = UART_STOP_BITS_1;
    cctalk_device->uart_config.parity = UART_PARITY_DISABLE;
    cctalk_device->uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cctalk_device->uart_num = uart_num;
    cctalk_device->source = source;
    cctalk_device->mutex = xSemaphoreCreateMutex();

    uart_param_config(uart_num, &cctalk_device->uart_config);
    uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(uart_num, MAX_BUFFER_SIZE * 2, MAX_BUFFER_SIZE, 10, &uart_queue, 0);
    uart_enable_rx_intr(uart_num);

    // initialise the state machine to known state
    cctalk_device->rxstate = RX_IDLE;
    cctalk_device->RX_ByteCount = 0;

    xTaskCreate(&uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

    return cctalk_device;

}

cctalk_response_t* cctalk_send_request(cctalk_device_t *device, cctalk_request_t *request) {
    ESP_LOGD(TAG, "Entering cctalk_send_request()");

    if (!xSemaphoreTake(device->mutex, pdMS_TO_TICKS(200))) {
        ESP_LOGD(TAG, "Could not take device mutex");
        return NULL;
    }

    memset(&cctalk_response, 0, sizeof (cctalk_response));

    uint8_t checksum = 0;

    device->TX_Buffer[0] = request->destination;
    device->TX_Buffer[1] = request->additionalDataSize;
    device->TX_Buffer[2] = device->source; // source
    device->TX_Buffer[3] = request->header;
    device->TX_ByteCount = 4;

    for (int i = 0; i < request->additionalDataSize; i++) {
        device->TX_Buffer[device->TX_ByteCount] = request->additionalData[i]; // copy additional data to TX_Buffer beginning after header 
        device->TX_ByteCount++;
    }

    // flush the UART and reset the queue        
    uart_flush_input(cctalk_device->uart_num);
    uart_set_rx_timeout(cctalk_device->uart_num, 20); // TODO: convert INTER_BYTE_TIMEOUT from ms    
    xQueueReset(uart_queue);
    startReceive(); // Reset values

    for (int i = 0; i < device->TX_ByteCount; i++) {
        checksum -= device->TX_Buffer[i];
        ESP_LOGD(TAG, "Writing byte #%d (%d) to uart", i, device->TX_Buffer[i]);
        uart_write_bytes(device->uart_num, &device->TX_Buffer[i], 1);
    }

    ESP_LOGD(TAG, "Checksum = %d", checksum);
    device->TX_Buffer[device->TX_ByteCount] = checksum;
    ESP_LOGD(TAG, "Writing checksum byte #%d (%d) to uart", device->TX_ByteCount, device->TX_Buffer[device->TX_ByteCount]);
    uart_write_bytes(device->uart_num, &device->TX_Buffer[device->TX_ByteCount], 1);
    device->TX_ByteCount++;

    vTaskDelay(pdMS_TO_TICKS(ANSWER_TIMEOUT));

    if (device->hasError || device->rxstate < RX_COMPLETE) {
        ESP_LOGD(TAG, "Timeout or error");

        device->hasError = true;
        device->error = RXERR_ANSWER_TIMEOUT;
    }

    if (device->hasError) {
        ESP_LOGD(TAG, "Error - returning null pointer");
        
        // Give back mutex
        if (!xSemaphoreGive(device->mutex)) {
            ESP_LOGD(TAG, "Could not give device mutex");
            return NULL;
        }
        
        return NULL;
    } else {
        uint8_t offset = device->TX_ByteCount;
        ESP_LOGD(TAG, "Building response -> destination: %d, additionalDataSize: %d, source: %d, header: %d, checksum: %d", device->RX_Buffer[offset + 0], device->RX_Buffer[offset + 1], device->RX_Buffer[offset + 2], device->RX_Buffer[offset + 3], device->RX_Buffer[offset + device->RX_Buffer[offset + 1]]);

        cctalk_response.destination = device->RX_Buffer[offset + 0];
        cctalk_response.additionalDataSize = device->RX_Buffer[offset + 1];
        cctalk_response.source = device->RX_Buffer[offset + 2];
        cctalk_response.header = device->RX_Buffer[offset + 3];

        for (int i = 0; i < cctalk_response.additionalDataSize; i++) {
            cctalk_response.additionalData[i] = device->RX_Buffer[offset + i + 4];
        }
        cctalk_response.checksum = device->RX_Buffer[offset + cctalk_response.additionalDataSize + 4];
    }

    if (!xSemaphoreGive(device->mutex)) {
        ESP_LOGD(TAG, "Could not give device mutex");
        return NULL;
    }

    ESP_LOGD(TAG, "Exiting cctalk_send_request()");
    return &cctalk_response;
}

