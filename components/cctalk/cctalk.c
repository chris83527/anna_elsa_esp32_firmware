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

static const char *TAG = "ccTalk";

const unsigned char MAX_DATA_LENGTH = 58; // the maximum data field length
const unsigned long INTER_BYTE_TIMEOUT = 75; // the timeout (in ms) for next byte in the same message 
const unsigned long ANSWER_TIMEOUT = 100; // the answer timeout (in ms) , it might be longer for some commands such as eeprom writing

static QueueHandle_t uart_queue;

static cctalk_device_t *cctalk_device;
static cctalk_response_t *cctalk_response;

static int64_t target;

void startReceive() {
    cctalk_device->rxstate = RX_LOOPBACK;
    cctalk_device->hasError = false;
    cctalk_device->error = 0;
    cctalk_device->RX_ByteCount = 0;

    uart_set_rx_timeout(cctalk_device->uart_num, INTER_BYTE_TIMEOUT); // TODO: convert INTER_BYTE_TIMEOUT from ms

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
    uart_event_t event;

    uint8_t* dtmp = (uint8_t*) calloc(MAX_BUFFER_SIZE, sizeof (uint8_t));

    for (;;) {

        if (xQueueReceive(uart_queue, (void *) &event, (portTickType) portMAX_DELAY)) {
            bzero(dtmp, MAX_BUFFER_SIZE);

            ESP_LOGI(TAG, "uart[%d] event:", cctalk_device->uart_num);

            if (cctalk_device->rxstate == RX_IDLE) {

                if (event.type == UART_DATA) {
                    ESP_LOGW(TAG, "Erroneous data received on ccTalk bus. Ignoring");
                    uart_flush_input(cctalk_device->uart_num);
                    xQueueReset(uart_queue);
                }

            }

            // copy the bytes received to the RX_Buffer
            if (event.type == UART_DATA) {
                ESP_LOGI(TAG, "Reading %d received bytes into RX_Buffer with rxstate %d", event.size, cctalk_device->rxstate);
                uart_read_bytes(cctalk_device->uart_num, dtmp, event.size, portMAX_DELAY);

                // copy bytes from dtmp to RX_Buffer (this may not be the first time we are called, so we add RX_ByteCount to i)
                for (int i = 0; i < event.size; i++) {
                    cctalk_device->RX_Buffer[cctalk_device->RX_ByteCount + i] = dtmp[i];
                    cctalk_device->RX_ByteCount += i;
                }
            }

            // because of the open collector circuit, we get the data that we sent back on the data line. We need to check the values are the same (no line errors) and discard this, before proceeding to the actual answer received.
            if (cctalk_device->rxstate == RX_LOOPBACK) {

                if (event.type == UART_DATA) {

                    if (cctalk_device->RX_ByteCount < cctalk_device->TX_ByteCount) {
                        ESP_LOGI(TAG, "RX_ByteCount < TX_ByteCount, waiting for next event.");
                        // we haven't received everything yet
                        break;
                    } else {
                        ESP_LOGI(TAG, "RXByteCount >= TX_ByteCount. Comparing loopback bytes");
                        for (int i = 0; i < cctalk_device->TX_ByteCount; i++) {
                            if (cctalk_device->RX_Buffer[i] != cctalk_device->TX_Buffer[i]) { // error, wrong loopback
                                cctalk_device->hasError = true;
                                cctalk_device->error = RXERR_WRONG_LOOPBACK;
                                break;
                            }
                        }

                    }

                    if (!cctalk_device->hasError) {

                        if (cctalk_device->RX_ByteCount > (cctalk_device->RX_Buffer[1] + 4)) { // loopback ready, prepare for answer
                            cctalk_device->RX_ByteCount = cctalk_device->RX_ByteCount - (cctalk_device->RX_Buffer[1] + 4); // we have bytes left over that weren't from the loopback
                            cctalk_device->rxstate = RX_ANSWER;
                        }
                    }

                } else if (event.timeout_flag) {
                    cctalk_device->hasError = true;
                    cctalk_device->error = RXERR_NO_LOOPBACK;
                    uart_flush_input(cctalk_device->uart_num);
                    xQueueReset(uart_queue);
                }

            }

            if (cctalk_device->rxstate == RX_ANSWER) {

                if (event.type == UART_DATA) {

                    // skip source and destination check leaving them for later
                    if ((cctalk_device->RX_ByteCount >= 1) && (cctalk_device->RX_Buffer[1] > MAX_DATA_LENGTH)) { // check message length
                        cctalk_device->error = RXERR_MSG_LENGTH;
                        cctalk_device->hasError = true;
                        //timer.startTimer((ANSWER_TIMEOUT));
                        break;
                    }

                    if (cctalk_device->RX_ByteCount > (cctalk_device->RX_Buffer[1] + 4)) { // message complete

                        uint8_t checksum = 0;

                        for (cctalk_device->RX_ByteCount = 0; cctalk_device->RX_ByteCount <= cctalk_device->RX_Buffer[1] + 4; cctalk_device->RX_ByteCount++) {
                            checksum -= cctalk_device->RX_Buffer[cctalk_device->RX_ByteCount];
                        }

                        if (checksum != 0) { // checksum error
                            cctalk_device->error = RXERR_CHECKSUM_FAILED;
                            cctalk_device->hasError = true;
                            //startTimer(ANSWER_TIMEOUT, RXERR_CHECKSUM_FAILED);
                            break;
                        } else { // successful                        
                            cctalk_device->rxstate = RX_COMPLETE;
                            cctalk_device->error = 0;
                            cctalk_device->hasError = false;
                            break;
                        }
                    }

                } else if (event.timeout_flag) {
                    cctalk_device->hasError = true;
                    cctalk_device->error = RXERR_ANSWER_TIMEOUT;
                    uart_flush_input(cctalk_device->uart_num);
                    xQueueReset(uart_queue);
                }

                break;
            }

            if (cctalk_device->hasError) { // all errors here
                // nothing can be done here we only keep an eye on the break length
                //if (_port->available()) { //read all bytes, the timer is set where the error is found.                        
                uart_flush_input(cctalk_device->uart_num);
                xQueueReset(uart_queue);
            }

        }
    }
}

void cctalk_init_device(uart_port_t uart_num, int tx_pin, int rx_pin, uint8_t source) {
    ESP_LOGI(TAG, "Initialising device on port %d using tx pin %d and rx pin %d with source address %d", uart_num, tx_pin, rx_pin, source);

    cctalk_device = (cctalk_device_t*) calloc(1, sizeof (cctalk_device_t));

    cctalk_device->uart_config.baud_rate = 9600;
    cctalk_device->uart_config.data_bits = UART_DATA_8_BITS;
    cctalk_device->uart_config.stop_bits = UART_STOP_BITS_1;
    cctalk_device->uart_config.parity = UART_PARITY_DISABLE;
    cctalk_device->uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cctalk_device->uart_num = uart_num;
    cctalk_device->source = source;

    uart_param_config(uart_num, &cctalk_device->uart_config);
    uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(uart_num, MAX_BUFFER_SIZE * 2, MAX_BUFFER_SIZE, 10, &uart_queue, 0);
    uart_enable_rx_intr(uart_num);

    // initialise the state machine to known state
    cctalk_device->rxstate = RX_IDLE;
    cctalk_device->RX_ByteCount = 0;

    xTaskCreate(&uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

}

cctalk_response_t* cctalk_send_request(cctalk_device_t *device, cctalk_request_t *request) {

    cctalk_response = (cctalk_response_t*) calloc(1, sizeof (cctalk_response_t));

    uint8_t temp = 0;

    device->TX_Buffer[0] = request->destination;
    device->TX_Buffer[1] = request->additionalDataSize;
    device->TX_Buffer[2] = device->source; // source
    device->TX_Buffer[3] = request->header;
    device->TX_ByteCount = 4;

    for (uint8_t i = 0; i < request->additionalDataSize; i++) {
        device->TX_Buffer[i + 4] = request->additionalData[i];
    }

    device->TX_Buffer[request->additionalDataSize + 4] = 0;
    for (temp = 0; temp < request->additionalDataSize + 4; temp++) {
        device->TX_Buffer[request->additionalDataSize + 4] -= device->TX_Buffer[temp];
        uart_write_bytes(device->uart_num, &device->TX_Buffer[temp], 1);
    }

    // Set TX_ByteCount to the size of the message including additional payload
    device->TX_ByteCount += request->additionalDataSize + 4;

    // flush the UART and reset the queue
    uart_flush_input(cctalk_device->uart_num);
    xQueueReset(uart_queue);

    uart_write_bytes(device->uart_num, &device->TX_Buffer[device->TX_ByteCount], 1);

    startReceive(); // Start receiving the answer

    while (!device->hasError && device->rxstate < RX_COMPLETE) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return cctalk_response;
}

