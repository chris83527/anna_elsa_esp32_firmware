/*
 * The MIT License
 *
 * Copyright 2021 chris.
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
 * File:   cctalk.h
 * Author: chris
 *
 * Created on October 24, 2021, 11:39 AM
 */

#ifndef CCTALK_H
#define CCTALK_H

#define MAX_BUFFER_SIZE 1024

#define CCTALK_HOST (1)
#define CCTALK_HOPPER (3)
#define CCTALK_COIN_VALIDATOR (2)

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_types.h"
#include "esp_event.h"
#include "esp_err.h"
#include "driver/uart.h"

#include "cctalkheaders.h"


#ifdef __cplusplus
extern "C" {
#endif

    typedef enum {
        RX_IDLE, RX_LOOPBACK, RX_ANSWER, RX_COMPLETE, RX_FLUSH
    } rx_state_e;

    typedef enum {
        RXERR_UNEXPECTED_BYTE_IN_IDLE, RXERR_NO_LOOPBACK, RXERR_WRONG_LOOPBACK, RXERR_ANSWER_TIMEOUT, RXERR_MSG_LENGTH, RXERR_CHECKSUM_FAILED
    } error_e;

    typedef struct {
        uint8_t destination;
        uint8_t source;
        uint8_t header;
        uint8_t additionalData[MAX_BUFFER_SIZE];
        uint8_t additionalDataSize;
        uint8_t checksum;
    } cctalk_response_t;

    typedef struct {
        uint8_t source;
        cctalk_header_e header;
        uint8_t additionalData[MAX_BUFFER_SIZE];
        uint8_t additionalDataSize;
        uint8_t destination;
    } cctalk_request_t;

    typedef struct {
        uart_config_t uart_config;
        uint8_t source;
        uint8_t RX_Buffer[MAX_BUFFER_SIZE];
        uint8_t TX_Buffer[MAX_BUFFER_SIZE];
        uint8_t RX_ByteCount;
        uint8_t TX_ByteCount;
        uart_port_t uart_num;
        rx_state_e rxstate;
        bool hasError;
        error_e error;
        SemaphoreHandle_t mutex; //!< Device mutex
    } cctalk_device_t;
    
    
    cctalk_device_t* cctalk_init_device(uart_port_t uart_num, int tx_pin, int rx_pin, uint8_t source);
    cctalk_response_t* cctalk_send_request(cctalk_device_t *device, cctalk_request_t *request);

#ifdef __cplusplus
}
#endif

#endif /* CCTALK_H */

