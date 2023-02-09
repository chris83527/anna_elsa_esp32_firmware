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

/* 
 * File:   cctalk.h
 * Author: Chris Woods
 *
 * Created on October 24, 2021, 11:39 AM
 */

#ifndef CCTALK_H
#define CCTALK_H

using namespace std;

#include "driver/uart.h"

#include "CctalkException.h"
#include "cctalk_enums.h"

#define MAX_BUFFER_SIZE 4096

#define CCTALK_HOST (1)
#define CCTALK_HOPPER (3)
#define CCTALK_COIN_VALIDATOR (2)

#define CCTALK_BAUD_RATE_DEFAULT            (9600)
#define CCTALK_QUEUE_LENGTH                 (20)

#define CCTALK_SERIAL_TASK_PRIO             (15)
#define CCTALK_SERIAL_TASK_STACK_SIZE       (4096)
#define CCTALK_SERIAL_TIMEOUT               (200) // 3.5*8 = 28 ticks, TOUT=3 -> ~24..33 ticks
#define CCTALK_PORT_SERIAL_ISR_FLAG         ESP_INTR_FLAG_IRAM

// Set buffer size for transmission
#define CCTALK_SERIAL_BUF_SIZE              (256)

// common definitions for serial port implementations
#define CCTALK_SERIAL_TX_TOUT_MS            (100) // maximum time for transmission of longest allowed frame buffer
#define CCTALK_SERIAL_TX_TOUT_TICKS         (pdMS_TO_TICKS(CCTALK_SERIAL_TX_TOUT_MS)) // timeout for transmission
#define CCTALK_SERIAL_RX_TOUT_MS            (100)
#define CCTALK_SERIAL_RX_TOUT_TICKS         (pdMS_TO_TICKS(CCTALK_SERIAL_RX_TOUT_MS)) // timeout for receive

#define CCTALK_MINIMUM_MESSAGE_LENGTH       (5)

#define CCTALK_PORT_CHECK(a, ret_val, str, ...) \
    if (!(a)) { \
        ESP_LOGE(TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return ret_val; \
    }


#define MAX_DATA_LENGTH  (58) // the maximum data field length
#define INTER_BYTE_TIMEOUT  (100) // the timeout for next byte in the same message 
#define ANSWER_TIMEOUT (300) // the answer timeout , it might be longer for some pools like eeprom writing

class CctalkRequest;
class CctalkResponse;

class Timer {
public:
    void startTimer(int tdelay); // start the countdown with tdelay in milliseconds
    bool isReady(void); // return true if timer expired
    unsigned long millis();

private:
    unsigned long target;
};

class Cctalk {
public:
    Cctalk(void);
    Cctalk(const uart_port_t uart_num, const int tx_pin, const int rx_pin);
    Cctalk(const Cctalk &orig);
    virtual ~Cctalk();


    bool initialise(void);
    Cctalk::Errors sendRequest(CctalkRequest &request, CctalkResponse &cctalk_response);


private:

    void vCctalkSerialEnable(bool bRxEnable);
    bool cctalkByteReceivedCallback(void);

    int usCctalkSerialRxPoll(size_t xEventSize);
    static void vUartTask(void* pvParameters);
    void startReceive(CctalkResponse &response);

    uart_port_t uartNumber;
    int txPin;
    int rxPin;

    uint8_t rxBuffer[MAX_BUFFER_SIZE];
    uint8_t txBuffer[MAX_BUFFER_SIZE];
    uint8_t rxByteCount;
    uint8_t txByteCount;
    int rxIndex;

    States rxState;
    Errors error;

    SemaphoreHandle_t mutex;

    // A queue to handle UART event.
    QueueHandle_t xCctalkUartQueue;
    TaskHandle_t xCctalkTaskHandle;

    Timer timer;

    bool bRxStateEnabled = false; // Receiver enabled flag
    bool bTxStateEnabled = false; // Transmitter enabled flag    

};

#endif /* CCTALK_H */

