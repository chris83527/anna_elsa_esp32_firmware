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

#define MAX_BUFFER_SIZE 2048

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

#define CCTALK_ACK (0)

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

    enum States : uint8_t{
        RX_IDLE, RX_LOOPBACK, RX_ANSWER, RX_COMPLETE, RX_FLUSH, RX_ERROR
    };

    enum Errors : uint8_t {
        CCTALK_ERR_OK, CCTALK_ERR_UNEXPECTED_BYTE_IN_IDLE, CCTALK_ERR_NO_LOOPBACK, CCTALK_ERR_WRONG_LOOPBACK, CCTALK_ERR_ANSWER_TIMEOUT, CCTALK_ERR_MSG_LENGTH, CCTALK_ERR_CHECKSUM_FAILED, CCTALK_ERR_GIVE_MUTEX, CCTALK_ERR_TAKE_MUTEX
    };

    enum Headers : uint8_t {
        RESET_DEVICE = 1,
        REQUEST_COMMS_STATUS_VARIABLES = 2,
        CLEAR_COMMS_STATUS_VARIABLES = 3,
        REQUEST_COMMS_REVISION = 4,
        REQUEST_SERVICE_STATUS = 104,
        DATA_STREAM = 105,
        REQUEST_ESCROW_STATUS = 106,
        OPERATE_ESCROW = 107,
        REQUEST_ENCRYPTED_MONETARY_ID = 108,
        REQUEST_ENCRYPTED_HOPPER_STATUS = 109,
        SWITCH_ENCRYPTION_KEY = 110,
        REQUEST_ENCRYPTION_SUPPORT = 111,
        READ_ENCRYPTED_EVENTS = 112,
        SWITCH_BAUD_RATE = 113,
        REQUEST_USB_ID = 114,
        REQUEST_REAL_TIME_CLOCK = 115,
        MODIFY_REAL_TIME_CLOCK = 116,
        REQUEST_CASHBOX_VALUE = 117,
        MODIFY_CASHBOX_VALUE = 118,
        REQUEST_HOPPER_BALANCE = 119,
        MODIFY_HOPPER_BALANCE = 120,
        PURGE_HOPPER = 121,
        REQUEST_ERROR_STATUS = 122,
        REQUEST_ACTIVITY_REGISTER = 123,
        VERIFY_MONEY_OUT = 124,
        PAY_MONEY_OUT = 125,
        CLEAR_MONEY_COUNTER = 126,
        REQUEST_MONEY_OUT = 127,
        REQUEST_MONEY_IN = 128,
        READ_BARCODE_DATA = 129,
        REQUEST_INDEXED_HOPPER_DISPENSE_COUNT = 130,
        REQUEST_HOPPER_COIN_VALUE = 131,
        EMERGENCY_STOP_VALUE = 132,
        REQUEST_HOPPER_POLLING_VALUE = 133,
        DISPENSE_HOPPER_VALUE = 134,
        SET_ACCEPT_LIMIT = 135,
        STORE_ENCRYPTION_CODE = 136,
        SWITCH_ENCRYPTION_CODE = 137,
        FINISH_FIRMWARE_UPGRADE = 138,
        BEGIN_FIRMWARE_UPGRADE = 139,
        UPLOAD_FIRMWARE = 140,
        REQUEST_FIRMWARE_UPGRADE_CAPABILITY = 141,
        FINISH_BILL_TABLE_UPGRADE = 142,
        BEGIN_BILL_TABLE_UPGRADE = 143,
        UPLOAD_BILL_TABLES = 144,
        REQUEST_CURRENCY_REVISION = 145,
        OPERATE_BIDIRECTIONAL_MOTORS = 146,
        PERFORM_STACKER_CYCLE = 147,
        READ_OPTO_VOLTAGES = 148,
        REQUEST_INDIVIDUAL_ERROR_COUNTER = 149,
        REQUEST_INDIVIDUAL_ACCEPT_COUNTER = 150,
        TEST_LAMPS = 151,
        REQUEST_BILL_OPERATING_MODE = 152,
        MODIFY_BILL_OPERATING_MODE = 153,
        ROUTE_BILL = 154,
        REQUEST_BILL_POSITION = 155,
        REQUEST_COUNTRY_SCALING_FACTOR = 156,
        REQUEST_BILL_ID = 157,
        MODIFY_BILL_ID = 158,
        READ_BUFFERED_BILL_EVENTS = 159,
        REQUEST_CIPHER_KEY = 160,
        PUMP_RNG = 161,
        MODIFY_INHIBIT_AND_OVERRIDE_REGISTERS = 162,
        TEST_HOPPER = 163,
        ENABLE_HOPPER = 164,
        MODIFY_VARIABLE_SET = 165,
        REQUEST_HOPPER_STATUS = 166,
        DISPENSE_HOPPER_COINS = 167,
        REQUEST_HOPPER_DISPENSE_COUNT = 168,
        REQUEST_ADDRESS_MODE = 169,
        REQUEST_BASE_YEAR = 170,
        REQUEST_HOPPER_COIN = 171,
        EMERGENCY_STOP = 172,
        REQUEST_THERMISTOR_READING = 173,
        REQUEST_PAYOUT_FLOAT = 174,
        MODIFY_PAYOUT_FLOAT = 175,
        REQUEST_ALARM_COUNTER = 176,
        HANDHELD_FUNCTION = 177,
        REQUEST_BANK_SELECT = 178,
        MODIFY_BANK_SELECT = 179,
        REQUEST_SECURITY_SETTING = 180,
        MODIFY_SECURITY_SETTING = 181,
        DOWNLOAD_CALIBRATION_INFO = 182,
        UPLOAD_WINDOW_DATA = 183,
        REQUEST_COIN_ID = 184,
        MODIFY_COIN_ID = 185,
        REQUEST_PAYOUT_CAPACITY = 186,
        MODIFY_PAYOUT_CAPACITY = 187,
        REQUEST_DEFAULT_SORTER_PATH = 188,
        MODIFY_DEFAULT_SORTER_PATH = 189,
        KEYPAD_CONTROL = 191,
        REQUEST_BUILD_CODE = 192,
        REQUEST_FRAUD_COUNTER = 193,
        REQUEST_REJECT_COUNTER = 194,
        REQUEST_LAST_MODIFICATION_DATE = 195,
        REQUEST_CREATION_DATE = 196,
        CALCULATE_ROM_CHECKSUM = 197,
        COUNTERS_TO_EEPROM = 198,
        CONFIGURATION_TO_EEPROM = 199,
        ACMI_UNENCRYPTED_PRODUCT_ID = 200,
        REQUEST_TEACH_STATUS = 201,
        TEACH_MODE_CONTROL = 202,
        DISPLAY_CONTROL = 203,
        METER_CONTROL = 204,
        REQUEST_PAYOUT_ABSOLUTE_COUNT = 207,
        MODIFY_PAYOUT_ABSOLUTE_COUNT = 208,
        REQUEST_SORTER_PATHS = 209,
        MODIFY_SORTER_PATHS = 210,
        POWER_MANAGEMENT_CONTROL = 211,
        REQUEST_COIN_POSITION = 212,
        REQUEST_OPTION_FLAGS = 213,
        WRITE_DATA_BLOCK = 214,
        READ_DATA_BLOCK = 215,
        REQUEST_DATA_STORAGE_AVAILABILITY = 216,
        REQUEST_PAYOUT_HIGHLOW_STATUS = 217,
        ENTER_PIN_NUMBER = 218,
        ENTER_NEW_PIN_NUMBER = 219,
        ACMI_ENCRYPTED_DATA = 220,
        REQUEST_SORTER_OVERRIDE_STATUS = 221,
        MODIFY_SORTER_OVERRIDE_STATUS = 222,
        MODIFY_ENCRYPTED_INHIBIT_AND_OVERRIDE_REGISTERS = 223,
        REQUEST_ENCRYPTED_PRODUCT_ID = 224,
        REQUEST_ACCEPT_COUNTER = 225,
        REQUEST_INSERTION_COUNTER = 226,
        REQUEST_MASTER_INHIBIT_STATUS = 227,
        MODIFY_MASTER_INHIBIT_STATUS = 228,
        READ_BUFFERED_CREDIT_OR_ERROR_CODES = 229,
        REQUEST_INHIBIT_STATUS = 230,
        MODIFY_INHIBIT_STATUS = 231,
        PERFORM_SELF_CHECK = 232,
        LATCH_OUTPUT_LINES = 233,
        SEND_DH_PUBLIC_KEY = 234,
        READ_DH_PUBLIC_KEY = 235,
        READ_OPTO_STATES = 236,
        READ_INPUT_LINES = 237,
        TEST_OUTPUT_LINES = 238,
        OPERATE_MOTORS = 239,
        TEST_SOLENOIDS = 240,
        REQUEST_SOFTWARE_REVISION = 241,
        REQUEST_SERIAL_NUMBER = 242,
        REQUEST_DATABASE_VERSION = 243,
        REQUEST_PRODUCT_CODE = 244,
        REQUEST_EQUIPMENT_CATEGORY_ID = 245,
        REQUEST_MANUFACTURER_ID = 246,
        REQUEST_VARIABLE_SET = 247,
        REQUEST_STATUS = 248,
        REQUEST_POLLING_PRIORITY = 249,
        ADDRESS_RANDOM = 250,
        ADDRESS_CHANGE = 251,
        ADDRESS_CLASH = 252,
        ADDRESS_POLL = 253,
        SIMPLE_POLL = 254,
        FACTORY_SETUP_AND_TEST = 255
    };

    bool initialise(void);
    Cctalk::Errors sendRequest(CctalkRequest &request, CctalkResponse &cctalk_response);


private:

    void vCctalkSerialEnable(bool bRxEnable);
    bool pxCctalkCBByteReceived(void);

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

