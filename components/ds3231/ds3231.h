/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Richard A Burton <richardaburton@gmail.com>
 * Copyright (c) 2016 Bhuvanchandra DV <bhuvanchandra.dv@gmail.com>
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file ds3231.h
 * @defgroup ds3231 ds3231
 * @{
 *
 * ESP-IDF driver for DS337 RTC and DS3231 high precision RTC module
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2015 Richard A Burton <richardaburton@gmail.com>\n
 * Copyright (c) 2016 Bhuvanchandra DV <bhuvanchandra.dv@gmail.com>\n
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __DS3231_H__
#define __DS3231_H__

#include <chrono>

#include "esp_err.h"
#include "esp_log.h"

#include "I2CManager.h"

#define DS3231_ADDR 0x68 //!< I2C address

class DS3231
{
public:
    /**
     * Alarms
     */
    typedef enum
    {
        DS3231_ALARM_NONE = 0, //!< No alarms
        DS3231_ALARM_1, //!< First alarm
        DS3231_ALARM_2, //!< Second alarm
        DS3231_ALARM_BOTH //!< Both alarms
    } alarm_t;

    /**
     * First alarm rate
     */
    typedef enum
    {
        DS3231_ALARM1_EVERY_SECOND = 0,
        DS3231_ALARM1_MATCH_SEC,
        DS3231_ALARM1_MATCH_SECMIN,
        DS3231_ALARM1_MATCH_SECMINHOUR,
        DS3231_ALARM1_MATCH_SECMINHOURDAY,
        DS3231_ALARM1_MATCH_SECMINHOURDATE
    } alarm1_rate_t;

    /**
     * Second alarm rate
     */
    typedef enum
    {
        DS3231_ALARM2_EVERY_MIN = 0,
        DS3231_ALARM2_MATCH_MIN,
        DS3231_ALARM2_MATCH_MINHOUR,
        DS3231_ALARM2_MATCH_MINHOURDAY,
        DS3231_ALARM2_MATCH_MINHOURDATE
    } alarm2_rate_t;

    /**
     * Squarewave frequency
     */
    typedef enum
    {
        DS3231_SQWAVE_1HZ = 0x00,
        DS3231_SQWAVE_1024HZ = 0x08,
        DS3231_SQWAVE_4096HZ = 0x10,
        DS3231_SQWAVE_8192HZ = 0x18
    } sqwave_freq_t;

    /** Create a DS3231 instance connected to specified I2C pins with specified
     * address
     *
     * @param i2cmgr The I2C port to use (default: 0)
     * @param address I2C-bus address (default: 0x20)
     */
    DS3231(I2CManager& i2cmgr, uint8_t address);

    ~DS3231();

    /**
     * @brief Set the time on the RTC
     *
     * Timezone agnostic, pass whatever you like.
     * I suggest using GMT and applying timezone and DST when read back.
     *
     * @return ESP_OK to indicate success
     */
    esp_err_t set_time(const struct tm* time);

    /**
     * @brief Get the time from the RTC, populates a supplied tm struct
     *
     * @param[out] time RTC time
     * @return ESP_OK to indicate success
     */
    esp_err_t get_time(struct tm& time);

    /**
     * @brief Set alarms
     *
     * `alarm1` works with seconds, minutes, hours and day of week/month, or fires
     * every second. `alarm2` works with minutes, hours and day of week/month, or
     * fires every minute.
     *
     * Not all combinations are supported, see `DS3231_ALARM1_*` and
     * `DS3231_ALARM2_*` defines for valid options you only need to populate the
     * fields you are using in the `tm` struct, and you can set both alarms at the
     * same time (pass `DS3231_ALARM_1`/`DS3231_ALARM_2`/`DS3231_ALARM_BOTH`).
     *
     * If only setting one alarm just pass 0 for `tm` struct and `option` field
     * for the other alarm. If using
     * ::DS3231_ALARM1_EVERY_SECOND/::DS3231_ALARM2_EVERY_MIN you can pass 0 for
     * `tm` struct.
     *
     * If you want to enable interrupts for the alarms you need to do that
     * separately.
     *
     * @return ESP_OK to indicate success
     */
    esp_err_t set_alarm(alarm_t alarms, const struct tm* time1,
                        alarm1_rate_t option1, const struct tm* time2,
                        alarm2_rate_t option2);

    /**
     * @brief Check if oscillator has previously stopped
     *
     * E.g. no power/battery or disabled
     * sets flag to true if there has been a stop
     *
     * @param[out] flag Stop flag
     * @return ESP_OK to indicate success
     */
    esp_err_t get_oscillator_stop_flag(bool& flag);

    /**
     * @brief Clear the oscillator stopped flag
     *
     * @return ESP_OK to indicate success
     */
    esp_err_t clear_oscillator_stop_flag();

    /**
     * @brief Check which alarm(s) have past
     *
     * Sets alarms to
     * `DS3231_ALARM_NONE`/`DS3231_ALARM_1`/`DS3231_ALARM_2`/`DS3231_ALARM_BOTH`
     *
     * @param[out] alarms Alarms
     * @return ESP_OK to indicate success
     */
    esp_err_t get_alarm_flags(alarm_t& alarms);

    /**
     * @brief Clear alarm past flag(s)
     *
     * Pass `DS3231_ALARM_1`/`DS3231_ALARM_2`/`DS3231_ALARM_BOTH`
     *
     * @param alarms Alarms
     * @return ESP_OK to indicate success
     */
    esp_err_t clear_alarm_flags(alarm_t alarms);

    /**
     * @brief enable alarm interrupts (and disables squarewave)
     *
     * Pass `DS3231_ALARM_1`/`DS3231_ALARM_2`/`DS3231_ALARM_BOTH`.
     *
     * If you set only one alarm the status of the other is not changed
     * you must also clear any alarm past flag(s) for alarms with
     * interrupt enabled, else it will trigger immediately.
     *
     * @param dev Device descriptor
     * @param alarms Alarms
     * @return ESP_OK to indicate success
     */
    esp_err_t enable_alarm_ints(alarm_t alarms);

    /**
     * @brief Disable alarm interrupts
     *
     * Does not (re-)enable squarewave
     *
     * @param dev Device descriptor
     * @param alarms Alarm
     * @return ESP_OK to indicate success
     */
    esp_err_t disable_alarm_ints(alarm_t alarms);

    /**
     * @brief Enable the output of 32khz signal
     *
     * **Supported only by DS3231**
     *
     * @return ESP_OK to indicate success
     */
    esp_err_t enable_32khz();

    /**
     * @brief Disable the output of 32khz signal
     *
     * **Supported only by DS3231**
     *
     * @return ESP_OK to indicate success
     */
    esp_err_t disable_32khz();

    /**
     * @brief Enable the squarewave output
     *
     * Disables alarm interrupt functionality.
     *
     * @return ESP_OK to indicate success
     */
    esp_err_t enable_squarewave();

    /**
     * @brief Disable the squarewave output
     *
     * Which re-enables alarm interrupts, but individual alarm interrupts also
     * need to be enabled, if not already, before they will trigger.
     *
     * @return ESP_OK to indicate success
     */
    esp_err_t disable_squarewave();

    /**
     * @brief Set the frequency of the squarewave output
     *
     * Does not enable squarewave output.
     *
     * @param dev Device descriptor
     * @param freq Squarewave frequency
     * @return ESP_OK to indicate success
     */
    esp_err_t set_squarewave_freq(sqwave_freq_t freq);

    /**
     * @brief Get the frequency of the squarewave output
     *
     * Does not enable squarewave output.
     *
     * @param dev Device descriptor
     * @param freq Squarewave frequency to store the output
     * @return ESP_OK to indicate success
     */
    esp_err_t get_squarewave_freq(sqwave_freq_t& freq);

    /**
     * @brief Get the raw temperature value
     *
     * **Supported only by DS3231**
     *
     * @param dev Device descriptor
     * @param[out] temp Raw temperature value
     * @return ESP_OK to indicate success
     */
    esp_err_t get_raw_temp(int16_t& temp);

    /**
     * @brief Get the temperature as an integer
     *
     * **Supported only by DS3231**
     *
     * @param[out] temp Temperature, degrees Celsius
     * @return ESP_OK to indicate success
     */
    esp_err_t get_temp_integer(int8_t& temp);

    /**
     * @brief Get the temperature as a float
     *
     * **Supported only by DS3231**
     *
     * @param[out] temp Temperature, degrees Celsius
     * @return ESP_OK to indicate success
     */
    esp_err_t get_temp_float(float& temp);

    /**
     * @brief Set the aging offset register to a new value.
     *
     * Positive aging values add capacitance to the array,
     * slowing the oscillator frequency. Negative values remove
     * capacitance from the array, increasing the oscillator
     * frequency.
     *
     * **Supported only by DS3231**
     *
     * @param age Aging offset (in range [-128, 127]) to be set
     * @return ESP_OK to indicate success
     */
    esp_err_t set_aging_offset(int8_t age);

    /**
     * @brief Get the aging offset register.
     *
     * **Supported only by DS3231**
     *
     * @param dev Device descriptor
     * @param[out] age Aging offset in range [-128, 127]
     * @return ESP_OK to indicate success
     */
    esp_err_t get_aging_offset(int8_t& age);

private:
    /* Set/clear bits in a byte register, or replace the byte altogether
     * pass the register address to modify, a byte to replace the existing
     * value with or containing the bits to set/clear and one of
     * DS3231_SET/DS3231_CLEAR/DS3231_REPLACE
     * returns true to indicate success
     */
    esp_err_t set_flag(uint8_t addr, uint8_t bits,
                       uint8_t mode);
    esp_err_t get_flag(uint8_t addr, uint8_t mask, uint8_t& flag);
    static uint8_t bcd2dec(uint8_t val);
    static uint8_t dec2bcd(uint8_t val);
    static inline int days_since_january_1st(int year, int month,
                                      int day);

    i2c_device_config_t deviceConfig{};
    i2c_master_dev_handle_t deviceHandle{};

    I2CManager i2c_manager;
};

#endif /* __DS3231_H__ */
