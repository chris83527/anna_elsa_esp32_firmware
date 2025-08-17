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
 * @file ds3231.cpp
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
#include <esp_err.h>
#include <cstdio>

#include "ds3231.h"
#include "hal/i2c_types.h"

#define DS3231_STAT_OSCILLATOR 0x80
#define DS3231_STAT_32KHZ 0x08
#define DS3231_STAT_ALARM_2 0x02
#define DS3231_STAT_ALARM_1 0x01

#define DS3231_CTRL_OSCILLATOR 0x80
#define DS3231_CTRL_TEMPCONV 0x20
#define DS3231_CTRL_ALARM_INTS 0x04
#define DS3231_CTRL_ALARM2_INT 0x02
#define DS3231_CTRL_ALARM1_INT 0x01

#define DS3231_ALARM_WDAY 0x40
#define DS3231_ALARM_NOTSET 0x80

#define DS3231_ADDR_TIME 0x00
#define DS3231_ADDR_ALARM1 0x07
#define DS3231_ADDR_ALARM2 0x0b
#define DS3231_ADDR_CONTROL 0x0e
#define DS3231_ADDR_STATUS 0x0f
#define DS3231_ADDR_AGING 0x10
#define DS3231_ADDR_TEMP 0x11

#define DS3231_12HOUR_FLAG 0x40
#define DS3231_12HOUR_MASK 0x1f
#define DS3231_PM_FLAG 0x20
#define DS3231_MONTH_MASK 0x1f

#define CHECK_ARG(ARG)                                                         \
  do {                                                                         \
    if (!(ARG))                                                                \
      return ESP_ERR_INVALID_ARG;                                              \
  } while (0)

enum { DS3231_SET = 0, DS3231_CLEAR, DS3231_REPLACE };

static const int days_per_month[] = {31, 28, 31, 30, 31, 30,
                                     31, 31, 30, 31, 30, 31};
static const int days_per_month_leap_year[] = {31, 29, 31, 30, 31, 30,
                                               31, 31, 30, 31, 30, 31};

static const char *TAG = "ds3231";

/** Create a DS3231 instance connected to specified I2C pins with specified
 * address
 *
 * @param i2cmgr The I2C manager
 * @param address I2C-bus address (default: 0x20)
 */
DS3231::DS3231(I2CManager &i2cmgr, const uint8_t address)
    : i2c_manager{i2cmgr} {
  ESP_LOGD(TAG, "i2c_address: %d", address);
  this->deviceConfig.device_address = address;
  this->deviceConfig.scl_speed_hz = 100000;
  this->deviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;

  i2c_manager.addDevice(this->deviceConfig, this->deviceHandle);
}

DS3231::~DS3231() = default;

esp_err_t DS3231::set_time(const struct tm *time) {
  CHECK_ARG(time);

  std::vector<uint8_t> data;

  /* time/date data */
  data.push_back(dec2bcd(time->tm_sec));
  data.push_back(dec2bcd(time->tm_min));
  data.push_back(dec2bcd(time->tm_hour));
  /* The week data must be in the range 1 to 7, and to keep the start on the
   * same day as for tm_wday have it start at 1 on Sunday. */
  data.push_back(dec2bcd(time->tm_wday + 1));
  data.push_back(dec2bcd(time->tm_mday));
  data.push_back(dec2bcd(time->tm_mon + 1));
  data.push_back(dec2bcd(time->tm_year - 100));

  return i2c_manager.writeRegister(this->deviceHandle, DS3231_ADDR_TIME, data);
}

esp_err_t DS3231::set_alarm(const alarm_t alarms, const struct tm *time1,
                            const alarm1_rate_t option1, const struct tm *time2,
                            const alarm2_rate_t option2) {

  int i = 0;
  std::vector<uint8_t> data;

  /* alarm 1 data */
  if (alarms != DS3231_ALARM_2) {
    CHECK_ARG(time1);
    data[i++] = (option1 >= DS3231_ALARM1_MATCH_SEC ? dec2bcd(time1->tm_sec)
                                                    : DS3231_ALARM_NOTSET);
    data[i++] = (option1 >= DS3231_ALARM1_MATCH_SECMIN ? dec2bcd(time1->tm_min)
                                                       : DS3231_ALARM_NOTSET);
    data[i++] =
        (option1 >= DS3231_ALARM1_MATCH_SECMINHOUR ? dec2bcd(time1->tm_hour)
                                                   : DS3231_ALARM_NOTSET);
    data[i++] = (option1 == DS3231_ALARM1_MATCH_SECMINHOURDAY
                     ? (dec2bcd(time1->tm_wday + 1) & DS3231_ALARM_WDAY)
                     : (option1 == DS3231_ALARM1_MATCH_SECMINHOURDATE
                            ? dec2bcd(time1->tm_mday)
                            : DS3231_ALARM_NOTSET));
  }

  /* alarm 2 data */
  if (alarms != DS3231_ALARM_1) {
    CHECK_ARG(time2);
    data[i++] = (option2 >= DS3231_ALARM2_MATCH_MIN ? dec2bcd(time2->tm_min)
                                                    : DS3231_ALARM_NOTSET);
    data[i++] =
        (option2 >= DS3231_ALARM2_MATCH_MINHOUR ? dec2bcd(time2->tm_hour)
                                                : DS3231_ALARM_NOTSET);
    data[i++] = (option2 == DS3231_ALARM2_MATCH_MINHOURDAY
                     ? (dec2bcd(time2->tm_wday + 1) & DS3231_ALARM_WDAY)
                     : (option2 == DS3231_ALARM2_MATCH_MINHOURDATE
                            ? dec2bcd(time2->tm_mday)
                            : DS3231_ALARM_NOTSET));
  }

  return i2c_manager.writeRegister(
      this->deviceHandle,
      (alarms == DS3231_ALARM_2 ? DS3231_ADDR_ALARM2 : DS3231_ADDR_ALARM1),
      data);
}

/* Get a byte containing just the requested bits
 * pass the register address to read, a mask to apply to the register and
 * an uint* for the output
 * you can test this value directly as true/false for specific bit mask
 * of use a mask of 0xff to just return the whole register byte
 * returns true to indicate success
 */
esp_err_t DS3231::get_flag(const uint8_t reg, const uint8_t mask,
                           uint8_t &flag) {
  std::vector<uint8_t> data;

  /* get register */
  esp_err_t res = i2c_manager.readRegister(this->deviceHandle, reg, data, 1);
  if (res != ESP_OK)
    return res;

  /* return only requested flag */
  flag = (data.at(0) & mask);
  return ESP_OK;
}

/* Set/clear bits in a byte register, or replace the byte altogether
 * pass the register address to modify, a byte to replace the existing
 * value with or containing the bits to set/clear and one of
 * DS3231_SET/DS3231_CLEAR/DS3231_REPLACE
 * returns true to indicate success
 */
esp_err_t DS3231::set_flag(const uint8_t reg, const uint8_t bits,
                           const uint8_t mode) {
  std::vector<uint8_t> data;

  /* get status register */
  esp_err_t res = i2c_manager.readRegister(this->deviceHandle, reg, data, 1);
  if (res != ESP_OK)
    return res;
  /* clear the flag */
  if (mode == DS3231_REPLACE)
    data.at(0) = bits;
  else if (mode == DS3231_SET)
    data.at(0) |= bits;
  else
    data.at(0) &= ~bits;

  return i2c_manager.writeRegister(this->deviceHandle, reg, data);
}

esp_err_t DS3231::get_oscillator_stop_flag(bool &flag) {
  CHECK_ARG(flag);

  uint8_t f;

  get_flag(DS3231_ADDR_STATUS, DS3231_STAT_OSCILLATOR, f);

  flag = (f ? true : false);

  return ESP_OK;
}

esp_err_t DS3231::clear_oscillator_stop_flag() {

  return set_flag(DS3231_ADDR_STATUS, DS3231_STAT_OSCILLATOR, DS3231_CLEAR);
}

esp_err_t DS3231::get_alarm_flags(alarm_t &alarms) {
  CHECK_ARG(alarms);

  return get_flag(DS3231_ADDR_STATUS, DS3231_ALARM_BOTH, reinterpret_cast<uint8_t&>(alarms));
}

esp_err_t DS3231::clear_alarm_flags(const alarm_t alarms) {
  return set_flag(DS3231_ADDR_STATUS, alarms, DS3231_CLEAR);
}

esp_err_t DS3231::enable_alarm_ints(const alarm_t alarms) {
  return set_flag(DS3231_ADDR_CONTROL, DS3231_CTRL_ALARM_INTS | alarms,
                  DS3231_SET);
}

esp_err_t DS3231::disable_alarm_ints(const alarm_t alarms) {

  /* Just disable specific alarm(s) requested
   * does not disable alarm interrupts generally (which would enable the
   * squarewave)
   */
  return set_flag(DS3231_ADDR_CONTROL, alarms, DS3231_CLEAR);
}

esp_err_t DS3231::enable_32khz() {
  return set_flag(DS3231_ADDR_STATUS, DS3231_STAT_32KHZ, DS3231_SET);
}

esp_err_t DS3231::disable_32khz() {
  return set_flag(DS3231_ADDR_STATUS, DS3231_STAT_32KHZ, DS3231_CLEAR);
}

esp_err_t DS3231::enable_squarewave() {
  return set_flag(DS3231_ADDR_CONTROL, DS3231_CTRL_ALARM_INTS, DS3231_CLEAR);
}

esp_err_t DS3231::disable_squarewave() {
  return set_flag(DS3231_ADDR_CONTROL, DS3231_CTRL_ALARM_INTS, DS3231_SET);
}

esp_err_t DS3231::set_squarewave_freq(const sqwave_freq_t freq) {
  uint8_t flag = 0;

  get_flag(DS3231_ADDR_CONTROL, 0xff, flag);

  flag &= ~DS3231_SQWAVE_8192HZ;
  flag |= freq;

  return set_flag(DS3231_ADDR_CONTROL, flag, DS3231_REPLACE);
}

esp_err_t DS3231::get_squarewave_freq(sqwave_freq_t &freq) {

  uint8_t flag = 0;

  get_flag(DS3231_ADDR_CONTROL, 0xff, flag);

  flag &= DS3231_SQWAVE_8192HZ;
  freq = static_cast<sqwave_freq_t>(flag);

  return ESP_OK;
}

esp_err_t DS3231::get_raw_temp(int16_t &temp) {
  CHECK_ARG(temp);

  std::vector<uint8_t> data;

  i2c_manager.readRegister(this->deviceHandle, DS3231_ADDR_TEMP, data, 2);

  temp = static_cast<int16_t>(static_cast<int8_t>(data[0])) << 2 | data[1] >> 6;

  return ESP_OK;
}

esp_err_t DS3231::get_temp_integer(int8_t &temp) {
  CHECK_ARG(temp);

  int16_t t_int;

  esp_err_t res = get_raw_temp(t_int);
  if (res == ESP_OK) {
    temp = t_int >> 2;
  }

  return res;
}

esp_err_t DS3231::get_temp_float(float &temp) {
  CHECK_ARG(temp);

  int16_t t_int;

  esp_err_t res = get_raw_temp(t_int);
  if (res == ESP_OK) {
    temp = 0.25 * t_int;
  }

  return res;
}

esp_err_t DS3231::get_time(struct tm &time) {
  std::vector<uint8_t> data;

  i2c_manager.readRegister(this->deviceHandle, DS3231_ADDR_TIME, data, 7);

  /* convert to unix time structure */
  time.tm_sec = bcd2dec(data[0]);
  time.tm_min = bcd2dec(data[1]);
  if (data[2] & DS3231_12HOUR_FLAG) {
    /* 12H */
    time.tm_hour = bcd2dec(data[2] & DS3231_12HOUR_MASK) - 1;
    /* AM/PM? */
    if (data[2] & DS3231_PM_FLAG)
      time.tm_hour += 12;
  } else
    time.tm_hour = bcd2dec(data[2]); /* 24H */
  time.tm_wday = bcd2dec(data[3]) - 1;
  time.tm_mday = bcd2dec(data[4]);
  time.tm_mon = bcd2dec(data[5] & DS3231_MONTH_MASK) - 1;
  time.tm_year = bcd2dec(data[6]) + 100;
  time.tm_isdst = 0;
  time.tm_yday =
      days_since_january_1st(time.tm_year, time.tm_mon, time.tm_mday);

  // apply a time zone (if you are not using localtime on the rtc or you want to
  // check/apply DST)
  // applyTZ(time);

  return ESP_OK;
}

esp_err_t DS3231::set_aging_offset(int8_t age) {

  std::vector<uint8_t> data;
  data.push_back(age);

  i2c_manager.writeRegister(this->deviceHandle, DS3231_ADDR_AGING, data);

  /**
   * To see the effects of the aging register on the 32kHz output
   * frequency immediately, a manual conversion should be started
   * after each aging register change.
   */
  set_flag(DS3231_ADDR_CONTROL, DS3231_CTRL_TEMPCONV, DS3231_SET);

  return ESP_OK;
}

esp_err_t DS3231::get_aging_offset(int8_t &age) {
  CHECK_ARG(age);

  std::vector<uint8_t> data;

  i2c_manager.readRegister(this->deviceHandle, DS3231_ADDR_AGING, data, 1);

  age = static_cast<int8_t>(data.at(0));

  return ESP_OK;
}

uint8_t DS3231::bcd2dec(uint8_t val) { return (val >> 4) * 10 + (val & 0x0f); }

uint8_t DS3231::dec2bcd(uint8_t val) { return ((val / 10) << 4) + (val % 10); }

// Function to convert year, month, and day to days since January 1st

inline int DS3231::days_since_january_1st(int year, int month, int day) {
  int days = day - 1;
  const int *ptr = days_per_month;

  // Handle leap year
  if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))
    ptr = days_per_month_leap_year;

  // Add days from previous months
  for (int i = 0; i < month; i++) {
    days += ptr[i];
  }

  return days;
}