/*
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file mcp23x17.c
 *
 * ESP-IDF driver for I2C/SPI 16 bit GPIO expanders MCP23017/MCP23S17
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */

#include <esp_log.h>
#include <string.h>

#include "mcp23x17.h"

static const char *TAG = "mcp23x17";

#define CHECK(x)                                                               \
  do {                                                                         \
    esp_err_t __;                                                              \
    if ((__ = x) != ESP_OK)                                                    \
      return __;                                                               \
  } while (0)
#define CHECK_ARG(VAL)                                                         \
  do {                                                                         \
    if (!(VAL))                                                                \
      return ESP_ERR_INVALID_ARG;                                              \
  } while (0)
#define BV(x) (1 << (x))

MCP23x17::MCP23x17(const I2CManager &i2cmgr, const uint8_t address)
    : i2c_manager{i2cmgr} {
  ESP_LOGD(TAG, "i2c_address: %d", address);
  this->deviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  this->deviceConfig.device_address = address;
  this->deviceConfig.scl_speed_hz = I2C_FREQ_HZ;

  this->i2c_manager.addDevice(this->deviceConfig, this->deviceHandle);
}

MCP23x17::~MCP23x17() {}

esp_err_t MCP23x17::getGPIOExpanderConfiguration(uint8_t &mode) {

  bool buf;
  CHECK(readRegisterBit8(REG_IOCON, buf, BIT_IOCON_ODR));
  if (buf) {
    mode = MCP23X17_OPEN_DRAIN;
    return ESP_OK;
  }
  CHECK(readRegisterBit8(REG_IOCON, buf, BIT_IOCON_INTPOL));
  mode = buf ? MCP23X17_ACTIVE_HIGH : MCP23X17_ACTIVE_LOW;

  return ESP_OK;
}

esp_err_t MCP23x17::setGPIOExpanderConfiguration(const uint8_t mode) {
  if (mode == MCP23X17_OPEN_DRAIN)
    return writeRegisterBit8(REG_IOCON, true, BIT_IOCON_ODR);

  // The INTPOL bit is only functional if the ODR bit is cleared.
  writeRegisterBit8(REG_IOCON, false, BIT_IOCON_ODR);
  return writeRegisterBit8(REG_IOCON, mode == MCP23X17_ACTIVE_HIGH,
                           BIT_IOCON_INTPOL);
}

esp_err_t MCP23x17::getGPIOAInputOutputMode(uint8_t &val) {

  return readRegister8(REG_IODIRA, val);
}

esp_err_t MCP23x17::getGPIOBInputOutputMode(uint8_t &val) {

  return readRegister8(REG_IODIRB, val);
}

esp_err_t MCP23x17::setGPIOAInputOutputMode(const uint8_t val) {
  return writeRegister8(REG_IODIRA, val);
}

esp_err_t MCP23x17::setGPIOBInputOutputMode(const uint8_t val) {
  return writeRegister8(REG_IODIRB, val);
}

esp_err_t MCP23x17::getGPIOAPullup(uint8_t &val) {
  return readRegister8(REG_GPPUA, val);
}

esp_err_t MCP23x17::setGPIOAPullup(const uint8_t val) {
  return writeRegister8(REG_GPPUA, val);
}

esp_err_t MCP23x17::getGPIOBPullup(uint8_t &val) {
  return readRegister8(REG_GPPUB, val);
}

esp_err_t MCP23x17::setGPIOBPullup(const uint8_t val) {
  return writeRegister8(REG_GPPUB, val);
}

esp_err_t MCP23x17::readGPIOA(uint8_t &val) {
  return readRegister8(REG_GPIOA, val);
}
esp_err_t MCP23x17::readGPIOB(uint8_t &val) {
  return readRegister8(REG_GPIOB, val);
}
esp_err_t MCP23x17::writeGPIOA(uint8_t val) {
  return writeRegister8(REG_GPIOA, val);
}
esp_err_t MCP23x17::writeGPIOB(uint8_t val) {
  return writeRegister8(REG_GPIOB, val);
}

esp_err_t MCP23x17::setGPIOAInputPolarity(uint8_t val) {
  return writeRegister8(REG_IPOLA, val);
}

esp_err_t MCP23x17::setGPIOBInputPolarity(uint8_t val) {
  return writeRegister8(REG_IPOLB, val);
}

esp_err_t MCP23x17::getGPIOAInputPolarity(uint8_t &val) {
  return readRegister8(REG_IPOLA, val);
}

esp_err_t MCP23x17::getGPIOBInputPolarity(uint8_t &val) {
  return readRegister8(REG_IPOLB, val);
}

esp_err_t MCP23x17::getGPIOAPinMode(const uint8_t pin, gpio_mode_t &mode) {

  bool buf;
  CHECK(readRegisterBit8(REG_IODIRA, buf, pin));
  mode = buf ? MCP23X17_GPIO_INPUT : MCP23X17_GPIO_OUTPUT;

  return ESP_OK;
}

esp_err_t MCP23x17::getGPIOBPinMode(const uint8_t pin, gpio_mode_t &mode) {

  bool buf;
  CHECK(readRegisterBit8(REG_IODIRB, buf, pin));
  mode = buf ? MCP23X17_GPIO_INPUT : MCP23X17_GPIO_OUTPUT;

  return ESP_OK;
}

esp_err_t MCP23x17::setGPIOAPinMode(const uint8_t pin, const gpio_mode_t mode) {
  return writeRegisterBit8(REG_IODIRA, mode, pin);
}

esp_err_t MCP23x17::setGPIOBPinMode(const uint8_t pin, const gpio_mode_t mode) {
  return writeRegisterBit8(REG_IODIRB, mode, pin);
}

esp_err_t MCP23x17::getGPIOAPinLevel(uint8_t pin, bool &val) {

  uint8_t buf;
  CHECK(readRegister8(REG_GPIOA, buf));
  val = ((buf & pin) == 1) ? true : false;

  return ESP_OK;
}

esp_err_t MCP23x17::getGPIOBPinLevel(uint8_t pin, bool &val) {

  uint8_t buf;
  CHECK(readRegister8(REG_GPIOB, buf));
  val = ((buf & pin) == 1) ? true : false;

  return ESP_OK;
}

esp_err_t MCP23x17::setGPIOAPinLevel(const uint8_t pin, const bool val) {
  return writeRegisterBit8(REG_GPIOA, val, pin);
}

esp_err_t MCP23x17::setGPIOBPinLevel(const uint8_t pin, const bool val) {
  return writeRegisterBit8(REG_GPIOB, val, pin);
}

esp_err_t MCP23x17::setGPIOAInterrupt(const uint8_t mask,
                                      const gpio_intr_t intr) {
  uint8_t int_en;
  CHECK(readRegister8(REG_GPINTENA, int_en));

  if (intr == MCP23X17_INT_DISABLED) {
    // disable interrupts
    int_en &= ~mask;
    CHECK(writeRegister8(REG_GPINTENA, int_en));

    return ESP_OK;
  }

  uint8_t int_con;
  CHECK(readRegister8(REG_INTCONA, int_con));

  if (intr == MCP23X17_INT_ANY_EDGE)
    int_con &= ~mask;
  else {
    int_con |= mask;

    uint8_t int_def;
    CHECK(readRegister8(REG_DEFVALA, int_def));
    if (intr == MCP23X17_INT_LOW_EDGE)
      int_def |= mask;
    else
      int_def &= ~mask;
    CHECK(writeRegister8(REG_DEFVALA, int_def));
  }

  CHECK(writeRegister8(REG_INTCONA, int_con));

  // enable interrupts
  int_en |= mask;
  CHECK(writeRegister8(REG_GPINTENA, int_en));

  return ESP_OK;
}

esp_err_t MCP23x17::setGPIOBInterrupt(const uint8_t mask,
                                      const gpio_intr_t intr) {
  uint8_t int_en;
  CHECK(readRegister8(REG_GPINTENB, int_en));

  if (intr == MCP23X17_INT_DISABLED) {
    // disable interrupts
    int_en &= ~mask;
    CHECK(writeRegister8(REG_GPINTENB, int_en));

    return ESP_OK;
  }

  uint8_t int_con;
  CHECK(readRegister8(REG_INTCONB, int_con));

  if (intr == MCP23X17_INT_ANY_EDGE)
    int_con &= ~mask;
  else {
    int_con |= mask;

    uint8_t int_def;
    CHECK(readRegister8(REG_DEFVALB, int_def));
    if (intr == MCP23X17_INT_LOW_EDGE)
      int_def |= mask;
    else
      int_def &= ~mask;
    CHECK(writeRegister8(REG_DEFVALB, int_def));
  }

  CHECK(writeRegister8(REG_INTCONB, int_con));

  // enable interrupts
  int_en |= mask;
  CHECK(writeRegister8(REG_GPINTENB, int_en));

  return ESP_OK;
}

esp_err_t MCP23x17::setGPIOAPinInterrupt(uint8_t pin, gpio_intr_t intr) {
  return setGPIOAInterrupt(BV(pin), intr);
}

esp_err_t MCP23x17::setGPIOBPinInterrupt(uint8_t pin, gpio_intr_t intr) {
  return setGPIOBInterrupt(BV(pin), intr);
}

// ---------------------

esp_err_t MCP23x17::readRegister8(const uint8_t reg, uint8_t &val) {
  _mutex.lock();

  std::vector<uint8_t> data;

  esp_err_t res =
      this->i2c_manager.readRegister(this->deviceHandle, reg, data, 1);

  val = data[0];

  _mutex.unlock();

  return res;
}

esp_err_t MCP23x17::readRegister16(const uint8_t reg, uint16_t &val) {
  _mutex.lock();
  std::vector<uint8_t> data;

  esp_err_t res =
      this->i2c_manager.readRegister(this->deviceHandle, reg, data, 2);

  val = (data[1] << 8 | data[0]);

  _mutex.unlock();

  return res;
}

esp_err_t MCP23x17::readRegisterBit16(const uint8_t reg, bool &val,
                                      const uint16_t bit) {
  uint16_t buf;

  CHECK(readRegister16(reg, buf));

  val = (buf & BV(bit)) >> bit;

  return ESP_OK;
}

esp_err_t MCP23x17::readRegisterBit8(const uint8_t reg, bool &val,
                                     uint8_t bit) {
  _mutex.lock();
  std::vector<uint8_t> data;

  esp_err_t ret =
      this->i2c_manager.readRegister(this->deviceHandle, reg, data, 1);

  val = (data.at(0) & BV(bit)) >> bit;
  _mutex.unlock();
  return ret;
}

esp_err_t MCP23x17::writeRegister16(const uint8_t reg, const uint16_t val) {
  _mutex.lock();
  std::vector<uint8_t> data;
  data.push_back((val & 0xff));
  data.push_back((val >> 8));

  esp_err_t ret =
      this->i2c_manager.writeRegister(this->deviceHandle, reg, data);
  _mutex.unlock();

  return ret;
}

esp_err_t MCP23x17::writeRegisterBit16(const uint8_t reg, bool val,
                                       uint16_t bit) {
  _mutex.lock();
  std::vector<uint8_t> data;

  esp_err_t ret =
      this->i2c_manager.readRegister(this->deviceHandle, reg, data, 2);

  uint16_t buf16 = (data[1] << 8 | data[0]);

  buf16 = (buf16 & ~BV(bit)) | (val ? BV(bit) : 0);

  std::vector<uint8_t> writeData;
  writeData.push_back((buf16 & 0xff));
  writeData.push_back((buf16 >> 8));
  ret &= this->i2c_manager.writeRegister(this->deviceHandle, reg, writeData);

  _mutex.unlock();

  return ret;
}

esp_err_t MCP23x17::writeRegister8(const uint8_t reg, const uint8_t val) {

  _mutex.lock();
  std::vector<uint8_t> data;
  data.push_back(val);

  esp_err_t ret =
      this->i2c_manager.writeRegister(this->deviceHandle, reg, data);

  _mutex.unlock();

  return ret;
}

esp_err_t MCP23x17::writeRegisterBit8(const uint8_t reg, const bool val,
                                      const uint8_t bit) {
  _mutex.lock();

  std::vector<uint8_t> data;

  this->i2c_manager.readRegister(this->deviceHandle, reg, data, 1);
  data.at(0) = (data.at(0) & ~BV(bit)) | (val ? BV(bit) : 0);
  esp_err_t ret =
      this->i2c_manager.writeRegister(this->deviceHandle, reg, data);

  _mutex.unlock();

  return ret;
}
