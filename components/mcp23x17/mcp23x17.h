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
 * @file mcp23x17.h
 * @defgroup mcp23x17 mcp23x17
 * @{
 *
 * ESP-IDF driver for I2C/SPI 16 bit GPIO expanders MCP23017/MCP23S17
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __MCP23X17_H__
#define __MCP23X17_H__

#include "I2CManager.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <cstdbool>
#include <cstdint>
#include <mutex>
#include <vector>

#define MCP23X17_ADDR_BASE 0x20

/** mcp23x17 class
 *
 *  This is a driver for the MCP23008 I2C 8-bit multiplexer
 *
 *  Example:
 *  @code
 *
 *  @endcode
 */
class MCP23x17 {
public:
  /**
   * GPIO mode
   */
  typedef enum { MCP23X17_GPIO_OUTPUT = 0, MCP23X17_GPIO_INPUT } gpio_mode_t;

  /**
   * INTA/INTB pins mode
   */
  typedef enum {
    MCP23X17_ACTIVE_LOW = 0, //!< Low level on interrupt
    MCP23X17_ACTIVE_HIGH,    //!< High level on interrupt
    MCP23X17_OPEN_DRAIN      //!< Open drain
  } int_out_mode_t;

  /**
   * Interrupt mode
   */
  typedef enum {
    MCP23X17_INT_DISABLED = 0, //!< No interrupt
    MCP23X17_INT_LOW_EDGE,     //!< Interrupt on low edge
    MCP23X17_INT_HIGH_EDGE,    //!< Interrupt on high edge
    MCP23X17_INT_ANY_EDGE      //!< Interrupt on any edge
  } gpio_intr_t;

  /** Create an MCP23x17 instance connected to specified I2C pins with specified
   * address
   *
   * @param i2c_port The I2C port to use (default: 0)
   * @param i2c_address I2C-bus address (default: 0x20)
   */
  MCP23x17(const I2CManager &i2cmgr, const uint8_t i2c_address);

  ~MCP23x17();

  /**
   * @brief Get INTA/INTB pins mode
   *
   * @param dev Pointer to device descriptor
   * @param[out] mode Buffer to store mode
   * @return `ESP_OK` on success
   */
  esp_err_t get_int_out_mode(int_out_mode_t &mode);

  /**
   * @brief Set INTA/INTB pins mode
   *
   * @param dev Pointer to device descriptor
   * @param mode INTA/INTB pins mode
   * @return `ESP_OK` on success
   */
  esp_err_t set_int_out_mode(int_out_mode_t mode);

  /**
   * @brief Get GPIO pins mode
   *
   * 0 - output, 1 - input for each bit in `val`
   *
   * @param dev Pointer to device descriptor
   * @param[out] val Buffer to store mode, 0 bit for PORTA/GPIO0..15 bit for
   * PORTB/GPIO7
   * @return
   */
  esp_err_t port_get_mode(uint16_t &val);

  /**
   * @brief Set GPIO pins mode
   *
   * 0 - output, 1 - input for each bit in `val`
   *
   * @param dev Pointer to device descriptor
   * @param val Mode, 0 bit for PORTA/GPIO0..15 bit for PORTB/GPIO7
   * @return `ESP_OK` on success
   */
  esp_err_t port_set_mode(uint16_t val);

  /**
   * @brief Get GPIO pullups status
   *
   * 0 - pullup disabled, 1 - pullup enabled for each bit in `val`
   *
   * @param dev Pointer to device descriptor
   * @param[out] val Pullup status, 0 bit for PORTA/GPIO0..15 bit for
   * PORTB/GPIO7
   * @return `ESP_OK` on success
   */
  esp_err_t port_get_pullup(uint16_t &val);

  /**
   * @brief Set GPIO pullups status
   *
   * 0 - pullup disabled, 1 - pullup enabled for each bit in `val`
   *
   * @param dev Pointer to device descriptor
   * @param val Pullup status, 0 bit for PORTA/GPIO0..15 bit for PORTB/GPIO7
   * @return `ESP_OK` on success
   */
  esp_err_t port_set_pullup(uint16_t val);

  /**
   * @brief Read GPIO port value
   *
   * @param dev Pointer to device descriptor
   * @param[out] val 16-bit GPIO port value, 0 bit for PORTA/GPIO0..15 bit for
   * PORTB/GPIO7
   * @return `ESP_OK` on success
   */
  esp_err_t port_read(uint16_t &val);

  /**
   * @brief Write value to GPIO port
   *
   * @param dev Pointer to device descriptor
   * @param val GPIO port value, 0 bit for PORTA/GPIO0..15 bit for PORTB/GPIO7
   * @return `ESP_OK` on success
   */
  esp_err_t port_write(uint16_t val);

  /**
   * @brief Get GPIO pin mode
   *
   * @param dev Pointer to device descriptor
   * @param pin Pin number, 0 for PORTA/GPIO0..15 for PORTB/GPIO7
   * @param[out] mode GPIO pin mode
   * @return `ESP_OK` on success
   */
  esp_err_t get_mode(uint8_t pin, gpio_mode_t &mode);

  /**
   * @brief Set GPIO pin mode
   *
   * @param dev Pointer to device descriptor
   * @param pin Pin number, 0 for PORTA/GPIO0..15 for PORTB/GPIO7
   * @param mode GPIO pin mode
   * @return `ESP_OK` on success
   */
  esp_err_t set_mode(uint8_t pin, gpio_mode_t mode);

  /**
   * @brief Get pullup mode of GPIO pin
   *
   * @param dev Pointer to device descriptor
   * @param pin Pin number, 0 for PORTA/GPIO0..15 for PORTB/GPIO7
   * @param[out] enable pullup mode
   * @return `ESP_OK` on success
   */
  esp_err_t get_pullup(uint8_t pin, bool &enable);

  /**
   * @brief Set pullup mode of GPIO pin
   *
   * @param dev Pointer to device descriptor
   * @param pin Pin number, 0 for PORTA/GPIO0..15 for PORTB/GPIO7
   * @param enable `true` to enable pullup
   * @return `ESP_OK` on success
   */
  esp_err_t set_pullup(uint8_t pin, bool enable);

  /**
   * @brief Read GPIO pin level
   *
   * @param dev Pointer to device descriptor
   * @param pin Pin number, 0 for PORTA/GPIO0..15 for PORTB/GPIO7
   * @param[out] val `true` if pin currently in high state
   * @return `ESP_OK` on success
   */
  esp_err_t get_level(uint8_t pin, uint32_t &val);

  /**
   * @brief Set GPIO pin level
   *
   * Pin must be set up as output
   *
   * @param dev Pointer to device descriptor
   * @param pin Pin number, 0 for PORTA/GPIO0..15 for PORTB/GPIO7
   * @param[out] val `true` if pin currently in high state
   * @return `ESP_OK` on success
   */
  esp_err_t set_level(uint8_t pin, uint32_t val);

  /**
   * @brief Setup interrupt for group of GPIO pins
   *
   * @param dev Pointer to device descriptor
   * @param mask Pins to setup
   * @param intr Interrupt mode
   * @return `ESP_OK` on success
   */
  esp_err_t port_set_interrupt(uint16_t mask, gpio_intr_t intr);

  /**
   * @brief Setup interrupt for GPIO pin
   *
   * @param dev Pointer to device descriptor
   * @param pin Pin number, 0 for PORTA/GPIO0..15 for PORTB/GPIO7
   * @param intr Interrupt mode
   * @return `ESP_OK` on success
   */
  esp_err_t set_interrupt(uint8_t pin, gpio_intr_t intr);

private:
  static constexpr int I2C_FREQ_HZ =
      1000000; // Max 1MHz for esp-idf, but device supports up to 1.7Mhz

  static constexpr uint8_t REG_IODIRA = 0x00;
  static constexpr uint8_t REG_IODIRB = 0x01;
  static constexpr uint8_t REG_IPOLA = 0x02;
  static constexpr uint8_t REG_IPOLB = 0x03;
  static constexpr uint8_t REG_GPINTENA = 0x04;
  static constexpr uint8_t REG_GPINTENB = 0x05;
  static constexpr uint8_t REG_DEFVALA = 0x06;
  static constexpr uint8_t REG_DEFVALB = 0x07;
  static constexpr uint8_t REG_INTCONA = 0x08;
  static constexpr uint8_t REG_INTCONB = 0x09;
  static constexpr uint8_t REG_IOCON = 0x0A;
  static constexpr uint8_t REG_GPPUA = 0x0C;
  static constexpr uint8_t REG_GPPUB = 0x0D;
  static constexpr uint8_t REG_INTFA = 0x0E;
  static constexpr uint8_t REG_INTFB = 0x0F;
  static constexpr uint8_t REG_INTCAPA = 0x10;
  static constexpr uint8_t REG_INTCAPB = 0x11;
  static constexpr uint8_t REG_GPIOA = 0x12;
  static constexpr uint8_t REG_GPIOB = 0x13;
  static constexpr uint8_t REG_OLATA = 0x14;
  static constexpr uint8_t REG_OLATB = 0x15;

  static constexpr uint8_t BIT_IOCON_INTPOL = 1;
  static constexpr uint8_t BIT_IOCON_ODR = 2;
  static constexpr uint8_t BIT_IOCON_HAEN = 3;
  static constexpr uint8_t BIT_IOCON_DISSLW = 4;
  static constexpr uint8_t BIT_IOCON_SEQOP = 5;
  static constexpr uint8_t BIT_IOCON_MIRROR = 6;
  static constexpr uint8_t BIT_IOCON_BANK = 7;

private:
  esp_err_t read_reg_16(const uint8_t reg, uint16_t &val);
  esp_err_t write_reg_16(const uint8_t reg, const uint16_t val);
  esp_err_t write_reg_bit_16(const uint8_t reg, bool val, uint8_t bit);
  esp_err_t read_reg_bit_8(const uint8_t reg, bool &val, uint8_t bit);
  esp_err_t write_reg_bit_8(const uint8_t reg, const bool val,
                            const uint8_t bit);
  esp_err_t read_reg_bit_16(const uint8_t reg, bool &val, const uint8_t bit);

  i2c_device_config_t deviceConfig;
  i2c_master_dev_handle_t deviceHandle;

  I2CManager i2c_manager;

  std::mutex _mutex;
};

#endif /* __MCP23X17_H__ */
