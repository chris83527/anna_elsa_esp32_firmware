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
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file mcp23008.h
 * @defgroup mcp23008 mcp23008
 * @{
 *
 * ESP-IDF driver for I2C 8 bit GPIO expander MCP23008
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __MCP23008_H__
#define __MCP23008_H__

#include <cstdbool>
#include <i2c_manager.h>
#include <driver/gpio.h>
#include <esp_err.h>

#define MCP23008_I2C_ADDR_BASE 0x20

class MCP23008 {
public:

    /**
     * GPIO mode
     */
    typedef enum {
        MCP23008_GPIO_OUTPUT = 0,
        MCP23008_GPIO_INPUT
    } gpio_mode_t;

    /**
     * INTA/INTB pins mode
     */
    typedef enum {
        MCP23008_ACTIVE_LOW = 0, //!< Low level on interrupt
        MCP23008_ACTIVE_HIGH, //!< High level on interrupt
        MCP23008_OPEN_DRAIN //!< Open drain
    } int_out_mode_t;

    /**
     * Interrupt mode
     */
    typedef enum {
        MCP23008_INT_DISABLED = 0, //!< No interrupt
        MCP23008_INT_LOW_EDGE, //!< Interrupt on low edge
        MCP23008_INT_HIGH_EDGE, //!< Interrupt on high edge
        MCP23008_INT_ANY_EDGE //!< Interrupt on any edge
    } gpio_intr_t;

    /** Create an MCP23008 instance connected to specified I2C pins with specified address
     *            
     * @param i2c_port The I2C port to use (default: 0)
     * @param i2c_address I2C-bus address (default: 0x20)     
     */
    MCP23008(const i2c_port_t port, const uint8_t address);
    ~MCP23008();

    /**
     * @brief Get INT pins mode
     *
     * @param dev Pointer to I2C device descriptor
     * @param[out] mode Buffer to store mode
     * @return `ESP_OK` on success
     */
    esp_err_t get_int_out_mode(int_out_mode_t& mode);

    /**
     * @brief Set INT pins mode
     *
     * @param dev Pointer to I2C device descriptor
     * @param mode INT pins mode
     * @return `ESP_OK` on success
     */
    esp_err_t set_int_out_mode(int_out_mode_t mode);

    /**
     * @brief Get GPIO pins mode
     *
     * 0 - output, 1 - input for each bit in `val`
     *
     * @param dev Pointer to I2C device descriptor
     * @param[out] val Buffer to store mode, 0 bit for GPIO0..7 bit for GPIO7
     * @return
     */
    esp_err_t port_get_mode(uint8_t& val);

    /**
     * @brief Set GPIO pins mode
     *
     * 0 - output, 1 - input for each bit in `val`
     *
     * @param dev Pointer to I2C device descriptor
     * @param val Mode, 0 bit for GPIO0..7 bit for GPIO7
     * @return `ESP_OK` on success
     */
    esp_err_t port_set_mode(const uint8_t val);

    /**
     * @brief Get GPIO pullups status
     *
     * 0 - pullup disabled, 1 - pullup enabled for each bit in `val`
     *
     * @param dev Pointer to I2C device descriptor
     * @param[out] val Pullup status, 0 bit for GPIO0..7 bit for GPIO7
     * @return `ESP_OK` on success
     */
    esp_err_t port_get_pullup(uint8_t& val);

    /**
     * @brief Set GPIO pullups status
     *
     * 0 - pullup disabled, 1 - pullup enabled for each bit in `val`
     *
     * @param dev Pointer to I2C device descriptor
     * @param val Pullup status, 0 bit for GPIO0..7 bit for GPIO7
     * @return `ESP_OK` on success
     */
    esp_err_t port_set_pullup(const uint8_t val);

    /**
     * @brief Read GPIO port value
     *
     * @param dev Pointer to I2C device descriptor
     * @param[out] val 8-bit GPIO port value, 0 bit for GPIO0..7 bit for GPIO7
     * @return `ESP_OK` on success
     */
    esp_err_t port_read(uint8_t& val);

    /**
     * @brief Write value to GPIO port
     *
     * @param dev Pointer to I2C device descriptor
     * @param val GPIO port value, 0 bit for GPIO0..7 bit for GPIO7
     * @return `ESP_OK` on success
     */
    esp_err_t port_write(const uint8_t val);

    /**
     * @brief Get GPIO pin mode
     *
     * @param dev Pointer to I2C device descriptor
     * @param pin Pin number, 0..7
     * @param[out] mode GPIO pin mode
     * @return `ESP_OK` on success
     */
    esp_err_t get_mode(const uint8_t pin, gpio_mode_t& mode);

    /**
     * @brief Set GPIO pin mode
     *
     * @param dev Pointer to I2C device descriptor
     * @param pin Pin number, 0..7
     * @param mode GPIO pin mode
     * @return `ESP_OK` on success
     */
    esp_err_t set_mode(const uint8_t pin, const gpio_mode_t mode);

    /**
     * @brief Get pullup mode of GPIO pin
     *
     * @param dev Pointer to I2C device descriptor
     * @param pin Pin number, 0..7
     * @param[out] enable pullup mode
     * @return `ESP_OK` on success
     */
    esp_err_t get_pullup(const uint8_t pin, bool& enable);

    /**
     * @brief Set pullup mode of GPIO pin
     *
     * @param dev Pointer to I2C device descriptor
     * @param pin Pin number, 0..7
     * @param enable `true` to enable pullup
     * @return `ESP_OK` on success
     */
    esp_err_t set_pullup(const uint8_t pin, const bool enable);

    /**
     * @brief Read GPIO pin level
     *
     * @param dev Pointer to I2C device descriptor
     * @param pin Pin number, 0..7
     * @param[out] val `true` if pin currently in high state
     * @return `ESP_OK` on success
     */
    esp_err_t get_level(const uint8_t pin, uint32_t& val);

    /**
     * @brief Set GPIO pin level
     *
     * Pin must be set up as output.
     *
     * @param dev Pointer to I2C device descriptor
     * @param pin Pin number, 0..7
     * @param[out] val `true` if pin currently in high state
     * @return `ESP_OK` on success
     */
    esp_err_t set_level(const uint8_t pin, const uint32_t val);

    /**
     * @brief Setup interrupt for group of GPIO pins
     *
     * @param dev Pointer to I2C device descriptor
     * @param mask Pins to setup
     * @param intr Interrupt mode
     * @return `ESP_OK` on success
     */
    esp_err_t port_set_interrupt(const uint8_t mask, const gpio_intr_t intr);

    /**
     * @brief Setup interrupt for GPIO pin
     *
     * @param dev Pointer to I2C device descriptor
     * @param pin Pin number, 0..7
     * @param intr Interrupt mode
     * @return `ESP_OK` on success
     */
    esp_err_t set_interrupt(const uint8_t pin, const gpio_intr_t intr);

private:
    
    esp_err_t read_reg(const uint8_t reg, uint8_t& val);
    esp_err_t write_reg(const uint8_t reg, uint8_t val);
    esp_err_t read_reg_bit(const uint8_t reg, bool& val, const uint8_t bit);
    esp_err_t write_reg_bit(const uint8_t reg, bool val, const uint8_t bit);
    
    i2c_port_t i2c_port;
    uint8_t i2c_address;
};

#endif /* __MCP23008_H__ */
