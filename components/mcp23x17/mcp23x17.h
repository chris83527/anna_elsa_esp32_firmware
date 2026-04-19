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

#include "typed_i2c_device.hpp"
#include "esp_err.h"
#include <mutex>

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
class MCP23x17 : public TypedI2CDevice
{
public:
    /**
     * GPIO mode
     */
    typedef enum { MCP23X17_GPIO_OUTPUT = 0, MCP23X17_GPIO_INPUT } gpio_mode_t;

    /**
     * INTA/INTB pins mode
     */
    typedef enum
    {
        MCP23X17_ACTIVE_LOW = 0, //!< Low level on interrupt
        MCP23X17_ACTIVE_HIGH, //!< High level on interrupt
        MCP23X17_OPEN_DRAIN //!< Open drain
    } in_out_mode_t;

    typedef enum { MCP23X17_SAME = 0, MCP23X17_OPPOSITE } direction_mode_t;

    /**
     * Interrupt mode
     */
    typedef enum
    {
        MCP23X17_INT_DISABLED = 0, //!< No interrupt
        MCP23X17_INT_LOW_EDGE, //!< Interrupt on low edge
        MCP23X17_INT_HIGH_EDGE, //!< Interrupt on high edge
        MCP23X17_INT_ANY_EDGE //!< Interrupt on any edge
    } gpio_intr_t;

    /** Create an MCP23x17 instance connected to specified I2C pins with specified
     * address
     *
     * @param i2cmgr The I2C port to use (default: 0)
     * @param i2c_address I2C-bus address (default: 0x20)
     */
    MCP23x17(I2CBus& i2c_bus, uint8_t address) : TypedI2CDevice(i2c_bus, address) {}
    ~MCP23x17() = default;

    esp_err_t setGPIOAInputPolarity(uint8_t polarity);
    esp_err_t setGPIOBInputPolarity(uint8_t polarity);

    esp_err_t getGPIOAInputPolarity(uint8_t& polarity);
    esp_err_t getGPIOBInputPolarity(uint8_t& polarity);

    esp_err_t getGPIOAPinInputPolarity(uint8_t pin,
                                       direction_mode_t& polarity);
    esp_err_t getGPIOBPinInputPolarity(uint8_t pin,
                                       direction_mode_t& polarity);

    /**
     * @brief Get the current value of the I/O expander configuration register
     * (ADDR 0x05)
     *
     */
    esp_err_t getGPIOExpanderConfiguration(uint8_t& config);

    /**
     * @brief Set the value of the I/O expander configuration register
     * (ADDR 0x05)
     *
     */
    esp_err_t setGPIOExpanderConfiguration(const uint8_t config);

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
    esp_err_t getGPIOAInputOutputMode(uint8_t& val);
    esp_err_t getGPIOBInputOutputMode(uint8_t& val);

    /**
     * @brief Set GPIO pins mode
     *
     * 0 - output, 1 - input for each bit in `val`
     *
     * @param dev Pointer to device descriptor
     * @param val Mode, 0 bit for PORTA/GPIO0..15 bit for PORTB/GPIO7
     * @return `ESP_OK` on success
     */
    esp_err_t setGPIOAInputOutputMode(const uint8_t val);
    esp_err_t setGPIOBInputOutputMode(const uint8_t val);

    /**
     * @brief Get GPIO A pullups status
     *
     * 0 - pullup disabled, 1 - pullup enabled for each bit in `val`
     *
     * @param dev Pointer to device descriptor
     * @param[out] val Pullup status, 0..7
     * @return `ESP_OK` on success
     */
    esp_err_t getGPIOAPullup(uint8_t& val);

    /**
     * @brief Get GPIO B pullups status
     *
     * 0 - pullup disabled, 1 - pullup enabled for each bit in `val`
     *
     * @param[out] val Pullup status, 0..7
     * @return `ESP_OK` on success
     */
    esp_err_t getGPIOBPullup(uint8_t& val);

    /**
     * @brief Set GPIO A pullups status
     *
     * 0 - pullup disabled, 1 - pullup enabled for each bit in `val`
     *
     * @param val Pullup status, 0 bit for PORTA/GPIO0..15 bit for PORTB/GPIO7
     * @return `ESP_OK` on success
     */
    esp_err_t setGPIOAPullup(const uint8_t val);
    esp_err_t setGPIOBPullup(const uint8_t val);

    /**
     * @brief Read value from GPIO port A
     *
     * @param val Reference to variable to store value
     * @return `ESP_OK` on success
     */
    esp_err_t readGPIOA(uint8_t& val);

    /**
     * @brief Read value from GPIO port B
     *
     * @param val Reference to variable to store value
     * @return `ESP_OK` on success
     */
    esp_err_t readGPIOB(uint8_t& val);

    /**
     * @brief Write value to GPIO port A
     *
     * @param val Reference to variable to store value
     * @return `ESP_OK` on success
     */
    esp_err_t writeGPIOA(const uint8_t val);

    /**
     * @brief Write value to GPIO port B
     *
     * @param val GPIO port value
     * @return `ESP_OK` on success
     */
    esp_err_t writeGPIOB(const uint8_t val);

    /**
     * @brief Get GPIO pin mode for GPIO A
     *
     * @param dev Pointer to device descriptor
     * @param pin Pin number, 0..7 of the port
     * @param[out] mode GPIO pin mode
     * @return `ESP_OK` on success
     */
    esp_err_t getGPIOAPinMode(uint8_t pin, gpio_mode_t& mode);

    /**
     * @brief Get GPIO pin mode for GPIO B
     *
     * @param pin Pin number, 0..7 of the port
     * @param[out] mode GPIO pin mode
     * @return `ESP_OK` on success
     */
    esp_err_t getGPIOBPinMode(uint8_t pin, gpio_mode_t& mode);

    /**
     * @brief Set GPIO pin mode
     *
     * @param pin Pin number, 0 for PORTA/GPIO0..15 for PORTB/GPIO7
     * @param mode GPIO pin mode
     * @return `ESP_OK` on success
     */
    esp_err_t setGPIOAPinMode(const uint8_t pin, const gpio_mode_t mode);
    esp_err_t setGPIOBPinMode(const uint8_t pin, const gpio_mode_t mode);

    /**
     * @brief Read GPIO pin level
     *
     * @param dev Pointer to device descriptor
     * @param pin Pin number, 0 for PORTA/GPIO0..15 for PORTB/GPIO7
     * @param[out] val `true` if pin currently in high state
     * @return `ESP_OK` on success
     */
    esp_err_t getGPIOAPinLevel(uint8_t pin, bool& val);
    esp_err_t getGPIOBPinLevel(uint8_t pin, bool& val);

    /**
     * @brief Set GPIO pin level
     *
     * Pin must be set up as output
     *
     * @param pin Pin number, 0..7
     * @param[out] val `true` if pin currently in high state
     * @return `ESP_OK` on success
     */
    esp_err_t setGPIOAPinLevel(const uint8_t pin, const bool val);
    esp_err_t setGPIOBPinLevel(const uint8_t pin, const bool val);

    /**
     * @brief Setup interrupt for group of GPIO pins
     *
     * @param mask Pins to setup (0..7)
     * @param intr Interrupt mode
     * @return `ESP_OK` on success
     */
    esp_err_t setGPIOAInterrupt(const uint8_t mask, const gpio_intr_t intr);
    esp_err_t setGPIOBInterrupt(const uint8_t mask, const gpio_intr_t intr);

    /**
     * @brief Setup interrupt for GPIO pin
     *
     * @param pin Pin number, 0..7
     * @param intr Interrupt mode
     * @return `ESP_OK` on success
     */
    esp_err_t setGPIOAPinInterrupt(const uint8_t pin, const gpio_intr_t intr);
    esp_err_t setGPIOBPinInterrupt(const uint8_t pin, const gpio_intr_t intr);

private:
    static constexpr int I2C_FREQ_HZ =
        400000; // Max 1MHz for esp-idf, but device supports up to 1.7Mhz

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
    esp_err_t readRegister8(const uint8_t reg, uint8_t& val);
    esp_err_t readRegister16(const uint8_t reg, uint16_t& val);
    esp_err_t writeRegister8(const uint8_t reg, const uint8_t val);
    esp_err_t writeRegister16(const uint8_t reg, const uint16_t val);
    esp_err_t writeRegisterBit16(const uint8_t reg, bool val, uint16_t bit);
    esp_err_t readRegisterBit16(const uint8_t reg, bool& val, const uint16_t bit);
    esp_err_t readRegisterBit8(const uint8_t reg, bool& val, uint8_t bit);
    esp_err_t writeRegisterBit8(const uint8_t reg, const bool val,
                                const uint8_t bit);

};

#endif /* __MCP23X17_H__ */
