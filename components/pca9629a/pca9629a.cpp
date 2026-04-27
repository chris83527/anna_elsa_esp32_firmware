/*
 * The MIT License
 *
 * Copyright 2023 chris.
 *
 * Permission is hereby granted, free of uint8_tge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
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

/**
 * Taken from datasheet at https://www.nxp.com/docs/en/data-sheet/PCA9629A.pdf
 */

#include <cstring>

#include "esp_log.h"
#include "pca9629a.h"
#include "pca9629a_regs.hpp"

namespace pca9629a
{
    static const char* TAG = "pca9629a";

    void Driver::initialise()
    {
        softwareReset();
        initRegisters();
    }

    esp_err_t Driver::softwareReset()
    {
        ESP_LOGI(TAG, "pca9629a software_reset");

        std::vector<uint8_t> data;
        data.push_back(0x06);

        esp_err_t ret = writeReg(static_cast<uint8_t>(RegisterName::REG_MODE), data);

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG,
                     "An error occurred in software_reset writing i2c data");
        }

        return ret;
    }

    esp_err_t Driver::initRegisters()
    {
        ESP_LOGI(TAG, "pca9629a init_registers");
        std::vector<uint8_t> data = {
            0x80, //  register access start address (0x00) with incremental access
            //  flag (MSB)
            0x21, // MODE
            0x0A, // WDTOI (10 second timeout)
            0x00, // WDCNTL
            0x01, // IO_CFG (P0 configured as input)
            0x21, // INTMODE (interrupt on falling edge for P0, 1ms noise suppression)
            0x1E, // MSK (Enable interrupt for I/O P0)
            0x00, // INTSTAT (Clears interrupt status register)
            0x00, // IP (read only register, writes to this register have no effect)
            0x01, // INT_MTR_ACT (stop motor on interrupt caused by P0)
            0x00, 0x00, // EXTRASTEPS0, EXTRASTEPS1
            //        0x50, // OP_CFG_PHS (two-phase drive outputs, OUT[3:0]
            //        configured as motor drive outputs)
            0xD0, // OP_CFG_PHS (half-step drive outputs, OUT[3:0] configured as motor
            // drive outputs)
            0x05, // OP_STAT_TO (output pins = HOLD)
            0x00, // RUCNTL (default values)
            0x00, // RDCNTL (default values)
            0x01, // PMA (perform specified motor action once)
            0x00, // LOOPDLY_CW (default value)
            0x00, // LOOPDLY_CCW (default value)
            0x00, 0x00, // CCWSCOUNTL, CCWSCOUNTH
            0x00, 0x00, // CCWSCOUNTL, CCWSCOUNTH
            // 0x05, 0x1F, // CWPWL, CWPWH
            // 0x05, 0x1F, // CCWPWL, CCWPWH
            0x88, 0x10, 0x88, 0x10,
            0x20, // MCNTL
            0xE2, 0xE4, 0xE6, // SUBADR1 - SUBADR3
            0xE0, // ALLCALLADR
        };

        esp_err_t ret = write(data);

        if (ret != ESP_OK)
        {
            ESP_LOGE(
                TAG,
                "An error occurred in set_all_registers writing i2c data");
        }

        return ret;
    }

    void Driver::moveSteps(Direction direction, uint16_t step_count,
                       uint8_t repeats)
    {
        performingAction = true;
        write8(RegisterName::REG_MSK, 0x1F); // Disable all interrupts
        write8(RegisterName::REG_INT_MTR_ACT, 0x00);
        write16((direction == Direction::CW) ? RegisterName::REG_CWSCOUNTL : RegisterName::REG_CCWSCOUNTL,
                step_count);
        write8(RegisterName::REG_PMA, repeats);
        write8(RegisterName::REG_INTSTAT, 0x00); // reset interrupt status register
        write8(RegisterName::REG_MCNTL, 0x80 | static_cast<uint8_t>(direction));
        performingAction = false;
    }

    void Driver::moveStepsAfterHome(Direction direction, uint16_t step_count,
                                uint8_t repeats)
    {
        performingAction = true;

        write8(RegisterName::REG_MSK, 0x1E); // Enable interrupt on P0
        write8(RegisterName::REG_PMA, 1);
        write8(RegisterName::REG_INT_MTR_ACT, 0x01); // Set enable interrupt based control of motor and stop motor on
        // interrupt caused by P0 in INT_MTR_ACT (= 0x01h) register
        write8(RegisterName::REG_INTSTAT, 0x00); // reset interrupt status register
        write16((direction == Direction::CW) ? RegisterName::REG_CWSCOUNTL : RegisterName::REG_CCWSCOUNTL, 255);
        write8(RegisterName::REG_MCNTL, 0x90 | static_cast<uint8_t>(direction));

        uint8_t data;
        read8(RegisterName::REG_MCNTL, data);
        while ((data & bit::MCNTL_BUSY) != 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            read8(RegisterName::REG_MCNTL, data);
        }

        // Now move the number of steps
        write8(RegisterName::REG_MSK, 0x1F); // Disable all interrupts
        write8(RegisterName::REG_INT_MTR_ACT, 0x00);
        write16((direction == Direction::CW) ? RegisterName::REG_CWSCOUNTL : RegisterName::REG_CCWSCOUNTL, step_count);
        write8(RegisterName::REG_PMA, repeats);
        write8(RegisterName::REG_INTSTAT, 0x00); // reset interrupt status register
        write8(RegisterName::REG_MCNTL, 0x80 | static_cast<uint8_t>(direction));

        performingAction = false;
    }

    void Driver::home(Direction dir)
    {
        performingAction = true;

        write8(RegisterName::REG_MSK, 0x1E); // Enable interrupt on P0
        write8(RegisterName::REG_PMA, 0x01);
        write8(RegisterName::REG_INT_MTR_ACT,
               0x01); // Set enable interrupt based control of motor and stop motor on
        // interrupt caused by P0 in INT_MTR_ACT (= 0x01h) register
        write8(RegisterName::REG_INTSTAT, 0x00); // reset interrupt status register
        write16((dir == Direction::CW) ? RegisterName::REG_CWSCOUNTL : RegisterName::REG_CCWSCOUNTL, 255);
        write8(RegisterName::REG_MCNTL, 0x90 | static_cast<uint8_t>(dir));
        performingAction = false;
    }

    void Driver::stop()
    {
        write8(RegisterName::REG_MCNTL, 0x00);
    }

    bool Driver::isStopped()
    {
        if (!performingAction)
        {
            uint8_t data;
            read8(RegisterName::REG_MCNTL, data);

            return (data & 0x80) == 0;
        }
        return false; // we are still performing the action, so pretend we are not
    }

    esp_err_t Driver::registerDump()
    {
        //     uint8_t data[ 34 ]; // number of registers
        //     uint8_t cmd = 0x80;
        //
        //     esp_err_t ret = i2c_manager_write(this->i2c_port, this->i2c_address,
        //     I2C_NO_REG, &cmd, 1);
        //
        //     if (ret != ESP_OK) {
        //         ESP_LOGE(TAG, "An error occurred in register_dump writing
        //         i2c data");
        //     }
        //
        //     ret |= i2c_manager_read(this->i2c_port, this->i2c_address, I2C_NO_REG,
        //     data, sizeof (data));
        //
        //     if (ret != ESP_OK) {
        //         ESP_LOGE(TAG, "An error occurred in register_dump reading
        //         i2c data");
        //     }
        //
        //     ESP_LOGI(TAG, "PCA9629 register dump");
        //     //
        //     //    for (int i = 0, int j = 0x14; i <= 0x12; i++, j++) {
        //     //        ESP_LOGI(TAG, "  %-13s (0x%02X): 0x%02X    %-13s (0x%02X):
        //     0x%02X", reg_name[ i ], i, data[ i ], reg_name[ j ], j, data[ j ]);
        //     //    }
        //     //
        //     //    ESP_LOGI(TAG, "  %-13s (0x%02X): 0x%02X", reg_name[ 0x13 ], 0x13,
        //     data[ 0x13 ]);
        //
        //     return ret;

        return ESP_OK;
    }

    // I2C Helper methods
    esp_err_t Driver::write8(RegisterName register_name, const uint8_t value)
    {
        std::vector<uint8_t> data;
        data.push_back(value);

        esp_err_t ret = writeReg(static_cast<uint8_t>(register_name), data);

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "An error occurred in write writing i2c data");
        }

        return ret;
    }

    esp_err_t Driver::write16(RegisterName register_name, const uint16_t value)
    {
        std::vector<uint8_t> data;

        data.push_back(value & 0xFF);
        data.push_back(value >> 8);

        // TODO:
        esp_err_t ret = writeReg(static_cast<uint8_t>(register_name) + 0x80, data); // + 0x80 for autoincrement

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "An error occurred in write16 writing i2c data");
        }

        return ret;
    }

    esp_err_t Driver::read8(RegisterName register_name, uint8_t& result)
    {
        std::vector<uint8_t> data;

        esp_err_t ret = readReg(static_cast<uint8_t>(register_name), data, 1);

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "An error occurred in read reading i2c data");
            result = -1;
        }
        else
        {
            result = data.at(0);
        }

        return ret;
    }

    esp_err_t Driver::read16(RegisterName register_name, uint16_t& result)
    {
        std::vector<uint8_t> data;

        esp_err_t ret = readReg(static_cast<uint8_t>(register_name), data, 2);

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "An error occurred in read16 reading i2c data");
            result = -1; // return as error
        }
        else
        {
            result = (data[1] << 8 | data[0]);
        }

        return ret;
    }

}
