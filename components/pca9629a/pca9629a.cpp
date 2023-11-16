/*
 * The MIT License
 *
 * Copyright 2023 chris.
 *
 * Permission is hereby granted, free of uint8_tge, to any person obtaining a copy
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

/**
 * Taken from datasheet at https://www.nxp.com/docs/en/data-sheet/PCA9629A.pdf
 */

#include <cstring>

#include "i2c_manager.h"
#include "pca9629a.h"
#include "esp_log.h"
#include "config.h"

static const char *TAG = "pca9629a";

const char *reg_name[] = {
    "MODE",
    "WDTOI",
    "WDTCNTL",
    "IO_CFG",
    "INTMODE",
    "MSK",
    "INTSTAT",
    "IP",
    "INT_MTR_ACT",
    "EXTRASTEPS0",
    "EXTRASTEPS1",
    "OP_CFG_PHS",
    "OP_STAT_TO",
    "RUCNTL",
    "RDCTNL",
    "PMA",
    "LOOPDLY_CW",
    "LOOPDLY_CCW",
    "CWSCOUNTL",
    "CWSCOUNTH",
    "CCWSCOUNTL",
    "CCWSCOUNTH",
    "CWPWL",
    "CWPWH",
    "CCWPWL",
    "CCWPWH",
    "MCNTL",
    "SUBADR1",
    "SUBADR2",
    "SUBADR3",
    "ALLCALLADR",
    "STEPCOUNT0",
    "STEPCOUNT1",
    "STEPCOUNT2",
    "STEPCOUNT3",
};

PCA9629A::PCA9629A(const i2c_port_t port, const uint8_t i2c_address) {

    ESP_LOGD(TAG, "i2c_port: %d, i2c_address: %d", i2c_port, i2c_address);
    this->i2c_port = port;
    this->i2c_address = i2c_address;
}

PCA9629A::~PCA9629A() {

}

void PCA9629A::initialise() {
    //software_reset();
    init_registers();
}

esp_err_t PCA9629A::software_reset(void) {
    ESP_LOGI(TAG, "pca9629a software_reset");
    uint8_t data = 0x06;

    esp_err_t ret = i2c_manager_write(this->i2c_port, this->i2c_address, static_cast<uint8_t> (REG_MODE), &data, 1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred in PCA9629A::software_reset writing i2c data");
    }

    return ret;
}

void PCA9629A::init_registers(void) {
    ESP_LOGI(TAG, "pca9629a init_registers");
    uint8_t init_array[] = {0x80, //  register access start address (0x00) with incremental access flag (MSB)
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
        //        0x50, // OP_CFG_PHS (two-phase drive outputs, OUT[3:0] configured as motor drive outputs)
        0xD0, // OP_CFG_PHS (half-step drive outputs, OUT[3:0] configured as motor drive outputs)
        0x05, // OP_STAT_TO (output pins = HOLD)
        0x00, // RUCNTL (default values)
        0x00, // RDCNTL (default values)
        0x01, // PMA (perform specified motor action once)
        0x00, // LOOPDLY_CW (default value)
        0x00, // LOOPDLY_CCW (default value)
        0x00, 0x00, // CCWSCOUNTL, CCWSCOUNTH
        0x00, 0x00, // CCWSCOUNTL, CCWSCOUNTH
        //0x05, 0x1F, // CWPWL, CWPWH
        //0x05, 0x1F, // CCWPWL, CCWPWH
        0x88, 0x10,
        0x88, 0x10,
        0x20, // MCNTL
        0xE2, 0xE4, 0xE6, // SUBADR1 - SUBADR3
        0xE0, // ALLCALLADR
    };

    set_all_registers(init_array, sizeof ( init_array) / sizeof (init_array[0]));
}

esp_err_t PCA9629A::set_all_registers(uint8_t* data, uint8_t size) {

    esp_err_t ret = i2c_manager_write(this->i2c_port, this->i2c_address, I2C_NO_REG, data, size);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred in PCA9629A::set_all_registers writing i2c data");
    }

    return ret;
}

esp_err_t PCA9629A::write(RegisterName register_name, const uint8_t value) {
    uint8_t cmd[1];
    cmd[0] = value;

    esp_err_t ret = i2c_manager_write(this->i2c_port, this->i2c_address, static_cast<uint8_t> (register_name), cmd, 1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred in PCA9629A::write writing i2c data");
    }

    return ret;
}

esp_err_t PCA9629A::write16(RegisterName register_name, const uint16_t value) {

    uint8_t cmd[ 2 ];

    cmd[ 0 ] = value & 0xFF;
    cmd[ 1 ] = value >> 8;

    esp_err_t ret = i2c_manager_write(this->i2c_port, this->i2c_address, static_cast<uint8_t> (register_name), &cmd[0], 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ret |= i2c_manager_write(this->i2c_port, this->i2c_address, static_cast<uint8_t> (register_name) + 1, &cmd[1], 1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred in PCA9629A::write16 writing i2c data");
    }

    return ret;
}

esp_err_t PCA9629A::read(RegisterName register_name, uint8_t& result) {
    uint8_t data;

    esp_err_t ret = i2c_manager_read(this->i2c_port, this->i2c_address, static_cast<uint8_t> (register_name), &data, 1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred in PCA9629A::read reading i2c data");
    }

    result = data;

    return ret;
}

esp_err_t PCA9629A::read16(RegisterName register_name, uint16_t& result) {

    uint8_t data[ 2 ];

    esp_err_t ret = i2c_manager_read(this->i2c_port, this->i2c_address, static_cast<uint8_t> (register_name), data, 2);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred in PCA9629A::read16 reading i2c data");
    }

    result = (data[ 1 ] << 8 | data[ 0 ]);

    return ret;
}

void PCA9629A::start(Direction direction, uint16_t step_count, uint8_t repeats) {
    _mutex.lock();
    performingAction = true;
    write(REG_MSK, 0x1F); // Disable all interrupts
    write(REG_INT_MTR_ACT, 0x00);
    write16((direction == CW) ? REG_CWSCOUNTL : REG_CCWSCOUNTL, step_count);
    write(REG_PMA, repeats);
    write(REG_INTSTAT, 0x00); // reset interrupt status register    
    write(REG_MCNTL, 0x80 | static_cast<uint8_t> (direction));
    performingAction = false;
    _mutex.unlock();
}

void PCA9629A::startAfterHome(Direction direction, uint16_t step_count, uint8_t repeats) {
    _mutex.lock();
    performingAction = true;

    write(REG_MSK, 0x1E); // Enable interrupt on P0
    write(REG_PMA, 1);
    write(REG_INT_MTR_ACT, 0x01); // Set enable interrupt based control of motor and stop motor on interrupt caused by P0 in INT_MTR_ACT (= 0x01h) register     
    write(REG_INTSTAT, 0x00); // reset interrupt status register
    write16((direction == CW) ? REG_CWSCOUNTL : REG_CCWSCOUNTL, 255);
    write(REG_MCNTL, 0x90 | static_cast<uint8_t> (direction));

    uint8_t data;
    read(REG_MCNTL, data);
    while ((data & 0x80) != 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
        read(REG_MCNTL, data);
    }

    write(REG_MSK, 0x1F); // Disable all interrupts
    write(REG_INT_MTR_ACT, 0x00);
    write16((direction == CW) ? REG_CWSCOUNTL : REG_CCWSCOUNTL, step_count);
    write(REG_PMA, repeats);
    write(REG_INTSTAT, 0x00); // reset interrupt status register    
    write(REG_MCNTL, 0x80 | static_cast<uint8_t> (direction));

    performingAction = false;

    _mutex.unlock();
}

void PCA9629A::home(Direction dir) {
    _mutex.lock();
    performingAction = true;

    write(REG_MSK, 0x1E); // Enable interrupt on P0
    write(REG_PMA, 1);
    write(REG_INT_MTR_ACT, 0x01); // Set enable interrupt based control of motor and stop motor on interrupt caused by P0 in INT_MTR_ACT (= 0x01h) register     
    write(REG_INTSTAT, 0x00); // reset interrupt status register
    write16((dir == CW) ? REG_CWSCOUNTL : REG_CCWSCOUNTL, 255);
    write(REG_MCNTL, 0x90 | static_cast<uint8_t> (dir));
    performingAction = false;
    _mutex.unlock();
}

bool PCA9629A::isStopped() {
    if (!performingAction) {
        _mutex.lock();
        uint8_t data;
        read(REG_MCNTL, data);

        //ESP_LOGD(TAG, "MCNTL register: %d", data);
        _mutex.unlock();
        return ((data & 0x80) == 0);
    }
    return (false); // we are still performing the action, so pretend we are not stopped
}

void PCA9629A::stop(void) {
    _mutex.lock();
    write(REG_MCNTL, 0x00);
    _mutex.unlock();
}

esp_err_t PCA9629A::register_dump(void) {
    uint8_t data[ 34 ]; // number of registers
    uint8_t cmd = 0x80;

    esp_err_t ret = i2c_manager_write(this->i2c_port, this->i2c_address, I2C_NO_REG, &cmd, 1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred in PCA9629A::register_dump writing i2c data");
    }

    ret |= i2c_manager_read(this->i2c_port, this->i2c_address, I2C_NO_REG, data, sizeof (data));

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred in PCA9629A::register_dump reading i2c data");
    }

    ESP_LOGI(TAG, "PCA9629 register dump");
    //
    //    for (int i = 0, int j = 0x14; i <= 0x12; i++, j++) {
    //        ESP_LOGI(TAG, "  %-13s (0x%02X): 0x%02X    %-13s (0x%02X): 0x%02X", reg_name[ i ], i, data[ i ], reg_name[ j ], j, data[ j ]);
    //    }
    //
    //    ESP_LOGI(TAG, "  %-13s (0x%02X): 0x%02X", reg_name[ 0x13 ], 0x13, data[ 0x13 ]);

    return ret;
}


