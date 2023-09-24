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

#include "pca9629a.h"
#include "esp_log.h"
#include "i2cdev.h"
#include "config.h"

#include <cstring>

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

PCA9629A::PCA9629A(
        const i2c_port_t port,
        const gpio_num_t i2c_sda,
        const gpio_num_t i2c_scl,
        const uint8_t i2c_address,
        const uint32_t clock_speed
        ) {

    ESP_LOGE(TAG, "Port: %d, sda: %d, scl: %d, i2c_addr: %d, clock speed: %dl", port, i2c_sda, i2c_scl, i2c_address, clock_speed);

    memset(&i2c_dev, 0, sizeof (i2c_dev_t));

    i2c_dev.port = this->port;
    i2c_dev.addr = this->i2c_address;
    i2c_dev.cfg.mode = I2C_MODE_MASTER;
    i2c_dev.cfg.sda_io_num = this->i2c_sda;
    i2c_dev.cfg.scl_io_num = this->i2c_scl;
    i2c_dev.cfg.clk_flags = 0;
    i2c_dev.cfg.master.clk_speed = this->clock_speed;

    i2c_dev_create_mutex(&i2c_dev);
}

PCA9629A::~PCA9629A() {
    i2c_dev_delete_mutex(&i2c_dev);
}

void PCA9629A::initialise() {
    //software_reset();
    init_registers();
}

esp_err_t PCA9629A::software_reset(void) {
    ESP_LOGI(TAG, "pca9629a software_reset");
    uint8_t data = 0x06;

    I2C_DEV_TAKE_MUTEX(&i2c_dev);
    I2C_DEV_CHECK(&i2c_dev, i2c_dev_write_reg(&i2c_dev, static_cast<uint8_t> (REG_MODE), &data, 1));
    I2C_DEV_GIVE_MUTEX(&i2c_dev);

    return ESP_OK;
}

void PCA9629A::init_registers(void) {
    ESP_LOGI(TAG, "pca9629a init_registers");
    uint8_t init_array[] = {0x80, //  register access start address (0x00) with incremental access flag (MSB)
        0x10, // MODE
        0xFF, // WDTOI
        0x00, // WDCNTL
        0x01, // IO_CFG (P0 configured as input)
        0x21, // INTMODE (interrupt on falling edge for P0, 1ms noise suppression)
        0x1E, // MSK (Enable interrupt for I/O P0)
        0x00, // INTSTAT (Clears interrupt status register)
        0x00, // IP (read only register, writes to this register have no effect)
        0x01, // INT_MTR_ACT (stop motor on interrupt caused by P0)
        0x00, 0x00, // EXTRASTEPS0, EXTRASTEPS1
        0x50, // OP_CFG_PHS (two-phase drive outputs, OUT[3:0] configured as motor drive outputs)
        0x00, // OP_STAT_TO (default values)
        0x09, // RUCNTL (default values)
        0x09, // RDCNTL (default values)
        0x01, // PMA (perform specified motor action once)
        0x7D, // LOOPDLY_CW (default value)
        0x7D, // LOOPDLY_CCW (default value)
        0xFF, 0x01, // CCWSCOUNTL, CCWSCOUNTH
        0xFF, 0x01, // CCWSCOUNTL, CCWSCOUNTH
        0x05, 0x0D, // CWPWL, CWPWH
        0x05, 0x0D, // CCWPWL, CCWPWH
        0x00, // MCNTL
        0xE2, 0xE4, 0xE6, // SUBADR1 - SUBADR3
        0xE0, // ALLCALLADR
        0x00, 0x00, 0x00, 0x00//STEPCOUNT0 - STEPCOUNT3            
    };

    set_all_registers(init_array, sizeof ( init_array));
}

esp_err_t PCA9629A::set_all_registers(uint8_t *data, uint8_t size) {
    I2C_DEV_TAKE_MUTEX(&i2c_dev);
    I2C_DEV_CHECK(&i2c_dev, i2c_dev_write(&i2c_dev, NULL, 0, data, size));
    I2C_DEV_GIVE_MUTEX(&i2c_dev);

    return ESP_OK;
}

esp_err_t PCA9629A::write(RegisterName register_name, const uint8_t value) {
    uint8_t cmd[1];
    cmd[0] = value;

    I2C_DEV_TAKE_MUTEX(&i2c_dev);
    I2C_DEV_CHECK(&i2c_dev, i2c_dev_write_reg(&i2c_dev, static_cast<uint8_t> (register_name), cmd, 1));
    I2C_DEV_GIVE_MUTEX(&i2c_dev);

    return ESP_OK;
}

esp_err_t PCA9629A::write16(RegisterName register_name, const uint16_t value) {

    uint8_t cmd[ 2 ];

    cmd[ 0 ] = value & 0xFF;
    cmd[ 1 ] = value >> 8;

    I2C_DEV_TAKE_MUTEX(&i2c_dev);
    I2C_DEV_CHECK(&i2c_dev, i2c_dev_write_reg(&i2c_dev, static_cast<uint8_t> (register_name), cmd, 2));
    I2C_DEV_GIVE_MUTEX(&i2c_dev);

    return ESP_OK;
}

esp_err_t PCA9629A::read(RegisterName register_name, uint8_t& result) {
    uint8_t data;

    I2C_DEV_TAKE_MUTEX(&i2c_dev);
    I2C_DEV_CHECK(&i2c_dev, i2c_dev_read_reg(&i2c_dev, static_cast<uint8_t> (register_name), &data, 1));
    I2C_DEV_GIVE_MUTEX(&i2c_dev);

    result = data;

    return ESP_OK;
}

esp_err_t PCA9629A::read16(RegisterName register_name, uint16_t& result) {

    uint8_t data[ 2 ];

    I2C_DEV_TAKE_MUTEX(&i2c_dev);
    I2C_DEV_CHECK(&i2c_dev, i2c_dev_read_reg(&i2c_dev, static_cast<uint8_t> (register_name), data, 2));
    I2C_DEV_GIVE_MUTEX(&i2c_dev);

    result = (data[ 1 ] << 8 | data[ 0 ]);

    return ESP_OK;
}

void PCA9629A::start(Direction dir, uint16_t step_count, uint8_t repeats) {
    write16((dir == CW) ? REG_CWSCOUNTL : REG_CCWSCOUNTL, step_count);
    write(REG_PMA, repeats);
    //    write(REG_MCNTL, 0xA8 | dir);
    write(REG_MCNTL, 0x80 | static_cast<uint8_t> (dir));
}

void PCA9629A::startWithHome(Direction dir, uint16_t step_count, uint8_t repeats) {
    write(REG_MSK, 0x1E); // Enable P0 interrupt
    write(REG_INTSTAT, 0x00); // reset interrupt status register
    write(REG_INT_MTR_ACT, 0x01); // Set enable interrupt based control of motor and stop motor on interrupt caused by P0 in INT_MTR_ACT (= 0x01h) register 
    write16((dir == CW) ? REG_CWSCOUNTL : REG_CCWSCOUNTL, step_count);
    write(REG_PMA, repeats);
    write(REG_MCNTL, 0x90 | static_cast<uint8_t> (dir));
}

void PCA9629A::stop(void) {
    write(REG_MCNTL, 0x00);
}

esp_err_t PCA9629A::register_dump(void) {
    uint8_t data[ 34 ]; // number of registers
    uint8_t cmd = 0x80;

    I2C_DEV_TAKE_MUTEX(&i2c_dev);
    I2C_DEV_CHECK(&i2c_dev, i2c_dev_write(&i2c_dev, NULL, 0, &cmd, 1));
    I2C_DEV_GIVE_MUTEX(&i2c_dev);

    I2C_DEV_TAKE_MUTEX(&i2c_dev);
    I2C_DEV_CHECK_LOGE(&i2c_dev, i2c_dev_read(&i2c_dev, NULL, 0, data, sizeof (data)), "An error occurred reading registers");
    I2C_DEV_GIVE_MUTEX(&i2c_dev);

    ESP_LOGI(TAG, "PCA9629 register dump");
    //
    //    for (int i = 0, int j = 0x14; i <= 0x12; i++, j++) {
    //        ESP_LOGI(TAG, "  %-13s (0x%02X): 0x%02X    %-13s (0x%02X): 0x%02X", reg_name[ i ], i, data[ i ], reg_name[ j ], j, data[ j ]);
    //    }
    //
    //    ESP_LOGI(TAG, "  %-13s (0x%02X): 0x%02X", reg_name[ 0x13 ], 0x13, data[ 0x13 ]);

    return ESP_OK;
}


