/*
 * The MIT License
 *
 * Copyright 2024 chris.
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
 * File:   I2CManager.cpp
 * Author: chris
 * 
 * Created on May 21, 2024, 4:46 PM
 */

#include <regex>
#include <vector>
#include <shared_mutex>

#include "driver/gpio.h"
#include "I2CManager.h"

I2CManager::I2CManager(i2c_port_num_t portNumber, gpio_num_t sclPin, gpio_num_t sdaPin) {

    _i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    _i2c_mst_config.i2c_port = portNumber;
    _i2c_mst_config.scl_io_num = sclPin;
    _i2c_mst_config.sda_io_num = sdaPin;
    _i2c_mst_config.glitch_ignore_cnt = 7;
    _i2c_mst_config.flags.enable_internal_pullup = true;

    i2c_new_master_bus(&_i2c_mst_config, &_bus_handle);
}

I2CManager::~I2CManager() {
}

esp_err_t I2CManager::addDevice(i2c_device_config_t& deviceConfig, i2c_master_dev_handle_t& deviceHandle) {
    return i2c_master_bus_add_device(_bus_handle, deviceConfig, deviceHandle);
}

esp_err_t I2CManager::writeRegister(i2c_master_dev_handle_t& deviceHandle, uint8_t reg, std::vector<uint8_t>& data) {
    _mutex.lock();
    data.insert(0, reg); // this will be the register to transmit to. Adding it here to the beginning of the data
    esp_err_t ret = i2c_master_transmit(deviceHandle, data.size(), data.data(), 2000);
    _mutex.unlock();

    return ret;
}

esp_err_t I2CManager::writeRegister(i2c_master_dev_handle_t& deviceHandle, uint8_t reg, uint8_t* data, int size) {


}

esp_err_t I2CManager::write(i2c_master_dev_handle_t& deviceHandle, std::vector<uint8_t> data) {
    _mutex.lock();

    esp_err_t ret = i2c_master_transmit(deviceHandle, data.data(), data.size(), 2000);
    _mutex.unlock();

    return ret;
}

esp_err_t I2CManager::write(i2c_master_dev_handle_t& deviceHandle, uint8_t* data, int size) {
    _mutex.lock();
    esp_err_t ret = i2c_master_transmit(deviceHandle, data, size, 2000);
    _mutex.unlock();

    return ret;
}

esp_err_t I2CManager::readRegister(i2c_master_dev_handle_t& deviceHandle, uint8_t reg, std::vector<uint8_t>& data, int bytesToRead) {
    _mutex.lock();
    
    esp_err_t ret = i2c_master_transmit_receive(i2c_master_dev_handle_t& deviceHandle, reg, 1, data.data(), data.size());

    _mutex.unlock();
    return ret;
}

esp_err_t I2CManager::readRegister(i2c_master_dev_handle_t& deviceHandle, uint8_t reg, uint8_t* data, int size) {
     _mutex.lock();

    esp_err_t ret = i2c_master_transmit_receive(i2c_master_dev_handle_t& deviceHandle, reg, 1, data, size);

    _mutex.unlock();
    return ret;
}


esp_err_t I2CManager::read(i2c_master_dev_handle_t& deviceHandle, uint8_t* data, int size) {
    _mutex.lock();
    esp_err_t ret = i2c_master_receive(deviceHandle, data, size, 2000);
    _mutex.unlock();
    return ret;
}

bool I2CManager::probe(int address) {
    _mutex.lock();
    esp_err_t ret = i2c_master_probe(_bus_handle, address, 2000) == ESP_OK;
    _mutex.unlock();
    return ret;
}

