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

#include <chrono>
#include <cstring>
#include <shared_mutex>

#include "driver/i2c_master.h"
#include "esp_log.h"

#include "I2CManager.h"

static const char *TAG = "i2cmanager";

std::mutex _mutex;

I2CManager::I2CManager(i2c_port_num_t portNumber, gpio_num_t sclPin,
                       gpio_num_t sdaPin) {

  ESP_LOGD(TAG,
           "Entering i2cmanager constructor: port number: %d, sclPin: %d, "
           "sdaPin: %d",
           portNumber, sclPin, sdaPin);

  _i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
  _i2c_mst_config.i2c_port = portNumber;
  _i2c_mst_config.scl_io_num = sclPin;
  _i2c_mst_config.sda_io_num = sdaPin;
  _i2c_mst_config.glitch_ignore_cnt = 7;
  _i2c_mst_config.flags.enable_internal_pullup = true;
  _i2c_mst_config.intr_priority = 0;
  _i2c_mst_config.trans_queue_depth = 0;

  ESP_LOGD(TAG, "Calling i2c_new_master_bus... ");
  i2c_new_master_bus(&_i2c_mst_config, &_bus_handle);

  ESP_LOGD(TAG, "Leaving constructor");
}

I2CManager::~I2CManager() {}

esp_err_t I2CManager::addDevice(i2c_device_config_t &deviceConfig,
                                i2c_master_dev_handle_t &deviceHandle) {
  return i2c_master_bus_add_device(_bus_handle, &deviceConfig, &deviceHandle);
}

esp_err_t I2CManager::writeRegister(i2c_master_dev_handle_t &deviceHandle,
                                    const uint8_t reg, uint8_t *data,
                                    int size) {
  _mutex.lock();

  uint8_t newdata[size + 1];
  newdata[0] = reg;
  std::memcpy(&(newdata[1]), data, size);

  esp_err_t ret = i2c_master_transmit(deviceHandle, newdata, size + 1, 2000);

  _mutex.unlock();

  return ret;
}

esp_err_t I2CManager::write(i2c_master_dev_handle_t &deviceHandle,
                            uint8_t *data, int size) {
  _mutex.lock();
  esp_err_t ret = i2c_master_transmit(deviceHandle, data, size, 2000);
  _mutex.unlock();

  return ret;
}

esp_err_t I2CManager::readRegister(i2c_master_dev_handle_t &deviceHandle,
                                   const uint8_t reg, uint8_t *data, int size) {
  _mutex.lock();

  uint8_t writeBuffer[1];
  writeBuffer[0] = reg;

  esp_err_t ret = i2c_master_transmit_receive(deviceHandle, writeBuffer, 1,
                                              data, size, 2000);

  _mutex.unlock();
  return ret;
}

esp_err_t I2CManager::read(i2c_master_dev_handle_t &deviceHandle, uint8_t *data,
                           int size) {
  _mutex.lock();
  esp_err_t ret = i2c_master_receive(deviceHandle, data, size, 2000);
  _mutex.unlock();
  return ret;
}

bool I2CManager::probe(int address) {
  _mutex.lock();
  esp_err_t ret = i2c_master_probe(_bus_handle, address, 2000);
  bool result = (ret == ESP_OK);
  _mutex.unlock();
  return result;
}

void I2CManager::scan() {
  bool res;
  printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
  printf("00:         ");
  for (int i = 3; i < 0x78; i++) {
    res = probe(i);
    if (i % 16 == 0)
      printf("\n%02x:", i);
    if (res) {
      printf(" %02x", i);
    } else {
      printf(" --");
    }
  }
  printf("\n\n");
}
