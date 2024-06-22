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
 * File:   I2CManager.h
 * Author: chris
 *
 * Created on May 21, 2024, 4:46 PM
 */

#ifndef I2CMANAGER_H
#define I2CMANAGER_H

#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>
#include <mutex>
#include <vector>

#include "driver/i2c_master.h"

class I2CManager {
public:
  I2CManager(i2c_port_num_t portNumber, gpio_num_t sclPin, gpio_num_t sdaPin);
  ~I2CManager();

  /**
   * Register a slave device on the bus with the given deviceConfig and return a
   * handle to the device via deviceHandle
   *
   * @param deviceConfig
   * @param deviceHandle
   * @return A handle to the device
   */
  esp_err_t addDevice(const i2c_device_config_t &deviceConfig,
                      i2c_master_dev_handle_t &deviceHandle);

  /**
   *
   *
   * @param deviceHandle
   * @param data
   * @param size
   * @return
   */
  esp_err_t writeRegister(const i2c_master_dev_handle_t &deviceHandle,
                          const uint8_t reg, std::vector<uint8_t> &data,
                          int size = 0);

  /**
   * Write data to an I2C device without specifying the register
   *
   * @param deviceHandle
   * @param data
   * @param size
   * @return
   */
  esp_err_t write(const i2c_master_dev_handle_t &deviceHandle,
                  std::vector<uint8_t> &data, int size = 0);

  /**
   * Read data from an I2C device from a specific register
   *
   * @param deviceHandle
   * @param reg
   * @param data
   * @return
   */
  esp_err_t readRegister(const i2c_master_dev_handle_t &deviceHandle,
                         const uint8_t reg, std::vector<uint8_t> &data,
                         int size);

  /**
   * Read data from an I2C device without specifying a register
   *
   * @param deviceHandle
   * @param data
   * @param size
   * @return
   */
  esp_err_t read(const i2c_master_dev_handle_t &deviceHandle,
                 std::vector<uint8_t> &data, int size);

  /**
   * Probe an i2c device to see if it is found at the given address
   *
   * @param address
   * @return true if an i2c device was found at the given address
   */
  bool probe(int address);

  void scan();

private:
  i2c_master_bus_config_t _i2c_mst_config;
  i2c_master_bus_handle_t _bus_handle;
};

#endif /* I2CMANAGER_H */
