/*
 * NvsController.h
 *
 *  Created on: Jul 14, 2024
 *      Author: chris
 */

#ifndef MAIN_NVSCONTROLLER_H_
#define MAIN_NVSCONTROLLER_H_

#include <cstdlib>
#include <memory>
#include <utility>

#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "nvs_handle.hpp"

class NvsController {
public:
  NvsController();
  virtual ~NvsController();

  esp_err_t initialise();
  void writeValueToNVS(const char *key, uint16_t value);
  uint16_t readValueFromNVS(const char *key);

public:
private:
  static constexpr std::string NVS_PARTITION_SETTINGS = "settings";

  std::unique_ptr<nvs::NVSHandle> nvsHandle;
};

#endif /* MAIN_NVSCONTROLLER_H_ */
