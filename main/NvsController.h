/*
 * NvsController.h
 *
 *  Created on: Jul 14, 2024
 *      Author: chris
 */

#ifndef MAIN_NVSCONTROLLER_H_
#define MAIN_NVSCONTROLLER_H_

#include <string>
#include <memory>

#include "esp_err.h"
#include "esp_log.h"

#include "nvs_handle.hpp"

class NvsController {
public:
  NvsController();
  virtual ~NvsController();

  esp_err_t initialise();
  void writeValueToNVS(const char *key, uint16_t value) const;
  esp_err_t writeStringValueToNVS(const char *key, const char* value) const;
  uint16_t readValueFromNVS(const char *key) const;
  esp_err_t readStringValueFromNVS(const char *key, char *value, size_t len) const;
  esp_err_t eraseValueFromNVS(const char *key) const;

public:
private:
  static constexpr std::string NVS_PARTITION_SETTINGS = "settings";

  std::unique_ptr<nvs::NVSHandle> nvsHandle;
};

#endif /* MAIN_NVSCONTROLLER_H_ */
