/*
 * NvsController.cpp
 *
 *  Created on: Jul 14, 2024
 *      Author: chris
 */

#include "NvsController.h"

#include "nvs_flash.h"

static const char* TAG = "NvsController";

NvsController::NvsController()
= default;

NvsController::~NvsController()
= default;

esp_err_t NvsController::initialise()
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    // ESP_ERROR_CHECK(err);
    if (err != ESP_OK)
    {
        return err;
    }

    err |= nvs_flash_init_partition(NVS_PARTITION_SETTINGS.c_str());
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase_partition(NVS_PARTITION_SETTINGS.c_str()));
        err |= nvs_flash_init_partition(NVS_PARTITION_SETTINGS.c_str());
    }
    // ESP_ERROR_CHECK(err);

    nvsHandle = nvs::open_nvs_handle_from_partition(
        NVS_PARTITION_SETTINGS.c_str(), NVS_PARTITION_SETTINGS.c_str(),
        NVS_READWRITE, &err);

    return err;
}

void NvsController::writeValueToNVS(const char* key, uint16_t value) const
{
    esp_err_t err;

    // Write
    ESP_LOGD(TAG, "Updating %s in NVS ... ", key);

    err = nvsHandle->set_item<uint16_t>(key, value);
    if (err == ESP_OK)
    {
        ESP_LOGD(TAG, "Done");
    } else {
        ESP_LOGE(TAG, "Failed!");
    }

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are
    // written to flash storage. Implementations may write to storage at other
    // times, but this is not guaranteed.
    ESP_LOGD(TAG, "Committing updates in NVS ... ");
    err = nvsHandle->commit();

    if (err == ESP_OK)
    {
        ESP_LOGD(TAG, "Commit Done");
    } else {
        ESP_LOGE(TAG, "Commit Failed!");
    }
}

uint16_t NvsController::readValueFromNVS(const char* key) const
{
    esp_err_t err;

    // Read
    ESP_LOGD(TAG, "Reading %s from NVS ... ", key);

    uint16_t value = 0; // value will default to 0, if not set yet in NVS
    err = nvsHandle->get_item<uint16_t>(key, value);
    switch (err)
    {
    case ESP_OK:
        ESP_LOGD(TAG, "Done");
        ESP_LOGD(TAG, "%s = %d", key, value);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGE(TAG,
                 "The value for %s is not initialized yet! Initialising now to 0",
                 key);
        writeValueToNVS(key, 0);

        break;
    default:
        ESP_LOGE(TAG, "Error reading %s!", esp_err_to_name(err));
    }

    return value;
}
