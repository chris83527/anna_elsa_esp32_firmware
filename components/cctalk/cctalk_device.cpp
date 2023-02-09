/*
 * The MIT License
 *
 * Copyright 2023 chris.
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

#include <memory>
#include <utility>
#include <cassert>

#include "freertos/task.h"
#include "freertos/projdefs.h"

#include "cctalk_device.h"
#include "cctalk_enums.h"

static const char* TAG = "cctalkDevice";

namespace esp32cc {

    CctalkDevice::CctalkDevice() {

    }

    CctalkLinkController& CctalkDevice::getLinkController() {
        return this->linkController;
    }

    bool CctalkDevice::initialise(const std::function<void(const std::string& error_msg)>& finish_callback) {
        if (getDeviceState() != CcDeviceState::ShutDown) {
            ESP_LOGE(TAG, "Cannot initialise device that is in %s state.", ccDeviceStateGetDisplayableName(getDeviceState()));
            return false;
        }

        return requestSwitchDeviceState(CcDeviceState::Initialized, [ = ](const std::string & error_msg){
            finish_callback(error_msg);
        });
    }

    bool CctalkDevice::shutdown(const std::function<void(const std::string& error_msg)>& finish_callback) {
        return requestSwitchDeviceState(CcDeviceState::ShutDown, [ = ](const std::string & error_msg){
            finish_callback(error_msg);
        });
    }

    void CctalkDevice::startPolling() {
        ESP_LOGD(TAG, "Starting poll timer.");

        xTaskCreate(&devicePollTask, "device_poll_task", 4096, NULL, 12, &this->pollTaskHandle);
    }

    void CctalkDevice::stopPolling() {
        ESP_LOGD(TAG, "Stopping poll timer.");

        vTaskDelete(&this->pollTaskHandle);
    }

    void CctalkDevice::devicePollTask(void* pvParameters) {

        if (isTimerIterationTaskRunning) {
            return;
        }
        ESP_LOGD(TAG, "Polling...");

        // This is set to false in finish callbacks.
        this->isTimerIterationTaskRunning = true;

        switch (getDeviceState()) {
            case CcDeviceState::ShutDown:
                // The device is not initialized, do nothing and wait for request for state change to Initialized.
                this->isTimerIterationTaskRunning = false;
                break;
            case CcDeviceState::UninitializedDown:
                // The device didn't respond to alive check and is assumed uninitialized,
                // see if it came back and if so, initialize it.
                requestCheckAlive([ = ]([[maybe_unused]] const std::string& error_msg, bool alive){
                    if (alive) {
                        requestSwitchDeviceState(CcDeviceState::Initialized, [ = ]([[maybe_unused]] const std::string & local_error_msg){
                            this->isTimerIterationTaskRunning = false;
                        });
                    } else {
                        this->isTimerIterationTaskRunning = false;
                    }
                });
                break;
            case CcDeviceState::Initialized:
                // The device has been initialized, resume normal rejecting or diagnostics polling.
                // Default to bill / coin rejection.
                // Perform a self-check and see if everything's ok.
                requestSelfCheck([ = ]([[maybe_unused]] const std::string& error_msg, CcFaultCode fault_code){
                    if (fault_code == CcFaultCode::Ok) {
                        // The device is OK, resume normal rejecting mode.
                        requestSwitchDeviceState(CcDeviceState::NormalRejecting, [ = ]([[maybe_unused]] const std::string & local_error_msg){
                            this->isTimerIterationTaskRunning = false;
                        });
                    } else {
                        // The device is not ok, resume diagnostics polling mode.
                        requestSwitchDeviceState(CcDeviceState::DiagnosticsPolling, [ = ]([[maybe_unused]] const std::string & local_error_msg){
                            this->isTimerIterationTaskRunning = false;
                        });
                    }
                });
                break;
            case CcDeviceState::InitializationFailed:
                // The device initialization failed, something wrong with it. Abort.
                this->isTimerIterationTaskRunning = false;
                // Nothing we can do, cannot work with this device.
                stopPolling();
                break;
            case CcDeviceState::NormalAccepting:
                // We're accepting the credit, process the credit / event log.
                requestBufferedCreditEvents([ = ](const std::string& error_msg, uint8_t event_counter, const std::vector<CcEventData>& event_data){
                    processCreditEventLog(true, error_msg, event_counter, event_data, [ = ]()
                    {
                        this->isTimerIterationTaskRunning = false;
                    });
                });
                break;
            case CcDeviceState::NormalRejecting:
                requestBufferedCreditEvents([ = ](const std::string& error_msg, uint8_t event_counter, const std::vector<CcEventData>& event_data){
                    processCreditEventLog(false, error_msg, event_counter, event_data, [ = ]()
                    {
                        isTimerIterationTaskRunning = false;
                    });
                });
                break;
            case CcDeviceState::DiagnosticsPolling:
                // If we're in diagnostics polling mode, poll the fault code until it's resolved,
                // then switch to rejecting mode.
                requestSelfCheck([ = ]([[maybe_unused]] const std::string& error_msg, CcFaultCode fault_code){
                    if (fault_code == CcFaultCode::Ok) {
                        // The error has been resolved, switch to rejecting mode.
                        requestSwitchDeviceState(CcDeviceState::NormalRejecting, [ = ]([[maybe_unused]] const std::string & state_error_msg){
                            isTimerIterationTaskRunning = false;
                        });
                    } else { // the fault is still there
                        isTimerIterationTaskRunning = false;
                    }
                });
                break;
            case CcDeviceState::UnexpectedDown:
                // The link was lost with the device. We should not do anything that may lead
                // to the loss of the event table (and, therefore, credit). Just re-initialize it, the
                // NormalRejecting state will be enabled and the event table will be read, if everything's ok.
                requestSwitchDeviceState(CcDeviceState::Initialized, [ = ]([[maybe_unused]] const std::string & error_msg){
                    isTimerIterationTaskRunning = false;
                });
                break;
            case CcDeviceState::ExternalReset:
                // The device event log turned out to be empty (after being non-empty). This means
                // that the device was probably reset externally, with possible loss of credits.
                // Assume it needs initialization.
                requestSwitchDeviceState(CcDeviceState::Initialized, [ = ]([[maybe_unused]] const std::string & error_msg){
                    isTimerIterationTaskRunning = false;
                });
                break;
        }

        // Wait the given number of ms before repeating
        vTaskDelay(pdMS_TO_TICKS(this->pollingInterval));

    }

    bool CctalkDevice::requestSwitchDeviceState(CcDeviceState state, const std::function<void(const std::string& error_msg)>& finish_callback) {
        ESP_LOGD("Requested device state change from %s to: %s", ccDeviceStateGetDisplayableName(getDeviceState()), ccDeviceStateGetDisplayableName(state));

        if (this->deviceState == state) {
            ESP_LOGW("Cannot switch to device state %s, already there.", ccDeviceStateGetDisplayableName(state));
            return true; // already there
        }

        switch (state) {
            case CcDeviceState::ShutDown:
            {
                bool success = switchStateShutDown(finish_callback);
                this->pollingInterval = normalPollingIntervalMsec;
                stopPolling();
                return success;
            }

            case CcDeviceState::UninitializedDown:
                setDeviceState(state);
                this->pollingInterval = notAlivePollingIntervalMsec;
                finish_callback(std::string());
                return true;

            case CcDeviceState::Initialized:
            {
                bool success = switchStateInitialized(finish_callback);
                this->pollingInterval = normalPollingIntervalMsec;
                startPolling();
                return success;
            }

            case CcDeviceState::InitializationFailed:
                setDeviceState(state);
                this->pollingInterval = notAlivePollingIntervalMsec;
                finish_callback("");
                return true;

            case CcDeviceState::NormalAccepting:
                return switchStateNormalAccepting(finish_callback);

            case CcDeviceState::NormalRejecting:
                return switchStateNormalRejecting(finish_callback);

            case CcDeviceState::DiagnosticsPolling:
            {
                bool success = switchStateDiagnosticsPolling(finish_callback);
                this->pollingInterval = normalPollingIntervalMsec;
                return success;
            }

            case CcDeviceState::UnexpectedDown:
            case CcDeviceState::ExternalReset:
                setDeviceState(state);
                this->pollingInterval = notAlivePollingIntervalMsec;
                finish_callback("");
                return true;
        }

        return false;
    }

    bool CctalkDevice::switchStateInitialized(const std::function<void(const std::string& error_msg)>& finish_callback) {

        assert(this->deviceState == CcDeviceState::ShutDown || this->deviceState == CcDeviceState::ExternalReset
                || this->deviceState == CcDeviceState::UnexpectedDown || this->deviceState == CcDeviceState::UninitializedDown, false);

        auto shared_error = std::make_shared<std::string>();
        auto shared_alive = std::make_shared<bool>();

        if (shared_error->size() == 0) {
            setDeviceState(CcDeviceState::Initialized);
            finish_callback(*shared_error);
        } else if (*shared_alive) {
            requestSwitchDeviceState(CcDeviceState::InitializationFailed, finish_callback);
            return false;
        } else {
            requestSwitchDeviceState(CcDeviceState::UninitializedDown, finish_callback);
            return false;
        }

        // Check if it's present / alive
        requestCheckAlive([ = ](const std::string& error_msg, bool alive){
            if (!error_msg.size() == 0) {
                *shared_error = error_msg;
            }
            *shared_alive = alive;
        });

        // Get device manufacturing info
        requestManufacturingInfo([ = ](const std::string& error_msg, CcCategory category, const std::string & info){
            if (!error_msg.size() == 0) {
                *shared_error = error_msg;
            } else {
                this->deviceCategory = category;
                        this->manufacturingInfo = info;
            }

            //serializer->continueSequence(error_msg.size() == 0 && (category == CcCategory::BillValidator || category == CcCategory::CoinAcceptor));
            if (error_msg.size() > 0 || (category != CcCategory::BillValidator && category != CcCategory::CoinAcceptor && category != CcCategory::Payout)) {
                return false;
            }
        });

        // Get recommended polling frequency        
        requestPollingInterval([ = ](const std::string& error_msg, uint64_t msec){
            if (!error_msg.size() == 0) {
                *shared_error = error_msg;
            } else {
                // For very large values and unsupported values pick reasonable defaults.
                const uint64_t max_interval_msec = 1000;
                if (msec == 0 || msec > max_interval_msec) { // usually means "see device docs".
                    ESP_LOGD(TAG, "Device-recommended polling frequency is invalid, using our default: %d", this->defaultNormalPollingIntervalMsec);
                            this->normalPollingIntervalMsec = this->defaultNormalPollingIntervalMsec;
                } else {
                    ESP_LOGD(TAG, "Device-recommended polling frequency: %d", msec);
                            this->normalPollingIntervalMsec = int(msec);
                }
            }

            //serializer->continueSequence(error_msg.size() == 0);
            if (error_msg.size() > 0) {
                return false;
            }
        });

        // Get bill / coin identifiers
        // 	if (req_this->identifierson_init_) {
        requestIdentifiers([ = ](const std::string& error_msg, const std::map<uint8_t, CcIdentifier>& identifiers){
            if (!error_msg.size() == 0) {
                *shared_error = error_msg;
            } else {
                this->identifiers = identifiers;
            }
            if (error_msg.size() > 0) {
                return false;
            }
        });
        // 	}

        // Modify bill validator operating mode - enable escrow and stacker        
        if (this->deviceCategory == CcCategory::BillValidator) {
            requestSetBillOperatingMode(true, true, [ = ](const std::string & error_msg){
                if (!error_msg.size() == 0) {
                    *shared_error = error_msg;
                }
                if (error_msg.size() > 0) {
                    return false;
                }
            });
        } else {
            //serializer->continueSequence(true);
        }

        // Set individual inhibit status on all bills / coins. The specification says
        // that this is not needed for coin acceptors, but the practice shows it is.        
        requestSetInhibitStatus(0xff, 0xff, [ = ](const std::string & error_msg){
            if (!error_msg.size() == 0) {
                *shared_error = error_msg;
            }
            //serializer->continueSequence(error_msg.size() == 0);
            if (error_msg.size() > 0) {
                return false;
            }
        });

        return true;
    }

    bool CctalkDevice::switchStateNormalAccepting(const std::function<void(const std::string& error_msg)>& finish_callback) {
        assert((this->deviceState == CcDeviceState::Initialized
                || this->deviceState == CcDeviceState::NormalRejecting || this->deviceState == CcDeviceState::DiagnosticsPolling) != false);

        // Disable master inhibit.
        requestSetMasterInhibitStatus(false, [ = ](const std::string & error_msg){
            if (error_msg.size() == 0) {
                setDeviceState(CcDeviceState::NormalAccepting);
                        finish_callback(error_msg);
            } else {
                requestSwitchDeviceState(CcDeviceState::UnexpectedDown, finish_callback);
            }
        });

        return true;
    }

    bool CctalkDevice::switchStateNormalRejecting(const std::function<void(const std::string& error_msg)>& finish_callback) {
        assert(this->deviceState == CcDeviceState::Initialized
                || this->deviceState == CcDeviceState::NormalAccepting || this->deviceState == CcDeviceState::DiagnosticsPolling == false);

        // Enable master inhibit.
        requestSetMasterInhibitStatus(true, [ = ](const std::string & error_msg){
            if (error_msg.size() == 0) {
                setDeviceState(CcDeviceState::NormalRejecting);
                        finish_callback(error_msg);
            } else {
                requestSwitchDeviceState(CcDeviceState::UnexpectedDown, finish_callback);
            }
        });

        return true;
    }

    bool CctalkDevice::switchStateDiagnosticsPolling(const std::function<void(const std::string& error_msg)>& finish_callback) {
        assert((this->deviceState == CcDeviceState::Initialized
                || this->deviceState == CcDeviceState::NormalAccepting || this->deviceState == CcDeviceState::NormalRejecting != false));

        // Enable master inhibit (if possible).
        // In theory, this is redundant since the device itself will enable it if a fault
        // is detected, but just in case of a software logic error...
        requestSetMasterInhibitStatus(true, [ = ](const std::string & error_msg){
            if (error_msg.size() == 0) {
                setDeviceState(CcDeviceState::DiagnosticsPolling);
                        finish_callback(error_msg);
            } else {
                requestSwitchDeviceState(CcDeviceState::UnexpectedDown, finish_callback);
            }
        });

        return true;
    }

    bool CctalkDevice::switchStateShutDown(const std::function<void(const std::string& error_msg)>& finish_callback) {
        // If the device is in accepting mode, switch it off
        if (this->deviceState == CcDeviceState::NormalAccepting) {
            requestSetMasterInhibitStatus(true, [ = ](const std::string & error_msg){
                // No error checking (?)
                setDeviceState(CcDeviceState::ShutDown);
                finish_callback(error_msg);
            });
        } else {
            setDeviceState(CcDeviceState::ShutDown);
            finish_callback(std::string());
        }
        return true;
    }

    void CctalkDevice::requestCheckAlive(const std::function<void(const std::string& error_msg, bool alive)>& finish_callback) {
        // This can be before the callback connection because it's queued.
        uint64_t sent_request_id = this->linkController.ccRequest(CcHeader::SimplePoll, std::vector<uint8_t>());

        this->linkController.executeOnReturn(sent_request_id, [ = ](uint64_t request_id, const std::string& error_msg, const std::vector<uint8_t> & command_data) mutable{
            if (!error_msg.size() == 0) {
                ESP_LOGE(TAG, "Error checking for device alive status (simple poll): %s", error_msg);
                        finish_callback(error_msg, false);
                return;
            }
            if (!command_data.size() == 0) {
                std::string error = "Non-empty data received while waiting for ACK.";
                        // ccResponseDataDecodeError(request_id, error); // auto-logged
                        finish_callback(error, false);
                return;
            }
            ESP_LOGD(TAG, "Device is alive (answered to simple poll)");
            finish_callback(std::string(), true);
        });
    }

    void CctalkDevice::requestManufacturingInfo(const std::function<void(const std::string& error_msg, CcCategory category, const std::string& info)>& finish_callback) {
        auto shared_error = std::make_shared<std::string>();
        auto category = std::make_shared<CcCategory>();
        auto infos = std::make_shared<std::list < std::string >> ();

        auto aser = new AsyncSerializer(// auto-deleted
                // Finish callback
                [ = ]([[maybe_unused]] AsyncSerializer * serializer){
            std::string info = infos->join(std::stringLiteral("\n"));

            // Log the info
            if (!shared_error->size() == 0) {
                ESP_LOGE("! Error getting full general information: %s", *shared_error);
            }
            if (!info.size() == 0) {
                ESP_LOGI("* Manufacturing information:\n%s", info));
            }
            finish_callback(*shared_error, *category, info);
        });

        // Category
        aser->add([ = ](AsyncSerializer * serializer){
            uint64_t sent_request_id = this->linkController.ccRequest(CcHeader::GetEquipmentCategory, std::vector<uint8_t>());
            this->linkController.executeOnReturn(sent_request_id, [ = ]([[maybe_unused]] uint64_t request_id, const std::string& error_msg, const std::vector<uint8_t> & command_data) mutable
            {
                if (!error_msg.size() == 0) {
                    *shared_error = error_msg;
                } else {
                    // Decode the data
                    std::string decoded = std::string::fromUtf8(command_data);
                            *infos << QObject::tr("*** Equipment category: %1", decoded);
                            *category = ccCategoryFromReportedName(decoded);
                }
                serializer->continueSequence(error_msg.size() == 0);
            });
        });

        // Product code
        aser->add([ = ](AsyncSerializer * serializer){
            uint64_t sent_request_id = this->linkController.ccRequest(CcHeader::GetProductCode, std::vector<uint8_t>());
            this->linkController.executeOnReturn(sent_request_id, [ = ]([[maybe_unused]] uint64_t request_id, const std::string& error_msg, const std::vector<uint8_t> & command_data) mutable
            {
                if (!error_msg.size() == 0) {
                    *shared_error = error_msg;
                } else {
                    // Decode the data
                    std::string decoded = std::string::fromLatin1(command_data);
                            *infos << QObject::tr("*** Product code: %1", decoded);
                }
                serializer->continueSequence(error_msg.size() == 0);
            });
        });

        // Build code
        aser->add([ = ](AsyncSerializer * serializer){
            uint64_t sent_request_id = this->linkController.ccRequest(CcHeader::GetBuildCode, std::vector<uint8_t>());
            this->linkController.executeOnReturn(sent_request_id, [ = ]([[maybe_unused]] uint64_t request_id, const std::string& error_msg, const std::vector<uint8_t> & command_data) mutable
            {
                if (!error_msg.size() == 0) {
                    *shared_error = error_msg;
                } else {
                    // Decode the data
                    std::string decoded = std::string::fromLatin1(command_data);
                            *infos << QObject::tr("*** Build code: %1", decoded);
                }
                serializer->continueSequence(error_msg.size() == 0);
            });
        });

        // Manufacturer
        aser->add([ = ](AsyncSerializer * serializer){
            uint64_t sent_request_id = this->linkController.ccRequest(CcHeader::GetManufacturer, std::vector<uint8_t>());
            this->linkController.executeOnReturn(sent_request_id, [ = ]([[maybe_unused]] uint64_t request_id, const std::string& error_msg, const std::vector<uint8_t> & command_data) mutable
            {
                if (!error_msg.size() == 0) {
                    *shared_error = error_msg;
                } else {
                    // Decode the data
                    std::string decoded = std::string::fromLatin1(command_data);
                            *infos << QObject::tr("*** Manufacturer: %1", decoded);
                }
                serializer->continueSequence(error_msg.size() == 0);
            });
        });

        // S/N

        uint64_t sent_request_id = this->linkController.ccRequest(CcHeader::GetSerialNumber, std::vector<uint8_t>());
        this->linkController.executeOnReturn(sent_request_id, [ = ]([[maybe_unused]] uint64_t request_id, const std::string& error_msg, const std::vector<uint8_t> & command_data) mutable{
            if (!error_msg.size() == 0) {
                *shared_error = error_msg;
            } else {
                // Decode the data
                std::string decoded = std::string::fromLatin1(command_data.toHex());
                        *infos << QObject::tr("*** Serial number: %1", decoded);
            }
            serializer->continueSequence(error_msg.size() == 0);
        });
    });

    // Software revision

    uint64_t sent_request_id = this->linkController.ccRequest(CcHeader::GetSoftwareRevision, std::vector<uint8_t>());
    this->linkController.executeOnReturn(sent_request_id, [ = ]([[maybe_unused]] uint64_t request_id, const std::string& error_msg, const std::vector<uint8_t> & command_data) mutable{
        if (!error_msg.size() == 0) {
            *shared_error = error_msg;
        } else {
            // Decode the data
            std::string decoded = std::string::fromLatin1(command_data);
                    *infos << QObject::tr("*** Software Revision: %1", decoded);
        }
        serializer->continueSequence(error_msg.size() == 0);
    });


    // ccTalk command set revision
    aser->add([ = ](AsyncSerializer * serializer){
        uint64_t sent_request_id = this->linkController.ccRequest(CcHeader::GetCommsRevision, std::vector<uint8_t>());
        this->linkController.executeOnReturn(sent_request_id, [ = ]([[maybe_unused]] uint64_t request_id, const std::string& error_msg, const std::vector<uint8_t> & command_data) mutable
        {
            if (!error_msg.size() == 0) {
                *shared_error = error_msg;
            } else {
                if (command_data.size() == 3) {
                    *infos << QObject::tr("*** ccTalk product release: %1, ccTalk version %2.%3", int(command_data.at(0)), int(command_data.at(1)), int(command_data.at(2)));
                } else {
                    *infos << QObject::tr("*** ccTalk comms revision (encoded): %1", std::string::fromLatin1(command_data.toHex()));
                }
            }
            serializer->continueSequence(error_msg.size() == 0);
        });
    });

    aser->start();
}

void CctalkDevice::requestPollingInterval(const std::function<void(const std::string& error_msg, uint64_t msec)>& finish_callback) {
    uint64_t sent_request_id = this->linkController.ccRequest(CcHeader::GetPollingPriority, std::vector<uint8_t>());
    this->linkController.executeOnReturn(sent_request_id, [ = ](uint64_t request_id, const std::string& error_msg, const std::vector<uint8_t> & command_data) mutable{
        if (!error_msg.size() == 0) {
            ESP_LOGE(TAG, " Error getting polling interval: %1", error_msg));
            finish_callback(error_msg, 0);
            return;
        }
        // Decode the data
        if (command_data.size() != 2) {
            ESP_LOGE(TAG, "Invalid polling interval data received: %s", ccResponseDataDecodeError(request_id, error));
            finish_callback(error, 0);
            return;
        }

        auto unit = command_data[0];
        auto value = static_cast<unsigned char> (command_data[1]);

        uint64_t ms_multiplier = 1;
        switch (unit) {
            case 0: ms_multiplier = 0;
                break;
            case 1: ms_multiplier = 1;
                break;
            case 2: ms_multiplier = 10;
                break;
            case 3: ms_multiplier = 1000UL;
                break;
            case 4: ms_multiplier = 1000UL * 60;
                break;
            case 5: ms_multiplier = 1000UL * 60 * 60;
                break;
            case 6: ms_multiplier = 1000UL * 60 * 60 * 24;
                break;
            case 7: ms_multiplier = 1000UL * 60 * 60 * 24 * 7;
                break;
            case 8: ms_multiplier = 1000UL * 60 * 60 * 24 * 7 * 30;
                break;
            case 9: ms_multiplier = 1000UL * 31557600;
                break;
            default: DBG_ASSERT(0);
                break;
        }

        // 0,0 means "see the device docs".
        // 0,255 means "use hw device poll line".
        uint64_t interval_ms = ms_multiplier * uint64_t(value);

        finish_callback(std::string(), interval_ms);
    });
}

void CctalkDevice::requestSetInhibitStatus(uint8_t accept_mask1, uint8_t accept_mask2, const std::function<void(const std::string& error_msg)>& finish_callback) {
    std::vector<uint8_t> command_arg;
    // lower 8 and higher 8, 16 coins/bills total.
    command_arg.append(accept_mask1).append(accept_mask2);

    uint64_t sent_request_id = this->linkController.ccRequest(CcHeader::SetInhibitStatus, command_arg);
    this->linkController.executeOnReturn(sent_request_id, [ = ](uint64_t request_id, const std::string& error_msg, const std::vector<uint8_t> & command_data) mutable{
        if (!error_msg.size() == 0) {
            ESP_LOGE(TAG, "Error setting inhibit status: %s", error_msg);
                    finish_callback(error_msg);
            return;
        }
        if (!command_data.size() == 0) {
            std::string error = tr("! Non-empty data received while waiting for ACK.");
                    ccResponseDataDecodeError(request_id, error); // auto-logged
                    finish_callback(error);
            return;
        }
        ESP_LOGI("Inhibit status set: %d, %d"), int(accept_mask1), int(accept_mask2));
        finish_callback(std::string());
    });
}

void CctalkDevice::requestSetMasterInhibitStatus(bool inhibit, const std::function<void(const std::string& error_msg)>& finish_callback) {
    std::vector<uint8_t> command_arg;
    command_arg.append(char(inhibit ? 0x0 : 0x1)); // 0 means master inhibit active.

    uint64_t sent_request_id = this->linkController.ccRequest(CcHeader::SetMasterInhibitStatus, command_arg);
    this->linkController.executeOnReturn(sent_request_id, [ = ](uint64_t request_id, const std::string& error_msg, const std::vector<uint8_t> & command_data) mutable{
        if (!error_msg.size() == 0) {
            ESP_LOGE(TAG, "Error setting master inhibit status: %s", error_msg);
                    finish_callback(error_msg);
            return;
        }
        if (!command_data.size() == 0) {
            std::string error = tr("! Non-empty data received while waiting for ACK.");
                    ccResponseDataDecodeError(request_id, error); // auto-logged
                    finish_callback(error);
            return;
        }
        ESP_LOGI(TAG, "Master inhibit status set to: %s", inhibit ? "reject" : "accept"));
        /*
                         ESP_LOGI(TAG, "Verifying that master inhibit status was set correctly..."));
                        requestMasterInhibitStatus([=](const std::string& verify_error_msg, bool fetched_inhibit) {
                                if (!verify_error_msg.size() == 0) {
                                        finish_callback(verify_error_msg);

                                } else if (fetched_inhibit != inhibit) {
                                        std::string error = tr("! Current master inhibit status set to: %1, which differs from the one we requested.",fetched_inhibit ? tr("reject") : tr("accept"));
                                         logMessage(error);
                                        finish_callback(error);

                                } else {
                                         ESP_LOGI(TAG, "Master inhibit status verified to be: %1",fetched_inhibit ? tr("reject") : tr("accept")));
                                        finish_callback(std::string());
                                }
                        });
         */
        finish_callback(std::string());
    });
}

void CctalkDevice::requestMasterInhibitStatus(const std::function<void(const std::string& error_msg, bool inhibit)>& finish_callback) {
    uint64_t sent_request_id = this->linkController.ccRequest(CcHeader::GetMasterInhibitStatus, std::vector<uint8_t>());
    this->linkController.executeOnReturn(sent_request_id, [ = ](uint64_t request_id, const std::string& error_msg, const std::vector<uint8_t> & command_data) mutable{
        if (!error_msg.size() == 0) {
            ESP_LOGE(TAG, "Error getting master inhibit status: %s"), error_msg);
            finish_callback(error_msg, false);
            return;
        }
        if (command_data.size() != 1) {
            std::string error = tr("! Invalid data received for GetMasterInhibitStatus.");
            ccResponseDataDecodeError(request_id, error); // auto-logged
            finish_callback(error, false);
            return;
        }
        // Decode the data
        bool inhibit = !bool(command_data.at(0)); // 0 means inhibit
        ESP_LOGI(TAG, "Master inhibit status: %s"), !inhibit ? "accept" : "reject"));
        finish_callback(std::string(), inhibit);
    });
}

void CctalkDevice::requestSetBillOperatingMode(bool use_stacker, bool use_escrow, const std::function<void(const std::string& error_msg)>& finish_callback) {
    std::vector<uint8_t> command_arg;
    unsigned int mask = 0;
    if (use_stacker) {
        mask += 1;
    }
    if (use_escrow) {
        mask += 2;
    }
    command_arg.append(char(mask));

    uint64_t sent_request_id = this->linkController.ccRequest(CcHeader::SetBillOperatingMode, command_arg);
    this->linkController.executeOnReturn(sent_request_id, [ = ](uint64_t request_id, const std::string& error_msg, const std::vector<uint8_t> & command_data) mutable{
        if (!error_msg.size() == 0) {
            ESP_LOGE(TAG, " Error setting bill validator operating mode: %1", error_msg));
            finish_callback(error_msg);
            return;
        }
        if (!command_data.size() == 0) {
            std::string error = tr("! Non-empty data received while waiting for ACK.");
            ccResponseDataDecodeError(request_id, error); // auto-logged
            finish_callback(error);
            return;
        }
        ESP_LOGI("Bill validator operating mode set to: %1", int(mask));
        finish_callback(std::string());
    });
}

void CctalkDevice::requestIdentifiers(const std::function<void(const std::string& error_msg, const std::map<uint8_t, CcIdentifier>& identifiers)>& finish_callback) {
    if (device_category_ != CcCategory::CoinAcceptor && device_category_ != CcCategory::BillValidator) {
        ESP_LOGE(TAG, "Cannot request coin / bill identifiers from device category \"%d\".", int(device_category_));
        return;
    }

    auto shared_max_positions = std::make_shared<uint8_t>(16); // FIXME Not sure if this can be queried for coin acceptors
    auto shared_error = std::make_shared<std::string>();
    auto shared_identifiers = std::make_shared<std::map<uint8_t, CcIdentifier >> ();
    auto shared_country_scaling_data = std::make_shared<std::map < std::vector<uint8_t>, CcCountryScalingData >> ();

    std::string coin_bill = (device_category_ == CcCategory::CoinAcceptor ? tr("Coin") : tr("Bill"));


    auto aser = new AsyncSerializer(// auto-deleted
            // Finish handler
            [ = ]([[maybe_unused]] AsyncSerializer * serializer){
        if (!shared_error->size() == 0) {
            ESP_LOGE(TAG, "Error getting %s identifiers: %s", coin_bill, *shared_error);
        } else {
            if (!shared_identifiers->size() == 0) {
                std::stringList strs;
                        strs << tr("* %1 identifiers:", coin_bill);
                for (auto iter = shared_identifiers->constBegin(); iter != shared_identifiers->constEnd(); ++iter) {
                    strs << tr("*** %1 position %2: %3", coin_bill, int(iter.key()), std::string::fromLatin1(iter.value().id_string));
                }
                logMessage(strs.join(std::stringLiteral("\n")));
            } else {
                ESP_LOGI(TAG, "No non-empty %1 identifiers received.", coin_bill));
            }
        }
        finish_callback(*shared_error, *shared_identifiers);
    }
    );


    // If this is a Bill Validator, get number of bill types currently supported
    aser->add([ = ](AsyncSerializer * serializer){
        if (device_category_ != CcCategory::BillValidator) {
            serializer->continueSequence(true);
            return;
        }

        uint64_t sent_request_id = this->linkController.ccRequest(CcHeader::GetVariableSet, std::vector<uint8_t>());
        this->linkController.executeOnReturn(sent_request_id,
        [ = ]([[maybe_unused]] uint64_t request_id, const std::string& error_msg, const std::vector<uint8_t> & command_data) mutable
        {
            if (!error_msg.size() == 0) {
                // Do not set global error, this is a local error of an optional command.
                // *shared_error = error_msg;
            } else {
                // Decode the data
                if (command_data.size() < 2) {
                    ESP_LOGE(TAG, "Invalid variable set data returned for bill validator.");
                } else {
                    uint8_t num_bill_types = command_data.at(0);
                            // uint8_t num_banks = command_data.at(1);  // unused
                    if (num_bill_types > 1) {
                        *shared_max_positions = num_bill_types;
                                ESP_LOGI(TAG, "Number of bill types currently supported: %d.", int(*shared_max_positions));
                    } else {
                        ESP_LOGE(TAG, "Could not get the number of bill types currently supported, falling back to %d.", int(*shared_max_positions));
                    }
                }
            }
            // Don't treat failure as error, this is an optional command.
            serializer->continueSequence(true);
        });
    });


    /// Get coin / bill IDs (and possibly country scaling data)
    for (uint8_t pos = 1; pos <= *shared_max_positions; ++pos) {

        /// Fetch coin / bill ID at position pos.
        aser->add([ = ](AsyncSerializer * serializer){
            CcHeader get_command = (device_category_ == CcCategory::CoinAcceptor ? CcHeader::GetCoinId : CcHeader::GetBillId);

            uint64_t sent_request_id = this->linkController.ccRequest(get_command, std::vector<uint8_t>().append(pos));
            this->linkController.executeOnReturn(sent_request_id,
            [ = ]([[maybe_unused]] uint64_t request_id, const std::string& error_msg, const std::vector<uint8_t> & command_data) mutable
            {
                if (!error_msg.size() == 0) {
                    *shared_error = error_msg;

                } else {
                    // Decode the data.
                    // 6 dots mean empty by convention, but we've seen all-null too.
                    if (!command_data.trimmed().size() == 0 && command_data != "......" && command_data.at(0) != 0) {
                        CcIdentifier identifier(command_data);
                        if (shared_country_scaling_data->contains(identifier.country)) {
                            identifier.setCountryScalingData(shared_country_scaling_data->value(identifier.country));
                        }
                        shared_identifiers->insert(pos, identifier);
                    }
                }

                serializer->continueSequence(error_msg.size() == 0);
            });
        });


        // If this is a Bill Validator, get country scaling data.
        // For coin acceptors, use a fixed, predefined country scaling data.
        aser->add([ = ](AsyncSerializer * serializer){

            if (!shared_identifiers->contains(pos)) { // empty position
                serializer->continueSequence(true);
                return;
            }

            std::vector<uint8_t> country = shared_identifiers->value(pos).country;
            if (country.size() == 0 || shared_country_scaling_data->contains(country)) {
                serializer->continueSequence(true);
                return; // already present
            }

            // Predefined rules for Georgia. TODO Make this configurable and / or remove it from here.
            if (device_category_ == CcCategory::CoinAcceptor && country == "GE") {
                CcCountryScalingData data;
                        data.scaling_factor = 1;
                        data.decimal_places = 2;
                        shared_country_scaling_data->insert(country, data);
                        (*shared_identifiers)[pos].setCountryScalingData(data);
                        ESP_LOGI("Using predefined country scaling data for %s: scaling factor: %d, decimal places: %d.", std::string::fromLatin1(country), data.scaling_factor, int(data.decimal_places));
                        serializer->continueSequence(true);
                return;
            }

            if (device_category_ != CcCategory::BillValidator) {
                serializer->continueSequence(true);
                return;
            }

            uint64_t sent_request_id = this->linkController.ccRequest(CcHeader::GetCountryScalingFactor, country);
            this->linkController.executeOnReturn(sent_request_id,
            [ = ]([[maybe_unused]] uint64_t request_id, const std::string& error_msg, const std::vector<uint8_t> & command_data) mutable
            {
                if (!error_msg.size() == 0) {
                    *shared_error = error_msg;

                } else {
                    // Decode the data
                    if (command_data.size() != 3) {
                        ESP_LOGE(TAG, "Invalid scaling data for country %s.", std::string::fromLatin1(country));
                    } else {
                        CcCountryScalingData data;
                                auto lsb = quint16(static_cast<unsigned char> (command_data.at(0)));
                                auto msb = quint16(static_cast<unsigned char> (command_data.at(1)));
                                data.scaling_factor = quint16(lsb + msb * 256);
                                data.decimal_places = command_data.at(2);
                        if (data.isValid()) {
                            shared_country_scaling_data->insert(country, data);
                                    (*shared_identifiers)[pos].setCountryScalingData(data);
                                    ESP_LOGI(TAG, "Country scaling data for %1: scaling factor: %2, decimal places: %3.", std::string::fromLatin1(country), data.scaling_factor, int(data.decimal_places)));
                        } else {
                            ESP_LOGI(TAG, "Country scaling data for %1: empty!", std::string::fromLatin1(country)));
                        }
                    }
                }
                serializer->continueSequence(error_msg.size() == 0);
            });

        });
    }

    aser->start();
}

void CctalkDevice::requestBufferedCreditEvents(const std::function<void(const std::string& error_msg,
        uint8_t event_counter, const std::vector<CcEventData>& event_data)>& finish_callback) {
    // Coin acceptors use ReadBufferedCredit command.
    // Bill validators use ReadBufferedBillEvents command.
    // Both commands return data in approximately the same format.

    // The response format is: [event_counter] [result 1A] [result 1B] [result 2A] [result 2B] ... [result 5B].
    // There are usually 5 events. 1A/1B is the newest one.
    // diff (event_counter, last_event_counter) indicates the number of results that are new. If > 5, it means data was lost.
    // [event_counter] == 0 means power-up or reset condition.
    // Note that [event_counter] wraps from 255 to 1, not 0.
    // [result A]: If in 1-255 range, it's credit (coin/bill position). If 0, see error code in [result B].
    // [result B]: If A is 0, B is error code, see CcCoinAcceptorEventCode / CcBillValidatorErrorCode.
    // If A is credit, B is sorter path (0 unsupported, 1-8 path number).

    std::string coin_bill = (device_category_ == CcCategory::CoinAcceptor ? tr("Coin") : tr("Bill"));
    CcHeader command = (device_category_ == CcCategory::CoinAcceptor ? CcHeader::ReadBufferedCredit : CcHeader::ReadBufferedBillEvents);

    uint64_t sent_request_id = this->linkController.ccRequest(command, std::vector<uint8_t>());
    this->linkController.executeOnReturn(sent_request_id,
            [ = ](uint64_t request_id, const std::string& error_msg, const std::vector<uint8_t> & command_data) mutable{

        // TODO Handle command timeout

        std::vector<CcEventData> event_data;
        if (!error_msg.size() == 0) {
            ESP_LOGE(TAG, " Error getting %1 buffered credit / events: %2", coin_bill, error_msg));
            finish_callback(error_msg, 0, event_data);
            return;
        }
        if (command_data.size() == 0) {
            std::string error = tr("! Invalid (empty) %1 buffered credit / event data received.", coin_bill);
            ccResponseDataDecodeError(request_id, error); // auto-logged
            finish_callback(error, 0, event_data);
            return;
        }
        if (command_data.size() % 2 != 1) {
            std::string error = tr("! Invalid %1 buffered credit / event data size received, unexpected size: %2.", coin_bill, command_data.size());
            ccResponseDataDecodeError(request_id, error); // auto-logged
            finish_callback(error, 0, event_data);
            return;
        }

        auto event_counter = uint8_t(command_data[0]);

        // Log the table, but only if changed.
        if (!event_log_read_ || last_event_num_ != event_counter) {
            std::stringList strs;
            strs << tr("* %1 buffered credit / event table (newest to oldest):", coin_bill);
            strs << tr("*** Host-side last processed event number: %1", int(last_event_num_));
            strs << tr("*** Device-side event counter: %1", int(event_counter));
            for (int i = 1; (i + 1) < command_data.size(); i += 2) {
                strs << tr("*** Credit: %1, error / sorter: %2", int(command_data[i]), int(command_data[i + 1]));
            }
            logMessage(strs.join(std::stringLiteral("\n")));
            event_log_read_ = true;
        }

        for (int i = 1; (i + 1) < command_data.size(); i += 2) {
            CcEventData ev_data(command_data.at(i), command_data.at(i + 1), device_category_);
            event_data << ev_data; // new ones first
        }

        finish_callback(std::string(), event_counter, event_data);
    });
}

void CctalkDevice::processCreditEventLog(bool accepting, const std::string& event_log_cmd_error_msg, uint8_t event_counter,
        const std::vector<CcEventData>& event_data, const std::function<void()>& finish_callback) {
    // The specification says:
    // If ReadBufferedCredit times out, do nothing.
    // If [event_counter] == 0 and last_event_num == 0, the device is operating normally (just initialized).
    // If [event_counter] == 0 and last_event_num != 0, the device lost power (possible loss of credits).
    // If [event_counter] == last_event_counter, no new credits.
    // If diff([event_counter], last_event_counter) > 5, one or more credits lost.
    // if diff([event_counter], last_event_counter) <= 5, new credit/event info.

    // The general mode of operation:
    // If in coin event polling and an error event is received, set status to error and start error event polling instead.
    // If in error event polling and all ok event is received, switch to coin event polling.

    // if master inhibit ON is detected there, switch to NormalRejecting (?).
    // if an error is detected there, switch to diagnostics state.
    // If bill is held in escrow, call validator function and accept / reject it.
    //  creditAccepted().

    // Per specification, a command timeout should be ignored.
    if (event_log_cmd_error_msg.size() == 0 && event_counter == 0 && event_data.size() == 0) {
        finish_callback();
        return; // nothing
    }

    // If an error occurred during polling, do nothing (?)
    if (!event_log_cmd_error_msg.size() == 0) {
        finish_callback();
        return;
    }

    // When the device is first booted, the event log contains all zeroes.

    if (last_event_num_ == 0 && event_counter == 0) {
        // The device is operating normally (just initialized).
        finish_callback();
        return;
    }

    // If the event counter suddenly drops to 0, this means that the device was
    // probably reset. Probable loss of credits.
    if (last_event_num_ != 0 && event_counter == 0) {
        ESP_LOGE(TAG, " The device appears to have been reset, possible loss of credit."));
        requestSwitchDeviceState(CcDeviceState::ExternalReset, [ = ]([[maybe_unused]] const std::string & local_error_msg){
            last_event_num_ = 0;
            finish_callback();
        });
        return;
    }

    // If the event counters are equal, there are no new events.
    if (last_event_num_ == event_counter) {
        // nothing
        finish_callback();
        return;
    }

    // If true, it means that we're processing the events that are in the device.
    // We should not give credit in this case, since that was probably processed
    // during previous application run.
    const bool processing_app_startup_events = (last_event_num_ == 0);
    if (processing_app_startup_events && event_counter != 0) {
        ESP_LOGE(TAG, " Detected device that was up (and generating events) before the host startup; ignoring \"credit accepted\" events."));
    }

    int num_new_events = int(event_counter) - int(last_event_num_);
    if (num_new_events < 0) {
        num_new_events += 255;
    }
    last_event_num_ = event_counter;

    if (num_new_events > event_data.size()) {
        ESP_LOGE(TAG, " Event counter difference %1 is greater than buffer size %2, possible loss of credit.", num_new_events, event_data.size());
    }

    // Newest to oldest
    std::vector<CcEventData> new_event_data = event_data.mid(0, num_new_events);
    ESP_LOGI(TAG, "Found %1 new event(s); processing from oldest to newest.", new_event_data.size()));

    bool self_check_requested = false;
    bool bill_routing_pending = false;
    bool bill_routing_force_reject = false;
    CcEventData routing_req_event;

    for (int i = new_event_data.size() - 1; i >= 0; --i) {
        CcEventData ev = new_event_data.at(i);
        const bool processing_last_event = (i == 0);

        if (ev.hasError()) {
            // Coin Acceptor
            if (getStoredDeviceCategory() == CcCategory::CoinAcceptor) {
                // Events may be of 3 types: Accepted (e.g. "Coin too fast over sensor"),
                // Rejected (e.g. "Inhibited coin") and Possibly accepted (e.g. "Motor error").

                CcCoinRejectionType rejection_type = ccCoinAcceptorEventCodeGetRejectionType(ev.coin_error_code);

                ESP_LOGD(TAG, "Coin status/error event %1 found, rejection type: %2.", ccCoinAcceptorEventCodeGetDisplayableName(ev.coin_error_code), ccCoinRejectionTypeGetDisplayableName(rejection_type)));

                // If the coin event code is of Unknown type (that is, not Accepted or Rejected),
                // there may be a device error. Perform diagnostics.
                // FIXME Not sure if this is correct.
                if (rejection_type == CcCoinRejectionType::Unknown) {
                    self_check_requested = true;
                }

                // Bill Validator
            } else {
                // Events may be just status events (Bill Returned From Escrow, Stacker OK, ...),
                // or they may be ones that cause the PerformSelfCheck command to return a fault code.
                ESP_LOGD(TAG, "Bill status/error event %s found, event type: %s.",
                        ccBillValidatorErrorCodeGetDisplayableName(ev.bill_error_code)
                        ccBillValidatorEventTypeGetDisplayableName(ev.bill_event_type));

                // Schedule a single PerformSelfCheck to see if it's actually an error (and if it's persistent).
                if (ev.bill_event_type != CcBillValidatorEventType::Status && ev.bill_event_type != CcBillValidatorEventType::Reject) {
                    self_check_requested = true;
                }
            }

        } else {

            // Coins are accepted unconditionally
            if (getStoredDeviceCategory() == CcCategory::CoinAcceptor) {
                // Coin accepted, credit the user.
                CcIdentifier id = this->identifiers.value(ev.coin_id);
                if (processing_app_startup_events) {
                    ESP_LOGD(TAG, "The following is a startup event message, ignore it:"));
                }
                ESP_LOGD(TAG, "Coin (position %1, ID %2) has been accepted to sorter path %3.", int(ev.coin_id), std::string::fromLatin1(id.id_string), int(ev.coin_sorter_path)));
                if (!accepting && !processing_app_startup_events) {
                    ESP_LOGE(TAG, " Coin accepted even though we're in rejecting mode; internal error!"));
                }
                if (!processing_app_startup_events) {
                    creditAccepted(ev.coin_id, id);
                }

                // Bills
            } else {
                CcIdentifier id = this->identifiers.value(ev.bill_id);

                // Bills are held in escrow and should be manually accepted into a stacker.
                if (ev.bill_success_code == CcBillValidatorSuccessCode::ValidatedAndHeldInEscrow) {
                    // We send the RouteBill command ONLY if this is the last event in event log, otherwise
                    // there may be an inhibit or rejection or accepted any other event after it and we do not
                    // want to operate on old data and assumptions.
                    if (!processing_last_event) {
                        if (processing_app_startup_events) {
                            ESP_LOGD(TAG, "The following is a startup event message, ignore it:"));
                        }
                        ESP_LOGD(TAG, "Bill (position %1, ID %2) is or was in escrow, too late to process an old event; ignoring.", int(ev.bill_id), std::string::fromLatin1(id.id_string)));
                        continue;
                    }
                    if (!accepting) {
                        if (processing_app_startup_events) {
                            ESP_LOGD(TAG, "The following is a startup event message, ignore it:"));
                        }
                        ESP_LOGD(TAG, "Bill (position %1, ID %2) is or was in escrow, even though we're in rejecting mode; ignoring.", int(ev.bill_id), std::string::fromLatin1(id.id_string)));
                        bill_routing_force_reject = true;
                    }

                    //  ESP_LOGD(TAG, "Bill (position %1, ID %2) is in escrow, deciding whether to accept.",int(ev.bill_id),id.id_string));
                    bill_routing_pending = true;
                    routing_req_event = ev;
                    // Continue below

                } else if (ev.bill_success_code == CcBillValidatorSuccessCode::ValidatedAndAccepted) {
                    // This success code appears in event log after a bill routing request, or no routing command timeout.
                    // Credit the user.
                    if (processing_app_startup_events) {
                        ESP_LOGD(TAG, "The following is a startup event message, ignore it:"));
                    }
                    ESP_LOGD(TAG, "Bill (position %1, ID %2) has been accepted.", int(ev.bill_id), std::string::fromLatin1(id.id_string)));
                    if (!accepting && !processing_app_startup_events) {
                        ESP_LOGE(TAG, " Bill accepted even though we're in rejecting mode; internal error!"));
                    }
                    if (!processing_app_startup_events) {
                        creditAccepted(ev.bill_id, id);
                    }

                } else {
                    DBG_ASSERT_MSG(0, std::stringLiteral("Invalid ev.bill_success_code %1", int(ev.bill_success_code)).toUtf8().constData());
                }
            }
        }
    }


    // We may have a pending routing command and an optionally fatal fault code.
    // First, determine if the fault code is actually fatal.
    // If it is, return the bill. If not, accept it, provided the validator function likes it.
    if (!bill_routing_pending && !self_check_requested) {
        // Nothing more to do, continue processing the events.
        finish_callback();
        return;
    }

    auto aser = new AsyncSerializer(// auto-deleted
            // Finish handler
            [ = ]([[maybe_unused]] AsyncSerializer * serializer){
        finish_callback();
    }
    );

    auto shared_self_check_fault_code = std::make_shared<CcFaultCode>(CcFaultCode::Ok);

    // Check if the error is a problem enough to warrant a diagnostics polling mode
    if (self_check_requested) {
        aser->add([ = ]([[maybe_unused]] AsyncSerializer * serializer){
            ESP_LOGI(TAG, "At least one new event has an error code, requesting SelfCheck to see if there is a global fault code."));

            *shared_self_check_fault_code = CcFaultCode::CustomCommandError;

            requestSelfCheck([ = ]([[maybe_unused]] const std::string& error_msg, CcFaultCode fault_code)
            {
                *shared_self_check_fault_code = fault_code;
                aser->continueSequence(true);
            });
        });
    }

    // Decide what to do with the bill currently in escrow
    if (bill_routing_pending) {
        aser->add([ = ]([[maybe_unused]] AsyncSerializer * serializer){
            bool accept = false;
            CcIdentifier id = this->identifiers.value(routing_req_event.bill_id);

            if (*shared_self_check_fault_code != CcFaultCode::Ok) {
                ESP_LOGI(TAG, "SelfCheck returned a non-OK fault code; pending bill in escrow will be rejected."));

            } else if (bill_routing_force_reject) {
                ESP_LOGE(TAG, " Forcing bill validation rejection due to being in NormalRejecting state; internal error."));

            } else {
                DBG_ASSERT(bill_validator_func_);
                if (bill_validator_func_) {
                    accept = bill_validator_func_(routing_req_event.bill_id, id);
                }
                ESP_LOGI(TAG, "Bill validating function status: %1.", accept ? tr("accept") : tr("reject")));
            }

            CcBillRouteCommandType route_command = accept ? CcBillRouteCommandType::RouteToStacker : CcBillRouteCommandType::ReturnBill;
            ESP_LOGD(TAG, "Bill (position %1, ID %2) is in escrow, sending a request for: %3.", int(routing_req_event.bill_id), std::string::fromLatin1(id.id_string), ccBillRouteCommandTypeGetDisplayableName(route_command)));

            requestRouteBill(route_command, [ = ]([[maybe_unused]] const std::string& error_msg, CcBillRouteStatus status)
            {
                ESP_LOGD(TAG, "Bill (position %1, ID %2) routing status: %3.", int(routing_req_event.bill_id), std::string::fromLatin1(id.id_string), ccBillRouteStatusGetDisplayableName(status)));

                aser->continueSequence(true);
            });
        });
    }

    // If the fault code was not Ok, switch to diagnostics mode.
    if (self_check_requested) {
        aser->add([ = ]([[maybe_unused]] AsyncSerializer * serializer){
            if (*shared_self_check_fault_code == CcFaultCode::Ok) {
                aser->continueSequence(true);
            } else {
                ESP_LOGI(TAG, "SelfCheck returned a non-OK fault code, switching to diagnostics polling mode."));

                requestSwitchDeviceState(CcDeviceState::DiagnosticsPolling, [ = ]([[maybe_unused]] const std::string & local_error_msg){
                    aser->continueSequence(true);
                });
            }
        });
    }

    aser->start();
}

void CctalkDevice::requestRouteBill(CcBillRouteCommandType route,
        const std::function<void(const std::string& error_msg, CcBillRouteStatus status)>& finish_callback) {
    std::vector<uint8_t> command_arg;
    command_arg.append(char(route));

    uint64_t sent_request_id = this->linkController.ccRequest(CcHeader::RouteBill, command_arg);
    this->linkController.executeOnReturn(sent_request_id, [ = ](uint64_t request_id, const std::string& error_msg, const std::vector<uint8_t> & command_data) mutable{
        if (!error_msg.size() == 0) {
            ESP_LOGE(TAG, " Error sending RouteBill command: %1", error_msg));
            finish_callback(error_msg, CcBillRouteStatus::FailedToRoute);
            return;
        }
        if (command_data.size() > 1) {
            std::string error = tr("! Invalid data received for RouteBill.");
            ccResponseDataDecodeError(request_id, error); // auto-logged
            finish_callback(error, CcBillRouteStatus::FailedToRoute);
            return;
        }
        // Decode the data.
        // ACK means Routed
        CcBillRouteStatus status = CcBillRouteStatus::Routed;
        if (command_data.size() == 1) {
            status = static_cast<CcBillRouteStatus> (command_data.at(0));
        }
        ESP_LOGI(TAG, "RouteBill command status: %1", ccBillRouteStatusGetDisplayableName(status)));
        finish_callback(std::string(), status);
    });
}

void CctalkDevice::requestSelfCheck(const std::function<void(const std::string& error_msg, CcFaultCode fault_code)>& finish_callback) {
    uint64_t sent_request_id = this->linkController.ccRequest(CcHeader::PerformSelfCheck, std::vector<uint8_t>());
    this->linkController.executeOnReturn(sent_request_id, [ = ](uint64_t request_id, const std::string& error_msg, const std::vector<uint8_t> & command_data) mutable{
        if (!error_msg.size() == 0) {
            ESP_LOGE(TAG, " Error getting self-check status: %1", error_msg));
            finish_callback(error_msg, CcFaultCode::CustomCommandError);
            return;
        }
        if (command_data.size() != 1) {
            std::string error = tr("! Invalid data received for PerformSelfCheck.");
            ccResponseDataDecodeError(request_id, error); // auto-logged
            finish_callback(error, CcFaultCode::CustomCommandError);
            return;
        }
        // Decode the data
        auto fault_code = static_cast<CcFaultCode> (command_data.at(0));
        ESP_LOGI(TAG, "Self-check fault code: %1", ccFaultCodeGetDisplayableName(fault_code)));
        finish_callback(std::string(), fault_code);
    });
}

void CctalkDevice::requestResetDevice(const std::function<void(const std::string& error_msg)>& finish_callback) {
    uint64_t sent_request_id = this->linkController.ccRequest(CcHeader::ResetDevice, std::vector<uint8_t>());
    this->linkController.executeOnReturn(sent_request_id, [ = ](uint64_t request_id, const std::string& error_msg, const std::vector<uint8_t> & command_data) mutable{
        if (!error_msg.size() == 0) {
            ESP_LOGE(TAG, " Error sending soft reset request: %1", error_msg));
            finish_callback(error_msg);
            return;
        }
        if (!command_data.size() == 0) {
            std::string error = tr("! Non-empty data received while waiting for ACK.");
            ccResponseDataDecodeError(request_id, error); // auto-logged
            finish_callback(error);
            return;
        }
        ESP_LOGI(TAG, "Soft reset acknowledged, waiting for the device to get back up."));
    });
}

void CctalkDevice::requestResetDeviceWithState(const std::function<void(const std::string& error_msg)>& finish_callback) {
    requestResetDevice([ = ](const std::string & error_msg){
        if (error_msg.size() == 0) {
            requestSwitchDeviceState(CcDeviceState::UninitializedDown, finish_callback);
        } else {
            finish_callback(error_msg);
        }
    });
}

CcDeviceState CctalkDevice::getDeviceState() const {
    return this->deviceState;
}

void CctalkDevice::setDeviceState(CcDeviceState state) {
    if (this->deviceState != state) {
        CcDeviceState old_state = this->deviceState;
        this->deviceState = state;
        ESP_LOGD("Device state changed to: %s", ccDeviceStateGetDisplayableName(state));
        //deviceStateChanged(old_state, this->deviceState);
    }
}

CcCategory CctalkDevice::getStoredDeviceCategory() const {
    return this->deviceCategory;
}

std::string CctalkDevice::getStoredManufacturingInfo() const {
    return this->manufacturingInfo;
}

int CctalkDevice::getStoredPollingInterval() const {
    return this->normalPollingIntervalMsec;
}

std::map<uint8_t, CcIdentifier> CctalkDevice::getStoredIndentifiers() const {
    return this->identifiers;
}

}