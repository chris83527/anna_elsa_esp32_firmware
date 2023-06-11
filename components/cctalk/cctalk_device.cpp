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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/projdefs.h"

#include "cctalk_device.h"
#include "cctalk_enums.h"

static const char* TAG = "cctalkDevice";

namespace esp32cc {

    CctalkDevice::CctalkDevice() {

    }

    CctalkDevice::CctalkDevice(const CctalkDevice& orig) {

    }

    CctalkDevice::~CctalkDevice() {

    }

    CctalkLinkController* CctalkDevice::getLinkController() {
        return this->linkController;
    }

    bool CctalkDevice::initialise(CctalkLinkController* linkController, const uint8_t deviceAddress, const std::function<void(const std::string& error_msg)>& finish_callback) {
        this->linkController = linkController;

        this->deviceAddress = deviceAddress;
        this->lastEventNumber = 0;

        if (getDeviceState() != CcDeviceState::ShutDown) {
            ESP_LOGE(TAG, "Cannot initialise device that is in %s state.", ccDeviceStateGetDisplayableName(getDeviceState()).c_str());
            return false;
        }

        return requestSwitchDeviceState(CcDeviceState::Initialized, [ & ](const std::string & error_msg) {
            finish_callback(error_msg);
        });
    }

    bool CctalkDevice::shutdown(const std::function<void(const std::string& error_msg)>& finish_callback) {
        return requestSwitchDeviceState(CcDeviceState::ShutDown, [ & ](const std::string & error_msg) {
            finish_callback(error_msg);
        });
    }

    void CctalkDevice::startPolling() {
        ESP_LOGD(TAG, "Starting poll timer.");

        // prevent a new thread from being spawned 
        if (!this->isPolling) {
            isPolling = true;
            auto cfg = esp_pthread_get_default_config();
            cfg.thread_name = ccCategoryDisplayNameFromCategory(this->getStoredDeviceCategory()).append("Polling").c_str();
            cfg.prio = 9;
            cfg.stack_size = 8192;
            esp_pthread_set_cfg(&cfg);
            pollThread.reset(new std::thread([this] {
                devicePollTask();
            }));
        }

    }

    void CctalkDevice::stopPolling() {
        ESP_LOGD(TAG, "Stopping poll timer.");

        this->isPolling = false;
        pollThread.get()->join();

    }

    void CctalkDevice::devicePollTask() {

        while (1) {
            ESP_LOGD(TAG, "Polling %s...", ccCategoryDisplayNameFromCategory(this->getStoredDeviceCategory()).c_str());

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
                    requestCheckAlive([ & ]([[maybe_unused]] const std::string& error_msg, bool alive) {
                        if (alive) {
                            requestSwitchDeviceState(CcDeviceState::Initialized, [ & ]([[maybe_unused]] const std::string & local_error_msg) {
                                this->isTimerIterationTaskRunning = false;
                            });
                        } else {
                            this->isTimerIterationTaskRunning = false;
                        }
                    });
                    break;
                case CcDeviceState::Initialized:
                    switch (this->deviceCategory) {
                        case CcCategory::Payout:
                            // Enable hopper
                            ESP_LOGD(TAG, "Enabling hopper");
                            enableHopper([&](const std::string & error_msg) {
                                if (error_msg.size() > 0) {
                                    ESP_LOGE(TAG, "Enable hopper command failed: %s", error_msg.c_str());
                                }
                            });
                            break;
                        case CcCategory::CoinAcceptor:
                        case CcCategory::BillValidator:
                            // The device has been initialized, resume normal rejecting or diagnostics polling.
                            // Default to bill / coin rejection.
                            // Perform a self-check and see if everything's ok.
                            requestSelfCheck([ & ]([[maybe_unused]] const std::string& error_msg, CcFaultCode fault_code) {
                                if (fault_code == CcFaultCode::Ok) {
                                    // The device is OK, resume normal rejecting mode.
                                    requestSwitchDeviceState(CcDeviceState::NormalRejecting, [ & ]([[maybe_unused]] const std::string & local_error_msg) {
                                        this->isTimerIterationTaskRunning = false;
                                    });
                                } else {
                                    // The device is not ok, resume diagnostics polling mode.
                                    requestSwitchDeviceState(CcDeviceState::DiagnosticsPolling, [ & ]([[maybe_unused]] const std::string & local_error_msg) {
                                        this->isTimerIterationTaskRunning = false;
                                    });
                                }
                            });
                            break;
                        default:
                            break;
                    }
                    break;
                case CcDeviceState::InitializationFailed:
                    // The device initialization failed, something wrong with it. Abort.
                    this->isTimerIterationTaskRunning = false;
                    // Nothing we can do, cannot work with this device.
                    stopPolling();
                    break;
                case CcDeviceState::NormalAccepting:
                    switch (this->deviceCategory) {
                        case CcCategory::Payout:
                            // We just need to regularly poll the hopper
                            // Test hopper
                            //ESP_LOGD(TAG, "Testing hopper");
                            requestHopperStatus([ & ](const std::string & error_msg, uint8_t event_counter, const std::vector<CcEventData>& hopperStatusData) {
                                processHopperStatus(error_msg, event_counter, hopperStatusData, [ & ]() {
                                    this->isTimerIterationTaskRunning = false;
                                });
                            });
                            break;
                        case CcCategory::CoinAcceptor:
                        case CcCategory::BillValidator:
                            // We're accepting the credit, process the credit / event log.
                            requestBufferedCreditEvents([ & ](const std::string& error_msg, uint8_t event_counter, const std::vector<CcEventData>& event_data) {
                                processCreditEventLog(true, error_msg, event_counter, event_data, [ & ]() {
                                    this->isTimerIterationTaskRunning = false;
                                });
                            });
                            break;
                        default:
                            break;
                    }

                    break;
                case CcDeviceState::NormalRejecting:
                    requestBufferedCreditEvents([ & ](const std::string& error_msg, uint8_t event_counter, const std::vector<CcEventData>& event_data) {
                        processCreditEventLog(false, error_msg, event_counter, event_data, [ & ]() {
                            isTimerIterationTaskRunning = false;
                        });
                    });
                    break;
                case CcDeviceState::DiagnosticsPolling:
                    // If we're in diagnostics polling mode, poll the fault code until it's resolved,
                    // then switch to rejecting mode.
                    requestSelfCheck([ & ]([[maybe_unused]] const std::string& error_msg, CcFaultCode fault_code) {
                        if (fault_code == CcFaultCode::Ok) {
                            // The error has been resolved, switch to rejecting mode.
                            requestSwitchDeviceState(CcDeviceState::NormalRejecting, [ & ]([[maybe_unused]] const std::string & state_error_msg) {
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
                    requestSwitchDeviceState(CcDeviceState::Initialized, [ & ]([[maybe_unused]] const std::string & error_msg) {
                        isTimerIterationTaskRunning = false;
                    });
                    break;
                case CcDeviceState::ExternalReset:
                    // The device event log turned out to be empty (after being non-empty). This means
                    // that the device was probably reset externally, with possible loss of credits.
                    // Assume it needs initialization.
                    requestSwitchDeviceState(CcDeviceState::Initialized, [ & ]([[maybe_unused]] const std::string & error_msg) {
                        isTimerIterationTaskRunning = false;
                    });
                    break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(this->normalPollingIntervalMsec));
        }
    }

    bool CctalkDevice::requestSwitchDeviceState(CcDeviceState state, const std::function<void(const std::string& error_msg)>& finish_callback) {
        ESP_LOGD(TAG, "Requested device state change from %s to: %s", ccDeviceStateGetDisplayableName(getDeviceState()).c_str(), ccDeviceStateGetDisplayableName(state).c_str());

        if (this->deviceState == state) {
            ESP_LOGW(TAG, "Cannot switch to device state %s, already there.", ccDeviceStateGetDisplayableName(state).c_str());
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
                this->pollingInterval = normalPollingIntervalMsec;
                bool success = switchStateDiagnosticsPolling(finish_callback);
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

        //        assert((this->deviceState == CcDeviceState::ShutDown || this->deviceState == CcDeviceState::ExternalReset
        //                || this->deviceState == CcDeviceState::UnexpectedDown || this->deviceState == CcDeviceState::UninitializedDown) == false);

        auto shared_error = std::make_shared<std::string>();
        auto shared_alive = std::make_shared<bool>();
        auto shared_continue = std::make_shared<bool>();

        *shared_continue = true;

        if (shared_error->size() == 0) {
            setDeviceState(CcDeviceState::Initialized);
            finish_callback(*shared_error);
        } else if (*shared_alive) {
            requestSwitchDeviceState(CcDeviceState::InitializationFailed, finish_callback);
            *shared_continue = false;
        } else {
            requestSwitchDeviceState(CcDeviceState::UninitializedDown, finish_callback);
            *shared_continue = false;
        }

        if (!*shared_continue) {
            return false;
        }

        ESP_LOGD(TAG, "Requesting checkAlive");
        // Check if it's present / alive
        requestCheckAlive([ & ](const std::string& error_msg, bool alive) {
            if (!error_msg.size() == 0) {
                *shared_error = error_msg;
                *shared_continue = false;
            }
            *shared_alive = alive;
        });

        if (!*shared_continue) {
            return false;
        }

        ESP_LOGD(TAG, "Requesting manufacturing info");
        // Get device manufacturing info
        requestManufacturingInfo([ & ](const std::string& error_msg, CcCategory category, const std::string & info) {
            if (!error_msg.size() == 0) {
                *shared_error = error_msg;
            } else {
                this->deviceCategory = category;
                this->manufacturingInfo = info;
            }

            if (error_msg.size() > 0 || (category != CcCategory::BillValidator && category != CcCategory::CoinAcceptor && category != CcCategory::Payout)) {
                *shared_continue = false;
            }
        });

        if (!*shared_continue) {
            return false;
        }

        if (this->deviceCategory == CcCategory::CoinAcceptor || this->deviceCategory == CcCategory::BillValidator) {
            ESP_LOGD(TAG, "Requesting polling interval");
            // Get recommended polling frequency        
            requestPollingInterval([ & ](const std::string& error_msg, uint64_t msec) {
                if (!error_msg.size() == 0) {
                    *shared_error = error_msg;
                } else {
                    // For very large values and unsupported values pick reasonable defaults.
                    const uint64_t max_interval_msec = 1000;
                    if (msec == 0 || msec > max_interval_msec) { // usually means "see device docs".
                        ESP_LOGD(TAG, "Device-recommended polling frequency is invalid, using our default: %d", this->defaultNormalPollingIntervalMsec);
                        this->normalPollingIntervalMsec = this->defaultNormalPollingIntervalMsec;
                    } else {
                        ESP_LOGD(TAG, "Device-recommended polling frequency: %d", int(msec));
                        this->normalPollingIntervalMsec = int(msec);
                    }
                }

                if (error_msg.size() > 0) {
                    *shared_continue = false;
                }
            });

            if (!*shared_continue) {
                return false;
            }

            // Get bill / coin identifiers (Only do this if coin /bill validator. Not for Hopper)
            if (this->deviceCategory == CcCategory::CoinAcceptor || this->deviceCategory == CcCategory::BillValidator) {
                ESP_LOGD(TAG, "Requesting identifiers");
                requestIdentifiers([ & ](const std::string& error_msg, const std::map<uint8_t, CcIdentifier>& identifiers) {
                    if (!error_msg.size() == 0) {
                        *shared_error = error_msg;
                    } else {
                        this->identifiers = identifiers;
                    }
                    if (error_msg.size() > 0) {
                        *shared_continue = false;
                    }
                });
            }

            if (!shared_continue) {
                return false;
            }

            // Modify bill validator operating mode - enable escrow and stacker        
            if (this->deviceCategory == CcCategory::BillValidator) {
                modifyBillOperatingMode(true, true, [ & ](const std::string & error_msg) {
                    if (!error_msg.size() == 0) {
                        *shared_error = error_msg;
                    }
                    if (error_msg.size() > 0) {
                        *shared_continue = false;
                    }
                });
            }

            if (!*shared_continue) {
                return false;
            }

            // Set individual inhibit status on all bills / coins. The specification says
            // that this is not needed for coin acceptors, but the practice shows it is.        
            modifyInhibitStatus(0xff, 0xff, [ & ](const std::string & error_msg) {
                if (!error_msg.size() == 0) {
                    *shared_error = error_msg;
                }

                if (error_msg.size() > 0) {
                    *shared_continue = false;
                }
            });

        }

        return *shared_continue;
    }

    bool CctalkDevice::switchStateNormalAccepting(const std::function<void(const std::string& error_msg)>& finish_callback) {
        //assert((this->deviceState == CcDeviceState::Initialized
        //        || this->deviceState == CcDeviceState::NormalRejecting || this->deviceState == CcDeviceState::DiagnosticsPolling) != false);

        // Disable master inhibit.
        modifyMasterInhibitStatus(false, [ & ](const std::string & error_msg) {
            if (error_msg.size() == 0) {
                setDeviceState(CcDeviceState::NormalAccepting);
                ESP_LOGD(TAG, "Device switched to NormalAccepting state");
                finish_callback(error_msg);
            } else {
                ESP_LOGE(TAG, "An error occurred modifying master inhibit status: %s", error_msg.c_str());
                requestSwitchDeviceState(CcDeviceState::UnexpectedDown, finish_callback);
            }
        });

        return true;
    }

    bool CctalkDevice::switchStateNormalRejecting(const std::function<void(const std::string& error_msg)>& finish_callback) {
        //assert((this->deviceState == CcDeviceState::Initialized
        //        || this->deviceState == CcDeviceState::NormalAccepting || this->deviceState == CcDeviceState::DiagnosticsPolling) == false);

        // Enable master inhibit.
        modifyMasterInhibitStatus(true, [ & ](const std::string & error_msg) {
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

        // Enable master inhibit (if possible).
        // In theory, this is redundant since the device itself will enable it if a fault
        // is detected, but just in case of a software logic error...
        modifyMasterInhibitStatus(true, [ & ](const std::string & error_msg) {
            if (error_msg.size() == 0) {
                setDeviceState(CcDeviceState::DiagnosticsPolling);
                finish_callback(error_msg);
            } else {
                ESP_LOGE(TAG, "An error occurred while modifying master inhibit status: %s", error_msg.c_str());
                requestSwitchDeviceState(CcDeviceState::UnexpectedDown, finish_callback);
            }
        });

        return true;
    }

    bool CctalkDevice::switchStateShutDown(const std::function<void(const std::string& error_msg)>& finish_callback) {
        // If the device is in accepting mode, switch it off
        if (this->deviceState == CcDeviceState::NormalAccepting) {
            modifyMasterInhibitStatus(true, [ & ](const std::string & error_msg) {
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

    void CctalkDevice::requestCheckAlive(const std::function<void(const std::string& errorMsg, bool alive)>& finish_callback) {

        ESP_LOGD(TAG, "Sending request for SimplePoll");
        std::vector<uint8_t> data;
        this->linkController->ccRequest(CcHeader::SimplePoll, this->deviceAddress, data, 200, [ & ](const std::string error_msg, const std::vector<uint8_t>& responseData) {
            if (!error_msg.size() == 0) {
                ESP_LOGE(TAG, "Error checking for device alive status (simple poll): %s", error_msg.c_str());
                finish_callback(error_msg, false);
                return;
            }

            if (!responseData.size() == 0) {
                std::string error = "Non-empty data received while waiting for ACK.";
                // //ccResponseDataDecodeError(request_id, error); // auto-logged
                finish_callback(error, false);
                return;
            }

            ESP_LOGD(TAG, "Device is alive (answered to simple poll)");

            finish_callback(std::string(), true);
        });
    }

    void CctalkDevice::requestManufacturingInfo(const std::function<void(const std::string& error_msg, CcCategory& category, const std::string& info)>& finish_callback) {
        auto shared_error = std::make_shared<std::string>();
        CcCategory category;
        std::string info;
        std::vector<uint8_t> data;

        // Category                    
        this->linkController->ccRequest(CcHeader::RequestEquipmentCategoryId, this->deviceAddress, data, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t>& responseData) {
            if (!error_msg.size() == 0) {
                *shared_error = error_msg;
            } else {
                // Decode the data
                std::string decoded = decodeResponseToString(responseData);
                info.append("*** Equipment category: " + decoded + "\n");
                category = ccCategoryFromReportedName(decoded);
            }
        });

        if ((*shared_error).size() != 0) {
            return;
        }

        // Product code        
        this->linkController->ccRequest(CcHeader::RequestProductCode, this->deviceAddress, data, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {
            if (!error_msg.size() == 0) {
                *shared_error = error_msg;
            } else {
                // Decode the data                
                info.append("*** Product code: " + decodeResponseToString(responseData) + "\n");
            }
        });


        // Build code        
        this->linkController->ccRequest(CcHeader::RequestBuildCode, this->deviceAddress, data, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {
            if (!error_msg.size() == 0) {
                *shared_error = error_msg;
            } else {
                // Decode the data
                info.append("*** Build code: " + decodeResponseToString(responseData) + "\n");
            }
        });

        // Manufacturer        
        this->linkController->ccRequest(CcHeader::RequestManufacturerId, this->deviceAddress, data, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {
            if (!error_msg.size() == 0) {
                *shared_error = error_msg;
            } else {
                // Decode the data
                std::string decoded = decodeResponseToString(responseData);
                info.append("*** Manufacturer: " + decodeResponseToString(responseData) + "\n");
            }
        });

        // S/N        
        this->linkController->ccRequest(CcHeader::RequestSerialNumber, this->deviceAddress, data, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t>& responseData) {
            if (!error_msg.size() == 0) {
                *shared_error = error_msg;
            } else {
                // Decode the data                
                info.append("*** Serial number: " + decodeResponseToHex(responseData) + "\n");
            }
        });


        // Software revision           
        this->linkController->ccRequest(CcHeader::RequestSoftwareRevision, this->deviceAddress, data, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t>& responseData) {
            if (!error_msg.size() == 0) {
                *shared_error = error_msg;
            } else {
                // Decode the data                
                info.append("*** Software Revision: " + decodeResponseToString(responseData) + "\n");
            }
        });


        // ccTalk command set revision        
        this->linkController->ccRequest(CcHeader::RequestCommsRevision, this->deviceAddress, data, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t>& responseData) {
            if (!error_msg.size() == 0) {
                *shared_error = error_msg;
            } else {
                if (responseData.size() == 3) {
                    info.append("*** ccTalk product release: " + std::to_string(responseData.at(0)) + ", ccTalk version " + std::to_string(responseData.at(1)) + "." + std::to_string(responseData.at(1)));
                } else {
                    info.append("*** ccTalk comms revision (encoded): " + decodeResponseToHex(responseData) + "\n");
                }
            }
        });

        finish_callback(*shared_error, category, info);
    }

    void CctalkDevice::requestPollingInterval(const std::function<void(const std::string& error_msg, uint64_t msec)>& finish_callback) {
        std::vector<uint8_t> data;

        this->linkController->ccRequest(CcHeader::RequestPollingPriority, this->deviceAddress, data, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {
            if (!error_msg.size() == 0) {
                ESP_LOGE(TAG, " Error getting polling interval: %s", error_msg.c_str());
                finish_callback(error_msg, 0);
                return;
            }
            // Decode the data
            if (responseData.size() != 2) {
                ESP_LOGE(TAG, "Invalid polling interval data received");
                finish_callback("Invalid polling interval data received", 0);
                return;
            }

            auto unit = responseData[0];
            auto value = static_cast<unsigned char> (responseData[1]);

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
                    //default: assert(true);

                    //  break;
            }

            // 0,0 means "see the device docs".
            // 0,255 means "use hw device poll line".
            uint64_t interval_ms = ms_multiplier * uint64_t(value);

            finish_callback(std::string(), interval_ms);
        });
    }

    void CctalkDevice::modifyInhibitStatus(uint8_t accept_mask1, uint8_t accept_mask2, const std::function<void(const std::string& error_msg)>& finish_callback) {
        std::vector<uint8_t> command_arg;
        // lower 8 and higher 8, 16 coins/bills total.
        command_arg.push_back(accept_mask1);
        command_arg.push_back(accept_mask2);

        this->linkController->ccRequest(CcHeader::ModifyInhibitStatus, this->deviceAddress, command_arg, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {
            if (error_msg.size() != 0) {
                ESP_LOGE(TAG, "Error setting inhibit status: %s", error_msg.c_str());
                finish_callback(error_msg);
                return;
            }
            if (responseData.size() > 0) {
                std::string error = "Non-empty data received while waiting for ACK.";
                //ccResponseDataDecodeError(request_id, error); // auto-logged
                finish_callback(error);

                return;
            }
            ESP_LOGD(TAG, "Inhibit status set: %d, %d", int(accept_mask1), int(accept_mask2));
            finish_callback(std::string());
        });
    }

    void CctalkDevice::modifyMasterInhibitStatus(bool inhibit, const std::function<void(const std::string& error_msg)>& finish_callback) {
        std::vector<uint8_t> command_arg;
        command_arg.push_back(char(inhibit ? 0x0 : 0x1)); // 0 means master inhibit active.        
        this->linkController->ccRequest(CcHeader::ModifyMasterInhibitStatus, this->deviceAddress, command_arg, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {
            if (error_msg.size() > 0) {
                ESP_LOGE(TAG, "Error setting master inhibit status: %s", error_msg.c_str());
                finish_callback(error_msg);
                return;
            }
            if (responseData.size() > 0) {
                std::string error = "Non-empty data received while waiting for ACK.";
                finish_callback(error);

                return;
            }
            ESP_LOGD(TAG, "Master inhibit status set to: %s", inhibit ? "reject" : "accept");
            /*
                             ESP_LOGD(TAG, "Verifying that master inhibit status was set correctly..."));
                            requestMasterInhibitStatus([=](const std::string& verify_error_msg, bool fetched_inhibit) {
                                    if (!verify_error_msg.size() == 0) {
                                            finish_callback(verify_error_msg);

                                    } else if (fetched_inhibit != inhibit) {
                                            std::string error = tr("! Current master inhibit status set to: %1, which differs from the one we requested.",fetched_inhibit ? tr("reject") : tr("accept"));
                                             logMessage(error);
                                            finish_callback(error);

                                    } else {
                                             ESP_LOGD(TAG, "Master inhibit status verified to be: %1",fetched_inhibit ? tr("reject") : tr("accept")));
                                            finish_callback(std::string());
                                    }
                            });
             */
            finish_callback(std::string());
        });
    }

    void CctalkDevice::requestMasterInhibitStatus(const std::function<void(const std::string& error_msg, bool inhibit)>& finish_callback) {
        std::vector<uint8_t> data;
        this->linkController->ccRequest(CcHeader::RequestMasterInhibitStatus, this->deviceAddress, data, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {
            if (!error_msg.size() == 0) {
                ESP_LOGE(TAG, "Error getting master inhibit status: %s", error_msg.c_str());
                finish_callback(error_msg, false);
                return;
            }
            if (responseData.size() != 1) {
                std::string error = "! Invalid data received for GetMasterInhibitStatus.";
                //ccResponseDataDecodeError(request_id, error); // auto-logged
                finish_callback(error, false);

                return;
            }
            // Decode the data
            bool inhibit = !bool(responseData.at(0)); // 0 means inhibit
            ESP_LOGD(TAG, "Master inhibit status: %s", !inhibit ? "accept" : "reject");
            finish_callback(std::string(), inhibit);
        });
    }

    void CctalkDevice::modifyBillOperatingMode(bool use_stacker, bool use_escrow, const std::function<void(const std::string& error_msg)>& finish_callback) {
        std::vector<uint8_t> command_arg;
        unsigned int mask = 0;
        if (use_stacker) {
            mask += 1;
        }
        if (use_escrow) {
            mask += 2;
        }
        command_arg.push_back(char(mask));

        this->linkController->ccRequest(CcHeader::ModifyBillOperatingMode, this->deviceAddress, command_arg, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {
            if (!error_msg.size() == 0) {
                ESP_LOGE(TAG, "Error setting bill validator operating mode: %s", error_msg.c_str());
                finish_callback(error_msg);
                return;
            }
            if (!responseData.size() == 0) {
                std::string error = "! Non-empty data received while waiting for ACK.";
                //ccResponseDataDecodeError(request_id, error); // auto-logged
                finish_callback(error);

                return;
            }
            ESP_LOGD(TAG, "Bill validator operating mode set to: %d", int(mask));
            finish_callback(std::string());
        });
    }

    void CctalkDevice::requestIdentifiers(const std::function<void(const std::string& error_msg, const std::map<uint8_t, CcIdentifier>& identifiers)>& finish_callback) {
        if (this->deviceCategory != CcCategory::CoinAcceptor && this->deviceCategory != CcCategory::BillValidator) {
            ESP_LOGW(TAG, "Cannot request coin / bill identifiers from device category \"%d\".", int(this->deviceCategory));
            return;
        }

        auto shared_max_positions = std::make_shared<uint8_t>(0);
        if (this->deviceCategory == CcCategory::CoinAcceptor) {
            shared_max_positions = std::make_shared<uint8_t>(6);
        } else {
            shared_max_positions = std::make_shared<uint8_t>(16);
        }

        auto shared_error = std::make_shared<std::string>();
        auto shared_identifiers = std::make_shared<std::map<uint8_t, CcIdentifier >> ();
        auto shared_country_scaling_data = std::make_shared<std::map < std::string, CcCountryScalingData >> ();

        std::string coin_bill = (this->deviceCategory == CcCategory::CoinAcceptor ? "Coin" : "Bill");



        // If this is a Bill Validator, get number of bill types currently supported    
        if (this->deviceCategory == CcCategory::BillValidator) {
            // Get variable set            
            std::vector<uint8_t> data;
            this->linkController->ccRequest(CcHeader::RequestVariableSet, this->deviceAddress, data, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {
                if (!error_msg.size() == 0) {
                    // Do not set global error, this is a local error of an optional command.
                    // *shared_error = error_msg;
                } else {
                    // Decode the data
                    if (responseData.size() < 2) {
                        ESP_LOGE(TAG, "Invalid variable set data returned for bill validator.");
                    } else {
                        uint8_t num_bill_types = responseData.at(0);
                        // uint8_t num_banks = responseData.at(1);  // unused
                        if (num_bill_types > 1) {
                            *shared_max_positions = num_bill_types;
                            ESP_LOGD(TAG, "Number of bill types currently supported: %d.", int(*shared_max_positions));
                        } else {
                            ESP_LOGE(TAG, "Could not get the number of bill types currently supported, falling back to %d.", int(*shared_max_positions));
                        }
                    }
                }
            });
        }

        std::vector<uint8_t> data;
        /// Get coin / bill IDs (and possibly country scaling data)
        for (uint8_t pos = 1; pos <= *shared_max_positions; ++pos) {

            /// Fetch coin / bill ID at position pos.
            CcHeader get_command = (this->deviceCategory == CcCategory::CoinAcceptor ? CcHeader::RequestCoinId : CcHeader::RequestBillId);
            data.clear();
            data.push_back(pos);
            this->linkController->ccRequest(get_command, this->deviceAddress, data, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {
                if (!error_msg.size() == 0) {
                    *shared_error = error_msg;

                } else {
                    // Decode the data.
                    // 6 dots mean empty by convention, but we've seen all-null too.
                    std::string decodedData = decodeResponseToString(responseData);
                    if (!decodedData.size() == 0 && decodedData != "......" && decodedData.at(0) != 0) {
                        CcIdentifier identifier(decodedData);
                        if (shared_country_scaling_data->count(identifier.country) > 0) {
                            identifier.setCountryScalingData(shared_country_scaling_data->at(identifier.country));
                        }
                        ESP_LOGD(TAG, "Adding coin identifier %s to position %d in shared_identifiers", identifier.id_string.c_str(), pos);
                        (*shared_identifiers).insert(std::make_pair(pos, identifier));
                    }
                }

                //serializer->continueSequence();
            });


            // If this is a Bill Validator, get country scaling data.
            // For coin acceptors, use a fixed, predefined country scaling data.


            if (shared_identifiers.get()->count(pos) == 0) { // empty position
                //serializer->continueSequence(true);
                //return;
            }

            std::string country = shared_identifiers.get()->at(pos).country;
            if (country.size() == 0 || shared_country_scaling_data->count(country) > 0) {
                //serializer->continueSequence(true);
                //return; // already present
            }


            // Predefined rules for Georgia. TODO Make this configurable and / or remove it from here.
            if (this->deviceCategory == CcCategory::CoinAcceptor && country == "GE") {
                CcCountryScalingData data;
                data.scaling_factor = 1;
                data.decimal_places = 2;
                (*shared_country_scaling_data).insert(std::make_pair(country, data));
                (*shared_identifiers).at(pos).country_scaling_data = data;
                ESP_LOGD(TAG, "Using predefined country scaling data for %s: scaling factor: %d, decimal places: %d.", country.c_str(), data.scaling_factor, int(data.decimal_places));
                //serializer->continueSequence(true);
                //return;
            }

            if (this->deviceCategory != CcCategory::BillValidator) {
                std::vector<uint8_t> countryVector = std::vector<uint8_t>(country.begin(), country.end());
                this->linkController->ccRequest(CcHeader::RequestCountryScalingFactor, this->deviceAddress, countryVector, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {
                    if (!error_msg.size() == 0) {
                        *shared_error = error_msg;

                    } else {
                        // Decode the data
                        if (responseData.size() != 3) {
                            ESP_LOGE(TAG, "Invalid scaling data for country %s.", country.c_str());
                        } else {
                            CcCountryScalingData data;
                            auto lsb = uint16_t(static_cast<unsigned char> (responseData.at(0)));
                            auto msb = uint16_t(static_cast<unsigned char> (responseData.at(1)));
                            data.scaling_factor = uint16_t(lsb + msb * 256);
                            data.decimal_places = responseData.at(2);
                            if (data.isValid()) {
                                (*shared_country_scaling_data).insert(std::make_pair(country, data));
                                (*shared_identifiers).at(pos).country_scaling_data = data;
                                ESP_LOGD(TAG, "Country scaling data for %s: scaling factor: %d, decimal places: %d.", country.c_str(), data.scaling_factor, int(data.decimal_places));
                            } else {
                                ESP_LOGD(TAG, "Country scaling data for %s: empty!", country.c_str());
                            }
                        }
                    }
                    //serializer->continueSequence(error_msg.size() == 0);
                });
            }
        }

        if (!shared_error->size() == 0) {
            ESP_LOGE(TAG, "Error getting %s identifiers: %s", coin_bill.c_str(), shared_error->c_str());
        } else {
            if (shared_identifiers->size() != 0) {
                // TODO: fix this
                //                std::string strs;
                //                strs.append("* %1 identifiers: " + coin_bill);
                for (auto iter = shared_identifiers->cbegin(); iter != shared_identifiers->cend(); ++iter) {
                    ESP_LOGD(TAG, "*** %s position %d: %s", coin_bill.c_str(), int(iter->first), iter->second.id_string.c_str());
                }
                //                logMessage(strs.join(std::stringLiteral("\n")));
            } else {
                ESP_LOGD(TAG, "No non-empty %s identifiers received.", coin_bill.c_str());
            }
        }

        finish_callback(*shared_error, *shared_identifiers);

    }

    void CctalkDevice::requestHopperStatus(const std::function<void(const std::string& error_msg, uint8_t eventCounter, const std::vector<CcEventData>& event_data)>& finish_callback) {
        std::vector<uint8_t> data;
        this->linkController->ccRequest(CcHeader::RequestHopperStatus, this->deviceAddress, data, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {

            // TODO Handle command timeout

            std::vector<CcEventData> event_data;

            if (!error_msg.size() == 0) {
                //ESP_LOGE(TAG, "Error getting %s buffered credit / events: %s", coin_bill.c_str(), error_msg.c_str());
                finish_callback(error_msg, 0, event_data);
                return;
            }

            if (responseData.size() == 0) {
                std::string error = std::string("Invalid (empty) hopper status data received.");
                        finish_callback(error, 0, event_data);
                return;
            }

            uint8_t eventCounter = 0; // TODO change this

            finish_callback(std::string(), eventCounter, event_data);
        });
    }

    void CctalkDevice::requestBufferedCreditEvents(const std::function<void(const std::string& error_msg, uint8_t event_counter, const std::vector<CcEventData>& event_data)>& finish_callback) {
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

        std::string coin_bill = (this->deviceCategory == CcCategory::CoinAcceptor ? "Coin" : "Bill");
        CcHeader command = (this->deviceCategory == CcCategory::CoinAcceptor ? CcHeader::ReadBufferedCreditOrErrorCodes : CcHeader::ReadBufferedBillEvents);

        std::vector<uint8_t> data;
        this->linkController->ccRequest(command, this->deviceAddress, data, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {

            // TODO Handle command timeout

            std::vector<CcEventData> event_data;

            if (!error_msg.size() == 0) {
                ESP_LOGE(TAG, "Error getting %s buffered credit / events: %s", coin_bill.c_str(), error_msg.c_str());
                        finish_callback(error_msg, 0, event_data);
                return;
            }

            if (responseData.size() == 0) {
                std::string error = std::string("Invalid (empty) " + coin_bill + " buffered credit / event data received.");
                        finish_callback(error, 0, event_data);
                return;
            }

            if (responseData.size() % 2 != 1) {
                std::string error = std::string("Invalid " + coin_bill + " buffered credit / event data size received, unexpected size: " + std::to_string(responseData.size()));
                        finish_callback(error, 0, event_data);
                return;
            }

            auto eventCounter = uint8_t(responseData[0]);

            // Log the table, but only if changed.
            if (!this->isEventLogRead || this->lastEventNumber != eventCounter) {
                //std::stringList strs;
                //strs << "* %1 buffered credit / event table (newest to oldest):", coin_bill);
                //strs << "*** Host-side last processed event number: %1", int(this->lastEventNumber));
                //strs << "*** Device-side event counter: %1", int(event_counter));
                for (int i = 1; (i + 1) < responseData.size(); i += 2) {
                    ESP_LOGD(TAG, "*** Credit: %d, error / sorter: %d", int(responseData[i]), int(responseData[i + 1]));
                }
                //logMessage(strs.join(std::stringLiteral("\n")));
                isEventLogRead = true;
            }

            for (int i = 1; (i + 1) < responseData.size(); i += 2) {

                CcEventData ev_data(responseData.at(i), responseData.at(i + 1), this->deviceCategory);
                        event_data.push_back(ev_data); // new ones first
            }

            finish_callback(std::string(), eventCounter, event_data);
        });
    }

    /**
     * Check to see if the hopper has any payout errors (e.g. stuck coin)
     * 
     * @param error_msg An error messaged returned by the cctalk command
     * @param hopperStatus 2 data bytes containing error codes from the hopper.
     */
    void CctalkDevice::processHopperStatus(const std::string & error_msg, uint8_t eventCounter, const std::vector<CcEventData>& hopperStatusData, const std::function<void()>& finish_callback) {
        // Per specification, a command timeout should be ignored.
        if (error_msg.size() == 0 && eventCounter == 0 && hopperStatusData.size() == 0) {
            finish_callback();
            return; // nothing
        }

        // If an error occurred during polling, do nothing (?)
        if (error_msg.size() != 0) {
            finish_callback();
            return;
        }

        // When the device is first booted, the event log contains all zeroes.
        if (this->lastEventNumber == 0 && eventCounter == 0) {
            // The device is operating normally (just initialized).
            finish_callback();
            return;
        }


    }

    /**
     * @brief Process any credit events from the coin acceptor or bill validator
     * 
     * The specification says:
     * If ReadBufferedCredit times out, do nothing.
     * If [event_counter] == 0 and last_event_num == 0, the device is operating normally (just initialized).
     * If [event_counter] == 0 and last_event_num != 0, the device lost power (possible loss of credits).
     * If [event_counter] == last_event_counter, no new credits.
     * If diff([event_counter], last_event_counter) > 5, one or more credits lost.
     * if diff([event_counter], last_event_counter) <= 5, new credit/event info.
     *
     * The general mode of operation:
     * If in coin event polling and an error event is received, set status to error and start error event polling instead.
     * If in error event polling and all ok event is received, switch to coin event polling.
     *
     * if master inhibit ON is detected there, switch to NormalRejecting (?).
     * if an error is detected there, switch to diagnostics state.
     * If bill is held in escrow, call validator function and accept / reject it.
     *  creditAccepted().
     * 
     * @param accepting Are we accepting coins at present? If we are, then call the callback method
     * @param event_log_cmd_error_msg
     * @param eventCounter
     * @param event_data
     * @param finish_callback
     */
    void CctalkDevice::processCreditEventLog(bool accepting, const std::string& event_log_cmd_error_msg, uint8_t eventCounter, const std::vector<CcEventData>& event_data, const std::function<void()>& finish_callback) {

        // Per specification, a command timeout should be ignored.
        if (event_log_cmd_error_msg.size() == 0 && eventCounter == 0 && event_data.size() == 0) {
            finish_callback();
            return; // nothing
        }

        // If an error occurred during polling, do nothing (?)
        if (!event_log_cmd_error_msg.size() == 0) {
            finish_callback();
            return;
        }

        // When the device is first booted, the event log contains all zeroes.

        if (this->lastEventNumber == 0 && eventCounter == 0) {
            // The device is operating normally (just initialized).
            finish_callback();
            return;
        }

        // If the event counter suddenly drops to 0, this means that the device was
        // probably reset. Probable loss of credits.
        if (this->lastEventNumber != 0 && eventCounter == 0) {
            ESP_LOGE(TAG, "The device appears to have been reset, possible loss of credit.");
            requestSwitchDeviceState(CcDeviceState::ExternalReset, [ & ]([[maybe_unused]] const std::string & local_error_msg) {
                this->lastEventNumber = 0;
                finish_callback();
            });
            return;
        }

        // If the event counters are equal, there are no new events.
        if (this->lastEventNumber == eventCounter) {
            // nothing
            finish_callback();
            return;
        }

        // If true, it means that we're processing the events that are in the device.
        // We should not give credit in this case, since that was probably processed
        // during previous application run.
        //        const bool processing_app_startup_events = (this->lastEventNumber == 0);
        //        if (processing_app_startup_events && eventCounter != 0) {
        //            ESP_LOGD(TAG, "last event number: %d, event counter: %d", this->lastEventNumber, eventCounter);
        //            ESP_LOGE(TAG, "Detected device that was up (and generating events) before the host startup; ignoring \"credit accepted\" events.");
        //        }

        int newEventCount = int(eventCounter) - int(this->lastEventNumber);
        if (newEventCount < 0) {
            newEventCount += 255;
        }
        this->lastEventNumber = eventCounter;

        if (newEventCount > event_data.size()) {
            ESP_LOGE(TAG, "Event counter difference %d is greater than buffer size %d", newEventCount, event_data.size());
        }

        // Newest to oldest
        std::vector<CcEventData> new_event_data = std::vector<CcEventData>(event_data.begin(), event_data.begin() + newEventCount);
        ESP_LOGD(TAG, "Found %d new event(s); processing from oldest to newest.", new_event_data.size());

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

                    //TODO: Causes kernel panic
                    //ESP_LOGD(TAG, "Coin status/error event %s found, rejection type: %s.",
                    //        ccCoinAcceptorEventCodeGetDisplayableName(ev.coin_error_code).c_str(),
                    //        ccCoinRejectionTypeGetDisplayableName(rejection_type).c_str());

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
                            ccBillValidatorErrorCodeGetDisplayableName(ev.bill_error_code).c_str(),
                            ccBillValidatorEventTypeGetDisplayableName(ev.bill_event_type).c_str());

                    // Schedule a single PerformSelfCheck to see if it's actually an error (and if it's persistent).
                    if (ev.bill_event_type != CcBillValidatorEventType::Status && ev.bill_event_type != CcBillValidatorEventType::Reject) {
                        self_check_requested = true;
                    }
                }

            } else {
                // Coins are accepted unconditionally
                if (getStoredDeviceCategory() == CcCategory::CoinAcceptor) {
                    ESP_LOGD(TAG, "Processing coin");
                    // Coin accepted, credit the user.
                    if (this->identifiers.find(ev.coin_id) != this->identifiers.cend()) {
                        CcIdentifier id = this->identifiers.at(ev.coin_id);
                        //                        if (processing_app_startup_events) {
                        //                            ESP_LOGD(TAG, "The following is a startup event message, ignore it:");
                        //                        }
                        ESP_LOGD(TAG, "Coin (position %d, ID %s) has been accepted to sorter path %d.", int(ev.coin_id), id.id_string.c_str(), int(ev.coin_sorter_path));
                        //if (!accepting && !processing_app_startup_events) {
                        if (!accepting) {
                            ESP_LOGE(TAG, "Coin accepted even though we're in rejecting mode; internal error!");
                        } else {
                            if (creditAcceptedCallback != nullptr) {
                                creditAcceptedCallback(ev.coin_id, id);
                            }
                        }
                    }

                    // Bills
                } else {
                    CcIdentifier id = this->identifiers.at(ev.bill_id);

                    // Bills are held in escrow and should be manually accepted into a stacker.
                    if (ev.bill_success_code == CcBillValidatorSuccessCode::ValidatedAndHeldInEscrow) {
                        // We send the RouteBill command ONLY if this is the last event in event log, otherwise
                        // there may be an inhibit or rejection or accepted any other event after it and we do not
                        // want to operate on old data and assumptions.
                        if (!processing_last_event) {
                            //                            if (processing_app_startup_events) {
                            //                                ESP_LOGD(TAG, "The following is a startup event message, ignore it:");
                            //                            }
                            ESP_LOGD(TAG, "Bill (position %d, ID %s) is or was in escrow, too late to process an old event; ignoring.", int(ev.bill_id), id.id_string.c_str());
                            continue;
                        }
                        if (!accepting) {
                            //                            if (processing_app_startup_events) {
                            //                                ESP_LOGD(TAG, "The following is a startup event message, ignore it:");
                            //                            }
                            ESP_LOGD(TAG, "Bill (position %d, ID %s) is or was in escrow, even though we're in rejecting mode; ignoring.", int(ev.bill_id), id.id_string.c_str());
                            bill_routing_force_reject = true;
                        }

                        //  ESP_LOGD(TAG, "Bill (position %1, ID %2) is in escrow, deciding whether to accept.",int(ev.bill_id),id.id_string));
                        bill_routing_pending = true;
                        routing_req_event = ev;
                        // Continue below

                    } else if (ev.bill_success_code == CcBillValidatorSuccessCode::ValidatedAndAccepted) {
                        // This success code appears in event log after a bill routing request, or no routing command timeout.
                        // Credit the user.
                        //                        if (processing_app_startup_events) {
                        //                            ESP_LOGD(TAG, "The following is a startup event message, ignore it:");
                        //                        }
                        ESP_LOGD(TAG, "Bill (position %d, ID %s) has been accepted.", int(ev.bill_id), id.id_string.c_str());
                        //if (!accepting && !processing_app_startup_events) {
                        if (!accepting) {
                            ESP_LOGE(TAG, " Bill accepted even though we're in rejecting mode; internal error!");
                        } else {
                            if (creditAcceptedCallback != nullptr) {
                                creditAcceptedCallback(ev.bill_id, id);
                            }
                        }

                    } else {
                        ESP_LOGE(TAG, "Invalid ev.bill_success_code %s", ccBillValidatorSuccessCodeGetDisplayableName(ev.bill_success_code).c_str());
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

        auto shared_self_check_fault_code = std::make_shared<CcFaultCode>(CcFaultCode::Ok);

        // Check if the error is a problem enough to warrant a diagnostics polling mode
        if (self_check_requested) {

            ESP_LOGD(TAG, "At least one new event has an error code, requesting SelfCheck to see if there is a global fault code.");

            *shared_self_check_fault_code = CcFaultCode::CustomCommandError;

            requestSelfCheck([ & ]([[maybe_unused]] const std::string& error_msg, CcFaultCode fault_code) {
                *shared_self_check_fault_code = fault_code;
            });
            finish_callback();
        }

        // Decide what to do with the bill currently in escrow
        if (bill_routing_pending) {

            bool accept = false;
            CcIdentifier id = this->identifiers.at(routing_req_event.bill_id);

            if (*shared_self_check_fault_code != CcFaultCode::Ok) {
                ESP_LOGD(TAG, "SelfCheck returned a non-OK fault code; pending bill in escrow will be rejected.");

            } else if (bill_routing_force_reject) {
                ESP_LOGE(TAG, " Forcing bill validation rejection due to being in NormalRejecting state; internal error.");

            } else {
                //      DBG_ASSERT(bill_validator_func_);
                if (this->billValidatorFunction) {
                    accept = this->billValidatorFunction(routing_req_event.bill_id, id);
                }
                ESP_LOGD(TAG, "Bill validating function status: %s.", accept ? "accept" : "reject");
            }

            CcBillRouteCommandType route_command = accept ? CcBillRouteCommandType::RouteToStacker : CcBillRouteCommandType::ReturnBill;
            ESP_LOGD(TAG, "Bill (position %d, ID %s) is in escrow, sending a request for: %s.", int(routing_req_event.bill_id), id.id_string.c_str(), ccBillRouteCommandTypeGetDisplayableName(route_command).c_str());

            requestRouteBill(route_command, [ & ]([[maybe_unused]] const std::string& error_msg, CcBillRouteStatus status) {
                ESP_LOGD(TAG, "Bill (position %d, ID %s) routing status: %s.", int(routing_req_event.bill_id), id.id_string.c_str(), ccBillRouteStatusGetDisplayableName(status).c_str());

                //        aser->continueSequence(true);
            });

        }

        // If the fault code was not Ok, switch to diagnostics mode.
        if (self_check_requested) {

            if (*shared_self_check_fault_code == CcFaultCode::Ok) {
                //aser->continueSequence(true);
            } else {

                ESP_LOGD(TAG, "SelfCheck returned a non-OK fault code, switching to diagnostics polling mode.");

                requestSwitchDeviceState(CcDeviceState::DiagnosticsPolling, [ & ]([[maybe_unused]] const std::string & local_error_msg) {
                    //aser->continueSequence(true);
                });
            }
        }

    }

    void CctalkDevice::requestRouteBill(CcBillRouteCommandType route, const std::function<void(const std::string& error_msg, CcBillRouteStatus status)>& finish_callback) {
        std::vector<uint8_t> command_arg;
        command_arg.push_back(char(route));
        this->linkController->ccRequest(CcHeader::RouteBill, this->deviceAddress, command_arg, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {

            if (!error_msg.size() == 0) {
                ESP_LOGE(TAG, "Error sending RouteBill command: %s", error_msg.c_str());
                finish_callback(error_msg, CcBillRouteStatus::FailedToRoute);
                return;
            }

            if (responseData.size() > 1) {
                std::string error = "Invalid data received for RouteBill.";
                //ccResponseDataDecodeError(request_id, error); // auto-logged
                finish_callback(error, CcBillRouteStatus::FailedToRoute);
                return;
            }

            // Decode the data.
            // ACK means Routed
            CcBillRouteStatus status = CcBillRouteStatus::Routed;
            if (responseData.size() == 1) {

                status = static_cast<CcBillRouteStatus> (responseData.at(0));
            }

            ESP_LOGD(TAG, "RouteBill command status: %s", ccBillRouteStatusGetDisplayableName(status).c_str());

            finish_callback(std::string(), status);
        });

    }

    void CctalkDevice::requestSelfCheck(const std::function<void(const std::string& error_msg, CcFaultCode fault_code)>& finish_callback) {
        std::vector<uint8_t> data;
        this->linkController->ccRequest(CcHeader::PerformSelfCheck, this->deviceAddress, data, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {
            if (error_msg.size() != 0) {
                ESP_LOGE(TAG, "Error getting self-check status: %s", error_msg.c_str());
                finish_callback(error_msg, CcFaultCode::CustomCommandError);
                return;
            }
            if (responseData.size() != 1) {
                std::string error = "! Invalid data received for PerformSelfCheck.";
                //ccResponseDataDecodeError(request_id, error); // auto-logged
                finish_callback(error, CcFaultCode::CustomCommandError);

                return;
            }
//            if (responseData.size() > 0) {
//                // Decode the data
//                auto fault_code = static_cast<CcFaultCode> (responseData.at(0));
//                ESP_LOGD(TAG, "Self-check fault code: %s", ccFaultCodeGetDisplayableName(fault_code).c_str());
//                finish_callback(ccFaultCodeGetDisplayableName(fault_code), fault_code);
//            } else {
                finish_callback(ccFaultCodeGetDisplayableName(CcFaultCode::Ok), CcFaultCode::Ok);
//            }
        });
    }

    void CctalkDevice::requestResetDevice(const std::function<void(const std::string& error_msg)>& finish_callback) {
        // Send the request
        std::vector<uint8_t> data;
        this->linkController->ccRequest(CcHeader::ResetDevice, this->deviceAddress, data, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {
            if (!error_msg.size() == 0) {
                ESP_LOGE(TAG, "Error sending soft reset request: %s", error_msg.c_str());
                finish_callback(error_msg);
                return;
            }
            if (responseData.size() != 0) {
                std::string error = "Non-empty data received while waiting for ACK.";
                //ccResponseDataDecodeError(request_id, error); // auto-logged
                finish_callback(error);

                return;
            }
            ESP_LOGD(TAG, "Soft reset acknowledged, waiting for the device to get back up.");
        });
    }

    void CctalkDevice::requestResetDeviceWithState(const std::function<void(const std::string& error_msg)>& finish_callback) {
        requestResetDevice([ & ](const std::string & error_msg) {
            if (error_msg.size() == 0) {
                requestSwitchDeviceState(CcDeviceState::UninitializedDown, finish_callback);
            } else {

                finish_callback(error_msg);
            }
        });
    }

    void CctalkDevice::modifySorterPath(const uint8_t coin_id, const uint8_t path, const std::function<void(const std::string& error_msg)>& finish_callback) {
        std::vector<uint8_t> data;
        data.push_back(coin_id);
        data.push_back(path);
        this->linkController->ccRequest(CcHeader::ModifySorterPaths, this->deviceAddress, data, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {
            if (!error_msg.size() == 0) {
                ESP_LOGE(TAG, "Error modifying sorter path %d: %s", int(path), error_msg.c_str());
                finish_callback(error_msg);
                return;
            }
            if (responseData.size() != 0) {
                std::string error = "Non-empty data received while waiting for ACK.";
                finish_callback(error);

                return;
            }
            ESP_LOGD(TAG, "Sorter path for coin_id %d set to %d", int(coin_id), int(path));
        });
    }

    void CctalkDevice::modifyDefaultSorterPath(const uint8_t path, const std::function<void(const std::string& error_msg)>& finish_callback) {

    }

    void CctalkDevice::enableHopper(const std::function<void(const std::string& error_msg)>& finish_callback) {
        std::vector<uint8_t> data;
        data.push_back(165); // always send this byte
        this->linkController->ccRequest(CcHeader::EnableHopper, this->deviceAddress, data, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {
            if (!error_msg.size() == 0) {
                ESP_LOGE(TAG, "Error enabling hopper: %s", error_msg.c_str());
                finish_callback(error_msg);
                return;
            }
            if (responseData.size() != 0) {
                std::string error = "Non-empty data received while waiting for ACK.";
                finish_callback(error);

                return;
            }
        });
    }

    void CctalkDevice::requestCipherKey(const std::function<void(const std::string & error_msg, const std::vector<uint8_t>& cipherKey)>& finish_callback) {
        std::vector<uint8_t> data;
        this->linkController->ccRequest(CcHeader::RequestCipherKey, this->deviceAddress, data, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {
            if (!error_msg.size() == 0) {
                ESP_LOGE(TAG, "Error requesting cipher key: %s", error_msg.c_str());
                finish_callback(error_msg, responseData);
                return;
            }

            if (responseData.size() != 8) {
                std::string errormsg = "Error requesting cipher key: No data returned";
                ESP_LOGE(TAG, "%s", errormsg.c_str());
                finish_callback(errormsg, responseData);
            } else {
                finish_callback(std::string(), responseData);
            }

        });
    }

    void CctalkDevice::requestPayoutHighLowStatus(const std::function<void(const std::string & error_msg, const std::vector<uint8_t>& highLowStatus)>& finish_callback) {
        std::vector<uint8_t> data;
        this->linkController->ccRequest(CcHeader::RequestPayoutHighLowStatus, this->deviceAddress, data, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {
            if (!error_msg.size() == 0) {
                ESP_LOGE(TAG, "Error requesting payout high/low status: %s", error_msg.c_str());
                finish_callback(error_msg, responseData);
                return;
            }
            finish_callback(std::string(), responseData);
            return;
        });
    }

    void CctalkDevice::testHopper(const std::function<void(const std::string & error_msg, const std::vector<uint8_t>& hopperStatus)>& finish_callback) {
        std::vector<uint8_t> data;
        this->linkController->ccRequest(CcHeader::TestHopper, this->deviceAddress, data, 200, [ & ](const std::string& error_msg, const std::vector<uint8_t> & responseData) {
            if (!error_msg.size() == 0) {
                ESP_LOGE(TAG, "Error testing hopper: %s", error_msg.c_str());
                finish_callback(error_msg, responseData);
                return;
            }

            finish_callback(std::string(), responseData);

            //            if (responseData.size() != 0) {
            //                std::string error = "Non-empty data received while waiting for ACK.";
            //                finish_callback(error);
            //                return;
            //            }
        });
    }

    void CctalkDevice::dispenseCoins(const int numberOfCoins, const std::function<void(const std::string & error_msg)>& finish_callback) {

        ESP_LOGD(TAG, "Dispense coins called: Dispensing %d coins: ", numberOfCoins);

        std::vector<uint8_t> data;
        bool doContinue = true;

        if (this->deviceCategory != CcCategory::Payout) {
            ESP_LOGE(TAG, "Incorrect device category for dispenseCoins command: %s", ccCategoryDisplayNameFromCategory(this->deviceCategory).c_str());
            finish_callback("Incorrect device category for dispenseCoins command: ");
            return;
        }

        // Request Cipher Key
        ESP_LOGD(TAG, "Requesting cipher key");
        requestCipherKey([&](const std::string& error_msg, const std::vector<uint8_t>& cipherKey) {
            if (error_msg.size() > 0) {
                ESP_LOGE(TAG, "Request cipher key command failed: %s", error_msg.c_str());
                doContinue = false;
                finish_callback("Request cipher key command failed.");
                return;
            }

            if (cipherKey.size() != 8) {
                ESP_LOGE(TAG, "Expecting 8 cipher bytes, received %d", cipherKey.size());
                doContinue = false;
            } else {
                data.push_back(cipherKey.at(0));
                data.push_back(cipherKey.at(1));
                data.push_back(cipherKey.at(2));
                data.push_back(cipherKey.at(3));
                data.push_back(cipherKey.at(4));
                data.push_back(cipherKey.at(5));
                data.push_back(cipherKey.at(6));
                data.push_back(cipherKey.at(7));
            }

        });

        if (!doContinue) {
            finish_callback("Request cipher key command failed.");
            return;
        }

        // Dispense Hopper Coins        
        data.push_back(numberOfCoins);

        ESP_LOGD(TAG, "Cipher Key: %03d %03d %03d %03d %03d %03d %03d %03d", data.at(0), data.at(1), data.at(2), data.at(3), data.at(4), data.at(5), data.at(6), data.at(7));
        ESP_LOGD(TAG, "Dispensing %02d coin(s).", numberOfCoins);

        this->linkController->ccRequest(CcHeader::DispenseHopperCoins, this->deviceAddress, data, 200, [&](const std::string& error_msg, const std::vector<uint8_t> & responseData) {
            finish_callback(std::string());
        });

    }

    CcDeviceState CctalkDevice::getDeviceState() const {

        return this->deviceState;
    }

    void CctalkDevice::setDeviceState(CcDeviceState state) {
        if (this->deviceState != state) {
            this->deviceState = state;
            ESP_LOGD(TAG, "Device state changed to: %s", ccDeviceStateGetDisplayableName(state).c_str());
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

    std::string CctalkDevice::decodeResponseToString(const std::vector<uint8_t>& responseData) {
        std::string responseString;
        for (uint8_t byte : responseData) {

            responseString.push_back(byte);
        }
        return responseString;
    }

    std::string CctalkDevice::decodeResponseToHex(const std::vector<uint8_t>& responseData) {
        std::string formatted_data;
        for (uint8_t tmpData : responseData) {

            std::stringstream stream;
            stream << "0x" << std::hex << tmpData;
            formatted_data.append(stream.str());
        }
        return formatted_data;
    }

    void CctalkDevice::setCreditAcceptedCallback(CreditAcceptedFunc callback) {
        this->creditAcceptedCallback = callback;
    }
}
