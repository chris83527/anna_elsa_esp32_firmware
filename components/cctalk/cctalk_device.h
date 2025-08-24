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

/*
 * File:   cctalk_device.h
 * Author: chris
 *
 * Created on February 3, 2023, 4:31 PM
 */

#ifndef CCTALK_DEVICE_H
#define CCTALK_DEVICE_H

#include <functional>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "cctalk_enums.h"
#include "cctalk_link_controller.h"

namespace esp32cc {

/// Device state.
/// Additional commands that may change the state:
/// - ResetDevice

enum class CcDeviceState {
  /// Initial state, device not probed yet, or shut down.
  /// Switching to this mode stops the timer / polling.
  ShutDown,

  /// The state is set automatically when the device fails to respond
  /// when entering Initialized stage, or after a soft reset.
  /// The device should be checked for Alive continuously and if alive,
  /// initialized.
  UninitializedDown,

  /// Switching from ShutDown, ExternalReset, UninitializedDown this is:
  /// - probe the device using SimplePoll
  /// - read device manufacturing info (category, serial, manufacturer, ...)
  /// - read device recommended polling frequency
  /// - initialize coin / bill IDs (including country scaling)
  /// - enable stacker and escrow for bill validators
  /// - set inhibit status off on all bills (but not coins!).
  /// If the device doesn't respond to SimplePoll, UninitializedDown state is
  /// entered.
  /// If any of the other initialization errors occur, InitializationFailed
  /// state is entered.
  /// Switching to this mode starts the timer / polling.
  Initialized,

  /// This can be set only during switching to Initialized.
  /// The device didn't respond to probing or other initialization requests.
  /// Device cannot be used.
  InitializationFailed,

  /// This sets master inhibit status off.
  /// The event table is polled continuously.
  NormalAccepting,

  /// This sets master inhibit status on.
  /// The event table is polled continuously.
  /// Note that if the device sets master inhibit to on due to polling timeout,
  /// this state is set (only for bill validators, since coin acceptors
  /// automatically
  /// disable master inhibit if the polling resumes after a timeout).
  NormalRejecting,

  /// The device is polling the diagnostics log. This state may be caused by
  /// an error condition in the event table. Once the error is resolved, the
  /// device switches to NormalRejecting state.
  DiagnosticsPolling,

  /// This state is set automatically if the device is found to be down during
  /// normal operation.
  /// Do NOT reset the device, since the event log (with the credits) will be
  /// lost.
  /// The device should be assumed to be just like in ShutDown state.
  UnexpectedDown,

  /// This state is set automatically if an external reset of the device is
  /// detected.
  /// The device should be assumed to be just like in ShutDown state.
  ExternalReset,
};

/// Get displayable name

inline std::string ccDeviceStateGetDisplayableName(CcDeviceState status) {
  static std::map<CcDeviceState, std::string> name_map = {
      {CcDeviceState::ShutDown, "ShutDown"},
      {CcDeviceState::UninitializedDown, "UninitializedDown"},
      {CcDeviceState::Initialized, "Initialized"},
      {CcDeviceState::InitializationFailed, "InitializationFailed"},
      {CcDeviceState::NormalAccepting, "NormalAccepting"},
      {CcDeviceState::NormalRejecting, "NormalRejecting"},
      {CcDeviceState::DiagnosticsPolling, "DiagnosticsPolling"},
      {CcDeviceState::UnexpectedDown, "UnexpectedDown"},
      {CcDeviceState::ExternalReset, "ExternalReset"},
  };

  try {
    return name_map.at(status);
  } catch (const std::exception &e) {
    return e.what();
  }
}

/* ccTalk device. This class contains high-level functions to manipulate
 * ccTalk devices. The actual messaging protocol is implemented in the
 * controller-thread-managing CctalkLinkController class.
 */
class CctalkDevice {
public:
  using BillValidatorFunc =
      std::function<bool(uint8_t bill_id, const CcIdentifier &identifier)>;
  using CreditAcceptedFunc =
      std::function<void(uint8_t coin_id, const CcIdentifier &identifier)>;
  using ResponseErrorFunc = std::function<void(const std::string &error_msg)>;

  /// Constructor
  CctalkDevice(CctalkLinkController &linkController,
               uint8_t deviceAddress);
  ~CctalkDevice();

  /// Start event-handling timer
  void startPolling();

  /// Stop event-handling timer
  void stopPolling();

  /// Request switching the device state.
  /// \return true if the request was successfully sent.
  bool requestSwitchDeviceState(
      CcDeviceState state,
      std::function<void(const std::string &error_msg)> const &finish_callback);

  /// Switch to Initialized state from ShutDown state.
  /// If the switch request fails, the devices is switched to
  /// InitializationFailed state. This sets the stored member variables. \return
  /// true if preconditions were acceptable and the switch has been initiated.
  bool switchStateInitialized(
      std::function<void(const std::string &error_msg)> const &finish_callback);

  /// Switch to NormalAccepting state.
  bool switchStateNormalAccepting(
      std::function<void(const std::string &error_msg)> const &finish_callback);

  /// Switch to NormalRejecting state.
  bool switchStateNormalRejecting(
      std::function<void(const std::string &error_msg)> const &finish_callback);

  /// Switch to DiagnosticsPolling state.
  bool switchStateDiagnosticsPolling(
      std::function<void(const std::string &error_msg)> const &finish_callback);

  /// Switch to ShutDown state.
  bool switchStateShutDown(
      std::function<void(const std::string &error_msg)> const &finish_callback);

  /// Send SimplePoll and return for ACK.
  void requestCheckAlive(
      std::function<void(const std::string &error_msg, bool alive)> const
          &finish_callback) const;

  /// Request manufacturing information info from the device.
  /// This includes category, serial number, manufacturer, ...
  void requestManufacturingInfo(
      std::function<void(const std::string &error_msg, CcCategory &category,
                         const std::string &info)> const &finish_callback) const;

  /// Get device-recommended polling interval in ms.
  void requestPollingInterval(
      std::function<void(const std::string &error_msg, uint64_t msec)> const
          &finish_callback) const;

  /// Request inhibit status modification. This is needed to enable coin/bill
  /// acceptance.
  void modifyInhibitStatus(
      uint8_t accept_mask1, uint8_t accept_mask2,
      std::function<void(const std::string &error_msg)> const &finish_callback) const;

  /// Request master inhibit status modification. This is needed to enable
  /// coin/bill acceptance.
  void modifyMasterInhibitStatus(
      bool inhibit,
      std::function<void(const std::string &error_msg)> const &finish_callback) const;

  /// Request master inhibit status retrieval,
  void requestMasterInhibitStatus(
      std::function<void(const std::string &error_msg, bool inhibit)> const
          &finish_callback) const;

  /// Request bill validator operating mode modification.
  void modifyBillOperatingMode(
      bool use_stacker, bool use_escrow,
      std::function<void(const std::string &error_msg)> const &finish_callback) const;

  void modifySorterPath(
      uint8_t coin_id, uint8_t path,
      std::function<void(const std::string &error_msg)> const &finish_callback) const;

  void modifyDefaultSorterPath(
      uint8_t path,
      std::function<void(const std::string &error_msg)> const &finish_callback) const;

  void modifySorterOverrideStatus(
      uint8_t overrideStatus,
      std::function<void(const std::string &error_msg)> const &finish_callback) const;

  void enableHopper(
      const std::function<void(const std::string &error_msg)>& finish_callback) const;

  void requestCipherKey(
      std::function<void(const std::string &error_msg,
                         const std::vector<uint8_t> &cipherKey)> const
          &finish_callback) const;

  void requestPayoutHighLowStatus(
      std::function<void(const std::string &error_msg,
                         const std::vector<uint8_t> &highLowStatus)> const
          &finish_callback) const;

  void testHopper(const std::function<void(const std::string &error_msg,
                                     const std::vector<uint8_t> &hopperStatus)>
                      & finish_callback) const;

  void dispenseCoins(
      int numberOfCoins,
      std::function<void(const std::string &error_msg)> const &finish_callback) const;

  void purgeHopper(
      std::function<void(const std::string &error_msg,
                         const std::vector<uint8_t> &hopperStatus)> const
          &finish_callback);

  /// Request coin / bill identifiers (quantity for bills, bill/coin names) and
  /// country scaling data (bills).
  void requestIdentifiers(
      std::function<void(const std::string &error_msg,
                         const std::map<uint8_t, CcIdentifier>
                             &identifiers)> const &finish_callback);

  /// Request buffered credit (coins / bills) events or error events using
  /// ReadBufferedBillEvents or ReadBufferedCredit commands. This function
  /// should be executed repeatedly when polling. \c event_data contains the
  /// event log from newest (at offset 0) to oldest (at offset 4). A command
  /// timeout here is not an error condition - an empty event counter / log and
  // an empty error message are returned. The caller should ignore this and
  // continue normally.
  void requestBufferedCreditEvents(
      std::function<void(const std::string &error_msg, uint8_t event_counter,
                         const std::vector<CcEventData> event_data)> const
          &finish_callback);
  void processHopperStatus(const std::string& error_msg, uint8_t eventCounter,
                           const std::vector<CcEventData>& hopperStatusData,
                           std::function<void()> const& finish_callback) const;

  /**
   * The data field consist in four bytes , an event counter similar to the
   * event counter in “read buffer bill events” from bill acceptors that is zero
   * right after power up and increment with each event up to 255 and back to 1,
   * one byte payout coins remaining , one byte last payout – coins paid and one
   * byte last payout – coins unpaid. As you see the hopper is right after a
   * power up , the last payout was 20 coins
   *
   * @param finish_callback The callback function to execute returning the
   * values
   */
  void requestHopperStatus(
      std::function<void(const std::string &error_msg, uint8_t event_counter,
                         const std::vector<CcEventData> event_data)> const
          &finish_callback) const;

  /// Process the credit/event log. This is used by timerIteration().
  void processCreditEventLog(bool accepting,
                             const std::string &event_log_cmd_error_msg,
                             uint8_t event_counter,
                             const std::vector<CcEventData>& event_data,
                             std::function<void()> const &finish_callback);

  /// Route a bill that is held in escrow.
  void requestRouteBill(
      CcBillRouteCommandType route,
      std::function<void(const std::string &error_msg,
                         CcBillRouteStatus status)> const &finish_callback) const;

  /// Request self-check (diagnostics mode). This function should be executed
  /// repeatedly when polling in diagnostics mode.
  void requestSelfCheck(
      std::function<void(const std::string &error_msg,
                         CcFaultCode fault_code)> const &finish_callback) const;

  /// Request soft reset. Finish callback is called when the device accepts the
  /// reset command.
  void requestResetDevice(
      std::function<void(const std::string &error_msg)> const &finish_callback) const;

  /// Call requestResetDevice() and set the state to UninitializedDown.
  void requestResetDeviceWithState(
      std::function<void(const std::string &error_msg)> const &finish_callback);

  /// Set the device status. Emits deviceStateChanged() if changed.
  void setDeviceState(CcDeviceState state);

  /// Get link controller
  CctalkLinkController &getLinkController() const;

  /// This function is called in NormalAccepting state when a bill is inserted
  /// and should be checked for validity by us. If the function returns true,
  /// the bill is accepted.
  void setBillValidationFunction(BillValidatorFunc validator);

  /// Request initialising the device from ShutDown state.
  /// Starts event timer.
  /// \return true if the request was successfully sent.
  bool initialise(
      std::function<void(const std::string &error_msg)> const &finish_callback);

  void setCreditAcceptedCallback(CreditAcceptedFunc const &callback);

  /// Request the device to be switched to ShutDown state.
  /// Stops event timer.
  /// \return true if the request was successfully sent.
  bool shutdown(
      std::function<void(const std::string &error_msg)> const &finish_callback);

  /// Get device status as set by the latest status-updating function
  CcDeviceState getDeviceState() const;

  /// Get requestManufacturingInfo() category result.
  CcCategory getStoredDeviceCategory() const;

  /// Get requestManufacturingInfo() free-form string result.
  [[nodiscard]] std::string getStoredManufacturingInfo() const;

  /// Get detectPollingInterval() result
  [[nodiscard]] int getStoredPollingInterval() const;

  /// Get requestIdentifiers() result
  [[nodiscard]] std::map<uint8_t, CcIdentifier> getStoredIndentifiers() const;

private:
  /// Emitted whenever a credit is accepted.
  CreditAcceptedFunc creditAcceptedCallback;

  /// Emitted whenever cctalk message data cannot be decoded (logic error)
  ResponseErrorFunc ccResponseDataDecodeError;

  /// Poll task
  static std::string decodeResponseToString(const std::vector<uint8_t> &responseData);
  static std::string decodeResponseToHex(const std::vector<uint8_t> &responseData);
  static std::string decodeSerialNumber(const std::vector<uint8_t> &responseData);

  void devicePollTask();

  CctalkLinkController
      &linkController; ///< Controller for serial worker thread
                       ///< with cctalk link management support.

  int normalPollingIntervalMsec =
      200; ///< Polling interval for normal and diagnostics modes.
  const int defaultNormalPollingIntervalMsec =
      200; ///< Default polling interval for normal and diagnostics modes.
  const int notAlivePollingIntervalMsec =
      1000; ///< Polling interval for modes when the device doesn't respond to
            ///< alive check.

  bool isTimerIterationTaskRunning =
      false; ///< Avoids parallel executions of state change, since it's
             ///< asynchronous

  CcDeviceState deviceState = CcDeviceState::ShutDown; ///< Current status

  BillValidatorFunc
      billValidatorFunction; ///< Bill validator function, which tells us to
                             ///< accept or reject a certain bill.

  CcCategory deviceCategory = CcCategory::Unknown; ///< Equipment category
  std::string manufacturingInfo; ///< Free-form text product information
                                 ///< (manufacturer, serial number, etc...)

  std::map<uint8_t, CcIdentifier>
      identifiers; ///< Coin positions / bill types and IDs (names)
  std::map<std::string, CcCountryScalingData> countryScalingData;

  bool isEventLogRead =
      false; ///< True if the event log was read at least once.
  volatile uint8_t lastEventNumber =
      0; ///< Last event number returned by ReadBufferedCredit command.

  std::thread pollThread;
  int pollingInterval = defaultNormalPollingIntervalMsec;
  volatile bool isPolling = false;

  uint8_t deviceAddress;
};

} // namespace esp32cc

#endif /* CCTALK_DEVICE_H */
