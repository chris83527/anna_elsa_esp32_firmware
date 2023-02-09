/*
 * The MIT License
 *
 * Copyright 2023 chris. Based on work by ashaduri (https://github.com/ashaduri)
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
 * File:   cctalk_enums.h
 * Author: chris
 *
 * Created on February 2, 2023, 6:21 PM
 */

#ifndef CCTALK_ENUMS_H
#define CCTALK_ENUMS_H

#include <string>
#include <utility>
#include <map>
#include <vector>
#include <algorithm>

namespace esp32cc {

    /*
     *  Equipment category
     */
    enum class CcCategory {
        Unknown,
        CoinAcceptor,
        Payout,
        Reel,
        BillValidator,
        CardReader,
        Changer,
        Display,
        Keypad,
        Dongle,
        Meter,
        Bootloader,
        Power,
        Printer,
        Rng,
        HopperScale,
        CoinFeeder,
        BillRecycler,
        Escrow,
        Debug,
    };

    enum class CcHeader : uint8_t {
        Ack = 0,
        ResetDevice = 1,
        RequestCommsStatusVariables = 2,
        ClearCommsStatusVariables = 3,
        RequestCommsRevision = 4,
        Busy = 6,
        RequestServiceStatus = 104,
        DataStream = 105,
        RequestEscrowStatus = 106,
        OperateEscrow = 107,
        RequestEncryptedMonetaryId = 108,
        RequestEncryptedHopperStatus = 109,
        SwitchEncryptionKey = 110,
        RequestEncryptionSupport = 111,
        ReadEncryptedEvents = 112,
        SwitchBaudRate = 113,
        RequestUsbId = 114,
        RequestRealTimeClock = 115,
        ModifyRealTimeClock = 116,
        RequestCashboxValue = 117,
        ModifyCashboxValue = 118,
        RequestHopperBalance = 119,
        ModifyHopperBalance = 120,
        PurgeHopper = 121,
        RequestErrorStatus = 122,
        RequestActivityRegister = 123,
        VerifyMoneyOut = 124,
        PayMoneyOut = 125,
        ClearMoneyCounter = 126,
        RequestMoneyOut = 127,
        RequestMoneyIn = 128,
        ReadBarcodeData = 129,
        RequestIndexedHopperDispenseCount = 130,
        RequestHopperCoinValue = 131,
        EmergencyStopValue = 132,
        RequestHopperPollingValue = 133,
        DispenseHopperValue = 134,
        SetAcceptLimit = 135,
        StoreEncryptionCode = 136,
        SwitchEncryptionCode = 137,
        FinishFirmwareUpgrade = 138,
        BeginFirmwareUpgrade = 139,
        UploadFirmware = 140,
        RequestFirmwareUpgradeCapability = 141,
        FinishBillTableUpgrade = 142,
        BeginBillTableUpgrade = 143,
        UploadBillTables = 144,
        RequestCurrencyRevision = 145,
        OperateBidirectionalMotors = 146,
        PerformStackerCycle = 147,
        ReadOptoVoltages = 148,
        RequestIndividualErrorCounter = 149,
        RequestIndividualAcceptCounter = 150,
        TestLamps = 151,
        RequestBillOperatingMode = 152,
        ModifyBillOperatingMode = 153,
        RouteBill = 154,
        RequestBillPosition = 155,
        RequestCountryScalingFactor = 156,
        RequestBillId = 157,
        ModifyBillId = 158,
        ReadBufferedBillEvents = 159,
        RequestCipherKey = 160,
        PumpRng = 161,
        ModifiyInhibitAndOverrideRegisters = 162,
        TestHopper = 163,
        EnableHopper = 164,
        ModifyVariableSet = 165,
        RequestHopperStatus = 166,
        DispenseHopperCoins = 167,
        RequestHopperDispenseCount = 168,
        RequestAddressMode = 169,
        RequestBaseYear = 170,
        RequestHopperCoin = 171,
        EmergencyStop = 172,
        RequestThermistorReading = 173,
        RequestPayoutFloat = 174,
        ModifyPayoutFloat = 175,
        RequestAlarmCounter = 176,
        HandheldFunction = 177,
        RequestBankSelect = 178,
        ModifyBankSelect = 179,
        RequestSecuritySetting = 180,
        ModifySecuritySetting = 181,
        DownloadCalibrationInfo = 182,
        UploadWindowData = 183,
        RequestCoinId = 184,
        ModifyCoinId = 185,
        RequestPayoutCapacity = 186,
        ModifyPayoutCapacity = 187,
        RequestDefaultSorterPath = 188,
        ModifyDefaultSorterPath = 189,
        KeypadControl = 191,
        RequestBuildCode = 192,
        RequestFraudCounter = 193,
        RequestRejectCounter = 194,
        RequestLastModificationDate = 195,
        RequestCreationDate = 196,
        CalculateRomChecksum = 197,
        CountersToEeprom = 198,
        ConfigurationToEeprom = 199,
        AcmiUnencryptedProductId = 200,
        RequestTeachStatus = 201,
        TeachModeControl = 202,
        DisplayControl = 203,
        MeterControl = 204,
        RequestPayoutAbsoluteCount = 207,
        ModifyPayoutAbsoluteCount = 208,
        RequestSorterPaths = 209,
        ModifySorterPaths = 210,
        PowerManagementControl = 211,
        RequestCoinPosition = 212,
        RequestOptionFlags = 213,
        WriteDataBlock = 214,
        ReadDataBlock = 215,
        RequestDataStorageAvailability = 216,
        RequestPayoutHighLowStatus = 217,
        EnterPinNumber = 218,
        EnterNewPinNumber = 219,
        AcmiEncryptedData = 220,
        RequestSorterOverrideStatus = 221,
        ModifySorterOverrideStatus = 222,
        ModifyEncryptedInhibitAndOverrideRegisters = 223,
        RequestEncryptedProductId = 224,
        RequestAcceptCounter = 225,
        RequestInsertionCounter = 226,
        RequestMasterInhibitStatus = 227,
        ModifyMasterInhibitStatus = 228,
        ReadBufferedCreditOrErrorCodes = 229,
        RequestInhibitStatus = 230,
        ModifyInhibitStatus = 231,
        PerformSelfCheck = 232,
        LatchOutputLines = 233,
        SendDhPublicKey = 234,
        ReadDhPublicKey = 235,
        ReadOptoStates = 236,
        ReadInputLines = 237,
        TestOutputLines = 238,
        OperateMotors = 239,
        TestSolenoids = 240,
        RequestSoftwareRevision = 241,
        RequestSerialNumber = 242,
        RequestDatabaseVersion = 243,
        RequestProductCode = 244,
        RequestEquipmentCategoryId = 245,
        RequestManufacturerId = 246,
        RequestVariableSet = 247,
        RequestStatus = 248,
        RequestPollingPriority = 249,
        AddressRandom = 250,
        AddressChange = 251,
        AddressClash = 252,
        AddressPoll = 253,
        SimplePoll = 254,
        FactorySetupAndTest = 255
    };

    /// Get displayable name

    inline std::string ccHeaderGetDisplayableName(CcHeader header) {
        static std::map<CcHeader, std::string> name_map = {
            {CcHeader::Ack, "Reply"},
            {CcHeader::ResetDevice, "ResetDevice"},
            {CcHeader::Busy, "Busy"},

            {CcHeader::SwitchBaudRate, "SwitchBaudRate"},

            {CcHeader::RequestFraudCounter, "RequestFraudCounter"},
            {CcHeader::RequestRejectCounter, "RequestRejectCounter"},
            {CcHeader::RequestAcceptCounter, "RequestAcceptCounter"},
            {CcHeader::RequestInsertionCounter, "RequestInsertionCounter"},

            {CcHeader::ReadBufferedBillEvents, "ReadBufferedBillEvents"},
            {CcHeader::RouteBill, "RouteBill"},
            {CcHeader::ReadBufferedCreditOrErrorCodes, "ReadBufferedCredit"},

            {CcHeader::PerformSelfCheck, "PerformSelfCheck"},

            {CcHeader::RequestInhibitStatus, "RequestInhibitStatus"},
            {CcHeader::ModifyInhibitStatus, "SetInhibitStatus"},
            {CcHeader::RequestMasterInhibitStatus, "RequestMasterInhibitStatus"},
            {CcHeader::ModifyMasterInhibitStatus, "SetMasterInhibitStatus"},
            {CcHeader::ModifyBillOperatingMode, "SetBillOperatingMode"},

            {CcHeader::RequestCountryScalingFactor, "RequestCountryScalingFactor"},
            {CcHeader::RequestVariableSet, "RequestVariableSet"},
            {CcHeader::RequestBillId, "RequestBillId"},
            {CcHeader::RequestCoinId, "RequestCoinId"},

            {CcHeader::RequestBaseYear, "RequestBaseYear"},
            {CcHeader::RequestCommsRevision, "RequestCommsRevision"},
            {CcHeader::RequestBuildCode, "RequestBuildCode"},
            {CcHeader::RequestSoftwareRevision, "RequestSoftwareRevision"},
            {CcHeader::RequestSerialNumber, "RequestSerialNumber"},
            {CcHeader::RequestProductCode, "RequestProductCode"},
            {CcHeader::RequestEquipmentCategoryId, "RequestEquipmentCategoryId"},
            {CcHeader::RequestManufacturerId, "RequestManufacturerId"},

            {CcHeader::RequestStatus, "RequestStatus"},

            {CcHeader::RequestPollingPriority, "RequestPollingPriority"},
            {CcHeader::AddressPoll, "AddressPoll"},
            {CcHeader::SimplePoll, "SimplePoll"},

            {CcHeader::FactorySetupAndTest, "FactorySetupAndTest"},
        };
        
        try {
            return name_map.at(header);
        } catch (const std::exception& e) {
            return "";
        }
    }

    /*
     * Fault code, as returned by PerformSelfCheck command.
     * Extra information may be returned in the second byte.
     * See spec part 3 table 3 and section 14.
     */
    enum class CcFaultCode : uint8_t {
        Ok = 0, ///< No fault
        EepromChecksumCorrupted = 1,
        FaultOnInductiveCoils = 2, ///< Extra info: Coil number
        FaultOnCreditSensor = 3,
        FaultOnPiezoSensor = 4,
        FaultOnReflectiveSensor = 5,
        FaultOnDiameterSensor = 6,
        FaultOnWakeUpSensor = 7,
        FaultOnSorterExitSensors = 8, ///< Extra info: Sensor number
        NvramChecksumCorrupted = 9,
        CoinDispensingError = 10,
        LowLevelSensorError = 11, ///< Extra info: Hopper or tube number
        HighLevelSensorError = 12, ///< Extra info: Hopper or tube number
        CoinCountingError = 13,
        KeypadError = 14, ///< Extra info: Key number
        ButtonError = 15,
        DisplayError = 16,
        CoinAuditingError = 17,
        FaultOnRejectSensor = 18,
        FaultOnCoinReturnMechanism = 19,
        FaultOnCosMechanism = 20,
        FaultOnRimSensor = 21,
        FaultOnThermistor = 22,
        PayoutMotorFault = 23, ///< Extra info: Hopper number
        PayoutTimeout = 24, ///< Extra info: Hopper or tube number
        PayoutJammed = 25, ///< Extra info: Hopper or tube number
        PayoutSensorFault = 26, ///< Extra info: Hopper or tube number
        LevelSensorError = 27, ///< Extra info: Hopper or tube number
        PersonalityModuleNotFitted = 28,
        PersonalityChecksumCorrupted = 29,
        RomChecksumMismatch = 30,
        MissingSlaveDevice = 31, ///< Extra info: Slave address
        InternalCommsBad = 32, ///< Extra info: Slave address
        SupplyVoltageOutsideOperatingLimits = 33,
        TemperatureOutsideOperatingLimits = 34,
        DceFault = 35, ///< Extra info: 1 = coin, 2 = token
        FaultOnBillValidatorSensor = 36, ///< Extra info: Sensor number
        FaultOnBillTransportMotor = 37,
        FaultOnStacker = 38,
        BillJammed = 39,
        RamTestFault = 40,
        FaultOnStringSensor = 41,
        AcceptGateFailedOpen = 42,
        AcceptGateFailedClosed = 43,
        StackerMissing = 44,
        StackerFull = 45,
        FlashMemoryEraseFault = 46,
        FlashMemoryWriteFail = 47,
        SlaveDeviceNotResponding = 48, ///< Extra info: Device number
        FaultOnOptoSensor = 49, ///< Extra info: Opto number
        BatteryFault = 50,
        DoorOpen = 51,
        MicroswitchFault = 52,
        RtcFault = 53,
        FirmwareError = 54,
        InitialisationError = 55,
        SupplyCurrentOutsideOperatingLimits = 56,
        ForcedBootloaderMode = 57,
        UnspecifiedFaultCode = 255, ///< Extra info: Further vendor-specific information

        CustomCommandError = 254, ///< Not in specification. Indicates a problem with getting the fault code.
    };

    /*
     * Event code returned in Result B byte of ReadBufferedCredit command
     * when Result A byte is 0.
     * See spec part 3, table 2 and section 12.2.
     */
    enum class CcCoinAcceptorEventCode : uint8_t {
        NoError = 0,
        RejectCoin = 1,
        InhibitedCoin = 2,
        MultipleWindow = 3,
        WakeupTimeout = 4,
        ValidationTimeout = 5,
        CreditSensorTimeout = 6,
        SorterOptoTimeout = 7,
        SecondCloseCoinError = 8,
        AcceptGateNotReady = 9,
        CreditSensorNotReady = 10,
        SorterNotReady = 11,
        RejectCoinNotCleared = 12,
        ValidationSensorNotReady = 13,
        CreditSensorBlocked = 14,
        SorterOptoBlocked = 15,
        CreditSequenceError = 16,
        CoinGoingBackwards = 17,
        CoinTooFastOverCreditSensor = 18,
        CoinTooSlowOverCreditSensor = 19,
        CosMechanismActivated = 20,
        DceOptoTimeout = 21,
        DceOptoNotSeen = 22,
        CreditSensorReachedTooEarly = 23,
        RejectCoinRepeatedSequentialTrip = 24,
        RejectSlug = 25,
        RejectSensorBlocked = 26,
        GamesOverload = 27,
        MaxCoinMeterPulsesExceeded = 28,
        AcceptGateOpenNotClosed = 29,
        AcceptGateClosedNotOpen = 30,
        ManifoldOptoTimeout = 31,
        ManifoldOptoBlocked = 32,
        ManifoldNotReady = 33,
        SecurityStatusChanged = 34,
        MotorException = 35,
        SwallowedCoin = 36,
        CoinTooFastOverValidationSensor = 37,
        CoinTooSlowOverValidationSensor = 38,
        CoinIncorrectlySorted = 39,
        ExternalLightAttack = 40,
        InhibitedCoinType1 = 128,
        InhibitedCoinType2 = 129,
        InhibitedCoinType3 = 130,
        InhibitedCoinType4 = 131,
        InhibitedCoinType5 = 132,
        InhibitedCoinType6 = 133,
        InhibitedCoinType7 = 134,
        InhibitedCoinType8 = 135,
        InhibitedCoinType9 = 136,
        InhibitedCoinType10 = 137,
        InhibitedCoinType11 = 138,
        InhibitedCoinType12 = 139,
        InhibitedCoinType13 = 140,
        InhibitedCoinType14 = 141,
        InhibitedCoinType15 = 142,
        InhibitedCoinType16 = 143,
        InhibitedCoinType17 = 144,
        InhibitedCoinType18 = 145,
        InhibitedCoinType19 = 146,
        InhibitedCoinType20 = 147,
        InhibitedCoinType21 = 148,
        InhibitedCoinType22 = 149,
        InhibitedCoinType23 = 150,
        InhibitedCoinType24 = 151,
        InhibitedCoinType25 = 152,
        InhibitedCoinType26 = 153,
        InhibitedCoinType27 = 154,
        InhibitedCoinType28 = 155,
        InhibitedCoinType29 = 156,
        InhibitedCoinType30 = 157,
        InhibitedCoinType31 = 158,
        InhibitedCoinType32 = 159,
        ReservedCreditCancelling1 = 160,
        ReservedCreditCancellingN = 191,
        DataBlockRequest = 253,
        CoinReturnMechanismActivated = 254,
        UnspecifiedAlarmCode = 255,
    };

    inline std::string ccFaultCodeGetDisplayableName(CcFaultCode code) {
        static std::map<CcFaultCode, std::string> name_map = {
            {CcFaultCode::Ok, "No fault"},
            {CcFaultCode::EepromChecksumCorrupted, "EEPROM checksum corrupted"},
            {CcFaultCode::FaultOnInductiveCoils, "Fault on inductive coils"},
            {CcFaultCode::FaultOnCreditSensor, "Fault on credit sensor"},
            {CcFaultCode::FaultOnPiezoSensor, "Fault on piezo sensor"},
            {CcFaultCode::FaultOnReflectiveSensor, "Fault on reflectiveSensor"},
            {CcFaultCode::FaultOnDiameterSensor, "Fault on diameterSensor"},
            {CcFaultCode::FaultOnWakeUpSensor, "Fault on wakeup sensor"},
            {CcFaultCode::FaultOnSorterExitSensors, "Fault on sorter exit sensors"},
            {CcFaultCode::NvramChecksumCorrupted, "NVRAM checksum corrupted"},
            {CcFaultCode::CoinDispensingError, "Coin dispensing error"},
            {CcFaultCode::LowLevelSensorError, "Low level sensor error"},
            {CcFaultCode::HighLevelSensorError, "High level sensor error"},
            {CcFaultCode::CoinCountingError, "Coin counting error"},
            {CcFaultCode::KeypadError, "Keypad error"},
            {CcFaultCode::ButtonError, "Button error"},
            {CcFaultCode::DisplayError, "Display error"},
            {CcFaultCode::CoinAuditingError, "Coin auditing error"},
            {CcFaultCode::FaultOnRejectSensor, "Fault on reject sensor"},
            {CcFaultCode::FaultOnCoinReturnMechanism, "Fault on coin return mechanism"},
            {CcFaultCode::FaultOnCosMechanism, "Fault on CoS mechanism"},
            {CcFaultCode::FaultOnRimSensor, "Fault on rim sensor"},
            {CcFaultCode::FaultOnThermistor, "Fault on thermistor"},
            {CcFaultCode::PayoutMotorFault, "Payout motor fault"},
            {CcFaultCode::PayoutTimeout, "Payout timeout"},
            {CcFaultCode::PayoutJammed, "Payout jammed"},
            {CcFaultCode::PayoutSensorFault, "Payout sensor fault"},
            {CcFaultCode::LevelSensorError, "Level sensor error"},
            {CcFaultCode::PersonalityModuleNotFitted, "Personality module not fitted"},
            {CcFaultCode::PersonalityChecksumCorrupted, "Personality checksum corrupted"},
            {CcFaultCode::RomChecksumMismatch, "ROM checksum mismatch"},
            {CcFaultCode::MissingSlaveDevice, "Missing slave device"},
            {CcFaultCode::InternalCommsBad, "Internal comms bad"},
            {CcFaultCode::SupplyVoltageOutsideOperatingLimits, "Supply voltage outside operating limits"},
            {CcFaultCode::TemperatureOutsideOperatingLimits, "Temperature outside operating limits"},
            {CcFaultCode::DceFault, "DCE fault"},
            {CcFaultCode::FaultOnBillValidatorSensor, "Fault on bill validator sensor"},
            {CcFaultCode::FaultOnBillTransportMotor, "Fault on bill transport motor"},
            {CcFaultCode::FaultOnStacker, "Fault on stacker"},
            {CcFaultCode::BillJammed, "Bill jammed"},
            {CcFaultCode::RamTestFault, "RAM test fault"},
            {CcFaultCode::FaultOnStringSensor, "Fault on string sensor"},
            {CcFaultCode::AcceptGateFailedOpen, "Accept gate failed open"},
            {CcFaultCode::AcceptGateFailedClosed, "Accept gate failed closed"},
            {CcFaultCode::StackerMissing, "Stacker missing"},
            {CcFaultCode::StackerFull, "Stacker full"},
            {CcFaultCode::FlashMemoryEraseFault, "Flash memory erase fault"},
            {CcFaultCode::FlashMemoryWriteFail, "Flash memory writeFail"},
            {CcFaultCode::SlaveDeviceNotResponding, "Slave device not responding"},
            {CcFaultCode::FaultOnOptoSensor, "Fault on opto sensor"},
            {CcFaultCode::BatteryFault, "Battery fault"},
            {CcFaultCode::DoorOpen, "Door open"},
            {CcFaultCode::MicroswitchFault, "Microswitch fault"},
            {CcFaultCode::RtcFault, "RTC fault"},
            {CcFaultCode::FirmwareError, "Firmware Error"},
            {CcFaultCode::InitialisationError, "Initialisation Error"},
            {CcFaultCode::SupplyCurrentOutsideOperatingLimits, "Supply current outside operating limits"},
            {CcFaultCode::ForcedBootloaderMode, "Forced bootloader mode"},
            {CcFaultCode::UnspecifiedFaultCode, "Unspecified fault code"},
            {CcFaultCode::CustomCommandError, "Custom command error"},
        };

        try {
            return name_map.at(code);
        } catch (const std::exception& e) {
            return "";
        }
    }

    /// Get displayable name

    inline std::string ccCoinAcceptorEventCodeGetDisplayableName(CcCoinAcceptorEventCode code) {
        static std::map<CcCoinAcceptorEventCode, std::string> name_map = {
            {CcCoinAcceptorEventCode::NoError, "NoError"},
            {CcCoinAcceptorEventCode::RejectCoin, "RejectCoin"},
            {CcCoinAcceptorEventCode::InhibitedCoin, "InhibitedCoin"},
            {CcCoinAcceptorEventCode::MultipleWindow, "MultipleWindow"},
            {CcCoinAcceptorEventCode::WakeupTimeout, "WakeupTimeout"},
            {CcCoinAcceptorEventCode::ValidationTimeout, "ValidationTimeout"},
            {CcCoinAcceptorEventCode::CreditSensorTimeout, "CreditSensorTimeout"},
            {CcCoinAcceptorEventCode::SorterOptoTimeout, "SorterOptoTimeout"},
            {CcCoinAcceptorEventCode::SecondCloseCoinError, "SecondCloseCoinError"},
            {CcCoinAcceptorEventCode::AcceptGateNotReady, "AcceptGateNotReady"},
            {CcCoinAcceptorEventCode::CreditSensorNotReady, "CreditSensorNotReady"},
            {CcCoinAcceptorEventCode::SorterNotReady, "SorterNotReady"},
            {CcCoinAcceptorEventCode::RejectCoinNotCleared, "RejectCoinNotCleared"},
            {CcCoinAcceptorEventCode::ValidationSensorNotReady, "ValidationSensorNotReady"},
            {CcCoinAcceptorEventCode::CreditSensorBlocked, "CreditSensorBlocked"},
            {CcCoinAcceptorEventCode::SorterOptoBlocked, "SorterOptoBlocked"},
            {CcCoinAcceptorEventCode::CreditSequenceError, "CreditSequenceError"},
            {CcCoinAcceptorEventCode::CoinGoingBackwards, "CoinGoingBackwards"},
            {CcCoinAcceptorEventCode::CoinTooFastOverCreditSensor, "CoinTooFastOverCreditSensor"},
            {CcCoinAcceptorEventCode::CoinTooSlowOverCreditSensor, "CoinTooSlowOverCreditSensor"},
            {CcCoinAcceptorEventCode::CosMechanismActivated, "CosMechanismActivated"},
            {CcCoinAcceptorEventCode::DceOptoTimeout, "DceOptoTimeout"},
            {CcCoinAcceptorEventCode::DceOptoNotSeen, "DceOptoNotSeen"},
            {CcCoinAcceptorEventCode::CreditSensorReachedTooEarly, "CreditSensorReachedTooEarly"},
            {CcCoinAcceptorEventCode::RejectCoinRepeatedSequentialTrip, "RejectCoinRepeatedSequentialTrip"},
            {CcCoinAcceptorEventCode::RejectSlug, "RejectSlug"},
            {CcCoinAcceptorEventCode::RejectSensorBlocked, "RejectSensorBlocked"},
            {CcCoinAcceptorEventCode::GamesOverload, "GamesOverload"},
            {CcCoinAcceptorEventCode::MaxCoinMeterPulsesExceeded, "MaxCoinMeterPulsesExceeded"},
            {CcCoinAcceptorEventCode::AcceptGateOpenNotClosed, "AcceptGateOpenNotClosed"},
            {CcCoinAcceptorEventCode::AcceptGateClosedNotOpen, "AcceptGateClosedNotOpen"},
            {CcCoinAcceptorEventCode::ManifoldOptoTimeout, "ManifoldOptoTimeout"},
            {CcCoinAcceptorEventCode::ManifoldOptoBlocked, "ManifoldOptoBlocked"},
            {CcCoinAcceptorEventCode::ManifoldNotReady, "ManifoldNotReady"},
            {CcCoinAcceptorEventCode::SecurityStatusChanged, "SecurityStatusChanged"},
            {CcCoinAcceptorEventCode::MotorException, "MotorException"},
            {CcCoinAcceptorEventCode::SwallowedCoin, "SwallowedCoin"},
            {CcCoinAcceptorEventCode::CoinTooFastOverValidationSensor, "CoinTooFastOverValidationSensor"},
            {CcCoinAcceptorEventCode::CoinTooSlowOverValidationSensor, "CoinTooSlowOverValidationSensor"},
            {CcCoinAcceptorEventCode::CoinIncorrectlySorted, "CoinIncorrectlySorted"},
            {CcCoinAcceptorEventCode::ExternalLightAttack, "ExternalLightAttack"},
            {CcCoinAcceptorEventCode::InhibitedCoinType1, "InhibitedCoinType1"},
            {CcCoinAcceptorEventCode::InhibitedCoinType2, "InhibitedCoinType2"},
            {CcCoinAcceptorEventCode::InhibitedCoinType3, "InhibitedCoinType3"},
            {CcCoinAcceptorEventCode::InhibitedCoinType4, "InhibitedCoinType4"},
            {CcCoinAcceptorEventCode::InhibitedCoinType5, "InhibitedCoinType5"},
            {CcCoinAcceptorEventCode::InhibitedCoinType6, "InhibitedCoinType6"},
            {CcCoinAcceptorEventCode::InhibitedCoinType7, "InhibitedCoinType7"},
            {CcCoinAcceptorEventCode::InhibitedCoinType8, "InhibitedCoinType8"},
            {CcCoinAcceptorEventCode::InhibitedCoinType9, "InhibitedCoinType9"},
            {CcCoinAcceptorEventCode::InhibitedCoinType10, "InhibitedCoinType10"},
            {CcCoinAcceptorEventCode::InhibitedCoinType11, "InhibitedCoinType11"},
            {CcCoinAcceptorEventCode::InhibitedCoinType12, "InhibitedCoinType12"},
            {CcCoinAcceptorEventCode::InhibitedCoinType13, "InhibitedCoinType13"},
            {CcCoinAcceptorEventCode::InhibitedCoinType14, "InhibitedCoinType14"},
            {CcCoinAcceptorEventCode::InhibitedCoinType15, "InhibitedCoinType15"},
            {CcCoinAcceptorEventCode::InhibitedCoinType16, "InhibitedCoinType16"},
            {CcCoinAcceptorEventCode::InhibitedCoinType17, "InhibitedCoinType17"},
            {CcCoinAcceptorEventCode::InhibitedCoinType18, "InhibitedCoinType18"},
            {CcCoinAcceptorEventCode::InhibitedCoinType19, "InhibitedCoinType19"},
            {CcCoinAcceptorEventCode::InhibitedCoinType20, "InhibitedCoinType20"},
            {CcCoinAcceptorEventCode::InhibitedCoinType21, "InhibitedCoinType21"},
            {CcCoinAcceptorEventCode::InhibitedCoinType22, "InhibitedCoinType22"},
            {CcCoinAcceptorEventCode::InhibitedCoinType23, "InhibitedCoinType23"},
            {CcCoinAcceptorEventCode::InhibitedCoinType24, "InhibitedCoinType24"},
            {CcCoinAcceptorEventCode::InhibitedCoinType25, "InhibitedCoinType25"},
            {CcCoinAcceptorEventCode::InhibitedCoinType26, "InhibitedCoinType26"},
            {CcCoinAcceptorEventCode::InhibitedCoinType27, "InhibitedCoinType27"},
            {CcCoinAcceptorEventCode::InhibitedCoinType28, "InhibitedCoinType28"},
            {CcCoinAcceptorEventCode::InhibitedCoinType29, "InhibitedCoinType29"},
            {CcCoinAcceptorEventCode::InhibitedCoinType30, "InhibitedCoinType30"},
            {CcCoinAcceptorEventCode::InhibitedCoinType31, "InhibitedCoinType31"},
            {CcCoinAcceptorEventCode::InhibitedCoinType32, "InhibitedCoinType32"},
            {CcCoinAcceptorEventCode::ReservedCreditCancelling1, "ReservedCreditCancelling1"},
            {CcCoinAcceptorEventCode::ReservedCreditCancellingN, "ReservedCreditCancellingN"},
            {CcCoinAcceptorEventCode::DataBlockRequest, "DataBlockRequest"},
            {CcCoinAcceptorEventCode::CoinReturnMechanismActivated, "CoinReturnMechanismActivated"},
            {CcCoinAcceptorEventCode::UnspecifiedAlarmCode, "UnspecifiedAlarmCode"},
        };

        try {
            return name_map.at(code);
        } catch (const std::exception& e) {
            return "";
        }
    }



    /// Coin rejection type for each CcCoinAcceptorEventCode

    enum class CcCoinRejectionType {
        Rejected,
        Accepted,
        Unknown,
    };



    /// Get displayable name

    inline std::string ccCoinRejectionTypeGetDisplayableName(CcCoinRejectionType type) {
        static std::map<CcCoinRejectionType, std::string> name_map = {
            {CcCoinRejectionType::Rejected, "Rejected"},
            {CcCoinRejectionType::Accepted, "Accepted"},
            {CcCoinRejectionType::Unknown, "Unknown"},
        };

        try {
            return name_map.at(type);
        } catch (const std::exception& e) {
            return "";
        }
    }



    /// See spec part 3, table 2 and section 12.2.

    inline CcCoinRejectionType ccCoinAcceptorEventCodeGetRejectionType(CcCoinAcceptorEventCode code) {
        switch (code) {
            case CcCoinAcceptorEventCode::NoError:
            case CcCoinAcceptorEventCode::SorterOptoTimeout:
            case CcCoinAcceptorEventCode::CreditSequenceError:
            case CcCoinAcceptorEventCode::CoinGoingBackwards:
            case CcCoinAcceptorEventCode::CoinTooFastOverCreditSensor:
            case CcCoinAcceptorEventCode::CoinTooSlowOverCreditSensor:
            case CcCoinAcceptorEventCode::CosMechanismActivated:
            case CcCoinAcceptorEventCode::CreditSensorReachedTooEarly:
            case CcCoinAcceptorEventCode::RejectSensorBlocked:
            case CcCoinAcceptorEventCode::GamesOverload:
            case CcCoinAcceptorEventCode::MaxCoinMeterPulsesExceeded:
            case CcCoinAcceptorEventCode::AcceptGateOpenNotClosed:
            case CcCoinAcceptorEventCode::ManifoldOptoTimeout:
            case CcCoinAcceptorEventCode::SwallowedCoin:
            case CcCoinAcceptorEventCode::CoinIncorrectlySorted:
            case CcCoinAcceptorEventCode::ExternalLightAttack:
            case CcCoinAcceptorEventCode::DataBlockRequest:
            case CcCoinAcceptorEventCode::CoinReturnMechanismActivated:
            case CcCoinAcceptorEventCode::UnspecifiedAlarmCode:
                return CcCoinRejectionType::Accepted;

            case CcCoinAcceptorEventCode::WakeupTimeout:
            case CcCoinAcceptorEventCode::ValidationTimeout:
            case CcCoinAcceptorEventCode::CreditSensorTimeout:
            case CcCoinAcceptorEventCode::DceOptoTimeout:
            case CcCoinAcceptorEventCode::SecurityStatusChanged:
            case CcCoinAcceptorEventCode::MotorException:
            case CcCoinAcceptorEventCode::ReservedCreditCancelling1:
            case CcCoinAcceptorEventCode::ReservedCreditCancellingN:
                return CcCoinRejectionType::Unknown;

            case CcCoinAcceptorEventCode::RejectCoin:
            case CcCoinAcceptorEventCode::InhibitedCoin:
            case CcCoinAcceptorEventCode::MultipleWindow:
            case CcCoinAcceptorEventCode::SecondCloseCoinError: // rejected 1 or more
            case CcCoinAcceptorEventCode::AcceptGateNotReady:
            case CcCoinAcceptorEventCode::CreditSensorNotReady:
            case CcCoinAcceptorEventCode::SorterNotReady:
            case CcCoinAcceptorEventCode::RejectCoinNotCleared:
            case CcCoinAcceptorEventCode::ValidationSensorNotReady:
            case CcCoinAcceptorEventCode::CreditSensorBlocked:
            case CcCoinAcceptorEventCode::SorterOptoBlocked:
            case CcCoinAcceptorEventCode::DceOptoNotSeen:
            case CcCoinAcceptorEventCode::RejectCoinRepeatedSequentialTrip:
            case CcCoinAcceptorEventCode::RejectSlug:
            case CcCoinAcceptorEventCode::AcceptGateClosedNotOpen:
            case CcCoinAcceptorEventCode::ManifoldOptoBlocked:
            case CcCoinAcceptorEventCode::ManifoldNotReady:
            case CcCoinAcceptorEventCode::CoinTooFastOverValidationSensor:
            case CcCoinAcceptorEventCode::CoinTooSlowOverValidationSensor:
            case CcCoinAcceptorEventCode::InhibitedCoinType1:
            case CcCoinAcceptorEventCode::InhibitedCoinType2:
            case CcCoinAcceptorEventCode::InhibitedCoinType3:
            case CcCoinAcceptorEventCode::InhibitedCoinType4:
            case CcCoinAcceptorEventCode::InhibitedCoinType5:
            case CcCoinAcceptorEventCode::InhibitedCoinType6:
            case CcCoinAcceptorEventCode::InhibitedCoinType7:
            case CcCoinAcceptorEventCode::InhibitedCoinType8:
            case CcCoinAcceptorEventCode::InhibitedCoinType9:
            case CcCoinAcceptorEventCode::InhibitedCoinType10:
            case CcCoinAcceptorEventCode::InhibitedCoinType11:
            case CcCoinAcceptorEventCode::InhibitedCoinType12:
            case CcCoinAcceptorEventCode::InhibitedCoinType13:
            case CcCoinAcceptorEventCode::InhibitedCoinType14:
            case CcCoinAcceptorEventCode::InhibitedCoinType15:
            case CcCoinAcceptorEventCode::InhibitedCoinType16:
            case CcCoinAcceptorEventCode::InhibitedCoinType17:
            case CcCoinAcceptorEventCode::InhibitedCoinType18:
            case CcCoinAcceptorEventCode::InhibitedCoinType19:
            case CcCoinAcceptorEventCode::InhibitedCoinType20:
            case CcCoinAcceptorEventCode::InhibitedCoinType21:
            case CcCoinAcceptorEventCode::InhibitedCoinType22:
            case CcCoinAcceptorEventCode::InhibitedCoinType23:
            case CcCoinAcceptorEventCode::InhibitedCoinType24:
            case CcCoinAcceptorEventCode::InhibitedCoinType25:
            case CcCoinAcceptorEventCode::InhibitedCoinType26:
            case CcCoinAcceptorEventCode::InhibitedCoinType27:
            case CcCoinAcceptorEventCode::InhibitedCoinType28:
            case CcCoinAcceptorEventCode::InhibitedCoinType29:
            case CcCoinAcceptorEventCode::InhibitedCoinType30:
            case CcCoinAcceptorEventCode::InhibitedCoinType31:
            case CcCoinAcceptorEventCode::InhibitedCoinType32:
                return CcCoinRejectionType::Rejected;
        }
        return CcCoinRejectionType::Unknown;
    }



    /// Event code returned in Result B byte of ReadBufferedBillEvents command
    /// when Result A byte is 0.
    /// See spec table 7.

    enum class CcBillValidatorErrorCode : uint8_t {
        MasterInhibitActive = 0,
        BillReturnedFromEscrow = 1,
        InvalidBillValidationFail = 2,
        InvalidBillTransportProblem = 3,
        InhibitedBillOnSerial = 4,
        InhibitedBillOnDipSwitches = 5,
        BillJammedInTransportUnsafeMode = 6,
        BillJammedInStacker = 7,
        BillPulledBackwards = 8,
        BillTamper = 9,
        StackerOk = 10,
        StackerRemoved = 11,
        StackerInserted = 12,
        StackerFaulty = 13,
        StackerFull = 14,
        StackerJammed = 15,
        BillJammedInTransportSafeMode = 16,
        OptoFraudDetected = 17,
        StringFraudDetected = 18,
        AntiStringMechanismFaulty = 19,
        BarcodeDetected = 20,
        UnknownBillTypeStacked = 21,

        CustomNoError = 255, ///< Not in specification, only for default-initializing variables.
    };



    /// Get displayable name

    inline std::string ccBillValidatorErrorCodeGetDisplayableName(CcBillValidatorErrorCode type) {
        static std::map<CcBillValidatorErrorCode, std::string> name_map = {
            {CcBillValidatorErrorCode::MasterInhibitActive, "MasterInhibitActive"},
            {CcBillValidatorErrorCode::BillReturnedFromEscrow, "BillReturnedFromEscrow"},
            {CcBillValidatorErrorCode::InvalidBillValidationFail, "InvalidBillValidationFail"},
            {CcBillValidatorErrorCode::InvalidBillTransportProblem, "InvalidBillTransportProblem"},
            {CcBillValidatorErrorCode::InhibitedBillOnSerial, "InhibitedBillOnSerial"},
            {CcBillValidatorErrorCode::InhibitedBillOnDipSwitches, "InhibitedBillOnDipSwitches"},
            {CcBillValidatorErrorCode::BillJammedInTransportUnsafeMode, "BillJammedInTransportUnsafeMode"},
            {CcBillValidatorErrorCode::BillJammedInStacker, "BillJammedInStacker"},
            {CcBillValidatorErrorCode::BillPulledBackwards, "BillPulledBackwards"},
            {CcBillValidatorErrorCode::BillTamper, "BillTamper"},
            {CcBillValidatorErrorCode::StackerOk, "StackerOk"},
            {CcBillValidatorErrorCode::StackerRemoved, "StackerRemoved"},
            {CcBillValidatorErrorCode::StackerInserted, "StackerInserted"},
            {CcBillValidatorErrorCode::StackerFaulty, "StackerFaulty"},
            {CcBillValidatorErrorCode::StackerFull, "StackerFull"},
            {CcBillValidatorErrorCode::StackerJammed, "StackerJammed"},
            {CcBillValidatorErrorCode::BillJammedInTransportSafeMode, "BillJammedInTransportSafeMode"},
            {CcBillValidatorErrorCode::OptoFraudDetected, "OptoFraudDetected"},
            {CcBillValidatorErrorCode::StringFraudDetected, "StringFraudDetected"},
            {CcBillValidatorErrorCode::AntiStringMechanismFaulty, "AntiStringMechanismFaulty"},
            {CcBillValidatorErrorCode::BarcodeDetected, "BarcodeDetected"},
            {CcBillValidatorErrorCode::UnknownBillTypeStacked, "UnknownBillTypeStacked"},

            {CcBillValidatorErrorCode::CustomNoError, "CustomNoError"},
        };

        try {
            return name_map.at(type);
        } catch (const std::exception& e) {
            return "";
        }
    }



    /// Success code returned in Result B byte of ReadBufferedBillEvents command
    /// when Result A byte is 1-255.

    enum class CcBillValidatorSuccessCode : uint8_t {
        ValidatedAndAccepted = 0, ///< Bill accepted, credit the customer
        ValidatedAndHeldInEscrow = 1, ///< Bill held in escrow, decide whether to accept it

        CustomUnknown = 255, ///< Not in specification, only for default-initializing variables.
    };



    /// Get displayable name

    inline std::string ccBillValidatorSuccessCodeGetDisplayableName(CcBillValidatorSuccessCode type) {
        static std::map<CcBillValidatorSuccessCode, std::string> name_map = {
            {CcBillValidatorSuccessCode::ValidatedAndAccepted, "ValidatedAndAccepted"},
            {CcBillValidatorSuccessCode::ValidatedAndHeldInEscrow, "ValidatedAndHeldInEscrow"},
            {CcBillValidatorSuccessCode::CustomUnknown, "CustomUnknown"},
        };

        try {
            return name_map.at(type);
        } catch (const std::exception& e) {
            return "";
        }
    }



    /// Each CcBillValidatorErrorCode and CcBillValidatorSuccessCode has an event type
    /// associated with it. See ccBillValidatorEventCodeGetType() and ccBillValidatorSuccessCodeGetEventType().
    /// See spec section 18.

    enum class CcBillValidatorEventType {
        CustomUnknown = 0, ///< Not in specification, only for default-initializing variables.

        // If Result A is 1-255
        // 	Credit,  ///< Bill accepted, credit the customer
        // 	PendingCredit,  ///< Bill held in escrow, decide whether to accept it

        // If Result A is 0
        Reject, ///< Bill rejected and returned to customer
        FraudAttempt, ///< Fraud detected. Possible machine alarm.
        FatalError, ///< Service callout
        Status, ///< Informational only
    };



    /// Get displayable name

    inline std::string ccBillValidatorEventTypeGetDisplayableName(CcBillValidatorEventType type) {
        static std::map<CcBillValidatorEventType, std::string> name_map = {
            {CcBillValidatorEventType::CustomUnknown, "Custom unknown"},
            // 		{CcBillValidatorEventType::Credit, "Credit"},
            // 		{CcBillValidatorEventType::PendingCredit, "PendingCredit"},
            {CcBillValidatorEventType::Reject, "Reject"},
            {CcBillValidatorEventType::FraudAttempt, "Fraud attempt"},
            {CcBillValidatorEventType::FatalError, "Fatal error"},
            {CcBillValidatorEventType::Status, "Status"},
        };

        try {
            return name_map.at(type);
        } catch (const std::exception& e) {
            return "";
        }
    }



    /// Get event type for CcBillValidatorErrorCode

    inline CcBillValidatorEventType ccBillValidatorErrorCodeGetEventType(CcBillValidatorErrorCode status) {
        static std::map<CcBillValidatorErrorCode, CcBillValidatorEventType> type_map = {
            {CcBillValidatorErrorCode::MasterInhibitActive, CcBillValidatorEventType::Status},
            {CcBillValidatorErrorCode::BillReturnedFromEscrow, CcBillValidatorEventType::Status},
            {CcBillValidatorErrorCode::InvalidBillValidationFail, CcBillValidatorEventType::Reject},
            {CcBillValidatorErrorCode::InvalidBillTransportProblem, CcBillValidatorEventType::Reject},
            {CcBillValidatorErrorCode::InhibitedBillOnSerial, CcBillValidatorEventType::Status},
            {CcBillValidatorErrorCode::InhibitedBillOnDipSwitches, CcBillValidatorEventType::Status},
            {CcBillValidatorErrorCode::BillJammedInTransportUnsafeMode, CcBillValidatorEventType::FatalError},
            {CcBillValidatorErrorCode::BillJammedInStacker, CcBillValidatorEventType::FatalError},
            {CcBillValidatorErrorCode::BillPulledBackwards, CcBillValidatorEventType::FraudAttempt},
            {CcBillValidatorErrorCode::BillTamper, CcBillValidatorEventType::FraudAttempt},
            {CcBillValidatorErrorCode::StackerOk, CcBillValidatorEventType::Status},
            {CcBillValidatorErrorCode::StackerRemoved, CcBillValidatorEventType::Status},
            {CcBillValidatorErrorCode::StackerInserted, CcBillValidatorEventType::Status},
            {CcBillValidatorErrorCode::StackerFaulty, CcBillValidatorEventType::FatalError},
            {CcBillValidatorErrorCode::StackerFull, CcBillValidatorEventType::Status},
            {CcBillValidatorErrorCode::StackerJammed, CcBillValidatorEventType::FatalError},
            {CcBillValidatorErrorCode::BillJammedInTransportSafeMode, CcBillValidatorEventType::FatalError},
            {CcBillValidatorErrorCode::OptoFraudDetected, CcBillValidatorEventType::FraudAttempt},
            {CcBillValidatorErrorCode::StringFraudDetected, CcBillValidatorEventType::FraudAttempt},
            {CcBillValidatorErrorCode::AntiStringMechanismFaulty, CcBillValidatorEventType::FatalError},
            {CcBillValidatorErrorCode::BarcodeDetected, CcBillValidatorEventType::Status},
            {CcBillValidatorErrorCode::UnknownBillTypeStacked, CcBillValidatorEventType::Status},
            {CcBillValidatorErrorCode::CustomNoError, CcBillValidatorEventType::FatalError},
        };

        try {
            return type_map.at(status);
        } catch (const std::exception& e) {
            return CcBillValidatorEventType::FatalError;
        }
    }

    /*
     *  Get default cctalk address for a device category
     */
    inline uint8_t ccCategoryGetDefaultAddress(CcCategory category) {
        static std::map<CcCategory, uint8_t> address_map = {
            {CcCategory::CoinAcceptor, 2},
            {CcCategory::Payout, 3},
            {CcCategory::Reel, 30},
            {CcCategory::BillValidator, 40},
            {CcCategory::CardReader, 50},
            {CcCategory::Changer, 55},
            {CcCategory::Display, 60},
            {CcCategory::Keypad, 70},
            {CcCategory::Dongle, 80},
            {CcCategory::Meter, 90},
            {CcCategory::Bootloader, 99},
            {CcCategory::Power, 100},
            {CcCategory::Printer, 110},
            {CcCategory::Rng, 120},
            {CcCategory::HopperScale, 130},
            {CcCategory::CoinFeeder, 140},
            {CcCategory::BillRecycler, 150},
            {CcCategory::Escrow, 160},
            {CcCategory::Debug, 240},
        };

        return address_map.at(category);
    }

    inline CcCategory ccCategoryFromAddress(uint8_t address) {
        if (address == 2 || (address >= 11 && address <= 17)) return CcCategory::CoinAcceptor;
        if (address == 3 || (address >= 4 && address <= 10)) return CcCategory::Payout;
        if (address == 30 || (address >= 31 && address <= 34)) return CcCategory::Reel;
        if (address == 40 || (address >= 41 && address <= 47)) return CcCategory::BillValidator;
        if (address == 50) return CcCategory::CardReader;
        if (address == 55) return CcCategory::Changer;
        if (address == 60) return CcCategory::Display;
        if (address == 70) return CcCategory::Keypad;
        if (address == 80 || (address >= 85 && address <= 89)) return CcCategory::Dongle;
        if (address == 90) return CcCategory::Meter;
        if (address == 99) return CcCategory::Bootloader;
        if (address == 100) return CcCategory::Power;
        if (address == 110) return CcCategory::Printer;
        if (address == 120) return CcCategory::Rng;
        if (address == 130) return CcCategory::HopperScale;
        if (address == 140) return CcCategory::CoinFeeder;
        if (address == 150) return CcCategory::BillRecycler;
        if (address == 160) return CcCategory::Escrow;
        if (address == 240 || address >= 241) return CcCategory::Debug;
        return CcCategory::Unknown;
    }

    /// Return equipment category from its reported name

    inline CcCategory ccCategoryFromReportedName(std::string reported_name) {
        static std::map<std::string, CcCategory> name_map = {
            {"Coin Acceptor", CcCategory::CoinAcceptor},
            {"Payout", CcCategory::Payout},
            {"Reel", CcCategory::Reel},
            {"Bill Validator", CcCategory::BillValidator},
            {"Card Reader", CcCategory::CardReader},
            {"Changer", CcCategory::Changer},
            {"Display", CcCategory::Display},
            {"Keypad", CcCategory::Keypad},
            {"Dongle", CcCategory::Dongle},
            {"Meter", CcCategory::Meter},
            {"Bootloader", CcCategory::Bootloader},
            {"Power", CcCategory::Power},
            {"Printer", CcCategory::Printer},
            {"RNG", CcCategory::Rng},
            {"Hopper Scale", CcCategory::HopperScale},
            {"Coin Feeder", CcCategory::CoinFeeder},
            {"Bill Recycler", CcCategory::BillRecycler},
            {"Escrow", CcCategory::Escrow},
            {"Debug", CcCategory::Debug},
        };

        // We replace _ with a space as an extension to support not-quite-compliant devices.
        try {
            std::replace(reported_name.begin(), reported_name.end(), '_', ' ');
            return name_map.at(reported_name); // TODO: trim
        } catch (const std::exception& e) {
            return CcCategory::Unknown;
        }
    }

    /*
     *  This is a parameter for RouteBill command.
     */
    enum class CcBillRouteCommandType : uint8_t {
        ReturnBill = 0, ///< Reject
        RouteToStacker = 1, ///< Accept
        IncreaseTimeout = 255, ///< Give more time to decide
    };

    /*
     *  Get displayable name
     */
    inline std::string ccBillRouteCommandTypeGetDisplayableName(CcBillRouteCommandType type) {
        static std::map<CcBillRouteCommandType, std::string> name_map = {
            {CcBillRouteCommandType::ReturnBill, "ReturnBill"},
            {CcBillRouteCommandType::RouteToStacker, "RouteToStacker"},
            {CcBillRouteCommandType::IncreaseTimeout, "IncreaseTimeout"},
        };
        try {
            return name_map.at(type);
        } catch (const std::exception& e) {
            return "";
        }
    }

    /* 
     * This is a returned value of the RouteBill command.     
     */
    enum class CcBillRouteStatus : uint8_t {
        Routed = 0, ///< The bill has been routed to stacker. Corresponds to ACK reply.
        EscrowEmpty = 254, ///< The escrow is empty, cannot route
        FailedToRoute = 255, ///< Failed to route
    };

    /*
     *  Get displayable name
     */
    inline std::string ccBillRouteStatusGetDisplayableName(CcBillRouteStatus type) {
        static std::map<CcBillRouteStatus, std::string> name_map = {
            {CcBillRouteStatus::Routed, "Routed"},
            {CcBillRouteStatus::EscrowEmpty, "EscrowEmpty"},
            {CcBillRouteStatus::FailedToRoute, "FailedToRoute"},
        };

        try {
            return name_map.at(type);
        } catch (const std::exception& e) {
            return "";
        }
    }

    /*
     * ccTalk event data, as returned by ReadBufferedBillEvents and ReadBufferedCredit commands
     */
    struct CcEventData {
        /// Container requirement
        CcEventData() = default;

        /// Constructor

        CcEventData(uint8_t arg_result_A, uint8_t arg_result_B, CcCategory device_category) : result_A(arg_result_A), result_B(arg_result_B) {
            if (device_category == CcCategory::CoinAcceptor) {
                if (result_A == 0) { // error
                    coin_id = 0;
                    coin_error_code = CcCoinAcceptorEventCode(result_B);
                } else {
                    coin_id = result_A;
                    coin_sorter_path = result_B;
                }
            } else if (device_category == CcCategory::BillValidator) {
                if (result_A == 0) { // error
                    bill_id = 0;
                    bill_error_code = CcBillValidatorErrorCode(result_B);
                    bill_event_type = ccBillValidatorErrorCodeGetEventType(bill_error_code);
                } else {
                    bill_id = result_A;
                    bill_success_code = CcBillValidatorSuccessCode(result_B);
                    // 				bill_event_type = ccBillValidatorSuccessCodeGetEventType(bill_success_code);
                }
            }
        }

        [[nodiscard]] bool hasError() const {
            return result_A == 0;
        }


        uint8_t result_A = 0; ///< Credit (coin position) if in [1-255] range and error_code contains sorter_path. If 0, error_code contains CcCoinAcceptorEventCode.
        uint8_t result_B = 0; ///< If credit, then this is a sorter path (unused). If error, this is an error code.

        uint8_t coin_id = 0; ///< 1-16. Coin ID number (position in GetCoinId). 0 means error.
        CcCoinAcceptorEventCode coin_error_code = CcCoinAcceptorEventCode::NoError;
        uint8_t coin_sorter_path = 0; ///< 0 if unsupported.

        uint8_t bill_id = 0; ///< 1-16, Bill ID number (position in GetBillId). 0 means error.
        CcBillValidatorErrorCode bill_error_code = CcBillValidatorErrorCode::CustomNoError; ///< If result A is 0.
        CcBillValidatorSuccessCode bill_success_code = CcBillValidatorSuccessCode::CustomUnknown; ///< If result A is 1-255.
        CcBillValidatorEventType bill_event_type = CcBillValidatorEventType::CustomUnknown; ///< Event type (both error and success)

    };

    /*
     *  Get coin values according to coin code (cctalk spec Appendix 3 (2.1))
     */
    inline uint64_t ccCoinValueCodeGetValue(const std::string& three_char_code, uint8_t& decimal_places) {
        // coin code -> (coin_value, decimal_places).
        static std::map<std::string, std::pair<uint64_t, uint8_t>> value_map = {
            {"5m0",
                {5, 3}}, // 0.005
            {"10m",
                {1, 2}}, // 0.01
            {".01",
                {1, 2}}, // 0.01
            {"20m",
                {2, 2}}, // 0.02
            {".02",
                {2, 2}}, // 0.02
            {"25m",
                {25, 3}}, // 0.025
            {"50m",
                {5, 2}}, // 0.05
            {".05",
                {5, 2}}, // 0.05
            {".10",
                {1, 1}}, // 0.10
            {".20",
                {2, 1}}, // 0.20
            {".25",
                {25, 2}}, // 0.25
            {".50",
                {5, 1}}, // 0.50
            {"001",
                {1, 0}}, // 1
            {"002",
                {1, 0}}, // 2
            {"2.5",
                {25, 1}}, // 2.5
            {"005",
                {5, 0}}, // 5
            {"010",
                {10, 0}}, // 10
            {"020",
                {20, 0}}, // 20
            {"025",
                {25, 0}}, // 25
            {"050",
                {50, 0}}, // 50
            {"100",
                {100, 0}}, // 100
            {"200",
                {200, 0}}, // 200
            {"250",
                {250, 0}}, // 250
            {"500",
                {500, 0}}, // 500
            {"1K0",
                {1000, 0}}, // 1 000
            {"2K0",
                {2000, 0}}, // 2 000
            {"2K5",
                {2500, 0}}, // 2 500
            {"5K0",
                {5000, 0}}, // 5 000
            {"10K",
                {10000, 0}}, // 10 000
            {"20K",
                {20000, 0}}, // 20 000
            {"25K",
                {25000, 0}}, // 25 000
            {"50K",
                {50000, 0}}, // 50 000
            {"M10",
                {100000, 0}}, // 100 000
            {"M20",
                {200000, 0}}, // 200 000
            {"M25",
                {250000, 0}}, // 250 000
            {"M50",
                {500000, 0}}, // 500 000
            {"1M0",
                {1000000, 0}}, // 1 000 000
            {"2M0",
                {2000000, 0}}, // 2 000 000
            {"2M5",
                {2500000, 0}}, // 2 500 000
            {"5M0",
                {5000000, 0}}, // 5 000 000
            {"10M",
                {10000000, 0}}, // 10 000 000
            {"20M",
                {20000000, 0}}, // 20 000 000
            {"25M",
                {25000000, 0}}, // 25 000 000
            {"50M",
                {50000000, 0}}, // 50 000 000
            {"G10",
                {100000000, 0}}, // 100 000 000
        };

        std::pair<uint64_t, uint8_t> value;
        try {
            value = value_map.at(three_char_code);
        } catch (const std::exception& e) {
            value = std::pair<uint64_t, uint8_t>(0, 0);
        }

        decimal_places = value.second;

        return value.first;
    }



    /// Country scaling data, as returned by GetCountryScalingFactor command.

    struct CcCountryScalingData {
        /// Scaling factor from country scaling data.
        /// The bill identifier values should be
        /// multiplied by this to get cents.
        uint16_t scaling_factor = 1;

        /// Decimal places from country scaling data. 2 for USD (10^2 cents in USD)
        uint8_t decimal_places = 0;

        /// If country code is unsupported, this returns false.
        [[nodiscard]] bool isValid() const {
            return scaling_factor != 0 || decimal_places != 0;
        }
    };



    /// ccTalk coin / bill identifier, as returned by GetBillId and GetCoinId commands.

    struct CcIdentifier {
        /// std::map requirement
        CcIdentifier() = default;

        // Parse ID string and store the results

        explicit CcIdentifier(std::string arg_id_string) : id_string(std::move(arg_id_string)) {
            if (arg_id_string.size() == 7) { // Bills
                country = arg_id_string.substr(0, 2);
                issue_code = arg_id_string.at(arg_id_string.size() - 1);

                value_code = std::stoull(arg_id_string.substr(2, 4), nullptr, 10);

            } else if (arg_id_string.size() == 6) { // Coins
                country = arg_id_string.substr(0, 2);
                issue_code = arg_id_string.at(arg_id_string.size() - 1);
                value_code = ccCoinValueCodeGetValue(arg_id_string.substr(2, 3), coin_decimals);

            } else {
                //DBG_ASSERT(0);
            }
        }

        /// Set country scaling data for bills and coins. For bills this can be retrieved
        /// from the device, while for coins it has to be predefined by us.

        void setCountryScalingData(CcCountryScalingData data) {
            country_scaling_data = data;
        }

        /// Get coin / bill value. The returned value should be divided by \c 10^divisor
        /// to get a value in country currency (e.g. USD).
        /// For coin acceptors the divisor is always 1.

        uint64_t getValue(uint64_t& divisor) const {
            divisor = country_scaling_data.decimal_places + coin_decimals;
            return value_code * country_scaling_data.scaling_factor;
        }


        std::string id_string; ///< Bill / coin identifier, e.g. "GE0005A" for the first ("A") issue of Georgian 5 lari (value code 0005).
        std::string country; ///< Country identifier, e.g. "GE".
        char issue_code = 0; ///< Issue code (A, B, C, ...), to differentiate various issues of the same-value coin.
        uint64_t value_code = 0; ///< Value code (before country scaling, if it's a bill).
        uint8_t coin_decimals = 0; ///< Value code should be divided by 10^coin_decimals to get the real value.

        CcCountryScalingData country_scaling_data; ///< Coin / bill scaling data
    };

}
#endif /* CCTALK_ENUMS_H */

