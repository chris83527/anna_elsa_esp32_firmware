#pragma once
#include <cstdint>
#include <map>

// Full numeric coverage: 0–255
namespace CctalkHeaders
{
    constexpr uint8_t Ack = 0;
    constexpr uint8_t ResetDevice = 1;
    constexpr uint8_t RequestCommsStatusVariables = 2;
    constexpr uint8_t ClearCommsStatusVariables = 3;
    constexpr uint8_t RequestCommsRevision = 4;
    constexpr uint8_t Busy = 6;
    constexpr uint8_t RequestServiceStatus = 104;
    constexpr uint8_t DataStream = 105;
    constexpr uint8_t RequestEscrowStatus = 106;
    constexpr uint8_t OperateEscrow = 107;
    constexpr uint8_t RequestEncryptedMonetaryId = 108;
    constexpr uint8_t RequestEncryptedHopperStatus = 109;
    constexpr uint8_t SwitchEncryptionKey = 110;
    constexpr uint8_t RequestEncryptionSupport = 111;
    constexpr uint8_t ReadEncryptedEvents = 112;
    constexpr uint8_t SwitchBaudRate = 113;
    constexpr uint8_t RequestUsbId = 114;
    constexpr uint8_t RequestRealTimeClock = 115;
    constexpr uint8_t ModifyRealTimeClock = 116;
    constexpr uint8_t RequestCashboxValue = 117;
    constexpr uint8_t ModifyCashboxValue = 118;
    constexpr uint8_t RequestHopperBalance = 119;
    constexpr uint8_t ModifyHopperBalance = 120;
    constexpr uint8_t PurgeHopper = 121;
    constexpr uint8_t RequestErrorStatus = 122;
    constexpr uint8_t RequestActivityRegister = 123;
    constexpr uint8_t VerifyMoneyOut = 124;
    constexpr uint8_t PayMoneyOut = 125;
    constexpr uint8_t ClearMoneyCounter = 126;
    constexpr uint8_t RequestMoneyOut = 127;
    constexpr uint8_t RequestMoneyIn = 128;
    constexpr uint8_t ReadBarcodeData = 129;
    constexpr uint8_t RequestIndexedHopperDispenseCount = 130;
    constexpr uint8_t RequestHopperCoinValue = 131;
    constexpr uint8_t EmergencyStopValue = 132;
    constexpr uint8_t RequestHopperPollingValue = 133;
    constexpr uint8_t DispenseHopperValue = 134;
    constexpr uint8_t SetAcceptLimit = 135;
    constexpr uint8_t StoreEncryptionCode = 136;
    constexpr uint8_t SwitchEncryptionCode = 137;
    constexpr uint8_t FinishFirmwareUpgrade = 138;
    constexpr uint8_t BeginFirmwareUpgrade = 139;
    constexpr uint8_t UploadFirmware = 140;
    constexpr uint8_t RequestFirmwareUpgradeCapability = 141;
    constexpr uint8_t FinishBillTableUpgrade = 142;
    constexpr uint8_t BeginBillTableUpgrade = 143;
    constexpr uint8_t UploadBillTables = 144;
    constexpr uint8_t RequestCurrencyRevision = 145;
    constexpr uint8_t OperateBidirectionalMotors = 146;
    constexpr uint8_t PerformStackerCycle = 147;
    constexpr uint8_t ReadOptoVoltages = 148;
    constexpr uint8_t RequestIndividualErrorCounter = 149;
    constexpr uint8_t RequestIndividualAcceptCounter = 150;
    constexpr uint8_t TestLamps = 151;
    constexpr uint8_t RequestBillOperatingMode = 152;
    constexpr uint8_t ModifyBillOperatingMode = 153;
    constexpr uint8_t RouteBill = 154;
    constexpr uint8_t RequestBillPosition = 155;
    constexpr uint8_t RequestCountryScalingFactor = 156;
    constexpr uint8_t RequestBillId = 157;
    constexpr uint8_t ModifyBillId = 158;
    constexpr uint8_t ReadBufferedBillEvents = 159;
    constexpr uint8_t RequestCipherKey = 160;
    constexpr uint8_t PumpRng = 161;
    constexpr uint8_t ModifiyInhibitAndOverrideRegisters = 162;
    constexpr uint8_t TestHopper = 163;
    constexpr uint8_t EnableHopper = 164;
    constexpr uint8_t ModifyVariableSet = 165;
    constexpr uint8_t RequestHopperStatus = 166;
    constexpr uint8_t DispenseHopperCoins = 167;
    constexpr uint8_t RequestHopperDispenseCount = 168;
    constexpr uint8_t RequestAddressMode = 169;
    constexpr uint8_t RequestBaseYear = 170;
    constexpr uint8_t RequestHopperCoin = 171;
    constexpr uint8_t EmergencyStop = 172;
    constexpr uint8_t RequestThermistorReading = 173;
    constexpr uint8_t RequestPayoutFloat = 174;
    constexpr uint8_t ModifyPayoutFloat = 175;
    constexpr uint8_t RequestAlarmCounter = 176;
    constexpr uint8_t HandheldFunction = 177;
    constexpr uint8_t RequestBankSelect = 178;
    constexpr uint8_t ModifyBankSelect = 179;
    constexpr uint8_t RequestSecuritySetting = 180;
    constexpr uint8_t ModifySecuritySetting = 181;
    constexpr uint8_t DownloadCalibrationInfo = 182;
    constexpr uint8_t UploadWindowData = 183;
    constexpr uint8_t RequestCoinId = 184;
    constexpr uint8_t ModifyCoinId = 185;
    constexpr uint8_t RequestPayoutCapacity = 186;
    constexpr uint8_t ModifyPayoutCapacity = 187;
    constexpr uint8_t RequestDefaultSorterPath = 188;
    constexpr uint8_t ModifyDefaultSorterPath = 189;
    constexpr uint8_t KeypadControl = 191;
    constexpr uint8_t RequestBuildCode = 192;
    constexpr uint8_t RequestFraudCounter = 193;
    constexpr uint8_t RequestRejectCounter = 194;
    constexpr uint8_t RequestLastModificationDate = 195;
    constexpr uint8_t RequestCreationDate = 196;
    constexpr uint8_t CalculateRomChecksum = 197;
    constexpr uint8_t CountersToEeprom = 198;
    constexpr uint8_t ConfigurationToEeprom = 199;
    constexpr uint8_t AcmiUnencryptedProductId = 200;
    constexpr uint8_t RequestTeachStatus = 201;
    constexpr uint8_t TeachModeControl = 202;
    constexpr uint8_t DisplayControl = 203;
    constexpr uint8_t MeterControl = 204;
    constexpr uint8_t RequestPayoutAbsoluteCount = 207;
    constexpr uint8_t ModifyPayoutAbsoluteCount = 208;
    constexpr uint8_t RequestSorterPaths = 209;
    constexpr uint8_t ModifySorterPaths = 210;
    constexpr uint8_t PowerManagementControl = 211;
    constexpr uint8_t RequestCoinPosition = 212;
    constexpr uint8_t RequestOptionFlags = 213;
    constexpr uint8_t WriteDataBlock = 214;
    constexpr uint8_t ReadDataBlock = 215;
    constexpr uint8_t RequestDataStorageAvailability = 216;
    constexpr uint8_t RequestPayoutHighLowStatus = 217;
    constexpr uint8_t EnterPinNumber = 218;
    constexpr uint8_t EnterNewPinNumber = 219;
    constexpr uint8_t AcmiEncryptedData = 220;
    constexpr uint8_t RequestSorterOverrideStatus = 221;
    constexpr uint8_t ModifySorterOverrideStatus = 222;
    constexpr uint8_t ModifyEncryptedInhibitAndOverrideRegisters = 223;
    constexpr uint8_t RequestEncryptedProductId = 224;
    constexpr uint8_t RequestAcceptCounter = 225;
    constexpr uint8_t RequestInsertionCounter = 226;
    constexpr uint8_t RequestMasterInhibitStatus = 227;
    constexpr uint8_t ModifyMasterInhibitStatus = 228;
    constexpr uint8_t ReadBufferedCreditOrErrorCodes = 229;
    constexpr uint8_t RequestInhibitStatus = 230;
    constexpr uint8_t ModifyInhibitStatus = 231;
    constexpr uint8_t PerformSelfCheck = 232;
    constexpr uint8_t LatchOutputLines = 233;
    constexpr uint8_t SendDhPublicKey = 234;
    constexpr uint8_t ReadDhPublicKey = 235;
    constexpr uint8_t ReadOptoStates = 236;
    constexpr uint8_t ReadInputLines = 237;
    constexpr uint8_t TestOutputLines = 238;
    constexpr uint8_t OperateMotors = 239;
    constexpr uint8_t TestSolenoids = 240;
    constexpr uint8_t RequestSoftwareRevision = 241;
    constexpr uint8_t RequestSerialNumber = 242;
    constexpr uint8_t RequestDatabaseVersion = 243;
    constexpr uint8_t RequestProductCode = 244;
    constexpr uint8_t RequestEquipmentCategoryId = 245;
    constexpr uint8_t RequestManufacturerId = 246;
    constexpr uint8_t RequestVariableSet = 247;
    constexpr uint8_t RequestStatus = 248;
    constexpr uint8_t RequestPollingPriority = 249;
    constexpr uint8_t AddressRandom = 250;
    constexpr uint8_t AddressChange = 251;
    constexpr uint8_t AddressClash = 252;
    constexpr uint8_t AddressPoll = 253;
    constexpr uint8_t SimplePoll = 254;
    constexpr uint8_t FactorySetupAndTest = 255;
}

// Named aliases for headers you actually use
inline std::string CctalkHeadersGetDisplayableName(uint8_t header)
{
    static std::map<uint8_t, std::string> name_map = {
        {CctalkHeaders::Ack, "Ready"},
        {CctalkHeaders::ResetDevice, "ResetDevice"},
        {CctalkHeaders::RequestCommsStatusVariables, "RequestCommsStatusVariables"},
        {CctalkHeaders::ClearCommsStatusVariables, "ClearCommsStatusVariables"},
        {CctalkHeaders::RequestCommsRevision, "RequestCommsRevision"},
        {CctalkHeaders::Busy, "Busy"},
        {CctalkHeaders::RequestServiceStatus, "RequestServiceStatus"},
        {CctalkHeaders::DataStream, "DataStream"},
        {CctalkHeaders::RequestEscrowStatus, "RequestEscrowStatus"},
        {CctalkHeaders::OperateEscrow, "OperateEscrow"},
        {CctalkHeaders::RequestEncryptedMonetaryId, "RequestEncryptedMonetaryId"},
        {CctalkHeaders::RequestEncryptedHopperStatus, "RequestEncryptedHopperStatus"},
        {CctalkHeaders::SwitchEncryptionKey, "SwitchEncryptionKey"},
        {CctalkHeaders::RequestEncryptionSupport, "RequestEncryptionSupport"},
        {CctalkHeaders::ReadEncryptedEvents, "eadEncryptedEvents"},
        {CctalkHeaders::SwitchBaudRate, "SwitchBaudRate"},
        {CctalkHeaders::RequestUsbId, "RequestUsbId"},
        {CctalkHeaders::RequestRealTimeClock, "RequestRealTimeClock"},
        {CctalkHeaders::ModifyRealTimeClock, "ModifyRealTimeClock"},
        {CctalkHeaders::RequestCashboxValue, "RequestCashboxValue"},
        {CctalkHeaders::ModifyCashboxValue, "ModifyCashboxValue"},
        {CctalkHeaders::RequestHopperBalance, "RequestHopperBalance"},
        {CctalkHeaders::ModifyHopperBalance, "ModifyHopperBalance"},
        {CctalkHeaders::PurgeHopper, "PurgeHopper"},
        {CctalkHeaders::RequestErrorStatus, "RequestErrorStatus"},
        {CctalkHeaders::RequestActivityRegister, "RequestActivityRegister"},
        {CctalkHeaders::VerifyMoneyOut, "VerifyMoneyOut"},
        {CctalkHeaders::PayMoneyOut, "PayMoneyOut"},
        {CctalkHeaders::ClearMoneyCounter, "ClearMoneyCounter"},
        {CctalkHeaders::RequestMoneyOut, "RequestMoneyOut"},
        {CctalkHeaders::RequestMoneyIn, "RequestMoneyIn"},
        {CctalkHeaders::ReadBarcodeData, "ReadBarcodeData"},
        {
            CctalkHeaders::RequestIndexedHopperDispenseCount,
            "RequestIndexedHopperDispenseCount"
        },
        {CctalkHeaders::RequestHopperCoinValue, "RequestHopperCoinValue"},
        {CctalkHeaders::EmergencyStopValue, "EmergencyStopValue"},
        {CctalkHeaders::RequestHopperPollingValue, "RequestHopperPollingValue"},
        {CctalkHeaders::DispenseHopperValue, "DispenseHopperValue"},
        {CctalkHeaders::SetAcceptLimit, "SetAcceptLimit"},
        {CctalkHeaders::StoreEncryptionCode, "StoreEncryptionCode"},
        {CctalkHeaders::SwitchEncryptionCode, "SwitchEncryptionCode"},
        {CctalkHeaders::FinishFirmwareUpgrade, "FinishFirmwareUpgrade"},
        {CctalkHeaders::BeginFirmwareUpgrade, "BeginFirmwareUpgrade"},
        {CctalkHeaders::UploadFirmware, "UploadFirmware"},
        {
            CctalkHeaders::RequestFirmwareUpgradeCapability,
            "RequestFirmwareUpgradeCapability"
        },
        {CctalkHeaders::FinishBillTableUpgrade, "FinishBillTableUpgrade"},
        {CctalkHeaders::BeginBillTableUpgrade, "BeginBillTableUpgrade"},
        {CctalkHeaders::UploadBillTables, "UploadBillTables"},
        {CctalkHeaders::RequestCurrencyRevision, "RequestCurrencyRevision"},
        {CctalkHeaders::OperateBidirectionalMotors, "OperateBidirectionalMotors"},
        {CctalkHeaders::PerformStackerCycle, "PerformStackerCycle"},
        {CctalkHeaders::ReadOptoVoltages, "ReadOptoVoltages"},
        {
            CctalkHeaders::RequestIndividualErrorCounter,
            "RequestIndividualErrorCounter"
        },
        {
            CctalkHeaders::RequestIndividualAcceptCounter,
            "RequestIndividualAcceptCounter"
        },
        {CctalkHeaders::TestLamps, "TestLamps"},
        {CctalkHeaders::RequestBillOperatingMode, "RequestBillOperatingMode"},
        {CctalkHeaders::ModifyBillOperatingMode, "ModifyBillOperatingMode"},
        {CctalkHeaders::RouteBill, "RouteBill"},
        {CctalkHeaders::RequestBillPosition, "RequestBillPosition"},
        {CctalkHeaders::RequestCountryScalingFactor, "RequestCountryScalingFactor"},
        {CctalkHeaders::RequestBillId, "RequestBillId"},
        {CctalkHeaders::ModifyBillId, "ModifyBillId"},
        {CctalkHeaders::ReadBufferedBillEvents, "ReadBufferedBillEvents"},
        {CctalkHeaders::RequestCipherKey, "RequestCipherKey"},
        {CctalkHeaders::PumpRng, "PumpRng"},
        {
            CctalkHeaders::ModifiyInhibitAndOverrideRegisters,
            "ModifiyInhibitAndOverrideRegisters"
        },
        {CctalkHeaders::TestHopper, "TestHopper"},
        {CctalkHeaders::EnableHopper, "EnableHopper"},
        {CctalkHeaders::ModifyVariableSet, "ModifyVariableSet"},
        {CctalkHeaders::RequestHopperStatus, "RequestHopperStatus"},
        {CctalkHeaders::DispenseHopperCoins, "DispenseHopperCoins"},
        {CctalkHeaders::RequestHopperDispenseCount, "RequestHopperDispenseCount"},
        {CctalkHeaders::RequestAddressMode, "RequestAddressMode"},
        {CctalkHeaders::RequestBaseYear, "RequestBaseYear"},
        {CctalkHeaders::RequestHopperCoin, "RequestHopperCoin"},
        {CctalkHeaders::EmergencyStop, "EmergencyStop"},
        {CctalkHeaders::RequestThermistorReading, "RequestThermistorReading"},
        {CctalkHeaders::RequestPayoutFloat, "RequestPayoutFloat"},
        {CctalkHeaders::ModifyPayoutFloat, "ModifyPayoutFloat"},
        {CctalkHeaders::RequestAlarmCounter, "RequestAlarmCounter"},
        {CctalkHeaders::HandheldFunction, "HandheldFunction"},
        {CctalkHeaders::RequestBankSelect, "RequestBankSelect"},
        {CctalkHeaders::ModifyBankSelect, "ModifyBankSelect"},
        {CctalkHeaders::RequestSecuritySetting, "RequestSecuritySetting"},
        {CctalkHeaders::ModifySecuritySetting, "ModifySecuritySetting"},
        {CctalkHeaders::DownloadCalibrationInfo, "DownloadCalibrationInfo"},
        {CctalkHeaders::UploadWindowData, "UploadWindowData"},
        {CctalkHeaders::RequestCoinId, "RequestCoinId"},
        {CctalkHeaders::ModifyCoinId, "ModifyCoinId"},
        {CctalkHeaders::RequestPayoutCapacity, "RequestPayoutCapacity"},
        {CctalkHeaders::ModifyPayoutCapacity, "ModifyPayoutCapacity"},
        {CctalkHeaders::RequestDefaultSorterPath, "RequestDefaultSorterPath"},
        {CctalkHeaders::ModifyDefaultSorterPath, "ModifyDefaultSorterPath"},
        {CctalkHeaders::KeypadControl, "KeypadControl"},
        {CctalkHeaders::RequestBuildCode, "RequestBuildCode"},
        {CctalkHeaders::RequestFraudCounter, "RequestFraudCounter"},
        {CctalkHeaders::RequestRejectCounter, "RequestRejectCounter"},
        {CctalkHeaders::RequestLastModificationDate, "RequestLastModificationDate"},
        {CctalkHeaders::RequestCreationDate, "RequestCreationDate"},
        {CctalkHeaders::CalculateRomChecksum, "CalculateRomChecksum"},
        {CctalkHeaders::CountersToEeprom, "CountersToEeprom"},
        {CctalkHeaders::ConfigurationToEeprom, "ConfigurationToEeprom"},
        {CctalkHeaders::AcmiUnencryptedProductId, "AcmiUnencryptedProductId"},
        {CctalkHeaders::RequestTeachStatus, "RequestTeachStatus"},
        {CctalkHeaders::TeachModeControl, "TeachModeControl"},
        {CctalkHeaders::DisplayControl, "DisplayControl"},
        {CctalkHeaders::MeterControl, "MeterControl"},
        {CctalkHeaders::RequestPayoutAbsoluteCount, "RequestPayoutAbsoluteCount"},
        {CctalkHeaders::ModifyPayoutAbsoluteCount, "ModifyPayoutAbsoluteCount"},
        {CctalkHeaders::RequestSorterPaths, "RequestSorterPaths"},
        {CctalkHeaders::ModifySorterPaths, "ModifySorterPaths"},
        {CctalkHeaders::PowerManagementControl, "PowerManagementControl"},
        {CctalkHeaders::RequestCoinPosition, "RequestCoinPosition"},
        {CctalkHeaders::RequestOptionFlags, "RequestOptionFlags"},
        {CctalkHeaders::WriteDataBlock, "WriteDataBlock"},
        {CctalkHeaders::ReadDataBlock, "ReadDataBlock"},
        {
            CctalkHeaders::RequestDataStorageAvailability,
            "RequestDataStorageAvailability"
        },
        {CctalkHeaders::RequestPayoutHighLowStatus, "RequestPayoutHighLowStatus"},
        {CctalkHeaders::EnterPinNumber, "EnterPinNumber"},
        {CctalkHeaders::EnterNewPinNumber, "EnterNewPinNumber"},
        {CctalkHeaders::AcmiEncryptedData, "AcmiEncryptedData"},
        {CctalkHeaders::RequestSorterOverrideStatus, "RequestSorterOverrideStatus"},
        {CctalkHeaders::ModifySorterOverrideStatus, "ModifySorterOverrideStatus"},
        {
            CctalkHeaders::ModifyEncryptedInhibitAndOverrideRegisters,
            "ModifyEncryptedInhibitAndOverrideRegisters"
        },
        {CctalkHeaders::RequestEncryptedProductId, "RequestEncryptedProductId"},
        {CctalkHeaders::RequestAcceptCounter, "RequestAcceptCounter"},
        {CctalkHeaders::RequestInsertionCounter, "RequestInsertionCounter"},
        {CctalkHeaders::RequestMasterInhibitStatus, "RequestMasterInhibitStatus"},
        {CctalkHeaders::ModifyMasterInhibitStatus, "ModifyMasterInhibitStatus"},
        {
            CctalkHeaders::ReadBufferedCreditOrErrorCodes,
            "ReadBufferedCreditOrErrorCodes"
        },
        {CctalkHeaders::RequestInhibitStatus, "RequestInhibitStatus"},
        {CctalkHeaders::ModifyInhibitStatus, "ModifyInhibitStatus"},
        {CctalkHeaders::PerformSelfCheck, "PerformSelfCheck"},
        {CctalkHeaders::LatchOutputLines, "LatchOutputLines"},
        {CctalkHeaders::SendDhPublicKey, "SendDhPublicKey"},
        {CctalkHeaders::ReadDhPublicKey, "ReadDhPublicKey"},
        {CctalkHeaders::ReadOptoStates, "ReadOptoStates"},
        {CctalkHeaders::ReadInputLines, "ReadInputLines"},
        {CctalkHeaders::TestOutputLines, "TestOutputLines"},
        {CctalkHeaders::OperateMotors, "OperateMotors"},
        {CctalkHeaders::TestSolenoids, "TestSolenoids"},
        {CctalkHeaders::RequestSoftwareRevision, "RequestSoftwareRevision"},
        {CctalkHeaders::RequestSerialNumber, "RequestSerialNumber"},
        {CctalkHeaders::RequestDatabaseVersion, "RequestDatabaseVersion"},
        {CctalkHeaders::RequestProductCode, "RequestProductCode"},
        {CctalkHeaders::RequestEquipmentCategoryId, "RequestEquipmentCategoryId"},
        {CctalkHeaders::RequestManufacturerId, "RequestManufacturerId"},
        {CctalkHeaders::RequestVariableSet, "RequestVariableSet"},
        {CctalkHeaders::RequestStatus, "RequestStatus"},
        {CctalkHeaders::RequestPollingPriority, "RequestPollingPriority"},
        {CctalkHeaders::AddressRandom, "AddressRandom"},
        {CctalkHeaders::AddressChange, "AddressChange"},
        {CctalkHeaders::AddressClash, "AddressClash"},
        {CctalkHeaders::AddressPoll, "AddressPoll"},
        {CctalkHeaders::SimplePoll, "SimplePoll"},
        {CctalkHeaders::FactorySetupAndTest, "FactorySetupAndTest"},
    };

    try
    {
        return name_map.at(header);
    }
    catch (const std::exception& e)
    {
        return "Unknown";
    }
}
