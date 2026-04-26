#pragma once

enum class CctalkError
{
    OK = 0,
    Timeout,
    UartError,
    BadChecksum,
    UnexpectedSource,
    UnexpectedDestination,
    MalformedFrame,
    DeviceError,
    InvalidHeader,
    Unknown
};

/*
 * Fault code, as returned by PerformSelfCheck command.
 * Extra information may be returned in the second byte.
 * See spec part 3 table 3 and section 14.
 */
enum class CcFaultCode : uint8_t
{
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
