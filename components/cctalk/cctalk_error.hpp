#pragma once

enum class CctalkError {
    OK = 0,
    Timeout,
    UartError,
    BadChecksum,
    UnexpectedSource,
    UnexpectedDestination,
    MalformedFrame,
    DeviceError,
    Unknown
};

