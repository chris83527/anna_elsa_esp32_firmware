//
// Created by chris on 06.04.26.
//
#pragma once
#include <cstdint>

enum class HopperStatusFlags : uint8_t {
    OK               = 0x00,
    Empty            = 0x01,
    Jammed           = 0x02,
    MotorRunning     = 0x04,
    MotorTimeout     = 0x08,
    SensorBlocked    = 0x10
};

struct HopperStatus {
    uint8_t raw_status;
    bool empty;
    bool jammed;
    bool motor_running;
    bool motor_timeout;
    bool sensor_blocked;
};
