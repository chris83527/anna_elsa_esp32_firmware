//
// Created by chris on 06.04.26.
//
#pragma once
#include <cstdint>

enum class HopperStatusRegister1 : uint8_t
{
    OK = (1 << 0),
    MAX_CURRENT_EXCEEDED = (1 << 1),
    PAYOUT_TIMEOUT_OCCURED = (1 << 2),
    MOTOR_REVERSED_DURING_LAST_PAYOUT = (1 << 3),
    OPTO_FRAUD_ATTEMPT_PATH_BLOCKED_DURING_IDLE = (1 << 4),
    OPTO_FRAUD_ATTEMPT_SHORT_CIRCUIT_DURING_IDLE = (1 << 5),
    POWER_UP_DETECTED = (1 << 6),
    PAYOUT_DISABLED = (1 << 7),
};

enum class HopperStatusRegister2 : uint8_t
{
    OPTO_FRAUD_ATTEMPT_SHORT_CIRCUIT_DURING_PAYOUT = (1 << 0),
    SINGLE_COIN_PAYOUT_MODE = (1 << 1),
    USE_OTHER_HOPPER_FOR_CHANGE = (1 << 2),
    OPTO_FRAUD_ATTEMPT_FINGER_SLIDER_MISMATCH = (1 << 3),
    MOTOR_REVERSE_LIMIT_REACHED = (1 << 4),
    INDUCTIVE_COIL_FAULT = (1 << 5),
    NV_MEMORY_CHECKSUM_ERROR = (1 << 6),
    PIN_NUMBER_MECHANISM = (1 << 7),
};

enum class HopperStatusRegister3 : uint8_t
{
    POWER_DOWN_DURING_PAYOUT = (1 << 0),
    UNKNOWN_COIN_TYPE_PAID = (1 << 1),
    PIN_NUMBER_INCORRECT = (1 << 2),
    INCORRECT_CIPHER_KEY = (1 << 3)
};

struct TestHopperStatus
{
    uint8_t statusRegister1;
    uint8_t statusRegister2;
    uint8_t statusRegister3;
};


struct HopperPayoutStatus
{
    uint8_t eventCounter = 0;
    uint8_t coinsRemaining = 0;
    uint8_t coinsPaid = 0;
    uint8_t coinsUnpaid = 0;
};
