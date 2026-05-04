//
// Created by chris on 03.05.26.
//

#ifndef ANNA_ELSA_ESP32_TAS5731M_REGS_HPP
#define ANNA_ELSA_ESP32_TAS5731M_REGS_HPP

#include <cstdint>

namespace TAS5731M_REGISTERS
{
    constexpr uint8_t CLOCK_CONTROL_REGISTER = 0x00;
    constexpr uint8_t DEVICE_ID_REGISTER = 0x01;
    constexpr uint8_t ERROR_STATUS_REGISTER = 0x02;
    constexpr uint8_t SYSTEM_CONTROL_REGISTER_1 = 0x03;
    constexpr uint8_t SERIAL_DATA_INTERFACE_REGISTER = 0x04;
    constexpr uint8_t SYSTEM_CONTROL_REGISTER_2 = 0x05;
    constexpr uint8_t SOFT_MUTE_REGISTER = 0x06;
    constexpr uint8_t MASTER_VOLUME = 0x07;
    constexpr uint8_t CHANNEL_1_VOL = 0x08;
    constexpr uint8_t CHANNEL_2_VOL = 0x09;
    constexpr uint8_t CHANNEL_3_VOL = 0x0a;
    constexpr uint8_t VOLUME_CONFIGURATION_REGISTER = 0x0e;
    constexpr uint8_t MODULATION_LIMIT_REGISTER = 0x10;
    constexpr uint8_t IC_DELAY_CHANNEL_1 = 0x11;
    constexpr uint8_t IC_DELAY_CHANNEL_2 = 0x12;
    constexpr uint8_t IC_DELAY_CHANNEL_3 = 0x13;
    constexpr uint8_t IC_DELAY_CHANNEL_4 = 0x14;
    constexpr uint8_t PWM_CHANNEL_SHUTDOWN_GROUP_REGISTER = 0x19;
    constexpr uint8_t START_STOP_PERIOD_REGISTER=0x1a;
    constexpr uint8_t OSCILLATOR_TRIM_REGISTER = 0x1b;
    constexpr uint8_t BKND_ERR_REGISTER = 0x1c;
    constexpr uint8_t INPUT_MUX_REGISTER = 0x20;
    constexpr uint8_t CH4_SOURCE_SELECT_REGISTER = 0x21;
    constexpr uint8_t PWM_MUX_REGISTER = 0x25;

    constexpr uint8_t MUTE_TIME_REG_ADDR = 0x51;
};

#endif //ANNA_ELSA_ESP32_TAS5731M_REGS_HPP
