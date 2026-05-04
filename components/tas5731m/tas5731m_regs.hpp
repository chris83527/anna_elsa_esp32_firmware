//
// Created by chris on 03.05.26.
//

#ifndef TAS5731M_REGS_HPP
#define TAS5731M_REGS_HPP

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

    // 20-byte biquad registers
    constexpr uint8_t CHANNEL_1_BQ0_REGISTER = 0x29;
    constexpr uint8_t CHANNEL_1_BQ1_REGISTER = 0x2a;
    constexpr uint8_t CHANNEL_1_BQ2_REGISTER = 0x2b;
    constexpr uint8_t CHANNEL_1_BQ3_REGISTER = 0x2c;
    constexpr uint8_t CHANNEL_1_BQ4_REGISTER = 0x2d;
    constexpr uint8_t CHANNEL_1_BQ5_REGISTER = 0x2e;
    constexpr uint8_t CHANNEL_1_BQ6_REGISTER = 0x2f;
    constexpr uint8_t CHANNEL_1_BQ7_REGISTER = 0x58;
    constexpr uint8_t CHANNEL_1_BQ8_REGISTER = 0x59;
    constexpr uint8_t SUBCHANNEL_BQ_0_REGISTER = 0x5a;
    constexpr uint8_t SUBCHANNEL_BQ_1_REGISTER = 0x5b;

    constexpr uint8_t CHANNEL_2_BQ0_REGISTER = 0x30;
    constexpr uint8_t CHANNEL_2_BQ1_REGISTER = 0x31;
    constexpr uint8_t CHANNEL_2_BQ2_REGISTER = 0x32;
    constexpr uint8_t CHANNEL_2_BQ3_REGISTER = 0x33;
    constexpr uint8_t CHANNEL_2_BQ4_REGISTER = 0x34;
    constexpr uint8_t CHANNEL_2_BQ5_REGISTER = 0x35;
    constexpr uint8_t CHANNEL_2_BQ6_REGISTER = 0x36;
    constexpr uint8_t CHANNEL_2_BQ7_REGISTER = 0x5c;
    constexpr uint8_t CHANNEL_2_BQ8_REGISTER = 0x5d;
    constexpr uint8_t PSEUDO_CH2_REGISTER = 0x5e;


    // "ae" stands for ∝ of energy filter, "aa" stands for ∝ of attack filter and "ad" stands for ∝ of decay filter and 1- ∝ = ω
    // 8-byte registers
    constexpr uint8_t DRC1_AE_REGISTER = 0x3a;
    constexpr uint8_t DRC1_AA_REGISTER = 0x3b;
    constexpr uint8_t DRC1_AD_REGISTER = 0x3c;

    constexpr uint8_t DRC2_AE_REGISTER = 0x3d;
    constexpr uint8_t DRC2_AA_REGISTER = 0x3e;
    constexpr uint8_t DRC2_AD_REGISTER = 0x3f;

    // 4-byte registers
    constexpr uint8_t DRC1_T_REGISTER = 0x40;
    constexpr uint8_t DRC1_K_REGISTER = 0x41;
    constexpr uint8_t DRC1_O_REGISTER = 0x42;
    constexpr uint8_t DRC2_T_REGISTER = 0x43;
    constexpr uint8_t DRC2_K_REGISTER = 0x44;
    constexpr uint8_t DRC2_O_REGISTER = 0x45;
    constexpr uint8_t DRC_CONTROL_REGISTER = 0x46;

    // 4-byte register
    constexpr uint8_t BANK_SWITCH_CONTROL_REGISTER = 0x50;

    // 12-byte registers
    constexpr uint8_t CH_1_OUTPUT_MIXER_REGISTER = 0x51;
    constexpr uint8_t CH_2_OUTPUT_MIXER_REGISTER = 0x52;

    // 16-byte registers
    constexpr uint8_t CH_1_INPUT_MIXER_REGISTER = 0x53;
    constexpr uint8_t CH_2_INPUT_MIXER_REGISTER = 0x54;

    // 12-byte register
    constexpr uint8_t CH_3_INPUT_MIXER_REGISTER = 0x55;

    // 4-byte registers
    constexpr uint8_t OUTPUT_POST_SCALE_REGISTER = 0x56;
    constexpr uint8_t OUTPUT_PRE_SCALE_REGISTER = 0x56;

    // 8-bit registers
    constexpr uint8_t CHANNEL_4_SUBCHANNEL_OUTPUT_MIXER = 0x60;
    constexpr uint8_t CHANNEL_4_SUBCHANNEL_INPUT_MIXER = 0x61;

    // 4-byte registers
    constexpr uint8_t IDF_POST_SCALE_REGISTER = 0x62;
    constexpr uint8_t DEFICE_ADDRESS_ENABLE_REGISTER = 0xf8;
    constexpr uint8_t DEFICE_ADDRESS_UPDATE_REGISTER = 0xf9;




};

#endif //TAS5731M_REGS_HPP
