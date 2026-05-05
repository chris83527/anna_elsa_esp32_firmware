//
// Created by chris on 26.04.26.
//

#pragma once

#include <cstdint>

namespace pca9629a
{
    /** name of the PCA9629 registers */

    constexpr uint8_t REG_MODE = 0x00; /**< Mode rgister */
    constexpr uint8_t REG_WDTOI = 0x01; /**< Watchdog time-out interval register */
    constexpr uint8_t REG_WDTCNTL = 0x02; /**< Watchdog control register */
    constexpr uint8_t REG_IO_CFG = 0x03; /**< I/O configuration register */
    constexpr uint8_t REG_INTMODE = 0x04; /**< Interrupt mode register */
    constexpr uint8_t REG_MSK = 0x05; /**< Mask interrupt register */
    constexpr uint8_t REG_INTSTAT = 0x06; /**< Interrupt status register */
    constexpr uint8_t REG_IP = 0x07; /**< Input port register */
    constexpr uint8_t REG_INT_MTR_ACT = 0x08; /**< Interrupt motor action control register */
    constexpr uint8_t REG_EXTRASTEPS0 = 0x09; /**< Count value for extra steps for INTP0 */
    constexpr uint8_t REG_EXTRASTEPS1 = 0x0a; /**< Count value for extra steps for INTP1 */
    constexpr uint8_t REG_OP_CFG_PHS = 0x0b; /**< Ouput port configuration and phase control register */
    constexpr uint8_t REG_OP_STAT_TO = 0x0c; /**< Output port state and time-out control register */
    constexpr uint8_t REG_RUCNTL = 0x0d; /**< Ramp up control register */
    constexpr uint8_t REG_RDCTNL = 0x0e; /**< Ramp down control register */
    constexpr uint8_t REG_PMA = 0x0f; /**< Perform multiple of actions control register */
    constexpr uint8_t REG_LOOPDLY_CW = 0x10; /**< Loopdelay time register */
    constexpr uint8_t REG_LOOPDLY_CCW = 0x11; /**< Loopdelay time register */
    constexpr uint8_t REG_CWSCOUNTL = 0x12; /**< Number of steps CW low byte */
    constexpr uint8_t REG_CWSCOUNTH = 0x13; /**< Number of steps CW high byte */
    constexpr uint8_t REG_CCWSCOUNTL = 0x14; /**< Number of steps CCW low byte */
    constexpr uint8_t REG_CCWSCOUNTH = 0x15; /**< Number of steps CCW high byte */
    constexpr uint8_t REG_CWPWL = 0x16; /**< Step pulse width for CW rotation low byte */
    constexpr uint8_t REG_CWPWH = 0x17; /**< Step pulse width for CW rotation high byte */
    constexpr uint8_t REG_CCWPWL = 0x18; /**< Step pulse width for CCW rotation low byte */
    constexpr uint8_t REG_CCWPWH = 0x19; /**< Step pulse width for CCW rotation high byte */
    constexpr uint8_t REG_MCNTL = 0x1a; /**< Control start/stop motor */
    constexpr uint8_t REG_SUBADR1 = 0x1b; /**< I2C-bus subaddress 1 */
    constexpr uint8_t REG_SUBADR2 = 0x1c; /**< I2C-bus subaddress 2 */
    constexpr uint8_t REG_SUBADR3 = 0x1d; /**< I2C-bus subaddress 3 */
    constexpr uint8_t REG_ALLCALLADR = 0x1e; /**< All call I2C-bus address */
    constexpr uint8_t REG_STEPCOUNT0 = 0x1f; /**< Step counter byte 0 */
    constexpr uint8_t REG_STEPCOUNT1 = 0x20; /**< Step counter byte 1 */
    constexpr uint8_t REG_STEPCOUNT2 = 0x21; /**< Step counter byte 2 */
    constexpr uint8_t REG_STEPCOUNT3 = 0x22; /**< Step counter byte 3 */


    namespace bit
    {
        // ---- MODE register bits (example; adapt to datasheet) ----
        constexpr uint8_t MODE_SLEEP = 1u << 4;
        constexpr uint8_t MODE_RESET = 1u << 7;

        // ---- Example: step mode / direction bits in the PCA9629A motor control register ----
        // Check the PCA9629A datasheet for the exact register name and bit positions.
        constexpr uint8_t STEP_MODE_MASK = 0b00000011;
        constexpr uint8_t STEP_MODE_WAVE = 0b00000000;
        constexpr uint8_t STEP_MODE_FULL = 0b00000001;
        constexpr uint8_t STEP_MODE_HALF = 0b00000010;

        constexpr uint8_t DIR_BIT = 1u << 2;

        // COMMAND register bits (names are from datasheet; addresses you fill)
        constexpr uint8_t CMD_START = 1u << 0;
        constexpr uint8_t CMD_STOP_RAMP = 1u << 1;
        constexpr uint8_t CMD_STOP_IMM = 1u << 2;
        constexpr uint8_t CMD_RESET_STEPS = 1u << 3;
        constexpr uint8_t CMD_SOFT_RESET = 1u << 4;

        // INTSTAT bits – check which bit indicates “motor running / action active”
        constexpr uint8_t INTSTAT_BUSY = 1u << 0;

        constexpr uint8_t MCNTL_BUSY = 0x80;
    } // namespace bit
} // namespace pca9629a
