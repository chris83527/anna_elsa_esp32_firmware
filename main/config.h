/*
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file config.h
 *
 * ESP-IDF driver for Holtek HT16K33 I2C LED Matrix driver chip
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <string>

#include "driver/gpio.h"
#include "ht16k33.h"

#define I2C_FREQ_HZ 400000

// Running LED
#define CPU_LED_GPIO GPIO_NUM_19

// ccTalk
#define CCTALK_GPIO_RX GPIO_NUM_16
#define CCTALK_GPIO_TX GPIO_NUM_17


// I2C
#define GPIO_I2C_SDA GPIO_NUM_21
#define GPIO_I2C_SCL GPIO_NUM_22

// LEDs
#define LED_GPIO GPIO_NUM_18
#define LED_COUNT 61

// VFD Display
#define MD_OE GPIO_NUM_4
#define MD_DATA GPIO_NUM_33
#define MD_CLK GPIO_NUM_5
#define MD_STROBE GPIO_NUM_32

// I2S Audio
#define AUDIO_MCLK GPIO_NUM_0
#define AUDIO_PDWN GPIO_NUM_2
#define AUDIO_LRCLK GPIO_NUM_25
#define AUDIO_SCLK GPIO_NUM_26
#define AUDIO_DOUT GPIO_NUM_27

// reels
#define GPIO_MOTOR_PWM_EN GPIO_NUM_23


// Buttons and button lamps (interfaced via MCP23017)
#define BTN_DOOR        (7)
#define BTN_START       (5)
#define BTN_COLLECT     (4)
#define BTN_HOLD_LO     (3)
#define BTN_HOLD        (2)
#define BTN_HOLD_HI     (1)
#define BTN_TRANSFER    (0)

#define LMP_START       (LED_COUNT + 0)
#define LMP_COLLECT     (LED_COUNT + 1)
#define LMP_HOLD_LO     (LED_COUNT + 2)
#define LMP_HOLD        (LED_COUNT + 3)
#define LMP_HOLD_HI     (LED_COUNT + 4)
#define LMP_TRANSFER    (LED_COUNT + 5)

// WS2128B LEDs
#define REEL_LAMP_L1                            0
#define REEL_LAMP_L2                            1
#define REEL_LAMP_L3                            2
#define REEL_LAMP_C1                            3
#define REEL_LAMP_C2                            4
#define REEL_LAMP_C3                            5
#define REEL_LAMP_R1                            6
#define REEL_LAMP_R2                            7
#define REEL_LAMP_R3                            8
#define LAMP_NUDGE_5        9
#define LAMP_NUDGE_4        10
#define LAMP_NUDGE_3        11
#define LAMP_NUDGE_2        12
#define LAMP_NUDGE_1        13
#define LAMP_PRIZE_PALACE   14
#define LAMP_PRIZE_ANNA     15
#define LAMP_PRIZE_ELSA     16
#define LAMP_PRIZE_CHRISTOPH    17
#define LAMP_PRIZE_SVEN     18
#define LAMP_PRIZE_OLAF_3   19
#define LAMP_PRIZE_OLAF_ANY 20
#define LAMP_PRIZE_HANS     21
#define LAMP_PRIZE_20_CENT  22
#define LAMP_PRIZE_40_CENT  23
#define LAMP_PRIZE_80_CENT  24
#define LAMP_PRIZE_120_CENT 25
#define LAMP_PRIZE_160_CENT 26
#define LAMP_PRIZE_200_CENT 27
#define LAMP_PRIZE_300_CENT 28
#define LAMP_PRIZE_400_CENT 29
#define LAMP_HI                                 30
#define LAMP_lO                                 31
#define LAMP_MATRIX_SHUFFLE_1_1                 32
#define LAMP_MATRIX_FREE_SPIN_1_2               33
#define LAMP_MATRIX_DOUBLE_MONEY_1_3            34
#define LAMP_MATRIX_PALACE_2_3                  35
#define LAMP_MATRIX_LOSE_2_2             36
#define LAMP_MATRIX_PALACE_2_1              37
#define LAMP_MATRIX_FREE_SPIN_3_1               38
#define LAMP_MATRIX_SHUFFLE_3_2                 39
#define LAMP_MATRIX_LOSE_3_3                    40
#define LAMP_MATRIX_FREE_SPIN_4_3               41
#define LAMP_MATRIX_HI_LO_4_2                   42
#define LAMP_MATRIX_PALACE_4_1                  43
#define LAMP_TRAIL_20_CENT        44
#define LAMP_TRAIL_40_CENT        45
#define LAMP_TRAIL_60_CENT        46
#define LAMP_TRAIL_80_CENT        47
#define LAMP_TRAIL_ONE_EURO       48
#define LAMP_TRAIL_ONE_TWENTY     49
#define LAMP_TRAIL_ONE_FOURTY     50
#define LAMP_TRAIL_ONE_SIXTY      51
#define LAMP_TRAIL_ONE_EIGHTY     52
#define LAMP_TRAIL_TWO_EURO       53
#define LAMP_TRAIL_TWO_FOURTY     54
#define LAMP_TRAIL_TWO_EIGHTY     55
#define LAMP_TRAIL_THREE_FOURTY   56
#define LAMP_TRAIL_THREE_EIGHTY   57
#define LAMP_TRAIL_FOUR_TWENTY    58
#define LAMP_TRAIL_FOUR_SIXTY     59
#define LAMP_TRAIL_FIVE_EURO      60

#define CREDIT_DISPLAY_ADDRESS      HT16K33_ADDR_BASE      // 0x70
#define BANK_DISPLAY_ADDRESS        HT16K33_ADDR_BASE + 1  // 0x71
#define MOVES_DISPLAY_ADDRESS       HT16K33_ADDR_BASE + 2  // 0x72

#define REEL_LEFT_I2C_ADDRESS       MCP23008_I2C_ADDR_BASE // 0x21
#define REEL_CENTRE_I2C_ADDRESS     MCP23008_I2C_ADDR_BASE + 1 // 0x22
#define REEL_RIGHT_I2C_ADDRESS      MCP23008_I2C_ADDR_BASE + 2 // 0x23

#define BUTTONS_I2C_ADDRESS         MCP23X17_ADDR_BASE + 7 // 0x27


#endif /* CONFIG_H */