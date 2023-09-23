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
#include "pca9629a.h"
#include "mcp23x17.h"

#define I2C_FREQ_HZ 100000

// Running LED
#define CPU_LED_GPIO GPIO_NUM_19

// ccTalk
#define CCTALK_GPIO_RX GPIO_NUM_16
#define CCTALK_GPIO_TX GPIO_NUM_17
#define CCTALK_UART UART_NUM_1

// I2C
#define GPIO_I2C_SDA GPIO_NUM_21
#define GPIO_I2C_SCL GPIO_NUM_22

// LEDs
#define LED_GPIO GPIO_NUM_18
#define LED_COUNT (61)

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

#define BTN_DOOR_MASK_BIT   (1<<BTN_DOOR)
#define BTN_START_MASK_BIT  (1<<BTN_START)
#define BTN_COLLECT_MASK_BIT (1<<BTN_COLLECT)
#define BTN_HOLD_LO_MASK_BIT (1<<BTN_HOLD_LO)
#define BTN_HOLD_MASK_BIT (1<<BTN_HOLD)
#define BTN_HOLD_HI_MASK_BIT (1<<BTN_HOLD_HI)
#define BTN_TRANSFER_MASK_BIT (1<<BTN_TRANSFER)


#define CREDIT_DISPLAY_ADDRESS      HT16K33_ADDR_BASE      // 0x70
#define BANK_DISPLAY_ADDRESS        HT16K33_ADDR_BASE + 1  // 0x71
#define MOVES_DISPLAY_ADDRESS       HT16K33_ADDR_BASE + 2  // 0x72

#define REEL_LEFT_I2C_ADDRESS       PCA9629A_I2C_ADDR_BASE // 0x40
#define REEL_CENTRE_I2C_ADDRESS     PCA9629A_I2C_ADDR_BASE + 2 // 0x42
#define REEL_RIGHT_I2C_ADDRESS      PCA9629A_I2C_ADDR_BASE + 4 // 0x44

#define BUTTONS_I2C_ADDRESS         MCP23X17_ADDR_BASE + 7 // 0x27


#endif /* CONFIG_H */
