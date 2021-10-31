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

// Running LED
#define CPU_LED_GPIO GPIO_NUM_19

// ccTalk
#define CCTALK_GPIO_TX GPIO_NUM_17
#define CCTALK_GPIO_RX GPIO_NUM_16

// I2C
#define GPIO_I2C_SDA GPIO_NUM_21
#define GPIO_I2C_SCL GPIO_NUM_22

// LEDs
#define LED_GPIO GPIO_NUM_18
#define LED_COUNT 45

// VFD Display
#define MD_OE GPIO_NUM_4
#define MD_DATA GPIO_NUM_0
#define MD_CLK GPIO_NUM_5
#define MD_STROBE GPIO_NUM_2

// I2S Audio
#define AUDIO_LRCLK GPIO_NUM_25
#define AUDIO_SCLK GPIO_NUM_26
#define AUDIO_DOUT GPIO_NUM_27

// Buttons and button lamps (interfaced via MCP23017)
#define BTN_DOOR        (1<<7)
#define BTN_START       (1<<5)
#define BTN_COLLECT     (1<<4)
#define BTN_HOLD_HI     (1<<3)
#define BTN_HOLD        (1<<2)
#define BTN_HOLD_LO     (1<<1)
#define BTN_TRANSFER    (1<<0)

#define LMP_START       (LED_COUNT + 1)
#define LMP_COLLECT     (LED_COUNT + 2)
#define LMP_HOLD_HI     (LED_COUNT + 3)
#define LMP_HOLD        (LED_COUNT + 4)
#define LMP_HOLD_LO     (LED_COUNT + 5)
#define LMP_TRANSFER    (LED_COUNT + 6)

// WS2128B LEDs
#define LAMP_HI                                 0
#define LAMP_lO                                 LAMP_HI + 1
#define LAMP_MATRIX_SHUFFLE_TOP_RIGHT           LAMP_HI + 2
#define LAMP_MATRIX_FREE_SPIN_CENTRE_RIGHT      LAMP_HI + 3
#define LAMP_MATRIX_DOUBLE_MONEY_BOTTOM_RIGHT   LAMP_HI + 4
#define LAMP_MATRIX_PALACE_BOTTOM_CENTRE        LAMP_HI + 5
#define LAMP_MATRIX_LOSE_CENTRE_CENTRE          LAMP_HI + 6
#define LAMP_MATRIX_PALACE_TOP_CENTRE           LAMP_HI + 7
#define LAMP_MATRIX_FREE_SPIN

#define LAMP_NUDGE_5        26
#define LAMP_NUDGE_4        25
#define LAMP_NUDGE_3        24
#define LAMP_NUDGE_2        23
#define LAMP_NUDGE_1        22


#define REEL_LAMP_L1                            0
#define REEL_LAMP_L2                            1
#define REEL_LAMP_L3                            2
#define REEL_LAMP_C1                            3
#define REEL_LAMP_C2                            4
#define REEL_LAMP_C3                            5
#define REEL_LAMP_R1                            6
#define REEL_LAMP_R2                            7
#define REEL_LAMP_R3                            8

// Audio files
#define SND_NOW_THATS_ICE   "nowthatsice.wav"
#define SND_LOSE            "lose.wav"
#define SND_LET_IT_GO       "letitgo.wav"
#define SND_THEYRE_TROLLES  "theyretrolls.wav"
#define SND_CANT_FEEL_LEGS  "cantfeellegs.wav"
#define SND_THATS_BETTER    "thatsbetter.wav"
#define SND_KERCHING        "kerching.wav"


#endif /* CONFIG_H */