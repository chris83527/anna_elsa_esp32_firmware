/*
 * The MIT License
 *
 * Copyright 2023 chris.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/* 
 * File:   tas5731m_reg_cfg.h
 * Author: chris
 *
 * Created on January 13, 2023, 7:13 PM
 */

#ifndef TAS5731M_REG_CFG_H
#define TAS5731M_REG_CFG_H

#ifdef __cplusplus
extern "C" {
#endif

#define CFG_META_SWITCH (255)
#define CFG_META_DELAY  (254)
#define CFG_META_BURST  (253)
#define CFG_END_1       (0Xaa)
#define CFG_END_2       (0Xcc)
#define CFG_END_3       (0Xee)

typedef struct {
    uint8_t reg;
    uint8_t value;
} tas5731m_cfg_reg_t;

static const uint8_t tas5731m_volume[] = {
    0xff, 0x9f, 0x8f, 0x7f, 0x6f, 0x5f, 0x5c, 0x5a,
    0x58, 0x54, 0x50, 0x4c, 0x4a, 0x48, 0x44, 0x40,
    0x3d, 0x3b, 0x39, 0x37, 0x35
};



#ifdef __cplusplus
}
#endif

#endif /* TAS5731M_REG_CFG_H */

