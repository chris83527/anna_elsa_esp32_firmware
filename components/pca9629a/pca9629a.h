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
 * File:   pca9629a.h
 * Author: chris
 *
 * Created on August 2, 2023, 7:12 PM
 */

#ifndef PCA9629A_H
#define PCA9629A_H

#include <stdint.h>

#include <thread>
#include <chrono>

#include "i2cdev.h"

#define PCA_9629A_DEFAULT_STEPS_PER_ROTATION  48
#define PCA9629A_I2C_ADDR_BASE        0x20
#define I2C_FREQ_HZ 100000

/** PCA9629A class
 *
 *  This is a driver code for the PCA9629A stepper motor controller.
 *  This class provides interface for PCA9629A operation and accessing its registers.
 *  Detail information is available here:
 *    https://www.nxp.com/docs/en/data-sheet/PCA9629A.pdf
 *    https://style.nxp.com/docs/en/application-note/AN11483.pdf
 *
 *  Example:
 *  @code
 *
 *  @endcode
 */
class PCA9629A {
public:

    /** name of the PCA9629 registers */
    typedef enum {
        REG_MODE = 0, /**< Mode rgister */
        REG_WDTOI, /**< Watchdog time-out interval register */
        REG_WDTCNTL, /**< Watchdog control register */
        REG_IO_CFG, /**< I/O configuration register */
        REG_INTMODE, /**< Interrupt mode register */
        REG_MSK, /**< Mask interrupt register */
        REG_INTSTAT, /**< Interrupt status register */
        REG_IP, /**< Input port register */
        REG_INT_MTR_ACT, /**< Interrupt motor action control register */
        REG_EXTRASTEPS0, /**< Count value for extra steps for INTP0 */
        REG_EXTRASTEPS1, /**< Count value for extra steps for INTP1 */
        REG_OP_CFG_PHS, /**< Ouput port configuration and phase control register */
        REG_OP_STAT_TO, /**< Output port state and time-out control register */
        REG_RUCNTL, /**< Ramp up control register */
        REG_RDCTNL, /**< Ramp down control register */
        REG_PMA, /**< Perform multiple of actions control register */
        REG_LOOPDLY_CW, /**< Loopdelay time register */
        REG_LOOPDLY_CCW, /**< Loopdelay time register */
        REG_CWSCOUNTL, /**< Number of steps CW low byte */
        REG_CWSCOUNTH, /**< Number of steps CW high byte */
        REG_CCWSCOUNTL, /**< Number of steps CCW low byte */
        REG_CCWSCOUNTH, /**< Number of steps CCW high byte */
        REG_CWPWL, /**< Step pulse width for CW rotation low byte */
        REG_CWPWH, /**< Step pulse width for CW rotation high byte */
        REG_CCWPWL, /**< Step pulse width for CCW rotation low byte */
        REG_CCWPWH, /**< Step pulse width for CCW rotation high byte */
        REG_MCNTL, /**< Control start/stop motor */
        REG_SUBADR1, /**< I2C-bus subaddress 1 */
        REG_SUBADR2, /**< I2C-bus subaddress 2 */
        REG_SUBADR3, /**< I2C-bus subaddress 3 */
        REG_ALLCALLADR, /**< All call I2C-bus address */
        REG_STEPCOUNT0, /**< Step counter byte 0 */
        REG_STEPCOUNT1, /**< Step counter byte 1 */
        REG_STEPCOUNT2, /**< Step counter byte 2 */
        REG_STEPCOUNT3, /**< Step counter byte 3 */
    } RegisterName;

private:


public:

    /** keyword to select direction of rotation */
    typedef enum {
        CW = 0, /**< Clockwise direction */
        CCW /**< ConterClockwise direction */
    } Direction;

    /** Create a PCA9629 instance connected to specified I2C pins with specified address
     *
     * @param I2C_sda I2C-bus SDA pin
     * @param I2C_scl I2C-bus SCL pin     
     * @param I2C_address I2C-bus address (default: 0x40)
     */
    PCA9629A(
            const i2c_port_t port,
            const gpio_num_t i2c_sda,
            const gpio_num_t i2c_scl,
            const uint8_t i2c_address = PCA9629A_I2C_ADDR_BASE,
            const uint32_t clock_speed = I2C_FREQ_HZ
            );
    
    ~PCA9629A();

    void initialise(void);
    
    /** Software reset
     *
     *  Performs software reset through I2C bus
     */
    esp_err_t software_reset(void);


    /** Motor start
     *
     *  Start command
     *  This function starts motor operation with hard-stop flag and rotation+step enabled, no repeat will be performed
     *  If custom start is required, use "write( PCA9629::MCNTL, 0xXX  )" to control each bits.
     *
     *  @param dir rotate direction ("CW" or "CCW")
     */
    void start(Direction dir, uint16_t steps, uint8_t repeats);

    /** Motor start (with home-position control)
     *
     *  Start command
     *  This function starts motor operation with hard-stop flag and rotation+step enabled, no repeat will be performed
     *  If custom start is required, use "write( PCA9629::MCNTL, 0xXX  )" to control each bits.
     *
     *  @param dir rotate direction ("CW" or "CCW")
     * @param steps
     * @param repeats 
     */
    void home(Direction dir);

    /** Motor stop
     *
     *  Stop command
     *
     */
    void stop(void);
    
    /**
     * Reads the MCNTL register and returns true if bit 7 is 0
     * @return 
     */
    bool isStopped(void);

    /** Register dump
     *
     *  Dumping all register data to serial console
     *
     */
    esp_err_t register_dump(void);

private:
    /** Initialize all registers
     *
     *  The initializing values are defined in the function
     */
    void init_registers(void);

    /** Initialize all registers
     *
     *  The initializing values are defined in the function
     */
    esp_err_t set_all_registers(uint8_t *a, uint8_t size);

    /** Write 1 byte data into a register
     *
     *  Setting 8 bits data into a register
     *
     *  @param register_name the register name: data writing into
     *  @param value 8 bits writing data
     */
    esp_err_t write(RegisterName register_name, const uint8_t value);

    /** Write 2 bytes data into a register
     *
     *  Setting 16 bits data into registers
     *
     *  @param register_name the register name: data writing into (it can be "SROTN_", "CWPW_", "CCWPW_", "CWRCOUNT_", "CCWSCOUNT_", "CWRCOUNT_" or "CCWRCOUNT_" )
     *  @param value 16 bits writing data
     */
    esp_err_t write16(RegisterName register_name, const uint16_t value);

    /** Read 1 byte data from a register
     *
     *  Setting data into a register
     *
     *  @param register_name the register name: data reading from
     *  @return read 8 bits data from the register
     */
    esp_err_t read(RegisterName register_name, uint8_t& result);

    /** Read 2 byte data from registers
     *
     *  Setting data into a register
     *
     *  @param register_name the register name: data writing into (it can be "SROTN_", "CWPW_", "CCWPW_", "CWRCOUNT_", "CCWSCOUNT_", "CWRCOUNT_" or "CCWRCOUNT_" )
     *  @return read 16 bits data from the registers
     */
    esp_err_t read16(RegisterName register_name, uint16_t& result);

    /* prescaler range setting */
    typedef enum {
        PRESCALER_FROM_40_TO_333333 = 0, /*< Prescaler range from   3us(333333pps) to   24.576ms(40   pps) */
        PRESCALER_FROM_20_TO_166667, /*< Prescaler range from   6us(166667pps) to   49.152ms(20   pps) */
        PRESCALER_FROM_10_TO_83333, /*< Prescaler range from  12us( 83333pps) to   98.304ms(10   pps) */
        PRESCALER_FROM_5_TO_41667, /*< Prescaler range from  24us( 41667pps) to  196.608ms( 5   pps) */
        PRESCALER_FROM_2_5_TO_20833, /*< Prescaler range from  48us( 20833pps) to  393.216ms( 2.5 pps) */
        PRESCALER_FROM_1_27_TO_10416, /*< Prescaler range from  96us( 10416pps) to  786.432ms( 1.27pps) */
        PRESCALER_FROM_0_64_TO_5208, /*< Prescaler range from 192us(  5208pps) to 1572.864ms( 0.64pps) */
        PRESCALER_FROM_0_32_TO_2604, /*< Prescaler range from 384us(  2604pps) to 3145.728ms( 0.32pps) */
    } PrescalerRange;


    i2c_port_t port;
    gpio_num_t i2c_sda;
    gpio_num_t i2c_scl;
    uint8_t i2c_address;
    uint32_t clock_speed;
    
    i2c_dev_t i2c_dev;
};


#endif /* PCA9629_H */

