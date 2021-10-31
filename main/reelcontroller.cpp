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
 * @file reels.c
 *
 * Higher level routines for controlling MCP23008 based reel driver board
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <string.h>
#include <mcp23008.h>
#include <esp_log.h>
#include <driver/ledc.h>
#include <driver/gpio.h>

#include "config.h"
#include "reelcontroller.h"

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY_QUARTER (4095)        // Set duty to 12,5%
#define LEDC_DUTY_FULL (8190)           // Set duty to 50%.((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY (100)            // Frequency in Hertz. Set frequency at 500Hz

static const char *TAG = "ReelController";

i2c_dev_t reel_left;
i2c_dev_t reel_centre;
i2c_dev_t reel_right;

/*
 * 
 * Step 	wire 1 	wire 2 	wire 3 	wire 4
 * 1 	    High 	low 	high 	low
 * 2 	    low 	high 	high 	low
 * 3 	    low 	high 	low 	high
 * 4 	    high 	low 	low 	high
 * 
 */
uint8_t cw_steps[4] = {5, 6, 10, 9};
uint8_t ccw_steps[4] = {9, 10, 6, 5};

// Prepare and then apply the LEDC PWM timer configuration
ReelController::ledc_timer = {
    .speed_mode = LEDC_MODE,
    .timer_num = LEDC_TIMER,
    .duty_resolution = LEDC_DUTY_RES,
    .freq_hz = LEDC_FREQUENCY, // Set output frequency at 100Hz
    .clk_cfg = LEDC_AUTO_CLK
};

// Prepare and then apply the LEDC PWM channel configuration
ReelController::ledc_channel = {
    .speed_mode = LEDC_MODE,
    .channel = LEDC_CHANNEL,
    .timer_sel = LEDC_TIMER,
    .intr_type = LEDC_INTR_DISABLE,
    .gpio_num = GPIO_MOTOR_PWM_EN,
    .duty = 0,
    .hpoint = 0
};

ReelController::ReelController(MainController *mainController) {
    this->mainController = mainController;
}

ReelController::ReelController(const ReelController &orig) {
}

void ReelController::hold() {
    _isHeld = true;
}

bool ReelController::isHeld() {
    return _isHeld;
}

bool ReelController::isCommandInProgress() {
    return commandInProgress;
}

void ReelController::getReelPositions(uint8_t &leftPos, uint8_t &centrePos, uint8_t &rightPos) {
    leftPos = _reelPosLeft;
    centrePos = _reelPosCentre;
    rightPos = _reelPosRight;
}

uint8_t ReelController::getStatus() {

}

esp_err_t ReelController::initialise(void) {
    memset(&reel_left, 0, sizeof (i2c_dev_t));
    memset(&reel_centre, 0, sizeof (i2c_dev_t));
    memset(&reel_right, 0, sizeof (i2c_dev_t));

    memset(&reel_status_data_left, 0, sizeof (reel_status_data_t));
    memset(&reel_status_data_centre, 0, sizeof (reel_status_data_t));
    memset(&reel_status_data_right, 0, sizeof (reel_status_data_t));

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    if (ledc_channel_config(&ledc_channel) != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred initialising PWM subsystem for reels");
        return ESP_FAIL;
    }

    if (mcp23008_init_desc(&reel_left, 0, MCP23008_I2C_ADDR_BASE, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred initialising left reel");
        return ESP_FAIL;
    } else {
        // motor outputs to our H-bridge
        if (mcp23008_set_mode(&reel_left, GPIO_MOTOR_1, MCP23008_GPIO_OUTPUT) != ESP_OK) {
            ESP_LOGE(TAG, "An error occurred initialising right reel");
            return ESP_FAIL;
        }
        mcp23008_set_mode(&reel_left, GPIO_MOTOR_2, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_left, GPIO_MOTOR_3, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_left, GPIO_MOTOR_4, MCP23008_GPIO_OUTPUT);
        // photointerrupter
        mcp23008_set_mode(&reel_left, GPIO_PHOTO_INTERRUPTER, MCP23008_GPIO_INPUT);
        mcp23008_set_pullup(&reel_left, GPIO_PHOTO_INTERRUPTER, true); // switch on pullup (should be on the board, but forgot)
        // following outputs are not used
        mcp23008_set_mode(&reel_left, 5, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_left, 6, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_left, 7, MCP23008_GPIO_OUTPUT);

        ESP_LOGI(TAG, "Left reel initialised ok.");
    }

    if (mcp23008_init_desc(&reel_centre, 0, MCP23008_I2C_ADDR_BASE + 1, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred initialising centre reel");
        return ESP_FAIL;
    } else {
        // motor outputs to our H-bridge
        if (mcp23008_set_mode(&reel_centre, GPIO_MOTOR_1, MCP23008_GPIO_OUTPUT) != ESP_OK) {
            ESP_LOGE(TAG, "An error occurred initialising centre reel");
            //return ESP_FAIL;
        }
        mcp23008_set_mode(&reel_centre, GPIO_MOTOR_2, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_centre, GPIO_MOTOR_3, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_centre, GPIO_MOTOR_4, MCP23008_GPIO_OUTPUT);
        // photointerrupter
        mcp23008_set_mode(&reel_centre, GPIO_PHOTO_INTERRUPTER, MCP23008_GPIO_INPUT);
        mcp23008_set_pullup(&reel_centre, GPIO_PHOTO_INTERRUPTER, true); // switch on pullup (should be on the board, but forgot)
        // following outputs are not used
        mcp23008_set_mode(&reel_centre, 5, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_centre, 6, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_centre, 7, MCP23008_GPIO_OUTPUT);

        ESP_LOGI(TAG, "Centre reel initialised ok.");
    }

    if (mcp23008_init_desc(&reel_right, 0, MCP23008_I2C_ADDR_BASE + 2, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred initialising right reel");
        return ESP_FAIL;
    } else {
        if (mcp23008_set_mode(&reel_right, GPIO_MOTOR_1, MCP23008_GPIO_OUTPUT) != ESP_OK) {
            ESP_LOGE(TAG, "An error occurred initialising right reel");
            //return ESP_FAIL;
        }
        mcp23008_set_mode(&reel_right, GPIO_MOTOR_2, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_right, GPIO_MOTOR_3, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_right, GPIO_MOTOR_4, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_right, GPIO_PHOTO_INTERRUPTER, MCP23008_GPIO_INPUT);
        mcp23008_set_pullup(&reel_right, GPIO_PHOTO_INTERRUPTER, true); // switch on pullup (should be on the board, but forgot)
        mcp23008_set_mode(&reel_right, 5, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_right, 6, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_right, 7, MCP23008_GPIO_OUTPUT);

        ESP_LOGI(TAG, "Right reel initialised ok.");
    }

    spinToZero();

    return ESP_OK;
}

void ReelController::move(int reels, direction_t dir_left, direction_t dir_centre, direction_t dir_right) {

    if (reels & REEL_LEFT) {
        if (dir_left == Clockwise) {
            reel_status_data_left.step_data = (cw_steps[reel_status_data_left.step]);
        } else {
            reel_status_data_left.step_data = (ccw_steps[reel_status_data_left.step]);
        }
        reel_status_data_left.step++;
    }

    if (reels & REEL_CENTRE) {
        if (dir_centre == Clockwise) {
            reel_status_data_centre.step_data = (cw_steps[reel_status_data_centre.step]);
        } else {
            reel_status_data_centre.step_data = (ccw_steps[reel_status_data_centre.step]);
        }
        reel_status_data_centre.step++;
    }

    if (reels & REEL_RIGHT) {
        if (dir_right == Clockwise) {
            reel_status_data_right.step_data = (cw_steps[reel_status_data_right.step]);
        } else {
            reel_status_data_right.step_data = (ccw_steps[reel_status_data_right.step]);
        }
        reel_status_data_right.step++;
    }

    //mcp23008_port_write(&reel_right, reel_status_data_right.step_data);
    //mcp23008_port_write(&reel_centre, reel_status_data_centre.step_data);
    ESP_LOGI(TAG, "Step data: %d", reel_status_data_left.step_data);
    mcp23008_port_write(&reel_left, reel_status_data_left.step_data);

    if (reel_status_data_left.step < 0)
        reel_status_data_left.step = 3;
    if (reel_status_data_left.step > 3)
        reel_status_data_left.step = 0;

    if (reel_status_data_centre.step < 0)
        reel_status_data_centre.step = 3;
    if (reel_status_data_centre.step > 3)
        reel_status_data_centre.step = 0;

    if (reel_status_data_right.step < 0)
        reel_status_data_right.step = 3;
    if (reel_status_data_right.step > 3)
        reel_status_data_right.step = 0;
}

/*

      
 * @brief performs maximum two complete spins of the reels and stops when at the null position.
 * 
 * Performs maximum two complete spins of the reels and stops when at the null position. If no null position is detected then the reel error is set.
 * 
 */
void ReelController::spinToZero() {
    ESP_LOGI(TAG, "spinToZero begin");

    int reels = REEL_LEFT | REEL_CENTRE | REEL_RIGHT;

    ESP_LOGI(TAG, "Setting 50pc duty cycle");

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FULL);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    bool leftOk = false;
    bool centreOk = false;
    bool rightOk = false;

    int counter = 0;

    uint32_t mcp23008_state;

    for (counter = 0; counter < ((MAX_STEPS * 2) * 4); counter++) // two spins, multiply by 4 steps
    {
        ESP_LOGI(TAG, "Calling move...");
        move(reels, Clockwise, Clockwise, Clockwise); // all reels forwards
        vTaskDelay(25 / portTICK_PERIOD_MS);
        mcp23008_get_level(&reel_left, GPIO_PHOTO_INTERRUPTER, &mcp23008_state); // read bit 4 (Photointerrupter)
        ESP_LOGI(TAG, "mcp23008 returned %d", mcp23008_state);
        if ((counter > 4) && (mcp23008_state == false)) // 4 means we made at least one whole step and triggered photointerrupter
        {
            reels &= ~REEL_LEFT;
            leftOk = true;
        }
        // mcp23008_get_level(&reel_centre, GPIO_PHOTO_INTERRUPTER, &mcp23008_state);
        // if ((counter > 4) && ((mcp23008_state & (1<<GPIO_PHOTO_INTERRUPTER)) == 0))
        // {
        //     reels &= ~REEL_CENTRE;
        //     centreOk = true;
        // }
        // mcp23008_get_level(&reel_right, GPIO_PHOTO_INTERRUPTER, &mcp23008_state);
        // if ((counter > 4) && ((mcp23008_state & (1<<GPIO_PHOTO_INTERRUPTER)) == 0))
        // {
        //     reels &= ~REEL_RIGHT;
        //     rightOk = true;
        // }

        //if (leftOk && centreOk && rightOk)
        if (leftOk) {
            break;
        }
    }

    if (leftOk) {
        reel_status_data_left.status |= STATUS_OK;
    } else {
        ESP_LOGE(TAG, "Left reel optic problem");
        reel_status_data_left.status |= STATUS_ERR_REEL_OPTIC;
    }

    if (centreOk) {
        reel_status_data_centre.status |= STATUS_OK;
    } else {
        ESP_LOGE(TAG, "Centre reel optic problem");
        reel_status_data_centre.status |= STATUS_ERR_REEL_OPTIC;
    }

    if (rightOk) {
        reel_status_data_right.status |= STATUS_OK;
    } else {
        ESP_LOGE(TAG, "Right reel optic problem");
        reel_status_data_right.status |= STATUS_ERR_REEL_OPTIC;
    }

    ESP_LOGI(TAG, "Setting 25pc duty cycle");
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_QUARTER);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    ESP_LOGI(TAG, "spinToZero end");
}

void ReelController::spin(const uint8_t leftPos, const uint8_t midPos, const uint8_t rightPos) {

    spinToZero(); // get us back to a known position

    int leftSteps = ((reel_status_data_left.stop + 75) * 4);
    int midSteps = ((reel_status_data_centre.stop + 50) * 4);
    int rightSteps = ((reel_status_data_right.stop + 25) * 4);

    int maxSteps = leftSteps;
    if (midSteps > maxSteps) {
        maxSteps = midSteps;
    }
    if (rightSteps > maxSteps) {
        maxSteps = rightSteps;
    }

    int reels = 0;
    int dly = 25;

    reel_status_data_left.status = 0; // reset status
    reel_status_data_centre.status = 0; // reset status
    reel_status_data_right.status = 0; // reset status

    // 50% duty cycle (~250Hz). Stop the stepper motor getting too hot
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FULL);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    for (int i = 0; i <= maxSteps; i++) {

        if (i < leftSteps) {
            reels |= REEL_LEFT;
        } else {
            reels &= ~REEL_LEFT;
            reel_status_data_left.status = STATUS_OK;
        }

        if (i < midSteps) {
            reels |= REEL_CENTRE;
        } else {
            reels &= ~REEL_CENTRE;
            reel_status_data_centre.status = STATUS_OK;
        }

        if (i < rightSteps) {
            reels |= REEL_RIGHT;
        } else {
            reels &= ~REEL_RIGHT;
            reel_status_data_right.status |= STATUS_OK;
        }

        move(reels, Clockwise, Clockwise, Clockwise); // 7 = all reels forward direction

        if (dly > 15) {
            dly -= 10;
        }

        vTaskDelay(dly / portTICK_PERIOD_MS);
    }

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_QUARTER);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void ReelController::shuffle(const uint8_t leftPos, const uint8_t midPos, const uint8_t rightPos) {
    spinToZero(); // get us back to a known position

    int leftSteps = ((reel_status_data_left.stop + 75) * 4);
    int midSteps = ((reel_status_data_centre.stop + 50) * 4);
    int rightSteps = ((reel_status_data_right.stop + 25) * 4);

    int maxSteps = leftSteps;
    if (midSteps > maxSteps) {
        maxSteps = midSteps;
    }
    if (rightSteps > maxSteps) {
        maxSteps = rightSteps;
    }

    int reels = 0;
    int dly = 75;

    reel_status_data_left.status = 0; // reset status
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FULL);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    for (int i = 0; i <= maxSteps; i++) {

        if (i < leftSteps) {
            reels |= REEL_LEFT;
        } else {
            reels &= ~REEL_LEFT;
            reel_status_data_left.status |= STATUS_OK;
        }

        if (i < midSteps) {
            reels |= REEL_CENTRE;
        } else {
            reels &= ~REEL_CENTRE;
            reel_status_data_centre.status |= STATUS_OK;
        }

        if (i < rightSteps) {
            reels |= REEL_RIGHT;
        } else {
            reels &= ~REEL_RIGHT;
            reel_status_data_right.status |= STATUS_OK;
        }

        move(reels, Clockwise, CounterClockwise, Clockwise); // 5= left, right forwards; centre reverse

        if (dly > 15) {
            dly -= 10;
        }

        vTaskDelay(dly / portTICK_PERIOD_MS);
    }

    //delay(100);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_QUARTER);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

void ReelController::nudge(const uint8_t leftPos, const uint8_t midPos, const uint8_t rightPos) {

    int leftSteps = reel_status_data_left.stop;
    int midSteps = reel_status_data_centre.stop;
    int rightSteps = reel_status_data_right.stop;

    int reels = 0;

    int maxSteps = leftSteps;

    if (midSteps > maxSteps) {
        maxSteps = midSteps;
    }

    if (rightSteps > maxSteps) {
        maxSteps = rightSteps;
    }

    reel_status_data_left.status = 0; // reset status
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FULL);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    for (int i = 0; i < maxSteps; i++) {
        if (i < leftSteps) {
            reels |= REEL_LEFT;
        } else {
            reels &= ~REEL_LEFT;
            reel_status_data_left.status |= STATUS_OK;
        }
        if (i < midSteps) {
            reels |= REEL_CENTRE;
        } else {
            reels &= ~REEL_CENTRE;
            reel_status_data_centre.status |= STATUS_OK;
        }
        if (i < rightSteps) {
            reels |= REEL_RIGHT;
        } else {
            reels &= ~REEL_RIGHT;
            reel_status_data_right.status |= STATUS_OK;
        }
        move(reels, Clockwise, Clockwise, Clockwise);
        vTaskDelay(25 / portTICK_PERIOD_MS);
        move(reels, Clockwise, Clockwise, Clockwise);
        vTaskDelay(25 / portTICK_PERIOD_MS);
        move(reels, Clockwise, Clockwise, Clockwise);
        vTaskDelay(25 / portTICK_PERIOD_MS);
        move(reels, Clockwise, Clockwise, Clockwise);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    //delay(100);
    //analogWrite(MOTOR_EN, 150);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_QUARTER);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}
