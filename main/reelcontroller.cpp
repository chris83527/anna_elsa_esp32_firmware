
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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mcp23008.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#include "config.h"
#include "reelcontroller.h"
#include "audiocontroller.h"
#include "esp_pthread.h"
#include "displaycontroller.h"

#define LEDC_TIMER LEDC_TIMER_1
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY_QUARTER (4095 / 2)        // Set duty to 12,5%
#define LEDC_DUTY_FULL (4095)           // Set duty to 50%.((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY (75)            // Frequency in Hertz. Set frequency at 150Hz

static const char *TAG = "ReelController";

i2c_dev_t reel_left;
i2c_dev_t reel_centre;
i2c_dev_t reel_right;

bool reelLeftInitOk;
bool reelCentreInitOk;
bool reelRightInitOk;

/*
Step	a+	b+	a-	b-
1	1	1	0	0
2	0	1	1	0
3	0	0	1	1
4	1	0	0	1
 */
uint8_t ccw_steps[STEPS_PER_STOP] = {
    GPIO_MOTOR_A_PLUS,
    GPIO_MOTOR_B_PLUS,
    GPIO_MOTOR_A_MINUS,
    GPIO_MOTOR_B_MINUS
};


uint8_t cw_steps[STEPS_PER_STOP] = {
    GPIO_MOTOR_B_MINUS,
    GPIO_MOTOR_A_MINUS,
    GPIO_MOTOR_B_PLUS,
    GPIO_MOTOR_A_PLUS
};

ReelController::ReelController(MainController *mainController) {
    ESP_LOGD(TAG, "Entering constructor");
    this->mainController = mainController;
    ESP_LOGD(TAG, "Leaving constructor");
}

ReelController::ReelController(const ReelController &orig) {
}

bool ReelController::isCommandInProgress() {
    return commandInProgress;
}

void ReelController::getReelPositions(uint8_t &leftPos, uint8_t &centrePos, uint8_t &rightPos) {
    leftPos = reel_status_data_left.stop;
    centrePos = reel_status_data_centre.stop;
    rightPos = reel_status_data_right.stop;
}

bool ReelController::initialise() {
    ESP_LOGD(TAG, "initialise() called");

    reelLeftInitOk = false;
    reelCentreInitOk = false;
    reelRightInitOk = false;

    memset(&reel_left, 0, sizeof (i2c_dev_t));
    memset(&reel_centre, 0, sizeof (i2c_dev_t));
    memset(&reel_right, 0, sizeof (i2c_dev_t));

    memset(&reel_status_data_left, 0, sizeof (reel_status_data_t));
    memset(&reel_status_data_centre, 0, sizeof (reel_status_data_t));
    memset(&reel_status_data_right, 0, sizeof (reel_status_data_t));

    memset(&ledc_timer, 0, sizeof (ledc_timer_config_t));
    memset(&ledc_channel, 0, sizeof (ledc_channel_config_t));

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer.speed_mode = LEDC_MODE;
    ledc_timer.timer_num = LEDC_TIMER;
    ledc_timer.duty_resolution = LEDC_DUTY_RES;
    ledc_timer.freq_hz = LEDC_FREQUENCY; // Set output frequency at 100Hz
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer.timer_num = LEDC_TIMER_1;


    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel.speed_mode = LEDC_MODE;
    ledc_channel.channel = LEDC_CHANNEL;
    ledc_channel.timer_sel = LEDC_TIMER;
    ledc_channel.intr_type = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num = GPIO_MOTOR_PWM_EN;
    ledc_channel.duty = 0;
    ledc_channel.hpoint = 0;

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    if (ledc_channel_config(&ledc_channel) != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred initialising PWM subsystem for reels");
        return ESP_FAIL;
    }

    reel_left.cfg.master.clk_speed = I2C_FREQ_HZ;
    reel_left.cfg.mode = I2C_MODE_MASTER;
    reel_left.cfg.scl_pullup_en = false;
    reel_left.cfg.sda_pullup_en = false;
    if (mcp23008_init_desc(&reel_left, REEL_LEFT_I2C_ADDRESS, 0, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred initialising left reel");
        return false;
    } else {
        // motor outputs to our H-bridge
        if (mcp23008_set_mode(&reel_left, GPIO_MOTOR_A_PLUS, MCP23008_GPIO_OUTPUT) != ESP_OK) {
            ESP_LOGE(TAG, "An error occurred initialising left reel");
            return false;
        }
        mcp23008_set_mode(&reel_left, GPIO_MOTOR_A_MINUS, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_left, GPIO_MOTOR_B_MINUS, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_left, GPIO_MOTOR_B_PLUS, MCP23008_GPIO_OUTPUT);
        // photointerrupter
        mcp23008_set_mode(&reel_left, GPIO_PHOTO_INTERRUPTER, MCP23008_GPIO_INPUT);
        mcp23008_set_pullup(&reel_left, GPIO_PHOTO_INTERRUPTER, true); // switch on pullup (should be on the board, but forgot)
        // following outputs are not used
        mcp23008_set_mode(&reel_left, 5, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_left, 6, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_left, 7, MCP23008_GPIO_OUTPUT);

        reelLeftInitOk = true;
        ESP_LOGD(TAG, "Left reel initialised ok.");
    }

    reel_centre.cfg.master.clk_speed = I2C_FREQ_HZ;
    reel_centre.cfg.mode = I2C_MODE_MASTER;
    reel_centre.cfg.scl_pullup_en = false;
    reel_centre.cfg.sda_pullup_en = false;
    if (mcp23008_init_desc(&reel_centre, REEL_CENTRE_I2C_ADDRESS, 0, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred initialising centre reel");
        return false;
    } else {
        // motor outputs to our H-bridge
        if (mcp23008_set_mode(&reel_centre, GPIO_MOTOR_A_PLUS, MCP23008_GPIO_OUTPUT) != ESP_OK) {
            ESP_LOGE(TAG, "An error occurred initialising centre reel");
            return false;
        }
        mcp23008_set_mode(&reel_centre, GPIO_MOTOR_A_MINUS, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_centre, GPIO_MOTOR_B_MINUS, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_centre, GPIO_MOTOR_B_PLUS, MCP23008_GPIO_OUTPUT);
        // photointerrupter
        mcp23008_set_mode(&reel_centre, GPIO_PHOTO_INTERRUPTER, MCP23008_GPIO_INPUT);
        mcp23008_set_pullup(&reel_centre, GPIO_PHOTO_INTERRUPTER, true); // switch on pullup (should be on the board, but forgot)
        // following outputs are not used
        mcp23008_set_mode(&reel_centre, 5, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_centre, 6, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_centre, 7, MCP23008_GPIO_OUTPUT);

        reelCentreInitOk = true;
        ESP_LOGD(TAG, "Centre reel initialised ok.");
    }

    reel_right.cfg.master.clk_speed = I2C_FREQ_HZ;
    reel_right.cfg.mode = I2C_MODE_MASTER;
    reel_right.cfg.scl_pullup_en = false;
    reel_right.cfg.sda_pullup_en = false;
    if (mcp23008_init_desc(&reel_right, REEL_RIGHT_I2C_ADDRESS, 0, GPIO_I2C_SDA, GPIO_I2C_SCL) != ESP_OK) {
        ESP_LOGE(TAG, "An error occurred initialising right reel");
        //return ESP_FAIL;
    } else {
        if (mcp23008_set_mode(&reel_right, GPIO_MOTOR_A_PLUS, MCP23008_GPIO_OUTPUT) != ESP_OK) {
            ESP_LOGE(TAG, "An error occurred initialising right reel");
            return false;
        }
        mcp23008_set_mode(&reel_right, GPIO_MOTOR_A_MINUS, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_right, GPIO_MOTOR_B_MINUS, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_right, GPIO_MOTOR_B_PLUS, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_right, GPIO_PHOTO_INTERRUPTER, MCP23008_GPIO_INPUT);
        mcp23008_set_pullup(&reel_right, GPIO_PHOTO_INTERRUPTER, true); // switch on pullup (should be on the board, but forgot)
        // following outputs are not used
        mcp23008_set_mode(&reel_right, 5, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_right, 6, MCP23008_GPIO_OUTPUT);
        mcp23008_set_mode(&reel_right, 7, MCP23008_GPIO_OUTPUT);

        reelRightInitOk = true;
        ESP_LOGD(TAG, "Right reel initialised ok.");
    }

    spinToZero();

    return true;
}

void ReelController::step(reel_event_t& event) {

    ESP_LOGD(TAG, "Step event: reels=%d, direction (left)=%d, direction (centre)=%d, direction (Right)=%d", event.reels, event.dir_left, event.dir_centre, event.dir_right);

    if (event.reels & REEL_LEFT) {
        if (event.dir_left == Clockwise) {
            this->reel_status_data_left.step_data = (cw_steps[this->reel_status_data_left.step_number]);
        } else {
            this->reel_status_data_left.step_data = (ccw_steps[this->reel_status_data_left.step_number]);
        }
        this->reel_status_data_left.step_number++;
    }

    if (event.reels & REEL_CENTRE) {
        if (event.dir_centre == Clockwise) {
            this->reel_status_data_centre.step_data = (cw_steps[this->reel_status_data_centre.step_number]);
        } else {
            this->reel_status_data_centre.step_data = (ccw_steps[this->reel_status_data_centre.step_number]);
        }
        this->reel_status_data_centre.step_number++;
    }

    if (event.reels & REEL_RIGHT) {
        if (event.dir_right == Clockwise) {
            this->reel_status_data_right.step_data = (cw_steps[this->reel_status_data_right.step_number]);
        } else {
            this->reel_status_data_right.step_data = (ccw_steps[this->reel_status_data_right.step_number]);
        }
        this->reel_status_data_right.step_number++;
    }

    if (this->reelRightInitOk) {
        ESP_LOGD(TAG, "Right reel Step data: %d", this->reel_status_data_right.step_data);
        mcp23008_port_write(&reel_right, this->reel_status_data_right.step_data);
    }
    if (this->reelCentreInitOk) {
        ESP_LOGD(TAG, "Centre reel Step data: %d", this->reel_status_data_centre.step_data);
        mcp23008_port_write(&reel_centre, this->reel_status_data_centre.step_data);
    }
    if (this->reelLeftInitOk) {
        ESP_LOGD(TAG, "Left reel Step data: %d", this->reel_status_data_left.step_data);
        mcp23008_port_write(&reel_left, this->reel_status_data_left.step_data);
    }

    if (this->reel_status_data_left.step_number = STEPS_PER_STOP) this->reel_status_data_left.step_number = 0;
    if (this->reel_status_data_centre.step_number = STEPS_PER_STOP) this->reel_status_data_centre.step_number = 0;
    if (this->reel_status_data_right.step_number = STEPS_PER_STOP) this->reel_status_data_right.step_number = 0;

    ESP_LOGI(TAG, "Written %d", this->reel_status_data_left.step_data);
    mainController->getDisplayController()->waitForButton(BTN_START_MASK_BIT); // DEBUG -> check motor is turning correctly
    
}

/*
 * @brief performs maximum two complete spins of the reels and stops when at the null position.
 * 
 * Performs maximum two complete spins of the reels and stops when at the null position. If no null position is detected then the reel error is set.
 * 
 */
void ReelController::spinToZero() {
    ESP_LOGD(TAG, "spinToZero begin");

    int reels = REEL_LEFT | REEL_CENTRE | REEL_RIGHT;

    ESP_LOGD(TAG, "Setting 50pc duty cycle");

    uint32_t mcp23008_left_state;
    uint32_t mcp23008_centre_state;
    uint32_t mcp23008_right_state;

    bool leftOk = false;
    bool centreOk = false;
    bool rightOk = false;

    auto cfg = esp_pthread_get_default_config();
    cfg.thread_name = "SpinReelThread";
    cfg.prio = 10;
    cfg.stack_size = 1024;
    esp_pthread_set_cfg(&cfg);
    this->spinReelThread.reset(new std::thread([ & ]() {
        reel_event_t event;

        ESP_LOGD(TAG, "Setting 50pc duty cycle");
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FULL);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

        int delay = 100;

        for (int counter = 0; counter < ((MAX_STOPS * 2) * STEPS_PER_STOP); counter++) // two spins, multiply by 4 steps
        {
            //            ESP_LOGD(TAG, "Calling move...");

            event.reels = reels;
                    event.dir_left = Clockwise;
                    event.dir_centre = Clockwise;
                    event.dir_right = Clockwise;

                    step(event);

            if (reelLeftInitOk) {
                mcp23008_get_level(&reel_left, GPIO_PHOTO_INTERRUPTER, &mcp23008_left_state); // read bit 4 (Photointerrupter)
                        ESP_LOGD(TAG, "Left reel photointerrupter returned %d", mcp23008_left_state);
                if ((counter > (MAX_STOPS * STEPS_PER_STOP)) && (mcp23008_left_state == 0)) // 4 means we made at least one whole step and triggered photointerrupter
                {
                    ESP_LOGD(TAG, "Disabling left reel");
                            reels &= ~REEL_LEFT;
                            leftOk = true;
                }
            }

            if (reelCentreInitOk) {
                mcp23008_get_level(&reel_centre, GPIO_PHOTO_INTERRUPTER, &mcp23008_centre_state);
                        ESP_LOGD(TAG, "Centre reel photointerrupter returned %d", mcp23008_centre_state);
                if ((counter > (MAX_STOPS * STEPS_PER_STOP)) && (mcp23008_centre_state == 0)) {
                    ESP_LOGD(TAG, "Disabling centre reel");
                            reels &= ~REEL_CENTRE;
                            centreOk = true;
                }
            }

            if (reelRightInitOk) {
                mcp23008_get_level(&reel_right, GPIO_PHOTO_INTERRUPTER, &mcp23008_right_state);
                        ESP_LOGD(TAG, "Right reel photointerrupter returned %d", mcp23008_right_state);
                if ((counter > (MAX_STOPS * STEPS_PER_STOP)) && (mcp23008_right_state == 0)) {
                    ESP_LOGD(TAG, "Disabling right reel");
                            reels &= ~REEL_RIGHT;
                            rightOk = true;
                }
            }

            if (delay > 5) {
                delay -= 5;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(delay));

        }
    }));

    this->spinReelThread->join();

    if (leftOk) {
        reel_status_data_left.status = STATUS_OK;
    } else {
        ESP_LOGE(TAG, "Left reel optic problem");
        reel_status_data_left.status = STATUS_ERR_REEL_OPTIC;
    }

    if (centreOk) {
        reel_status_data_centre.status = STATUS_OK;
    } else {
        ESP_LOGE(TAG, "Centre reel optic problem");
        reel_status_data_centre.status = STATUS_ERR_REEL_OPTIC;
    }

    if (rightOk) {
        reel_status_data_right.status = STATUS_OK;
    } else {

        ESP_LOGE(TAG, "Right reel optic problem");
        reel_status_data_right.status = STATUS_ERR_REEL_OPTIC;
    }

    ESP_LOGD(TAG, "Setting 25pc duty cycle");
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_QUARTER);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    ESP_LOGD(TAG, "spinToZero end");
}

void ReelController::spin(const uint8_t leftPos, const uint8_t midPos, const uint8_t rightPos) {

    this->commandInProgress = true;

    reel_status_data_left.stop = leftPos;
    reel_status_data_centre.stop = midPos;
    reel_status_data_right.stop = rightPos;

    // Use different values (75, 50, 25) to get the effect of one reel stopping after another
    int leftSteps = ((reel_status_data_left.stop + 75) * STEPS_PER_STOP);
    int midSteps = ((reel_status_data_centre.stop + 50) * STEPS_PER_STOP);
    int rightSteps = ((reel_status_data_right.stop + 25) * STEPS_PER_STOP);

    int maxSteps = leftSteps;
    if (midSteps > maxSteps) {
        maxSteps = midSteps;
    }
    if (rightSteps > maxSteps) {
        maxSteps = rightSteps;
    }

    int reels = 0;

    reel_status_data_left.status = STATUS_INITIAL; // reset status
    reel_status_data_centre.status = STATUS_INITIAL; // reset status
    reel_status_data_right.status = STATUS_INITIAL; // reset status

    spinToZero(); // get us back to a known position

    ESP_LOGD(TAG, "Setting 50pc duty cycle");
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FULL);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    auto cfg = esp_pthread_get_default_config();
    cfg.thread_name = "SpinReelThread";
    cfg.prio = 10;
    cfg.stack_size = 1024;
    esp_pthread_set_cfg(&cfg);
    this->spinReelThread.reset(new std::thread([ & ]() {
        reel_event_t event;

        int delay = 75;

        for (int i = 0; i <= maxSteps; i++) {

            if (reel_status_data_left.status != STATUS_OK) {
                if (i < leftSteps) {
                    reels |= REEL_LEFT;
                } else {
                    reels &= ~REEL_LEFT;
                            reel_status_data_left.status = STATUS_OK;
                            this->mainController->getAudioController()->playAudioFile(Sounds::SND_REEL_STOP);
                }
            }

            if (reel_status_data_centre.status != STATUS_OK) {
                if (i < midSteps) {
                    reels |= REEL_CENTRE;
                } else {
                    reels &= ~REEL_CENTRE;
                            reel_status_data_centre.status = STATUS_OK;
                            this->mainController->getAudioController()->playAudioFile(Sounds::SND_REEL_STOP);
                }
            }

            if (reel_status_data_right.status != STATUS_OK) {
                if (i < rightSteps) {
                    reels |= REEL_RIGHT;
                } else {
                    reels &= ~REEL_RIGHT;
                            reel_status_data_right.status = STATUS_OK;
                            this->mainController->getAudioController()->playAudioFile(Sounds::SND_REEL_STOP);
                }
            }

            event.reels = reels;
                    event.dir_left = Clockwise;
                    event.dir_centre = Clockwise;
                    event.dir_right = Clockwise;

                    step(event);

            if (delay > 10) {
                delay -= 10;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        }
    }));

    this->spinReelThread->join();

    ESP_LOGD(TAG, "Setting 25pc duty cycle");
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_QUARTER);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    this->commandInProgress = false;
}

void ReelController::shuffle(const uint8_t leftPos, const uint8_t midPos, const uint8_t rightPos) {

    this->commandInProgress = true;

    reel_status_data_left.stop = leftPos;
    reel_status_data_centre.stop = midPos;
    reel_status_data_right.stop = rightPos;

    int leftSteps = ((reel_status_data_left.stop + 75) * STEPS_PER_STOP);
    int midSteps = ((reel_status_data_centre.stop + 50) * STEPS_PER_STOP);
    int rightSteps = ((reel_status_data_right.stop + 25) * STEPS_PER_STOP);

    int maxSteps = leftSteps;
    if (midSteps > maxSteps) {
        maxSteps = midSteps;
    }
    if (rightSteps > maxSteps) {
        maxSteps = rightSteps;
    }

    int reels = 0;

    reel_status_data_left.status = STATUS_INITIAL; // reset status
    reel_status_data_centre.status = STATUS_INITIAL; // reset status
    reel_status_data_right.status = STATUS_INITIAL; // reset status

    spinToZero(); // get us back to a known position

    ESP_LOGD(TAG, "Setting 50pc duty cycle");

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FULL);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    this->spinReelThread.reset(new std::thread([ & ]() {
        reel_event_t event;

        int delay = 75;

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
                        reel_status_data_right.status = STATUS_OK;
            }

            event.reels = reels;
                    event.dir_left = Clockwise;
                    event.dir_centre = CounterClockwise;
                    event.dir_right = Clockwise;

                    step(event);

            if (delay > 10) {
                delay -= 10;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        }
    }));

    this->spinReelThread->join();

    ESP_LOGD(TAG, "Setting 25pc duty cycle");
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_QUARTER);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    this->commandInProgress = false;
}

void ReelController::nudge(const uint8_t leftStops, const uint8_t midStops, const uint8_t rightStops) {

    this->commandInProgress = true;
    //ESP_LOGD(TAG, "nudge() called: leftStops: %d, midStops: %d, rightStops: %d", leftStops, midStops, rightStops);

    reel_status_data_left.stop += leftStops;
    reel_status_data_centre.stop += midStops;
    reel_status_data_right.stop += rightStops;

    int leftSteps = leftStops * STEPS_PER_STOP;
    int midSteps = midStops * STEPS_PER_STOP;
    int rightSteps = rightStops * STEPS_PER_STOP;

    int reels = 0;

    int maxSteps = leftSteps;

    if (midSteps > maxSteps) {
        maxSteps = midSteps;
    }

    if (rightSteps > maxSteps) {
        maxSteps = rightSteps;
    }

    reel_status_data_left.status = STATUS_INITIAL; // reset status
    reel_status_data_centre.status = STATUS_INITIAL; // reset status
    reel_status_data_right.status = STATUS_INITIAL; // reset status

    int32_t speed_current = 32;

    ESP_LOGD(TAG, "Setting 50pc duty cycle");
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_FULL);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);


    this->spinReelThread.reset(new std::thread([ & ]() {
        reel_event_t event;

        for (int i = 0; i < maxSteps; i++) {
            if (i < leftSteps) {
                reels |= REEL_LEFT;
            } else {
                reels &= ~REEL_LEFT;
                        reel_status_data_left.status = STATUS_OK;
                        this->mainController->getAudioController()->playAudioFile(Sounds::SND_REEL_STOP);
            }
            if (i < midSteps) {
                reels |= REEL_CENTRE;
            } else {
                reels &= ~REEL_CENTRE;
                        reel_status_data_centre.status = STATUS_OK;
                        this->mainController->getAudioController()->playAudioFile(Sounds::SND_REEL_STOP);
            }
            if (i < rightSteps) {
                reels |= REEL_RIGHT;
            } else {
                reels &= ~REEL_RIGHT;
                        reel_status_data_right.status = STATUS_OK;
                        this->mainController->getAudioController()->playAudioFile(Sounds::SND_REEL_STOP);
            }

            event.reels = reels;
                    event.dir_left = Clockwise;
                    event.dir_centre = Clockwise;
                    event.dir_right = Clockwise;

                    step(event);

                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }));

    this->spinReelThread->join();

    ESP_LOGD(TAG, "Setting 25pc duty cycle");
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_QUARTER);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    this->commandInProgress = false;
}
