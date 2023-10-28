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

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "i2c_manager.h"

#include "include/ssd1306.h"

#define tag "SSD1306"

static const char *TAG = "ssd1306_i2c";

void i2c_init(SSD1306_t * dev, int width, int height) {
    
    dev->_flip = false;
    dev->_width = width;
    dev->_height = height;
    dev->_pages = 8;
    if (dev->_height == 32) dev->_pages = 4;

    uint8_t data[27];
    data[0] = OLED_CONTROL_BYTE_CMD_STREAM;
    data[1] = OLED_CMD_DISPLAY_OFF; // AE
    data[2] = OLED_CMD_SET_MUX_RATIO; // AB
    if (dev->_height == 64) {
        data[3] = 0x3f; // 0x3f = height 64 // 0x1f = height 32,
    } else {
        data[3] = 0x1f;
    }
    data[4] = OLED_CMD_SET_DISPLAY_OFFSET;
    data[5] = 0x00;
    data[6] = OLED_CMD_SET_DISPLAY_START_LINE;
    if (dev->_flip) {
        data[7] = OLED_CMD_SET_SEGMENT_REMAP_0;
    } else {
        data[7] = OLED_CMD_SET_SEGMENT_REMAP_1;
    }
    data[8] = OLED_CMD_SET_COM_SCAN_MODE;
    data[9] = OLED_CMD_SET_DISPLAY_CLK_DIV;
    data[10] = 0x80;
    data[11] = OLED_CMD_SET_COM_PIN_MAP;
    if (dev->_height == 64) {
        data[12] = 0x12; // height 64
    } else {
        data[12] = 0x02;
    }
    data[13] = OLED_CMD_SET_CONTRAST;
    data[14] = 0xff;
    data[15] = OLED_CMD_DISPLAY_RAM;
    data[16] = OLED_CMD_SET_VCOMH_DESELCT;
    data[17] = 0x40;
    data[18] = OLED_CMD_SET_MEMORY_ADDR_MODE;
    data[19] = OLED_CMD_SET_PAGE_ADDR_MODE;
    data[20] = 0x00;
    data[21] = 0x10;
    data[22] = OLED_CMD_SET_CHARGE_PUMP;
    data[23] = 0x14;
    data[24] = OLED_CMD_DEACTIVE_SCROLL;
    data[25] = OLED_CMD_DISPLAY_NORMAL;
    data[26] = OLED_CMD_DISPLAY_ON;

    // initialise display
    i2c_manager_write(dev->_port, dev->_address, I2C_NO_REG, data, 27);

}

void i2c_display_image(SSD1306_t * dev, int page, int seg, uint8_t * images, int width) {

    if (page >= dev->_pages) return;
    if (seg >= dev->_width) return;

    int _seg = seg + CONFIG_OFFSETX;
    uint8_t columLow = _seg & 0x0F;
    uint8_t columHigh = (_seg >> 4) & 0x0F;

    int _page = page;
    if (dev->_flip) {
        _page = (dev->_pages - page) - 1;
    }

    uint8_t data[4];
    data[0] = OLED_CONTROL_BYTE_CMD_STREAM;
    data[1] = (0x00 + columLow);
    data[2] = (0x10 + columHigh);
    data[3] = (0xB0 | _page);

    i2c_manager_write(dev->_port, dev->_address, I2C_NO_REG, data, 4);

    uint8_t *imagedata = (uint8_t*) calloc(width + 1, sizeof (uint8_t));
    imagedata[0] = OLED_CONTROL_BYTE_DATA_STREAM;

    memcpy(&imagedata[1], images, width);
    
    i2c_manager_write(dev->_port, dev->_address, I2C_NO_REG, imagedata, width + 1);

    free(imagedata);
}

void i2c_contrast(SSD1306_t * dev, int contrast) {

    int _contrast = contrast;

    if (contrast < 0x0) _contrast = 0;
    if (contrast > 0xFF) _contrast = 0xFF;

    uint8_t data[3] = {
        OLED_CONTROL_BYTE_CMD_STREAM,
        OLED_CMD_SET_CONTRAST,
        (uint8_t) _contrast,
    };

    i2c_manager_write(dev->_port, dev->_address, I2C_NO_REG, data, 3);
}

//void i2c_hardware_scroll(SSD1306_t * dev, ssd1306_scroll_type_t scroll) {
//    esp_err_t espRc;
//
//    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
//
//    if (scroll == SCROLL_RIGHT) {
//        i2c_master_write_byte(cmd, OLED_CMD_HORIZONTAL_RIGHT, true); // 26
//        i2c_master_write_byte(cmd, 0x00, true); // Dummy byte
//        i2c_master_write_byte(cmd, 0x00, true); // Define start page address
//        i2c_master_write_byte(cmd, 0x07, true); // Frame frequency
//        i2c_master_write_byte(cmd, 0x07, true); // Define end page address
//        i2c_master_write_byte(cmd, 0x00, true); //
//        i2c_master_write_byte(cmd, 0xFF, true); //
//        i2c_master_write_byte(cmd, OLED_CMD_ACTIVE_SCROLL, true); // 2F
//    }
//
//    if (scroll == SCROLL_LEFT) {
//        i2c_master_write_byte(cmd, OLED_CMD_HORIZONTAL_LEFT, true); // 27
//        i2c_master_write_byte(cmd, 0x00, true); // Dummy byte
//        i2c_master_write_byte(cmd, 0x00, true); // Define start page address
//        i2c_master_write_byte(cmd, 0x07, true); // Frame frequency
//        i2c_master_write_byte(cmd, 0x07, true); // Define end page address
//        i2c_master_write_byte(cmd, 0x00, true); //
//        i2c_master_write_byte(cmd, 0xFF, true); //
//        i2c_master_write_byte(cmd, OLED_CMD_ACTIVE_SCROLL, true); // 2F
//    }
//
//    if (scroll == SCROLL_DOWN) {
//        i2c_master_write_byte(cmd, OLED_CMD_CONTINUOUS_SCROLL, true); // 29
//        i2c_master_write_byte(cmd, 0x00, true); // Dummy byte
//        i2c_master_write_byte(cmd, 0x00, true); // Define start page address
//        i2c_master_write_byte(cmd, 0x07, true); // Frame frequency
//        //i2c_master_write_byte(cmd, 0x01, true); // Define end page address
//        i2c_master_write_byte(cmd, 0x00, true); // Define end page address
//        i2c_master_write_byte(cmd, 0x3F, true); // Vertical scrolling offset
//
//        i2c_master_write_byte(cmd, OLED_CMD_VERTICAL, true); // A3
//        i2c_master_write_byte(cmd, 0x00, true);
//        if (dev->_height == 64)
//            //i2c_master_write_byte(cmd, 0x7F, true);
//            i2c_master_write_byte(cmd, 0x40, true);
//        if (dev->_height == 32)
//            i2c_master_write_byte(cmd, 0x20, true);
//        i2c_master_write_byte(cmd, OLED_CMD_ACTIVE_SCROLL, true); // 2F
//    }
//
//    if (scroll == SCROLL_UP) {
//        i2c_master_write_byte(cmd, OLED_CMD_CONTINUOUS_SCROLL, true); // 29
//        i2c_master_write_byte(cmd, 0x00, true); // Dummy byte
//        i2c_master_write_byte(cmd, 0x00, true); // Define start page address
//        i2c_master_write_byte(cmd, 0x07, true); // Frame frequency
//        //i2c_master_write_byte(cmd, 0x01, true); // Define end page address
//        i2c_master_write_byte(cmd, 0x00, true); // Define end page address
//        i2c_master_write_byte(cmd, 0x01, true); // Vertical scrolling offset
//
//        i2c_master_write_byte(cmd, OLED_CMD_VERTICAL, true); // A3
//        i2c_master_write_byte(cmd, 0x00, true);
//        if (dev->_height == 64)
//            //i2c_master_write_byte(cmd, 0x7F, true);
//            i2c_master_write_byte(cmd, 0x40, true);
//        if (dev->_height == 32)
//            i2c_master_write_byte(cmd, 0x20, true);
//        i2c_master_write_byte(cmd, OLED_CMD_ACTIVE_SCROLL, true); // 2F
//    }
//
//    if (scroll == SCROLL_STOP) {
//        i2c_master_write_byte(cmd, OLED_CMD_DEACTIVE_SCROLL, true); // 2E
//    }
//
//    i2c_master_stop(cmd);
//    espRc = i2c_master_cmd_begin(I2C_NUM, cmd, 10 / portTICK_PERIOD_MS);
//    if (espRc == ESP_OK) {
//        ESP_LOGD(tag, "Scroll command succeeded");
//    } else {
//        ESP_LOGE(tag, "Scroll command failed. code: 0x%.2X", espRc);
//    }
//
//    i2c_cmd_link_delete(cmd);
//}
