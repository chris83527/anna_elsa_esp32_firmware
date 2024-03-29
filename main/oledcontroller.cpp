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
 * File:   oledcontroller.cpp
 * Author: chris
 * 
 * Created on January 11, 2023, 9:53 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "ssd1306.h"
#include "font8x8_basic.h"

#include "sdkconfig.h"
#include "config.h"

#include "oledcontroller.h"

#include <cstring>

static const char *TAG = "oledcontroller";

int i2c_address = 0x3c;

oledcontroller::oledcontroller() {
    #if CONFIG_SSD1306_128x64
        ssd1306 = new SSD1306(I2C_NUM_0, i2c_address, 128, 64);
    #endif // CONFIG_SSD1306_128x64
    #if CONFIG_SSD1306_128x32
        ssd1306 = new SSD1306(I2C_NUM_0, i2c_address, 128, 32);
    #endif
}

oledcontroller::oledcontroller(const oledcontroller& orig) {
}

oledcontroller::~oledcontroller() {
}

void oledcontroller::initialise() {  
    ssd1306->init();

#if CONFIG_FLIP
    ssd1306->_flip = true;
    ESP_LOGW(TAG, "Flip upside down");
#endif

    ssd1306->clear_screen(false);
    ssd1306->contrast(0xff);

}

void oledcontroller::clearDisplay() {
    ssd1306->clear_screen(false);
}

void oledcontroller::scrollText(std::string textToDisplay) {    
    ssd1306->software_scroll((ssd1306->get_pages() - 1), 1);    
    ssd1306->scroll_text(textToDisplay.append(20 - textToDisplay.size(), ' '), false);
}

void oledcontroller::displayText(std::string textToDisplay, int lineNumber, bool invert) { 
    //ESP_LOGI(TAG, "Displaying text %s", textToDisplay.c_str());
    if (textToDisplay.size() > 20) {
        textToDisplay = textToDisplay.substr(0,20);
    }
    ssd1306->display_text(lineNumber, textToDisplay.append(20 - textToDisplay.size(), ' '), invert);
}



//void oledcontroller::testDisplay() {
//    ssd1306->display_text_x3(0, "Test", 4, false);
//    vTaskDelay(3000 / portTICK_PERIOD_MS);
//
//#if CONFIG_SSD1306_128x64
//    top = 2;
//    center = 3;
//    bottom = 8;
//    ssd1306->display_text(0, "SSD1306 128x64", 14, false);
//    ssd1306->display_text(1, "ABCDEFGHIJKLMNOP", 16, false);
//    ssd1306->display_text(2, "abcdefghijklmnop", 16, false);
//    ssd1306->display_text(3, "Hello World!!", 13, false);
//    //ssd1306->clear_line(4, true);
//    //ssd1306->clear_line(5, true);
//    //ssd1306->clear_line(6, true);
//    //ssd1306->clear_line(7, true);
//    ssd1306->display_text(4, "SSD1306 128x64", 14, true);
//    ssd1306->display_text(5, "ABCDEFGHIJKLMNOP", 16, true);
//    ssd1306->display_text(6, "abcdefghijklmnop", 16, true);
//    ssd1306->display_text(7, "Hello World!!", 13, true);
//#endif // CONFIG_SSD1306_128x64
//
//#if CONFIG_SSD1306_128x32
//    top = 1;
//    center = 1;
//    bottom = 4;
//    ssd1306->display_text(0, "SSD1306 128x32", 14, false);
//    ssd1306->display_text(1, "Hello World!!", 13, false);
//    //ssd1306->clear_line(2, true);
//    //ssd1306->clear_line(3, true);
//    ssd1306->display_text(2, "SSD1306 128x32", 14, true);
//    ssd1306->display_text(3, "Hello World!!", 13, true);
//#endif // CONFIG_SSD1306_128x32
//    vTaskDelay(3000 / portTICK_PERIOD_MS);
//
//    // Display Count Down
//    uint8_t image[24];
//    memset(image, 0, sizeof (image));
//    ssd1306->display_image(top, (6 * 8 - 1), image, sizeof (image));
//    ssd1306->display_image(top + 1, (6 * 8 - 1), image, sizeof (image));
//    ssd1306->display_image(top + 2, (6 * 8 - 1), image, sizeof (image));
//    for (int font = 0x39; font > 0x30; font--) {
//        memset(image, 0, sizeof (image));
//        ssd1306->display_image(top + 1, (7 * 8 - 1), image, 8);
//        memcpy(image, font8x8_basic_tr[font], 8);
//        if (ssd1306->_flip) ssd1306->flip(image, 8);
//        ssd1306->display_image(top + 1, (7 * 8 - 1), image, 8);
//        vTaskDelay(1000 / portTICK_PERIOD_MS);
//    }
//
//    // Scroll Up
//    ssd1306->clear_screen(false);
//    ssd1306->contrast(0xff);
//    ssd1306->display_text(0, "---Scroll  UP---", 16, true);
//    //ssd1306->software_scroll(7, 1);
//    ssd1306->software_scroll((ssd1306->_pages - 1), 1);
//    for (int line = 0; line < bottom + 10; line++) {
//        lineChar[0] = 0x01;
//        sprintf(&lineChar[1], " Line %02d", line);
//        ssd1306->scroll_text(lineChar, strlen(lineChar), false);
//        vTaskDelay(500 / portTICK_PERIOD_MS);
//    }
//    vTaskDelay(3000 / portTICK_PERIOD_MS);
//
//    // Scroll Down
//    ssd1306->clear_screen(false);
//    ssd1306->contrast(0xff);
//    ssd1306->display_text(0, "--Scroll  DOWN--", 16, true);
//    //ssd1306->software_scroll(1, 7);
//    ssd1306->software_scroll(1, (ssd1306->_pages - 1));
//    for (int line = 0; line < bottom + 10; line++) {
//        lineChar[0] = 0x02;
//        sprintf(&lineChar[1], " Line %02d", line);
//        ssd1306->scroll_text(lineChar, strlen(lineChar), false);
//        vTaskDelay(500 / portTICK_PERIOD_MS);
//    }
//    vTaskDelay(3000 / portTICK_PERIOD_MS);
//
//    // Page Down
//    ssd1306->clear_screen(false);
//    ssd1306->contrast(0xff);
//    ssd1306->display_text(0, "---Page	DOWN---", 16, true);
//    ssd1306->software_scroll(1, (ssd1306->_pages - 1));
//    for (int line = 0; line < bottom + 10; line++) {
//        //if ( (line % 7) == 0) ssd1306->scroll_clear(&dev);
//        if ((line % (ssd1306->_pages - 1)) == 0) ssd1306->scroll_clear(&dev);
//        lineChar[0] = 0x02;
//        sprintf(&lineChar[1], " Line %02d", line);
//        ssd1306->scroll_text(lineChar, strlen(lineChar), false);
//        vTaskDelay(500 / portTICK_PERIOD_MS);
//    }
//    vTaskDelay(3000 / portTICK_PERIOD_MS);
//
//    // Horizontal Scroll
//    ssd1306->clear_screen(false);
//    ssd1306->contrast(0xff);
//    ssd1306->display_text(center, "Horizontal", 10, false);
//    ssd1306->hardware_scroll(SCROLL_RIGHT);
//    vTaskDelay(5000 / portTICK_PERIOD_MS);
//    ssd1306->hardware_scroll(SCROLL_LEFT);
//    vTaskDelay(5000 / portTICK_PERIOD_MS);
//    ssd1306->hardware_scroll(SCROLL_STOP);
//
//    // Vertical Scroll
//    ssd1306->clear_screen(false);
//    ssd1306->contrast(0xff);
//    ssd1306->display_text(center, "Vertical", 8, false);
//    ssd1306->hardware_scroll(SCROLL_DOWN);
//    vTaskDelay(5000 / portTICK_PERIOD_MS);
//    ssd1306->hardware_scroll(SCROLL_UP);
//    vTaskDelay(5000 / portTICK_PERIOD_MS);
//    ssd1306->hardware_scroll(SCROLL_STOP);
//
//    // Invert
//    ssd1306->clear_screen(true);
//    ssd1306->contrast(0xff);
//    ssd1306->display_text(center, "  Good Bye!!", 12, true);
//    vTaskDelay(5000 / portTICK_PERIOD_MS);
//
//
//    // Fade Out
//    ssd1306->fadeout(&dev);
//
//#if 0
//    // Fade Out
//    for (int contrast = 0xff; contrast > 0; contrast = contrast - 0x20) {
//        ssd1306->contrast(contrast);
//        vTaskDelay(40);
//    }
//#endif

//}