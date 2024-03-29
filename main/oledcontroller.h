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
 * File:   oledcontroller.h
 * Author: chris
 *
 * Created on January 11, 2023, 9:53 PM
 */

#ifndef OLEDCONTROLLER_H
#define OLEDCONTROLLER_H

#include "ssd1306.h"

#include <cstring>

class oledcontroller {
public:
    oledcontroller();
    oledcontroller(const oledcontroller& orig);
    virtual ~oledcontroller();
    void initialise(void);
    void displayText(std::string textToDisplay, int lineNumber, bool invert);
    void scrollText(std::string textToDisplay);
    void clearDisplay(void);
    void testDisplay();    

private:

    SSD1306* ssd1306;
    
    int top = 2;
    int center = 3;
    int bottom = 8;
    char lineChar[20];


};

#endif /* OLEDCONTROLLER_H */

