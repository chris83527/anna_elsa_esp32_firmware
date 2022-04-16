/*
 * The MIT License
 *
 * Copyright 2022 chris.
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
 * File:   CctalkException.h
 * Author: chris
 *
 * Created on January 6, 2022, 12:28 PM
 */

#ifndef CCTALKEXCEPTION_H
#define CCTALKEXCEPTION_H

#include <exception>

using namespace std;

class CctalkException: public exception {
public:
    CctalkException(const std::string& msg);
    CctalkException(const CctalkException& orig);
    virtual ~CctalkException();
    
    const char* what() const noexcept override {
        return message.c_str();
    }
private:
    std::string message;
};

#endif /* CCTALKEXCEPTION_H */

