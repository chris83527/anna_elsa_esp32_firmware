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
 * File:   CctalkRequest.h
 * Author: chris
 *
 * Created on January 2, 2022, 8:08 PM
 */

#ifndef CCTALKREQUEST_H
#define CCTALKREQUEST_H

using namespace std;

#include <vector>
#include <string>

#include "cctalk.h"

class CctalkRequest {
public:
    CctalkRequest();    
    CctalkRequest(const CctalkRequest& orig);    
    virtual ~CctalkRequest();
    
    void initialise(void);
    
    uint8_t getSource(void);
    uint8_t getHeader(void);
    std::vector<uint8_t>& getAdditionalData(void);
    uint8_t getDestination(void);
    uint8_t getChecksum();    
    bool isValidRequest();
    
    void setSource(const uint8_t source);
    void setHeader(const uint8_t header);
    void setDestination(const uint8_t destination);
    void setAdditionalData(const std::vector<uint8_t>& additionalData);    
    
    
private:
        uint8_t source;
        uint8_t header;
        std::vector<uint8_t> additionalData;
        uint8_t destination;
};

#endif /* CCTALKREQUEST_H */

