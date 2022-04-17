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
 * File:   CctalkRequest.cpp
 * Author: chris
 * 
 * Created on January 2, 2022, 8:08 PM
 */

#include <vector>
#include <string>

#include "esp_log.h"

#include "CctalkRequest.h"

static const char *TAG = "CCTALK_REQUEST";

CctalkRequest::CctalkRequest() {
    
}

CctalkRequest::CctalkRequest(const CctalkRequest& orig) {
}

CctalkRequest::~CctalkRequest() {
}

void CctalkRequest::initialise() {
    source = 0;
    header = 0;
    additionalData.clear();
    this->destination = 0;    
}

uint8_t CctalkRequest::getSource(void) {
    return this->source;
}

uint8_t CctalkRequest::getHeader(void) {
    return this->header;
}

std::vector<uint8_t>& CctalkRequest::getAdditionalData(void) {
    return this->additionalData;
}

uint8_t CctalkRequest::getDestination(void) {
    return this->destination;
}

void CctalkRequest::setSource(const uint8_t source) {
    this->source = source;
}

void CctalkRequest::setHeader(const uint8_t header) {
    this->header = header;
}

void CctalkRequest::setAdditionalData(const std::vector<uint8_t>& additionalData) {
    this->additionalData = additionalData;
}

void CctalkRequest::setDestination(const uint8_t destination) {
    this->destination = destination;
}

uint8_t CctalkRequest::getChecksum() {
    ESP_LOGD(TAG, "getChecksum() called");
    uint8_t checksum = 0;
    
    checksum -= this->destination;
    checksum -= this->additionalData.size();
    checksum -= this->source;
    checksum -= this->header;
    for (uint8_t dataByte : additionalData) {
        checksum -= dataByte;
    }
    
    return checksum;  
}