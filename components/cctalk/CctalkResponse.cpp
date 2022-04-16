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
 * File:   CctalkResponse.cpp
 * Author: chris
 * 
 * Created on January 2, 2022, 8:17 PM
 */
#include <vector>
#include <string>

#include "esp_log.h"

#include "CctalkResponse.h"

static const char *TAG = "CCTALK_RESPONSE";

CctalkResponse::CctalkResponse() {

}

CctalkResponse::CctalkResponse(const CctalkResponse& orig) {
}

CctalkResponse::~CctalkResponse() {

}

void CctalkResponse::initialise() {
    this->source = 0;
    this->header = 0;
    this->additionalDataLength = 0;
    this->destination = 0;
    this->additionalData = std::vector<uint8_t>();
    this->checksum = 0;    
    this->error = 0;
}

void CctalkResponse::setDestination(const uint8_t destination) {
    this->destination = destination;
}

uint8_t CctalkResponse::getDestination() {
    return this->destination;
}

void CctalkResponse::setAdditionalDataLength(const uint8_t additionalDataLength) {
    this->additionalDataLength = additionalDataLength;
}

uint8_t CctalkResponse::getAdditionalDataLength() {
    return this->additionalDataLength;
}

void CctalkResponse::setHeader(const uint8_t header) {
    this->header = header;
}

void CctalkResponse::setError(const uint8_t error) {
    this->error = error;
}

uint8_t CctalkResponse::getError() {
    return this->error;
}

uint8_t CctalkResponse::getHeader() {
    return this->header;
}

void CctalkResponse::setSource(const uint8_t source) {
    this->source = source;
}

uint8_t CctalkResponse::getSource() {
    return this->source;
}

void CctalkResponse::setAdditionalData(const std::vector<uint8_t> additionalData) {
    this->additionalData = additionalData;
}

std::vector<uint8_t>& CctalkResponse::getAdditionalData() {
    return this->additionalData;
}

bool CctalkResponse::isValidResponse() {
    uint8_t tmpChecksum = 0;

    if (error != 0) {
        return false;
    }
    
    tmpChecksum -= this->destination;
    tmpChecksum -= this->additionalDataLength;
    tmpChecksum -= this->source;
    tmpChecksum -= this->header;
    for (uint8_t dataByte : additionalData) {
        tmpChecksum -= dataByte;
    }

    ESP_LOGD(TAG, "Validating checksum. Calculated checksum: %03d, provided checksum: %03d", tmpChecksum, this->checksum);
    ESP_LOGD(TAG, "Given additionalDataLength: %d, vector size: %d (should be equal)", additionalDataLength, additionalData.size());

    return (((tmpChecksum -= checksum) == 0) && (additionalData.size() == additionalDataLength) && (this->header == CCTALK_ACK));
}

void CctalkResponse::setChecksum(uint8_t checksum) {
    this->checksum = checksum;
}

uint8_t CctalkResponse::getChecksum() {

    return this->checksum;
}

std::string CctalkResponse::asStringResponse() {

    std::string tmpResponse;

    for (uint8_t dataByte : this->additionalData) {
        tmpResponse.push_back(dataByte);
    }

    return tmpResponse;
}
