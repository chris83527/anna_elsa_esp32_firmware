/*
 * coin_acceptor_device.cpp
 *
 *  Created on: Jun 23, 2024
 *      Author: chris
 */

#include "coin_hopper_device.h"

esp32cc::CoinHopperDevice::CoinHopperDevice(
    const CctalkLinkController &linkController, const uint8_t deviceAddress)
    : CctalkDevice{linkController, deviceAddress} {}

esp32cc::CoinHopperDevice::~CoinHopperDevice() {}