#pragma once

#include "coin_acceptor/coin_acceptor_types.hpp"
#include "hopper/hopper_types.hpp"

enum class CctalkEventType {
    CoinAccepted,
    HopperStatusChanged
};

struct CctalkEvent {
    CctalkEventType type;
    union {
        CoinEvent coin;
        HopperStatus hopper;
    };

    static CctalkEvent makeCoin(const CoinEvent& ev) {
        CctalkEvent e{};
        e.type = CctalkEventType::CoinAccepted;
        e.coin = ev;
        return e;
    }
    static CctalkEvent makeHopper(const HopperStatus& st) {
        CctalkEvent e{};
        e.type = CctalkEventType::HopperStatusChanged;
        e.hopper = st;
        return e;
    }
};
