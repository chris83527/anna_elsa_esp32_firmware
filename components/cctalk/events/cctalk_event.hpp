#pragma once

#include "coin_acceptor/coin_acceptor_types.hpp"
#include "hopper/hopper_types.hpp"

enum class CctalkEventType {
    CoinAccepted,
    HopperPayoutStatusChanged
};

struct CctalkEvent {
    CctalkEventType type;
    union {
        CoinEvent coin;
        HopperPayoutStatus hopper;
    };

    static CctalkEvent makeCoin(const CoinEvent& ev) {
        CctalkEvent e{};
        e.type = CctalkEventType::CoinAccepted;
        e.coin = ev;
        return e;
    }
    
    static CctalkEvent makeHopper(const HopperPayoutStatus& st) {
        CctalkEvent e{};
        e.type = CctalkEventType::HopperPayoutStatusChanged;
        e.hopper = st;
        return e;
    }
};
