#pragma once

#include "coin_acceptor/coin_acceptor_types.hpp"
#include "hopper/hopper_types.hpp"

enum class CctalkEventType {
    CoinAccepted,
    TestHopperStatus,
};

struct CctalkEvent {
    CctalkEventType type;
    union {
        CoinEvent coin;
        TestHopperStatus hopper;
    };

    static CctalkEvent makeCoin(const CoinEvent& ev) {
        CctalkEvent e{};
        e.type = CctalkEventType::CoinAccepted;
        e.coin = ev;
        return e;
    }
    
    static CctalkEvent makeHopper(const TestHopperStatus& testHopperStatus) {
        CctalkEvent e{};
        e.type = CctalkEventType::TestHopperStatus;
        e.hopper = testHopperStatus;
        return e;
    }
};
