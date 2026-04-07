#pragma once
#include <thread>
#include <atomic>
#include "cctalk_event_queue.hpp"
#include "esp_log.h"

class EventDispatcherThread {
public:
    EventDispatcherThread(CctalkEventQueue& queue)
        : queue_(queue) {}

    void start() {
        running_ = true;
        thread_ = std::thread(&EventDispatcherThread::run, this);
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

private:
    void run() {
        while (running_) {
            CctalkEvent evt = queue_.popBlocking();

            switch (evt.type) {
            case CctalkEventType::CoinAccepted:
                ESP_LOGI("EVENT", "Coin accepted: %u cents",
                         evt.coin.coin_value);
                break;

            case CctalkEventType::HopperStatusChanged:
                ESP_LOGI("EVENT", "Hopper status changed: raw=0x%02X",
                         evt.hopper.raw_status);
                break;
            }

            // TODO: more
        }
    }

    std::atomic<bool> running_{false};
    std::thread thread_;
    CctalkEventQueue& queue_;
};
