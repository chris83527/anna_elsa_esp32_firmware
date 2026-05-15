#pragma once
#include <thread>
#include <atomic>
#include "cctalk_device_facade.hpp"
#include "cctalk_event_queue.hpp"


class HopperThread {
public:

    HopperThread(CctalkDeviceFacade& facade,
                 CctalkEventQueue& queue)
        : facade_(facade), queue_(queue) {}

    void start() {
        running_ = true;
        thread_ = std::thread(&HopperThread::run, this);
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

private:

    static constexpr const char* TAG = "HopperThread";

    void run() {

        while (running_) {
            ESP_LOGI(TAG, "Testing hopper status");
            TestHopperStatus testHopperStatus{};
            CctalkError err = facade_.testHopper(testHopperStatus);
            queue_.push(CctalkEvent::makeHopper(testHopperStatus));

            std::this_thread::sleep_for(std::chrono::seconds(5));
        }

    }

    std::atomic<bool> running_{false};
    std::thread thread_;
    CctalkDeviceFacade& facade_;
    CctalkEventQueue& queue_;
};
