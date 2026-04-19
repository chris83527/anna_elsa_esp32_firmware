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
    void run() {
        HopperStatus last{};

        while (running_) {
            HopperStatus st{};
            facade_.getHopperStatus(st);

            if (st.raw_status != last.raw_status) {
                queue_.push(CctalkEvent::makeHopper(st));
                last = st;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    std::atomic<bool> running_{false};
    std::thread thread_;
    CctalkDeviceFacade& facade_;
    CctalkEventQueue& queue_;
};
