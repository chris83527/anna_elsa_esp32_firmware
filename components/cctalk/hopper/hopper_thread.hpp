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
        HopperPayoutStatus last{};

        while (running_) {
            HopperPayoutStatus st{};
            facade_.getHopperStatus(st);

            if (st.dispenseCount != last.dispenseCount || st.coinsRemaining != last.coinsRemaining || st.coinsPaid != last.coinsPaid || st.coinsUnpaid != last.coinsUnpaid) {
                ESP_LOGD(TAG, "Hopper dispenseCount changed from %d to %d", last.dispenseCount, st.dispenseCount);
                ESP_LOGD(TAG, "Hopper coinsRemaining changed from %d to %d", last.coinsRemaining, st.coinsRemaining);
                ESP_LOGD(TAG, "Hopper coinsPaid changed from %d to %d", last.coinsPaid, st.coinsPaid);
                ESP_LOGD(TAG, "Hopper coinsUnpaid changed from %d to %d", last.coinsUnpaid, st.coinsUnpaid);

                queue_.push(CctalkEvent::makeHopper(st));
                last = st;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    std::atomic<bool> running_{false};
    std::thread thread_;
    CctalkDeviceFacade& facade_;
    CctalkEventQueue& queue_;
};
