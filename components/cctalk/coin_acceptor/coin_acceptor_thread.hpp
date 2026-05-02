#pragma once
#include <thread>
#include <atomic>

#include "esp_log.h"

#include "cctalk_device_facade.hpp"
#include "cctalk_event_queue.hpp"



class CoinAcceptorThread {
public:
    CoinAcceptorThread(CctalkDeviceFacade& facade,
                       CctalkEventQueue& queue)
        : facade_(facade), queue_(queue) {}

    void start() {
        running_ = true;
        thread_ = std::thread(&CoinAcceptorThread::run, this);
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

private:
    static constexpr const char *TAG = "CoinAcceptorThread";

    void run() {
        std::vector<CoinChannelInfo> channels;
        facade_.getChannelValues(channels);

        while (running_) {
            std::vector<CoinEvent> events;
            facade_.readBufferedCreditEvents(channels, events);

            for (auto& ev : events) {
                ESP_LOGI(TAG, "Coin id: %d", ev.coin_id);
                if (ev.routing == CoinRouting::Accepted)
                {
                    queue_.push(CctalkEvent::makeCoin(ev));
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }

    std::atomic<bool> running_{false};
    std::thread thread_;
    CctalkDeviceFacade& facade_;
    CctalkEventQueue& queue_;
};
