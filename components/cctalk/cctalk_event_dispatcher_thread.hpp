#pragma once
#include <thread>
#include <atomic>
#include "cctalk_event_queue.hpp"
#include "esp_log.h"

class EventDispatcherThread {
public:
    using Handler = std::function<void(const CctalkEvent&)>;

    EventDispatcherThread(CctalkEventQueue& queue, Handler handler);
    void start();
    void stop();
};
