#pragma once

#include <thread>
#include <atomic>
#include <functional>

#include "cctalk_event_queue.hpp"
#include "cctalk_event.hpp"

class EventDispatcherThread {
public:
    using Handler = std::function<void(const CctalkEvent&)>;

    EventDispatcherThread(CctalkEventQueue& queue, Handler handler)
        : queue_(queue), handler_(std::move(handler)) {}

    void start() {
        running_ = true;
        thread_ = std::thread(&EventDispatcherThread::run, this);
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) {
            // Wake the thread by pushing a dummy event if needed
            queue_.push(CctalkEvent{});
            thread_.join();
        }
    }

private:
    void run() {
        while (running_) {
            CctalkEvent evt = queue_.popBlocking();

            if (!running_) break;

            if (handler_) {
                handler_(evt);
            }
        }
    }

    std::atomic<bool> running_{false};
    std::thread thread_;
    CctalkEventQueue& queue_;
    Handler handler_;
};
