#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include "cctalk_event.hpp"

class CctalkEventQueue {
public:
    void push(const CctalkEvent& evt);

    CctalkEvent popBlocking();

    bool tryPop(CctalkEvent& out);

private:
    std::queue<CctalkEvent> queue_;
    std::mutex mutex_;
    std::condition_variable cv_;
};
