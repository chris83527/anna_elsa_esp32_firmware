//
// Created by chris on 06.04.26.
//
#include "cctalk_event_queue.hpp"

void CctalkEventQueue::push(const CctalkEvent& evt) {
    {
        std::lock_guard lock(mutex_);
        queue_.push(evt);
    }
    cv_.notify_one();
}

CctalkEvent CctalkEventQueue::popBlocking() {
    std::unique_lock lock(mutex_);
    cv_.wait(lock, [&]{ return !queue_.empty(); });
    CctalkEvent evt = queue_.front();
    queue_.pop();
    return evt;
}

bool CctalkEventQueue::tryPop(CctalkEvent& out) {
    std::lock_guard lock(mutex_);
    if (queue_.empty()) return false;
    out = queue_.front();
    queue_.pop();
    return true;
}

