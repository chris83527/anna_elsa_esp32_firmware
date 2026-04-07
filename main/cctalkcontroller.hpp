#pragma once

#include <memory>
#include <thread>
#include <atomic>

#include "cctalk.hpp"
#include "cctalk_device_facade.hpp"
#include "cctalk_event_queue.hpp"

#include "coin_acceptor_thread.hpp"
#include "hopper_thread.hpp"
#include "cctalk_event_dispatcher_thread.hpp"

class CctalkController {
public:
    CctalkController(std::unique_ptr<ICctalkUart> uart,
                  uint8_t hostAddr,
                  uint8_t coinAcceptorAddr,
                  uint8_t hopperAddr);

    ~CctalkController();

    void start();
    void stop();

private:
    std::unique_ptr<ICctalkUart> uart_;
    std::unique_ptr<CctalkBus> bus_;
    std::unique_ptr<CctalkDeviceFacade> facade_;

    CctalkEventQueue eventQueue_;

    std::unique_ptr<CoinAcceptorThread> acceptorThread_;
    std::unique_ptr<HopperThread> hopperThread_;
    std::unique_ptr<EventDispatcherThread> dispatcherThread_;

    std::atomic<bool> running_{false};
};
