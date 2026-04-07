#include "cctalkcontroller.hpp"

#include <memory>

CctalkController::CctalkController(std::unique_ptr<ICctalkUart> uart,
                                   uint8_t hostAddr,
                                   uint8_t coinAcceptorAddr,
                                   uint8_t hopperAddr)
    : uart_(std::move(uart))
{
    bus_ = std::make_unique<CctalkBus>(*uart_, hostAddr);

    // Create high-level façade
    facade_ = std::make_unique<CctalkDeviceFacade>(
        *bus_, hostAddr, coinAcceptorAddr, hopperAddr
    );

    // Create worker threads
    acceptorThread_   = std::make_unique<CoinAcceptorThread>(*facade_, eventQueue_);
    hopperThread_     = std::make_unique<HopperThread>(*facade_, eventQueue_);
    dispatcherThread_ = std::make_unique<EventDispatcherThread>(eventQueue_);
}

CctalkController::~CctalkController() {
    stop();
}

void CctalkController::start() {
    if (running_) return;
    running_ = true;

    acceptorThread_->start();
    hopperThread_->start();
    dispatcherThread_->start();
}

void CctalkController::stop() {
    if (!running_) return;
    running_ = false;

    acceptorThread_->stop();
    hopperThread_->stop();
    dispatcherThread_->stop();
}

void CctalkController::initialiseDevices()
{
    if (!running_) return;

    // Example: use the façade directly
    std::string category;
    if (facade_.get()->getCategoryId(category) == CctalkError::OK) {
        ESP_LOGI("MAIN", "Coin acceptor category: %s", category.c_str());
    }
}