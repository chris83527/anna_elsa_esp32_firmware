/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   MainController.h
 * Author: chris
 *
 * Created on July 28, 2018, 6:33 PM
 */

#ifndef MAINCONTROLLER_H
#define MAINCONTROLLER_H

#include <chrono>
#include <thread>

#include "i2c_bus.hpp"
#include "NvsController.h"
#include "audiocontroller.h"
#include "displaycontroller.h"
#include "ds3231.h"
#include "game.h"
#include "reelcontroller.h"
#include "wifi_manager.hpp"
#include "httpcontroller.hpp"
#include "dns_server.hpp"
#include "paymentcontroller.h"
#include "httpcontroller.hpp"

class MainController
{
public:
    explicit MainController(std::unique_ptr<ICctalkUart> uart);
    //MainController(const MainController &orig);
    ~MainController();

    void start();

    //void print_binary(uint8_t value);
    void setDateTime();
    //time_t getDateTime();

    void error(int errorCode);

    AudioController& getAudioController();
    ReelController& getReelController();
    DisplayController& getDisplayController();
    PaymentController& getPaymentController();
    HttpController& getHttpController();
    Game& getGame();
    DS3231& getDs3231();
    NvsController& getNvsController();

private:
    // EEProm_Data eeprom_data;
    static void blinkCPUStatusLEDCallback(void* param);
    static void updateStatisticsDisplayCallback(void* param);

    uint8_t letitgoCountdown = 0;

    uint8_t oldReelStatus = 0;

    I2CBus i2c_bus;
    NvsController nvsController;
    PaymentController paymentController;
    DisplayController displayController;
    AudioController audioController;
    ReelController reelController;
    Game game;
    DS3231 ds3231;
    WifiManager wifi;
    HttpController httpController;
    DnsServer dnsServer;

    enum class MachineState : uint8_t
    {
        INITIALISING,
        IDLE,
        ATTRACT,
        IN_GAME,
        PAYING_OUT
    };

    std::thread gameThread;
};

#endif /* MAINCONTROLLER_H */
