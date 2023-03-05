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

#include <cstdlib>
#include <thread>
#include <memory>
#include <utility>
#include <chrono>

#include "nvs_flash.h"
#include "nvs.h"
#include "nvs_handle.hpp"
#include "ds3231.h"
#include "cctalkcontroller.h"
#include "cctalk_enums.h"
#include "httpcontroller.h"

class ReelController;
class DisplayController;
class AudioController;
class MoneyController;
class Game;
class DisplayController;
class oledcontroller;
class HttpController;

class MainController {
public:
    MainController();
    MainController(const MainController& orig);

    void start();

    void print_binary(uint8_t value);
    //void dumpEEPROMValues();
    void setDateTime();
    time_t getDateTime();
    //void printDate(Stream *stream);
    uint8_t getVolume();
    //EEProm_Data* getEEPromData();

    void error(int errorCode);

    void writeValueToNVS(const char * key, uint16_t value);
    uint16_t readValueFromNVS(const char * key);

    std::shared_ptr<AudioController> getAudioController();
    std::shared_ptr<DisplayController> getDisplayController();
    std::shared_ptr<ReelController> getReelController();
    std::shared_ptr<CCTalkController> getCCTalkController();
    std::shared_ptr<Game> getGame();
    std::shared_ptr<MoneyController> getMoneyController();
    std::shared_ptr<oledcontroller> getOledController();
    //std::shared_ptr<WIFI::Wifi> getWifiController();

    i2c_dev_t* getDs3231();


private:

    i2c_dev_t ds3231;

    //EEProm_Data eeprom_data;    
    void blinkCPUStatusLEDTask(void);
    void updateStatisticsDisplayTask(void);

    int reels = 0;

    unsigned int state = 0;
    unsigned int animationStage = 0;

    bool startPolling = false;

    volatile float counter = 0;
    volatile float motorSpeed = 0;

    uint8_t letitgoCountdown = 0;

    uint8_t oldReelStatus;

    uint8_t volume = 0;

    std::shared_ptr<Game> game;
    std::shared_ptr<ReelController> reelController;
    std::shared_ptr<DisplayController> displayController;
    std::shared_ptr<CCTalkController> cctalkController;
    std::shared_ptr<AudioController> audioController;
    std::shared_ptr<MoneyController> moneyController;
    std::shared_ptr<oledcontroller> oledController;    
    std::shared_ptr<HttpController> httpController;

    std::unique_ptr<nvs::NVSHandle> nvs_handle;
    
    const char* NVS_PARTITION_SETTINGS = "settings";

    enum class MachineState : uint8_t {
        INITIALISING,
        IDLE,
        ANIMATION,
        IN_GAME,
        PAYING_OUT
    };

    std::unique_ptr<std::thread> updateStatisticsThread;
    std::unique_ptr<std::thread> blinkCPUStatusLEDThread;    
};


#endif /* MAINCONTROLLER_H */

