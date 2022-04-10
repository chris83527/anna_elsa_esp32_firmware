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

#include "ds3231.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "nvs_handle.hpp"

class CCTalkController;
class ReelController;
class DisplayController;
class AudioController;
class MoneyController;
class Game;
class DisplayController;

class MainController {
public:
    MainController();
    MainController(const MainController& orig);

    void start();

    void payout();

    void print_binary(uint8_t value);
    //void dumpEEPROMValues();
    void setDateTime();
    time_t getDateTime();
    //void printDate(Stream *stream);
    uint8_t getVolume();
    //EEProm_Data* getEEPromData();
    void checkHopperLevel(); // TODO: this should be in cctalkcontroller
    void processHopperErrors();
    void error(int errorCode);

    void writeValueToNVS(const char * key, uint16_t value);
    uint16_t readValueFromNVS(const char * key);

    AudioController* getAudioController();
    DisplayController* getDisplayController();
    ReelController* getReelController();
    CCTalkController* getCCTalkController();
    Game* getGame();
    MoneyController* getMoneyController();
    
    static void cctalkStatusCheckTask(void *pvParameter);

private:

    i2c_dev_t ds3231;
    
    //EEProm_Data eeprom_data;

    int reels = 0;

    unsigned int state = 0;
    unsigned int animationStage = 0;  

    bool startPolling = false;

    volatile float counter = 0;
    volatile float motorSpeed = 0;

    uint8_t letitgoCountdown = 0;

    uint8_t oldReelStatus;

    uint8_t volume = 0;

    Game * game;
    ReelController * reelController;
    DisplayController * displayController;
    CCTalkController * cctalkController;
    AudioController * audioController;
    MoneyController * moneyController;

    std::unique_ptr<nvs::NVSHandle> nvs_handle;

    const char* NVS_PARTITION_SETTINGS = "settings";

    enum class MachineState : uint8_t {
        INITIALISING,
        IDLE,
        ANIMATION,
        IN_GAME,
        PAYING_OUT
    };
    
    int hopperEventCounter = 0;
        
};



void blinkCPUStatusLEDTask(void *pvParameters);



#endif /* MAINCONTROLLER_H */

