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

#include <stdlib.h>
#include <nvs_flash.h>

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
    void refreshStatus();
    void setCredit(uint16_t value);
    void setBank(uint16_t value);
    void setTransfer(uint16_t value);
    uint16_t getBank();
    uint16_t getTransfer();
    uint16_t getCredit();
    void addToBank(uint16_t value);
    void addToCredit(uint16_t value);
    void addToTransfer(uint16_t value);
    void payout();
    void incrementGameCounter();
    void print_binary(uint8_t value);
    //void dumpEEPROMValues();
    void setDateTime();
    time_t getDateTime(); 
    //void printDate(Stream *stream);
    uint8_t getVolume();
    //EEProm_Data* getEEPromData();
    void checkHopperLevel();
    void processHopperErrors();
    void error(int errorCode);
    
    void writeValueToNVS(const char * key, int value);
    int readValueFromNVS(const char * key);
    
    AudioController* getAudioController();
    DisplayController* getDisplayController();
    ReelController* getReelController();
    CCTalkController* getCCTalkController();    
    Game* getGame();
    MoneyController* getMoneyController();
    
private:
   
    //EEProm_Data eeprom_data;

    int reels = 0;

    unsigned int state = 0;
    unsigned int animationStage = 0;
    
    const unsigned long interval = 3000;

    const unsigned long tableauRefreshInterval = 100;
    unsigned long tableauPreviousMillis = 0;

    unsigned long previousMillis;
    unsigned long currentMillis;

    const unsigned long coinRefreshInterval = 200;
    unsigned long coinCheckPreviousMillis = 0;

    const unsigned long hopperStatusPollInterval = 100;
    unsigned long hopperStatusPollPreviousMillis = 0;

    const unsigned long serialReadInterval = 400;
    unsigned long serialReadPreviousMillis = 0;

    const unsigned long reelRefreshInterval = 2;
    unsigned long reelRefreshPreviousMillis = 0;

    const unsigned long timeUpdateInterval = 1000;
    unsigned long timeUpdatePreviousMillis = 0;

    const unsigned long msgUpdateRefreshInterval = 4000;
    unsigned long msgUpdatePreviousMillis = 0;

    
    uint8_t validatorEventCounter = 0;
    uint8_t hopperEventCounter = 0;
    
    bool startPolling = false;

    volatile float counter = 0;
    volatile float motorSpeed = 0;

    uint8_t letitgoCountdown = 0;

    uint8_t oldReelStatus;

    uint8_t volume = 0;

    bool postInProgress = true;
    bool payoutInProgress = false;

    Game * game;
    ReelController * reelController;
    DisplayController * displayController;
    CCTalkController * cctalkController;    
    AudioController * audioController;
    MoneyController * moneyController;
    
    nvs_handle_t nvs_handle;        

};

enum class MachineState : uint8_t {
    INITIALISING,
    IDLE,
    ANIMATION,
    IN_GAME,
    PAYING_OUT
};

void blinkTask(void *pvParameters);

#endif /* MAINCONTROLLER_H */

