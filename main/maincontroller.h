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

#include "NvsController.h"
#include "audiocontroller.h"
#include "cctalkcontroller.h"
#include "displaycontroller.h"
#include "ds3231.h"
#include "game.h"
#include "moneycontroller.h"
#include "reelcontroller.h"

class MainController {
public:
  MainController();
  //MainController(const MainController &orig);

  void start();

  //void print_binary(uint8_t value);
  void setDateTime();
  //time_t getDateTime();

  void error(int errorCode);

  AudioController &getAudioController();
  ReelController &getReelController();
  CCTalkController &getCCTalkController();
  DisplayController &getDisplayController();
  MoneyController &getMoneyController();
  Game &getGame();
  DS3231 &getDs3231();

  // std::shared_ptr<WIFI::Wifi> getWifiController();

private:
  // EEProm_Data eeprom_data;
  static void blinkCPUStatusLEDTask();
  void updateStatisticsDisplayTask();

private:
  int reels = 0;

  unsigned int state = 0;
  unsigned int animationStage = 0;

  uint8_t letitgoCountdown = 0;

  uint8_t oldReelStatus;

  I2CManager i2c_manager;
  NvsController nvsController;
  MoneyController moneyController;
  DisplayController displayController;
  AudioController audioController;
  CCTalkController cctalkController;
  ReelController reelController;
  Game game;
  DS3231 ds3231;

  // HttpController httpController;

  enum class MachineState : uint8_t {
    INITIALISING,
    IDLE,
    ATTRACT,
    IN_GAME,
    PAYING_OUT
  };

  std::thread updateStatisticsThread;
  std::thread blinkCPUStatusLEDThread;
  std::thread gameThread;
};

#endif /* MAINCONTROLLER_H */
