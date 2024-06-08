/*
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file audiocontroller.h
 *
 * Definitions
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __AUDIOCONTROLLER_H__
#define __AUDIOCONTROLLER_H__

#include "driver/i2s_std.h"
#include "I2CManager.h"

#include "spiffs.h"

constexpr int TAS5731M_I2C_ADDRESS = 0x1a;

class Sounds {
public:
  // Audio files
  static constexpr const char *SND_ANNA_PUNCHES_HANS =      "spiffs/anna_punches_hans.wav";
  static constexpr const char *SND_NOW_THATS_ICE = "spiffs/nowthatsice.wav";
  static constexpr const char *SND_COME_ON_BUDDY =      "spiffs/come-on-buddy-faster.wav";
  static constexpr const char *SND_DO_THE_MAGIC = "spiffs/do-the-magic.wav";
  static constexpr const char *SND_CANT_RUN_FROM_THIS =      "spiffs/you-cant-run-from-this.wav";
  static constexpr const char *SND_BRING_BACK_SUMMER =      "spiffs/bring-back-summer.wav";
  static constexpr const char *SND_COLDER_BY_THE_MINUTE =      "spiffs/colder-by-the-minute.wav";
  static constexpr const char *SND_MAYBE_NOT_RIGHT_THIS_SECOND =      "spiffs/not-right-this-second.wav";
  static constexpr const char *SND_PRINCE_HANS = "spiffs/prince_hans.wav";
  static constexpr const char *SND_STAY_OUT_OF_SIGHT =      "spiffs/stay-out-of-sight-olaf.wav";
  static constexpr const char *SND_SUPPLY_AND_DEMAND =      "spiffs/supply-and-demand.wav";
  static constexpr const char *SND_WONT_GET_AWAY_WITH_THIS =      "spiffs/wont-get-away-with-this.wav";
  static constexpr const char *SND_LET_IT_GO = "spiffs/letitgo.wav";
  static constexpr const char *SND_THEYRE_TROLLES = "spiffs/theyretrolls.wav";
  static constexpr const char *SND_CANT_FEEL_LEGS = "spiffs/cantfeellegs.wav";
  static constexpr const char *SND_THATS_BETTER = "spiffs/thatsbetter.wav";
  static constexpr const char *SND_KERCHING = "spiffs/kerching.wav";
  static constexpr const char *SND_REEL_STOP = "spiffs/reelstop.wav";
  static constexpr const char *SND_STARTUP = "spiffs/startup.wav";
};

class AudioController {
public:
  AudioController(I2CManager& i2cmgr);  
  virtual ~AudioController();

  void initialise(void);
  void playAudioFile(const char *filepath);
  void playAudioFileAsync(const char *filepath);
  void stopPlaying(void);
  void setVolume(int volume);
  bool isPlaying(void);
  uint8_t getErrors();
public:
  
private:
  esp_err_t i2sInit(void);
  esp_err_t i2cInit(void);
  
private:
  i2s_chan_handle_t channelHandle;
  i2s_chan_config_t channelConfig;
  i2s_std_config_t i2sConfig;
  I2CManager& i2cManager;  
  i2c_device_config_t deviceConfig;
  i2c_master_dev_handle_t deviceHandle;   
  
  static constexpr int AUDIO_BUFFER = 2048;
 
};

#endif /* __AUDIOCONTROLLER_H__ */
