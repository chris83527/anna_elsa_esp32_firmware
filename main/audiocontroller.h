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

#include <vector>

#include "esp_pthread.h"
#include "I2CManager.h"
#include "driver/i2s_std.h"
#include "soc/io_mux_reg.h"

#include "spiffs.h"

class Sounds {
public:
  // Audio files
  static constexpr const char *SND_ANNA_PUNCHES_HANS = "anna_punches_hans.wav";
  static constexpr const char *SND_NOW_THATS_ICE = "nowthatsice.wav";
  static constexpr const char *SND_COME_ON_BUDDY = "come-on-buddy-faster.wav";
  static constexpr const char *SND_DO_THE_MAGIC = "do-the-magic.wav";
  static constexpr const char *SND_CANT_RUN_FROM_THIS =
      "you-cant-run-from-this.wav";
  static constexpr const char *SND_BRING_BACK_SUMMER = "bring-back-summer.wav";
  static constexpr const char *SND_COLDER_BY_THE_MINUTE =
      "colder-by-the-minute.wav";
  static constexpr const char *SND_MAYBE_NOT_RIGHT_THIS_SECOND =
      "not-right-this-second.wav";
  static constexpr const char *SND_PRINCE_HANS = "spiffs/prince_hans.wav";
  static constexpr const char *SND_STAY_OUT_OF_SIGHT =
      "stay-out-of-sight-olaf.wav";
  static constexpr const char *SND_SUPPLY_AND_DEMAND = "supply-and-demand.wav";
  static constexpr const char *SND_WONT_GET_AWAY_WITH_THIS =
      "wont-get-away-with-this.wav";
  static constexpr const char *SND_LET_IT_GO = "letitgo.wav";
  static constexpr const char *SND_THEYRE_TROLLES = "theyretrolls.wav";
  static constexpr const char *SND_CANT_FEEL_LEGS = "cantfeellegs.wav";
  static constexpr const char *SND_THATS_BETTER = "thatsbetter.wav";
  static constexpr const char *SND_KERCHING = "kerching.wav";
  static constexpr const char *SND_REEL_STOP = "reelstop.wav";
  static constexpr const char *SND_STARTUP = "startup.wav";
};

class AudioController {
public:
  AudioController(I2CManager &i2cmgr);
  virtual ~AudioController();

  void initialise(void);
  void playAudioFile(const char *filepath);
  void playAudioFileAsync(const char *filepath);
  void stopPlaying(void);
  void getVolume(int &volume);
  void setVolume(int volume);
  bool isMute();
  void setMute();
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
  I2CManager i2cManager;
  i2c_device_config_t deviceConfig;
  i2c_master_dev_handle_t deviceHandle;

  static constexpr int AUDIO_BUFFER = 1024;
  static constexpr int TAS5731M_I2C_ADDRESS = 0x1a;
  static constexpr int MASTER_VOL_REG_ADDR = 0x07;
  static constexpr int MUTE_TIME_REG_ADDR = 0x51;

  static constexpr int TAS5731M_VOLUME_MAX = 100;
  static constexpr int TAS5731M_VOLUME_MIN = 100;

  static constexpr gpio_num_t TAS5731M_RST_GPIO = GPIO_NUM_14;
  static constexpr gpio_num_t TAS5731M_PDWN_GPIO = GPIO_NUM_2;

  static constexpr uint8_t tas5731m_volume[] = {
      0xff, 0x9f, 0x8f, 0x7f, 0x6f, 0x5f, 0x5c, 0x5a, 0x58, 0x54, 0x50,
      0x4c, 0x4a, 0x48, 0x44, 0x40, 0x3d, 0x3b, 0x39, 0x37, 0x35};

  bool playing;
  int vol;
};

#endif /* __AUDIOCONTROLLER_H__ */
