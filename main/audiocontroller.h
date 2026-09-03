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
#include "tas5731m.hpp"
#include <atomic>
#include <future>

class MainController;

class Sounds
{
public:
    // Audio files
    static constexpr const char* SND_ANNA_PUNCHES_HANS = "anna_punches_hans.wav";
    static constexpr const char* SND_NOW_THATS_ICE = "nowthatsice.wav";
    static constexpr const char* SND_COME_ON_BUDDY = "come-on-buddy-faster.wav";
    static constexpr const char* SND_DO_THE_MAGIC = "do-the-magic.wav";
    static constexpr const char* SND_CANT_RUN_FROM_THIS =
        "you-cant-run-from-this.wav";
    static constexpr const char* SND_BRING_BACK_SUMMER = "bring-back-summer.wav";
    static constexpr const char* SND_COLDER_BY_THE_MINUTE =
        "colder-by-the-minute.wav";
    static constexpr const char* SND_MAYBE_NOT_RIGHT_THIS_SECOND =
        "not-right-this-second.wav";
    static constexpr const char* SND_PRINCE_HANS = "spiffs/prince_hans.wav";
    static constexpr const char* SND_STAY_OUT_OF_SIGHT =
        "stay-out-of-sight-olaf.wav";
    static constexpr const char* SND_SUPPLY_AND_DEMAND = "supply-and-demand.wav";
    static constexpr const char* SND_WONT_GET_AWAY_WITH_THIS =
        "wont-get-away-with-this.wav";
    static constexpr const char* SND_LET_IT_GO = "letitgo.wav";
    static constexpr const char* SND_THEYRE_TROLLS = "theyretrolls.wav";
    static constexpr const char* SND_CANT_FEEL_LEGS = "cantfeellegs.wav";
    static constexpr const char* SND_THATS_BETTER = "thatsbetter.wav";
    static constexpr const char* SND_KERCHING = "kerching.wav";
    static constexpr const char* SND_REEL_STOP = "reelstop.wav";
    static constexpr const char* SND_STARTUP = "startup.wav";
};

class AudioController
{
public:
    explicit AudioController(I2CBus& bus, MainController* mainController);

    ~AudioController() = default;

    void initialise();

    void playAudioFileSync(const char* filepath);
    std::future<void> playAudioFileAsync(const char* filepath);

    //void stopPlaying();

    int getVolume();
    void setVolume(int volume);
    void mute();
    void unmute();

private:
    int vol{};

    static constexpr gpio_num_t TAS5731M_RST_GPIO = GPIO_NUM_14;
    static constexpr gpio_num_t TAS5731M_PDWN_GPIO = GPIO_NUM_2;

    static constexpr int TAS5731M_I2C_ADDRESS = 0x1a;

    static constexpr int AUDIO_BUFFER = 1024;

    TAS5731M tas5731m;

    MainController* mainController;

    void playAudioFile(const char* filepath, std::size_t myGen);

    std::mutex mutex_;
    std::atomic<std::size_t> generation_;
};

#endif /* __AUDIOCONTROLLER_H__ */
