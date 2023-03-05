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
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

#include "driver/i2s.h"
#include "spiffs_stream.h"
#include "i2s_stream.h"
#include "ogg_decoder.h"
#include "board.h"
#include "audio_def.h"
#include "esp_audio.h"

#include "spiffs.h"

class Sounds {
public:
    // Audio files
    static constexpr const char* SND_NOW_THATS_ICE = "spiffs/nowthatsice.ogg";
    static constexpr const char* SND_LOSE = "spiffs/lose.ogg";
    static constexpr const char* SND_LET_IT_GO = "spiffs/letitgo.ogg";
    static constexpr const char* SND_THEYRE_TROLLES = "spiffs/theyretrolls.ogg";
    static constexpr const char* SND_CANT_FEEL_LEGS = "spiffs/cantfeellegs.ogg";
    static constexpr const char* SND_THATS_BETTER = "spiffs/thatsbetter.ogg";
    static constexpr const char* SND_KERCHING = "spiffs/kerching.ogg";
    static constexpr const char* SND_REEL_STOP = "spiffs/reelstop.ogg";
    static constexpr const char* SND_STARTUP = "spiffs/startup.ogg";
};

class AudioController {
public:

    AudioController(void);
    AudioController(const AudioController &orig);
    virtual ~AudioController();

    void initialise(void);
    void playAudioFile(const char* filepath);
    void playAudioFileSync(const char* filepath);
    void stopPlaying(void);
    void setVolume(int volume);
    bool isPlaying(void);
    uint8_t getErrors();

private:
    esp_audio_handle_t player;
    esp_audio_cfg_t cfg;
    spiffs_stream_cfg_t spiffs_reader;
    audio_board_handle_t board_handle;
    i2s_stream_cfg_t i2s_writer;
    ogg_decoder_cfg_t ogg_dec_cfg;
    i2s_port_t I2S_NUM;
    
    void i2s_init(void);
};

#endif /* __AUDIOCONTROLLER_H__ */
