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
 * @file wave.c
 *
 * Definitions 
 *
 * Copyright (c) 2021 Chris Woods <chris@cmwoods.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/i2s.h"
#include "esp_log.h"

#include "wave.h"
#include "config.h"

#define CCCC(c1, c2, c3, c4)    ((c4 << 24) | (c3 << 16) | (c2 << 8) | c1)

static const char *TAG = "wave";

i2s_port_t I2S_NUM = 0; // i2s port number

i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 36000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2,       // high interrupt priority
    .dma_buf_count = 8,                             // 8 buffers
    .dma_buf_len = 64                            // 1K per buffer, so 8K of buffer space    
};

i2s_pin_config_t pin_config = {
    .bck_io_num = AUDIO_SCLK,
    .ws_io_num = AUDIO_LRCLK,
    .data_out_num = AUDIO_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
};


/* these are data structures to process wav file */
typedef enum headerState_e {
    HEADER_RIFF, HEADER_FMT, HEADER_DATA, DATA
} headerState_t;

typedef struct wavRIFF_s 
{
    uint32_t chunkID;
    uint32_t chunkSize;
    uint32_t format;
} wavRIFF_t;

typedef struct wavProperties_s
{
    uint32_t chunkID;
    uint32_t chunkSize;
    uint16_t audioFormat;
    uint16_t numChannels;
    uint32_t sampleRate;
    uint32_t byteRate;
    uint16_t blockAlign;
    uint16_t bitsPerSample;
} wavProperties_t;

headerState_t state = HEADER_RIFF;
wavProperties_t wavProperties;

/* write sample data to I2S */
size_t i2s_write_sample_nb(uint32_t sample) 
{
    size_t bytes_written;
    i2s_write(I2S_NUM, (const char *)&sample, sizeof(uint32_t), &bytes_written, 100);
    return bytes_written;
}

/* read 4 bytes of data from wav file */
int readChunk(FILE *file, uint32_t *chunkId){
  int n = fread((uint8_t *)chunkId, sizeof(uint32_t), 1, file);
  return n;
}

/* these are function to process wav file */
int readRiff(FILE *file, wavRIFF_t *wavRiff){
  int n = fread((uint8_t *)wavRiff, sizeof(wavRIFF_t), 1, file);
  return n;
}

int readProps(FILE *file, wavProperties_t *wavProps){
  int n = fread((uint8_t *)wavProps, sizeof(wavProperties_t), 1, file);
  return n;
}


void play_audio(const char *filename)
{
    FILE *file = fopen(filename, "rb");

    if (file == NULL)
    {
        ESP_LOGE(TAG, "An error occurred trying to open %s read only", filename);
        return;
    }
    else
    {        
        size_t n;
        wavRIFF_t wavRIFF;
        uint32_t chunkId, chunkSize;
        uint32_t data; 
        bool has_error = false;

        while (!has_error) 
        {
            switch (state)
            {
                case HEADER_RIFF:
                    ESP_LOGI(TAG, "Reading RIFF header from file");
                    n = readRiff(file, &wavRIFF);
                    if (n == sizeof(wavRIFF_t)) 
                    {
                        if (wavRIFF.chunkID == CCCC('R', 'I', 'F', 'F') && wavRIFF.format == CCCC('W', 'A', 'V', 'E'))
                        {
                            state = HEADER_FMT;
                            ESP_LOGI(TAG, "HEADER_RIFF");
                        }
                    } 
                    else{
                        ESP_LOGE(TAG, "An error occurred reading RIFF header from file");
                        has_error = true;
                    }
                    break;
                case HEADER_FMT:
                    n = readProps(file, &wavProperties);
                    if (n == sizeof(wavProperties_t))
                    {
                        state = HEADER_DATA;
                        ESP_LOGI(TAG, "HEADER_FMT");
                    }
                    else{
                        ESP_LOGE(TAG, "An error occurred reading header format from file");
                        has_error = true;
                    }
                    break;
                case HEADER_DATA:
                    
                    n = readChunk(file, &chunkId);
                    if (n == 4)
                    {
                        if (chunkId == CCCC('d', 'a', 't', 'a'))
                        {
                            ESP_LOGI(TAG, "HEADER_DATA");
                        }
                    }
                    else{
                        ESP_LOGE(TAG, "An error occurred reading header data from file");
                        has_error = true;
                    }
                    n = readChunk(file, &chunkSize);
                    if (n == 4)
                    {
                        ESP_LOGI(TAG, "Prepare data");
                        state = DATA;

                        i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
                        i2s_set_pin(I2S_NUM, &pin_config);
                        i2s_set_sample_rates(I2S_NUM, wavProperties.sampleRate);
                    }
                    else{
                        has_error = true;
                    }
                    
                    break;
                    /* after processing wav file, it is time to process music data */
                case DATA:
                    ESP_LOGI(TAG, "Sending data");
                    n = readChunk(file, &data);
                    if (n == 4)
                    {
                        i2s_write_sample_nb(data); 
                    }
                    else 
                    {
                        has_error = true;
                    }
                    
                    break;
            }
        }

        ESP_LOGI(TAG, "Closing file.");
        fclose(file);
        i2s_driver_uninstall(I2S_NUM); //stop & destroy i2s driver 
    }
}