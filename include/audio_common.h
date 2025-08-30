/*
 * Copyright (C) 2025 Alejandro Zeise
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#ifndef AUDIO_COMMON_H
#define AUDIO_COMMON_H

#include "util.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"

#define AUDIO_WORDSIZE 32

/**
 * A function that produces AUDIO_WORDSIZE-bit N-channel interleaved audio samples. 
 * Used by the encoders for obtaining samples to encode and send over bluetooth.
 *
 * NOTE: Because multiple channels may be requested, the samples will be interleaved.
 * So for 2 channels and a requested 2 samples, 4 entries will be placed in the buffer pointer.
 * The function will then return a value of 2, as two interleaved samples were placed in the buffer.
 * The buffer MUST be appropriately sized!
 *
 * @param uint     Number of channels.
 * @param int32_t* Buffer to place the samples in. Function may not place more than 
 *                 AUDIO_SAMPLE_PROVIDER_FUNC_MAX_SAMPLES * 2 in the buffer.
 * @param unsigned int Maxium amount of N-channel samples to place in the buffer
 * @return The number of INTERLEAVED samples placed in the buffer, or zero if none are available
 */
typedef int (*InterleavedSampleProvider)(unsigned int, int32_t*, unsigned int);

/**
 * A function that consumes a buffer.
 *
 * @param int8_t* Pointer to a buffer containing the samples. If they need
 *                 to be retained upon function exit they MUST BE COPIED.
 * @param unsigned int The amount of bytes in the provided buffer
 */
typedef void (*BufferConsumer)(uint8_t*, unsigned int);

static const InterleavedSampleProvider AUDIO_PLAYBACK_PROVIDER = &Util_sineAudioProducer;

static const PIO AUDIO_ADC_PIO = pio0;
static const PIO AUDIO_DAC_PIO = pio1;
static const uint32_t AUDIO_PLAYBACK_SAMPLE_RATE = 48000;
static const uint32_t AUDIO_SAMPLE_RATE = AUDIO_PLAYBACK_SAMPLE_RATE;
static const uint8_t AUDIO_CHANNELS = 2;

#endif // AUDIO_COMMON_H
