/*
 * Copyright (C) 2016 BlueKitchen GmbH
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
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */
#include "util.h"
#include "audio_common.h"

#include "pico/time.h"
#include <pico/types.h>

// input signal: pre-computed int16 sine wave, 44100 Hz at 441 Hz
static const int16_t sine_int16_44100[] = {
     0,    2057,    4107,    6140,    8149,   10126,   12062,   13952,   15786,   17557,
 19260,   20886,   22431,   23886,   25247,   26509,   27666,   28714,   29648,   30466,
 31163,   31738,   32187,   32509,   32702,   32767,   32702,   32509,   32187,   31738,
 31163,   30466,   29648,   28714,   27666,   26509,   25247,   23886,   22431,   20886,
 19260,   17557,   15786,   13952,   12062,   10126,    8149,    6140,    4107,    2057,
     0,   -2057,   -4107,   -6140,   -8149,  -10126,  -12062,  -13952,  -15786,  -17557,
-19260,  -20886,  -22431,  -23886,  -25247,  -26509,  -27666,  -28714,  -29648,  -30466,
-31163,  -31738,  -32187,  -32509,  -32702,  -32767,  -32702,  -32509,  -32187,  -31738,
-31163,  -30466,  -29648,  -28714,  -27666,  -26509,  -25247,  -23886,  -22431,  -20886,
-19260,  -17557,  -15786,  -13952,  -12062,  -10126,   -8149,   -6140,   -4107,   -2057,
};

static const int num_samples_sine_int16_44100 = sizeof(sine_int16_44100) / 2;

// input signal: pre-computed int16 sine wave, 48000 Hz at 441 Hz
static const int16_t sine_int16_48000[] = {
     0,    1905,    3804,    5690,    7557,    9398,   11207,   12978,   14706,   16383,
 18006,   19567,   21062,   22486,   23834,   25101,   26283,   27376,   28377,   29282,
 30087,   30791,   31390,   31884,   32269,   32545,   32712,   32767,   32712,   32545,
 32269,   31884,   31390,   30791,   30087,   29282,   28377,   27376,   26283,   25101,
 23834,   22486,   21062,   19567,   18006,   16383,   14706,   12978,   11207,    9398,
  7557,    5690,    3804,    1905,       0,   -1905,   -3804,   -5690,   -7557,   -9398,
-11207,  -12978,  -14706,  -16384,  -18006,  -19567,  -21062,  -22486,  -23834,  -25101,
-26283,  -27376,  -28377,  -29282,  -30087,  -30791,  -31390,  -31884,  -32269,  -32545,
-32712,  -32767,  -32712,  -32545,  -32269,  -31884,  -31390,  -30791,  -30087,  -29282,
-28377,  -27376,  -26283,  -25101,  -23834,  -22486,  -21062,  -19567,  -18006,  -16384,
-14706,  -12978,  -11207,   -9398,   -7557,   -5690,   -3804,   -1905,  };

static const int num_samples_sine_int16_48000 = sizeof(sine_int16_48000) / 2;

uint32_t Util_getTimeMs(void) {
    uint64_t usSinceBoot = time_us_64();
    absolute_time_t absTime = from_us_since_boot(usSinceBoot);
    
    return to_ms_since_boot(absTime);
}

void Util_pcm16To32(int16_t *pcm16, int32_t *pcm32Out, unsigned int sampleCount) {
    for (int i = 0; i < sampleCount; i++) {
        // Shift each sample by 16 bits to align it correctly in 32 bits
        pcm32Out[i] = pcm16[i];
    }
}

void Util_pcm32To16(int32_t *pcm32, int16_t *pcm16Out, unsigned int sampleCount) {
    for (int i = 0; i < sampleCount; i++) {
        // Shift each sample by 16 bits, dropping least significant bits
        pcm16Out[i] = pcm32[i];
    }
}

/* From BlueKitchen BTStack example code */
int Util_produceSinAudio(unsigned int sampleRate, int32_t * pcm_buffer, unsigned int num_samples_to_write) {
    static unsigned int sine_phase = 0;
    int16_t samples[2];

    for (int count = 0; count < num_samples_to_write ; count += 2){
        switch (sampleRate){
            case 44100:
                samples[0] = sine_int16_44100[sine_phase];
                samples[1] = samples[0];

                sine_phase++;
                if (sine_phase >= num_samples_sine_int16_44100){
                    sine_phase -= num_samples_sine_int16_44100;
                }
                break;
            case 48000:
                samples[0] = sine_int16_48000[sine_phase];
                samples[1] = samples[0];

                sine_phase++;
                if (sine_phase >= num_samples_sine_int16_48000){
                    sine_phase -= num_samples_sine_int16_48000;
                }
                break;
            default:
                break;
        }

        Util_pcm16To32(samples, &pcm_buffer[count], 2);
    }

    return num_samples_to_write;

#ifdef VOLUME_REDUCTION
    int i;
    for (i=0;i<num_samples*2;i++){
        if (pcm_buffer[i] > 0){
            pcm_buffer[i] =     pcm_buffer[i]  >> VOLUME_REDUCTION;
        } else {
            pcm_buffer[i] = -((-pcm_buffer[i]) >> VOLUME_REDUCTION);
        }
    }
#endif
}

int Util_produceSinAudio16(unsigned int sampleRate, int16_t * pcm_buffer, unsigned int num_samples_to_write) {
    static unsigned int sine_phase = 0;
    int count;
    for (count = 0; count < num_samples_to_write ; count++){
        switch (sampleRate){
            case 44100:
                pcm_buffer[count * 2]     = sine_int16_44100[sine_phase];
                pcm_buffer[count * 2 + 1] = sine_int16_44100[sine_phase];
                sine_phase++;
                if (sine_phase >= num_samples_sine_int16_44100){
                    sine_phase -= num_samples_sine_int16_44100;
                }
                break;
            case 48000:
                pcm_buffer[count * 2]     = sine_int16_48000[sine_phase];
                pcm_buffer[count * 2 + 1] = sine_int16_48000[sine_phase];
                sine_phase++;
                if (sine_phase >= num_samples_sine_int16_48000){
                    sine_phase -= num_samples_sine_int16_48000;
                }
                break;
            default:
                break;
        }   
    }

    return count;
}

int Util_sineAudioProducer(unsigned int channels, int32_t *pcmBuf, unsigned int numSamples) {
    // TODO: support mono
    return Util_produceSinAudio(AUDIO_PLAYBACK_SAMPLE_RATE, pcmBuf, numSamples);
}
