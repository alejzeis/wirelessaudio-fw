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
#include "dac_controller.h"
#include "audio_common.h"
#include "pins.h"
#include "util.h"

#include "i2s.h"

#include "hardware/dma.h"
#include "hardware/pio.h"

static const i2s_config i2sConfig = {
    AUDIO_SAMPLE_RATE, 
    256, // SCK multiplier (unused)
    AUDIO_WORDSIZE,
    PIN_ADC_GPIO, // SCK pin (unused)
    PIN_DAC_SD, // Data out pin (Don't care about this one, pick unused pin here)
    PIN_ADC_SD, // Data in pin (Don't care about this one, pick unused pin here)
    PIN_DAC_BCK, // Base clock pin, (bit clock) 
    false // No system clock needed, DAC will use internal PLL 
};

static __attribute__((aligned(8))) pio_i2s i2s;

static void process_audio(const int32_t* input, int32_t* output, size_t num_frames) {
#if 0
    // Just copy the input to the output
    for (size_t i = 0; i < num_frames * 2; i++) {
        output[i] = input[i];
    }
#endif
    Util_produceSinAudio(AUDIO_SAMPLE_RATE, output, num_frames);
}

static void dma_i2s_in_handler(void) {
    /* We're double buffering using chained TCBs. By checking which buffer the
     * DMA is currently reading from, we can identify which buffer it has just
     * finished reading (the completion of which has triggered this interrupt).
     */
    if (*(int32_t**)dma_hw->ch[i2s.dma_ch_in_ctrl].read_addr == i2s.input_buffer) {
        // It is inputting to the second buffer so we can overwrite the first
        process_audio(i2s.input_buffer, i2s.output_buffer, AUDIO_BUFFER_FRAMES);
    } else {
        // It is currently inputting the first buffer, so we write to the second
        process_audio(&i2s.input_buffer[STEREO_BUFFER_SIZE], &i2s.output_buffer[STEREO_BUFFER_SIZE], AUDIO_BUFFER_FRAMES);
    }
    dma_hw->ints0 = 1u << i2s.dma_ch_in_data;  // clear the IRQ
}

void DACController_init(void) {
    printf("[DAC]: Starting DAC...\n");

    gpio_init(PIN_DAC_XSMT);
    gpio_init(PIN_DAC_FLT);
    gpio_set_dir(PIN_DAC_XSMT, GPIO_OUT);
    gpio_set_dir(PIN_DAC_FLT, GPIO_OUT);

    // TODO: constants
    gpio_put(PIN_DAC_XSMT, 1); // Un-mute
    gpio_put(PIN_DAC_FLT, 0); // Normal latency

    i2s_program_start_synched(AUDIO_DAC_PIO, &i2sConfig, dma_i2s_in_handler, &i2s);

    printf("[DAC]: I2S Master Started.\n");
}

void DACController_update(void) {
}
