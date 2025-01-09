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
#include "adc_controller.h"
#include "audio_common.h"
#include "pins.h"

#include "i2s.h"

#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"

static const uint8_t ADC_I2C_ADDRESS = 0x9C;
static const uint8_t ADC_REG_MST_CFG0 = 0x13;
static const uint8_t ADC_REG_MST_CFG1 = 0x14;
static const uint8_t ADC_REG_CLK_SRC = 0x16;
static const uint8_t ADC_REG_GPIO_CFG0 = 0x21;

static const uint32_t ADC_SCK_MULTIPLIER = 128; // 96KHz * 128 = 12.288 MHz (supported by ADC)
/* 
 * Pin to output the Master Clock on.
 * The ADC supports using it's multipurpose "GPIO" pin
 * as a system clock input. Using this system clock it will internally generate
 * it's required clock with a PLL, and among other things, will generate the bit clock (BCLK),
 * and the word select signals (LRCLK). Thus we can be the target (slave) while the ADC acts as
 * controller (master)
 */
static const uint8_t ADC_MCLK_PIN = PIN_ADC_GPIO;

static const i2s_config i2sConfig = {
    AUDIO_SAMPLE_RATE, 
    ADC_SCK_MULTIPLIER,
    AUDIO_WORDSIZE,
    ADC_MCLK_PIN, // SCK pin
    5, // Data out pin (Don't care about this one, pick unused pin here)
    PIN_ADC_SD, // Data in pin (Sound data) 
    PIN_ADC_BCLK, // Base clock pin, (bit clock) 
    true // Generate a System Clock signal to output on SCK
};

static __attribute__((aligned(8))) pio_i2s i2s;

static void process_audio(const int32_t* input, int32_t* output, size_t num_frames) {
    // TODO: Pass this data to the Bluetooth encoder 
    for (size_t i = 0; i < num_frames * 2; i++) {
        output[i] = input[i];
    }
}

static void ADCController_i2sDMAHandler(void) {
    /* We're double buffering using chained TCBs. By checking which buffer the
     * DMA is currently reading from, we can identify which buffer it has just
     * finished reading (the completion of which has triggered this interrupt).
     */
    if (*(int32_t**) dma_hw->ch[i2s.dma_ch_in_ctrl].read_addr == i2s.input_buffer) {
        // It is inputting to the second buffer so we can overwrite the first
        process_audio(i2s.input_buffer, i2s.output_buffer, AUDIO_BUFFER_FRAMES);
    } else {
        // It is currently inputting the first buffer, so we write to the second
        process_audio(&i2s.input_buffer[STEREO_BUFFER_SIZE],
                      &i2s.output_buffer[STEREO_BUFFER_SIZE],
                      AUDIO_BUFFER_FRAMES);
    }

    dma_hw->ints0 = 1u << i2s.dma_ch_in_data;  // clear the IRQ
}

static void configureADC(void) {
    uint8_t gpioCfg0Write[] = { ADC_REG_GPIO_CFG0,
        // Configure ADC Pin GPIO0 as MCLK input
        0b10100000
    };

    uint8_t mstCfg0Write[] = { ADC_REG_MST_CFG0,
        // Sets ADC to I2S MASTER mode,
        // selects a sample rate of multiple of 48 KHz,
        // enables PLL, auto clock configuration,
        // and notifies that MCLK will be 12.288 MHz
        0b10000001 
    };

    uint8_t mstCfg1Write[] = { ADC_REG_MST_CFG1,
        // 96 KHz and 32 FSYNC/BCLK ratio
        0b01010010
    };

    // Write register contents
    i2c_write_blocking(I2C_BUS_INSTANCE, ADC_I2C_ADDRESS, gpioCfg0Write, 2, false);
    i2c_write_blocking(I2C_BUS_INSTANCE, ADC_I2C_ADDRESS, mstCfg0Write, 2, false);
    i2c_write_blocking(I2C_BUS_INSTANCE, ADC_I2C_ADDRESS, mstCfg1Write, 2, false);

    printf("[ADC]: Configured device.\n");
}

void ADCController_init(void) {
    configureADC();

    // Start i2s operation
    i2s_program_start_slaved(AUDIO_ADC_PIO, &i2sConfig, ADCController_i2sDMAHandler, &i2s);
}

void ADCController_update(void) {

}
