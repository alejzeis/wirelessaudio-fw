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

#include "pico/util/queue.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "util.h"
#include <pico/time.h>

#include <inttypes.h>

static const uint32_t ADC_STATUS_UPDATE_INTERVAL = 1000;

static const uint8_t ADC_I2C_ADDRESS = 78;
static const uint8_t ADC_REG_SLEEP_CFG = 0x2;
static const uint8_t ADC_REG_ASI_CFG0 = 0x7;
static const uint8_t ADC_REG_MST_CFG0 = 0x13;
static const uint8_t ADC_REG_MST_CFG1 = 0x14;
static const uint8_t ADC_REG_CLK_SRC = 0x16;
static const uint8_t ADC_REG_GPIO_CFG0 = 0x21;
static const uint8_t ADC_REG_GPO_CFG0 = 0x22;
static const uint8_t ADC_REG_GPI_CFG0 = 0x2B;
static const uint8_t ADC_REG_CH1_CFG0 = 0x3C;
static const uint8_t ADC_REG_CH2_CFG0 = 0x41;
static const uint8_t ADC_REG_IN_CH_EN = 0x73;
static const uint8_t ADC_REG_ASI_OUT_CH_EN = 0x74;
static const uint8_t ADC_REG_PWR_CFG = 0x75;
static const uint8_t ADC_REG_DEV_STS0 = 0x76;
static const uint8_t ADC_REG_DEV_STS1 = 0x77;

static const uint32_t ADC_SCK_MULTIPLIER = 256; // 48KHz * 256 = 12.288 MHz (supported by ADC)
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
    PIN_DAC_SD, // Data out pin (Don't care about this one, pick unused pin here)
    PIN_ADC_SD, // Data in pin (Sound data) 
    PIN_ADC_BCLK, // Base clock pin, (bit clock) 
    true // Generate a System Clock signal to output on SCK
};

static __attribute__((aligned(8))) pio_i2s i2s;
static int32_t firstBuffer[STEREO_BUFFER_SIZE];

static void adc_write(uint8_t registerAddress, uint8_t value) {
    uint8_t data[2];

    data[0] = registerAddress;
    data[1] = value;

    i2c_write_blocking(I2C_BUS_INSTANCE, ADC_I2C_ADDRESS, data, 2, false);
}

static uint8_t adc_read(uint8_t registerAddress) {
    uint8_t value = 0;

    if (i2c_write_blocking(I2C_BUS_INSTANCE, ADC_I2C_ADDRESS, &registerAddress, 1, false) == 1) {
        i2c_read_blocking(I2C_BUS_INSTANCE, ADC_I2C_ADDRESS, &value, 1, false);
    } else {
        printf("[ADC]: I2C Write failed to device 0x%X, register 0x%X\n", ADC_I2C_ADDRESS, registerAddress);
    }

    return value;
}

static void process_audio(const int32_t* input, int32_t* output, size_t num_frames) {
    static bool recieved_data = false;
    int32_t junk;

    // TODO: Pass this data to the Bluetooth encoder 
    for (size_t i = 0; i < num_frames * 2; i++) {
        firstBuffer[i] = input[i];
    }

    if (!recieved_data) {
        printf("[ADC] First I2S Samples receieved\n");

        recieved_data = true;
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
    // Configure ADC Pin GPIO0 as MCLK input
    uint8_t gpioCfg0Write = 0b10100001;

    // Set to use I2S, 32bit word size
    uint8_t asiCfg0Write = 0b01110000;

    // Sets ADC to I2S MASTER mode,
    // selects a sample rate of multiple of 48 KHz,
    // enables PLL, auto clock configuration,
    // and notifies that MCLK will be 12.288 MHz
    uint8_t mstCfg0Write = 0b10000001;

    // 48 KHz and 256 FSYNC/BCLK ratio
    uint8_t mstCfg1Write = 0b01001000;

    // Set Channel 1,2 input type to LINE IN, single-ended, disabled dynamic Range Enhancer
    uint8_t ch1ConfigWrite = 0b10100000;
    uint8_t ch2ConfigWrite = 0b10100000;

    // Enable CH1,2 output
    uint8_t outChWrite = 0b11000000;

    // Enable CH1,CH2
    uint8_t inChEnWrite = 0b11000000;
    // Enable all enabled ADC channels, enable PLL
    uint8_t pwrConfigWrite = 0b11100000;
   
    // Enable internal 1.8V AREG supply, wake up the chip
    uint8_t sleepCfgWrite = 0b10000001;

    // Write register contents
    // Wakeup from sleep
    adc_write(ADC_REG_SLEEP_CFG, sleepCfgWrite);

    sleep_ms(20);
    
    // Disable GPIO pin, configure as MCLK input
    adc_write(ADC_REG_GPIO_CFG0, gpioCfg0Write);
    // Disable GPI/GPO pin functions (to allow using GPI/GPO pins as channel 2 input)
    adc_write(ADC_REG_GPI_CFG0, 0);
    adc_write(ADC_REG_GPO_CFG0, 0);

    adc_write(ADC_REG_ASI_CFG0, asiCfg0Write);
    adc_write(ADC_REG_MST_CFG0, mstCfg0Write);
    adc_write(ADC_REG_MST_CFG1, mstCfg1Write);
    adc_write(ADC_REG_CH1_CFG0, ch1ConfigWrite);
    adc_write(ADC_REG_CH2_CFG0, ch2ConfigWrite);
    // Enable ASI Bus error detection
    adc_write(0x9, 0b00100000);
    adc_write(ADC_REG_ASI_OUT_CH_EN, outChWrite);
    sleep_ms(10);
    adc_write(ADC_REG_IN_CH_EN, inChEnWrite);

    sleep_ms(10);

    // Power on channels
    adc_write(ADC_REG_PWR_CFG, pwrConfigWrite);

    printf("[ADC]: Configured device.\n");
}

static void dumpADCStatus(void) {
    uint8_t sts0;
    uint8_t sts1;
    uint8_t sleepStatus;
    uint8_t interruptStatus;
    uint8_t asiStatus;
    bool channel0Power;
    bool channel1Power;


    sleepStatus = adc_read(ADC_REG_SLEEP_CFG);
    interruptStatus = adc_read(0x36);
    sts0 = adc_read(ADC_REG_DEV_STS0);
    sts1 = adc_read(ADC_REG_DEV_STS1);
    asiStatus = adc_read(0x15);

    printf("[ADC]: Report (STS0: 0x%X, STS1: 0x%X, SLEEP_CFG: 0x%X, INT: 0x%X)\n", sts0, sts1, sleepStatus, interruptStatus);
    printf("[ADC]: ASI Bus STS: 0x%X\n", asiStatus);
}

void ADCController_init(void) {
    printf("[ADC]: Configuring device (I2C Address 0x%X)\n", ADC_I2C_ADDRESS);

    // Start i2s operation
    i2s_program_start_slaved(AUDIO_ADC_PIO, &i2sConfig, ADCController_i2sDMAHandler, &i2s);

    configureADC();

    sleep_ms(50);
    dumpADCStatus();
}

void ADCController_update(void) {
    static uint32_t lastUpdate = 0;
    static bool ledState = false;

    uint32_t now = Util_getTimeMs();

    // Blink the status LED according to the period
    if (now - lastUpdate >= ADC_STATUS_UPDATE_INTERVAL) {
        dumpADCStatus();

        printf("Buffer Data: ");
        for (size_t i = 0; i < AUDIO_BUFFER_FRAMES * 2; i++) {
            printf("%#" PRIx32 " ", firstBuffer[i]);
            if (i % 16 == 0) {
                printf("\n");
            }
        }
        printf("\n");

        lastUpdate = now;
    }
}

int ADCController_audioSampleProvider(unsigned int channels, int32_t* samplesOut, unsigned int numReq) {
    static int position = 0;
    for (int i = 0; i < numReq * 2; i++) {
        samplesOut[i] = firstBuffer[position];
        position++;

        if (position >= STEREO_BUFFER_SIZE) {
            position = 0;
        }
    }

    return numReq;
}
