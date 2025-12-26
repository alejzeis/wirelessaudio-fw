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
#ifndef PINS_H
#define PINS_H

#include <hardware/i2c.h>

static const i2c_inst_t* I2C_BUS_INSTANCE = i2c0;
static const unsigned int I2C_BUS_SPEED = 100 * 1000;

static const unsigned int PIN_I2C_SCL = 1;
static const unsigned int PIN_I2C_SDA = 0;

static const unsigned int PIN_SW_LEFT = 7;
static const unsigned int PIN_SW_OK = 8;
static const unsigned int PIN_SW_RIGHT = 9;
static const unsigned int PIN_ADC_SD = 10;
static const unsigned int PIN_ADC_BCLK = 11;
static const unsigned int PIN_ADC_WS = 12;
static const unsigned int PIN_ADC_GPIO = 13;

static const unsigned int PIN_POT_TAP = 28;
static const unsigned int PIN_DAC_FLT = 27;
static const unsigned int PIN_DAC_BCK = PIN_ADC_BCLK;
static const unsigned int PIN_DAC_SD = 26;
static const unsigned int PIN_DAC_WS = PIN_ADC_WS;
static const unsigned int PIN_DAC_XSMT = 21;
static const unsigned int PIN_LCD_NRST = 20;
static const unsigned int PIN_SPI_TX = 19;
static const unsigned int PIN_SPI_CLK = 18;
static const unsigned int PIN_LCD_CS = 17;
static const unsigned int PIN_LCD_A0 = 16;

#endif // PINS_H
