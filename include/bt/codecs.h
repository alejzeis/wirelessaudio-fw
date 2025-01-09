/*
 * Multiprocessing Codecs Module
 * Manages encoding and decoding data for the A2DP source and sink modules.
 * These functions are used to communicate between both cores, as all encoding
 * and decoding will take place on Core 1, while BT and the main firmware 
 * runs on Core 0.
 *
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
#ifndef BT_CODECS_H
#define BT_CODECS_H

#include "bt/sbc.h"

#define BT_CODECS_LARGEST_VALUE 0xFF

static const unsigned int BT_CODECS_PCM_QUEUE_SIZE = 1024;
static const unsigned int BT_CODECS_ENCODED_QUEUE_SIZE = 4096;

typedef enum BT_Codecs_E {
    BT_SBC_CODEC,
    BT_AAC_CODEC,
    BT_LDAC_CODEC,
    BT_UNKNOWN_CODEC = BT_CODECS_LARGEST_VALUE
} BT_Codecs;


/**
 * Initialize the Codecs module, which will reset and launch on Core 1.
 */
void BT_Codecs_init(void);

/**
 * Concurrently-safe: Switch active codec to SBC.
 */
void BT_Codecs_setActiveCodecSBC(struct BT_SBC_Configuration_T config);

/**
 * Get the currently active Codec
 */
BT_Codecs BT_Codecs_getActiveCodec(void);

/**
 * Get the name of a codec as a string, given the Codec enum value.
 */
const char* BT_Codecs_getName(BT_Codecs codec);

/**
 * Adds one or more samples to the encode queue. These will then be processed
 * by the encoder and will be available as output in the future.
 *
 * This function is non-blocking. If the internal queue becomes full then
 * this function will return, even if not all samples were added.
 *
 * @param samples Pointer to array of samples to be encoded
 * @param sampleCount Amount of samples in the array to be encoded
 *
 * @return The amount of samples added to the queue
 */
int BT_Codecs_queueEncodeAsync(int32_t *samples, unsigned int sampleCount);

/**
 * Adds one or more samples to the encode queue. These will then be processed
 * by the encoder and will be available as output in the future.
 *
 * This function is blocking. If the internal queue becomes full then
 * this function will wait until the queue has space available.
 *
 * @param samples Pointer to array of samples to be encoded
 * @param sampleCount Amount of samples in the array to be encoded
 */
void BT_Codecs_queueEncode(int32_t *samples, unsigned int sampleCount);

/**
 * Get the output from the encoder. This function is non-blocking,
 * if no output is available this function will return immediately.
 *
 * The output from the encoder is in terms of frames, which may have different sizing
 * depending on which Codec is selected. The number of bytes will be returned.
 *
 * @param output Pointer to a buffer to place the output bytes in.
 * @param numFrames Number of frames to retrieve from the encoder output queue.
 *
 * @return The number of bytes read from the encoder output. 
 *         May be zero if no data available.
 */
unsigned int BT_Codecs_getEncoderOutputAsync(uint8_t *output, unsigned int numFrames);

/**
 * Returns the size in bytes of a "frame" for the currently active codec.
 */
unsigned int BT_Codecs_getFrameSize(void);

/**
 * Returns true if the encoder input queue is full.
 */
bool BT_Codecs_isEncoderOverloaded(void);

#endif // BT_CODECS_H
