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
#ifndef BT_SBC_H
#define BT_SBC_H

#include "audio_common.h"
#include "btstack.h"

#include <stdint.h>

struct BT_SBC_Configuration_T {
    bool supported;

    int local_seid;
    int remote_seid;

    int reconfigure;
    
    int num_channels;
    int sampling_frequency;
    int block_length;
    int subbands;
    int min_bitpool_value;
    int max_bitpool_value;
    btstack_sbc_channel_mode_t      channel_mode;
    btstack_sbc_allocation_method_t allocation_method;
};

typedef struct BT_SBC_Context_T {
    struct BT_SBC_Configuration_T config;

    const btstack_sbc_encoder_t *   _encoderInstance;
    btstack_sbc_encoder_bluedroid_t _encoderState;
} BT_SBC_Context;

/**
 * Processes a A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION event,
 * constructs a new BT_SBC_Configuration_T struct. 
 *
 * @param packet The A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION packet
 * @param cfg A BT_SBC_Configuration_T struct pointer to update with the SBC configuration.
 */
void BT_SBC_processConfigPacket(uint8_t *packet, struct BT_SBC_Configuration_T *cfg);

/**
 * Sets up a new SBC encoder context for the provided SBC configuration.
 */
void BT_SBC_encoderSetup(struct BT_SBC_Configuration_T config, BT_SBC_Context *newCtx);

/**
 * Encodes raw interleaved PCM data, up to a maximum, from the provided AudioSampleProviderFunc function pointer,
 * until the size of the encoded data exceeds the maxMediaPacketSize in the configuration.
 *
 * The limit provided is the maximum amount of interleaved samples to try to obtain from the dataProviderFunc.
 * It is not guaranteed that the full maximum amount of samples will be used, depending on if there
 * are enough for an SBC frame or not. Check return value to see how many samples were read.
 *
 * Encoded data will be given via dataConsumerFunc callback function.
 *
 * @return The number of samples retrieved.
 */
int BT_SBC_encodeWithLimit(BT_SBC_Context *ctx, 
                       InterleavedSampleProvider dataProviderFunc,
                       BufferConsumer dataConsumerFunc,
                       unsigned int maxSamples);

/**
 * Get the payload for an AVDTP media payload, including the SBC header (number of frames).
 *
 * Meant to be called before sending an A2DP media payload packet. encodeDataUtilFull should have been called before.
 *
 * This function returns a pointer to the data, which is stored in the "encodedData" field in the SBC Context.
 * The length of the data will be stored in the "packedLength" pointer. The RTP timestamp pointer is updated
 * to reflect the new timestamp with the newly encoded data.
 */
uint8_t* BT_SBC_packPayload(BT_SBC_Context *ctx, unsigned int *packedLength, uint32_t *rtpTimestamp);

/**
 * Obtain the minimum amount of samples needed to fill an SBC encoder
 * buffer.
 */
unsigned int BT_SBC_getMinimumSampleCount(BT_SBC_Context *ctx);

/**
 * Get the number of bytes in an SBC frame.
 */
unsigned int BT_SBC_getFrameSize(BT_SBC_Context *ctx);

/**
 * Cleans-up and deinitializes a BT_SBC_Context context.
 */
void BT_SBC_cleanup(BT_SBC_Context *ctx);

#endif // BT_SBC_H
