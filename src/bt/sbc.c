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
#include "bt/sbc.h"
#include "audio_common.h"

#include "pico/stdio.h"
#include "util.h"

#include <classic/avdtp.h>
#include <stdint.h>
#include <stdio.h>

#define SBC_PCM_BUFFER_SIZE     512
#define SBC_ENCODED_BUFFER_SIZE 1030

static void dump_sbc_configuration(struct BT_SBC_Configuration_T* configuration){
    printf("Received media codec configuration:\n");
    printf("    - num_channels: %d\n", configuration->num_channels);
    printf("    - sampling_frequency: %d\n", configuration->sampling_frequency);
    printf("    - channel_mode: %d\n", configuration->channel_mode);
    printf("    - block_length: %d\n", configuration->block_length);
    printf("    - subbands: %d\n", configuration->subbands);
    printf("    - allocation_method: %d\n", configuration->allocation_method);
    printf("    - bitpool_value [%d, %d] \n", configuration->min_bitpool_value, configuration->max_bitpool_value);
}

void BT_SBC_processConfigPacket(uint8_t *packet, struct BT_SBC_Configuration_T *sbc_configuration) {
    avdtp_channel_mode_t channel_mode;
    uint8_t allocation_method;

    sbc_configuration->supported = true;

    sbc_configuration->remote_seid = a2dp_subevent_signaling_media_codec_sbc_configuration_get_remote_seid(packet);

    sbc_configuration->reconfigure = a2dp_subevent_signaling_media_codec_sbc_configuration_get_reconfigure(packet);
    sbc_configuration->num_channels = a2dp_subevent_signaling_media_codec_sbc_configuration_get_num_channels(packet);
    sbc_configuration->sampling_frequency = a2dp_subevent_signaling_media_codec_sbc_configuration_get_sampling_frequency(packet);
    sbc_configuration->block_length = a2dp_subevent_signaling_media_codec_sbc_configuration_get_block_length(packet);
    sbc_configuration->subbands = a2dp_subevent_signaling_media_codec_sbc_configuration_get_subbands(packet);
    sbc_configuration->min_bitpool_value = a2dp_subevent_signaling_media_codec_sbc_configuration_get_min_bitpool_value(packet);
    sbc_configuration->max_bitpool_value = a2dp_subevent_signaling_media_codec_sbc_configuration_get_max_bitpool_value(packet);

    channel_mode = (avdtp_channel_mode_t) a2dp_subevent_signaling_media_codec_sbc_configuration_get_channel_mode(packet);
    allocation_method = a2dp_subevent_signaling_media_codec_sbc_configuration_get_allocation_method(packet);

    printf("[SBC]: Received codec configuration, sampling frequency %u, local seid 0x%02x, remote seid 0x%02x.\n",
           sbc_configuration->sampling_frequency,
           a2dp_subevent_signaling_media_codec_sbc_configuration_get_local_seid(packet),
           a2dp_subevent_signaling_media_codec_sbc_configuration_get_remote_seid(packet));

    // Adapt Bluetooth spec definition to SBC Encoder expected input
    sbc_configuration->allocation_method = (btstack_sbc_allocation_method_t)(allocation_method - 1);
    switch (channel_mode){
        case AVDTP_CHANNEL_MODE_JOINT_STEREO:
            sbc_configuration->channel_mode = SBC_CHANNEL_MODE_JOINT_STEREO;
            break;
        case AVDTP_CHANNEL_MODE_STEREO:
            sbc_configuration->channel_mode = SBC_CHANNEL_MODE_STEREO;
            break;
        case AVDTP_CHANNEL_MODE_DUAL_CHANNEL:
            sbc_configuration->channel_mode = SBC_CHANNEL_MODE_DUAL_CHANNEL;
            break;
        case AVDTP_CHANNEL_MODE_MONO:
            sbc_configuration->channel_mode = SBC_CHANNEL_MODE_MONO;
            break;
        default:
            btstack_assert(false);
            break;
    }
    dump_sbc_configuration(sbc_configuration);

    if (sbc_configuration->sampling_frequency != AUDIO_PLAYBACK_SAMPLE_RATE) {
        printf("[SBC]: WARNING sample rate mismatch (Ours %i, requested %i).\n",
                AUDIO_PLAYBACK_SAMPLE_RATE, sbc_configuration->sampling_frequency);
    } 
}

void BT_SBC_encoderSetup(struct BT_SBC_Configuration_T config, BT_SBC_Context *ctx) {
    printf("[SBC]: Encoder Initializing\n");
    ctx->config = config;

    ctx->_encoderInstance = btstack_sbc_encoder_bluedroid_init_instance(&ctx->_encoderState);
    uint8_t status = ctx->_encoderInstance->configure(&ctx->_encoderState, SBC_MODE_STANDARD,
                                    config.block_length, config.subbands,
                                    config.allocation_method, config.sampling_frequency,
                                    config.max_bitpool_value,
                                    config.channel_mode);
}

#if 0
int BT_SBC_encodeDataUntilFull(BT_SBC_Context *ctx,
                                AudioSampleProviderFunc dataProviderFunc,
                                uint32_t numSamples) {
    int16_t pcm_frame[256 * ctx->config.num_channels];
    uint16_t sbc_frame_size; 

    unsigned int encodedSampleCount = 0;

    unsigned int numSamplesPerBuffer = ctx->_encoderInstance->num_audio_frames(&ctx->_encoderState);
    uint16_t bufferSize = ctx->_encoderInstance->sbc_buffer_length(&ctx->_encoderState);

    while ((numSamples >= numSamplesPerBuffer)
            && (ctx->config.maxMediaPacketSize - ctx->encodedDataCount) >= bufferSize) {
        // Obtain new PCM frames
        dataProviderFunc(ctx->config.sampling_frequency, pcm_frame, numSamplesPerBuffer);

        // encode into sbc storage buffer, first byte contains sbc media header
        // media header generated later by packPayload()
        ctx->_encoderInstance->encode_signed_16(&ctx->_encoderState,
                                                pcm_frame,
                                                &ctx->encodedData[1 + ctx->encodedDataCount]);

        sbc_frame_size = ctx->_encoderInstance->sbc_buffer_length(&ctx->_encoderState);
        ctx->encodedDataCount += sbc_frame_size;
        
        numSamples -= numSamplesPerBuffer;
        encodedSampleCount += numSamplesPerBuffer;
    }

    return encodedSampleCount;
}
#endif

int BT_SBC_encodeWithLimit(BT_SBC_Context *ctx,
                           InterleavedSampleProvider dataProviderFunc,
                           BufferConsumer dataConsumerFunc,
                           unsigned int maxSamples) {
    int32_t pcm32_frames[SBC_PCM_BUFFER_SIZE];
    int16_t pcm16_frames[SBC_PCM_BUFFER_SIZE];
    uint8_t encodedData[SBC_ENCODED_BUFFER_SIZE];

    bool providerBusy = false;

    unsigned int pcmFrameCount;
    unsigned int samplesProvided;
    unsigned int samplesUsed = 0;
    unsigned int encodedDataCount = 0;

    unsigned int numSamplesPerBuffer = ctx->_encoderInstance->num_audio_frames(&ctx->_encoderState);
    uint16_t sbc_frame_size = ctx->_encoderInstance->sbc_buffer_length(&ctx->_encoderState);

    /*
     * Loop while there is still space available in "encodedData" for another
     * SBC frame, and while we have not reached the maximum sample read limit,
     * and while the provider function is still returning enough data to fill a buffer.
     */
    while (((encodedDataCount + sbc_frame_size) < SBC_ENCODED_BUFFER_SIZE)
            && ((maxSamples - samplesUsed) > numSamplesPerBuffer) 
            && !providerBusy) {
        // Obtain new interleaved PCM frames
        samplesProvided = dataProviderFunc(ctx->config.num_channels,
                                           pcm32_frames, 
                                           numSamplesPerBuffer);
        if (samplesProvided < numSamplesPerBuffer) {
            // Provider did not give us enough frames to fill an SBC buffer, so quit
            providerBusy = true;
            printf("[SBC]: Dropping %i frames, not enough to fill SBC buffer!\n", samplesProvided);
        } else {
            // Samples are interleaved, so the total amount of PCM data is N * sample count
            pcmFrameCount = samplesProvided * ctx->config.num_channels;

            // Convert from 32 bit to 16 bit
            Util_pcm32To16(pcm32_frames, pcm16_frames, pcmFrameCount);

            ctx->_encoderInstance->encode_signed_16(&ctx->_encoderState,
                                                    pcm16_frames,
                                                    encodedData);

            sbc_frame_size = ctx->_encoderInstance->sbc_buffer_length(&ctx->_encoderState);
            encodedDataCount += sbc_frame_size;
            samplesUsed += numSamplesPerBuffer;

            // Give the data to the consumer
            dataConsumerFunc(encodedData, sbc_frame_size);
        }
    }

    return samplesUsed;
}

uint8_t* BT_SBC_packPayload(BT_SBC_Context *ctx, unsigned int *packedLength, uint32_t *rtpTimestamp) {
#if 0
    int num_bytes_in_frame = ctx->_encoderInstance->sbc_buffer_length(&ctx->_encoderState);
    unsigned int numSamplesPerBuffer = ctx->_encoderInstance->num_audio_frames(&ctx->_encoderState);
    int bytes_in_storage = ctx->encodedDataCount;
    uint8_t num_sbc_frames = bytes_in_storage / num_bytes_in_frame;

    // Prepend SBC Header
    ctx->encodedData[0] = num_sbc_frames;

    *packedLength = bytes_in_storage + 1;
    
    // update rtp_timestamp
    *rtpTimestamp += num_sbc_frames * numSamplesPerBuffer;

    // Reset the length to clear our encode buffer
    ctx->encodedDataCount = 0;

    return ctx->encodedData;
#endif
    return NULL;
}

void BT_SBC_cleanup(BT_SBC_Context *ctx) {
    
}

unsigned int BT_SBC_getMinimumSampleCount(BT_SBC_Context *ctx) {
    return ctx->_encoderInstance->num_audio_frames(&ctx->_encoderState);
}

unsigned int BT_SBC_getFrameSize(BT_SBC_Context *ctx) {
    assert(ctx->_encoderInstance != NULL);

    return ctx->_encoderInstance->sbc_buffer_length(&ctx->_encoderState);
}

