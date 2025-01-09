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
#include "bt/codecs.h"
#include "bt/sbc.h"
#include "audio_common.h"

#include "pico/stdio.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/util/queue.h"
#include "util.h"

#include <pico/time.h>
#include <stdio.h>

static const char *BT_CODECS_NAMES[] = {
    [BT_SBC_CODEC]     = "SBC",
    [BT_AAC_CODEC]     = "AAC",
    [BT_LDAC_CODEC]    = "LDAC",
    [BT_UNKNOWN_CODEC] = "Unknown",
    "",
};

typedef struct CodecsContext_T {
    mutex_t mut;

    BT_Codecs activeCodec;
    // Function pointers to the encode function for the currently active codec
    void (*encodeFunction)();
    // Function pointer to the decode function for the currently active codec
    void (*decodeFunction)();

    // These queues are already concurrent-safe, so no need to use the mutex with them
    queue_t encodeInputQueue;
    queue_t encodeOutputQueue;
    queue_t decodeInputQueue;
    queue_t decodeOutputQueue;

    // Only ever used by Core 1, no mutex needed
    BT_SBC_Context sbcContext;
} CodecsContext;

/**
 * Holds the context for the Codecs module
 * Shared between cores 0 and 1. Use the "mut" mutex
 * field to safely access/modify fields inside.
 */
static CodecsContext codecContext;

static inline absolute_time_t MUTEX_WAIT_TIME() {
    return make_timeout_time_ms(1000);
}

/**
 * Switches the active codec. Clears all queues, sets the encode/decode
 * function pointers and sets the "activeCodec" field.
 *
 * Mutex MUST BE locked prior to calling this!
 */
static void switchCodec(BT_Codecs codec);

/**
 * Main loop for the Codecs Module. This executes on Core 1.
 */
static void codecsCoreMain(void);

static void codecsUpdate(void);

/**
 * AudioSampleProvider implementation
 *
 * This is fed as a function pointer to the encoders, it's job is to grab
 * samples from the encodeInputQueue to give to the encoder.
 *
 * @see AudioSampleProvider, "audio_common.h"
 */
static int encodeSampleProvider(unsigned int sampleRate, 
                                int32_t *pcmFrames, 
                                unsigned int maxSamples);

/**
 * BufferConsumer implementation
 *
 * This is fed as a function pointer to the encoders.
 * It takes the encoded samples given by the encoder and places them in the output queue.
 * It will block until the queue has space available if necessary.
 */
static void encodedSamplesConsumer(uint8_t *data, unsigned int size);

/**
 * Dummy function that does nothing, placeholder for codecs who aren't implemented yet.
 */
static void codecUnimplementedHandler(void);

static void encodeSBC(void);
static void encodeAAC(void);
static void encodeLDAC(void);

static void decodeSBC(void);
static void decodeAAC(void);
static void decodeLDAC(void);

static bool started = false;

void BT_Codecs_init(void) {
    // Reset Core 1 in case it is running for some reason
    multicore_reset_core1();

    mutex_init(&codecContext.mut);
    mutex_enter_blocking(&codecContext.mut);
    
    // Set active codec to Unknown
    codecContext.activeCodec = BT_UNKNOWN_CODEC;
    codecContext.encodeFunction = codecUnimplementedHandler;
    codecContext.decodeFunction = codecUnimplementedHandler;

    // Create mutex and queues
    queue_init(&codecContext.encodeInputQueue, AUDIO_WORDSIZE, BT_CODECS_PCM_QUEUE_SIZE);
    queue_init(&codecContext.decodeInputQueue, 1, BT_CODECS_ENCODED_QUEUE_SIZE);
    queue_init(&codecContext.encodeOutputQueue, 1, BT_CODECS_ENCODED_QUEUE_SIZE);
    queue_init(&codecContext.decodeOutputQueue, AUDIO_WORDSIZE, BT_CODECS_PCM_QUEUE_SIZE);

    mutex_exit(&codecContext.mut);

    // Launch Codecs module main function on Core 1
    multicore_launch_core1(codecsCoreMain);
    started = true;

    printf("[Codecs]: Codecs Module launched on Core 1.\n");
}

void BT_Codecs_setActiveCodecSBC(struct BT_SBC_Configuration_T cfg) {
    printf("[Codecs]: Switching active Codec to SBC\n");
    stdio_flush();

    // Acquire mutex for the codecs struct
    mutex_enter_blocking(&codecContext.mut);

    switchCodec(BT_SBC_CODEC);

    // Inititalize the SBC encoder
    BT_SBC_encoderSetup(cfg, &codecContext.sbcContext);

    mutex_exit(&codecContext.mut);
}

const char* BT_Codecs_getName(BT_Codecs codec) {
    assert(codec < BT_CODECS_LARGEST_VALUE);

    return BT_CODECS_NAMES[codec];
}


int BT_Codecs_queueEncodeAsync(int32_t *samples, unsigned int sampleCount) {
    int i = 0;

    while (i < sampleCount && queue_try_add(&codecContext.encodeInputQueue, &samples[i])) {
        i++;
    }

    return i;
}

void BT_Codecs_queueEncode(int32_t *samples, unsigned int sampleCount) {
    for (int i = 0; i < sampleCount; i++) {
        queue_add_blocking(&codecContext.encodeInputQueue, &samples[i]);
    }
}

unsigned int BT_Codecs_getEncoderOutputAsync(uint8_t *output, unsigned int numFrames) {
    unsigned int availableBytes;
    unsigned int requestBytes;
    unsigned int byteCount = 0;
    unsigned int frameSize = BT_Codecs_getFrameSize();

    // Calculate how many bytes need to be removed
    requestBytes = numFrames * frameSize;

    // Calculate the number of bytes available, truncating non-full frame bytes
    // We can't remove partial frames!
    availableBytes = queue_get_level(&codecContext.encodeOutputQueue);
    availableBytes = availableBytes - (availableBytes % frameSize);

    // Set requestBytes to the minimum of the two
    if (requestBytes > availableBytes) {
        requestBytes = availableBytes;
    }

    // Pop bytes from the queue until empty or reach requested number of bytes
    while (byteCount < requestBytes 
            && queue_try_remove(&codecContext.encodeOutputQueue, &output[byteCount])) {
        byteCount++;
    }

    return byteCount;
}

unsigned int BT_Codecs_getFrameSize(void) {
    unsigned int frameSize = 1;

    // Need mutex to determine the frame size (depends on each codec)
    if (mutex_try_enter(&codecContext.mut, NULL)) {
        // Get bytes per frame
        switch (codecContext.activeCodec) {
            case BT_SBC_CODEC:
                frameSize = BT_SBC_getFrameSize(&codecContext.sbcContext);
                break;
            default:
                frameSize = 1;
                break;
        }
   
        // Release mux
        mutex_exit(&codecContext.mut);
    }

    return frameSize;
}

bool BT_Codecs_isEncoderOverloaded(void) {
    return queue_is_full(&codecContext.encodeInputQueue);
}

static void switchCodec(BT_Codecs codec) {
    uint8_t junk[AUDIO_WORDSIZE];
    
    // Assume mutex is already locked

    // Clear the queues of any existing data
    while(queue_try_remove(&codecContext.encodeInputQueue, junk));
    while(queue_try_remove(&codecContext.decodeInputQueue, junk));
    while(queue_try_remove(&codecContext.encodeOutputQueue, junk));
    while(queue_try_remove(&codecContext.decodeOutputQueue, junk));

    // Cleanup existing codec
    switch (codecContext.activeCodec) {
        case BT_SBC_CODEC:
            BT_SBC_cleanup(&codecContext.sbcContext);
            break;
        default:
            // Unsupported codec, nothing to cleanup
            break;
    }

    codecContext.activeCodec = codec;
    
    // Set function pointers to which codec we are using
    switch (codec) {
        case BT_SBC_CODEC:
            codecContext.encodeFunction = encodeSBC;
            codecContext.decodeFunction = codecUnimplementedHandler;
            break;

        default:
            printf("[Codecs]: WARNING: Active Codec is unimplemented!\n");
            codecContext.encodeFunction = codecUnimplementedHandler;
            codecContext.decodeFunction = codecUnimplementedHandler;
            break;
    }

}

static void codecsCoreMain(void) {
    for (;;) {
        codecsUpdate();
    }
}

static void codecsUpdate(void) {
    void (*encodeFunction)();
    void (*decodeFunction)();

    // Acquire mutex to grab function pointers
    mutex_enter_blocking(&codecContext.mut);
    
    encodeFunction = codecContext.encodeFunction;
    decodeFunction = codecContext.decodeFunction;

    // Release mutex
    mutex_exit(&codecContext.mut);

    // Check if queue has stuff in it, and acquire mutex 
    if (!queue_is_empty(&codecContext.encodeInputQueue)) {
        // Do encoding
        codecContext.encodeFunction();
    }


    if (!queue_is_empty(&codecContext.decodeInputQueue)) {
        // Do decoding
        codecContext.decodeFunction();
    }
}

int encodeSampleProvider(unsigned int channels, int32_t *pcmFrames, unsigned int maxSamples) {
    // Samples are interleaved, so if 2 channels then there are 2x elements requested
    unsigned int totalSamples = channels * maxSamples;
    unsigned int sampleCount = 0;
    bool queueEmpty = false;

    /*
     * Remove  from the encode queue until we either reach the number of samples requested
     * by the encoder, OR the queue becomes empty. Place them in the pcmFrames array.
     */
    while (sampleCount < totalSamples
            && queue_try_remove(&codecContext.encodeInputQueue, &pcmFrames[sampleCount])) {

        // Increment our sample count
        sampleCount++;
    }

    // Max number of samples requested or we have no data in the queue

    // Interleaved sample count, so divide by number of channels
    return (sampleCount / channels);
}

static void encodedSamplesConsumer(uint8_t *data, unsigned int size) {
    unsigned int samplesAdded = 0;
    for (int i = 0; i < size; i++) {
        if (queue_try_add(&codecContext.encodeOutputQueue, &data[i])) {
            samplesAdded += 1;
        }
    }

    if (samplesAdded != size) {
        printf("[Codecs]: Output queue is full, dropping %i bytes\n", size - samplesAdded);
    }
}

static void codecUnimplementedHandler(void) {
    // Do nothing
}

static void encodeSBC(void) {
    CodecsContext *ctx = &codecContext;
    unsigned int samplesAvailable = queue_get_level(&ctx->encodeInputQueue) / ctx->sbcContext.config.num_channels;

    // Ensure we have enough data in the queue to fill an SBC encode buffer
    // If we didn't the encoder would hang waiting for more data
    if (samplesAvailable >= BT_SBC_getMinimumSampleCount(&ctx->sbcContext)) {
        //encodeSampleProvider(0, samples, BT_SBC_getMinimumSampleCount(&ctx->sbcContext));
        BT_SBC_encodeWithLimit(&ctx->sbcContext, &encodeSampleProvider, &encodedSamplesConsumer, samplesAvailable);
        //i = BT_SBC_encodeFrameManual(&ctx->sbcContext, samples, encodedData);
        //encodedSamplesConsumer(encodedData, i);
    }
}
