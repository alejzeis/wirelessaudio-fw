/*
 * Main A2DP Source Code. Based on example from BlueKitchen BTStack.
 *
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
#include "bt/a2dp_source.h"
#include "bt/bt_controller.h"
#include "bt/sbc.h"
#include "bt/codecs.h"
#include "ui_controller.h"
#include "audio_common.h"
#include "util.h"

#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "btstack.h"

#include <bluetooth.h>
#include <btstack_defines.h>
#include <btstack_event.h>
#include <btstack_util.h>
#include <classic/a2dp_source.h>
#include <classic/avdtp.h>
#include <classic/avdtp_source.h>
#include <classic/avdtp_util.h>
#include <classic/avrcp.h>
#include <classic/avrcp_controller.h>
#include <stdint.h>
#include <stdio.h>

#define A2DP_ENCODE_SAMPLE_BATCH_SIZE 2

static const uint32_t AVRCP_DEFAULT_TRACK_POSITION = 0xFFFFFFFF; 

static const unsigned int AUDIO_TIMEOUT_MS = 5;

typedef struct a2dp_codec_info_t {
    BT_Codecs codec;
    uint8_t numChannels;
    uint8_t remote_seid;
    uint8_t local_seid;
} a2dp_codec_info;

typedef struct {
    uint16_t a2dp_cid;
    uint8_t  stream_opened;
    uint16_t avrcp_cid;

    a2dp_codec_info codecs[BT_CODECS_LARGEST_VALUE];
    BT_Codecs currentCodec;

    uint32_t time_audio_data_sent; // ms
    uint32_t acc_num_missed_samples;
    uint32_t samples_ready;
    btstack_timer_source_t audio_timer;
    uint8_t  streaming;
    int      max_media_payload_size;
    uint32_t rtp_timestamp;

    uint8_t encodedData[A2DP_ENCODER_BUFFER_SIZE];
    unsigned int encodedDataCount;
    unsigned int encodedFrameCount;

    bool ready_to_send;

    InterleavedSampleProvider audioProviderFunc;

    struct BT_SBC_Configuration_T sbcConfig;
} a2dp_media_sending_context_t;

static uint8_t media_sbc_codec_capabilities[] = {
    // Only support 48 KHz sample rate
    (AVDTP_SBC_48000 << 4) | AVDTP_SBC_STEREO,
    0xFF,//(AVDTP_SBC_BLOCK_LENGTH_16 << 4) | (AVDTP_SBC_SUBBANDS_8 << 2) | AVDTP_SBC_ALLOCATION_METHOD_LOUDNESS,
    2, 53
}; 

static uint8_t media_ldac_codec_capabilities[] = {
        0x2D, 0x1, 0x0, 0x0,
        0xAA, 0,
        0x20,
        0x01,
        0x1
};

static uint8_t sdp_a2dp_source_service_buffer[150];
static uint8_t sdp_avrcp_target_service_buffer[200];
static uint8_t sdp_avrcp_controller_service_buffer[200];
static uint8_t device_id_sdp_service_buffer[100];

static uint8_t media_sbc_codec_configuration[4];
static uint8_t ldac_media_codec_configuration[9];
static a2dp_media_sending_context_t media_tracker;


const avrcp_track_t DEFAULT_TRACK_INFO = {
    .track_id = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}, 
    .track_nr = 1, 
    .title = "Audio IN Stream", 
    .artist = "Wireless Audio Device", 
    .album = "", 
    .genre = "", 
    .song_length_ms = 0xFFFFFFFF, 
    .song_position_ms = 0xFFFFFFFF
};

/* AVRCP Target context END */

/* @section Main Application Setup
 *
 * @text The Listing MainConfiguration shows how to setup AD2P Source and AVRCP services. 
 * Besides calling init() method for each service, you'll also need to register several packet handlers:
 * - hci_packet_handler - handles legacy pairing, here by using fixed '0000' pin code.
 * - a2dp_source_packet_handler - handles events on stream connection status (established, released), the media codec configuration, and, the commands on stream itself (open, pause, stopp).
 * - avrcp_packet_handler - receives connect/disconnect event.
 * - avrcp_controller_packet_handler - receives answers for sent AVRCP commands.
 * - avrcp_target_packet_handler - receives AVRCP commands, and registered notifications.
*
 * @text To announce A2DP Source and AVRCP services, you need to create corresponding
 * SDP records and register them with the SDP service. 
 */

static a2dp_codec_info* findCodecByRemoteSeid(uint8_t remote_seid);
static uint8_t getActiveLocalSeid(a2dp_media_sending_context_t *ctx);

static bool setupEndpoints(void);

static void sendSamplesToEncoder(a2dp_media_sending_context_t *ctx);
static void readSamplesFromEncoder(a2dp_media_sending_context_t *ctx);

static void onSBCConfigurationRecieved(uint8_t *packet);
static void onA2DPStreamEstablished(uint8_t *packet);

static void a2dp_source_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * event, uint16_t event_size);
static void avrcp_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void avrcp_target_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void avrcp_controller_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static a2dp_codec_info* findCodecByRemoteSeid(uint8_t remote_seid) {
    a2dp_codec_info *codec = NULL;

    // Loop through codecs info array until we find a matching remote seid
    for (int i = 0; i < BT_CODECS_LARGEST_VALUE; i++) {
        if (media_tracker.codecs[i].remote_seid == remote_seid) {
            codec = &media_tracker.codecs[i];
            break;
        }
    }

    return codec;
}

static uint8_t getActiveLocalSeid(a2dp_media_sending_context_t *ctx) {
    return ctx->codecs[ctx->currentCodec].local_seid;
}

static void sendSamplesToEncoder(a2dp_media_sending_context_t *ctx) {
    unsigned int sampleCount = 0;
    unsigned int channels = ctx->codecs[ctx->currentCodec].numChannels;
    unsigned int queuedSampleCount = 0;
    int32_t currentSamples[A2DP_ENCODE_SAMPLE_BATCH_SIZE] = { 0, };

    // Send as many raw samples as we can to the Encoder on Core 1

    while (ctx->samples_ready > A2DP_ENCODE_SAMPLE_BATCH_SIZE 
            && !BT_Codecs_isEncoderOverloaded()) {

        sampleCount = ctx->audioProviderFunc(channels,
                                             currentSamples, 
                                             A2DP_ENCODE_SAMPLE_BATCH_SIZE / channels);

        if (sampleCount > 0) {
            queuedSampleCount = BT_Codecs_queueEncodeAsync(currentSamples, sampleCount * channels);

            if (queuedSampleCount != (sampleCount * channels)) {
                printf("[A2DP Source]: Codecs encode queue full, dropped %i samples\n",
                        (sampleCount * channels) - queuedSampleCount);
                stdio_flush();
            }

            ctx->samples_ready -= sampleCount;
        }
    }
}

static void readSamplesFromEncoder(a2dp_media_sending_context_t *ctx) {
    unsigned int numBytes;
    
    unsigned int frameSize = BT_Codecs_getFrameSize();
    bool encoderBusy = false;

    // Read as many samples as we can from the Encoder on Core 1,
    // until we gather enough for a full media packet or the Encoder stops giving us
    // them without timing out.
    while((ctx->encodedDataCount + frameSize) < ctx->max_media_payload_size 
            && !encoderBusy) {
        // Grab one frame at a time
        // 1 Byte offset to store the header (if necessary) in front
        numBytes = BT_Codecs_getEncoderOutputAsync(&ctx->encodedData[ctx->encodedDataCount + 1], 1);
        
        // Increment our encoded Data buffer counter
        // And number of frames counter
        ctx->encodedDataCount += numBytes;
        ctx->encodedFrameCount += 1;

        // If the encoder returned zero bytes then there is no data available, so quit asking for now
        encoderBusy = (numBytes == 0);
    }


    // Set ready_to_send flag if we can not fit anymore into a packet
    ctx->ready_to_send = ((ctx->encodedDataCount + frameSize) > ctx->max_media_payload_size);
}

void BT_A2DPSource_connect(bd_addr_t address) {
    a2dp_source_establish_stream(address, &media_tracker.a2dp_cid);
}

void BT_A2DPSource_update(void) {
    static uint32_t lastUpdateAt = 0;

    if (media_tracker.streaming) {

    }
}

void BT_A2DPSource_updateAudioStatus(struct BT_Controller_AudioStatus_T *status) {
    status->streamConnected = media_tracker.stream_opened;
    status->playing = media_tracker.streaming;
}

void BT_A2DPSource_init(InterleavedSampleProvider audioProvider) {
    printf("[A2DP Source]: Initializing.\n");

    gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_SNIFF_MODE | LM_LINK_POLICY_ENABLE_ROLE_SWITCH);

    // Request role change on reconnecting headset to always use them in slave mode
    hci_set_master_slave_policy(HCI_ROLE_MASTER);

    l2cap_init();

    // Initialize  A2DP Source
    a2dp_source_init();
    a2dp_source_register_packet_handler(&a2dp_source_packet_handler);

    // Store Audio Provider function pointer
    media_tracker.audioProviderFunc = audioProvider;

    if (!setupEndpoints()) {
        printf("[A2DP Source]: failed to create stream endpoints!\n");
    } else {
        // Initialize AVRCP Service
        avrcp_init();
        avrcp_register_packet_handler(&avrcp_packet_handler);

        // Initialize AVRCP Target
        avrcp_target_init();
        avrcp_target_register_packet_handler(&avrcp_target_packet_handler);

        // Initialize AVRCP Controller
        avrcp_controller_init();
        avrcp_controller_register_packet_handler(&avrcp_controller_packet_handler);

        // Initialize SDP, 
        sdp_init();
        
        // Create A2DP Source service record and register it with SDP
        memset(sdp_a2dp_source_service_buffer, 0, sizeof(sdp_a2dp_source_service_buffer));
        a2dp_source_create_sdp_record(sdp_a2dp_source_service_buffer,
                                      sdp_create_service_record_handle(), 
                                      AVDTP_SOURCE_FEATURE_MASK_PLAYER,
                                      NULL, NULL);
        btstack_assert(de_get_len( sdp_a2dp_source_service_buffer) <= sizeof(sdp_a2dp_source_service_buffer));
        sdp_register_service(sdp_a2dp_source_service_buffer);
        
        // Create AVRCP Target service record and register it with SDP. We receive Category 1 commands from the headphone, e.g. play/pause
        memset(sdp_avrcp_target_service_buffer, 0, sizeof(sdp_avrcp_target_service_buffer));
        uint16_t supported_features = AVRCP_FEATURE_MASK_CATEGORY_PLAYER_OR_RECORDER;
#ifdef AVRCP_BROWSING_ENABLED
        supported_features |= AVRCP_FEATURE_MASK_BROWSING;
#endif
        avrcp_target_create_sdp_record(sdp_avrcp_target_service_buffer, 
                                       sdp_create_service_record_handle(),
                                       supported_features, NULL, NULL);
        btstack_assert(de_get_len( sdp_avrcp_target_service_buffer) <= sizeof(sdp_avrcp_target_service_buffer));
        sdp_register_service(sdp_avrcp_target_service_buffer);

        // Create AVRCP Controller service record and register it with SDP. We send Category 2 commands to the headphone, e.g. volume up/down
        memset(sdp_avrcp_controller_service_buffer, 0, sizeof(sdp_avrcp_controller_service_buffer));
        uint16_t controller_supported_features = AVRCP_FEATURE_MASK_CATEGORY_MONITOR_OR_AMPLIFIER;
        avrcp_controller_create_sdp_record(sdp_avrcp_controller_service_buffer,
                                           sdp_create_service_record_handle(),
                                           controller_supported_features, NULL, NULL);
        btstack_assert(de_get_len( sdp_avrcp_controller_service_buffer) <= sizeof(sdp_avrcp_controller_service_buffer));
        sdp_register_service(sdp_avrcp_controller_service_buffer);

        // Register Device ID (PnP) service SDP record
        memset(device_id_sdp_service_buffer, 0, sizeof(device_id_sdp_service_buffer));
        device_id_create_sdp_record(device_id_sdp_service_buffer,
                                    sdp_create_service_record_handle(),
                                    DEVICE_ID_VENDOR_ID_SOURCE_BLUETOOTH,
                                    BLUETOOTH_COMPANY_ID_BLUEKITCHEN_GMBH, 1, 1);
        btstack_assert(de_get_len( device_id_sdp_service_buffer) <= sizeof(device_id_sdp_service_buffer));
        sdp_register_service(device_id_sdp_service_buffer);


        printf("[A2DP Source]: Initialized.\n");
    }
}

static bool setupEndpoints(void) {
    bool success = false;
    avdtp_stream_endpoint_t *endpointSBC;
    avdtp_stream_endpoint_t *endpointAAC;
    avdtp_stream_endpoint_t *endpointLDAC;
    
    // Create stream endpoints
    endpointSBC = a2dp_source_create_stream_endpoint(AVDTP_AUDIO,
                                                     AVDTP_CODEC_SBC, 
                                                     media_sbc_codec_capabilities, 
                                                     sizeof(media_sbc_codec_capabilities),
                                                     media_sbc_codec_configuration, 
                                                     sizeof(media_sbc_codec_configuration));
#if 0
    endpointLDAC = a2dp_source_create_stream_endpoint(AVDTP_AUDIO,
                                                      AVDTP_CODEC_NON_A2DP,
                                                      media_ldac_codec_capabilities,
                                                      sizeof(media_ldac_codec_capabilities),
                                                      ldac_media_codec_configuration,
                                                      sizeof(ldac_media_codec_configuration));
    if (endpointSBC && endpointLDAC) {
#endif
    if (endpointSBC) {
        success = true;

        avdtp_set_preferred_sampling_frequency(endpointSBC, AUDIO_PLAYBACK_SAMPLE_RATE);

        // Store stream enpoint's SEP ID, as it is used by A2DP API to indentify the stream endpoint
        media_tracker.codecs[BT_SBC_CODEC].local_seid = avdtp_local_seid(endpointSBC);

        printf("[A2DP Source]: SBC Endpoint created, local SEP ID: 0x%02x\n",
                media_tracker.codecs[BT_SBC_CODEC].local_seid);
#if 0
        avdtp_set_preferred_sampling_frequency(endpointLDAC, AUDIO_PLAYBACK_SAMPLE_RATE);

        media_tracker.codecs[BT_LDAC_CODEC].local_seid = avdtp_local_seid(endpointLDAC);

        printf("[A2DP Source]: LDAC Endpoint created, local SEP ID: 0x%02x\n",
                media_tracker.codecs[BT_LDAC_CODEC].local_seid);
#endif
    }

    return success;
}

static void a2dp_demo_send_media_packet(void) {
    uint8_t *packedData;
    unsigned int packedLength;

    // calculate header, and place in front (if required)
    if (media_tracker.currentCodec == BT_SBC_CODEC || media_tracker.currentCodec == BT_LDAC_CODEC) {
        // Add header to first byte (stores the number of frames)
        media_tracker.encodedData[0] = media_tracker.encodedFrameCount;

        packedData = &media_tracker.encodedData[0];
        packedLength = media_tracker.encodedDataCount + 1;
    } else {
        // No header required, skip first empty byte
        packedData = &media_tracker.encodedData[1];
        packedLength = media_tracker.encodedDataCount;
    }

    a2dp_source_stream_send_media_payload_rtp(media_tracker.a2dp_cid, getActiveLocalSeid(&media_tracker), 0,
                                              media_tracker.rtp_timestamp,
                                              packedData, packedLength);

    media_tracker.rtp_timestamp += media_tracker.encodedFrameCount;

    // Clear encoded Data buffer
    media_tracker.encodedDataCount = 0;
    media_tracker.encodedFrameCount = 0;
    media_tracker.ready_to_send = false;
}

static void a2dp_demo_audio_timeout_handler(btstack_timer_source_t * timer){
    int encodedSampleCount;
    a2dp_media_sending_context_t * context = (a2dp_media_sending_context_t *) btstack_run_loop_get_timer_context(timer);
    btstack_run_loop_set_timer(&context->audio_timer, AUDIO_TIMEOUT_MS); 
    btstack_run_loop_add_timer(&context->audio_timer);
    uint32_t now = btstack_run_loop_get_time_ms();

    uint32_t update_period_ms = AUDIO_TIMEOUT_MS;
    if (context->time_audio_data_sent > 0){
        update_period_ms = now - context->time_audio_data_sent;
    } 

    uint32_t num_samples = (update_period_ms * AUDIO_PLAYBACK_SAMPLE_RATE) / 1000;
    context->acc_num_missed_samples += (update_period_ms * AUDIO_PLAYBACK_SAMPLE_RATE) % 1000;
    
    while (context->acc_num_missed_samples >= 1000){
        num_samples++;
        context->acc_num_missed_samples -= 1000;
    }
    context->time_audio_data_sent = now;
    context->samples_ready += num_samples;

    sendSamplesToEncoder(&media_tracker);
    readSamplesFromEncoder(&media_tracker);

    if (context->ready_to_send) {
        // schedule sending
        a2dp_source_stream_endpoint_request_can_send_now(context->a2dp_cid, getActiveLocalSeid(context));
    }
}

static void a2dp_demo_timer_start(a2dp_media_sending_context_t * context){
    context->max_media_payload_size = A2DP_MAX_MEDIA_PAYLOAD_SIZE; 
    //btstack_min(a2dp_max_media_payload_size(context->a2dp_cid, context->local_seid), SBC_ENCODER_STORAGE_SIZE);
    context->ready_to_send = false;
    context->encodedDataCount = 0;
    context->streaming = 1;
    context->rtp_timestamp = 0;

    btstack_run_loop_remove_timer(&context->audio_timer);
    btstack_run_loop_set_timer_handler(&context->audio_timer, a2dp_demo_audio_timeout_handler);
    btstack_run_loop_set_timer_context(&context->audio_timer, context);
    btstack_run_loop_set_timer(&context->audio_timer, AUDIO_TIMEOUT_MS); 
    btstack_run_loop_add_timer(&context->audio_timer);
}

static void a2dp_demo_timer_stop(a2dp_media_sending_context_t * context){
    context->time_audio_data_sent = 0;
    context->acc_num_missed_samples = 0;
    context->samples_ready = 0;
    context->streaming = 0;
    context->encodedDataCount = 0;
    context->ready_to_send = false;

    btstack_run_loop_remove_timer(&context->audio_timer);
}

static void onSBCConfigurationRecieved(uint8_t *packet) {
    uint8_t remote_seid;
    BT_SBC_processConfigPacket(packet, &media_tracker.sbcConfig);

    remote_seid = a2dp_subevent_signaling_media_codec_sbc_configuration_get_remote_seid(packet);

    media_tracker.codecs[BT_SBC_CODEC].remote_seid = remote_seid;
    media_tracker.codecs[BT_SBC_CODEC].numChannels = media_tracker.sbcConfig.num_channels;
}

static void onAACConfigurationReceived(uint8_t *packet) {
    // TODO
    printf("[A2DP Source]: Got AAC configuration, but not implemented.\n");
}

static void onLDACConfigurationReceived(uint8_t *packet) {
    printf("[A2DP Source]: Got LDAC configuration, but not implemented.\n");
}

static void onA2DPStreamEstablished(uint8_t *packet) {
    char lcdStatusText[256];
    uint8_t remote_seid;
    uint8_t local_seid;
    uint8_t cid;
    a2dp_codec_info *codec;
    uint8_t status = a2dp_subevent_stream_established_get_status(packet);

    if (status != ERROR_CODE_SUCCESS){
        printf("[A2DP Source]: Stream failed, status 0x%02x.\n", status);
    } else {
        remote_seid = a2dp_subevent_stream_established_get_remote_seid(packet);
        local_seid = a2dp_subevent_stream_established_get_local_seid(packet);
        cid = a2dp_subevent_stream_established_get_a2dp_cid(packet);
        
        printf("[A2DP Source]: Stream established a2dp_cid 0x%02x, local_seid 0x%02x, remote_seid 0x%02x\n",
                cid, local_seid, remote_seid);

        // TODO: I don't like how this is done right now
        if (media_tracker.stream_opened) {
            printf("Stream ALREADY Opened!\n");
        } else {
            media_tracker.stream_opened = 1;

            codec = findCodecByRemoteSeid(remote_seid);
            if (codec == NULL) {
                printf("[A2DP Source]: Failed to determine Codec by Remote SEID, something very wrong\n");
                assert(0);
            } else {
                switch(media_tracker.currentCodec) {
                    case BT_SBC_CODEC:
                        BT_Codecs_setActiveCodecSBC(media_tracker.sbcConfig);

                        status = a2dp_source_start_stream(media_tracker.a2dp_cid, local_seid);

                        sprintf(lcdStatusText, "SBC %ib @ %i kHz", AUDIO_WORDSIZE, media_tracker.sbcConfig.sampling_frequency);
                        UIController_setBottomText(lcdStatusText);

                        break;
                    default:
                        printf("[A2DP Source]: Failed to determine Codec by Remote SEID, something very wrong\n");
                        assert(0);
                        break;
                }
            }
        }
    }
}

static void a2dp_source_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    char uiStatusText[64];
    UNUSED(channel);
    UNUSED(size);
    struct BT_SBC_Configuration_T sbc_configuration;
    uint8_t status;
    uint8_t local_seid;
    uint8_t remote_seid;
    uint8_t *otherCodecInfo;
    bd_addr_t address;
    uint16_t cid;
    a2dp_codec_info *codec;

    avdtp_channel_mode_t channel_mode;
    uint8_t allocation_method;

    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_A2DP_META) return;

    switch (hci_event_a2dp_meta_get_subevent_code(packet)){
        case A2DP_SUBEVENT_SIGNALING_CONNECTION_ESTABLISHED:
            a2dp_subevent_signaling_connection_established_get_bd_addr(packet, address);
            cid = a2dp_subevent_signaling_connection_established_get_a2dp_cid(packet);
            status = a2dp_subevent_signaling_connection_established_get_status(packet);

            if (status != ERROR_CODE_SUCCESS){
                printf("[A2DP Source]: Connection failed, status 0x%02x, cid 0x%02x, a2dp_cid 0x%02x \n", 
                        status, cid, media_tracker.a2dp_cid);
                media_tracker.a2dp_cid = 0;
                break;
            }

            media_tracker.a2dp_cid = cid;

            printf("[A2DP Source]: Connected to address %s, a2dp cid 0x%02x\n", 
                    bd_addr_to_str(address), 
                    media_tracker.a2dp_cid);

            sprintf(uiStatusText, "To: %s", bd_addr_to_str(address));
            UIController_setTopText(uiStatusText);

            a2dp_source_establish_stream(address, &media_tracker.a2dp_cid);
            break;

        case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION:
            cid  = avdtp_subevent_signaling_media_codec_sbc_configuration_get_avdtp_cid(packet);
            if (cid != media_tracker.a2dp_cid) return;

            onSBCConfigurationRecieved(packet);
            break;

        case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_MPEG_AAC_CONFIGURATION:
            cid  = avdtp_subevent_signaling_media_codec_mpeg_aac_capability_get_avdtp_cid(packet);
            if (cid != media_tracker.a2dp_cid) return;

            onAACConfigurationReceived(packet);
            break;

        case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_OTHER_CONFIGURATION:
            printf("Got OTHER configuration\n");
            break;

        case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_OTHER_CAPABILITY:
            cid = a2dp_subevent_signaling_media_codec_other_capability_get_a2dp_cid(packet);
            if (cid != media_tracker.a2dp_cid) return;

            break;

        case A2DP_SUBEVENT_SIGNALING_DELAY_REPORTING_CAPABILITY:
            printf("[A2DP Source]: remote supports delay report, remote seid %d\n", 
                avdtp_subevent_signaling_delay_reporting_capability_get_remote_seid(packet));
            break;

        case A2DP_SUBEVENT_SIGNALING_CAPABILITIES_DONE:
            printf("[A2DP Source]: All capabilities reported, remote seid %d\n", 
                avdtp_subevent_signaling_capabilities_done_get_remote_seid(packet));
            break;

        case A2DP_SUBEVENT_SIGNALING_DELAY_REPORT:
            printf("[A2DP Source]: Received delay report of %d.%0d ms, local seid %d\n", 
                avdtp_subevent_signaling_delay_report_get_delay_100us(packet)/10,
                avdtp_subevent_signaling_delay_report_get_delay_100us(packet)%10,
                avdtp_subevent_signaling_delay_report_get_local_seid(packet));
            break;
       
        case A2DP_SUBEVENT_STREAM_ESTABLISHED:
            onA2DPStreamEstablished(packet);
            break;

        case A2DP_SUBEVENT_STREAM_RECONFIGURED:
            status = a2dp_subevent_stream_reconfigured_get_status(packet);
            local_seid = a2dp_subevent_stream_reconfigured_get_local_seid(packet);
            cid = a2dp_subevent_stream_reconfigured_get_a2dp_cid(packet);

            if (status != ERROR_CODE_SUCCESS){
                printf("[A2DP Source]: Stream reconfiguration failed, status 0x%02x\n", status);
                break;
            }

            printf("[A2DP Source]: Stream reconfigured a2dp_cid 0x%02x, local_seid 0x%02x\n", cid, local_seid);
            status = a2dp_source_start_stream(media_tracker.a2dp_cid, local_seid);
            break;

        case A2DP_SUBEVENT_STREAM_STARTED:
            local_seid = a2dp_subevent_stream_started_get_local_seid(packet);
            cid = a2dp_subevent_stream_started_get_a2dp_cid(packet);

            // We don't have "tracks", since streaming from ADC, so just pass some default info in.
            avrcp_target_set_now_playing_info(media_tracker.avrcp_cid, &DEFAULT_TRACK_INFO, 1);

            a2dp_demo_timer_start(&media_tracker);
            printf("[A2DP Source]: Stream started, a2dp_cid 0x%02x, local_seid 0x%02x\n", cid, local_seid);
            break;

        case A2DP_SUBEVENT_STREAMING_CAN_SEND_MEDIA_PACKET_NOW:
            local_seid = a2dp_subevent_streaming_can_send_media_packet_now_get_local_seid(packet);
            cid = a2dp_subevent_signaling_media_codec_sbc_configuration_get_a2dp_cid(packet);
            a2dp_demo_send_media_packet();
            break;        

        case A2DP_SUBEVENT_STREAM_SUSPENDED:
            local_seid = a2dp_subevent_stream_suspended_get_local_seid(packet);
            cid = a2dp_subevent_stream_suspended_get_a2dp_cid(packet);
            
            printf("[A2DP Source]: Stream paused, a2dp_cid 0x%02x, local_seid 0x%02x\n", cid, local_seid);
            
            a2dp_demo_timer_stop(&media_tracker);
            break;

        case A2DP_SUBEVENT_STREAM_RELEASED:
            cid = a2dp_subevent_stream_released_get_a2dp_cid(packet);
            local_seid = a2dp_subevent_stream_released_get_local_seid(packet);
            
            printf("[A2DP Source]: Stream released, a2dp_cid 0x%02x, local_seid 0x%02x\n", cid, local_seid);

            if (cid == media_tracker.a2dp_cid) {
                media_tracker.stream_opened = 0;
                printf("[A2DP Source]: Stream released.\n");
                UIController_setBottomText("IDLE");
            }
            a2dp_demo_timer_stop(&media_tracker);
            break;
        case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
            cid = a2dp_subevent_signaling_connection_released_get_a2dp_cid(packet);
            if (cid == media_tracker.a2dp_cid) {
                media_tracker.avrcp_cid = 0;
                media_tracker.a2dp_cid = 0;
                printf("[A2DP Source]: Signaling released.\n\n");

                UIController_setTopText("-> Not Connected");
            }
            break;
        default:
            break; 
    }
}

static void avrcp_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    bd_addr_t event_addr;
    uint16_t local_cid;
    uint8_t  status = ERROR_CODE_SUCCESS;

    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) return;
    
    switch (packet[2]){
        case AVRCP_SUBEVENT_CONNECTION_ESTABLISHED: 
            local_cid = avrcp_subevent_connection_established_get_avrcp_cid(packet);
            status = avrcp_subevent_connection_established_get_status(packet);
            if (status != ERROR_CODE_SUCCESS){
                printf("[A2DP Source]: AVRCP: Connection failed, local cid 0x%02x, status 0x%02x\n",
                        local_cid, status);
                return;
            }
            media_tracker.avrcp_cid = local_cid;
            avrcp_subevent_connection_established_get_bd_addr(packet, event_addr);

            printf("[A2DP Source]: AVRCP: Channel to %s successfully opened, avrcp_cid 0x%02x\n", 
                    bd_addr_to_str(event_addr), media_tracker.avrcp_cid);

#if 0 // Not supporting these, since we are streaming from raw ADC audio
            avrcp_target_support_event(media_tracker.avrcp_cid, AVRCP_NOTIFICATION_EVENT_PLAYBACK_STATUS_CHANGED);
            avrcp_target_support_event(media_tracker.avrcp_cid, AVRCP_NOTIFICATION_EVENT_TRACK_CHANGED);
            avrcp_target_support_event(media_tracker.avrcp_cid, AVRCP_NOTIFICATION_EVENT_NOW_PLAYING_CONTENT_CHANGED);
            avrcp_target_set_now_playing_info(media_tracker.avrcp_cid, NULL, sizeof(tracks)/sizeof(avrcp_track_t));
#endif

            avrcp_controller_enable_notification(media_tracker.avrcp_cid, AVRCP_NOTIFICATION_EVENT_VOLUME_CHANGED);
            avrcp_controller_enable_notification(media_tracker.avrcp_cid, AVRCP_NOTIFICATION_EVENT_BATT_STATUS_CHANGED);
            return;
        
        case AVRCP_SUBEVENT_CONNECTION_RELEASED:
            printf("[A2DP Source]: AVRCP Target: Disconnected, avrcp_cid 0x%02x\n",
                    avrcp_subevent_connection_released_get_avrcp_cid(packet));
            media_tracker.avrcp_cid = 0;
            return;
        default:
            break;
    }

    if (status != ERROR_CODE_SUCCESS){
        printf("[A2DP Source]: Responding to event 0x%02x failed, status 0x%02x\n", packet[2], status);
    }
}

static void avrcp_target_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    uint8_t  status = ERROR_CODE_SUCCESS;

    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) return;

    bool button_pressed;
    char const * button_state;
    avrcp_operation_id_t operation_id;

    switch (packet[2]){
        case AVRCP_SUBEVENT_OPERATION:
            operation_id = (avrcp_operation_id_t) avrcp_subevent_operation_get_operation_id(packet);
            button_pressed = avrcp_subevent_operation_get_button_pressed(packet) > 0;
            button_state = button_pressed ? "PRESS" : "RELEASE";

            printf("[A2DP Source]: AVRCP Target: operation %s (%s)\n",
                    avrcp_operation2str(operation_id), button_state);

            if (!button_pressed){
                break;
            }
            switch (operation_id) {
                case AVRCP_OPERATION_ID_PLAY:
                    status = a2dp_source_start_stream(media_tracker.a2dp_cid, getActiveLocalSeid(&media_tracker));
                    break;
                case AVRCP_OPERATION_ID_PAUSE:
                    status = a2dp_source_pause_stream(media_tracker.a2dp_cid, getActiveLocalSeid(&media_tracker));
                    break;
                case AVRCP_OPERATION_ID_STOP:
                    status = a2dp_source_disconnect(media_tracker.a2dp_cid);
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }

    if (status != ERROR_CODE_SUCCESS){
        printf("[A2DP Source]: AVRCP Responding to event 0x%02x failed, status 0x%02x\n", packet[2], status);
    }
}

static void avrcp_controller_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    
    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) return;
    if (!media_tracker.avrcp_cid) return;
    
    switch (packet[2]){
        case AVRCP_SUBEVENT_NOTIFICATION_VOLUME_CHANGED:
            printf("[A2DP Source]: AVRCP Controller: Notification Absolute Volume %d %%\n",
                    avrcp_subevent_notification_volume_changed_get_absolute_volume(packet) * 100 / 127);
            break;
        case AVRCP_SUBEVENT_NOTIFICATION_EVENT_BATT_STATUS_CHANGED:
            // see avrcp_battery_status_t
            printf("[A2DP Source]: AVRCP Controller: Notification Battery Status 0x%02x\n",
                    avrcp_subevent_notification_event_batt_status_changed_get_battery_status(packet));
            break;
        case AVRCP_SUBEVENT_NOTIFICATION_STATE:
            printf("[A2DP Source]: AVRCP Controller: Notification %s - %s\n", 
                avrcp_event2str(avrcp_subevent_notification_state_get_event_id(packet)), 
                avrcp_subevent_notification_state_get_enabled(packet) != 0 ? "enabled" : "disabled");
            break;
        default:
            break;
    }  
}
