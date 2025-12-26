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
#include "bt/bt_controller.h"
#include "audio_common.h"
#include "adc_controller.h"
#include "bt/a2dp_source.h"
#include "bt/codecs.h"
#include "util.h"

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"

#include <btstack_defines.h>
#include <btstack_event.h>
#include <btstack_util.h>
#include <gap.h>
#include <pico/stdio.h>
#include <stdint.h>
#include <stdio.h>

static const InterleavedSampleProvider AUDIO_PLAYBACK_PROVIDER = &ADCController_audioSampleProvider;
//static const InterleavedSampleProvider AUDIO_PLAYBACK_PROVIDER = &Util_produceSinAudio;

static const char *BT_DEVICE_NAME = "Wireless Audio Device";

/** Duration of scanning for devices, in units of 1280ms */
static const uint8_t INQUIRY_DURATION = 12;
static const BT_Controller_Mode BTMODE = BTCONTROLLER_AUDIO_SOURCE;

static BT_Controller_Context btContext;

static void onScanDeviceFound(uint8_t *eventPacket);
static void hciPacketHandler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static void onScanDeviceFound(uint8_t *eventPacket) {
    struct BT_Controller_ScannedDevice_T *deviceStruct;
    unsigned int nameLen;

    // Check if there is space in the device array for another
    if (btContext.scanResults.numDevices >= BT_CONTROLLER_MAX_DEVICES_SCANNED) {
        // Not enough space in array, drop the device
        printf("[BT]: Scanned Device Array full, dropping found device.\n");
    } else {
        // Grab the next available device struct in the array
        deviceStruct = &btContext.scanResults.devices[btContext.scanResults.numDevices];

        // Obtain address and class of device we found
        gap_event_inquiry_result_get_bd_addr(eventPacket, deviceStruct->address);
        deviceStruct->classOfDevice = gap_event_inquiry_result_get_class_of_device(eventPacket);

        // Get device name information
        if (gap_event_inquiry_result_get_name_available(eventPacket)) {
            nameLen = gap_event_inquiry_result_get_name_len(eventPacket);

            // Cut off any extra characters if larger than the name buffer
            nameLen = (nameLen >= BT_CONTROLLER_MAX_DEVICE_NAME_SIZE) 
                        ? (BT_CONTROLLER_MAX_DEVICE_NAME_SIZE - 1) : nameLen;

            // Store the name in the struct
            memcpy(deviceStruct->name, gap_event_inquiry_result_get_name(eventPacket), nameLen);

            // Zero terminate string
            deviceStruct->name[nameLen] = 0;
        }

        printf("[BT]: Device found: %s (%s), COD: %i", 
                bd_addr_to_str(deviceStruct->address),
                deviceStruct->name,
                deviceStruct->classOfDevice);

        // Increment device 
        btContext.scanResults.numDevices++;
    }
}

static void hciPacketHandler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    // Service Class: Rendering | Audio, Major Device Class: Audio
    const uint32_t bluetooth_speaker_cod = 0x200000 | 0x040000 | 0x000400;
    bd_addr_t address;
    const char *remoteName;

    if (packet_type == HCI_EVENT_PACKET) {
        switch (hci_event_packet_get_type(packet)){
            case  BTSTACK_EVENT_STATE:
                if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING
                    && BTMODE == BTCONTROLLER_AUDIO_SOURCE) {
                    printf("[BT]: Starting scan...\n");
                    
                    btContext.scanResults.scanActive = true;
                    btContext.scanResults.numDevices = 0;
                    gap_inquiry_start(INQUIRY_DURATION);
                }
                break;

            case HCI_EVENT_CONNECTION_COMPLETE:
                // Obtain the BT address of the device we just connected to. 
                hci_event_connection_complete_get_bd_addr(packet, btContext.peerAddr);
                btContext.connected = true;

                gap_remote_name_request(btContext.peerAddr, 0, 0);

                // Stop scanning, a device is connected
                BT_Controller_stopScan();

                printf("[BT]: Device Connected: %s\n", bd_addr_to_str(btContext.peerAddr));
                break;

            case HCI_EVENT_REMOTE_NAME_REQUEST_COMPLETE:
                remoteName = hci_event_remote_name_request_complete_get_remote_name(packet);
                btstack_strcpy(btContext.peerName, BT_CONTROLLER_MAX_DEVICE_NAME_SIZE, remoteName);

                printf("[BT]: Connected device reports a name of: \"%s\"\n", remoteName);
                break;

            case HCI_EVENT_DISCONNECTION_COMPLETE:
                printf("[BT]: Device disconnected.\n");
                btContext.connected = false;
                break;

            case HCI_EVENT_PIN_CODE_REQUEST:
                printf("[BT]: Pin code request - using '0000'\n");
                hci_event_pin_code_request_get_bd_addr(packet, address);
                gap_pin_code_response(address, "0000");
                break;

            case GAP_EVENT_INQUIRY_RESULT:
                onScanDeviceFound(packet);
                break;

            case GAP_EVENT_INQUIRY_COMPLETE:
                printf("[BT]: Scan complete.\n");

#if CONTINOUS_SCANNING
                if (btContext.scanResults.scanActive && !btContext.connected && !btContext.connecting) {
                    printf("[BT]: Starting scan...\n");

                    btContext.scanResults.numDevices = 0;
                    gap_inquiry_start(INQUIRY_DURATION);
                } else if (btContext.scanResults.scanActive) {
                    btContext.scanResults.scanActive = false;
                }
#else 
                btContext.scanResults.scanActive = false;
#endif // CONTINOUS_SCANNING
                break;

            default:
                break;
        }
    }
}

void BT_Controller_init(void) {
    static btstack_packet_callback_registration_t hci_event_callback_registration;

    // initialize bluetooth chip driver
    if (cyw43_arch_init()) {
        printf("[BT]: failed to initialise cyw43_arch\n");
        stdio_flush();
        assert(0);
    }

    // enabled EIR
    hci_set_inquiry_mode(INQUIRY_MODE_RSSI_AND_EIR);

    // Start Codecs Module
    BT_Codecs_init();

    // Initialize as A2DP Source or Sink depending on our mode
    if (BTMODE == BTCONTROLLER_AUDIO_SOURCE) {
        BT_A2DPSource_init(AUDIO_PLAYBACK_PROVIDER);
    }

    // Set our advertised Bluetooth device name and make discoverable
    gap_set_local_name(BT_DEVICE_NAME);
    gap_discoverable_control(1);
    gap_set_class_of_device(0x200408);
    
    // Register HCI event handler
    hci_event_callback_registration.callback = &hciPacketHandler;
    hci_add_event_handler(&hci_event_callback_registration);

    // Turn on Bluetooth chip
    hci_power_control(HCI_POWER_ON);
    printf("[BT]: Powered on\n");
}

void BT_Controller_update(void) {
    static uint32_t lastUpdateAt = 0;

    uint32_t currentTime = Util_getTimeMs();
    if (currentTime - lastUpdateAt >= 2000) {
        // TODO: This is just for testing, hardcoded an auto connect to a test BT device
        if (!btContext.connecting && !btContext.connected) {
            for (int i = 0; i < btContext.scanResults.numDevices; i++) {
                if (strcmp(btContext.scanResults.devices[i].name, "Nothing Ear") == 0) {
                    printf("Found nothing ear!\n");
                    BT_Controller_connect(btContext.scanResults.devices[i].address);
                    break;
                } else {
                    printf("No match for nothing ear. [%s]\n", btContext.scanResults.devices[i].name);
                }
            }
        }

        lastUpdateAt = currentTime;
    }

    BT_A2DPSource_update();
}

void BT_Controller_connect(bd_addr_t address) {
    // TODO: check mode
    btContext.connecting = true;
    
    if (btContext.scanResults.scanActive) {
        btContext.scanResults.scanActive = false;
        gap_inquiry_stop();
    }

    BT_A2DPSource_connect(address);
}

void BT_Controller_startScan(void) {
    if (!btContext.scanResults.scanActive) {
        gap_inquiry_start(INQUIRY_DURATION);
    }
}

void BT_Controller_stopScan(void) {
    if (btContext.scanResults.scanActive) {
        gap_inquiry_stop();
    }
}
