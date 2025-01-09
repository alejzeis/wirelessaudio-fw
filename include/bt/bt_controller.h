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
#ifndef BT_BT_CONTROLLER_H
#define BT_BT_CONTROLLER_H

#include "btstack.h"

#define BT_CONTROLLER_SCAN_CHECK_COD 0

#define BT_CONTROLLER_MAX_DEVICES_SCANNED 32
#define BT_CONTROLLER_MAX_DEVICE_NAME_SIZE 96

#define BT_CONTROLLER_MAX_CODEC_NAME_SIZE 32
#define BT_CONTROLLER_MAX_TRACK_TITLE_SIZE 96

typedef enum BT_Controller_Mode_E {
    BTCONTROLLER_AUDIO_SINK,
    BTCONTROLLER_AUDIO_SOURCE
} BT_Controller_Mode;

struct BT_Controller_ScannedDevice_T {
    // Address of this bluetooth device
    bd_addr_t address;

    // Stores the bluetooth Class Of Device (COD)
    uint32_t classOfDevice;

    // The name and length of the device. May be zero
    char name[BT_CONTROLLER_MAX_DEVICE_NAME_SIZE];
    unsigned int nameLength;
};

struct BT_Controller_AudioStatus_T {
    bool streamConnected;
    bool playing;

    char codec[BT_CONTROLLER_MAX_CODEC_NAME_SIZE];
    char title[BT_CONTROLLER_MAX_TRACK_TITLE_SIZE];
};

struct BT_Controller_ScanResults_T {
    // Indicates if a scan is currently in progress
    bool scanActive;
    // Array containing all currently found devices
    struct BT_Controller_ScannedDevice_T devices[BT_CONTROLLER_MAX_DEVICES_SCANNED];
    // The amount of devices present in the "devices" array above.
    unsigned int numDevices;
};

typedef struct BT_Controller_Context_T {
    struct BT_Controller_ScanResults_T scanResults;

    bool connecting;
    bool connected;
    /** Bluetooth address of the device currently connected to. */
    bd_addr_t peerAddr;
    /** The name of the device currently connected to. */
    char peerName[BT_CONTROLLER_MAX_DEVICE_NAME_SIZE];

    /* Audio Status */
    struct BT_Controller_AudioStatus_T audioStatus;
} BT_Controller_Context;


void BT_Controller_init(void);

void BT_Controller_update(void);

void BT_Controller_connect(bd_addr_t address);

void BT_Controller_startScan(void);
void BT_Controller_stopScan(void);

#endif // BT_BT_CONTROLLER_H
