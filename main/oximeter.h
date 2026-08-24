/*
 * SomnoTrace - O2 Ring (OxyII) oximeter BLE sync — public API
 * Copyright (C) 2026 Ilya Kruchinin <https://github.com/ilyakruchinin>
 *
 * This file is part of SomnoTrace.
 *
 * SomnoTrace is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * SomnoTrace is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <https://www.gnu.org/licenses/>.
 *
 * ADDITIONAL TERM (GPLv3 Section 7(b)): Redistributions must preserve the
 * attribution "Based on SomnoTrace, originally created by Ilya Kruchinin
 * (https://github.com/ilyakruchinin)." See the NOTICE file for details.
 *
 * Clean-room implementation of the OxyII BLE protocol for Wellue O2 Ring S
 * and SleepHQ O2 Ring Pro. Protocol studied from published documentation;
 * no third-party source code copied. See spec/0003-o2ring-ble-sync.md.
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "cJSON.h"

/* Oximeter state machine states (returned by oximeter_get_status). */
#define OX_STATUS_IDLE       "idle"
#define OX_STATUS_SCANNING   "scanning"
#define OX_STATUS_CONNECTING "connecting"
#define OX_STATUS_PULLING    "pulling"
#define OX_STATUS_PAIRED     "paired"
#define OX_STATUS_ERROR      "error"

/* Protocol backend identifiers (persisted in NVS + paired.json).
 * These are distinct Wellue protocols with incompatible framing:
 *  - OXYII:  O2 Ring S / SleepHQ O2 Ring Pro, 0xA5 frames
 *            (oximeter_oxyii.c)
 *  - LEGACY: legacy Wellue rings (P02 / O2Ring), 0xAA frames
 *            (oximeter_legacy.c) */
#define OX_PROTO_OXYII  "oxyii"
#define OX_PROTO_LEGACY "legacy"

/* Initialise the oximeter module.  Must be called after as11_ble_init()
 * (shares the NimBLE host) and sd_storage_init().  Loads paired serial
 * from NVS and starts the background watch task. */
esp_err_t oximeter_init(void);

/* Start a BLE scan for OxyII rings (SHQO2Pro / S8-AW, mfg 0xF34E).
 * Blocks until scan completes.  AS11 connection stays up.
 * Results retrieved via oximeter_get_scan_results(). */
esp_err_t oximeter_scan(int timeout_sec);

/* Cancel an in-progress oximeter_scan() early.  Results collected so far
 * are kept; harmless if no scan is running. */
void oximeter_scan_cancel(void);

/* Return a cJSON array of discovered OxyII rings.
 * Each element: {"addr":"AA:BB:...","name":"SHQO2Pro ...","rssi":-65}
 * Caller must cJSON_Delete(). */
cJSON *oximeter_get_scan_results(void);

/* Pair with the ring at the given BLE address.  Connects, runs GET_INFO,
 * stores the serial in NVS + paired.json, disconnects.  Replaces any
 * previously paired ring.  Non-blocking — runs in a background task. */
esp_err_t oximeter_pair(const char *addr_str);

/* Forget the paired ring: erase NVS + paired.json.  Keeps files/. */
esp_err_t oximeter_forget(void);

/* Return the current state string (one of OX_STATUS_*). */
const char *oximeter_get_status(void);

/* Return the last error message (valid when status is "error"). */
const char *oximeter_get_error(void);

/* Return true if a ring serial is stored in NVS. */
bool oximeter_is_paired(void);

/* Coarse live presence of the paired ring, derived from the background
 * watch scans and any in-flight session:
 *   OX_PRESENCE_SYNCING  - BLE session running right now
 *   OX_PRESENCE_READY    - visible and sync window open (transfer due)
 *   OX_PRESENCE_DETECTED - visible, no transfer pending (post-sync curfew:
 *                          on hand but deliberately left alone)
 *   OX_PRESENCE_RECORDING- worn & recording: on-air via its recording-mode
 *                          advert, but never connectable mid-recording
 *   OX_PRESENCE_OFFLINE  - no adverts for several scans (asleep, worn,
 *                          out of range, off) */
#define OX_PRESENCE_SYNCING   "syncing"
#define OX_PRESENCE_READY     "ready"
#define OX_PRESENCE_DETECTED  "detected"
#define OX_PRESENCE_RECORDING "recording"
#define OX_PRESENCE_OFFLINE   "offline"
const char *oximeter_get_presence(void);

/* Return a cJSON object with paired ring info: serial, firmware,
 * name_prefix, last_addr.  NULL if not paired.  Caller cJSON_Delete(). */
cJSON *oximeter_get_paired_info(void);
