/*
 * SomnoTrace - Legacy Wellue ring (P02 / O2Ring) BLE protocol — public API
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
 */

/* Legacy Wellue ring protocol backend ("0xAA protocol").
 *
 * Clean-room implementation studied from published reverse-engineering
 * documentation (farolone/wellue-o2ring-protocol, MackeyStingray/o2r);
 * no third-party source code copied.
 *
 * Frame layout — requests and responses differ in the first byte:
 *   request : AA CMD  CMD^FF BLOCK(2 LE) LEN(2 LE) DATA[LEN] CRC8
 *   response: 55 STATUS STATUS^FF BLOCK(2 LE) LEN(2 LE) DATA[LEN] CRC8
 * CRC-8, poly 0x07 / init 0x00, over every byte except the final CRC.
 * Responses carry a STATUS code in byte 1 (0 = success), not an echo of
 * the command; requests/responses are strictly lockstep (one outstanding).
 *
 * This backend owns only protocol transport + file download.  Scanning,
 * sync scheduling, pairing persistence and SD storage stay with the
 * common oximeter layer (oximeter_oxyii.c watch task + oximeter_store.c).
 *
 * Full byte-level reference for BOTH backends (OxyII 0xA5 and legacy
 * 0xAA): see ADD_LEGACY_OXIMETER.md in the repository root.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "host/ble_hs.h"        /* ble_addr_t (NimBLE) */

#ifdef __cplusplus
extern "C" {
#endif

/* Create the backend's synchronisation primitives.  Call once from
 * oximeter_init(); sessions must not run before this. */
void legacy_init(void);

/* Session result codes for legacy_sync_session(). */
typedef enum {
    LEGACY_SYNC_OK = 0,         /* connected, INFO parsed, files handled */
    LEGACY_SYNC_ERR_CONNECT,    /* connection/GATT setup failed          */
    LEGACY_SYNC_ERR_INFO,       /* INFO missing/invalid (incl. no SN)    */
    LEGACY_SYNC_ERR_IDENTITY,   /* SN does not match expect_serial       */
    LEGACY_SYNC_ERR_TRANSFER,   /* a recording download failed           */
} legacy_sync_err_t;

/* Full sync session against one ring:
 *   connect → discover → subscribe → INFO → [download every not-yet-
 *   stored recording from FileList via ox_store] → disconnect.
 *
 * Runs synchronously; caller must hold the shared oximeter ops mutex so
 * no other BLE operation interleaves.  The NimBLE host must be ready
 * (as11_ble_is_host_ready()).
 *
 * download_files: false runs identify-only (INFO read, nothing
 *                 downloaded) — used by pairing, which must stay quick;
 *                 the background watch syncs recordings afterwards.
 * expect_serial: when non-NULL (watch-task syncs), INFO SN is compared
 *                against it and LEGACY_SYNC_ERR_IDENTITY is returned on
 *                mismatch.  NULL (pairing) accepts any device.
 * out_serial:    receives the reported SN (may be NULL).
 * out_files_pulled: receives the number of recordings downloaded during
 *                this session (may be NULL).
 */
legacy_sync_err_t legacy_sync_session(const ble_addr_t *addr,
                                      const char *expect_serial,
                                      bool download_files,
                                      char *out_serial, size_t serial_sz,
                                      int *out_files_pulled);

/* ── Advertisement classification (used by the common scan handler) ── */

/* True if the advertised local name belongs to the legacy Wellue ring
 * family that speaks this protocol (O2Ring, P02, Checkme/Sleep/Wear/Kids/
 * Baby O2 variants, Oxylink).  Case-insensitive substring match, same
 * name list as the reference discovery filter. */
bool legacy_name_match(const char *name);

/* True if the raw advertisement payload carries this protocol's 128-bit
 * GATT service UUID (AD types 0x06/0x07). */
bool legacy_adv_service_match(const uint8_t *adv_data, int adv_len);

/* Last INFO data seen from any legacy ring during a session (battery,
 * model, recording count) — for the status UI.  Returns false until the
 * first successful INFO exchange of this boot.  age_s = seconds since
 * that INFO. */
bool legacy_get_last_seen(char *model, size_t model_sz,
                          int *battery, int *files_on_ring,
                          long *age_s);

#ifdef __cplusplus
}
#endif
