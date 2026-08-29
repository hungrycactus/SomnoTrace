/*
 * SomnoTrace - Oximeter backend interface
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

/* Contract between the common oximeter layer (oximeter_common.c: scanning,
 * pairing, sync watch, sleep curfew, public API) and the
 * per-protocol protocol backends.
 *
 * To add a new oximeter family:
 *   1. write oximeter_<name>.c implementing every member of
 *      oximeter_backend_t (all state stays private to that file),
 *   2. define `const oximeter_backend_t oximeter_backend_<name>` there,
 *   3. add it to the registry at the top of oximeter_common.c,
 *   4. add its OX_PROTO_* identifier to oximeter.h.
 * Nothing else changes — classification, pairing, sync windows, storage,
 * curfew and UI all come for free.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "cJSON.h"
#include "host/ble_hs.h"        /* ble_addr_t (NimBLE) */

#ifdef __cplusplus
extern "C" {
#endif

/* Result of a full sync session (backend->sync). */
typedef enum {
    OX_SYNC_OK = 0,
    OX_SYNC_ERR_CONNECT,        /* connection / GATT setup failed          */
    OX_SYNC_ERR_INFO,           /* identity info missing or unparseable    */
    OX_SYNC_ERR_IDENTITY,       /* device is not the expected serial       */
    OX_SYNC_ERR_TRANSFER,       /* recording listing/download failed       */
    OX_SYNC_NOT_READY,          /* reachable but not sync-ready right now  */
                                /* (e.g. worn): common layer backs off and
                                   retries WITHOUT counting a failure      */
} ox_sync_err_t;

typedef struct oximeter_backend {
    /* Persisted protocol identifier — must be one of the OX_PROTO_*
     * strings from oximeter.h. */
    const char *proto_id;

    /* Called once from oximeter_init() before any other member. */
    void (*init)(void);

    /* Advertisement scoring for the shared scan handler.
     * Confidence tier — higher wins regardless of registration order:
     *   3  explicit device-name match (authoritative)
     *   2  id-based heuristic (manufacturer id etc.)
     *   1  weak signal (service UUID seen in raw payload)
     *   0  not ours
     *  -1  ours, but visible-only: never a sync candidate (e.g. OxyII
     *      worn/recording adverts).  The common layer surfaces this as
     *      the "recording" presence state while still counting it as
     *      absence for the connect-curfew model. */
    int (*adv_score)(const char *name, uint16_t mfg_cid,
                     const uint8_t *raw_adv, int raw_len);

    /* Pairing: connect once, read device identity, disconnect.
     * On success fill `serial` (required) and `firmware` ("" if the
     * protocol has no equivalent).  Returns false on failure. */
    bool (*identify)(const ble_addr_t *addr,
                     char *serial, size_t serial_sz,
                     char *firmware, size_t fw_sz);

    /* Full sync session against one device, including connect and
     * disconnect.  download=false runs an identify-only pass (no file
     * transfers).  expect_serial NULL accepts any device; otherwise SN is
     * compared and OX_SYNC_ERR_IDENTITY returned on mismatch.
     * files_pulled (optional) receives the number of recordings
     * downloaded during THIS session. */
    ox_sync_err_t (*sync)(const ble_addr_t *addr, const char *expect_serial,
                          bool download,
                          char *serial, size_t serial_sz,
                          int *files_pulled);

    /* Optional (may be NULL): add backend-specific keys to the
     * "oximeter.device" object served by /api/status (battery, model,
     * recordings-on-ring, ...). */
    void (*report_status)(cJSON *info);

    /* Consecutive failed syncs tolerated before the common layer marks
     * the window served and stops reconnecting until the device
     * disappears again.  0 disables the valve. */
    int max_consecutive_fails;
} oximeter_backend_t;

/* Registered backends — iterate via ox_backend_count()/ox_backend_at(). */
int  ox_backend_count(void);
const oximeter_backend_t *ox_backend_at(int index);

extern const oximeter_backend_t oximeter_backend_oxyii;
extern const oximeter_backend_t oximeter_backend_legacy;

#ifdef __cplusplus
}
#endif