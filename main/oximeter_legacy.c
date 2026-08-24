/*
<<<<<<< HEAD
 * SomnoTrace - O2 Ring (Legacy/Gen1) BLE protocol codec and session
=======
 * SomnoTrace - Legacy Wellue ring (P02 / O2Ring) BLE protocol codec and session
>>>>>>> 1faf953 (Add support for legacy Wellue O2 oximeter ring; refactored oximeter backend)
 * Copyright (C) 2026 Ilya Kruchinin <https://github.com/ilyakruchinin>
 *
 * This file is part of SomnoTrace.
 *
 * SomnoTrace is free software: you can redistribute it and/or modify it under
<<<<<<< HEAD
 * the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
=======
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
>>>>>>> 1faf953 (Add support for legacy Wellue O2 oximeter ring; refactored oximeter backend)
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
<<<<<<< HEAD
 *
 * Clean-room Legacy BLE protocol for Wellue O2 Ring (Gen1) / ViaTom rings.
 * Protocol studied from published documentation; no third-party source
 * code copied.
 */

#include "oximeter.h"
#include "oximeter_internal.h"
#include "sd_storage.h"
#include "as11_ble.h"
#include "psram_task.h"
#include "nvs_writer.h"
#include "oximetry_canonical.h"
#include "time_sync.h"
#include "upload_sched.h"
#include "log_stream.h"
=======
 */

/* Legacy Wellue ring protocol ("0xAA protocol").
 *
 * Protocol facts are taken from published reverse-engineering docs
 * (farolone/wellue-o2ring-protocol + MackeyStingray/o2r research):
 *   - Requests begin with 0xAA; responses begin with 0x55 and carry a
 *     STATUS code in byte 1 (0 = OK), not a command echo.
 *   - BLOCK and LEN are 2-byte little-endian; total frame = LEN + 8.
 *   - CRC-8 poly 0x07 / init 0x00 over all bytes except the final CRC
 *     byte (verified equivalent to the reference's bit-table routine).
 *   - FILE_OPEN payload is the filename including the terminating NUL;
 *     response DATA[0..3] is the 32-bit LE file size.
 *   - FILE_READ takes the block number in the request BLOCK field,
 *     blocks counting from 0; DATA carries the chunk.
 *   - INFO (0x14) returns an ASCII JSON object; FileList is a
 *     comma-separated list of native recording names.
 *   - Writes must be chunked to 20-byte ATT payloads; notifications can
 *     fragment/multiplex frames, so RX reassembles before parsing.
 *
 * Documented but intentionally NOT implemented here:
 * READ_SENSORS (0x17) live streaming — its response layout is not
 * needed for stored-recording sync and is left undefined rather than
 * guessed.  VLD contents are likewise stored byte-for-byte, never
 * decoded here.
 *
 * Byte-exact frame tables, worked hex examples and the OxyII/legacy
 * comparison live in ADD_LEGACY_OXIMETER.md (repository root).
 *
 * Concurrency: every entry point runs under the shared oximeter ops
 * mutex held by the caller (watch/pair tasks in oximeter_oxyii.c).
 */

#include "oximeter.h"
#include "oximeter_backend.h"
#include "oximeter_store.h"
#include "as11_ble.h"
>>>>>>> 1faf953 (Add support for legacy Wellue O2 oximeter ring; refactored oximeter backend)

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
<<<<<<< HEAD
#include <time.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
=======
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "esp_log.h"
#include "esp_err.h"
>>>>>>> 1faf953 (Add support for legacy Wellue O2 oximeter ring; refactored oximeter backend)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

<<<<<<< HEAD
static const char *TAG = "ox_legacy";

/* ── Store forward declarations (oximeter_store.c) ─────────────────── */
void ox_store_ensure_dirs(void);
bool ox_store_load_paired(char *serial, size_t serial_sz,
                          char *firmware, size_t fw_sz,
                          char *name_prefix, size_t prefix_sz,
                          char *last_addr, size_t addr_sz,
                          char *driver, size_t driver_sz,
                          char *ble_name, size_t ble_name_sz);
void ox_store_save_paired(const char *serial, const char *firmware,
                          const char *name_prefix, const char *last_addr,
                          const char *driver, const char *ble_name);
void ox_store_delete_paired(void);
int  ox_store_index_check(const char *serial, const char *name);
int  ox_store_index_conversion_check(const char *serial, const char *name);
void ox_store_index_add(const char *serial, const char *name,
                        uint32_t bytes, bool finalised);
void ox_store_index_mark_converted(const char *serial, const char *name,
                                   bool converted, const char *error);
long ox_store_part_size(const char *name);
esp_err_t ox_store_part_append(const char *name, const uint8_t *data, size_t len);
bool ox_store_promote_vld3(const char *serial, const char *name);
bool ox_store_finalize_native(const char *serial, const char *name,
                              long declared_size);
void ox_store_part_remove(const char *name);

/* ── Legacy protocol constants ──────────────────────────────────────── */
#define LEGACY_REQ_LEAD     0xAA
#define LEGACY_RSP_LEAD     0x55
#define LEGACY_HEADER_LEN   7   /* sync | cmd | cmd^0xFF | block(2) | len(2) */
#define LEGACY_MAX_FRAME    2048
#define LEGACY_BLE_CHUNK    20  /* max bytes per BLE write */

/* Command codes */
#define CMD_FILE_OPEN       0x03
#define CMD_FILE_READ       0x04
#define CMD_FILE_CLOSE      0x05
#define CMD_INFO            0x14
#define CMD_PING            0x15
#define CMD_CONFIG          0x16

/* VLD3 file format constants */
#define VLD3_HEADER_LEN     40
#define VLD3_RECORD_LEN     5
#define VLD3_NO_FINGER      0xFF

/* Legacy GATT UUIDs (128-bit, stored little-endian for NimBLE).
 * Service:  14839ac4-7d7e-415c-9a42-167340cf2339
 * Write:    8b00ace7-eb0b-49b0-bbe9-9aee0a26e1a3
 * Notify:   0734594a-a8e7-4b1a-a6b1-cd5243059a57 */
=======
static const char *TAG = "legacy";

/* ── Protocol constants ────────────────────────────────────────────── */
#define LEGACY_REQ_LEAD       0xAA
#define LEGACY_RSP_LEAD       0x55
#define LEGACY_HDR_LEN        7          /* AA/CMD/^CMD/BLOCK(2)/LEN(2) */
#define LEGACY_MAX_FRAME      1024
#define LEGACY_MAX_PAYLOAD    (LEGACY_MAX_FRAME - LEGACY_HDR_LEN - 1)
#define LEGACY_WRITE_CHUNK    20         /* legacy ATT payload limit */

#define LEGACY_CMD_FILE_OPEN  0x03
#define LEGACY_CMD_FILE_READ  0x04
#define LEGACY_CMD_FILE_CLOSE 0x05
#define LEGACY_CMD_INFO       0x14
/* 0x15 PING / 0x16 CONFIG / 0x18 FACTORY_DEFAULT unused.
 * 0x17 READ_SENSORS deliberately deferred — see file comment. */

#define LEGACY_STATUS_OK      0x00

/* Legacy GATT UUIDs (128-bit, NimBLE wants them little-endian).
 * Service 14839ac4-7d7e-415c-9a42-167340cf2339,
 * Notify  0734594a-a8e7-4b1a-a6b1-cd5243059a57,
 * Write   8b00ace7-eb0b-49b0-bbe9-9aee0a26e1a3. */
>>>>>>> 1faf953 (Add support for legacy Wellue O2 oximeter ring; refactored oximeter backend)
static const ble_uuid128_t LEGACY_SVC_UUID =
    BLE_UUID128_INIT(0x39, 0x23, 0xcf, 0x40, 0x73, 0x16, 0x42, 0x9a,
                     0x5c, 0x41, 0x7e, 0x7d, 0xc4, 0x9a, 0x83, 0x14);
static const ble_uuid128_t LEGACY_WRITE_UUID =
    BLE_UUID128_INIT(0xa3, 0xe1, 0x26, 0x0a, 0xee, 0x9a, 0xe9, 0xbb,
                     0xb0, 0x49, 0x0b, 0xeb, 0xe7, 0xac, 0x00, 0x8b);
static const ble_uuid128_t LEGACY_NOTIFY_UUID =
    BLE_UUID128_INIT(0x57, 0x9a, 0x05, 0x43, 0x52, 0xcd, 0xb1, 0xa6,
                     0x1a, 0x4b, 0xe7, 0xa8, 0x4a, 0x59, 0x34, 0x07);

<<<<<<< HEAD
/* ── CRC8 (poly=0x07, init=0) — same as OxyII ──────────────────────── */
=======
/* On-air (little-endian) service UUID for advertisement matching. */
static const uint8_t LEGACY_SVC_ADV_LE[16] = {
    0x39, 0x23, 0xcf, 0x40, 0x73, 0x16, 0x42, 0x9a,
    0x5c, 0x41, 0x7e, 0x7d, 0xc4, 0x9a, 0x83, 0x14,
};

/* Name fragments used by the reference discovery filter for this
 * protocol family.  Matched case-insensitively as substrings. */
static const char *const LEGACY_NAME_KEYS[] = {
    "O2RING", "CHECKME_O2", "CHECKO2", "SLEEPU", "SLEEPO2",
    "WEARO2", "KIDSO2", "BABYO2", "OXYLINK",
};

/* ── Transfer limits / timeouts ────────────────────────────────────── */
#define LEGACY_MAX_FILES          16
#define LEGACY_NAME_MAX           31     /* + NUL fits a 32-byte buffer   */
#define LEGACY_MAX_FILE_SIZE      (8u << 20)   /* reject absurd sizes      */
#define LEGACY_INFO_JSON_MAX      1024

#define LEGACY_T_CONNECT_MS       15000
#define LEGACY_T_GATT_MS          10000
#define LEGACY_T_WRITE_MS         5000
#define LEGACY_T_INFO_MS          8000
#define LEGACY_T_OPEN_MS          5000
#define LEGACY_T_READ_MS          10000
#define LEGACY_T_CLOSE_MS         3000
#define LEGACY_BLOCK_RETRIES      3      /* per-block attempts before abort */

/* ── Module state (single session at a time, ops mutex serialises) ─── */
static SemaphoreHandle_t s_op_sem;      /* GATT op completion            */
static SemaphoreHandle_t s_conn_sem;    /* connect/disconnect completion */
static SemaphoreHandle_t s_resp_sem;    /* complete response frame ready */

static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_write_handle;
static uint16_t s_notify_handle;
static uint16_t s_cccd_handle;
static uint16_t s_svc_start, s_svc_end;
static volatile int s_op_status;
static volatile int s_conn_status;

/* Notification accumulator + last decoded response frame. */
static uint8_t s_acc[LEGACY_MAX_FRAME];
static int s_acc_len;
static volatile bool s_have_rsp;
static volatile uint8_t s_rsp_status;
static volatile uint16_t s_rsp_block;
static int s_rsp_len;
static uint8_t s_rsp_payload[LEGACY_MAX_PAYLOAD];

/* Last INFO data seen from any ring (surfaced in the status UI). */
static struct {
    bool valid;
    char model[24];
    int battery;                /* percent, -1 unknown */
    int nfiles;                 /* recordings on ring  */
    time_t seen;                /* epoch of that INFO  */
} s_last_seen;

/* Create the backend's synchronisation primitives.  Call once at boot
 * (oximeter_init); sessions must not run before this. */
static void legacy_init(void)
{
    if (s_op_sem) return;                       /* already initialised */
    s_op_sem   = xSemaphoreCreateBinary();
    s_conn_sem = xSemaphoreCreateBinary();
    s_resp_sem = xSemaphoreCreateBinary();
    if (!s_op_sem || !s_conn_sem || !s_resp_sem) {
        ESP_LOGE(TAG, "semaphore creation failed");
        return;
    }
}

/* ── CRC-8 (poly 0x07, init 0x00, no reflection) ─────────────────────
 * MSB-first form of the reference's per-bit XOR routine; covers every
 * byte of the frame except the trailing CRC itself. */
>>>>>>> 1faf953 (Add support for legacy Wellue O2 oximeter ring; refactored oximeter backend)
static uint8_t legacy_crc8(const uint8_t *data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else            crc <<= 1;
        }
    }
    return crc;
}

/* ── Frame codec ───────────────────────────────────────────────────── */
<<<<<<< HEAD
/* Encode a Legacy request frame into buf.  Returns total frame length.
 * Request: 0xAA | cmd | cmd^0xFF | block(LE16) | len(LE16) | data | crc8 */
static int legacy_encode(uint8_t *buf, int bufsz, uint8_t cmd,
                         uint16_t block,
                         const uint8_t *payload, int payload_len)
{
    int total = LEGACY_HEADER_LEN + payload_len + 1;
=======
/* Build AA CMD ^CMD BLOCK LEN DATA CRC.  Returns total length, -1 if
 * it does not fit. */
static int legacy_encode(uint8_t *buf, int bufsz, uint8_t cmd, uint16_t block,
                      const void *payload, int plen)
{
    if (plen < 0 || plen > LEGACY_MAX_PAYLOAD) return -1;
    int total = LEGACY_HDR_LEN + plen + 1;
>>>>>>> 1faf953 (Add support for legacy Wellue O2 oximeter ring; refactored oximeter backend)
    if (total > bufsz) return -1;

    buf[0] = LEGACY_REQ_LEAD;
    buf[1] = cmd;
    buf[2] = cmd ^ 0xFF;
    buf[3] = block & 0xFF;
    buf[4] = (block >> 8) & 0xFF;
<<<<<<< HEAD
    buf[5] = payload_len & 0xFF;
    buf[6] = (payload_len >> 8) & 0xFF;
    if (payload && payload_len > 0)
        memcpy(buf + 7, payload, payload_len);
=======
    buf[5] = plen & 0xFF;
    buf[6] = (plen >> 8) & 0xFF;
    if (plen > 0 && payload)
        memcpy(buf + LEGACY_HDR_LEN, payload, plen);
>>>>>>> 1faf953 (Add support for legacy Wellue O2 oximeter ring; refactored oximeter backend)
    buf[total - 1] = legacy_crc8(buf, total - 1);
    return total;
}

<<<<<<< HEAD
/* Try to decode a Legacy response frame from buf.
 * Response: 0x55 | status | status^0xFF | block(LE16) | len(LE16) | data | crc8
 * Returns total frame length on success, -1 if incomplete, -2 if invalid. */
static int legacy_try_decode(const uint8_t *buf, int len,
                              uint8_t *status, uint16_t *block,
                              uint8_t *payload, int *payload_len,
                              int payload_cap)
{
    if (len < LEGACY_HEADER_LEN) return -1;
    if (buf[0] != LEGACY_RSP_LEAD) return -2;
    if ((uint8_t)(buf[1] ^ 0xFF) != buf[2]) return -2;

    int plen = buf[5] | (buf[6] << 8);
    int total = LEGACY_HEADER_LEN + plen + 1;
    if (len < total) return -1;

    if (legacy_crc8(buf, total - 1) != buf[total - 1]) return -2;

    if (status) *status = buf[1];
    if (block)  *block = buf[3] | (buf[4] << 8);
    if (payload && payload_cap > 0) {
        int n = plen < payload_cap ? plen : payload_cap;
        memcpy(payload, buf + 7, n);
    }
    if (payload_len) *payload_len = plen;
    return total;
}

/* ── Module state ──────────────────────────────────────────────────── */
#define OX_SCAN_MAX 16

struct ox_scan_result {
    ble_addr_t addr;
    char name[32];
    int rssi;
};

/* Argument passed to pair_task (must be freed by the task). */
struct pair_arg {
    char addr_str[24];
    char ble_name[40];
};

static SemaphoreHandle_t s_state_mtx;
static SemaphoreHandle_t s_ops_mtx;
static SemaphoreHandle_t s_op_sem;
static SemaphoreHandle_t s_conn_sem;
static SemaphoreHandle_t s_resp_sem;
static SemaphoreHandle_t s_scan_done;
static volatile int s_op_status;
static volatile int s_conn_status;
static bool s_initialized;

static char s_status[24] = OX_STATUS_IDLE;
static char s_error[128];

static char s_serial[32];
static char s_firmware[16];
static char s_name_prefix[16];
static char s_ble_name[40];      /* BLE advertised name or constructed display name */
static char s_paired_addr[18];
static bool s_paired = false;
static bool s_presence_served = false;
static bool s_ring_present = false;
static TickType_t s_served_at;
static int s_pull_fail_count = 0;   /* consecutive failed pulls in this presence */
#define OX_PULL_MAX_FAST_RETRIES 3  /* quick retries before applying curfew */

/* BLE connection state */
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_write_handle;
static uint16_t s_notify_handle;
static uint16_t s_cccd_handle;
static uint16_t s_svc_start, s_svc_end;

/* Scan state */
static struct ox_scan_result *s_scan;
static int s_scan_count;

/* Notification accumulation buffer — PSRAM-allocated at init */
static uint8_t *s_resp_buf;
static int s_resp_len;
static uint8_t s_resp_status;
static uint16_t s_resp_block;
static uint8_t *s_resp_payload;
static int s_resp_payload_len;

/* ── Helpers ───────────────────────────────────────────────────────── */
static void set_state(const char *st)
{
    xSemaphoreTake(s_state_mtx, portMAX_DELAY);
    strlcpy(s_status, st, sizeof(s_status));
    xSemaphoreGive(s_state_mtx);
    log_stream_request_ox_push();
}

static void set_error(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    xSemaphoreTake(s_state_mtx, portMAX_DELAY);
    vsnprintf(s_error, sizeof(s_error), fmt, ap);
    strlcpy(s_status, OX_STATUS_ERROR, sizeof(s_status));
    xSemaphoreGive(s_state_mtx);
    va_end(ap);
}

=======
/* Try to decode one response frame (55 STATUS ^STATUS BLOCK LEN DATA CRC).
 * Returns bytes consumed (>0), 0 if incomplete, -1 if invalid. */
static int legacy_try_decode(const uint8_t *buf, int len,
                          uint8_t *status, uint16_t *block,
                          uint8_t *payload, int cap, int *payload_len)
{
    if (len < LEGACY_HDR_LEN + 1) return 0;      /* shortest possible frame */
    if (buf[0] != LEGACY_RSP_LEAD) return -1;
    if ((uint8_t)(buf[1] ^ 0xFF) != buf[2]) return -1;

    int plen = buf[5] | (buf[6] << 8);
    int total = LEGACY_HDR_LEN + plen + 1;
    if (total > LEGACY_MAX_FRAME) return -1;
    if (len < total) return 0;
    if (legacy_crc8(buf, total - 1) != buf[total - 1]) return -1;

    if (status) *status = buf[1];
    if (block)  *block = (uint16_t)(buf[3] | (buf[4] << 8));
    if (payload_len) *payload_len = plen;
    if (payload && cap > 0 && plen > 0) {
        int n = plen < cap ? plen : cap;
        memcpy(payload, buf + LEGACY_HDR_LEN, n);
    }
    return total;
}

/* ── Notification reassembly ───────────────────────────────────────── */
/* Fragments accumulate until one or more whole frames are available;
 * malformed frames drop single bytes until the stream resynchronises. */
static void handle_notify_rx(const uint8_t *data, int len)
{
    if (s_acc_len + len > (int)sizeof(s_acc)) {
        ESP_LOGW(TAG, "notify overflow (%d+%d B)", s_acc_len, len);
        s_acc_len = 0;
        return;
    }
    memcpy(s_acc + s_acc_len, data, len);
    s_acc_len += len;

    while (true) {
        uint8_t st;
        uint16_t bl;
        int pl;
        int rc = legacy_try_decode(s_acc, s_acc_len, &st, &bl,
                                s_rsp_payload, sizeof(s_rsp_payload), &pl);
        if (rc > 0) {
            memmove(s_acc, s_acc + rc, s_acc_len - rc);
            s_acc_len -= rc;
            s_rsp_status = st;
            s_rsp_block = bl;
            s_rsp_len = pl;
            ESP_LOGD(TAG, "rx frame status=%u block=%u len=%d", st, bl, pl);
            s_have_rsp = true;
            xSemaphoreGive(s_resp_sem);
        } else if (rc < 0) {
            ESP_LOGW(TAG, "bad frame (lead=0x%02x complement/crc) — dropping byte",
                     s_acc[0]);
            memmove(s_acc, s_acc + 1, (size_t)(s_acc_len - 1));
            s_acc_len--;
            if (s_acc_len <= 0) break;
        } else {
            break;                         /* need more fragments */
        }
    }
}

/* ── Helpers ───────────────────────────────────────────────────────── */
>>>>>>> 1faf953 (Add support for legacy Wellue O2 oximeter ring; refactored oximeter backend)
static void clear_op_sem(void)
{
    while (xSemaphoreTake(s_op_sem, 0) == pdTRUE) { }
}

static int wait_op(int timeout_ms)
{
    if (xSemaphoreTake(s_op_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
        return BLE_HS_ETIMEOUT;
    return s_op_status;
}

<<<<<<< HEAD
/* Check if raw BLE advertisement data contains the Gen1 Legacy service UUID
 * 14839ac4-7d7e-415c-9a42-167340cf2339.
 * In BLE AD data, 128-bit service UUIDs are stored as 16-byte LE sequences
 * under AD type 0x06 (incomplete) or 0x07 (complete). */
static const uint8_t LEGACY_SVC_UUID_LE[16] = {
    0xc4, 0x9a, 0x83, 0x14, 0x7e, 0x7d, 0x5c, 0x41,
    0x42, 0x9a, 0x73, 0x16, 0xcf, 0x40, 0x23, 0x39
};

static bool adv_has_legacy_service_uuid(const uint8_t *raw, int raw_len)
{
    for (int off = 0; off + 1 < raw_len; ) {
        uint8_t ad_len = raw[off];
        if (ad_len == 0 || off + 1 + ad_len > raw_len) break;
        uint8_t ad_type = raw[off + 1];
        const uint8_t *ad_data = raw + off + 2;
        int ad_data_len = ad_len - 1;

        if ((ad_type == 0x06 || ad_type == 0x07) && ad_data_len >= 16) {
            for (int i = 0; i + 16 <= ad_data_len; i += 16) {
                if (memcmp(ad_data + i, LEGACY_SVC_UUID_LE, 16) == 0)
                    return true;
            }
        }
        off += 1 + ad_len;
    }
    return false;
}

static void addr_to_str(const ble_addr_t *a, char *out, size_t outsz)
{
    snprintf(out, outsz, "%02x:%02x:%02x:%02x:%02x:%02x",
             a->val[5], a->val[4], a->val[3], a->val[2], a->val[1], a->val[0]);
}

static bool parse_addr(const char *str, ble_addr_t *out)
{
    unsigned int v[6];
    int n = sscanf(str, "%x:%x:%x:%x:%x:%x",
                   &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]);
    if (n != 6) return false;
    out->val[5] = v[0]; out->val[4] = v[1]; out->val[3] = v[2];
    out->val[2] = v[3]; out->val[1] = v[4]; out->val[0] = v[5];
    out->type = (v[0] & 0xC0) == 0xC0 ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
    return true;
}

/* Find the scan result whose address matches s_paired_addr.
 * Returns the index, or -1 if not found. */
static int find_paired_in_scan(void)
{
    for (int i = 0; i < s_scan_count; i++) {
        char addr_str[18];
        addr_to_str(&s_scan[i].addr, addr_str, sizeof(addr_str));
        if (strcmp(addr_str, s_paired_addr) == 0)
            return i;
    }
    return -1;
}

/* ── GAP event handler ─────────────────────────────────────────────── */
static int gap_event(struct ble_gap_event *event, void *arg);

static void handle_notify_rx(const uint8_t *data, int len)
{
    if (s_resp_len + len > LEGACY_MAX_FRAME) {
        ESP_LOGW(TAG, "notify overflow: resp_len=%d + %d > %d",
                 s_resp_len, len, LEGACY_MAX_FRAME);
        s_resp_len = 0;
    }
    memcpy(s_resp_buf + s_resp_len, data, len);
    s_resp_len += len;

    /* Try to decode as many complete frames as are in the buffer. */
    while (s_resp_len > 0) {
        uint8_t status;
        uint16_t block;
        int plen;
        int rc = legacy_try_decode(s_resp_buf, s_resp_len, &status, &block,
                                   s_resp_payload, &plen, LEGACY_MAX_FRAME);
        if (rc > 0) {
            s_resp_status = status;
            s_resp_block = block;
            s_resp_payload_len = plen;
            /* Shift leftover bytes to front */
            int remaining = s_resp_len - rc;
            if (remaining > 0)
                memmove(s_resp_buf, s_resp_buf + rc, remaining);
            s_resp_len = remaining;
            xSemaphoreGive(s_resp_sem);
            return;
        }
        if (rc == -1) {
            /* Incomplete frame — wait for more data */
            return;
        }
        /* rc == -2: invalid frame — drop one byte and try to resync
         * on the next 0x55 lead byte */
        ESP_LOGW(TAG, "notify decode error at offset 0, resyncing");
        int remaining = s_resp_len - 1;
        if (remaining > 0)
            memmove(s_resp_buf, s_resp_buf + 1, remaining);
        s_resp_len = remaining;
    }
}

=======
/* Reject anything that could escape the storage directory or overflow
 * buffers: printable, no separators, bounded length. */
static bool name_is_safe(const char *n, int len)
{
    if (len <= 0 || len > LEGACY_NAME_MAX) return false;
    if (n[0] == '.') return false;
    for (int i = 0; i < len; i++) {
        char c = n[i];
        if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') ||
              (c >= 'a' && c <= 'z') || c == '.' || c == '_' || c == '-'))
            return false;
    }
    return true;
}

static void fmt_addr(const ble_addr_t *a, char *out, size_t outsz)
{
    snprintf(out, outsz, "%02x:%02x:%02x:%02x:%02x:%02x",
             a->val[5], a->val[4], a->val[3],
             a->val[2], a->val[1], a->val[0]);
}

static const char *sync_err_str(ox_sync_err_t r)
{
    switch (r) {
    case OX_SYNC_OK:         return "ok";
    case OX_SYNC_ERR_CONNECT:return "connect-failed";
    case OX_SYNC_ERR_INFO:   return "info-failed";
    case OX_SYNC_ERR_IDENTITY: return "serial-mismatch";
    case OX_SYNC_ERR_TRANSFER: return "transfer-failed";
    case OX_SYNC_NOT_READY:    return "not-ready";
    }
    return "unknown";
}

/* One-line hex dump of up to cap leading bytes (DEBUG level). */
static void log_hex_prefix(const char *what, const uint8_t *buf, int len,
                           int cap)
{
    char line[3 * 16 + 1];
    int n = len < cap ? len : cap;
    int off = 0;
    for (int i = 0; i < n; i++)
        off += snprintf(line + off, sizeof(line) - off, "%02x ", buf[i]);
    if (n < len) off += snprintf(line + off, sizeof(line) - off, "...");
    ESP_LOGD(TAG, "%s[%d]: %s", what, len, line);
}

/* ── GAP event handler (this backend's connections only) ───────────── */
>>>>>>> 1faf953 (Add support for legacy Wellue O2 oximeter ring; refactored oximeter backend)
static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
<<<<<<< HEAD
    case BLE_GAP_EVENT_DISC: {
        char addr_str[18];
        addr_to_str(&event->disc.addr, addr_str, sizeof(addr_str));

        struct ble_hs_adv_fields f;
        char name[32] = {0};
        const uint8_t *raw = event->disc.data;
        int raw_len = event->disc.length_data;

        memset(&f, 0, sizeof(f));
        if (ble_hs_adv_parse_fields(&f, raw, raw_len) != 0) {
            for (int off = 0; off + 1 < raw_len; ) {
                uint8_t ad_len = raw[off];
                if (ad_len == 0 || off + 1 + ad_len > raw_len) break;
                uint8_t ad_type = raw[off + 1];
                const uint8_t *ad_data = raw + off + 2;
                int ad_data_len = ad_len - 1;
                if (ad_type == 0x09 || ad_type == 0x08) {
                    int nl = ad_data_len < 31 ? ad_data_len : 31;
                    memcpy(name, ad_data, nl);
                    name[nl] = '\0';
                }
                off += 1 + ad_len;
            }
        } else {
            if (f.name && f.name_len > 0) {
                int nl = f.name_len < 31 ? f.name_len : 31;
                memcpy(name, f.name, nl);
                name[nl] = '\0';
            }
        }

        /* Match Gen1 rings purely by the legacy service UUID in advertisement
         * data (14839ac4-...).  This is more reliable than name matching:
         * ecostech/viatom-ble confirms "some rings drop the name from ads
         * after being connected once, but the service UUID stays."  It also
         * avoids false positives: Gen2 names like "SHQO2Pro" contain "O2"
         * and would match the old name-based fallback. */
        bool is_legacy = adv_has_legacy_service_uuid(raw, raw_len);
        if (!is_legacy) return 0;
        if (name[0] == '\0')
            strlcpy(name, "O2Ring", sizeof(name));

        /* Dedupe by address */
        for (int i = 0; i < s_scan_count; i++) {
            if (memcmp(&s_scan[i].addr, &event->disc.addr,
                       sizeof(ble_addr_t)) == 0) {
                s_scan[i].rssi = event->disc.rssi;
                if (name[0])
                    strlcpy(s_scan[i].name, name, sizeof(s_scan[i].name));
                return 0;
            }
        }
        if (s_scan_count < OX_SCAN_MAX) {
            s_scan[s_scan_count].addr = event->disc.addr;
            strlcpy(s_scan[s_scan_count].name, name,
                    sizeof(s_scan[s_scan_count].name));
            s_scan[s_scan_count].rssi = event->disc.rssi;
            s_scan_count++;
            ESP_LOGD(TAG, "scan: '%s' rssi=%d addr=%s",
                     name, event->disc.rssi, addr_str);
        }
        return 0;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
        xSemaphoreGive(s_scan_done);
        return 0;

=======
>>>>>>> 1faf953 (Add support for legacy Wellue O2 oximeter ring; refactored oximeter backend)
    case BLE_GAP_EVENT_CONNECT:
        s_conn_handle = event->connect.conn_handle;
        s_conn_status = event->connect.status;
        xSemaphoreGive(s_conn_sem);
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected (reason=%d)",
                 event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
<<<<<<< HEAD
        xSemaphoreGive(s_resp_sem);
        xSemaphoreGive(s_conn_sem);
=======
        /* Unblock any pending connect/request waits. */
        xSemaphoreGive(s_conn_sem);
        xSemaphoreGive(s_resp_sem);
>>>>>>> 1faf953 (Add support for legacy Wellue O2 oximeter ring; refactored oximeter backend)
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        if (event->notify_rx.conn_handle != s_conn_handle)
            return 0;
        int len = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (len <= 0) return 0;
        uint8_t *data = malloc(len);
        if (!data) return 0;
        os_mbuf_copydata(event->notify_rx.om, 0, len, data);
        handle_notify_rx(data, len);
        free(data);
        return 0;
    }
<<<<<<< HEAD
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU: %d", event->mtu.value);
        return 0;
=======

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU negotiated: %d", event->mtu.value);
        return 0;

>>>>>>> 1faf953 (Add support for legacy Wellue O2 oximeter ring; refactored oximeter backend)
    default:
        return 0;
    }
}

/* ── GATT discovery callbacks ──────────────────────────────────────── */
<<<<<<< HEAD
=======
static int on_mtu(uint16_t conn, const struct ble_gatt_error *err,
                  uint16_t mtu, void *arg)
{
    (void)conn; (void)arg; (void)mtu;
    s_op_status = err ? err->status : 0;
    xSemaphoreGive(s_op_sem);
    return 0;
}

>>>>>>> 1faf953 (Add support for legacy Wellue O2 oximeter ring; refactored oximeter backend)
static int on_disc_svc(uint16_t conn, const struct ble_gatt_error *err,
                       const struct ble_gatt_svc *svc, void *arg)
{
    (void)conn; (void)arg;
    if (err && err->status == 0 && svc) {
        s_svc_start = svc->start_handle;
        s_svc_end = svc->end_handle;
    }
    if (err && (err->status == BLE_HS_EDONE || err->status != 0)) {
        s_op_status = (err->status == BLE_HS_EDONE) ? 0 : err->status;
        xSemaphoreGive(s_op_sem);
    }
    return 0;
}

static int on_disc_chr(uint16_t conn, const struct ble_gatt_error *err,
                       const struct ble_gatt_chr *chr, void *arg)
{
    (void)conn; (void)arg;
    if (err && err->status == 0 && chr) {
<<<<<<< HEAD
=======
        if (chr->uuid.u.type == BLE_UUID_TYPE_128) {
            char u[37];
            const uint8_t *v = chr->uuid.u128.value;   /* little-endian */
            snprintf(u, sizeof(u),
                     "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-"
                     "%02x%02x%02x%02x%02x%02x",
                     v[15], v[14], v[13], v[12], v[11], v[10], v[9], v[8],
                     v[7], v[6], v[5], v[4], v[3], v[2], v[1], v[0]);
            ESP_LOGD(TAG, "char: uuid=%s handle=%d props=0x%02x",
                     u, chr->val_handle, chr->properties);
        }
>>>>>>> 1faf953 (Add support for legacy Wellue O2 oximeter ring; refactored oximeter backend)
        if (ble_uuid_cmp(&chr->uuid.u, &LEGACY_WRITE_UUID.u) == 0)
            s_write_handle = chr->val_handle;
        else if (ble_uuid_cmp(&chr->uuid.u, &LEGACY_NOTIFY_UUID.u) == 0)
            s_notify_handle = chr->val_handle;
    }
    if (err && (err->status == BLE_HS_EDONE || err->status != 0)) {
        s_op_status = (err->status == BLE_HS_EDONE) ? 0 : err->status;
        xSemaphoreGive(s_op_sem);
    }
    return 0;
}

static int on_disc_dsc(uint16_t conn, const struct ble_gatt_error *err,
                       uint16_t chr_val_handle, const struct ble_gatt_dsc *dsc,
                       void *arg)
{
    (void)conn; (void)chr_val_handle; (void)arg;
    if (err && err->status == 0 && dsc) {
        const ble_uuid16_t cccd = BLE_UUID16_INIT(0x2902);
        if (ble_uuid_cmp(&dsc->uuid.u, &cccd.u) == 0 && s_cccd_handle == 0)
            s_cccd_handle = dsc->handle;
    }
    if (err && (err->status == BLE_HS_EDONE || err->status != 0)) {
        s_op_status = (err->status == BLE_HS_EDONE) ? 0 : err->status;
        xSemaphoreGive(s_op_sem);
    }
    return 0;
}

static int on_write_done(uint16_t conn, const struct ble_gatt_error *err,
                         struct ble_gatt_attr *attr, void *arg)
{
    (void)conn; (void)attr; (void)arg;
    s_op_status = err ? err->status : 0;
    xSemaphoreGive(s_op_sem);
    return 0;
}

<<<<<<< HEAD
/* ── Connect and discover GATT services ────────────────────────────── */
static esp_err_t do_connect_and_discover(ble_addr_t *target)
{
    s_write_handle = s_notify_handle = s_cccd_handle = 0;
    s_svc_start = s_svc_end = 0;
    s_resp_len = 0;

    /* Drain any stale s_conn_sem token left by a previous remote
     * disconnect.  Without this, xSemaphoreTake below returns
     * immediately on the stale token and we proceed with a dead
     * handle (BLE_HS_CONN_HANDLE_NONE), causing every subsequent
     * GATT operation to fail with "Legacy service not found". */
    while (xSemaphoreTake(s_conn_sem, 0) == pdTRUE) { }
    s_conn_status = -1;
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
=======
/* ── Connect, discover this protocol's characteristics, subscribe ──── */
static esp_err_t connect_and_discover(const ble_addr_t *target)
{
    s_write_handle = s_notify_handle = s_cccd_handle = 0;
    s_svc_start = s_svc_end = 0;
    s_acc_len = 0;
    s_have_rsp = false;

    char taddr[18];
    fmt_addr(target, taddr, sizeof(taddr));
>>>>>>> 1faf953 (Add support for legacy Wellue O2 oximeter ring; refactored oximeter backend)

    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(target->type, &own_addr_type);
    if (rc != 0) {
<<<<<<< HEAD
        set_error("addr infer failed: %d", rc);
        return ESP_FAIL;
    }
    clear_op_sem();
    rc = ble_gap_connect(own_addr_type, target,
                         15000, NULL, gap_event, NULL);
    if (rc != 0) { set_error("connect start failed: %d", rc); return ESP_FAIL; }
    if (xSemaphoreTake(s_conn_sem, pdMS_TO_TICKS(16000)) != pdTRUE) {
        set_error("connect timeout"); return ESP_FAIL;
    }
    if (s_conn_status != 0) {
        set_error("connect failed: %d", s_conn_status); return ESP_FAIL;
    }
    ESP_LOGI(TAG, "connected, handle=%d", s_conn_handle);

    /* No MTU exchange needed for Legacy (20-byte writes are default). */

    /* Discover Legacy service by UUID */
    clear_op_sem();
    rc = ble_gattc_disc_svc_by_uuid(s_conn_handle, &LEGACY_SVC_UUID.u,
                                     on_disc_svc, NULL);
    if (rc != 0 || wait_op(10000) != 0) {
        set_error("Legacy service not found"); return ESP_FAIL;
    }
    if (s_svc_start == 0) { set_error("service range empty"); return ESP_FAIL; }
    ESP_LOGI(TAG, "service: 0x%04x-0x%04x", s_svc_start, s_svc_end);

    /* Discover characteristics */
    clear_op_sem();
    rc = ble_gattc_disc_all_chrs(s_conn_handle, s_svc_start, s_svc_end,
                                 on_disc_chr, NULL);
    if (rc != 0 || wait_op(10000) != 0) {
        set_error("characteristic discovery failed"); return ESP_FAIL;
    }
    if (s_write_handle == 0 || s_notify_handle == 0) {
        set_error("write/notify char not found"); return ESP_FAIL;
    }
    ESP_LOGI(TAG, "write=%d notify=%d", s_write_handle, s_notify_handle);

    /* Discover CCCD for notify characteristic */
    clear_op_sem();
    rc = ble_gattc_disc_all_dscs(s_conn_handle, s_notify_handle, s_svc_end,
                                 on_disc_dsc, NULL);
    if (rc != 0 || wait_op(10000) != 0) {
        set_error("CCCD discovery failed"); return ESP_FAIL;
    }
    if (s_cccd_handle == 0) { set_error("CCCD not found"); return ESP_FAIL; }

    /* Enable notifications */
=======
        ESP_LOGE(TAG, "addr infer failed: %d", rc);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "connecting to %s (peer type=%d, own type=%d)",
             taddr, target->type, own_addr_type);

    clear_op_sem();
    rc = ble_gap_connect(own_addr_type, target,
                         LEGACY_T_CONNECT_MS, NULL, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "connect start failed: %d", rc);
        return ESP_FAIL;
    }
    if (xSemaphoreTake(s_conn_sem, pdMS_TO_TICKS(LEGACY_T_CONNECT_MS + 1000)) != pdTRUE) {
        ESP_LOGE(TAG, "connect timeout");
        return ESP_FAIL;
    }
    if (s_conn_status != 0) {
        ESP_LOGE(TAG, "connect failed: %d", s_conn_status);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "connected, handle=%d addr=%s", s_conn_handle, taddr);

    /* MTU exchange — best effort; writes stay at 20-byte chunks anyway. */
    clear_op_sem();
    ble_gattc_exchange_mtu(s_conn_handle, on_mtu, NULL);
    wait_op(2000);

    clear_op_sem();
    rc = ble_gattc_disc_svc_by_uuid(s_conn_handle, &LEGACY_SVC_UUID.u,
                                    on_disc_svc, NULL);
    if (rc != 0 || wait_op(LEGACY_T_GATT_MS) != 0) {
        ESP_LOGE(TAG, "legacy ring service not found");
        return ESP_FAIL;
    }
    if (s_svc_start == 0) {
        ESP_LOGE(TAG, "service range empty");
        return ESP_FAIL;
    }

    clear_op_sem();
    rc = ble_gattc_disc_all_chrs(s_conn_handle, s_svc_start, s_svc_end,
                                 on_disc_chr, NULL);
    if (rc != 0 || wait_op(LEGACY_T_GATT_MS) != 0) {
        ESP_LOGE(TAG, "characteristic discovery failed");
        return ESP_FAIL;
    }
    if (s_write_handle == 0 || s_notify_handle == 0) {
        ESP_LOGE(TAG, "write/notify characteristic not found");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "write=%d notify=%d", s_write_handle, s_notify_handle);

    clear_op_sem();
    rc = ble_gattc_disc_all_dscs(s_conn_handle, s_notify_handle, s_svc_end,
                                 on_disc_dsc, NULL);
    if (rc != 0 || wait_op(LEGACY_T_GATT_MS) != 0) {
        ESP_LOGE(TAG, "CCCD discovery failed");
        return ESP_FAIL;
    }
    if (s_cccd_handle == 0) {
        ESP_LOGE(TAG, "CCCD not found");
        return ESP_FAIL;
    }

>>>>>>> 1faf953 (Add support for legacy Wellue O2 oximeter ring; refactored oximeter backend)
    uint8_t cccd_val[2] = { 0x01, 0x00 };
    clear_op_sem();
    rc = ble_gattc_write_flat(s_conn_handle, s_cccd_handle,
                              cccd_val, 2, on_write_done, NULL);
<<<<<<< HEAD
    if (rc != 0 || wait_op(5000) != 0) {
        set_error("enable notify failed"); return ESP_FAIL;
    }
    ESP_LOGI(TAG, "notifications enabled (cccd=%d)", s_cccd_handle);
=======
    if (rc != 0 || wait_op(LEGACY_T_WRITE_MS) != 0) {
        ESP_LOGE(TAG, "enable notify failed");
        return ESP_FAIL;
    }
>>>>>>> 1faf953 (Add support for legacy Wellue O2 oximeter ring; refactored oximeter backend)
    return ESP_OK;
}

static void do_disconnect(void)
{
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        xSemaphoreTake(s_conn_sem, pdMS_TO_TICKS(3000));
    }
}

<<<<<<< HEAD
/* ── Protocol request/response ─────────────────────────────────────── */
/* Send a Legacy request and optionally wait for a response.
 * Uses write-with-response (Legacy protocol uses WRITE_REQ, not WRITE_CMD).
 * Frames larger than 20 bytes are split into multiple writes. */
static esp_err_t legacy_request(uint8_t cmd, uint16_t block,
                                 const uint8_t *payload, int plen,
                                 bool expect_reply, int timeout_ms)
{
    uint8_t frame[LEGACY_MAX_FRAME];
    int flen = legacy_encode(frame, sizeof(frame), cmd, block,
                              payload, plen);
    if (flen < 0) return ESP_FAIL;

    if (expect_reply) {
        while (xSemaphoreTake(s_resp_sem, 0) == pdTRUE) { }
        s_resp_len = 0;
    }

    /* Write the frame in 20-byte chunks using write-with-response for
     * every chunk.  The Legacy protocol requires WRITE_REQUEST for each
     * chunk; using WRITE_CMD for subsequent chunks can cause drops or
     * ordering issues on rings with small ATT buffers. */
    int offset = 0;
    while (offset < flen) {
        int chunk = flen - offset;
        if (chunk > LEGACY_BLE_CHUNK) chunk = LEGACY_BLE_CHUNK;

        clear_op_sem();
        int rc = ble_gattc_write_flat(s_conn_handle, s_write_handle,
                                       frame + offset, chunk,
                                       on_write_done, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "write failed cmd=0x%02x offset=%d rc=%d",
                     cmd, offset, rc);
            return ESP_FAIL;
        }
        if (wait_op(5000) != 0) {
            ESP_LOGE(TAG, "write timeout cmd=0x%02x offset=%d", cmd, offset);
            return ESP_FAIL;
        }
        offset += chunk;
    }

    if (!expect_reply) return ESP_OK;

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (true) {
        TickType_t now = xTaskGetTickCount();
        if ((int32_t)(deadline - now) <= 0) {
            ESP_LOGE(TAG, "response timeout cmd=0x%02x", cmd);
            return ESP_ERR_TIMEOUT;
        }
        if (xSemaphoreTake(s_resp_sem, deadline - now) != pdTRUE) {
            ESP_LOGE(TAG, "response timeout cmd=0x%02x", cmd);
            return ESP_ERR_TIMEOUT;
        }
        if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE)
            return ESP_FAIL;
        /* Legacy protocol: any response with status 0 is accepted.
         * The status byte indicates success (0) or failure. */
        if (s_resp_status == 0)
            return ESP_OK;
        ESP_LOGW(TAG, "response status=%d for cmd=0x%02x", s_resp_status, cmd);
        return ESP_FAIL;
    }
}

/* ── CMD_INFO: get device info as JSON ─────────────────────────────── */
static esp_err_t legacy_get_info(char *serial, size_t serial_sz,
                                  char *firmware, size_t fw_sz,
                                  char *file_list, size_t file_list_sz,
                                  char *current_time, size_t current_time_sz)
{
    if (legacy_request(CMD_INFO, 0, NULL, 0, true, 5000) != ESP_OK)
        return ESP_FAIL;

    /* Response is ASCII JSON.  Parse it. */
    if (s_resp_payload_len <= 0) return ESP_FAIL;

    /* Ensure null-terminated */
    char *json_str = heap_caps_malloc(s_resp_payload_len + 1, MALLOC_CAP_SPIRAM);
    if (!json_str) json_str = malloc(s_resp_payload_len + 1);
    if (!json_str) return ESP_ERR_NO_MEM;
    memcpy(json_str, s_resp_payload, s_resp_payload_len);
    json_str[s_resp_payload_len] = '\0';

    cJSON *j = cJSON_Parse(json_str);
    free(json_str);
    if (!j) {
        ESP_LOGE(TAG, "CMD_INFO: JSON parse failed");
        return ESP_FAIL;
    }

    esp_err_t ret = ESP_FAIL;
    cJSON *sn = cJSON_GetObjectItem(j, "SN");
    if (sn && cJSON_IsString(sn) && serial) {
        strlcpy(serial, sn->valuestring, serial_sz);
        ret = ESP_OK;
    }
    cJSON *fw = cJSON_GetObjectItem(j, "Model");
    if (fw && cJSON_IsString(fw) && firmware)
        strlcpy(firmware, fw->valuestring, fw_sz);
    cJSON *fl = cJSON_GetObjectItem(j, "FileList");
    if (fl && cJSON_IsString(fl) && file_list)
        strlcpy(file_list, fl->valuestring, file_list_sz);
    cJSON *ct = cJSON_GetObjectItem(j, "CurTIME");
    if (ct && cJSON_IsString(ct) && current_time)
        strlcpy(current_time, ct->valuestring, current_time_sz);

    cJSON_Delete(j);
    return ret;
}

/* ── CMD_CONFIG: set device time ───────────────────────────────────── */
static esp_err_t legacy_set_time(void)
{
    if (!time_is_usable()) {
        ESP_LOGW(TAG, "not setting ring clock: host time is unusable");
        return ESP_ERR_INVALID_STATE;
    }
    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);
    char time_str[80];
    snprintf(time_str, sizeof(time_str), "%04d-%02d-%02d,%02d:%02d:%02d",
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec);

    /* Build JSON config: {"SetTIME":"YYYY-MM-DD,HH:MM:SS"} */
    char json[128];
    snprintf(json, sizeof(json), "{\"SetTIME\":\"%s\"}", time_str);

    return legacy_request(CMD_CONFIG, 0, (uint8_t *)json, strlen(json),
                          true, 5000);
}

static time_t legacy_time_value(const char *value)
{
    if (!value) return (time_t)-1;
    int year, month, day, hour, minute, second;
    if (sscanf(value, "%d-%d-%d,%d:%d:%d", &year, &month, &day,
               &hour, &minute, &second) != 6)
        return (time_t)-1;
    struct tm tm = {0};
    tm.tm_year = year - 1900; tm.tm_mon = month - 1; tm.tm_mday = day;
    tm.tm_hour = hour; tm.tm_min = minute; tm.tm_sec = second; tm.tm_isdst = -1;
    time_t result = mktime(&tm);
    struct tm check;
    if (result == (time_t)-1 || !localtime_r(&result, &check) ||
        check.tm_year != year - 1900 || check.tm_mon != month - 1 ||
        check.tm_mday != day || check.tm_hour != hour ||
        check.tm_min != minute || check.tm_sec != second)
        return (time_t)-1;
    return result;
}

static esp_err_t legacy_sync_time_if_needed(const char *ring_time,
                                             const char *expected_serial)
{
    if (!time_is_usable()) return ESP_ERR_INVALID_STATE;
    time_t now = time(NULL);
    time_t device = legacy_time_value(ring_time);
    if (device != (time_t)-1 && llabs((long long)(now - device)) <= 2)
        return ESP_OK;
    if (legacy_set_time() != ESP_OK) return ESP_FAIL;

    char serial[32] = {0}, verify_time[32] = {0};
    if (legacy_get_info(serial, sizeof(serial), NULL, 0, NULL, 0,
                        verify_time, sizeof(verify_time)) != ESP_OK ||
        !expected_serial || strcmp(serial, expected_serial) != 0)
        return ESP_FAIL;
    device = legacy_time_value(verify_time);
    now = time(NULL);
    if (device == (time_t)-1 || llabs((long long)(now - device)) > 5) {
        ESP_LOGW(TAG, "ring clock verification failed: '%s'", verify_time);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "ring clock synchronized and verified");
    return ESP_OK;
}

/* ── File download: FILE_OPEN / FILE_READ / FILE_CLOSE ─────────────── */

/* Parse the FileList from CMD_INFO into individual filenames.
 * Returns the number of filenames parsed. */
static int parse_file_list(const char *file_list,
                            char names[][32], int max_count)
{
    if (!file_list || !file_list[0]) return 0;
    int count = 0;
    const char *p = file_list;
    while (count < max_count && *p) {
        const char *comma = strchr(p, ',');
        int len = comma ? (int)(comma - p) : (int)strlen(p);
        if (len > 0 && len < 32) {
            memcpy(names[count], p, len);
            names[count][len] = '\0';
            count++;
        }
        if (!comma) break;
        p = comma + 1;
    }
    return count;
}

static esp_err_t legacy_convert_stored(const char *name)
{
    char source_path[640];
    char recording_id[32];
    strlcpy(recording_id, name, sizeof(recording_id));
    char *dot = strrchr(recording_id, '.');
    if (dot) *dot = '\0';
    snprintf(source_path, sizeof(source_path), SD_OXYMETRY_DIR "/files/%s/%s",
             s_serial, name);
    esp_err_t conversion = oximetry_canonical_convert_vld3(s_serial, recording_id, source_path);
    if (conversion != ESP_OK) {
        ESP_LOGW(TAG, "canonical conversion pending for '%s': %s", name,
                 esp_err_to_name(conversion));
        ox_store_index_mark_converted(s_serial, name, false, esp_err_to_name(conversion));
        return ESP_ERR_INVALID_STATE;
    }
    ox_store_index_mark_converted(s_serial, name, true, NULL);
    upload_sched_request_scan();
    return ESP_OK;
}

static esp_err_t legacy_pull_file(const char *name)
{
    /* FILE_OPEN: payload = filename + null terminator */
    uint8_t open_pl[32];
    int name_len = strlen(name);
    if (name_len > 30) return ESP_FAIL;
    memcpy(open_pl, name, name_len);
    open_pl[name_len] = '\0';
    int open_pl_len = name_len + 1;

    if (legacy_request(CMD_FILE_OPEN, 0, open_pl, open_pl_len, true, 5000) != ESP_OK)
        return ESP_FAIL;

    uint32_t file_size = 0;
    if (s_resp_payload_len >= 4)
        file_size = s_resp_payload[0] | (s_resp_payload[1] << 8) |
                    (s_resp_payload[2] << 16) | (s_resp_payload[3] << 24);
    ESP_LOGI(TAG, "pulling '%s' (%lu bytes)", name, (unsigned long)file_size);

    if (file_size == 0) {
        ESP_LOGW(TAG, "file '%s' has zero size", name);
        legacy_request(CMD_FILE_CLOSE, 0, NULL, 0, true, 2000);
        return ESP_FAIL;
    }

    /* Gen1 FILE_READ is sequential (no random access by offset).
     * If a .part exists, we must re-download from scratch.
     * Files are small (~10-15KB for 8h at 4s/sample), so this is acceptable. */
    long prior = ox_store_part_size(name);
    if (prior > 0) {
        ESP_LOGI(TAG, "discarding %ld bytes of .part (Gen1 has no resume)", prior);
        ox_store_part_remove(name);
    }

    uint32_t offset = 0;
    uint16_t block = 0;
    int empty_count = 0;

    while (offset < file_size) {
        if (legacy_request(CMD_FILE_READ, block, NULL, 0, true, 10000) != ESP_OK) {
            ESP_LOGW(TAG, "file read timeout at block=%u offset=%lu",
                     block, (unsigned long)offset);
            break;
        }

        /* Verify block number matches (documented framing collision issue) */
        if (s_resp_block != block) {
            ESP_LOGW(TAG, "block mismatch: got %u, expected %u — retrying",
                     s_resp_block, block);
            if (++empty_count > 3) break;
            continue;
        }

        int chunk_len = s_resp_payload_len;
        if (chunk_len <= 0) {
            if (++empty_count > 3) break;
            continue;
        }
        empty_count = 0;

        if ((uint64_t)offset + chunk_len > file_size) {
            ESP_LOGW(TAG, "data past file size at offset=%lu",
                     (unsigned long)offset);
            /* Truncate to file_size */
            chunk_len = file_size - offset;
        }

        if (ox_store_part_append(name, s_resp_payload, chunk_len) != ESP_OK) {
            ESP_LOGE(TAG, "SD write failed at offset=%lu", (unsigned long)offset);
            break;
        }
        offset += chunk_len;
        block++;
    }

    /* FILE_CLOSE — always, even on error */
    legacy_request(CMD_FILE_CLOSE, 0, NULL, 0, true, 2000);

    if (offset != file_size) {
        ESP_LOGW(TAG, "incomplete '%s': %lu/%lu bytes; retaining .part",
                 name, (unsigned long)offset, (unsigned long)file_size);
        return ESP_FAIL;
    }

    bool finalised = ox_store_finalize_native(s_serial, name, (long)file_size);
    ESP_LOGI(TAG, "pulled '%s': %lu bytes, finalised=%d",
             name, (unsigned long)offset, finalised);
    if (!finalised) return ESP_FAIL;

    /* Convert to canonical SNT v3 format */
    return legacy_convert_stored(name);
}

/* ── NVS persistence ───────────────────────────────────────────────── */
#define OX_NVS_NS "oximeter"

struct ox_nvs_arg {
    char serial[32];
    char firmware[16];
    char name_prefix[16];
    char ble_name[40];
    char last_addr[18];
};

static esp_err_t do_save_nvs(void *arg)
{
    const struct ox_nvs_arg *a = arg;
    struct ox_nvs_arg local = *a;
    nvs_handle_t h;
    esp_err_t e = nvs_open(OX_NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    nvs_set_str(h, "serial", local.serial);
    nvs_set_str(h, "firmware", local.firmware);
    nvs_set_str(h, "name_prefix", local.name_prefix);
    nvs_set_str(h, "ble_name", local.ble_name);
    nvs_set_str(h, "last_addr", local.last_addr);
    nvs_set_u8(h, "driver", (uint8_t)OX_DRIVER_LEGACY);
    e = nvs_commit(h);
    nvs_close(h);
    return e;
}

static esp_err_t do_erase_nvs(void *arg)
{
    (void)arg;
    nvs_handle_t h;
    esp_err_t e = nvs_open(OX_NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    nvs_erase_key(h, "serial");
    nvs_erase_key(h, "firmware");
    nvs_erase_key(h, "name_prefix");
    nvs_erase_key(h, "ble_name");
    nvs_erase_key(h, "last_addr");
    nvs_erase_key(h, "driver");
    nvs_erase_key(h, "probe_mode");
    e = nvs_commit(h);
    nvs_close(h);
    return e;
}

static esp_err_t do_save_probe_mode(void *arg)
{
    uint8_t mode = (uint8_t)(intptr_t)arg;
    nvs_handle_t h;
    esp_err_t e = nvs_open(OX_NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    nvs_set_u8(h, "probe_mode", mode);
    e = nvs_commit(h);
    nvs_close(h);
    return e;
}

static ox_probe_mode_t s_probe_mode = OX_PROBE_PERSISTENT;

static void load_paired_from_nvs(void)
{
    nvs_handle_t h;
    nvs_writer_lock();
    if (nvs_open(OX_NVS_NS, NVS_READONLY, &h) != ESP_OK) { nvs_writer_unlock(); return; }
    size_t len;

    uint8_t drv;
    bool driver_matches = nvs_get_u8(h, "driver", &drv) == ESP_OK && drv == OX_DRIVER_LEGACY;
    len = sizeof(s_serial);
    if (driver_matches && nvs_get_str(h, "serial", s_serial, &len) == ESP_OK && s_serial[0]) {
        s_paired = true;
        len = sizeof(s_firmware);
        nvs_get_str(h, "firmware", s_firmware, &len);
        len = sizeof(s_name_prefix);
        nvs_get_str(h, "name_prefix", s_name_prefix, &len);
        len = sizeof(s_ble_name);
        nvs_get_str(h, "ble_name", s_ble_name, &len);
        len = sizeof(s_paired_addr);
        nvs_get_str(h, "last_addr", s_paired_addr, &len);
    } else {
        s_serial[0] = '\0';
    }
    uint8_t pm;
    if (nvs_get_u8(h, "probe_mode", &pm) == ESP_OK && pm <= 1)
        s_probe_mode = (ox_probe_mode_t)pm;
    nvs_close(h);
    nvs_writer_unlock();

    /* Also try loading from paired.json (SD) as fallback */
    if (!s_paired) {
        char serial[32], fw[16], prefix[16], addr[18], drv[16], bname[40];
        if (ox_store_load_paired(serial, sizeof(serial),
                                 fw, sizeof(fw),
                                 prefix, sizeof(prefix),
                                 addr, sizeof(addr),
                                 drv, sizeof(drv),
                                 bname, sizeof(bname))) {
            if (strcmp(drv, "wellue_legacy") == 0) {
                strlcpy(s_serial, serial, sizeof(s_serial));
                strlcpy(s_firmware, fw, sizeof(s_firmware));
                strlcpy(s_name_prefix, prefix, sizeof(s_name_prefix));
                strlcpy(s_ble_name, bname, sizeof(s_ble_name));
                strlcpy(s_paired_addr, addr, sizeof(s_paired_addr));
                s_paired = true;
            }
        }
    }
}

/* ── Pair task ─────────────────────────────────────────────────────── */
static void pair_task(void *arg)
{
    struct pair_arg *pa = (struct pair_arg *)arg;
    const char *addr_str = pa->addr_str;
    const char *ble_name = pa->ble_name;
    ble_addr_t target;

    xSemaphoreTake(s_ops_mtx, portMAX_DELAY);
    set_state(OX_STATUS_CONNECTING);

    if (!parse_addr(addr_str, &target)) {
        set_error("invalid address: %s", addr_str);
        free(pa);
        xSemaphoreGive(s_ops_mtx);
        vTaskDelete(NULL);
        return;
    }

    if (do_connect_and_discover(&target) != ESP_OK) {
        do_disconnect();
        free(pa);
        xSemaphoreGive(s_ops_mtx);
        vTaskDelete(NULL);
        return;
    }

    /* No AUTH/SETUP needed for Legacy — just CMD_INFO */
    char serial[32] = {0}, firmware[16] = {0}, file_list[512] = {0}, ring_time[32] = {0};
    if (legacy_get_info(serial, sizeof(serial), firmware, sizeof(firmware),
                        file_list, sizeof(file_list), ring_time, sizeof(ring_time)) != ESP_OK) {
        set_error("get_info failed");
        do_disconnect();
        free(pa);
        xSemaphoreGive(s_ops_mtx);
        vTaskDelete(NULL);
        return;
    }

    if (serial[0] == '\0') {
        set_error("empty serial");
        do_disconnect();
        free(pa);
        xSemaphoreGive(s_ops_mtx);
        vTaskDelete(NULL);
        return;
    }

    if (legacy_sync_time_if_needed(ring_time, serial) != ESP_OK)
        ESP_LOGW(TAG, "pairing completed but ring clock could not be verified");

    /* Derive name_prefix from model name in serial (first 6 chars typically) */
    char prefix[16];
    /* Use the model name from the serial if possible (e.g. "O2Ring") */
    strlcpy(prefix, serial, sizeof(prefix));
    /* If serial starts with a known pattern, use a shorter prefix */
    if (strncmp(serial, "O2Ring", 6) == 0)
        strlcpy(prefix, "O2Ring", sizeof(prefix));
    else if (strncmp(serial, "KidsO2", 6) == 0)
        strlcpy(prefix, "KidsO2", sizeof(prefix));
    else
        strlcpy(prefix, "O2Ring", sizeof(prefix));

    /* Build a human-readable display name.  For Gen1 rings the advertised
     * name is typically just "O2Ring" or "CMRing" without a serial suffix,
     * so we append the last 4 digits of the serial for uniqueness. */
    char display_name[40];
    if (ble_name && ble_name[0] && strstr(ble_name, serial) != NULL) {
        /* Advertised name already contains the serial — use it as-is */
        strlcpy(display_name, ble_name, sizeof(display_name));
    } else {
        int slen = (int)strlen(serial);
        char last4[5];
        if (slen >= 4) {
            memcpy(last4, serial + slen - 4, 4);
            last4[4] = '\0';
        } else {
            strlcpy(last4, serial, sizeof(last4));
        }
        snprintf(display_name, sizeof(display_name), "%s %s", prefix, last4);
    }

    /* Save to NVS */
    struct ox_nvs_arg nvs_arg;
    strlcpy(nvs_arg.serial, serial, sizeof(nvs_arg.serial));
    strlcpy(nvs_arg.firmware, firmware, sizeof(nvs_arg.firmware));
    strlcpy(nvs_arg.name_prefix, prefix, sizeof(nvs_arg.name_prefix));
    strlcpy(nvs_arg.ble_name, display_name, sizeof(nvs_arg.ble_name));
    strlcpy(nvs_arg.last_addr, addr_str, sizeof(nvs_arg.last_addr));
    nvs_writer_run(do_save_nvs, &nvs_arg);

    /* Save to paired.json on SD */
    ox_store_save_paired(serial, firmware, prefix, addr_str, "wellue_legacy", display_name);

    /* Update in-RAM state */
    strlcpy(s_serial, serial, sizeof(s_serial));
    strlcpy(s_firmware, firmware, sizeof(s_firmware));
    strlcpy(s_name_prefix, prefix, sizeof(s_name_prefix));
    strlcpy(s_ble_name, display_name, sizeof(s_ble_name));
    strlcpy(s_paired_addr, addr_str, sizeof(s_paired_addr));
    s_paired = true;
    s_presence_served = false;
    s_pull_fail_count = 0;

    do_disconnect();
    set_state(OX_STATUS_PAIRED);
    ESP_LOGI(TAG, "paired: serial=%s fw=%s name=%s", serial, firmware, display_name);

    free(pa);
    xSemaphoreGive(s_ops_mtx);
    vTaskDelete(NULL);
}

/* ── Low-duty scan (caller holds s_ops_mtx) ─────────────────────────── */
static esp_err_t do_scan(int timeout_sec)
{
    s_scan_count = 0;

    struct ble_gap_disc_params dp = {
        .itvl = 160,
        .window = 48,
        .filter_policy = 0,
        .limited = 0,
        .passive = 1,
    };

    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(BLE_ADDR_RANDOM, &own_addr_type);
    if (rc != 0) own_addr_type = as11_ble_get_own_addr_type();

    while (xSemaphoreTake(s_scan_done, 0) == pdTRUE) { }

    rc = ble_gap_disc(own_addr_type,
                      timeout_sec * 1000, &dp, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "scan start failed: %d", rc);
        return ESP_FAIL;
    }

    xSemaphoreTake(s_scan_done, pdMS_TO_TICKS((timeout_sec + 2) * 1000));
    return ESP_OK;
}

/* ── File pull helper ──────────────────────────────────────────────── */
static bool do_pull_and_mark(bool *pulled_any)
{
    if (pulled_any) *pulled_any = false;
    set_state(OX_STATUS_PULLING);

    /* Get device info with file list */
    char serial[32] = {0}, firmware[16] = {0}, file_list[512] = {0}, ring_time[32] = {0};
    if (legacy_get_info(serial, sizeof(serial), firmware, sizeof(firmware),
                        file_list, sizeof(file_list), ring_time, sizeof(ring_time)) != ESP_OK) {
        ESP_LOGW(TAG, "CMD_INFO failed during pull");
        return false;
    }

    /* Verify serial matches paired device */
    if (serial[0] == '\0' || strcmp(serial, s_serial) != 0) {
        ESP_LOGW(TAG, "serial mismatch (got '%s', want '%s')",
                 serial, s_serial);
        return false;
    }
    if (legacy_sync_time_if_needed(ring_time, serial) != ESP_OK)
        ESP_LOGW(TAG, "ring clock sync unavailable — files will retain source time provenance");

    /* Parse file list */
    char names[32][32];
    int count = parse_file_list(file_list, names, 32);
    ESP_LOGI(TAG, "file list: %d files", count);

    bool pull_ok = true;
    for (int i = 0; i < count; i++) {
        if (names[i][0] == '\0') continue;

        /* Strip .vld extension for index check */
        char base_name[32];
        strlcpy(base_name, names[i], sizeof(base_name));
        char *dot = strrchr(base_name, '.');
        if (dot) *dot = '\0';

        int idx = ox_store_index_check(s_serial, base_name);
        if (idx != 1) idx = ox_store_index_check(s_serial, names[i]);
        if (idx == 1) {
            if (ox_store_index_conversion_check(s_serial, names[i]) != 1 &&
                legacy_convert_stored(names[i]) != ESP_OK) {
                ESP_LOGW(TAG, "conversion still pending for '%s'", names[i]);
                pull_ok = false;
            } else {
                ESP_LOGD(TAG, "skip '%s' (already finalised and converted)", names[i]);
            }
            continue;
        }

        ESP_LOGI(TAG, "pulling file %d/%d: '%s'", i + 1, count, names[i]);
        esp_err_t result = legacy_pull_file(names[i]);
        if (result != ESP_OK) {
            pull_ok = false;
            if (result == ESP_ERR_INVALID_STATE) {
                if (pulled_any) *pulled_any = true;
                ESP_LOGW(TAG, "downloaded '%s'; conversion deferred", names[i]);
                continue;
            }
            ESP_LOGW(TAG, "transfer failed for '%s' — ending this sync attempt", names[i]);
            break;
        }
        if (pulled_any) *pulled_any = true;
    }
    return pull_ok;
}

/* ── Background watch task ─────────────────────────────────────────── */
#define OX_END_WINDOW_MS  130000

static void pull_task(void *arg)
{
    (void)arg;

    int wait_ms = 0;
    while (!as11_ble_is_host_ready() && wait_ms < 15000) {
        vTaskDelay(pdMS_TO_TICKS(500));
        wait_ms += 500;
    }
    if (!as11_ble_is_host_ready()) {
        ESP_LOGW(TAG, "watch: host not ready, aborting");
        vTaskDelete(NULL);
        return;
    }

    ox_store_ensure_dirs();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(15000));

        if (!s_paired || s_serial[0] == '\0')
            continue;
        if (xSemaphoreTake(s_ops_mtx, 0) != pdTRUE)
            continue;

        if (!sd_storage_is_ready()) {
            xSemaphoreGive(s_ops_mtx);
            continue;
        }

        if (do_scan(4) != ESP_OK) {
            xSemaphoreGive(s_ops_mtx);
            continue;
        }

        if (s_scan_count == 0) {
            if (s_ring_present)
                ESP_LOGI(TAG, "ring gone — next appearance is a new sync window");
            s_ring_present = false;
            if (s_presence_served)
                s_presence_served = false;
            s_pull_fail_count = 0;
            xSemaphoreGive(s_ops_mtx);
            continue;
        }

        /* Select the paired device from scan results.  On first boot
         * after pairing, s_paired_addr holds the address from the pair
         * scan.  MAC can rotate on factory reset, so if the stored
         * address isn't found, fall back to the first result (the
         * serial check in do_pull_and_mark is the ultimate gate). */
        int idx = find_paired_in_scan();
        if (idx < 0) {
            if (s_paired_addr[0] != '\0') {
                ESP_LOGW(TAG, "paired addr %s not in scan results; "
                         "using first device (serial will be verified)",
                         s_paired_addr);
            }
            idx = 0;
        }

        if (!s_ring_present) {
            ESP_LOGI(TAG, "ring present: '%s' rssi=%d",
                     s_scan[idx].name, s_scan[idx].rssi);
            s_ring_present = true;
        }

        /* Update paired addr hint from the selected device. */
        addr_to_str(&s_scan[idx].addr, s_paired_addr, sizeof(s_paired_addr));

        if (s_presence_served) {
            if ((xTaskGetTickCount() - s_served_at) < pdMS_TO_TICKS(OX_END_WINDOW_MS)) {
                xSemaphoreGive(s_ops_mtx);
                continue;
            }
            ESP_LOGI(TAG, "watch: still advertising past END window — re-worn, resuming probes");
            s_presence_served = false;
            s_pull_fail_count = 0;
        }

        /* Connect and probe */
        set_state(OX_STATUS_CONNECTING);

        if (do_connect_and_discover(&s_scan[idx].addr) != ESP_OK) {
            ESP_LOGW(TAG, "watch: connect failed: %s", s_error);
            do_disconnect();
            set_state(OX_STATUS_PAIRED);
            xSemaphoreGive(s_ops_mtx);
            continue;
        }

        /* Gen1 rings only advertise when in Standby Mode (off-finger,
         * post-recording).  If the ring is visible during scan, it is
         * already off-finger and ready for file download.  The curfew
         * mechanism (130s no-reconnect after successful pull) ensures
         * the ring can power off on its own (~2 min auto-off timeout).
         * CMD_READ_SENSORS (0x17) returns live SpO2/HR data, not a
         * reliable worn-state flag — its byte 11 interpretation is
         * inconsistent across reference projects, so we skip the
         * worn-state check entirely. */

        /* Give the ring time to flush the recording */
        vTaskDelay(pdMS_TO_TICKS(3000));
        if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            do_disconnect();
            set_state(OX_STATUS_PAIRED);
            xSemaphoreGive(s_ops_mtx);
            continue;
        }

        bool pulled_any = false;
        bool pull_ok = do_pull_and_mark(&pulled_any);

        /* If no new files, wait and retry once */
        if (pull_ok && !pulled_any &&
            s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ESP_LOGI(TAG, "watch: no new files — waiting 5s for ring to finalize");
            vTaskDelay(pdMS_TO_TICKS(5000));
            if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE)
                pull_ok = do_pull_and_mark(&pulled_any);
        }

        do_disconnect();

        if (pull_ok) {
            s_presence_served = true;
            s_served_at = xTaskGetTickCount();
            s_pull_fail_count = 0;
            ESP_LOGI(TAG, "sync window served — no reconnect; ring powers off on its own");
        } else if (!s_presence_served) {
            /* Pull failed.  Retry quickly while the ring is still advertising
             * (it advertises for ~19s per cycle).  After OX_PULL_MAX_FAST_RETRIES
             * consecutive failures, apply the 130-second curfew so the ring can
             * power off and conserve battery between advertising cycles. */
            if (++s_pull_fail_count < OX_PULL_MAX_FAST_RETRIES) {
                ESP_LOGW(TAG, "sync incomplete — fast retry %d/%d (ring still advertising)",
                         s_pull_fail_count, OX_PULL_MAX_FAST_RETRIES);
            } else {
                s_presence_served = true;
                s_served_at = xTaskGetTickCount();
                ESP_LOGW(TAG, "sync incomplete after %d retries — curfew %ds (let ring rest)",
                         s_pull_fail_count, OX_END_WINDOW_MS / 1000);
            }
        }
        set_state(OX_STATUS_PAIRED);

        xSemaphoreGive(s_ops_mtx);
    }
}

/* ── Public API (driver vtable) ────────────────────────────────────── */
static void legacy_init(void)
{
    if (s_initialized) return;
    s_initialized = true;
    s_state_mtx = xSemaphoreCreateMutex();
    s_ops_mtx   = xSemaphoreCreateMutex();
    s_op_sem    = xSemaphoreCreateBinary();
    s_conn_sem  = xSemaphoreCreateBinary();
    s_resp_sem  = xSemaphoreCreateBinary();
    s_scan_done = xSemaphoreCreateBinary();
    if (!s_state_mtx || !s_ops_mtx || !s_op_sem || !s_conn_sem ||
        !s_resp_sem || !s_scan_done)
        return;

    s_scan = heap_caps_malloc(sizeof(struct ox_scan_result) * OX_SCAN_MAX,
                              MALLOC_CAP_SPIRAM);
    s_resp_buf = heap_caps_malloc(LEGACY_MAX_FRAME, MALLOC_CAP_SPIRAM);
    s_resp_payload = heap_caps_malloc(LEGACY_MAX_FRAME, MALLOC_CAP_SPIRAM);
    if (!s_scan || !s_resp_buf || !s_resp_payload) {
        ESP_LOGE(TAG, "init: failed to allocate PSRAM buffers");
        return;
    }

    load_paired_from_nvs();
    ox_store_ensure_dirs();
    if (sd_storage_is_ready()) oximetry_canonical_ensure_dirs();

    if (s_paired)
        set_state(OX_STATUS_PAIRED);

    TaskHandle_t h = psram_task_create(pull_task, "ox_leg_pull", 8192, NULL, 3,
                                       tskNO_AFFINITY, NULL, NULL);
    if (!h) {
        ESP_LOGW(TAG, "failed to create pull task");
    }
}

static esp_err_t legacy_scan(int timeout_sec)
{
    if (!as11_ble_is_host_ready())
        return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_ops_mtx, 0) != pdTRUE)
        return ESP_ERR_INVALID_STATE;

    s_scan_count = 0;
    set_state(OX_STATUS_SCANNING);

    struct ble_gap_disc_params dp = {
        .itvl = 96,
        .window = 96,
        .filter_policy = 0,
        .limited = 0,
        .passive = 0,
    };

    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(BLE_ADDR_RANDOM, &own_addr_type);
    if (rc != 0) own_addr_type = as11_ble_get_own_addr_type();

    rc = ble_gap_disc(own_addr_type,
                      timeout_sec * 1000, &dp, gap_event, NULL);
    if (rc != 0) {
        set_error("scan start failed: %d", rc);
        xSemaphoreGive(s_ops_mtx);
        return ESP_FAIL;
    }

    xSemaphoreTake(s_scan_done, pdMS_TO_TICKS((timeout_sec + 2) * 1000));

    if (s_paired)
        set_state(OX_STATUS_PAIRED);
    else
        set_state(OX_STATUS_IDLE);

    xSemaphoreGive(s_ops_mtx);
    return ESP_OK;
}

static cJSON *legacy_get_scan_results(void)
{
    cJSON *arr = cJSON_CreateArray();
    for (int i = 0; i < s_scan_count; i++) {
        cJSON *e = cJSON_CreateObject();
        char addr_str[18];
        addr_to_str(&s_scan[i].addr, addr_str, sizeof(addr_str));
        cJSON_AddStringToObject(e, "addr", addr_str);
        cJSON_AddStringToObject(e, "name", s_scan[i].name);
        cJSON_AddNumberToObject(e, "rssi", s_scan[i].rssi);
        cJSON_AddStringToObject(e, "type", "legacy");
        cJSON_AddItemToArray(arr, e);
    }
    return arr;
}

static esp_err_t legacy_pair(const char *addr_str)
{
    if (!as11_ble_is_host_ready())
        return ESP_ERR_INVALID_STATE;

    struct pair_arg *pa = calloc(1, sizeof(*pa));
    if (!pa) return ESP_ERR_NO_MEM;
    strlcpy(pa->addr_str, addr_str, sizeof(pa->addr_str));

    /* Look up the BLE advertised name from the last scan results. */
    for (int i = 0; i < s_scan_count; i++) {
        char scan_addr[24];
        addr_to_str(&s_scan[i].addr, scan_addr, sizeof(scan_addr));
        if (strcmp(scan_addr, addr_str) == 0) {
            strlcpy(pa->ble_name, s_scan[i].name, sizeof(pa->ble_name));
            break;
        }
    }

    TaskHandle_t h = psram_task_create(pair_task, "ox_leg_pair", 8192, pa, 5,
                                       tskNO_AFFINITY, NULL, NULL);
    if (!h) {
        free(pa);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static void legacy_forget(void)
{
    s_paired = false;
    s_presence_served = false;
    s_pull_fail_count = 0;
    s_serial[0] = '\0';
    s_firmware[0] = '\0';
    s_name_prefix[0] = '\0';
    s_ble_name[0] = '\0';
    s_paired_addr[0] = '\0';

    nvs_writer_run(do_erase_nvs, NULL);
    ox_store_delete_paired();

    set_state(OX_STATUS_IDLE);
}

static const char *legacy_get_status(void)
{
    return s_status;
}

static const char *legacy_get_error(void)
{
    return s_error;
}

static bool legacy_is_paired(void)
{
    return s_paired;
}

static cJSON *legacy_get_paired_info(void)
{
    if (!s_paired) return NULL;
    cJSON *info = cJSON_CreateObject();
    cJSON_AddStringToObject(info, "serial", s_serial);
    if (s_firmware[0]) cJSON_AddStringToObject(info, "firmware", s_firmware);
    if (s_name_prefix[0]) cJSON_AddStringToObject(info, "name_prefix", s_name_prefix);
    if (s_ble_name[0]) cJSON_AddStringToObject(info, "ble_name", s_ble_name);
    if (s_paired_addr[0]) cJSON_AddStringToObject(info, "addr", s_paired_addr);
    cJSON_AddStringToObject(info, "driver", "wellue_legacy");
    return info;
}

static ox_probe_mode_t legacy_get_probe_mode(void)
{
    return s_probe_mode;
}

static esp_err_t legacy_set_probe_mode(ox_probe_mode_t mode)
{
    if (mode != OX_PROBE_LEGACY && mode != OX_PROBE_PERSISTENT)
        return ESP_ERR_INVALID_ARG;
    if (mode == s_probe_mode)
        return ESP_OK;
    s_probe_mode = mode;
    nvs_writer_run(do_save_probe_mode, (void *)(intptr_t)mode);
    ESP_LOGI(TAG, "probe mode set to %s",
             mode == OX_PROBE_PERSISTENT ? "persistent" : "legacy");
    return ESP_OK;
}

const ox_driver_ops_t legacy_driver_ops = {
    .init             = legacy_init,
    .scan             = legacy_scan,
    .get_scan_results = legacy_get_scan_results,
    .pair             = legacy_pair,
    .forget           = legacy_forget,
    .get_status       = legacy_get_status,
    .get_error        = legacy_get_error,
    .is_paired        = legacy_is_paired,
    .get_paired_info  = legacy_get_paired_info,
    .get_probe_mode   = legacy_get_probe_mode,
    .set_probe_mode   = legacy_set_probe_mode,
=======
/* ── Request/response transport ────────────────────────────────────── */
/* Split into ≤20-byte chunks written sequentially with WRITE_REQUESTs
 * (the reference clients await each chunk; ordering is guaranteed). */
static esp_err_t write_chunks(const uint8_t *data, int len)
{
    for (int off = 0; off < len; off += LEGACY_WRITE_CHUNK) {
        int n = len - off < LEGACY_WRITE_CHUNK ? len - off : LEGACY_WRITE_CHUNK;
        clear_op_sem();
        int rc = ble_gattc_write_flat(s_conn_handle, s_write_handle,
                                      data + off, n, on_write_done, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "chunk write failed rc=%d", rc);
            return ESP_FAIL;
        }
        if (wait_op(LEGACY_T_WRITE_MS) != 0) {
            ESP_LOGE(TAG, "chunk write timeout");
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

/* Send a command and wait for its (single outstanding) response frame.
 * The device does not echo the command code — responses carry STATUS —
 * so strict lockstep is what associates replies with requests. */
static esp_err_t legacy_request(uint8_t cmd, uint16_t block,
                             const void *payload, int plen,
                             int timeout_ms)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return ESP_FAIL;

    uint8_t frame[LEGACY_MAX_FRAME];
    int flen = legacy_encode(frame, sizeof(frame), cmd, block, payload, plen);
    if (flen < 0) return ESP_FAIL;

    while (xSemaphoreTake(s_resp_sem, 0) == pdTRUE) { }
    s_acc_len = 0;
    s_have_rsp = false;

    ESP_LOGD(TAG, "tx cmd=0x%02x block=%u len=%d frame=%d",
             cmd, block, plen, flen);
    log_hex_prefix("tx", frame, flen, 12);
    if (write_chunks(frame, flen) != ESP_OK) return ESP_FAIL;

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (!s_have_rsp) {
        TickType_t now = xTaskGetTickCount();
        if ((int32_t)(deadline - now) <= 0) {
            ESP_LOGE(TAG, "response timeout cmd=0x%02x block=%u", cmd, block);
            return ESP_ERR_TIMEOUT;
        }
        if (xSemaphoreTake(s_resp_sem, deadline - now) != pdTRUE) {
            ESP_LOGE(TAG, "response timeout cmd=0x%02x block=%u", cmd, block);
            return ESP_ERR_TIMEOUT;
        }
        if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) return ESP_FAIL;
    }
    return ESP_OK;
}

/* ── INFO (0x14): JSON with CurBAT / FileList / Model / SN ─────────── */
struct legacy_info {
    char model[24];
    char sn[32];
    int battery;                                /* percent, -1 unknown  */
    char files[LEGACY_MAX_FILES][LEGACY_NAME_MAX + 1];
    int nfiles;
};

static esp_err_t parse_file_list(const char *flist, struct legacy_info *out)
{
    out->nfiles = 0;
    if (!flist) return ESP_OK;                  /* empty list is valid  */

    while (*flist && out->nfiles < LEGACY_MAX_FILES) {
        const char *comma = strchr(flist, ',');
        int nlen = comma ? (int)(comma - flist) : (int)strlen(flist);
        /* trim surrounding whitespace / CR LF */
        while (nlen > 0 && isspace((unsigned char)*flist)) { flist++; nlen--; }
        while (nlen > 0 && isspace((unsigned char)flist[nlen - 1])) nlen--;
        if (nlen > 0) {
            if (name_is_safe(flist, nlen)) {
                memcpy(out->files[out->nfiles], flist, nlen);
                out->files[out->nfiles][nlen] = '\0';
                out->nfiles++;
            } else {
                ESP_LOGW(TAG, "file list: ignoring unsafe name (%d chars)",
                         nlen);
            }
        }
        if (!comma) break;
        flist = comma + 1;
    }
    return ESP_OK;
}

static esp_err_t legacy_get_info(struct legacy_info *out)
{
    memset(out, 0, sizeof(*out));
    out->battery = -1;

    if (legacy_request(LEGACY_CMD_INFO, 0, NULL, 0, LEGACY_T_INFO_MS) != ESP_OK)
        return ESP_FAIL;
    if (s_rsp_status != LEGACY_STATUS_OK) {
        ESP_LOGE(TAG, "INFO status=%u", s_rsp_status);
        return ESP_FAIL;
    }

    int jlen = s_rsp_len;
    /* s_rsp_len may exceed the capture buffer if a peer ever sends an
     * oversized frame — never read past what was actually captured. */
    if (jlen <= 0 || jlen > LEGACY_INFO_JSON_MAX ||
        jlen > (int)sizeof(s_rsp_payload)) {
        ESP_LOGE(TAG, "INFO bad payload len=%d", jlen);
        return ESP_FAIL;
    }

    char *json = malloc(jlen + 1);
    if (!json) return ESP_FAIL;
    memcpy(json, s_rsp_payload, jlen);
    json[jlen] = '\0';
    ESP_LOGD(TAG, "INFO json[%d]: %s", jlen, json);

    cJSON *j = cJSON_Parse(json);
    free(json);
    if (!j) {
        ESP_LOGE(TAG, "INFO JSON parse failed");
        return ESP_FAIL;
    }

    cJSON *sn = cJSON_GetObjectItem(j, "SN");
    if (cJSON_IsString(sn) && sn->valuestring[0])
        strlcpy(out->sn, sn->valuestring, sizeof(out->sn));

    cJSON *model = cJSON_GetObjectItem(j, "Model");
    if (cJSON_IsString(model) && model->valuestring[0])
        strlcpy(out->model, model->valuestring, sizeof(out->model));

    cJSON *bat = cJSON_GetObjectItem(j, "CurBAT");
    if (cJSON_IsString(bat) && bat->valuestring[0])
        out->battery = atoi(bat->valuestring);   /* "75%" → 75 */
    else if (cJSON_IsNumber(bat))
        out->battery = bat->valueint;

    cJSON *flist = cJSON_GetObjectItem(j, "FileList");
    if (cJSON_IsString(flist))
        parse_file_list(flist->valuestring, out);
    /* Missing FileList ⇒ zero recordings. Unknown fields ignored. */

    cJSON_Delete(j);

    ESP_LOGI(TAG, "INFO model=%s sn=%s battery=%d files=%d",
             out->model[0] ? out->model : "?",
             out->sn[0] ? out->sn : "?",
             out->battery, out->nfiles);
    if (out->nfiles > 0) {
        char list[(LEGACY_NAME_MAX + 3) * 4 + 24];   /* first few names */
        int off = 0;
        for (int i = 0; i < out->nfiles && off < (int)sizeof(list) - 4; i++) {
            int w = snprintf(list + off, sizeof(list) - off, "%s%s",
                             i ? ", " : "", out->files[i]);
            if (w < 0 || off + w >= (int)sizeof(list) - 4) {
                off += snprintf(list + off, sizeof(list) - off, "…");
                break;
            }
            off += w;
        }
        ESP_LOGI(TAG, "recordings on ring: %s", list);
    }
    return ESP_OK;
}

/* ── FILE_OPEN / FILE_READ / FILE_CLOSE transfer loop ──────────────── */
static void send_close_best_effort(void)
{
    if (legacy_request(LEGACY_CMD_FILE_CLOSE, 0, NULL, 0, LEGACY_T_CLOSE_MS) == ESP_OK &&
        s_rsp_status == LEGACY_STATUS_OK) {
        ESP_LOGD(TAG, "close ack ok");
        return;
    }
    ESP_LOGW(TAG, "FILE_CLOSE ack missing — continuing");
}

/* Download one recording into inbox/<name>.part and promote it verbatim
 * to files/<serial>/<name> after an exact size check.  Never leaves a
 * partial file behind and never promotes a size-mismatched file. */
static esp_err_t pull_file(const char *serial, const char *name)
{
    int namelen = (int)strlen(name);
    if (!name_is_safe(name, namelen)) {
        ESP_LOGE(TAG, "refusing unsafe filename '%s'", name);
        return ESP_FAIL;
    }

    /* FILE_OPEN — payload is filename + mandatory NUL terminator. */
    uint8_t open_pl[LEGACY_NAME_MAX + 1];
    memcpy(open_pl, name, namelen);
    open_pl[namelen] = '\0';
    if (legacy_request(LEGACY_CMD_FILE_OPEN, 0, open_pl, namelen + 1,
                    LEGACY_T_OPEN_MS) != ESP_OK)
        return ESP_FAIL;
    if (s_rsp_status != LEGACY_STATUS_OK) {
        ESP_LOGE(TAG, "FILE_OPEN '%s' failed status=%u", name, s_rsp_status);
        send_close_best_effort();
        return ESP_FAIL;
    }
    if (s_rsp_len < 4) {
        ESP_LOGE(TAG, "FILE_OPEN short reply len=%d", s_rsp_len);
        send_close_best_effort();
        return ESP_FAIL;
    }
    uint32_t size = (uint32_t)s_rsp_payload[0] |
                    ((uint32_t)s_rsp_payload[1] << 8) |
                    ((uint32_t)s_rsp_payload[2] << 16) |
                    ((uint32_t)s_rsp_payload[3] << 24);
    log_hex_prefix("open reply", s_rsp_payload, s_rsp_len, 8);
    if (size == 0 || size > LEGACY_MAX_FILE_SIZE) {
        ESP_LOGE(TAG, "FILE_OPEN '%s' unreasonable size=%lu", name,
                 (unsigned long)size);
        send_close_best_effort();
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "opening %s (size=%lu)", name, (unsigned long)size);

    ox_store_part_remove(name);                 /* always start fresh */

    uint32_t received = 0;
    uint16_t block = 0;
    int errs = 0;
    uint32_t next_progress = 0;

    while (received < size) {
        if (legacy_request(LEGACY_CMD_FILE_READ, block, NULL, 0,
                        LEGACY_T_READ_MS) != ESP_OK) {
            if (++errs >= LEGACY_BLOCK_RETRIES) {
                ESP_LOGE(TAG, "FILE_READ block=%u failed %d times",
                         block, errs);
                goto fail;
            }
            continue;                           /* retry same block */
        }
        if (s_rsp_status != LEGACY_STATUS_OK) {
            ESP_LOGE(TAG, "FILE_READ block=%u status=%u", block, s_rsp_status);
            goto fail;
        }
        if (s_rsp_block != block) {
            /* Out-of-order/duplicate block would corrupt the stream. */
            ESP_LOGE(TAG, "FILE_READ desync: want block=%u got block=%u",
                     block, s_rsp_block);
            goto fail;
        }
        if (s_rsp_len <= 0) {
            if (++errs >= LEGACY_BLOCK_RETRIES) {
                ESP_LOGE(TAG, "FILE_READ block=%u empty %d times", block, errs);
                goto fail;
            }
            continue;
        }

        uint32_t remain = size - received;
        int n = (uint32_t)s_rsp_len < remain ? s_rsp_len : (int)remain;
        if ((uint32_t)s_rsp_len > remain)
            ESP_LOGW(TAG, "final block over-long (%d > %lu) — truncating",
                     s_rsp_len, (unsigned long)remain);

        if (ox_store_part_append(name, s_rsp_payload, n) != ESP_OK) {
            ESP_LOGE(TAG, "SD append failed at %lu/%lu",
                     (unsigned long)received, (unsigned long)size);
            goto fail;
        }
        received += n;
        block++;
        errs = 0;
        ESP_LOGD(TAG, "blk %u: +%d bytes (%lu/%lu)",
                 block - 1, n, (unsigned long)received, (unsigned long)size);

        if (received >= next_progress) {
            ESP_LOGI(TAG, "received=%lu/%lu",
                     (unsigned long)received, (unsigned long)size);
            if (next_progress == 0) next_progress = 16384;
            else next_progress <<= 1;
        }
    }

    /* FILE_CLOSE — leave no stale open handle on the ring. */
    ESP_LOGI(TAG, "file complete (%lu bytes), closing", (unsigned long)received);
    send_close_best_effort();

    /* Exact-size validation happens inside the store; only then is the
     * native .vld placed under files/<serial>/ and indexed. */
    if (!ox_store_promote_exact(serial, name, (long)size)) {
        ESP_LOGE(TAG, "promotion failed — partial download discarded");
        ox_store_part_remove(name);
        return ESP_FAIL;
    }
    return ESP_OK;

fail:
    send_close_best_effort();
    ox_store_part_remove(name);
    return ESP_FAIL;
}

/* ── Public: full sync session ─────────────────────────────────────── */
static ox_sync_err_t legacy_sync_session(const ble_addr_t *addr,
                                      const char *expect_serial,
                                      bool download_files,
                                      char *out_serial, size_t serial_sz,
                                      int *out_files_pulled)
{
    if (out_files_pulled) *out_files_pulled = 0;
    if (out_serial && serial_sz > 0) out_serial[0] = '\0';

    char taddr[18];
    fmt_addr(addr, taddr, sizeof(taddr));

    /* Fail cleanly rather than asserting on NULL handles if the caller
     * ever skips legacy_init(). */
    if (!s_op_sem || !s_conn_sem || !s_resp_sem) {
        ESP_LOGE(TAG, "session begin on uninitialised backend — call legacy_init() first");
        return OX_SYNC_ERR_CONNECT;
    }

    ESP_LOGI(TAG, "session begin addr=%s expect=%s download=%d",
             taddr, expect_serial ? expect_serial : "*", download_files);

    if (connect_and_discover(addr) != ESP_OK) {
        do_disconnect();
        ESP_LOGI(TAG, "session end: %s", sync_err_str(OX_SYNC_ERR_CONNECT));
        return OX_SYNC_ERR_CONNECT;
    }

    struct legacy_info info;
    if (legacy_get_info(&info) != ESP_OK || info.sn[0] == '\0') {
        ESP_LOGW(TAG, "INFO unavailable — ring may be worn/recording");
        do_disconnect();
        ESP_LOGI(TAG, "session end: %s", sync_err_str(OX_SYNC_ERR_INFO));
        return OX_SYNC_ERR_INFO;
    }

    /* Snapshot for the status UI (any successful INFO). */
    strlcpy(s_last_seen.model, info.model, sizeof(s_last_seen.model));
    s_last_seen.battery = info.battery;
    s_last_seen.nfiles = info.nfiles;
    s_last_seen.seen = time(NULL);
    s_last_seen.valid = true;

    if (expect_serial && strcmp(info.sn, expect_serial) != 0) {
        ESP_LOGW(TAG, "serial mismatch (got '%s', want '%s')",
                 info.sn, expect_serial);
        do_disconnect();
        ESP_LOGI(TAG, "session end: %s", sync_err_str(OX_SYNC_ERR_IDENTITY));
        return OX_SYNC_ERR_IDENTITY;
    }

    if (out_serial && serial_sz > 0)
        strlcpy(out_serial, info.sn, serial_sz);

    ox_sync_err_t result = OX_SYNC_OK;
    int pulled = 0;
    if (download_files) {
        for (int i = 0; i < info.nfiles; i++) {
            const char *name = info.files[i];

            int idx = ox_store_index_check(info.sn, name);
            if (idx == 1) {
                ESP_LOGI(TAG, "skip '%s' (already finalised)", name);
                continue;
            }

            ESP_LOGI(TAG, "pulling file %d/%d: '%s'", i + 1, info.nfiles, name);
            if (pull_file(info.sn, name) != ESP_OK) {
                ESP_LOGW(TAG, "pull failed for '%s'", name);
                result = OX_SYNC_ERR_TRANSFER;
                break;                          /* link state suspect */
            }
            pulled++;
        }
    } else {
        ESP_LOGI(TAG, "identify-only (files=%d on ring)", info.nfiles);
    }

    do_disconnect();

    if (out_files_pulled) *out_files_pulled = pulled;
    if (result == OX_SYNC_OK)
        ESP_LOGI(TAG, "session end: ok (pulled=%d)", pulled);
    else
        ESP_LOGW(TAG, "session end: %s (pulled=%d)",
                 sync_err_str(result), pulled);
    return result;
}

/* ── Public: advertisement classification ──────────────────────────── */
static bool legacy_last_seen(char *model, size_t model_sz,
                             int *battery, int *files_on_ring, long *age_s)
{
    if (!s_last_seen.valid) return false;
    if (model && model_sz) strlcpy(model, s_last_seen.model, model_sz);
    if (battery) *battery = s_last_seen.battery;
    if (files_on_ring) *files_on_ring = s_last_seen.nfiles;
    if (age_s) *age_s = (long)(time(NULL) - s_last_seen.seen);
    return true;
}

static bool legacy_name_match(const char *name)
{
    if (!name || !name[0]) return false;
    char up[32];
    int i;
    for (i = 0; i < 31 && name[i]; i++)
        up[i] = toupper((unsigned char)name[i]);
    up[i] = '\0';
    for (size_t k = 0; k < sizeof(LEGACY_NAME_KEYS) / sizeof(LEGACY_NAME_KEYS[0]);
         k++)
        if (strstr(up, LEGACY_NAME_KEYS[k])) return true;
    return false;
}

static bool legacy_adv_service_match(const uint8_t *adv_data, int adv_len)
{
    if (!adv_data) return false;
    int off = 0;
    while (off + 1 < adv_len) {
        uint8_t ad_len = adv_data[off];         /* length of AD element */
        if (ad_len == 0 || off + 1 + ad_len > adv_len) break;
        uint8_t ad_type = adv_data[off + 1];
        if ((ad_type == 0x06 || ad_type == 0x07) && ad_len >= 17) {
            /* incomplete/complete 128-bit service class UUIDs, LE */
            for (int p = 0; p + 16 <= ad_len - 1; p += 16)
                if (memcmp(adv_data + off + 2 + p, LEGACY_SVC_ADV_LE, 16) == 0)
                    return true;
        }
        off += 1 + ad_len;
    }
    return false;
}


/* ══ Backend interface implementation ════════════════════════════════ */

/* Advert scoring — see oximeter_backend.h for tier semantics.  An
 * explicit family name (tier 3) outranks the shared Viatom mfg-id
 * heuristic used by the OxyII backend, so an "O2Ring XXXX" advert is
 * never misrouted even though both families share id 0xF34E. */
static int legacy_adv_score(const char *name, uint16_t mfg_cid,
                            const uint8_t *raw_adv, int raw_len)
{
    (void)mfg_cid;
    if (legacy_name_match(name))
        return 3;                       /* explicit device-name match */
    if (legacy_adv_service_match(raw_adv, raw_len))
        return 1;                       /* service UUID seen in payload */
    return 0;
}

/* Pairing: identify-only session (no downloads). */
static bool legacy_identify(const ble_addr_t *addr,
                            char *serial, size_t serial_sz,
                            char *firmware, size_t fw_sz)
{
    if (serial_sz) serial[0] = '\0';
    if (fw_sz) firmware[0] = '\0';     /* protocol has no firmware field */

    ox_sync_err_t r = legacy_sync_session(addr, NULL, false,
                                          serial, serial_sz, NULL);
    return r == OX_SYNC_OK && serial[0] != '\0';
}

/* Status extras: last INFO data seen from any ring this boot. */
static void legacy_report_status(cJSON *info)
{
    char model[24];
    int bat = -1, nfiles = -1;
    long age = 0;
    if (!legacy_last_seen(model, sizeof(model), &bat, &nfiles, &age))
        return;
    if (model[0]) cJSON_AddStringToObject(info, "model", model);
    cJSON_AddNumberToObject(info, "battery", bat);
    cJSON_AddNumberToObject(info, "files_on_ring", nfiles);
}

const oximeter_backend_t oximeter_backend_legacy = {
    .proto_id              = OX_PROTO_LEGACY,
    .init                  = legacy_init,
    .adv_score             = legacy_adv_score,
    .identify              = legacy_identify,
    .sync                  = legacy_sync_session,
    .report_status         = legacy_report_status,
    .max_consecutive_fails = 3,
>>>>>>> 1faf953 (Add support for legacy Wellue O2 oximeter ring; refactored oximeter backend)
};
