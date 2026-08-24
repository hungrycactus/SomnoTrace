/*
 * SomnoTrace - Legacy Wellue ring (P02 / O2Ring) BLE protocol codec and session
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

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
static const ble_uuid128_t LEGACY_SVC_UUID =
    BLE_UUID128_INIT(0x39, 0x23, 0xcf, 0x40, 0x73, 0x16, 0x42, 0x9a,
                     0x5c, 0x41, 0x7e, 0x7d, 0xc4, 0x9a, 0x83, 0x14);
static const ble_uuid128_t LEGACY_WRITE_UUID =
    BLE_UUID128_INIT(0xa3, 0xe1, 0x26, 0x0a, 0xee, 0x9a, 0xe9, 0xbb,
                     0xb0, 0x49, 0x0b, 0xeb, 0xe7, 0xac, 0x00, 0x8b);
static const ble_uuid128_t LEGACY_NOTIFY_UUID =
    BLE_UUID128_INIT(0x57, 0x9a, 0x05, 0x43, 0x52, 0xcd, 0xb1, 0xa6,
                     0x1a, 0x4b, 0xe7, 0xa8, 0x4a, 0x59, 0x34, 0x07);

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
/* Build AA CMD ^CMD BLOCK LEN DATA CRC.  Returns total length, -1 if
 * it does not fit. */
static int legacy_encode(uint8_t *buf, int bufsz, uint8_t cmd, uint16_t block,
                      const void *payload, int plen)
{
    if (plen < 0 || plen > LEGACY_MAX_PAYLOAD) return -1;
    int total = LEGACY_HDR_LEN + plen + 1;
    if (total > bufsz) return -1;

    buf[0] = LEGACY_REQ_LEAD;
    buf[1] = cmd;
    buf[2] = cmd ^ 0xFF;
    buf[3] = block & 0xFF;
    buf[4] = (block >> 8) & 0xFF;
    buf[5] = plen & 0xFF;
    buf[6] = (plen >> 8) & 0xFF;
    if (plen > 0 && payload)
        memcpy(buf + LEGACY_HDR_LEN, payload, plen);
    buf[total - 1] = legacy_crc8(buf, total - 1);
    return total;
}

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
static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        s_conn_handle = event->connect.conn_handle;
        s_conn_status = event->connect.status;
        xSemaphoreGive(s_conn_sem);
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected (reason=%d)",
                 event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        /* Unblock any pending connect/request waits. */
        xSemaphoreGive(s_conn_sem);
        xSemaphoreGive(s_resp_sem);
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

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU negotiated: %d", event->mtu.value);
        return 0;

    default:
        return 0;
    }
}

/* ── GATT discovery callbacks ──────────────────────────────────────── */
static int on_mtu(uint16_t conn, const struct ble_gatt_error *err,
                  uint16_t mtu, void *arg)
{
    (void)conn; (void)arg; (void)mtu;
    s_op_status = err ? err->status : 0;
    xSemaphoreGive(s_op_sem);
    return 0;
}

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

/* ── Connect, discover this protocol's characteristics, subscribe ──── */
static esp_err_t connect_and_discover(const ble_addr_t *target)
{
    s_write_handle = s_notify_handle = s_cccd_handle = 0;
    s_svc_start = s_svc_end = 0;
    s_acc_len = 0;
    s_have_rsp = false;

    char taddr[18];
    fmt_addr(target, taddr, sizeof(taddr));

    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(target->type, &own_addr_type);
    if (rc != 0) {
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

    uint8_t cccd_val[2] = { 0x01, 0x00 };
    clear_op_sem();
    rc = ble_gattc_write_flat(s_conn_handle, s_cccd_handle,
                              cccd_val, 2, on_write_done, NULL);
    if (rc != 0 || wait_op(LEGACY_T_WRITE_MS) != 0) {
        ESP_LOGE(TAG, "enable notify failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void do_disconnect(void)
{
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        xSemaphoreTake(s_conn_sem, pdMS_TO_TICKS(3000));
    }
}

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
};
