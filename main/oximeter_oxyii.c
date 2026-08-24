/*
 * SomnoTrace - O2S / OxyII ring BLE protocol backend ("0xA5 protocol")
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

/* Clean-room OxyII BLE protocol backend for Wellue O2 Ring S /
 * SleepHQ O2 Ring Pro.  Implements the oximeter_backend_t interface;
 * scanning, pairing persistence, sync scheduling and storage live in
 * oximeter_common.c / oximeter_store.c.  Byte-level reference for both
 * backends: ADD_LEGACY_OXIMETER.md.
 */

#include "oximeter.h"
#include "oximeter_backend.h"
#include "oximeter_store.h"
#include "as11_ble.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <time.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "host/ble_hs.h"
#include "host/util/util.h"

static const char *TAG = "oxyii";

/* ── Protocol constants ────────────────────────────────────────────── */
#define OXYII_LEAD         0xA5
#define OXYII_HEADER_LEN   7
#define OXYII_MAX_FRAME    2048

#define OP_GET_CONFIG      0x00
#define OP_LIVE_B          0x04
#define OP_SETUP           0x10
#define OP_SET_UTC_TIME    0xC0
#define OP_GET_INFO        0xE1
#define OP_GET_BATTERY     0xE4
#define OP_GET_FILE_LIST   0xF1
#define OP_READ_FILE_START 0xF2
#define OP_READ_FILE_DATA  0xF3
#define OP_READ_FILE_END   0xF4
#define OP_AUTH            0xFF

/* Manufacturer ids seen in adverts. */
#define MFG_OXYII          0xF34E
#define MFG_RECORDING      0x036F   /* worn / recording — never a candidate */

/* MD5("lepucloud") = c2a7cf50dafed885a8f8f7eac44335f3 */
static const uint8_t LEPUCLOUD_MD5[16] = {
    0xc2, 0xa7, 0xcf, 0x50, 0xda, 0xfe, 0xd8, 0x85,
    0xa8, 0xf8, 0xf7, 0xea, 0xc4, 0x43, 0x35, 0xf3,
};

/* OxyII GATT UUIDs (128-bit, stored little-endian for NimBLE) */
static const ble_uuid128_t OXYII_SVC_UUID =
    BLE_UUID128_INIT(0x48, 0x12, 0xd0, 0x41, 0x29, 0x4e, 0x1b, 0x83,
                     0xf9, 0x98, 0x4b, 0xa1, 0x01, 0x00, 0xfb, 0xe8);
static const ble_uuid128_t OXYII_WRITE_UUID =
    BLE_UUID128_INIT(0x48, 0x12, 0xd0, 0x41, 0x29, 0x4e, 0x1b, 0x83,
                     0xf9, 0x98, 0x4b, 0xa1, 0x02, 0x00, 0xfb, 0xe8);
static const ble_uuid128_t OXYII_NOTIFY_UUID =
    BLE_UUID128_INIT(0x48, 0x12, 0xd0, 0x41, 0x29, 0x4e, 0x1b, 0x83,
                     0xf9, 0x98, 0x4b, 0xa1, 0x03, 0x00, 0xfb, 0xe8);

/* ── CRC8 (poly=0x07, init=0) over all bytes except trailing CRC ───── */
static uint8_t oxyii_crc8(const uint8_t *data, int len)
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
/* Encode an OxyII frame into buf.  Returns total frame length. */
static int oxyii_encode(uint8_t *buf, int bufsz, uint8_t op,
                         uint8_t flag, uint8_t seq,
                         const uint8_t *payload, int payload_len)
{
    int total = OXYII_HEADER_LEN + payload_len + 1;
    if (total > bufsz) return -1;

    buf[0] = OXYII_LEAD;
    buf[1] = op;
    buf[2] = ~op;
    buf[3] = flag;
    buf[4] = seq;
    buf[5] = payload_len & 0xFF;
    buf[6] = (payload_len >> 8) & 0xFF;
    if (payload && payload_len > 0)
        memcpy(buf + 7, payload, payload_len);
    buf[total - 1] = oxyii_crc8(buf, total - 1);
    return total;
}

/* Try to decode a frame from buf.  Returns total frame length on success,
 * -1 if incomplete (need more data), -2 if invalid (bad lead/crc). */
static int oxyii_try_decode(const uint8_t *buf, int len,
                             uint8_t *op, uint8_t *flag, uint8_t *seq,
                             uint8_t *payload, int *payload_len,
                             int payload_cap)
{
    if (len < OXYII_HEADER_LEN) return -1;
    if (buf[0] != OXYII_LEAD) return -2;
    if ((uint8_t)(~buf[1]) != buf[2]) return -2;

    int plen = buf[5] | (buf[6] << 8);
    int total = OXYII_HEADER_LEN + plen + 1;
    if (len < total) return -1;

    if (oxyii_crc8(buf, total - 1) != buf[total - 1]) return -2;

    if (op)   *op = buf[1];
    if (flag) *flag = buf[3];
    if (seq)  *seq = buf[4];
    if (payload && payload_cap > 0) {
        int n = plen < payload_cap ? plen : payload_cap;
        memcpy(payload, buf + 7, n);
    }
    if (payload_len) *payload_len = plen;
    return total;
}

/* ── Auth payload ──────────────────────────────────────────────────── */
/* Derive session key and XOR with LEPUCLOUD_MD5 to produce auth payload. */
static void oxyii_auth_payload(uint8_t *out16)
{
    uint8_t key[16];
    /* key[0..7] = LEPUCLOUD_MD5 even-indexed bytes */
    for (int i = 0; i < 8; i++)
        key[i] = LEPUCLOUD_MD5[i * 2];
    /* key[8..11] = "0000" (default serial prefix) */
    memcpy(key + 8, "0000", 4);
    /* key[12..15] = (ts >> 0), (ts >> 1), (ts >> 2), (ts >> 3) — not LE bytes */
    time_t now = time(NULL);
    for (int i = 0; i < 4; i++)
        key[12 + i] = (now >> i) & 0xFF;
    /* auth = key XOR LEPUCLOUD_MD5 */
    for (int i = 0; i < 16; i++)
        out16[i] = key[i] ^ LEPUCLOUD_MD5[i];
}

/* ── SET_UTC_TIME payload (8 bytes) ────────────────────────────────── */
static void oxyii_time_payload(uint8_t *out8)
{
    time_t now = time(NULL);
    struct tm tm;
    localtime_r(&now, &tm);
    out8[0] = (tm.tm_year + 1900) & 0xFF;
    out8[1] = ((tm.tm_year + 1900) >> 8) & 0xFF;
    out8[2] = tm.tm_mon + 1;
    out8[3] = tm.tm_mday;
    out8[4] = tm.tm_hour;
    out8[5] = tm.tm_min;
    out8[6] = tm.tm_sec;
    out8[7] = 0x00;
}

/* ── READ_FILE_START payload (20 bytes) ────────────────────────────── */
static void oxyii_file_start_payload(uint8_t *out20, const char *name)
{
    memset(out20, 0, 20);
    size_t n = strlen(name);
    if (n > 16) n = 16;
    memcpy(out20, name, n);
    /* bytes 16..19: file type = 0 */
}

/* ── READ_FILE_DATA payload (4 bytes absolute offset) ──────────────── */
static void oxyii_file_data_payload(uint8_t *out4, uint32_t offset)
{
    out4[0] = offset & 0xFF;
    out4[1] = (offset >> 8) & 0xFF;
    out4[2] = (offset >> 16) & 0xFF;
    out4[3] = (offset >> 24) & 0xFF;
}

/* ── Backend state (single connection at a time, ops mutex serialises) */
static SemaphoreHandle_t s_op_sem;      /* GATT op completion */
static SemaphoreHandle_t s_conn_sem;    /* connect completion */
static SemaphoreHandle_t s_resp_sem;    /* notification response */
static volatile int s_op_status;
static volatile int s_conn_status;

/* BLE connection state */
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_write_handle;
static uint16_t s_notify_handle;
static uint16_t s_cccd_handle;
static uint16_t s_svc_start, s_svc_end;
static uint8_t s_seq = 0;

/* Notification accumulation buffer */
static uint8_t s_resp_buf[OXYII_MAX_FRAME];
static int s_resp_len;
static uint8_t s_resp_opcode;
static uint8_t s_resp_payload[OXYII_MAX_FRAME];
static int s_resp_payload_len;

/* Serial of the currently paired device — needed by pull_file() to pick
 * the storage directory.  Set by sync()/identify() via common layer. */
static char s_dev_serial[32];

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

static bool name_is_oxyii(const char *name)
{
    if (!name || !name[0]) return false;
    char up[32];
    int i;
    for (i = 0; i < 31 && name[i]; i++)
        up[i] = toupper((unsigned char)name[i]);
    up[i] = '\0';
    return strncmp(up, "S8-AW", 5) == 0 ||
           strncmp(up, "SHQO2PRO", 8) == 0;
}

/* ── Connection GAP event handler (this backend's connections only) ── */
static void handle_notify_rx(const uint8_t *data, int len);

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
        /* Unblock any waiting request */
        xSemaphoreGive(s_resp_sem);
        xSemaphoreGive(s_conn_sem);
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

/* Handle notification: accumulate and try to decode. */
static void handle_notify_rx(const uint8_t *data, int len)
{
    if (s_resp_len + len > (int)sizeof(s_resp_buf)) {
        ESP_LOGW(TAG, "notify overflow: resp_len=%d + %d > %d",
                 s_resp_len, len, (int)sizeof(s_resp_buf));
        s_resp_len = 0;
    }
    memcpy(s_resp_buf + s_resp_len, data, len);
    s_resp_len += len;

    uint8_t op, flag, seq;
    int plen;
    int rc = oxyii_try_decode(s_resp_buf, s_resp_len, &op, &flag, &seq,
                               s_resp_payload, &plen, sizeof(s_resp_payload));
    if (rc > 0) {
        s_resp_opcode = op;
        s_resp_payload_len = plen;
        s_resp_len = 0;
        xSemaphoreGive(s_resp_sem);
    } else if (rc == -2) {
        ESP_LOGW(TAG, "notify decode error, resetting buffer");
        s_resp_len = 0;
    }
    /* rc == -1: incomplete, wait for more data */
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
        if (ble_uuid_cmp(&chr->uuid.u, &OXYII_WRITE_UUID.u) == 0)
            s_write_handle = chr->val_handle;
        else if (ble_uuid_cmp(&chr->uuid.u, &OXYII_NOTIFY_UUID.u) == 0)
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

/* ── Connect and discover GATT services ────────────────────────────── */
static esp_err_t connect_and_discover(const ble_addr_t *target)
{
    s_write_handle = s_notify_handle = s_cccd_handle = 0;
    s_svc_start = s_svc_end = 0;
    s_resp_len = 0;

    /* Connect — infer own address type based on peer address type
     * (random peer requires random own address). */
    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(target->type, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "addr infer failed: %d", rc);
        return ESP_FAIL;
    }
    clear_op_sem();
    rc = ble_gap_connect(own_addr_type, target,
                         15000, NULL, gap_event, NULL);
    if (rc != 0) { ESP_LOGE(TAG, "connect start failed: %d", rc); return ESP_FAIL; }
    if (xSemaphoreTake(s_conn_sem, pdMS_TO_TICKS(16000)) != pdTRUE) {
        ESP_LOGE(TAG, "connect timeout"); return ESP_FAIL;
    }
    if (s_conn_status != 0) {
        ESP_LOGE(TAG, "connect failed: %d", s_conn_status); return ESP_FAIL;
    }
    ESP_LOGI(TAG, "connected, handle=%d", s_conn_handle);

    /* MTU exchange */
    clear_op_sem();
    ble_gattc_exchange_mtu(s_conn_handle, on_mtu, NULL);
    wait_op(2000);

    /* Discover OxyII service by UUID */
    clear_op_sem();
    rc = ble_gattc_disc_svc_by_uuid(s_conn_handle, &OXYII_SVC_UUID.u,
                                     on_disc_svc, NULL);
    if (rc != 0 || wait_op(10000) != 0) {
        ESP_LOGE(TAG, "OxyII service not found"); return ESP_FAIL;
    }
    if (s_svc_start == 0) { ESP_LOGE(TAG, "service range empty"); return ESP_FAIL; }
    ESP_LOGI(TAG, "service: 0x%04x-0x%04x", s_svc_start, s_svc_end);

    /* Discover characteristics */
    clear_op_sem();
    rc = ble_gattc_disc_all_chrs(s_conn_handle, s_svc_start, s_svc_end,
                                 on_disc_chr, NULL);
    if (rc != 0 || wait_op(10000) != 0) {
        ESP_LOGE(TAG, "characteristic discovery failed"); return ESP_FAIL;
    }
    if (s_write_handle == 0 || s_notify_handle == 0) {
        ESP_LOGE(TAG, "write/notify char not found"); return ESP_FAIL;
    }
    ESP_LOGI(TAG, "write=%d notify=%d", s_write_handle, s_notify_handle);

    /* Discover CCCD for notify characteristic */
    clear_op_sem();
    rc = ble_gattc_disc_all_dscs(s_conn_handle, s_notify_handle, s_svc_end,
                                 on_disc_dsc, NULL);
    if (rc != 0 || wait_op(10000) != 0) {
        ESP_LOGE(TAG, "CCCD discovery failed"); return ESP_FAIL;
    }
    if (s_cccd_handle == 0) { ESP_LOGE(TAG, "CCCD not found"); return ESP_FAIL; }

    /* Enable notifications */
    uint8_t cccd_val[2] = { 0x01, 0x00 };
    clear_op_sem();
    rc = ble_gattc_write_flat(s_conn_handle, s_cccd_handle,
                              cccd_val, 2, on_write_done, NULL);
    if (rc != 0 || wait_op(5000) != 0) {
        ESP_LOGE(TAG, "enable notify failed"); return ESP_FAIL;
    }
    ESP_LOGI(TAG, "notifications enabled (cccd=%d)", s_cccd_handle);
    return ESP_OK;
}

static void do_disconnect(void)
{
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        xSemaphoreTake(s_conn_sem, pdMS_TO_TICKS(3000));
    }
}

/* ── Protocol request/response ─────────────────────────────────────── */
static esp_err_t oxyii_request(uint8_t op, const uint8_t *payload, int plen,
                                bool expect_reply, int timeout_ms)
{
    uint8_t frame[OXYII_MAX_FRAME];
    int flen = oxyii_encode(frame, sizeof(frame), op, 0, s_seq++,
                             payload, plen);
    if (flen < 0) return ESP_FAIL;

    if (expect_reply) {
        while (xSemaphoreTake(s_resp_sem, 0) == pdTRUE) { }
        s_resp_len = 0;
    }

    /* Use write-without-response (OxyII protocol uses WRITE_CMD, not WRITE_REQ).
     * This eliminates one BLE round-trip per request, roughly doubling throughput. */
    int rc = ble_gattc_write_no_rsp_flat(s_conn_handle, s_write_handle,
                                          frame, flen);
    if (rc != 0) {
        ESP_LOGE(TAG, "write failed op=0x%02x rc=%d", op, rc);
        return ESP_FAIL;
    }

    if (!expect_reply) return ESP_OK;

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (true) {
        TickType_t now = xTaskGetTickCount();
        if ((int32_t)(deadline - now) <= 0) {
            ESP_LOGE(TAG, "response timeout op=0x%02x", op);
            return ESP_ERR_TIMEOUT;
        }
        if (xSemaphoreTake(s_resp_sem, deadline - now) != pdTRUE) {
            ESP_LOGE(TAG, "response timeout op=0x%02x", op);
            return ESP_ERR_TIMEOUT;
        }
        if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE)
            return ESP_FAIL;
        if (s_resp_opcode == op)
            return ESP_OK;
        ESP_LOGW(TAG, "ignoring leftover notify op=0x%02x (want 0x%02x, %d B)",
                 s_resp_opcode, op, s_resp_payload_len);
    }
}

/* AUTH + SETUP only. Enough for GET_INFO / LIVE_B. Do not send F4
 * here — that closes a recording handle. Measured (SHQO2Pro 2D010003):
 * LIVE_B replies even with no AUTH; GET_INFO needs this prefix. */
static esp_err_t oxyii_session_open(void)
{
    s_seq = 0;
    uint8_t auth[16];
    oxyii_auth_payload(auth);
    if (oxyii_request(OP_AUTH, auth, 16, false, 0) != ESP_OK)
        return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(200));

    uint8_t setup = 0x00;
    if (oxyii_request(OP_SETUP, &setup, 1, true, 5000) != ESP_OK)
        return ESP_FAIL;
    return ESP_OK;
}

/* File-session prep. Only after LIVE_B says off-finger.
 * Documented order: SET_UTC → GET_CONFIG → F4. F4 clears a leftover
 * recording handle; without it, F1 is silently dropped (the F1 wedge). */
static esp_err_t oxyii_prepare_files(void)
{
    uint8_t tp[8];
    oxyii_time_payload(tp);
    if (oxyii_request(OP_SET_UTC_TIME, tp, 8, true, 5000) != ESP_OK)
        return ESP_FAIL;

    if (oxyii_request(OP_GET_CONFIG, NULL, 0, true, 5000) != ESP_OK)
        return ESP_FAIL;

    /* F4 acks. Must consume it or the next F1 sees this frame. */
    if (oxyii_request(OP_READ_FILE_END, NULL, 0, true, 2000) != ESP_OK)
        ESP_LOGW(TAG, "F4 ack missing — continuing");
    return ESP_OK;
}

/* ── GET_INFO: extract firmware + serial ───────────────────────────── */
static esp_err_t oxyii_get_info(char *serial, size_t serial_sz,
                                 char *firmware, size_t fw_sz)
{
    if (oxyii_request(OP_GET_INFO, NULL, 0, true, 5000) != ESP_OK)
        return ESP_FAIL;

    /* Payload layout (from protocol study):
     *   bytes 9..16: firmware (8 chars, null-padded)
     *   byte 37: serial length
     *   bytes 38..: serial string */
    if (s_resp_payload_len < 38) return ESP_FAIL;

    if (firmware && fw_sz > 0) {
        int fl = s_resp_payload_len - 9 < 8 ? s_resp_payload_len - 9 : 8;
        if (fl < 0) fl = 0;
        memcpy(firmware, s_resp_payload + 9, fl);
        firmware[fl] = '\0';
        /* Trim trailing nulls */
        for (int i = fl - 1; i >= 0 && firmware[i] == 0; i--)
            firmware[i] = '\0';
    }

    if (serial && serial_sz > 0) {
        int slen = s_resp_payload[37];
        if (slen > 0 && 38 + slen <= s_resp_payload_len) {
            int n = slen < (int)serial_sz - 1 ? slen : (int)serial_sz - 1;
            memcpy(serial, s_resp_payload + 38, n);
            serial[n] = '\0';
        } else {
            serial[0] = '\0';
        }
    }
    return ESP_OK;
}

/* LIVE_B contact. Returns:
 *   1  off-finger / END window (state 0x00) — pull is allowed
 *   0  on-finger or file handle open — do not pull
 *  -1  no usable reply — do not pull (fail closed)
 *
 * README: [5] 0x00 = no finger, 0x01 = finger present, 0x03 = file open. */
static int oxyii_off_finger(void)
{
    if (oxyii_request(OP_LIVE_B, NULL, 0, true, 2000) != ESP_OK)
        return -1;
    if (s_resp_payload_len < 9) return -1;
    uint8_t state = s_resp_payload[5];
    uint8_t spo2  = s_resp_payload[6];
    uint8_t hr    = s_resp_payload[8];
    ESP_LOGI(TAG, "live_b: state=0x%02x spo2=%u hr=%u", state, spo2, hr);

    /* 0x03 = leftover file handle from the ring's own recording
     * (README "F1 wedge": F1 silently hangs until F4). Closing it is
     * safe and lets END report its true contact state. */
    if (state == 0x03) {
        ESP_LOGI(TAG, "live_b: file handle open — clearing wedge with F4");
        oxyii_request(OP_READ_FILE_END, NULL, 0, true, 2000);
        if (oxyii_request(OP_LIVE_B, NULL, 0, true, 2000) != ESP_OK)
            return -1;
        if (s_resp_payload_len < 9) return -1;
        state = s_resp_payload[5];
        spo2  = s_resp_payload[6];
        hr    = s_resp_payload[8];
        ESP_LOGI(TAG, "live_b after F4: state=0x%02x spo2=%u hr=%u",
                 state, spo2, hr);
    }

    if (state == 0x00) return 1;
    return 0;
}

/* ── GET_FILE_LIST: return count + names ───────────────────────────── */
static int oxyii_get_file_list(char names[][17], int max_count)
{
    int rc = oxyii_request(OP_GET_FILE_LIST, NULL, 0, true, 5000);
    if (rc != ESP_OK) {
        /* Spec 4.7: F1 timeout → F4, retry once on this link, then fail. */
        ESP_LOGW(TAG, "F1 timed out (rc=%d) — F4 then retry once", rc);
        oxyii_request(OP_READ_FILE_END, NULL, 0, true, 2000);
        if (oxyii_request(OP_GET_FILE_LIST, NULL, 0, true, 5000) != ESP_OK)
            return -1;
    }

    if (s_resp_payload_len < 1) return -1;
    int count = s_resp_payload[0];
    if (count > max_count) count = max_count;

    int pos = 1;
    for (int i = 0; i < count; i++) {
        if (pos + 16 > s_resp_payload_len) break;
        memcpy(names[i], s_resp_payload + pos, 16);
        names[i][16] = '\0';
        /* Trim trailing nulls */
        for (int j = 15; j >= 0 && names[i][j] == 0; j--)
            names[i][j] = '\0';
        pos += 16;
    }
    return count;
}

/* ── Pull a single file ────────────────────────────────────────────── */
static esp_err_t oxyii_pull_file(const char *name)
{
    /* READ_FILE_START */
    uint8_t start_pl[20];
    oxyii_file_start_payload(start_pl, name);
    if (oxyii_request(OP_READ_FILE_START, start_pl, 20, true, 5000) != ESP_OK)
        return ESP_FAIL;

    uint32_t file_size = 0;
    if (s_resp_payload_len >= 4)
        file_size = s_resp_payload[0] | (s_resp_payload[1] << 8) |
                    (s_resp_payload[2] << 16) | (s_resp_payload[3] << 24);
    ESP_LOGI(TAG, "pulling '%s' (%lu bytes)", name, (unsigned long)file_size);

    /* Remove any stale .part file — always start fresh */
    ox_store_part_remove(name);

    /* READ_FILE_DATA loop */
    uint32_t offset = 0;
    int empty_count = 0;
    while (true) {
        uint8_t off_pl[4];
        oxyii_file_data_payload(off_pl, offset);
        if (oxyii_request(OP_READ_FILE_DATA, off_pl, 4, true, 10000) != ESP_OK) {
            ESP_LOGW(TAG, "file data timeout at offset=%lu", (unsigned long)offset);
            break;
        }

        /* Payload is the file data chunk */
        int chunk_len = s_resp_payload_len;
        if (chunk_len <= 0) {
            if (++empty_count > 2) break;
            continue;
        }
        empty_count = 0;

        if (ox_store_part_append(name, s_resp_payload, chunk_len) != ESP_OK) {
            ESP_LOGE(TAG, "SD write failed at offset=%lu", (unsigned long)offset);
            break;
        }
        offset += chunk_len;

        if (file_size > 0 && offset >= file_size) break;
    }

    /* READ_FILE_END */
    if (oxyii_request(OP_READ_FILE_END, NULL, 0, true, 2000) != ESP_OK)
        ESP_LOGW(TAG, "F4 ack missing after '%s'", name);

    /* Promote .part to .bin */
    bool finalised = ox_store_promote(s_dev_serial, name);
    ESP_LOGI(TAG, "pulled '%s': %lu bytes, finalised=%d",
             name, (unsigned long)offset, finalised);

    return ESP_OK;
}

/* ══ Backend interface implementation ════════════════════════════════ */

static void oxyii_init(void)
{
    if (s_op_sem) return;
    s_op_sem   = xSemaphoreCreateBinary();
    s_conn_sem = xSemaphoreCreateBinary();
    s_resp_sem = xSemaphoreCreateBinary();
    if (!s_op_sem || !s_conn_sem || !s_resp_sem)
        ESP_LOGE(TAG, "semaphore creation failed");
}

/* Advert scoring — see oximeter_backend.h for tier semantics. */
static int oxyii_adv_score(const char *name, uint16_t mfg_cid,
                           const uint8_t *raw_adv, int raw_len)
{
    (void)raw_adv; (void)raw_len;
    /* Worn/recording adverts are visible-only: never connect (a GATT
     * link would reset the ring's power-off timer mid-recording), but
     * report them so presence can show "worn · recording" instead of
     * pretending the ring vanished.  They still count as absence for
     * the connect-curfew model — that is what detects end-of-recording. */
    if (mfg_cid == MFG_RECORDING)
        return -1;
    if (name_is_oxyii(name))
        return 3;                       /* explicit device-name match */
    if (mfg_cid == MFG_OXYII)
        return 2;                       /* shared Viatom id heuristic */
    return 0;
}

static bool oxyii_identify(const ble_addr_t *addr,
                           char *serial, size_t serial_sz,
                           char *firmware, size_t fw_sz)
{
    if (serial_sz) serial[0] = '\0';
    if (fw_sz) firmware[0] = '\0';

    strlcpy(s_dev_serial, "", sizeof(s_dev_serial));

    if (connect_and_discover(addr) != ESP_OK) {
        do_disconnect();
        return false;
    }
    if (oxyii_session_open() != ESP_OK ||
        oxyii_get_info(serial, serial_sz, firmware, fw_sz) != ESP_OK ||
        serial[0] == '\0') {
        do_disconnect();
        return false;
    }
    strlcpy(s_dev_serial, serial, sizeof(s_dev_serial));
    do_disconnect();
    return true;
}

static ox_sync_err_t oxyii_sync(const ble_addr_t *addr,
                                const char *expect_serial,
                                bool download,
                                char *serial, size_t serial_sz,
                                int *files_pulled)
{
    if (files_pulled) *files_pulled = 0;
    if (serial_sz) serial[0] = '\0';

    if (connect_and_discover(addr) != ESP_OK) {
        do_disconnect();
        return OX_SYNC_ERR_CONNECT;
    }

    /* Probe only: AUTH+SETUP. Never F4 while we may still be recording. */
    if (oxyii_session_open() != ESP_OK) {
        ESP_LOGW(TAG, "session open failed");
        do_disconnect();
        return OX_SYNC_ERR_CONNECT;
    }

    char fw[16] = {0};
    char dev[32] = {0};
    if (oxyii_get_info(dev, sizeof(dev), fw, sizeof(fw)) != ESP_OK ||
        dev[0] == '\0') {
        ESP_LOGW(TAG, "get_info failed");
        do_disconnect();
        return OX_SYNC_ERR_INFO;
    }
    if (expect_serial && strcmp(dev, expect_serial) != 0) {
        ESP_LOGW(TAG, "serial mismatch (got '%s', want '%s')",
                 dev, expect_serial);
        do_disconnect();
        return OX_SYNC_ERR_IDENTITY;
    }
    if (serial_sz) strlcpy(serial, dev, serial_sz);
    strlcpy(s_dev_serial, dev, sizeof(s_dev_serial));

    /* Pull only in the documented no-contact END window
     * (LIVE_B [5] == 0x00). On-finger / unknown / F1-wedge: drop the
     * link immediately so the ring can keep recording or finish
     * countdown and sleep. NOT_READY ⇒ common layer backs off without
     * counting a failure. */
    if (download) {
        int off = oxyii_off_finger();
        if (off != 1) {
            ESP_LOGI(TAG, "not off-finger (live_b=%d)", off);
            do_disconnect();
            return OX_SYNC_NOT_READY;
        }

        if (oxyii_prepare_files() != ESP_OK) {
            ESP_LOGW(TAG, "file prep failed");
            do_disconnect();
            return OX_SYNC_ERR_TRANSFER;
        }

        char names[32][17];
        int count = oxyii_get_file_list(names, 32);
        if (count < 0) {
            ESP_LOGW(TAG, "file list failed");
            do_disconnect();
            return OX_SYNC_ERR_TRANSFER;
        }
        ESP_LOGI(TAG, "file list: %d files", count);

        for (int i = 0; i < count; i++) {
            if (names[i][0] == '\0') continue;

            int idx = ox_store_index_check(s_dev_serial, names[i]);
            if (idx == 1) {
                ESP_LOGD(TAG, "skip '%s' (already finalised)", names[i]);
                continue;
            }

            ESP_LOGI(TAG, "pulling file %d/%d: '%s'", i + 1, count, names[i]);
            if (oxyii_pull_file(names[i]) != ESP_OK) {
                ESP_LOGW(TAG, "pull failed for '%s'", names[i]);
                do_disconnect();
                return OX_SYNC_ERR_TRANSFER;
            }
            if (files_pulled) (*files_pulled)++;
        }
    }

    do_disconnect();
    return OX_SYNC_OK;
}

const oximeter_backend_t oximeter_backend_oxyii = {
    .proto_id              = OX_PROTO_OXYII,
    .init                  = oxyii_init,
    .adv_score             = oxyii_adv_score,
    .identify              = oxyii_identify,
    .sync                  = oxyii_sync,
    .report_status         = NULL,      /* nothing beyond base fields today */
    .max_consecutive_fails = 3,         /* mirrors the historical F1 retry cap */
};
