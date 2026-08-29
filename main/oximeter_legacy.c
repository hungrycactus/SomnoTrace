/*
 * SomnoTrace - O2 Ring (Legacy/Gen1) BLE protocol codec and session
 * Copyright (C) 2026 Ilya Kruchinin <https://github.com/ilyakruchinin>
 *
 * This file is part of SomnoTrace.
 *
 * SomnoTrace is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
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
 * Clean-room Legacy BLE protocol for Wellue O2 Ring (Gen1) / ViaTom rings.
 * Protocol studied from published documentation; no third-party source
 * code copied.
 */

#include "oximeter.h"
#include "oximeter_internal.h"
#include "oximeter_store.h"
#include "sd_storage.h"
#include "as11_ble.h"
#include "psram_task.h"
#include "nvs_writer.h"
#include "oximetry_canonical.h"
#include "time_sync.h"
#include "upload_sched.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

static const char *TAG = "ox_legacy";

/* ── Store forward declarations (oximeter_store.c) ─────────────────── */
void ox_store_ensure_dirs(void);
bool ox_store_load_paired(char *serial, size_t serial_sz,
                          char *firmware, size_t fw_sz,
                          char *name_prefix, size_t prefix_sz,
                          char *last_addr, size_t addr_sz,
                          char *driver, size_t driver_sz);
void ox_store_save_paired(const char *serial, const char *firmware,
                          const char *name_prefix, const char *last_addr,
                          const char *driver);
void ox_store_delete_paired(void);
int  ox_store_index_check(const char *serial, const char *name);
void ox_store_index_add(const char *serial, const char *name,
                        uint32_t bytes, bool finalised);
long ox_store_part_size(const char *name);
esp_err_t ox_store_part_append(const char *name, const uint8_t *data, size_t len);
bool ox_store_promote_vld3(const char *serial, const char *name);
void ox_store_part_remove(const char *name);

/* ── Legacy protocol constants ──────────────────────────────────────── */
#define LEGACY_REQ_LEAD       0xAA
#define LEGACY_RSP_LEAD       0x55
#define LEGACY_HDR_LEN        7          /* AA/CMD/^CMD/BLOCK(2)/LEN(2) */
#define LEGACY_MAX_FRAME      1024
#define LEGACY_MAX_PAYLOAD    (LEGACY_MAX_FRAME - LEGACY_HDR_LEN - 1)
#define LEGACY_WRITE_CHUNK    20         /* legacy ATT payload limit */

/* Command codes */
#define CMD_FILE_OPEN       0x03
#define CMD_FILE_READ       0x04
#define CMD_FILE_CLOSE      0x05
#define CMD_INFO            0x14
#define CMD_PING            0x15
#define CMD_CONFIG          0x16
#define CMD_READ_SENSORS    0x17
#define CMD_RT_DATA         0x1B

/* VLD3 file format constants */
#define VLD3_HEADER_LEN     40
#define VLD3_RECORD_LEN     5
#define VLD3_NO_FINGER      0xFF

/* Legacy GATT UUIDs (128-bit, stored little-endian for NimBLE).
 * Service:  14839ac4-7d7e-415c-9a42-167340cf2339
 * Write:    8b00ace7-eb0b-49b0-bbe9-9aee0a26e1a3
 * Notify:   0734594a-a8e7-4b1a-a6b1-cd5243059a57 */
static const ble_uuid128_t LEGACY_SVC_UUID =
    BLE_UUID128_INIT(0x39, 0x23, 0xcf, 0x40, 0x73, 0x16, 0x42, 0x9a,
                     0x5c, 0x41, 0x7e, 0x7d, 0xc4, 0x9a, 0x83, 0x14);
static const ble_uuid128_t LEGACY_WRITE_UUID =
    BLE_UUID128_INIT(0xa3, 0xe1, 0x26, 0x0a, 0xee, 0x9a, 0xe9, 0xbb,
                     0xb0, 0x49, 0x0b, 0xeb, 0xe7, 0xac, 0x00, 0x8b);
static const ble_uuid128_t LEGACY_NOTIFY_UUID =
    BLE_UUID128_INIT(0x57, 0x9a, 0x05, 0x43, 0x52, 0xcd, 0xb1, 0xa6,
                     0x1a, 0x4b, 0xe7, 0xa8, 0x4a, 0x59, 0x34, 0x07);

/* ── CRC8 (poly=0x07, init=0) — same as OxyII ──────────────────────── */
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

/* Forward declarations */
static bool legacy_adv_service_match(const uint8_t *adv_data, int adv_len);
static void send_close_best_effort(void);

/* ── Module state ──────────────────────────────────────────────────── */
#define OX_SCAN_MAX 16

struct ox_scan_result {
    ble_addr_t addr;
    char name[32];
    int rssi;
    uint16_t mfg;
    const char *proto;      /* OX_PROTO_* — which backend owns this device */
};

static SemaphoreHandle_t s_state_mtx;
static SemaphoreHandle_t s_ops_mtx;
static SemaphoreHandle_t s_op_sem;
static SemaphoreHandle_t s_conn_sem;
static SemaphoreHandle_t s_resp_sem;
static SemaphoreHandle_t s_scan_done;
static volatile int s_op_status;
static volatile int s_conn_status;

static char s_status[24] = OX_STATUS_IDLE;
static char s_error[128];

static char s_serial[32];
static char s_firmware[16];
static char s_name_prefix[16];
static char s_paired_addr[18];
static bool s_paired = false;
static bool s_presence_served = false;
static bool s_ring_present = false;
static TickType_t s_served_at;

/* BLE connection state */
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_write_handle;
static uint16_t s_notify_handle;
static uint16_t s_cccd_handle;
static uint16_t s_svc_start, s_svc_end;

/* Scan state */
static struct ox_scan_result *s_scan;
static int s_scan_count;
static volatile uint32_t s_adv_seen;    /* raw adverts received this scan */

/* Notification accumulation buffer */
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

/* Worn/recording visibility tracking */
static bool s_rec_adv_seen;
static bool s_ring_recording = false;
static int s_rec_absent_streak = 0;

/* Address-hint tracking */
static ble_addr_t s_hint_addr;
static bool s_hint_valid = false;
static bool s_hint_confirmed = false;
static int s_stranger_streak = 0;

/* Absence tracking */
static int s_absent_streak = 0;

/* Failure tracking */
static int s_fail_count = 0;

/* Sync tracking */
static time_t s_last_sync_time = 0;
static int s_last_pulled = -1;
static time_t s_last_seen_time = 0;

/* ── Helpers ───────────────────────────────────────────────────────── */
static void set_state(const char *st)
{
    xSemaphoreTake(s_state_mtx, portMAX_DELAY);
    strlcpy(s_status, st, sizeof(s_status));
    xSemaphoreGive(s_state_mtx);
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
    if (len <= 0 || len > 31) return false;
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

static const char *sync_err_str(int r)
{
    switch (r) {
    case 0: return "ok";
    case 1: return "connect-failed";
    case 2: return "info-failed";
    case 3: return "serial-mismatch";
    case 4: return "transfer-failed";
    case 5: return "not-ready";
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

/* Check if a BLE advert name matches a Gen1 O2 Ring / ViaTom device.
 * Match against known name fragments (case-insensitive substring). */
static bool name_is_legacy(const char *name)
{
    if (!name || !name[0]) return false;
    char up[32];
    int i;
    for (i = 0; i < 31 && name[i]; i++)
        up[i] = toupper((unsigned char)name[i]);
    up[i] = '\0';

    static const char *const LEGACY_NAME_KEYS[] = {
        "O2RING", "CHECKME_O2", "CHECKO2", "SLEEPU", "SLEEPO2",
        "WEARO2", "KIDSO2", "BABYO2", "OXYLINK",
    };

    for (size_t k = 0; k < sizeof(LEGACY_NAME_KEYS) / sizeof(LEGACY_NAME_KEYS[0]); k++)
        if (strstr(up, LEGACY_NAME_KEYS[k])) return true;
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
    /* Static random addresses have the top two bits of the most-significant
     * byte set (0xC0 mask).  The rings use static random addresses. */
    out->type = (v[0] & 0xC0) == 0xC0 ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
    return true;
}

/* ── GAP event handler ─────────────────────────────────────────────── */
static int gap_event(struct ble_gap_event *event, void *arg);

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

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        char addr_str[18];
        addr_to_str(&event->disc.addr, addr_str, sizeof(addr_str));
        s_adv_seen++;

        /* Parse name from adv data (with manual fallback) */
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
                } else if (ad_type == 0xFF && ad_data_len >= 2) {
                    f.mfg_data = ad_data;
                    f.mfg_data_len = ad_data_len;
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

        uint16_t cid = 0;
        if (f.mfg_data && f.mfg_data_len >= 2)
            cid = f.mfg_data[0] | (f.mfg_data[1] << 8);

        /* Classify: name match (tier 3) or service UUID in adv (tier 1) */
        bool match = name_is_legacy(name);
        bool svc_match = legacy_adv_service_match(raw, raw_len);

        if (!match && !svc_match) return 0;
        if (name[0] == '\0')
            strlcpy(name, "O2Ring", sizeof(name));

        /* Dedupe by address */
        for (int i = 0; i < s_scan_count; i++) {
            if (memcmp(&s_scan[i].addr, &event->disc.addr,
                       sizeof(ble_addr_t)) == 0) {
                s_scan[i].rssi = event->disc.rssi;
                s_scan[i].mfg = cid;
                if (name[0])
                    strlcpy(s_scan[i].name, name, sizeof(s_scan[i].name));
                s_scan[i].proto = "legacy";
                return 0;
            }
        }
        if (s_scan_count < OX_SCAN_MAX) {
            s_scan[s_scan_count].addr = event->disc.addr;
            strlcpy(s_scan[s_scan_count].name, name,
                    sizeof(s_scan[s_scan_count].name));
            s_scan[s_scan_count].rssi = event->disc.rssi;
            s_scan[s_scan_count].mfg = cid;
            s_scan[s_scan_count].proto = "legacy";
            s_scan_count++;
            ESP_LOGI(TAG, "scan: '%s' rssi=%d addr=%s mfg=0x%04x",
                     name, event->disc.rssi, addr_str, cid);
        }
        return 0;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
        xSemaphoreGive(s_scan_done);
        return 0;

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

/* On-air (little-endian) service UUID for advertisement matching. */
static const uint8_t LEGACY_SVC_ADV_LE[16] = {
    0x39, 0x23, 0xcf, 0x40, 0x73, 0x16, 0x42, 0x9a,
    0x5c, 0x41, 0x7e, 0x7d, 0xc4, 0x9a, 0x83, 0x14,
};

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
                         15000, NULL, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "connect start failed: %d", rc);
        return ESP_FAIL;
    }
    if (xSemaphoreTake(s_conn_sem, pdMS_TO_TICKS(15000 + 1000)) != pdTRUE) {
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
    if (rc != 0 || wait_op(10000) != 0) {
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
    if (rc != 0 || wait_op(10000) != 0) {
        ESP_LOGE(TAG, "characteristic discovery failed");
        return ESP_FAIL;
    }
    if (s_write_handle == 0 || s_notify_handle == 0) {
        ESP_LOGE(TAG, "write/notify characteristic not found");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "write=%d notify=%d", s_write_handle, s_notify_handle);

    /* Discover CCCD for notify characteristic */
    clear_op_sem();
    rc = ble_gattc_disc_all_dscs(s_conn_handle, s_notify_handle, s_svc_end,
                                 on_disc_dsc, NULL);
    if (rc != 0 || wait_op(10000) != 0) {
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
    if (rc != 0 || wait_op(5000) != 0) {
        ESP_LOGE(TAG, "enable notify failed");
        return ESP_FAIL;
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
        if (wait_op(5000) != 0) {
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
    char files[16][32];
    int nfiles;
};

static esp_err_t parse_file_list(const char *flist, struct legacy_info *out)
{
    out->nfiles = 0;
    if (!flist) return ESP_OK;                  /* empty list is valid  */

    while (*flist && out->nfiles < 16) {
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

    if (legacy_request(0x14, 0, NULL, 0, 8000) != ESP_OK)
        return ESP_FAIL;
    if (s_rsp_status != 0) {
        ESP_LOGE(TAG, "INFO status=%u", s_rsp_status);
        return ESP_FAIL;
    }

    int jlen = s_rsp_len;
    if (jlen <= 0 || jlen > 1024 ||
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
        char list[(32 + 3) * 4 + 24];   /* first few names */
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
    uint8_t open_pl[32];
    memcpy(open_pl, name, namelen);
    open_pl[namelen] = '\0';
    if (legacy_request(0x03, 0, open_pl, namelen + 1,
                    5000) != ESP_OK)
        return ESP_FAIL;
    if (s_rsp_status != 0) {
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
    if (size == 0 || size > (8u << 20)) {
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
        if (legacy_request(0x04, block, NULL, 0,
                        10000) != ESP_OK) {
            if (++errs >= 3) {
                ESP_LOGE(TAG, "FILE_READ block=%u failed %d times",
                         block, errs);
                goto fail;
            }
            continue;                           /* retry same block */
        }
        if (s_rsp_status != 0) {
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
            if (++errs >= 3) {
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

static void send_close_best_effort(void)
{
    if (legacy_request(0x05, 0, NULL, 0, 3000) == ESP_OK &&
        s_rsp_status == 0) {
        ESP_LOGD(TAG, "close ack ok");
        return;
    }
    ESP_LOGW(TAG, "FILE_CLOSE ack missing — continuing");
}

/* ── NVS persistence ───────────────────────────────────────────────── */
#define OX_NVS_NS "oximeter"

struct ox_nvs_arg {
    char serial[32];
    char firmware[16];
    char name_prefix[16];
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

    len = sizeof(s_serial);
    if (nvs_get_str(h, "serial", s_serial, &len) == ESP_OK && s_serial[0]) {
        /* Check driver type — only load if this is a Legacy device */
        uint8_t drv = OX_DRIVER_OXYII;
        nvs_get_u8(h, "driver", &drv);
        if (drv != OX_DRIVER_LEGACY) {
            /* Not our device — don't load */
            s_serial[0] = '\0';
            nvs_close(h);
            nvs_writer_unlock();
            return;
        }
        s_paired = true;
        len = sizeof(s_firmware);
        nvs_get_str(h, "firmware", s_firmware, &len);
        len = sizeof(s_name_prefix);
        nvs_get_str(h, "name_prefix", s_name_prefix, &len);
        len = sizeof(s_paired_addr);
        nvs_get_str(h, "last_addr", s_paired_addr, &len);
    }
    uint8_t pm;
    if (nvs_get_u8(h, "probe_mode", &pm) == ESP_OK && pm <= 1)
        s_probe_mode = (ox_probe_mode_t)pm;
    nvs_close(h);
    nvs_writer_unlock();

    /* Also try loading from paired.json (SD) as fallback */
    if (!s_paired) {
        char serial[32], fw[16], prefix[16], addr[18], drv[16];
        if (ox_store_load_paired(serial, sizeof(serial),
                                 fw, sizeof(fw),
                                 prefix, sizeof(prefix),
                                 addr, sizeof(addr),
                                 drv, sizeof(drv))) {
            if (strcmp(drv, "wellue_legacy") == 0) {
                strlcpy(s_serial, serial, sizeof(s_serial));
                strlcpy(s_firmware, fw, sizeof(s_firmware));
                strlcpy(s_name_prefix, prefix, sizeof(s_name_prefix));
                strlcpy(s_paired_addr, addr, sizeof(s_paired_addr));
                s_paired = true;
            }
        }
    }
}

/* ── Pair task: identify-only session (no downloads) ───────────────── */
static void pair_task(void *arg)
{
    char *addr_str = (char *)arg;
    ble_addr_t target;

    xSemaphoreTake(s_ops_mtx, portMAX_DELAY);
    set_state(OX_STATUS_CONNECTING);

    if (!parse_addr(addr_str, &target)) {
        set_error("invalid address: %s", addr_str);
        free(addr_str);
        xSemaphoreGive(s_ops_mtx);
        vTaskDelete(NULL);
        return;
    }

    if (connect_and_discover(&target) != ESP_OK) {
        do_disconnect();
        free(addr_str);
        xSemaphoreGive(s_ops_mtx);
        vTaskDelete(NULL);
        return;
    }

    struct legacy_info info;
    if (legacy_get_info(&info) != ESP_OK || info.sn[0] == '\0') {
        set_error("get_info failed");
        do_disconnect();
        free(addr_str);
        xSemaphoreGive(s_ops_mtx);
        vTaskDelete(NULL);
        return;
    }

    char prefix[5];
    snprintf(prefix, sizeof(prefix), "%.4s", info.sn);

    /* Look up scanned name from current scan results for display */
    const char *scanned_name = legacy_get_scanned_name(addr_str);

    struct ox_nvs_arg nvs_arg;
    memset(&nvs_arg, 0, sizeof(nvs_arg));
    strlcpy(nvs_arg.serial, info.sn, sizeof(nvs_arg.serial));
    strlcpy(nvs_arg.firmware, info.model, sizeof(nvs_arg.firmware));
    strlcpy(nvs_arg.name_prefix, prefix, sizeof(nvs_arg.name_prefix));
    strlcpy(nvs_arg.last_addr, addr_str, sizeof(nvs_arg.last_addr));
    if (scanned_name) {
        strlcpy(nvs_arg.name_prefix, scanned_name, sizeof(nvs_arg.name_prefix));
    }
    nvs_writer_run(do_save_nvs, &nvs_arg);

    ox_store_save_paired(info.sn, info.model, scanned_name ? scanned_name : prefix, addr_str, "wellue_legacy");

    strlcpy(s_serial, info.sn, sizeof(s_serial));
    strlcpy(s_firmware, info.model, sizeof(s_firmware));
    if (scanned_name) {
        strlcpy(s_name_prefix, scanned_name, sizeof(s_name_prefix));
    } else {
        strlcpy(s_name_prefix, prefix, sizeof(s_name_prefix));
    }
    strlcpy(s_paired_addr, addr_str, sizeof(s_paired_addr));
    s_paired = true;
    s_presence_served = false;

    do_disconnect();
    set_state(OX_STATUS_PAIRED);
    ESP_LOGI(TAG, "paired: serial=%s model=%s", info.sn, info.model);

    free(addr_str);
    xSemaphoreGive(s_ops_mtx);
    vTaskDelete(NULL);
}

/* ── Low-duty scan (caller holds s_ops_mtx) ─────────────────────────── */
static esp_err_t do_scan(int timeout_sec)
{
    s_scan_count = 0;
    s_adv_seen = 0;

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

static void canonical_migration_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(35000));
    while (sd_storage_recording_active())
        vTaskDelay(pdMS_TO_TICKS(5000));
    if (sd_storage_is_ready()) {
        oximetry_canonical_reconcile();
        oximetry_canonical_migrate_all_legacy();
    }
    vTaskDelete(NULL);
}

/* ── Public: full sync session ─────────────────────────────────────── */
typedef enum {
    OX_SYNC_OK = 0,
    OX_SYNC_ERR_CONNECT,
    OX_SYNC_ERR_INFO,
    OX_SYNC_ERR_IDENTITY,
    OX_SYNC_ERR_TRANSFER,
    OX_SYNC_NOT_READY,
} ox_sync_err_t;

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

    if (!s_op_sem || !s_conn_sem || !s_resp_sem) {
        ESP_LOGE(TAG, "session begin on uninitialised backend — call legacy_init() first");
        return OX_SYNC_ERR_CONNECT;
    }

    ESP_LOGI(TAG, "session begin addr=%s expect=%s download=%d",
             taddr, expect_serial ? expect_serial : "*", download_files);

    if (connect_and_discover(addr) != ESP_OK) {
        do_disconnect();
        ESP_LOGI(TAG, "session end: %s", "connect-failed");
        return OX_SYNC_ERR_CONNECT;
    }

    struct legacy_info info;
    if (legacy_get_info(&info) != ESP_OK || info.sn[0] == '\0') {
        ESP_LOGW(TAG, "INFO unavailable — ring may be worn/recording");
        do_disconnect();
        ESP_LOGI(TAG, "session end: %s", "info-failed");
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
        ESP_LOGI(TAG, "session end: %s", "serial-mismatch");
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
                break;
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
                 "transfer-failed", pulled);
    return result;
}

/* ── File pull helper (legacy session API) ─────────────────────────── */
static bool do_pull_and_mark(bool *pulled_any)
{
    if (pulled_any) *pulled_any = false;
    set_state(OX_STATUS_PULLING);

    char serial[32] = {0};
    int pulled = 0;
    ox_sync_err_t r = legacy_sync_session(&s_scan[0].addr, s_serial, true,
                                          serial, sizeof(serial), &pulled);
    if (pulled_any) *pulled_any = (pulled > 0);
    return r == OX_SYNC_OK;
}

/* ── Background watch: scan-only, connect only in a fresh sync window ─── */
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

        /* Recording-mode visibility (OxyII worn advert), debounced with
         * the same strike counter as absence.  Purely informational —
         * recording adverts never open a sync window. */
        if (s_rec_adv_seen) {
            s_rec_absent_streak = 0;
            if (!s_ring_recording) {
                ESP_LOGI(TAG, "ring worn — recording in progress "
                               "(recording-mode advert)");
                s_ring_recording = true;
            }
        } else if (s_ring_recording &&
                   ++s_rec_absent_streak >= 4) {
            s_ring_recording = false;
        }

        /* Pick the scan hit for the paired ring.  An exact-address match
         * wins; same-family adverts from other addresses are only
         * trusted while we have never confirmed the stored address on
         * air (fresh boot), so a foreign same-brand device cannot fake
         * presence.  A persistent family-only signal later on is treated
         * as MAC rotation (factory reset) and adopted. */
        int ti = -1;
        int hint_i = -1;
        int fam_i = -1;
        for (int i = 0; i < s_scan_count; i++) {
            if (!s_scan[i].proto ||
                strcmp(s_scan[i].proto, "legacy") != 0)
                continue;
            if (s_hint_valid && memcmp(&s_scan[i].addr, &s_hint_addr,
                       sizeof(ble_addr_t)) == 0)
                hint_i = i;
            else if (fam_i < 0)
                fam_i = i;
        }
        if (hint_i >= 0) {
            ti = hint_i;
            s_stranger_streak = 0;
            if (!s_hint_confirmed)
                ESP_LOGI(TAG, "paired ring address %s seen on air",
                         s_paired_addr);
            s_hint_confirmed = true;
        } else if (fam_i >= 0) {
            if (!s_hint_valid || !s_hint_confirmed) {
                ti = fam_i;             /* adopt until confirmed */
            } else {
                s_stranger_streak++;
                ESP_LOGI(TAG, "same-family advert from unknown address "
                         "while paired ring silent (%d/%d)",
                         s_stranger_streak, 10);
                if (s_stranger_streak >= 10) {
                    ti = fam_i;         /* assume rotated MAC — adopt */
                    s_stranger_streak = 0;
                    ESP_LOGW(TAG, "adopting rotated ring address");
                }
            }
        }
        for (int i = 0; i < s_scan_count; i++) {
            char a[18];
            addr_to_str(&s_scan[i].addr, a, sizeof(a));
            ESP_LOGD(TAG, "watch: hit[%d] '%s' rssi=%d addr=%s proto=%s",
                     i, s_scan[i].name[0] ? s_scan[i].name : "-",
                     s_scan[i].rssi, a,
                     s_scan[i].proto ? s_scan[i].proto : "-");
        }

        if (ti < 0) {
            /* Passive scans miss adverts under Wi-Fi coexistence —
             * require several consecutive empty scans before declaring
             * the ring gone, otherwise unlucky misses would break the
             * post-sync silence and keep resetting the ring's sleep
             * timer. */
            s_absent_streak++;
            if (s_ring_present)
                ESP_LOGI(TAG, "watch: paired ring not seen (absent %d/%d)",
                         s_absent_streak, 4);
            else
                ESP_LOGD(TAG, "watch: no hit for paired protocol '%s' "
                         "(absent streak %d/%d)",
                         "legacy", s_absent_streak, 4);
            if (s_absent_streak >= 4) {
                if (s_ring_present)
                    ESP_LOGI(TAG, "ring gone — next appearance is a new sync window");
                s_ring_present = false;
                if (s_presence_served)
                    s_presence_served = false;
            }
            xSemaphoreGive(s_ops_mtx);
            continue;
        }
        s_absent_streak = 0;

        if (!s_ring_present) {
            ESP_LOGI(TAG, "ring present: '%s' rssi=%d",
                     s_scan[ti].name, s_scan[ti].rssi);
            s_ring_present = true;
        }

        /* Remember last seen addr (hint; MAC can rotate on factory reset). */
        addr_to_str(&s_scan[ti].addr, s_paired_addr, sizeof(s_paired_addr));
        s_hint_valid = true;
        s_hint_confirmed = true;
        s_last_seen_time = time(NULL);

        if (s_presence_served) {
            /* Post-sync connection curfew.  While the ring stays visible
             * it is left completely alone so its own power-off timer can
             * finally run out — a docked/charging ring advertises
             * indefinitely and must not be re-probed into staying awake.
             * The window reopens only when visibility breaks for
             * OX_ABSENT_STRIKES scans (handled above), or after a long
             * safety timeout in case a recording ever happens while the
             * ring remains visible. */
            if ((xTaskGetTickCount() - s_served_at) < pdMS_TO_TICKS(1800000)) {
                ESP_LOGD(TAG, "watch: served %lus ago, ring visible — curfew, no reconnect",
                         (unsigned long)((xTaskGetTickCount() - s_served_at) / configTICK_RATE_HZ));
                xSemaphoreGive(s_ops_mtx);
                continue;
            }
            ESP_LOGI(TAG, "watch: ring visible %ld min since last sync — scheduled re-probe",
                     (long)((xTaskGetTickCount() - s_served_at) / pdMS_TO_TICKS(60000)));
            s_presence_served = false;
        }

        set_state(OX_STATUS_CONNECTING);

        /* Sync window open: hand the whole session to the paired
         * protocol's backend (connect → identify → download →
         * disconnect). */
        char serial[32] = {0};
        int pulled = 0;
        ox_sync_err_t r = legacy_sync_session(&s_scan[ti].addr, s_serial, true,
                                              serial, sizeof(serial), &pulled);

        if (r == OX_SYNC_OK) {
            s_presence_served = true;
            s_served_at = xTaskGetTickCount();
            s_last_sync_time = time(NULL);
            s_last_pulled = pulled;
            ESP_LOGI(TAG, "sync window served (%d file(s) pulled) — curfew until ring disappears",
                     pulled);
        } else if (r == OX_SYNC_NOT_READY) {
            /* Reachable but busy/worn — back off without counting a
             * failure so a genuinely broken device still trips the
             * fail valve eventually. */
            ESP_LOGI(TAG, "device not ready (legacy) — retry in %ds",
                     60000 / 1000);
            set_state(OX_STATUS_PAIRED);
            xSemaphoreGive(s_ops_mtx);
            vTaskDelay(pdMS_TO_TICKS(60000));
            continue;
        } else if (++s_fail_count >= 3) {
            s_presence_served = true;
            s_served_at = xTaskGetTickCount();
            ESP_LOGW(TAG, "legacy unreachable after %d failed attempts — treating window as served until next appearance",
                     s_fail_count);
        } else {
            ESP_LOGW(TAG, "legacy sync failed (r=%d), attempt %d/3",
                     r, s_fail_count);
        }
        set_state(OX_STATUS_PAIRED);
        xSemaphoreGive(s_ops_mtx);
    }
}

/* ── Public API (driver vtable) ────────────────────────────────────── */
static void legacy_init(void)
{
    if (!s_state_mtx) s_state_mtx = xSemaphoreCreateMutex();
    if (!s_ops_mtx)   s_ops_mtx   = xSemaphoreCreateMutex();
    if (!s_op_sem)    s_op_sem    = xSemaphoreCreateBinary();
    if (!s_conn_sem)  s_conn_sem  = xSemaphoreCreateBinary();
    if (!s_resp_sem)  s_resp_sem  = xSemaphoreCreateBinary();
    if (!s_scan_done) s_scan_done = xSemaphoreCreateBinary();
    if (!s_state_mtx || !s_ops_mtx || !s_op_sem || !s_conn_sem ||
        !s_resp_sem || !s_scan_done)
        return;

    if (!s_scan) {
        s_scan = heap_caps_malloc(sizeof(struct ox_scan_result) * OX_SCAN_MAX,
                                  MALLOC_CAP_SPIRAM);
        if (!s_scan) {
            ESP_LOGE(TAG, "init: failed to allocate PSRAM buffers");
            return;
        }
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
    TaskHandle_t migration = psram_task_create(canonical_migration_task,
                                               "ox_leg_mig", 12288, NULL, 1,
                                               0, NULL, NULL);
    if (!migration) ESP_LOGW(TAG, "failed to create canonical migration task");
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

    char *addr_copy = strdup(addr_str);
    if (!addr_copy) return ESP_ERR_NO_MEM;

    TaskHandle_t h = psram_task_create(pair_task, "ox_leg_pair", 8192, addr_copy, 5,
                                       tskNO_AFFINITY, NULL, NULL);
    if (!h) {
        free(addr_copy);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static void legacy_forget(void)
{
    s_paired = false;
    s_presence_served = false;
    s_serial[0] = '\0';
    s_firmware[0] = '\0';
    s_name_prefix[0] = '\0';
    s_paired_addr[0] = '\0';

    nvs_writer_run(do_erase_nvs, NULL);
    ox_store_delete_paired();

    set_state(OX_STATUS_IDLE);
}

static const char *legacy_get_status(void)
{
    return s_status;
}

/* Get the scanned device name for a given address. */
const char *legacy_get_scanned_name(const char *addr_str)
{
    if (!addr_str || !addr_str[0]) return NULL;
    for (int i = 0; i < s_scan_count; i++) {
        char a[18];
        addr_to_str(&s_scan[i].addr, a, sizeof(a));
        if (strcmp(a, addr_str) == 0 && s_scan[i].name[0]) {
            return s_scan[i].name;
        }
    }
    return NULL;
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
    if (s_name_prefix[0]) cJSON_AddStringToObject(info, "scanned_name", s_name_prefix);
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
};
