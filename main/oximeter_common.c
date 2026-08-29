/*
 * SomnoTrace - Common oximeter layer (scanning, pairing, sync watch)
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

/* Protocol-agnostic oximeter orchestration.  Everything here works through
 * the oximeter_backend_t interface (oximeter_backend.h); per-protocol
 * details live exclusively in the backend files.
 *
 *   ┌──────────── common (this file) ────────────┐
 *   │ passive watch scans + advert classification│
 *   │ pairing flow + NVS/paired.json persistence │
 *   │ sync-window state machine + sleep curfew   │
 *   │ public API + /api/status composition       │
 *   └──────┬───────────────────────────┬─────────┘
 *          ▼                           ▼
 *   oxyii backend                legacy backend
 * (oximeter_oxyii.c)         (oximeter_legacy.c)
 *
 * See ADD_LEGACY_OXIMETER.md for the full architecture and byte-level
 * protocol reference. */

#include "oximeter.h"
#include "oximeter_backend.h"
#include "oximeter_store.h"
#include "sd_storage.h"
#include "as11_ble.h"
#include "psram_task.h"
#include "nvs_writer.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "host/ble_hs.h"
#include "host/util/util.h"

static const char *TAG = "oximeter";

/* ── Backend registry ────────────────────────────────────────────────────
 * Add new oximeter families here (and nowhere else). */
static const oximeter_backend_t *const s_backends[] = {
    &oximeter_backend_oxyii,
    &oximeter_backend_legacy,
};
#define OX_BACKEND_COUNT (sizeof(s_backends) / sizeof(s_backends[0]))

int  ox_backend_count(void) { return (int)OX_BACKEND_COUNT; }
const oximeter_backend_t *ox_backend_at(int i)
{
    return (i >= 0 && i < (int)OX_BACKEND_COUNT) ? s_backends[i] : NULL;
}

/* ── Tunables ────────────────────────────────────────────────────────────
 * Sleep guarantee: after a served session, NO connections happen while
 * the ring stays visible — a docked/charging ring advertises forever, so
 * only true radio-silence (slept/worn/out-of-range) or a long timeout
 * reopens the sync window. */
#define OX_SCAN_MAX            16
#define OX_ABSENT_STRIKES      4    /* consecutive empty scans = gone (~60s) */
#define OX_ROTATE_ADOPT_STRIKES 10  /* family-hit w/ hint silent ≈ rotated MAC */
#define OX_SERVED_REPROBE_MS   (30ul * 60 * 1000) /* visibility safety valve */
#define OX_NOT_READY_BACKOFF_MS 60000 /* worn/not-ready retry delay */

/* ── Module state ─────────────────────────────────────────────────────── */
struct ox_scan_result {
    ble_addr_t addr;
    char name[32];
    int rssi;
    uint16_t mfg;
    const char *proto;      /* OX_PROTO_* — which backend owns this device */
};

static SemaphoreHandle_t s_state_mtx;
static SemaphoreHandle_t s_ops_mtx;     /* serialise BLE ops (scan/pair/pull) */
static SemaphoreHandle_t s_scan_done;

static char s_status[24] = OX_STATUS_IDLE;
static char s_error[128];

/* Paired device info (loaded from NVS at init) */
static char s_serial[32];
static char s_firmware[16];
static char s_name_prefix[16];
static char s_paired_addr[18];
static const char *s_protocol = OX_PROTO_OXYII;
static bool s_paired = false;
static bool s_presence_served = false;
static bool s_ring_present = false;
static TickType_t s_served_at;
static time_t s_last_sync_time = 0; /* epoch of last completed sync, 0=never */
static int s_last_pulled = -1;      /* recordings pulled in that sync */
static time_t s_last_seen_time = 0; /* epoch of last watch-scan match, 0=never */
static int s_absent_streak = 0;     /* consecutive scans with no paired-proto hit */
static int s_fail_count[OX_BACKEND_COUNT]; /* consecutive failed syncs, per backend */

/* Worn/recording visibility (OxyII recording-mode adverts): tracked in
 * parallel to s_ring_present — the ring is *on air* but is never a
 * connect candidate while that advert lasts. */
static bool s_rec_adv_seen;         /* recording advert received this scan */
static bool s_ring_recording = false;
static int s_rec_absent_streak = 0; /* consecutive scans with no recording advert */

/* Address-hint tracking: s_paired_addr (string) parsed into s_hint_addr.
 * Once the stored address has been observed on air this boot, same-family
 * adverts from OTHER addresses no longer count as presence — a nearby
 * foreign device (Viatom/Wellue share mfg 0xF34E) must not fake it. */
static ble_addr_t s_hint_addr;
static bool s_hint_valid = false;       /* s_paired_addr parses cleanly */
static bool s_hint_confirmed = false;   /* hint seen advertising this boot */
static int s_stranger_streak = 0;       /* family hits while hint silent */

/* Unclassified adverts heard during the current scan (field-diagnosis
 * tally — summarised at INFO when the scan completes). */
struct ox_unclass {
    ble_addr_t addr;
    char name[16];
    uint16_t cid;
    int count;
    int best_rssi;
};
#define OX_UNCLASS_MAX 8
static struct ox_unclass s_unclass[OX_UNCLASS_MAX];
static int s_unclass_count;

/* Scan state */
static struct ox_scan_result s_scan[OX_SCAN_MAX];
static int s_scan_count;
static volatile uint32_t s_adv_seen;    /* raw adverts received this scan */

/* ── Small helpers ─────────────────────────────────────────────────────── */
static void set_state(const char *st)
{
    xSemaphoreTake(s_state_mtx, portMAX_DELAY);
    bool changed = strcmp(s_status, st) != 0;
    strlcpy(s_status, st, sizeof(s_status));
    xSemaphoreGive(s_state_mtx);
    if (changed)
        ESP_LOGI(TAG, "state -> %s", st);
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

static bool addr_eq(const ble_addr_t *a, const ble_addr_t *b)
{
    return a->type == b->type &&
           memcmp(a->val, b->val, sizeof(a->val)) == 0;
}

/* Re-parse the string hint after any change to s_paired_addr. */
static void refresh_addr_hint(void)
{
    s_hint_valid = s_paired_addr[0] != '\0' &&
                   parse_addr(s_paired_addr, &s_hint_addr);
    if (!s_hint_valid)
        s_hint_confirmed = false;
}

static int backend_index_by_proto(const char *proto)
{
    if (!proto) return -1;
    for (int i = 0; i < (int)OX_BACKEND_COUNT; i++)
        if (strcmp(s_backends[i]->proto_id, proto) == 0) return i;
    return -1;
}

static const char *proto_from_str(const char *s)
{
    if (!s || s[0] == '\0') return OX_PROTO_OXYII;
    /* "o2ring" = records written before the backend was renamed. */
    if (strcmp(s, OX_PROTO_LEGACY) == 0 || strcmp(s, "o2ring") == 0)
        return OX_PROTO_LEGACY;
    return OX_PROTO_OXYII;
}

/* ── Scan GAP event handler (shared by all backends) ───────────────────── */
static int gap_event(struct ble_gap_event *event, void *arg);

/* Tally one unrecognised advert for the scan summary. */
static void unclass_tally(const struct ble_gap_event *event,
                          const char *name, uint16_t cid)
{
    for (int i = 0; i < s_unclass_count; i++) {
        if (memcmp(&s_unclass[i].addr, &event->disc.addr,
                   sizeof(ble_addr_t)) == 0) {
            s_unclass[i].count++;
            if (event->disc.rssi > s_unclass[i].best_rssi)
                s_unclass[i].best_rssi = event->disc.rssi;
            if (name[0])
                strlcpy(s_unclass[i].name, name, sizeof(s_unclass[i].name));
            return;
        }
    }
    if (s_unclass_count >= OX_UNCLASS_MAX) return;
    struct ox_unclass *u = &s_unclass[s_unclass_count++];
    memset(u, 0, sizeof(*u));
    u->addr = event->disc.addr;
    u->cid = cid;
    u->count = 1;
    u->best_rssi = event->disc.rssi;
    strlcpy(u->name, name[0] ? name : "-", sizeof(u->name));
}

static void handle_scan_adv(struct ble_gap_event *event)
{
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

    /* Classify: highest adv_score across all backends wins.  Negative
     * scores mark visible-only adverts (worn/recording) that must never
     * become connect candidates. */
    int best_score = 0;
    bool rec_adv = false;
    const oximeter_backend_t *best = NULL;
    for (int i = 0; i < (int)OX_BACKEND_COUNT; i++) {
        int sc = s_backends[i]->adv_score(name, cid, raw, raw_len);
        if (sc < 0) {
            rec_adv = true;
        } else if (sc > best_score) {
            best_score = sc;
            best = s_backends[i];
        }
    }
    if (rec_adv) {
        s_rec_adv_seen = true;
        ESP_LOGD(TAG, "scan: recording-mode advert '%s' rssi=%d addr=%s mfg=0x%04x",
                 name[0] ? name : "-", event->disc.rssi, addr_str, cid);
        return;
    }
    if (!best) {
        /* Not an oximeter we handle — DEBUG keeps the full RF picture,
         * a compact tally feeds the INFO scan summary. */
        ESP_LOGD(TAG, "scan: ignoring '%s' rssi=%d addr=%s mfg=0x%04x",
                 name[0] ? name : "-", event->disc.rssi, addr_str, cid);
        unclass_tally(event, name, cid);
        return;
    }
    ESP_LOGD(TAG, "scan: classified '%s' -> %s (score %d, mfg=0x%04x)",
             name[0] ? name : "-", best->proto_id, best_score, cid);

    if (name[0] == '\0')
        strlcpy(name, best->proto_id, sizeof(name));

    /* Dedupe by address */
    for (int i = 0; i < s_scan_count; i++) {
        if (memcmp(&s_scan[i].addr, &event->disc.addr,
                   sizeof(ble_addr_t)) == 0) {
            s_scan[i].rssi = event->disc.rssi;
            s_scan[i].mfg = cid;
            s_scan[i].proto = best->proto_id;
            strlcpy(s_scan[i].name, name, sizeof(s_scan[i].name));
            return;
        }
    }
    if (s_scan_count < OX_SCAN_MAX) {
        s_scan[s_scan_count].addr = event->disc.addr;
        strlcpy(s_scan[s_scan_count].name, name,
                sizeof(s_scan[s_scan_count].name));
        s_scan[s_scan_count].rssi = event->disc.rssi;
        s_scan[s_scan_count].mfg = cid;
        s_scan[s_scan_count].proto = best->proto_id;
        s_scan_count++;
        ESP_LOGI(TAG, "scan: '%s' rssi=%d addr=%s mfg=0x%04x proto=%s",
                 name, event->disc.rssi, addr_str, cid, best->proto_id);
    }
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        handle_scan_adv(event);
        return 0;
    case BLE_GAP_EVENT_DISC_COMPLETE:
        xSemaphoreGive(s_scan_done);
        return 0;
    default:
        return 0;
    }
}

/* ── Scan (caller holds s_ops_mtx) ───────────────────────────────────────
 * passive: low-duty listen-only for the background watch.  active:
 * full-duty with scan requests — needed to pull names/UUIDs from scan
 * responses when classifying a device for pairing.
 * Note: NimBLE runs a single discovery procedure host-wide.  An AS11
 * scan started while this runs — or vice versa — makes the loser fail
 * with BLE_HS_EBUSY; the modules share the radio but not a mutex. */
static esp_err_t do_scan(int timeout_sec, bool active)
{
    s_scan_count = 0;
    s_adv_seen = 0;
    s_rec_adv_seen = false;
    s_unclass_count = 0;

    struct ble_gap_disc_params dp = {
        .itvl = active ? 96 : 160,
        .window = active ? 96 : 48,   /* active = pairing duty (96/96) */
        .filter_policy = 0,
        .limited = 0,
        .passive = !active,           /* watch is listen-only */
    };
    ESP_LOGI(TAG, "%s scan (%ds): start",
             active ? "active" : "passive", timeout_sec);

    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(BLE_ADDR_RANDOM, &own_addr_type);
    if (rc != 0) own_addr_type = as11_ble_get_own_addr_type();

    TickType_t t0 = xTaskGetTickCount();
    while (xSemaphoreTake(s_scan_done, 0) == pdTRUE) { }

    rc = ble_gap_disc(own_addr_type,
                      timeout_sec * 1000, &dp, gap_event, NULL);
    if (rc != 0) {
        if (rc == BLE_HS_EBUSY)
            ESP_LOGW(TAG, "scan start failed: BUSY (AS11 scan running? "
                          "only one BLE discovery at a time)");
        else
            ESP_LOGW(TAG, "scan start failed: %d", rc);
        return ESP_FAIL;
    }

    bool done = xSemaphoreTake(s_scan_done,
                               pdMS_TO_TICKS((timeout_sec + 2) * 1000)) == pdTRUE;
    ESP_LOGI(TAG, "%s scan done after %lus: %d matched device(s), "
             "%lu raw advert(s)%s",
             active ? "active" : "passive",
             (unsigned long)((xTaskGetTickCount() - t0) / configTICK_RATE_HZ),
             s_scan_count, (unsigned long)s_adv_seen,
             done ? "" : " (timeout)");

    /* Compact RF picture at INFO: everything heard but not recognised.
     * Makes field diagnosis ("my ring is right here!") possible without
     * raising the log level. */
    if (s_unclass_count) {
        char buf[192];
        int off = 0;
        for (int i = 0; i < s_unclass_count; i++) {
            int n = snprintf(buf + off, sizeof(buf) - off, "%s%s/0x%04x/%ddB/x%d",
                             i ? ", " : "",
                             s_unclass[i].name[0] ? s_unclass[i].name : "-",
                             s_unclass[i].cid, s_unclass[i].best_rssi,
                             s_unclass[i].count);
            if (n < 0 || off + n >= (int)sizeof(buf) - 1) break;
            off += n;
        }
        ESP_LOGI(TAG, "%s scan: %d unrecognised device(s): %s",
                 active ? "active" : "passive", s_unclass_count, buf);
    }
    return ESP_OK;
}

static const char *scan_proto_for_addr(const char *addr_str)
{
    for (int i = 0; i < s_scan_count; i++) {
        char a[18];
        addr_to_str(&s_scan[i].addr, a, sizeof(a));
        if (strcmp(a, addr_str) == 0 && s_scan[i].proto)
            return s_scan[i].proto;
    }
    return NULL;
}

/* ── NVS persistence ───────────────────────────────────────────────────── */
#define OX_NVS_NS "oximeter"

struct ox_nvs_arg {
    char serial[32];
    char firmware[16];
    char name_prefix[16];
    char last_addr[18];
    char protocol[12];
};

static esp_err_t do_save_nvs(void *arg)
{
    struct ox_nvs_arg *a = arg;
    nvs_handle_t h;
    esp_err_t e = nvs_open(OX_NVS_NS, NVS_READWRITE, &h);
    if (e != ESP_OK) return e;
    nvs_set_str(h, "serial", a->serial);
    nvs_set_str(h, "firmware", a->firmware);
    nvs_set_str(h, "name_prefix", a->name_prefix);
    nvs_set_str(h, "last_addr", a->last_addr);
    nvs_set_str(h, "protocol",
                a->protocol[0] ? a->protocol : OX_PROTO_OXYII);
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
    nvs_erase_key(h, "protocol");
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

static void load_paired_from_nvs(void)
{
    nvs_handle_t h;
    if (nvs_open(OX_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        size_t len;
        char serial[32] = {0}, fw[16] = {0}, prefix[16] = {0},
             addr[18] = {0}, proto[12] = {0};

        size_t len = sizeof(serial);
        if (nvs_get_str(h, "serial", serial, &len) == ESP_OK && serial[0]) {
            len = sizeof(fw);
            nvs_get_str(h, "firmware", fw, &len);
            len = sizeof(prefix);
            nvs_get_str(h, "name_prefix", prefix, &len);
            len = sizeof(addr);
            nvs_get_str(h, "last_addr", addr, &len);
            len = sizeof(proto);
            nvs_get_str(h, "protocol", proto, &len);

            strlcpy(s_serial, serial, sizeof(s_serial));
            strlcpy(s_firmware, fw, sizeof(s_firmware));
            strlcpy(s_name_prefix, prefix, sizeof(s_name_prefix));
            strlcpy(s_paired_addr, addr, sizeof(s_paired_addr));
            s_protocol = proto_from_str(proto);
            s_paired = true;
        }
        nvs_close(h);
    }

    /* Also try loading from paired.json (SD) as fallback */
    if (!s_paired) {
        char serial[32], fw[16], prefix[16], addr[18], proto[12] = {0};
        if (ox_store_load_paired(serial, sizeof(serial),
                                 fw, sizeof(fw),
                                 prefix, sizeof(prefix),
                                 addr, sizeof(addr),
                                 proto, sizeof(proto))) {
            strlcpy(s_serial, serial, sizeof(s_serial));
            strlcpy(s_firmware, fw, sizeof(s_firmware));
            strlcpy(s_name_prefix, prefix, sizeof(s_name_prefix));
            strlcpy(s_paired_addr, addr, sizeof(s_paired_addr));
            s_protocol = proto_from_str(proto);
            s_paired = true;
        }
    }
    refresh_addr_hint();
}

/* ── Pair task ─────────────────────────────────────────────────────────── */
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

    /* Protocol comes from how this address advertised during the last
     * scan (the UI always scans before listing).  The background watch
     * rebuilds that cache every cycle, so the entry may have been
     * evicted between listing and tapping Pair — re-scan actively once
     * before giving up.  Unknown addresses fall back to the first
     * registered backend. */
    const char *proto = scan_proto_for_addr(addr_str);
    if (!proto) {
        ESP_LOGI(TAG, "pair: '%s' not in scan cache — rescanning", addr_str);
        do_scan(4, true);
        proto = scan_proto_for_addr(addr_str);
        ESP_LOGI(TAG, "rescan found %d device(s)", s_scan_count);
    }
    int bi = backend_index_by_proto(proto);
    if (bi < 0) bi = 0;
    const oximeter_backend_t *B = s_backends[bi];
    ESP_LOGI(TAG, "pairing %s device at %s", B->proto_id, addr_str);

    char serial[32] = {0};
    char firmware[16] = {0};
    if (!B->identify(&target, serial, sizeof(serial),
                     firmware, sizeof(firmware)) || serial[0] == '\0') {
        set_error("%s identify failed (%s)", B->proto_id,
                  esp_err_to_name(ESP_FAIL));
        free(addr_str);
        xSemaphoreGive(s_ops_mtx);
        vTaskDelete(NULL);
        return;
    }

    char prefix[5];
    snprintf(prefix, sizeof(prefix), "%.4s", serial);

    struct ox_nvs_arg nvs_arg;
    memset(&nvs_arg, 0, sizeof(nvs_arg));
    strlcpy(nvs_arg.serial, serial, sizeof(nvs_arg.serial));
    strlcpy(nvs_arg.firmware, firmware, sizeof(nvs_arg.firmware));
    strlcpy(nvs_arg.name_prefix, prefix, sizeof(nvs_arg.name_prefix));
    strlcpy(nvs_arg.last_addr, addr_str, sizeof(nvs_arg.last_addr));
    strlcpy(nvs_arg.protocol, B->proto_id, sizeof(nvs_arg.protocol));
    nvs_writer_run(do_save_nvs, &nvs_arg);

    ox_store_save_paired(serial, firmware, prefix, addr_str, B->proto_id);

    strlcpy(s_serial, serial, sizeof(s_serial));
    strlcpy(s_firmware, firmware, sizeof(s_firmware));
    strlcpy(s_name_prefix, prefix, sizeof(s_name_prefix));
    strlcpy(s_paired_addr, addr_str, sizeof(s_paired_addr));
    refresh_addr_hint();
    s_hint_confirmed = false;
    s_protocol = B->proto_id;
    s_paired = true;
    s_presence_served = false;

    set_state(OX_STATUS_PAIRED);
    ESP_LOGI(TAG, "paired: protocol=%s serial=%s fw=%s",
             B->proto_id, serial, firmware);

    free(addr_str);
    xSemaphoreGive(s_ops_mtx);
    vTaskDelete(NULL);
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

        if (do_scan(4, false) != ESP_OK) {
            xSemaphoreGive(s_ops_mtx);
            continue;
        }

        /* Recording-mode visibility (OxyII recording-mode adverts): tracked in
         * parallel to s_ring_present — the ring is *on air* but is never a
         * connect candidate while that advert lasts. */
        if (s_rec_adv_seen) {
            s_rec_absent_streak = 0;
            if (!s_ring_recording) {
                ESP_LOGI(TAG, "ring worn — recording in progress "
                             "(recording-mode advert)");
                s_ring_recording = true;
            }
        } else if (s_ring_recording &&
                   ++s_rec_absent_streak >= OX_ABSENT_STRIKES) {
            s_ring_recording = false;
        }

        /* Pick the scan hit for the paired ring.  An exact-address match
         * wins; same-family adverts from other addresses are only
         * trusted while we have never confirmed the stored address on
         * air (fresh boot), so a nearby foreign device (Viatom/Wellue share
         * mfg 0xF34E) cannot fake presence.  A persistent family-only signal
         * later on is treated as MAC rotation (factory reset) and adopted. */
        int ti = -1;
        int hint_i = -1;
        int fam_i = -1;
        for (int i = 0; i < s_scan_count; i++) {
            if (!s_scan[i].proto ||
                strcmp(s_scan[i].proto, s_protocol) != 0)
                continue;
            if (s_hint_valid && addr_eq(&s_scan[i].addr, &s_hint_addr))
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
                         s_stranger_streak, OX_ROTATE_ADOPT_STRIKES);
                if (s_stranger_streak >= OX_ROTATE_ADOPT_STRIKES) {
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
                         s_absent_streak, OX_ABSENT_STRIKES);
            else
                ESP_LOGD(TAG, "watch: no hit for paired protocol '%s' "
                         "(absent streak %d/%d)",
                         s_protocol, s_absent_streak, OX_ABSENT_STRIKES);
            if (s_absent_streak >= OX_ABSENT_STRIKES) {
                if (s_ring_present)
                    ESP_LOGI(TAG, "ring gone — next appearance is a new sync window");
                s_ring_present = false;
                if (s_presence_served)
                    s_presence_served = false;
                memset(s_fail_count, 0, sizeof(s_fail_count));
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
        refresh_addr_hint();
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
            TickType_t served_for = xTaskGetTickCount() - s_served_at;
            if (served_for < pdMS_TO_TICKS(OX_SERVED_REPROBE_MS)) {
                ESP_LOGD(TAG, "watch: served %lus ago, ring visible — curfew, no reconnect",
                         (unsigned long)(served_for / configTICK_RATE_HZ));
                xSemaphoreGive(s_ops_mtx);
                continue;
            }
            ESP_LOGI(TAG, "watch: ring visible %ld min since last sync — scheduled re-probe",
                     (long)(served_for / pdMS_TO_TICKS(60000)));
            s_presence_served = false;
            memset(s_fail_count, 0, sizeof(s_fail_count));
        }

        set_state(OX_STATUS_CONNECTING);

        /* Sync window open: hand the whole session to the paired
         * protocol's backend (connect → identify → download →
         * disconnect). */
        int bi = backend_index_by_proto(s_protocol);
        if (bi < 0) bi = 0;
        const oximeter_backend_t *B = s_backends[bi];

        char serial[32] = {0};
        int pulled = 0;
        SemaphoreHandle_t ble_mtx = as11_ble_get_ble_mutex();
        if (ble_mtx) xSemaphoreTake(ble_mtx, portMAX_DELAY);

        ox_sync_err_t r = B->sync(&s_scan[ti].addr, s_serial, true,
                                  serial, sizeof(serial), &pulled);

        if (ble_mtx) xSemaphoreGive(ble_mtx);

        if (r == OX_SYNC_OK) {
            s_fail_count[bi] = 0;
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
            ESP_LOGI(TAG, "device not ready (%s) — retry in %ds",
                     B->proto_id, OX_NOT_READY_BACKOFF_MS / 1000);
            set_state(OX_STATUS_PAIRED);
            xSemaphoreGive(s_ops_mtx);
            vTaskDelay(pdMS_TO_TICKS(OX_NOT_READY_BACKOFF_MS));
            continue;
        } else if (++s_fail_count[bi] >= B->max_consecutive_fails &&
                   B->max_consecutive_fails > 0) {
            s_presence_served = true;
            s_served_at = xTaskGetTickCount();
            ESP_LOGW(TAG, "%s unreachable after %d failed attempts — treating window as served until next appearance",
                     B->proto_id, s_fail_count[bi]);
        } else {
            ESP_LOGW(TAG, "%s sync failed (r=%d), attempt %d/%d",
                     B->proto_id, r, s_fail_count[bi],
                     B->max_consecutive_fails);
        }
        set_state(OX_STATUS_PAIRED);
        xSemaphoreGive(s_ops_mtx);
    }
}

/* ── Public API ────────────────────────────────────────────────────────── */
esp_err_t oximeter_init(void)
{
    s_state_mtx = xSemaphoreCreateMutex();
    s_ops_mtx   = xSemaphoreCreateMutex();
    s_scan_done = xSemaphoreCreateBinary();
    if (!s_state_mtx || !s_ops_mtx || !s_scan_done)
        return ESP_ERR_NO_MEM;

    for (int i = 0; i < (int)OX_BACKEND_COUNT; i++)
        s_backends[i]->init();

    load_paired_from_nvs();
    ox_store_ensure_dirs();

    if (s_paired)
        set_state(OX_STATUS_PAIRED);

    TaskHandle_t h = psram_task_create(pull_task, "ox_pull", 8192, NULL, 3,
                                       tskNO_AFFINITY, NULL, NULL);
    if (!h) {
        ESP_LOGW(TAG, "failed to create pull task");
    }
    return ESP_OK;
}

esp_err_t oximeter_scan(int timeout_sec)
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
        /* Transient radio contention (AS11 discovery owns the host) —
         * do NOT poison module state with a sticky error; report busy
         * so the UI can offer a friendly retry. */
        ESP_LOGW(TAG, "scan start failed: %d (radio busy?)", rc);
        xSemaphoreGive(s_ops_mtx);
        if (s_paired)
            set_state(OX_STATUS_PAIRED);
        else
            set_state(OX_STATUS_IDLE);
        return ESP_ERR_INVALID_STATE;
    }

    /* Returns early when oximeter_scan_cancel() terminates discovery. */
    xSemaphoreTake(s_scan_done, pdMS_TO_TICKS((timeout_sec + 2) * 1000));

    if (s_paired)
        set_state(OX_STATUS_PAIRED);
    else
        set_state(OX_STATUS_IDLE);

    xSemaphoreGive(s_ops_mtx);
    return ESP_OK;
}

void oximeter_scan_cancel(void)
{
    /* Firing DISC_COMPLETE unblocks the pending oximeter_scan() wait;
     * results collected so far are kept.  Harmless if none is active. */
    ble_gap_disc_cancel();
}

cJSON *oximeter_get_scan_results(void)
{
    cJSON *arr = cJSON_CreateArray();
    for (int i = 0; i < s_scan_count; i++) {
        cJSON *e = cJSON_CreateObject();
        char addr_str[18];
        addr_to_str(&s_scan[i].addr, addr_str, sizeof(addr_str));
        cJSON_AddStringToObject(e, "addr", addr_str);
        cJSON_AddStringToObject(e, "name", s_scan[i].name);
        cJSON_AddNumberToObject(e, "rssi", s_scan[i].rssi);
        if (s_scan[i].proto)
            cJSON_AddStringToObject(e, "proto", s_scan[i].proto);
        cJSON_AddItemToArray(arr, e);
    }
    return arr;
}

esp_err_t oximeter_pair(const char *addr_str)
{
    if (!as11_ble_is_host_ready())
        return ESP_ERR_INVALID_STATE;
    char *addr_copy = strdup(addr_str);
    if (!addr_copy) return ESP_ERR_NO_MEM;

    TaskHandle_t h = psram_task_create(pair_task, "ox_pair", 8192, addr_copy, 5,
                                       tskNO_AFFINITY, NULL, NULL);
    if (!h) {
        free(addr_copy);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void oximeter_forget(void)
{
    s_paired = false;
    s_presence_served = false;
    s_ring_recording = false;
    s_rec_absent_streak = 0;
    s_serial[0] = '\0';
    s_firmware[0] = '\0';
    s_name_prefix[0] = '\0';
    s_paired_addr[0] = '\0';
    refresh_addr_hint();
    s_stranger_streak = 0;
    s_protocol = OX_PROTO_OXYII;

    nvs_writer_run(do_erase_nvs, NULL);
    ox_store_delete_paired();

    set_state(OX_STATUS_IDLE);
    return ESP_OK;
}

const char *oximeter_get_status(void)
{
    return s_status;
}

const char *oximeter_get_error(void)
{
    return s_error;
}

bool oximeter_is_paired(void)
{
    return s_paired;
}

const char *oximeter_get_presence(void)
{
    if (strcmp(s_status, OX_STATUS_CONNECTING) == 0 ||
        strcmp(s_status, OX_STATUS_PULLING) == 0)
        return OX_PRESENCE_SYNCING;
    if (s_ring_present)
        /* Visible with a sync window still open = ready to transfer; once
         * a window has been served the curfew leaves it merely "on hand". */
        return s_presence_served ? OX_PRESENCE_DETECTED : OX_PRESENCE_READY;
    if (s_ring_recording)
        /* On-air but worn/recording — visible, never connectable. */
        return OX_PRESENCE_RECORDING;
    return OX_PRESENCE_OFFLINE;
}

cJSON *oximeter_get_paired_info(void)
{
    if (!s_paired) return NULL;
    cJSON *info = cJSON_CreateObject();
    cJSON_AddStringToObject(info, "serial", s_serial);
    if (s_firmware[0]) cJSON_AddStringToObject(info, "firmware", s_firmware);
    if (s_name_prefix[0]) cJSON_AddStringToObject(info, "name_prefix", s_name_prefix);
    if (s_paired_addr[0]) cJSON_AddStringToObject(info, "addr", s_paired_addr);
    cJSON_AddStringToObject(info, "protocol", s_protocol);

    /* Backend-specific extras (battery/model/recordings-on-ring/...). */
    int bi = backend_index_by_proto(s_protocol);
    if (bi >= 0 && s_backends[bi]->report_status)
        s_backends[bi]->report_status(info);

    if (s_last_sync_time > 0) {
        cJSON_AddNumberToObject(info, "last_sync", (double)s_last_sync_time);
        cJSON_AddNumberToObject(info, "last_pulled", s_last_pulled);
    }
    if (s_last_seen_time > 0)
        cJSON_AddNumberToObject(info, "last_seen", (double)s_last_seen_time);
    return info;
}

const char *oximeter_get_driver_for_addr(const char *addr_str)
{
    for (int i = 0; i < s_scan_count; i++) {
        char a[18];
        addr_to_str(&s_scan[i].addr, a, sizeof(a));
        if (strcmp(a, addr_str) == 0 && s_scan[i].proto)
            return s_scan[i].proto;
    }
    return NULL;
}

ox_probe_mode_t oximeter_get_probe_mode(void)
{
    int bi = backend_index_by_proto(s_protocol);
    if (bi >= 0 && s_backends[bi]->report_status)
        return s_backends[bi]->report_status ? OX_PROBE_PERSISTENT : OX_PROBE_LEGACY;
    return OX_PROBE_LEGACY;
}

esp_err_t oximeter_set_probe_mode(ox_probe_mode_t mode)
{
    int bi = backend_index_by_proto(s_protocol);
    if (bi >= 0 && s_backends[bi]->report_status)
        return s_backends[bi]->report_status ? ESP_OK : ESP_ERR_NOT_SUPPORTED;
    return ESP_ERR_NOT_SUPPORTED;
}