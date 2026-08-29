/*
 * SomnoTrace - Oximeter multi-driver dispatcher
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
 * The dispatcher loads the paired driver type from NVS at init and routes
 * public API calls to the appropriate driver (OxyII for Gen2, Legacy for
 * Gen1).  Both drivers are compiled in; only one is active at a time.
 * The scan function runs both drivers' scans and merges results so the
 * user can see all compatible rings in a single scan.
 */

#include "oximeter.h"
#include "oximeter_internal.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "nvs_writer.h"

#include <string.h>

static const char *TAG = "oximeter";

#define OX_NVS_NS "oximeter"

static const ox_driver_ops_t *s_active = &oxyii_driver_ops;
static ox_driver_t s_driver_type = OX_DRIVER_OXYII;
static bool s_both_inited = false;

/* Forward declarations for store functions */
bool ox_store_load_paired(char *serial, size_t serial_sz,
                          char *firmware, size_t fw_sz,
                          char *name_prefix, size_t prefix_sz,
                          char *last_addr, size_t addr_sz,
                          char *driver, size_t driver_sz);

static void load_driver_type(void)
{
    /* Try NVS first */
    nvs_handle_t h;
    nvs_writer_lock();
    if (nvs_open(OX_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        uint8_t drv = OX_DRIVER_OXYII;
        if (nvs_get_u8(h, "driver", &drv) == ESP_OK) {
            s_driver_type = (ox_driver_t)drv;
        }
        nvs_close(h);
    }
    nvs_writer_unlock();

    /* Fall back to paired.json on SD */
    if (s_driver_type == OX_DRIVER_OXYII) {
        char drv[16] = {0};
        if (ox_store_load_paired(NULL, 0, NULL, 0, NULL, 0, NULL, 0,
                                 drv, sizeof(drv))) {
            if (strcmp(drv, "wellue_legacy") == 0)
                s_driver_type = OX_DRIVER_LEGACY;
        }
    }

    s_active = (s_driver_type == OX_DRIVER_LEGACY)
        ? &legacy_driver_ops
        : &oxyii_driver_ops;

    ESP_LOGI(TAG, "active driver: %s",
             s_driver_type == OX_DRIVER_LEGACY ? "legacy" : "oxyii");
}

/* ── Public API — delegates to active driver ──────────────────────── */

esp_err_t oximeter_init(void)
{
    load_driver_type();
    /* Initialise both drivers so scan can run both, and pairing can
     * switch drivers without re-initialisation. */
    oxyii_driver_ops.init();
    legacy_driver_ops.init();
    s_both_inited = true;
    s_active->init();
    return ESP_OK;
}

/* Ensure both drivers are initialised (called on first scan) */
static void ensure_both_inited(void)
{
    if (s_both_inited) return;
    oxyii_driver_ops.init();
    legacy_driver_ops.init();
    s_both_inited = true;
}

/* Scan runs both drivers' scans and merges results so the user sees
 * all compatible rings (OxyII and Legacy) in a single scan. */
esp_err_t oximeter_scan(int timeout_sec)
{
    ensure_both_inited();

    esp_err_t err = ESP_OK;

    /* Run OxyII scan */
    err = oxyii_driver_ops.scan(timeout_sec);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "OxyII scan failed: %s", esp_err_to_name(err));
    }

    /* Run Legacy scan */
    err = legacy_driver_ops.scan(timeout_sec);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Legacy scan failed: %s", esp_err_to_name(err));
    }

    return ESP_OK;
}

/* Deduplicate scan results by BLE address. If same address appears in
 * both OxyII and Legacy results, prefer the one with explicit name match
 * (higher confidence). */
static void dedupe_scan_results(cJSON *merged, cJSON *src)
{
    if (!src) return;
    int n = cJSON_GetArraySize(src);
    for (int i = 0; i < n; i++) {
        cJSON *item = cJSON_GetArrayItem(src, i);
        cJSON *addr_item = cJSON_GetObjectItem(item, "addr");
        if (!cJSON_IsString(addr_item)) continue;
        const char *addr = addr_item->valuestring;

        /* Check if this address already exists in merged */
        bool exists = false;
        int m = cJSON_GetArraySize(merged);
        for (int j = 0; j < m; j++) {
            cJSON *existing = cJSON_GetArrayItem(merged, j);
            cJSON *existing_addr = cJSON_GetObjectItem(existing, "addr");
            if (cJSON_IsString(existing_addr) &&
                strcmp(existing_addr->valuestring, addr) == 0) {
                exists = true;
                break;
            }
        }
        if (!exists) {
            cJSON_AddItemToArray(merged, cJSON_Duplicate(item, 1));
        }
    }
}

cJSON *oximeter_get_scan_results(void)
{
    ensure_both_inited();

    cJSON *oxyii_results = oxyii_driver_ops.get_scan_results();
    cJSON *legacy_results = legacy_driver_ops.get_scan_results();

    cJSON *merged = cJSON_CreateArray();

    /* Add Legacy results first. Both drivers match on mfg_id 0xF34E,
     * so Legacy devices appear in both scans. By adding Legacy first,
     * we ensure Legacy protocol is used for pairing (OxyII only matches
     * on explicit name prefixes S8-AW/SHQO2Pro for true Gen2 devices). */
    dedupe_scan_results(merged, legacy_results);
    if (legacy_results) cJSON_Delete(legacy_results);

    /* Add OxyII results, skipping duplicates by address */
    dedupe_scan_results(merged, oxyii_results);
    if (oxyii_results) cJSON_Delete(oxyii_results);

    return merged;
}

esp_err_t oximeter_pair(const char *addr_str, ox_driver_t driver)
{
    /* If pairing with a different driver type, switch active driver */
    if (driver != s_driver_type) {
        /* Forget the current driver's state */
        s_active->forget();

        /* Switch driver */
        s_driver_type = driver;
        s_active = (driver == OX_DRIVER_LEGACY)
            ? &legacy_driver_ops
            : &oxyii_driver_ops;

        /* Initialize the new driver if not already done */
        s_active->init();

        ESP_LOGI(TAG, "switched to driver: %s",
                 driver == OX_DRIVER_LEGACY ? "legacy" : "oxyii");
    }

    return s_active->pair(addr_str);
}

esp_err_t oximeter_forget(void)
{
    s_active->forget();
    return ESP_OK;
}

const char *oximeter_get_status(void)
{
    const char *st = s_active->get_status();
    /* Safeguard: if paired but driver reports idle, report paired */
    if (strcmp(st, OX_STATUS_IDLE) == 0 && s_active->is_paired())
        return OX_STATUS_PAIRED;
    return st;
}

const char *oximeter_get_error(void)
{
    return s_active->get_error();
}

bool oximeter_is_paired(void)
{
    return s_active->is_paired();
}

cJSON *oximeter_get_paired_info(void)
{
    cJSON *info = s_active->get_paired_info();
    if (info) {
        /* Add protocol field so frontend knows which protocol */
        cJSON_AddStringToObject(info, "protocol", s_active == &legacy_driver_ops ? "legacy" : "oxyii");
    }
    return info;
}

ox_driver_t oximeter_get_driver(void)
{
    return s_driver_type;
}

ox_probe_mode_t oximeter_get_probe_mode(void)
{
    return s_active->get_probe_mode();
}

esp_err_t oximeter_set_probe_mode(ox_probe_mode_t mode)
{
    return s_active->set_probe_mode(mode);
}

const char *oximeter_get_scanned_name(const char *addr_str)
{
    if (!addr_str || !addr_str[0]) return NULL;
    const char *name = NULL;
    if (s_active == &legacy_driver_ops)
        name = legacy_get_scanned_name(addr_str);
    else if (s_active == &oxyii_driver_ops)
        name = oxyii_get_scanned_name(addr_str);
    return name;
}
