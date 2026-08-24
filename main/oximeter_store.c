/*
 * SomnoTrace - Oximetry recording storage (SD card)
 *
 * Shared by both protocol backends.  Recordings land in
 * oximetry/inbox/<name>.part as they stream in, then are promoted:
 *   - OxyII (Format A)  → files/<serial>/<name>.bin, accepted only if
 *     the Format-A trailer magic is present at size - 44.
 *   - Legacy (.vld)     → files/<serial>/<name> verbatim, accepted only
 *     if the byte count equals the size declared by FILE_OPEN.
 * index.json records every promotion for duplicate suppression; see
 * ADD_LEGACY_OXIMETER.md for the full layout.
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

#include "oximeter.h"
#include "sd_storage.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/stat.h>
#include <dirent.h>

#include "esp_log.h"
#include "cJSON.h"

static const char *TAG = "ox_store";

#define OXY_BASE       SD_OXYMETRY_DIR
#define OXY_INBOX      OXY_BASE "/inbox"
#define OXY_FILES      OXY_BASE "/files"
#define OXY_PAIRED_JSON OXY_BASE "/paired.json"
#define OXY_INDEX_JSON  OXY_BASE "/index.json"

/* Trailer magic at file_size - 44 (offset 4 within the 48-byte trailer). */
static const uint8_t TRAILER_MAGIC[4] = { 0x48, 0x12, 0x5A, 0xDA };
#define TRAILER_LEN  48

/* ── Directory helpers ─────────────────────────────────────────────── */

void ox_store_ensure_dirs(void)
{
    mkdir(OXY_BASE, 0775);
    mkdir(OXY_INBOX, 0775);
    mkdir(OXY_FILES, 0775);
}

/* ── paired.json ──────────────────────────────────────────────────── */

bool ox_store_load_paired(char *serial, size_t serial_sz,
                          char *firmware, size_t fw_sz,
                          char *name_prefix, size_t prefix_sz,
                          char *last_addr, size_t addr_sz,
                          char *protocol, size_t protocol_sz)
{
    FILE *f = fopen(OXY_PAIRED_JSON, "r");
    if (!f) return false;

    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > 1024) { fclose(f); return false; }

    char *buf = malloc(sz + 1);
    if (!buf) { fclose(f); return false; }
    int n = fread(buf, 1, sz, f);
    fclose(f);
    buf[n] = '\0';

    cJSON *j = cJSON_Parse(buf);
    free(buf);
    if (!j) return false;

    bool ok = false;
    cJSON *s = cJSON_GetObjectItem(j, "serial");
    if (cJSON_IsString(s) && s->valuestring[0]) {
        strlcpy(serial, s->valuestring, serial_sz);
        ok = true;
    }
    cJSON *fw = cJSON_GetObjectItem(j, "firmware");
    if (fw && cJSON_IsString(fw) && firmware)
        strlcpy(firmware, fw->valuestring, fw_sz);
    cJSON *np = cJSON_GetObjectItem(j, "name_prefix");
    if (np && cJSON_IsString(np) && name_prefix)
        strlcpy(name_prefix, np->valuestring, prefix_sz);
    cJSON *la = cJSON_GetObjectItem(j, "last_addr");
    if (la && cJSON_IsString(la) && last_addr)
        strlcpy(last_addr, la->valuestring, addr_sz);
    cJSON *pr = cJSON_GetObjectItem(j, "protocol");
    if (pr && cJSON_IsString(pr) && protocol && protocol_sz > 0)
        strlcpy(protocol, pr->valuestring, protocol_sz);

    cJSON_Delete(j);
    return ok;
}

void ox_store_save_paired(const char *serial, const char *firmware,
                          const char *name_prefix, const char *last_addr,
                          const char *protocol)
{
    ox_store_ensure_dirs();
    cJSON *j = cJSON_CreateObject();
    cJSON_AddStringToObject(j, "serial", serial);
    if (firmware) cJSON_AddStringToObject(j, "firmware", firmware);
    if (name_prefix) cJSON_AddStringToObject(j, "name_prefix", name_prefix);
    if (last_addr) cJSON_AddStringToObject(j, "last_addr", last_addr);
    if (protocol) cJSON_AddStringToObject(j, "protocol", protocol);
    char *json = cJSON_PrintUnformatted(j);
    cJSON_Delete(j);
    if (!json) return;

    FILE *f = fopen(OXY_PAIRED_JSON, "w");
    if (f) {
        fputs(json, f);
        fclose(f);
    }
    cJSON_free(json);
}

void ox_store_delete_paired(void)
{
    remove(OXY_PAIRED_JSON);
}

/* ── index.json ───────────────────────────────────────────────────── */

/* Check if a file name + serial is already in index.json and finalised.
 * Returns: 1 = finalised, 0 = present but not finalised, -1 = not found. */
int ox_store_index_check(const char *serial, const char *name)
{
    FILE *f = fopen(OXY_INDEX_JSON, "r");
    if (!f) return -1;

    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > 65536) { fclose(f); return -1; }

    char *buf = malloc(sz + 1);
    if (!buf) { fclose(f); return -1; }
    int n = fread(buf, 1, sz, f);
    fclose(f);
    buf[n] = '\0';

    cJSON *arr = cJSON_Parse(buf);
    free(buf);
    if (!arr || !cJSON_IsArray(arr)) {
        if (arr) cJSON_Delete(arr);
        return -1;
    }

    int result = -1;
    cJSON *entry;
    cJSON_ArrayForEach(entry, arr) {
        cJSON *e_serial = cJSON_GetObjectItem(entry, "serial");
        cJSON *e_name = cJSON_GetObjectItem(entry, "name");
        cJSON *e_fin = cJSON_GetObjectItem(entry, "finalised");
        if (cJSON_IsString(e_serial) && cJSON_IsString(e_name) &&
            strcmp(e_serial->valuestring, serial) == 0 &&
            strcmp(e_name->valuestring, name) == 0) {
            if (cJSON_IsBool(e_fin) && cJSON_IsTrue(e_fin))
                result = 1;
            else
                result = 0;
            break;
        }
    }
    cJSON_Delete(arr);
    return result;
}

/* Add or update an entry in index.json. */
void ox_store_index_add(const char *serial, const char *name,
                        uint32_t bytes, bool finalised)
{
    FILE *f = fopen(OXY_INDEX_JSON, "r");
    cJSON *arr = NULL;
    if (f) {
        fseek(f, 0, SEEK_END);
        long sz = ftell(f);
        fseek(f, 0, SEEK_SET);
        if (sz > 0 && sz < 65536) {
            char *buf = malloc(sz + 1);
            if (buf) {
                int n = fread(buf, 1, sz, f);
                buf[n] = '\0';
                arr = cJSON_Parse(buf);
                free(buf);
            }
        }
        fclose(f);
    }
    if (!arr || !cJSON_IsArray(arr)) {
        if (arr) cJSON_Delete(arr);
        arr = cJSON_CreateArray();
    }

    /* Remove existing entry for this serial+name if present. */
    for (int i = cJSON_GetArraySize(arr) - 1; i >= 0; i--) {
        cJSON *e = cJSON_GetArrayItem(arr, i);
        cJSON *es = cJSON_GetObjectItem(e, "serial");
        cJSON *en = cJSON_GetObjectItem(e, "name");
        if (cJSON_IsString(es) && cJSON_IsString(en) &&
            strcmp(es->valuestring, serial) == 0 &&
            strcmp(en->valuestring, name) == 0) {
            cJSON_DeleteItemFromArray(arr, i);
        }
    }

    cJSON *entry = cJSON_CreateObject();
    cJSON_AddStringToObject(entry, "serial", serial);
    cJSON_AddStringToObject(entry, "name", name);
    cJSON_AddNumberToObject(entry, "bytes", bytes);
    cJSON_AddBoolToObject(entry, "finalised", finalised);
    cJSON_AddItemToArray(arr, entry);

    char *json = cJSON_PrintUnformatted(arr);
    cJSON_Delete(arr);
    if (!json) return;

    f = fopen(OXY_INDEX_JSON, "w");
    if (f) {
        fputs(json, f);
        fclose(f);
    }
    cJSON_free(json);
}

/* ── inbox / file management ───────────────────────────────────────── */

/* Check if a .part file exists in inbox and return its size. */
long ox_store_part_size(const char *name)
{
    char path[128];
    snprintf(path, sizeof(path), "%s/%s.part", OXY_INBOX, name);
    struct stat st;
    if (stat(path, &st) != 0) return 0;
    return st.st_size;
}

/* Append data to a .part file in inbox.  Returns ESP_OK on success. */
esp_err_t ox_store_part_append(const char *name, const uint8_t *data, size_t len)
{
    char path[128];
    snprintf(path, sizeof(path), "%s/%s.part", OXY_INBOX, name);

    FILE *f = fopen(path, "ab");
    if (!f) {
        ESP_LOGE(TAG, "cannot open %s for append", path);
        return ESP_FAIL;
    }
    size_t w = fwrite(data, 1, len, f);
    fclose(f);
    if (w != len) {
        ESP_LOGE(TAG, "short write to %s: %zu/%zu", path, w, len);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* Shared promotion path: validate inbox/<name>.part, copy it verbatim to
 * files/<serial>/<dst_name>, remove the .part and index the recording.
 * Validation mode:
 *   trailer_rule  - OxyII Format-A: magic at file_size - 44 must be
 *                   present; `finalised` reflects the check result.
 *   !trailer_rule - native-format recordings: exact byte size match
 *                   required, otherwise nothing is stored or indexed. */
static bool ox_store_promote_impl(const char *serial, const char *name,
                                  bool append_bin, bool trailer_rule,
                                  long expected_size)
{
    char part_path[128];
    snprintf(part_path, sizeof(part_path), "%s/%s.part", OXY_INBOX, name);

    FILE *f = fopen(part_path, "rb");
    if (!f) return false;

    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);

    bool finalised;
    if (trailer_rule) {
        if (fsize < TRAILER_LEN) {
            fclose(f);
            return false;
        }
        uint8_t magic[4];
        finalised = fseek(f, fsize - 44, SEEK_SET) == 0 &&
                    fread(magic, 1, 4, f) == 4 &&
                    memcmp(magic, TRAILER_MAGIC, 4) == 0;
    } else {
        if (expected_size <= 0 || fsize != expected_size) {
            ESP_LOGE(TAG, "promote '%s': size mismatch (have %ld, want %ld) — not stored",
                     name, fsize, expected_size);
            fclose(f);
            return false;
        }
        finalised = true;
    }

    /* Read the whole file into memory (files are typically < 300 KB). */
    fseek(f, 0, SEEK_SET);
    uint8_t *data = malloc(fsize);
    if (!data) {
        fclose(f);
        ESP_LOGE(TAG, "promote: OOM %ld bytes", fsize);
        return false;
    }
    int n = fread(data, 1, fsize, f);
    fclose(f);
    if (n != fsize) {
        free(data);
        return false;
    }

    /* Create files/<serial>/ directory. */
    char serial_dir[160];
    snprintf(serial_dir, sizeof(serial_dir), "%s/%s", OXY_FILES, serial);
    mkdir(OXY_FILES, 0775);
    mkdir(serial_dir, 0775);

    /* Write the final file — native filename preserved when the caller
     * does not request the legacy .bin suffix. */
    char bin_path[256];
    if (append_bin)
        snprintf(bin_path, sizeof(bin_path), "%s/%s/%s.bin",
                 OXY_FILES, serial, name);
    else
        snprintf(bin_path, sizeof(bin_path), "%s/%s/%s",
                 OXY_FILES, serial, name);
    f = fopen(bin_path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "cannot create %s", bin_path);
        free(data);
        return false;
    }
    fwrite(data, 1, fsize, f);
    fclose(f);
    free(data);

    /* Remove the .part file. */
    remove(part_path);

    /* Update index. */
    ox_store_index_add(serial, name, (uint32_t)fsize, finalised);

    ESP_LOGI(TAG, "promoted %s (%ld bytes, finalised=%d)", bin_path, fsize, finalised);
    return finalised;
}

/* Promote a .part file to files/<serial>/<name>.bin if trailer magic
 * is present.  Returns true if promoted (finalised), false otherwise. */
bool ox_store_promote(const char *serial, const char *name)
{
    return ox_store_promote_impl(serial, name, true, true, 0);
}

/* Promote inbox/<name>.part verbatim after an exact byte-size check.
 * Used for native-format recordings (legacy-ring .vld files). */
bool ox_store_promote_exact(const char *serial, const char *name,
                            long expected_size)
{
    return ox_store_promote_impl(serial, name, false, false, expected_size);
}

/* Remove a .part file (e.g. after failed promotion or to restart). */
void ox_store_part_remove(const char *name)
{
    char path[128];
    snprintf(path, sizeof(path), "%s/%s.part", OXY_INBOX, name);
    remove(path);
}
