/*
 * SomnoTrace - Oximetry recording storage (SD card) — public API
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

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/* Ensure the oximetry directory tree exists on SD. */
void ox_store_ensure_dirs(void);

/* Paired-ring record (NVS mirror + paired.json on SD).
 * `protocol` selects the backend that owns the pairing; see the
 * OX_PROTO_* constants in oximeter.h.  May be NULL/omitted on save
 * (older records default to OX_PROTO_OXYII). */
bool ox_store_load_paired(char *serial, size_t serial_sz,
                          char *firmware, size_t fw_sz,
                          char *name_prefix, size_t prefix_sz,
                          char *last_addr, size_t addr_sz,
                          char *protocol, size_t protocol_sz);
void ox_store_save_paired(const char *serial, const char *firmware,
                          const char *name_prefix, const char *last_addr,
                          const char *protocol);
void ox_store_delete_paired(void);

/* Downloaded-recording index (index.json).
 * Returns: 1 = finalised, 0 = present but not finalised, -1 = not found. */
int  ox_store_index_check(const char *serial, const char *name);
void ox_store_index_add(const char *serial, const char *name,
                        uint32_t bytes, bool finalised);

/* Partial-file inbox (inbox/<name>.part). */
long ox_store_part_size(const char *name);
esp_err_t ox_store_part_append(const char *name, const uint8_t *data, size_t len);
void ox_store_part_remove(const char *name);

/* Promote inbox/<name>.part → files/<serial>/<name>.bin after verifying
 * the OxyII Format-A trailer magic.  Returns true only if finalised. */
bool ox_store_promote(const char *serial, const char *name);

/* Promote inbox/<name>.part → files/<serial>/<name> verbatim (native
 * filename preserved, no .bin suffix), validating the exact byte size.
 * Used for native-format recordings (e.g. legacy-ring .vld files) which do
 * NOT carry the OxyII trailer.  Returns false — without storing or
 * indexing anything — if the partial file size differs from expected. */
bool ox_store_promote_exact(const char *serial, const char *name,
                            long expected_size);
