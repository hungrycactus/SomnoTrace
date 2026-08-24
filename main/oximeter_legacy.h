/*
 * SomnoTrace - Legacy Wellue ring (P02 / O2Ring) BLE protocol backend
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
 * no third-party source copied.
 *
 * This file is a pure protocol backend implementing oximeter_backend_t;
 * scanning, pairing persistence, sync scheduling and SD storage live in
 * oximeter_common.c / oximeter_store.c.  Byte-level reference for BOTH
 * backends (OxyII 0xA5 and legacy 0xAA), including worked hex examples:
 * see ADD_LEGACY_OXIMETER.md in the repository root.
 */

#pragma once

#include "oximeter_backend.h"
