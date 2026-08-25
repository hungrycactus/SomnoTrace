/*
 * SomnoTrace - ST7789 LCD driver
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
 */


#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* Framebuffer geometry (RGB565, row-major). */
#define BSP_DISPLAY_FB_W 240
#define BSP_DISPLAY_FB_H 240

esp_err_t bsp_display_init(void);
void bsp_display_show_number(uint32_t value);
void bsp_display_show_lines(const char *title, const char *const *lines, int n_lines);

/* Persistent amber notice banner across the bottom of the status screen.
 * Survives any subsequent bsp_display_show_lines() call, so transient
 * messages cannot hide it. Pass NULL or "" to clear. */
void bsp_display_set_notice(const char *text);
void bsp_display_set_wifi_connected(bool connected);
void bsp_display_set_as11_paired(bool paired);
void bsp_display_set_battery(int percent, bool charging);

/* Therapy graph mode */
void bsp_display_set_therapy_active(bool active);
void bsp_display_push_flow(float flow_lpm);
bool bsp_display_is_therapy_active(void);

/* Backlight control (LEDC PWM on GPIO 46).
 * set_brightness: 1-200 (tenth-percent units: 1=0.1%, 200=20.0%), applied immediately.
 * set_backlight: hard on/off (used for therapy LCD-off mode).
 * get_brightness: returns last set brightness value. */
void bsp_display_set_brightness(uint8_t percent);
void bsp_display_set_backlight(bool on);
uint8_t bsp_display_get_brightness(void);

/* Apply the current backlight policy based on lcd_therapy_mode and therapy
 * state.  Called after boot completes, when entering/leaving SoftAP, or
 * when the mode is changed at runtime.
 *   force_on: if true, always turn backlight on (used for SoftAP mode). */
void bsp_display_apply_backlight_policy(bool force_on);

/* Set LCD rotation in degrees (0, 90, 180, 270). Applies to hardware
 * immediately via ST7789 MADCTL. Must be re-applied after panel reset. */
void bsp_display_set_rotation(uint8_t degrees);

/* Coherent screen capture (used by the /api/screenshot web endpoint).
 * take(): queues a framebuffer copy that is performed by the render task
 * (the sole owner of the framebuffer, so the image is never torn) and blocks
 * up to timeout_ms for it to complete. Returns a pointer to BSP_DISPLAY_FB_W
 * x BSP_DISPLAY_FB_H RGB565 uint16_t values in native byte order, or NULL if
 * the display is not initialised, a previous snapshot has not been released,
 * or the request timed out. A timed-out request stays queued and is answered
 * by a later call. Call release() exactly once after each successful take;
 * until then further takes return NULL. */
const uint16_t *bsp_display_snapshot_take(uint32_t timeout_ms);
void bsp_display_snapshot_release(void);
