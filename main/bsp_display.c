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


#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>

#include "bsp_display.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "font_roboto.h"
#include "esp_wifi.h"
#include "driver/ledc.h"
#include "device_settings.h"
#include "psram_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define LCD_PIN_SCLK        38
#define LCD_PIN_MOSI        39
#define LCD_PIN_DC          45
#define LCD_PIN_CS          21
#define LCD_PIN_RST         40
#define LCD_PIN_BL          46

#define LCD_H_RES           240
#define LCD_V_RES           240
#define LCD_PIXEL_CLOCK_HZ  (40 * 1000 * 1000)  /* ST7789 write timing: keep well within spec; 80 MHz caused overnight GRAM freeze */
#define LCD_SPI_HOST        SPI2_HOST
#define LCD_CMD_BITS        8
#define LCD_PARAM_BITS      8
#define LCD_INVERT_COLOR    true

/* LEDC PWM for backlight dimming */
#define BL_LEDC_TIMER       LEDC_TIMER_0
#define BL_LEDC_CHANNEL     LEDC_CHANNEL_0
#define BL_LEDC_FREQ_HZ     5000
#define BL_LEDC_RESOLUTION  LEDC_TIMER_10_BIT  /* 0-1023 duty */
#define BL_DUTY_MAX         ((1 << 10) - 1)    /* 1023 */

static uint8_t s_brightness = 100;  /* current brightness (tenth-percent: 1=0.1%, 200=20%) */
static bool s_backlight_on = true;  /* backlight hardware state */
static bool s_backlight_force_on = false;  /* SoftAP/portal: keep backlight on */

static const char *TAG = "bsp_display";

/* Forward declarations */
static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b);
static void fb_clear(uint16_t color);
static int fb_draw_string_aa(int x, int y, const font_info_t *font, const char *str, uint16_t color);
static void fb_draw_wifi_indicator(int x, int y, bool connected);
static void render_graph(void);
static void render_status(void);
static void display_task(void *arg);
static void apply_panel_rotation(uint8_t degrees);
static void logo_init(void);
static void logo_render_frame(int cx, int cy);

static esp_lcd_panel_handle_t s_panel = NULL;
static esp_lcd_panel_io_handle_t s_io = NULL;
static uint16_t *s_fb = NULL;
static bool s_wifi_connected = false;
static bool s_as11_paired = false;
static uint8_t s_rotation = 0;  /* current LCD rotation in degrees */

/* Strip blit: the framebuffer lives in PSRAM (not DMA-capable), so it is
 * pushed to the panel in chunks via small internal DMA-capable buffers.
 * Two buffers are used so the DMA of one strip overlaps the CPU copy of the
 * next (pipelining). A counting semaphore tracks completed transfers so a
 * buffer is never reused while its DMA is still in flight. */
#define LCD_STRIP_ROWS 40
#define LCD_STRIP_BUFS 2
static uint16_t *s_strip[LCD_STRIP_BUFS] = { NULL, NULL };
static SemaphoreHandle_t s_flush_done = NULL;
static TaskHandle_t s_display_task = NULL;

/* ── Display state (single-owner render task model) ──────────────────
 *
 * Only the render task (display_task) ever touches the framebuffer s_fb
 * or calls esp_lcd_panel_draw_bitmap(). Every other function merely
 * mutates shared state under s_state_mutex and never draws. This makes
 * the display impossible to corrupt via concurrent access and guarantees
 * a clean full-frame redraw on every mode transition (no leftover
 * artifacts, no partial frames, no stuck modes). */

typedef enum {
    DISP_MODE_STATUS = 0,
    DISP_MODE_GRAPH,
} disp_mode_t;

#define FLOW_BUF_SIZE      240   /* one flow sample per pixel column */
#define MAX_STATUS_LINES   4
#define STATUS_TITLE_LEN   32
#define STATUS_LINE_LEN    48

#define DISPLAY_TASK_STACK 4096
/* Status refresh cadence. Doubles as the frame clock for the animated brand
 * badge (10 fps — deliberately modest; the therapy graph streams at 25 Hz).
 * The badge's phase follows a logical clock that never advances more than
 * ~1.25 frames at a step, so a late or interrupted render slows the
 * animation briefly instead of letting it jump/stutter. Raise this to save
 * power at the cost of smoothness. */
#define STATUS_FRAME_MS    100

/* ── Flow graph layout (static, non-adaptive) ───────────────────────────
 * GRAPH_FULL_SCALE is the fixed full-scale deflection (L/min) from the zero
 * line to the top/bottom of the plot. It is intentionally static so the view
 * does not jitter; values beyond it clip at the plot edge (the "hard cut").
 * Adjust this single constant to match the device's flow range. */
#define GRAPH_FULL_SCALE   150.0f
#define GRAPH_AXIS_W       34    /* left scale-axis gutter width (px) */
#define GRAPH_PLOT_X0      GRAPH_AXIS_W
#define GRAPH_PLOT_TOP     16
#define GRAPH_PLOT_BOT     224
#define GRAPH_PLOT_W       (LCD_H_RES - GRAPH_AXIS_W)

static SemaphoreHandle_t s_state_mutex = NULL;  /* protects all shared state below */
static disp_mode_t s_mode = DISP_MODE_STATUS;
static bool s_status_dirty = true;              /* status content changed, force redraw */

/* Status-screen content (copied from callers) */
static char s_status_title[STATUS_TITLE_LEN];
static char s_status_lines[MAX_STATUS_LINES][STATUS_LINE_LEN];
static int  s_status_nlines = 0;

/* Persistent notice banner rendered at the bottom of the status screen in
 * warning colours.  Independent of the status lines above so transient
 * messages (Wi-Fi reconnecting, SD errors) can never clobber it. */
static char s_notice[STATUS_LINE_LEN] = "";

/* Battery indicator state */
static int s_batt_percent = -1;   /* -1 = unknown/not set */
static bool s_batt_charging = false;

/* Live flow ring buffer for the therapy graph (PSRAM). */
static float *s_flow_buf;
static float *s_flow_local;   /* render-task snapshot */
static float *s_flow_yf;      /* render-task y coordinates */
static int    s_flow_head = 0;
static int    s_flow_count = 0;

/* Screenshot support. The framebuffer has a single owner (the render task),
 * so captures are requested via s_snap_want and performed inside
 * display_task; callers read the finished copy from s_snap_buf. All three
 * flags are guarded by s_state_mutex. */
static uint16_t *s_snap_buf = NULL;          /* module-owned snapshot buffer */
static SemaphoreHandle_t s_snap_sem = NULL;  /* given when a copy completes */
static bool s_snap_want = false;             /* copy requested, not yet done */
static bool s_snap_busy = false;             /* caller is reading s_snap_buf */

/* ── Public state-mutating API (never draws; render task handles drawing) ── */

void bsp_display_set_therapy_active(bool active)
{
    if (!s_state_mutex) {
        ESP_LOGW(TAG, "set_therapy_active(%s) called before init — ignored",
                 active ? "true" : "false");
        return;
    }

    /* Check LCD therapy mode setting */
    const device_settings_t *dev = device_settings_get();
    bool lcd_off_mode = (dev->lcd_therapy_mode != LCD_THERAPY_GRAPH);

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    disp_mode_t new_mode = active ? DISP_MODE_GRAPH : DISP_MODE_STATUS;
    if (s_mode != new_mode) {
        s_mode = new_mode;
        if (active) {
            s_flow_head = 0;
            s_flow_count = 0;
            ESP_LOGI(TAG, "therapy graph mode enabled (display_task=%s)",
                     s_display_task ? "alive" : "NULL");
        } else {
            s_status_dirty = true;  /* force immediate status redraw */
            ESP_LOGI(TAG, "therapy graph mode disabled");
        }
    } else {
        ESP_LOGD(TAG, "set_therapy_active(%s) — mode already %s, no-op",
                 active ? "true" : "false",
                 s_mode == DISP_MODE_GRAPH ? "GRAPH" : "STATUS");
    }
    xSemaphoreGive(s_state_mutex);

    /* Backlight policy:
     *   LCD_THERAPY_GRAPH:      always on
     *   LCD_THERAPY_OFF:        off during therapy, on when therapy stops
     *   LCD_THERAPY_ALWAYS_OFF: off during therapy, stays off when therapy stops */
    if (lcd_off_mode) {
        if (active) {
            bsp_display_set_backlight(false);
        } else if (dev->lcd_therapy_mode == LCD_THERAPY_OFF) {
            bsp_display_set_backlight(true);
        }
        /* ALWAYS_OFF: backlight stays off when therapy stops */
    }

    /* Wake the render task so the mode change is reflected immediately. */
    if (s_display_task) xTaskNotifyGive(s_display_task);
}

bool bsp_display_is_therapy_active(void)
{
    if (!s_state_mutex) return false;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    bool active = (s_mode == DISP_MODE_GRAPH);
    xSemaphoreGive(s_state_mutex);
    return active;
}

void bsp_display_push_flow(float flow_lpm)
{
    if (!s_state_mutex) return;
    bool notify = false;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    if (s_mode == DISP_MODE_GRAPH && s_flow_buf) {
        s_flow_buf[s_flow_head] = flow_lpm * 60.0f;
        s_flow_head = (s_flow_head + 1) % FLOW_BUF_SIZE;
        if (s_flow_count < FLOW_BUF_SIZE) s_flow_count++;
        notify = true;
    }
    xSemaphoreGive(s_state_mutex);
    /* Wake the render task so the graph advances at the data rate. Multiple
     * notifications between renders coalesce into a single redraw. */
    if (notify && s_display_task) xTaskNotifyGive(s_display_task);
}

const uint16_t *bsp_display_snapshot_take(uint32_t timeout_ms)
{
    if (!s_state_mutex || !s_snap_sem || !s_snap_buf ||
        !s_display_task || !s_fb) return NULL;

    bool queued = false;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    if (s_snap_busy) {
        xSemaphoreGive(s_state_mutex);
        return NULL;   /* previous snapshot not released yet */
    }
    if (!s_snap_want) {
        s_snap_want = true;
        queued = true;
    }
    xSemaphoreGive(s_state_mutex);

    /* A request that timed out earlier may still be queued; the semaphore is
     * answered by whichever copy completes, and s_snap_buf always holds one
     * coherent frame once it does. */
    if (queued) xTaskNotifyGive(s_display_task);

    const uint16_t *frame = NULL;
    if (xSemaphoreTake(s_snap_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        s_snap_busy = true;
        xSemaphoreGive(s_state_mutex);
        frame = s_snap_buf;
    }
    return frame;
}

void bsp_display_snapshot_release(void)
{
    if (!s_state_mutex) return;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_snap_busy = false;
    xSemaphoreGive(s_state_mutex);
}

/* ── Framebuffer → panel blit ───────────────────────────────────────── */

static bool IRAM_ATTR lcd_color_done_cb(esp_lcd_panel_io_handle_t io,
                                        esp_lcd_panel_io_event_data_t *edata,
                                        void *user_ctx)
{
    (void)io; (void)edata; (void)user_ctx;
    BaseType_t hp = pdFALSE;
    if (s_flush_done) xSemaphoreGiveFromISR(s_flush_done, &hp);
    return hp == pdTRUE;
}

/* Push the entire PSRAM framebuffer to the LCD in horizontal strips.
 *
 * The framebuffer is in PSRAM, which is not DMA-capable for the SPI master,
 * so a direct full-frame draw_bitmap() forces the driver to allocate a
 * ~115 KB internal DMA bounce buffer every frame. That allocation fails once
 * Wi-Fi and SDMMC have claimed internal RAM, silently dropping frames. By
 * copying each strip into a small, permanently-allocated internal DMA buffer
 * we guarantee the transfer always succeeds regardless of heap state. */

/* Hardware-reset the panel and re-send the full init sequence.  This is the
 * only reliable way to recover a wedged ST7789 that has stopped applying
 * RAMWR (frozen screen) while the CPU side keeps running normally.
 * Called from display_task only. */
static void lcd_panel_hw_recover(void)
{
    if (!s_panel) return;
    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);
    esp_lcd_panel_invert_color(s_panel, LCD_INVERT_COLOR);
    esp_lcd_panel_set_gap(s_panel, 0, 0);
    /* Re-apply rotation — panel reset wipes MADCTL back to 0° */
    if (s_rotation != 0) {
        apply_panel_rotation(s_rotation);
    }
    esp_lcd_panel_disp_on_off(s_panel, true);
    ESP_LOGI(TAG, "panel hardware reset + re-init (rot=%u)", s_rotation);
}

/* Apply rotation to the ST7789 panel via MADCTL (swap_xy + mirror).
 * Called after panel init/recover and when the user changes the setting.
 * The framebuffer layout (s_fb[y * 240 + x]) stays the same; the panel
 * remaps GRAM addressing so the physical pixels appear rotated. */
static void apply_panel_rotation(uint8_t degrees)
{
    if (!s_panel) return;
    bool swap_xy = false;
    bool mirror_x = false, mirror_y = false;

    switch (degrees) {
        case 0:
            break;
        case 90:   /* clockwise 90° */
            swap_xy = true;
            mirror_x = true;
            break;
        default:
            return;
    }

    esp_lcd_panel_swap_xy(s_panel, swap_xy);
    esp_lcd_panel_mirror(s_panel, mirror_x, mirror_y);
}

void bsp_display_set_rotation(uint8_t degrees)
{
    switch (degrees) {
        case 0: case 90:
            break;
        default:
            ESP_LOGW(TAG, "set_rotation: invalid %u", degrees);
            return;
    }
    s_rotation = degrees;
    apply_panel_rotation(degrees);
    ESP_LOGI(TAG, "rotation set to %u°", degrees);
}

static void lcd_flush(void)
{
    if (!s_panel || !s_fb) return;

    if (!s_strip[0] || !s_flush_done) {
        /* Fallback (e.g. strip alloc failed): single direct draw. */
        esp_lcd_panel_draw_bitmap(s_panel, 0, 0, LCD_H_RES, LCD_V_RES, s_fb);
        return;
    }

    int inflight = 0;
    int bi = 0;
    for (int y0 = 0; y0 < LCD_V_RES; y0 += LCD_STRIP_ROWS) {
        int rows = LCD_V_RES - y0;
        if (rows > LCD_STRIP_ROWS) rows = LCD_STRIP_ROWS;

        /* Wait for a strip buffer to become free.  Never abandon a transfer
         * mid-flight — reusing a buffer whose DMA is still running corrupts
         * the panel's DC/RAMWR stream and wedges the controller. */
        if (inflight >= LCD_STRIP_BUFS) {
            xSemaphoreTake(s_flush_done, portMAX_DELAY);
            inflight--;
        }

        uint16_t *buf = s_strip[bi];
        bi = (bi + 1) % LCD_STRIP_BUFS;
        memcpy(buf, &s_fb[y0 * LCD_H_RES],
               (size_t)rows * LCD_H_RES * sizeof(uint16_t));
        /* Queue asynchronously; the DMA of this strip overlaps the memcpy of
         * the next one (the completion callback gives s_flush_done). */
        esp_lcd_panel_draw_bitmap(s_panel, 0, y0, LCD_H_RES, y0 + rows, buf);
        inflight++;
    }
    /* Drain remaining in-flight transfers. */
    while (inflight > 0) {
        xSemaphoreTake(s_flush_done, portMAX_DELAY);
        inflight--;
    }
}

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    return (c >> 8) | (c << 8);
}

static inline uint32_t utf8_decode(const char **s)
{
    const uint8_t *p = (const uint8_t *)*s;
    if (!p || !*p) return 0;

    uint32_t c = *p;
    if (c < 0x80) {
        (*s)++;
        return c;
    }

    if ((c & 0xE0) == 0xC0) {
        if (!p[1]) {
            *s += 1;
            return '?';
        }
        c = ((c & 0x1F) << 6) | (p[1] & 0x3F);
        *s += 2;
        return c;
    }

    if ((c & 0xF0) == 0xE0) {
        if (!p[1]) {
            *s += 1;
            return '?';
        }
        if (!p[2]) {
            *s += 2;
            return '?';
        }
        c = ((c & 0x0F) << 12) | ((p[1] & 0x3F) << 6) | (p[2] & 0x3F);
        *s += 3;
        return c;
    }

    if ((c & 0xF8) == 0xF0) {
        if (!p[1]) {
            *s += 1;
            return '?';
        }
        if (!p[2]) {
            *s += 2;
            return '?';
        }
        if (!p[3]) {
            *s += 3;
            return '?';
        }
        c = ((c & 0x07) << 18) | ((p[1] & 0x3F) << 12) | ((p[2] & 0x3F) << 6) | (p[3] & 0x3F);
        *s += 4;
        return c;
    }

    (*s)++;
    return '?';
}

static const font_glyph_t *find_glyph(const font_info_t *font, uint32_t codepoint)
{
    int low = 0;
    int high = font->glyph_count - 1;
    while (low <= high) {
        int mid = (low + high) / 2;
        uint32_t cp = font->glyphs[mid].codepoint;
        if (cp == codepoint) {
            return &font->glyphs[mid];
        } else if (cp < codepoint) {
            low = mid + 1;
        } else {
            high = mid - 1;
        }
    }

    if (codepoint != '?') {
        return find_glyph(font, '?');
    }
    return NULL;
}

static inline void unpack_rgb565(uint16_t color, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint16_t c = (color >> 8) | (color << 8);
    *r = (c >> 8) & 0xF8;
    *r |= (*r >> 5);
    *g = (c >> 3) & 0xFC;
    *g |= (*g >> 6);
    *b = (c << 3) & 0xF8;
    *b |= (*b >> 5);
}

static inline uint16_t blend_pixels(uint16_t bg_color, uint16_t fg_color, uint8_t alpha)
{
    if (alpha == 0) return bg_color;
    if (alpha == 15) return fg_color;

    uint8_t bg_r, bg_g, bg_b;
    uint8_t fg_r, fg_g, fg_b;

    unpack_rgb565(bg_color, &bg_r, &bg_g, &bg_b);
    unpack_rgb565(fg_color, &fg_r, &fg_g, &fg_b);

    uint8_t blended_r = (fg_r * alpha + bg_r * (15 - alpha)) / 15;
    uint8_t blended_g = (fg_g * alpha + bg_g * (15 - alpha)) / 15;
    uint8_t blended_b = (fg_b * alpha + bg_b * (15 - alpha)) / 15;

    return rgb565(blended_r, blended_g, blended_b);
}

static void fb_draw_char_aa(int x, int y, const font_info_t *font, const font_glyph_t *glyph, uint16_t color)
{
    if (glyph->width == 0 || glyph->height == 0) return;

    uint32_t offset = glyph->bitmap_offset;

    for (int row = 0; row < glyph->height; row++) {
        int target_y = y + glyph->bearing_y + row;
        if (target_y < 0 || target_y >= LCD_V_RES) continue;

        for (int col = 0; col < glyph->width; col++) {
            int target_x = x + glyph->bearing_x + col;
            if (target_x < 0 || target_x >= LCD_H_RES) continue;

            uint32_t pixel_idx = row * glyph->width + col;
            uint32_t byte_idx = offset + (pixel_idx / 2);
            uint8_t byte_val = font->bitmaps[byte_idx];
            uint8_t alpha;
            if (pixel_idx % 2 == 0) {
                alpha = byte_val >> 4;
            } else {
                alpha = byte_val & 0x0F;
            }

            if (alpha > 0) {
                uint32_t fb_idx = target_y * LCD_H_RES + target_x;
                s_fb[fb_idx] = blend_pixels(s_fb[fb_idx], color, alpha);
            }
        }
    }
}

static int fb_draw_string_aa(int x, int y, const font_info_t *font, const char *str, uint16_t color)
{
    int cx = x;
    const char *p = str;
    while (*p) {
        uint32_t cp = utf8_decode(&p);
        if (cp == 0) break;
        const font_glyph_t *glyph = find_glyph(font, cp);
        if (glyph) {
            fb_draw_char_aa(cx, y, font, glyph, color);
            cx += glyph->advance;
        }
    }
    return cx - x;
}

static int str_width_aa(const font_info_t *font, const char *str)
{
    int width = 0;
    const char *p = str;
    while (*p) {
        uint32_t cp = utf8_decode(&p);
        if (cp == 0) break;
        const font_glyph_t *glyph = find_glyph(font, cp);
        if (glyph) {
            width += glyph->advance;
        }
    }
    return width;
}

esp_err_t bsp_display_init(void)
{
    /* The project's default log level is DEBUG; spi_master emits several DEBUG
     * lines per DMA transaction. At the LCD's transfer rate that is a real CPU
     * and I/O drain, so quiet it down to WARN regardless of the global level. */
    esp_log_level_set("spi_master", ESP_LOG_WARN);

    /* Backlight: LEDC PWM for brightness control */
    ledc_timer_config_t bl_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = BL_LEDC_RESOLUTION,
        .timer_num = BL_LEDC_TIMER,
        .freq_hz = BL_LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&bl_timer));

    ledc_channel_config_t bl_ch = {
        .gpio_num = LCD_PIN_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = BL_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BL_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&bl_ch));
    s_backlight_on = true;
    s_brightness = 100;

    s_flush_done = xSemaphoreCreateCounting(LCD_STRIP_BUFS, 0);

    spi_bus_config_t bus_cfg = {
        .sclk_io_num = LCD_PIN_SCLK,
        .mosi_io_num = LCD_PIN_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = LCD_PIN_DC,
        .cs_gpio_num = LCD_PIN_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = lcd_color_done_cb,
        .user_ctx = NULL,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_cfg, &io));
    s_io = io;

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &panel_cfg, &s_panel));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, LCD_INVERT_COLOR));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(s_panel, 0, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    /* Allocate the framebuffer from PSRAM. The ESP32-S3 GDMA can stream SPI
     * pixel data directly from external RAM, so the ~115 KB framebuffer does
     * not need scarce internal DMA-capable RAM (which Wi-Fi SoftAP beacon and
     * SDMMC buffers require). Fall back to internal DMA RAM if PSRAM is absent. */
    s_fb = heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    if (!s_fb) {
        s_fb = heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t), MALLOC_CAP_DMA);
    }
    if (!s_fb) {
        ESP_LOGE(TAG, "framebuffer alloc failed");
        return ESP_ERR_NO_MEM;
    }

    s_flow_buf = heap_caps_calloc(FLOW_BUF_SIZE, sizeof(float), MALLOC_CAP_SPIRAM);
    s_flow_local = heap_caps_malloc(FLOW_BUF_SIZE * sizeof(float), MALLOC_CAP_SPIRAM);
    s_flow_yf = heap_caps_malloc(LCD_H_RES * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!s_flow_buf || !s_flow_local || !s_flow_yf) {
        ESP_LOGE(TAG, "flow graph buffer alloc failed");
        return ESP_ERR_NO_MEM;
    }

    /* Permanently allocate the internal DMA-capable strip buffers up front,
     * while internal RAM is still free. */
    for (int i = 0; i < LCD_STRIP_BUFS; i++) {
        s_strip[i] = heap_caps_malloc(LCD_H_RES * LCD_STRIP_ROWS * sizeof(uint16_t),
                                      MALLOC_CAP_DMA);
        if (!s_strip[i]) {
            ESP_LOGW(TAG, "strip buffer %d alloc failed; using direct flush", i);
        }
    }

    /* Bake the animated brand watermark's static emblem layer (non-fatal on
     * failure — the status screen renders normally without it). */
    logo_init();

    /* Create the state mutex and start the single-owner render task. Only this
     * task ever touches the framebuffer or the LCD panel. */
    s_state_mutex = xSemaphoreCreateMutex();
    s_snap_sem = xSemaphoreCreateBinary();
    s_snap_buf = heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
                                  MALLOC_CAP_SPIRAM);
    if (!s_snap_sem || !s_snap_buf) {
        ESP_LOGW(TAG, "screenshot buffers unavailable; capture API disabled");
    }
    if (s_state_mutex) {
        s_display_task = psram_task_create(display_task, "display", DISPLAY_TASK_STACK, NULL, 4, tskNO_AFFINITY, NULL, NULL);
    } else {
        ESP_LOGE(TAG, "state mutex alloc failed, display task not started");
    }

    ESP_LOGI(TAG, "ST7789 display initialised");
    return ESP_OK;
}

void bsp_display_set_wifi_connected(bool connected)
{
    if (!s_state_mutex) {
        s_wifi_connected = connected;
        return;
    }
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_wifi_connected = connected;
    s_status_dirty = true;
    xSemaphoreGive(s_state_mutex);
    if (s_display_task) xTaskNotifyGive(s_display_task);
}

void bsp_display_set_as11_paired(bool paired)
{
    if (!s_state_mutex) {
        s_as11_paired = paired;
        return;
    }
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_as11_paired = paired;
    s_status_dirty = true;
    xSemaphoreGive(s_state_mutex);
    if (s_display_task) xTaskNotifyGive(s_display_task);
}

void bsp_display_set_battery(int percent, bool charging)
{
    if (!s_state_mutex) return;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_batt_percent = percent;
    s_batt_charging = charging;
    s_status_dirty = true;
    xSemaphoreGive(s_state_mutex);
    if (s_display_task) xTaskNotifyGive(s_display_task);
}

/* ── Backlight control ─────────────────────────────────────────────── */

void bsp_display_set_brightness(uint8_t percent)
{
    if (percent < 1) percent = 1;
    if (percent > 200) percent = 200;
    s_brightness = percent;
    if (s_backlight_on) {
        /* percent is in tenth-percent units (1=0.1%), so divide by 1000 */
        uint32_t duty = (uint32_t)(percent) * BL_DUTY_MAX / 1000;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CHANNEL);
    }
}

void bsp_display_set_backlight(bool on)
{
    if (on == s_backlight_on) return;
    s_backlight_on = on;
    if (on) {
        uint32_t duty = (uint32_t)(s_brightness) * BL_DUTY_MAX / 1000;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CHANNEL);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CHANNEL);
    }
}

uint8_t bsp_display_get_brightness(void)
{
    return s_brightness;
}

void bsp_display_apply_backlight_policy(bool force_on)
{
    if (force_on) {
        s_backlight_force_on = true;
        bsp_display_set_backlight(true);
        return;
    }

    /* If force-on is active (SoftAP), keep backlight on regardless of mode */
    if (s_backlight_force_on) {
        bsp_display_set_backlight(true);
        return;
    }

    const device_settings_t *dev = device_settings_get();
    bool therapy_active = bsp_display_is_therapy_active();

    switch (dev->lcd_therapy_mode) {
    case LCD_THERAPY_GRAPH:
        bsp_display_set_backlight(true);
        break;
    case LCD_THERAPY_OFF:
        bsp_display_set_backlight(!therapy_active);
        break;
    case LCD_THERAPY_ALWAYS_OFF:
        bsp_display_set_backlight(false);
        break;
    default:
        bsp_display_set_backlight(true);
        break;
    }
}

static void fb_fill_rect(int x, int y, int w, int h, uint16_t color)
{
    for (int row = y; row < y + h; row++) {
        if (row < 0 || row >= LCD_V_RES) continue;
        for (int col = x; col < x + w; col++) {
            if (col < 0 || col >= LCD_H_RES) continue;
            s_fb[row * LCD_H_RES + col] = color;
        }
    }
}

static void fb_clear(uint16_t color)
{
    for (int i = 0; i < LCD_H_RES * LCD_V_RES; i++) {
        s_fb[i] = color;
    }
}

/* Alpha-blend src over dst. Both are byte-swapped RGB565 (the on-wire format
 * produced by rgb565()); a is 0..255 coverage. */
static inline uint16_t blend565(uint16_t dst_sw, uint16_t src_sw, uint8_t a)
{
    uint16_t d = (uint16_t)((dst_sw >> 8) | (dst_sw << 8));
    uint16_t s = (uint16_t)((src_sw >> 8) | (src_sw << 8));
    uint16_t ia = (uint16_t)(255 - a);
    uint16_t dr = (d >> 11) & 0x1F, dg = (d >> 5) & 0x3F, db = d & 0x1F;
    uint16_t sr = (s >> 11) & 0x1F, sg = (s >> 5) & 0x3F, sb = s & 0x1F;
    uint16_t rr = (uint16_t)((sr * a + dr * ia) / 255);
    uint16_t rg = (uint16_t)((sg * a + dg * ia) / 255);
    uint16_t rb = (uint16_t)((sb * a + db * ia) / 255);
    uint16_t r = (uint16_t)((rr << 11) | (rg << 5) | rb);
    return (uint16_t)((r >> 8) | (r << 8));
}

static inline void fb_blend(int x, int y, uint16_t color_sw, uint8_t a)
{
    if (a == 0 || x < 0 || x >= LCD_H_RES || y < 0 || y >= LCD_V_RES) return;
    uint16_t *p = &s_fb[y * LCD_H_RES + x];
    *p = blend565(*p, color_sw, a);
}

/* Antialiased line of given thickness, drawn via per-pixel distance coverage
 * and alpha-blended at up to max_alpha. Round end caps. */
static void fb_draw_line_aa(float x0, float y0, float x1, float y1,
                            float thick, uint16_t color, uint8_t max_alpha)
{
    float half = thick * 0.5f;
    int ix0 = (int)floorf(fminf(x0, x1) - half - 1.0f);
    int ix1 = (int)ceilf(fmaxf(x0, x1) + half + 1.0f);
    int iy0 = (int)floorf(fminf(y0, y1) - half - 1.0f);
    int iy1 = (int)ceilf(fmaxf(y0, y1) + half + 1.0f);

    float dx = x1 - x0, dy = y1 - y0;
    float len2 = dx * dx + dy * dy;

    for (int y = iy0; y <= iy1; y++) {
        if (y < 0 || y >= LCD_V_RES) continue;
        for (int x = ix0; x <= ix1; x++) {
            if (x < 0 || x >= LCD_H_RES) continue;
            float t = len2 > 0.0f ? ((x - x0) * dx + (y - y0) * dy) / len2 : 0.0f;
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            float cx = x0 + t * dx, cy = y0 + t * dy;
            float ex = x - cx, ey = y - cy;
            float dist = sqrtf(ex * ex + ey * ey);
            float cov = half + 0.5f - dist;   /* coverage in px */
            if (cov <= 0.0f) continue;
            if (cov > 1.0f) cov = 1.0f;
            fb_blend(x, y, color, (uint8_t)(cov * max_alpha));
        }
    }
}

void bsp_display_show_number(uint32_t value)
{
    char buf[12];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)value);
    const char *lines[] = { buf };
    bsp_display_show_lines(NULL, lines, 1);
}

static uint16_t get_wifi_rssi_color(int rssi)
{
    if (rssi >= -60) {
        return rgb565(0, 255, 120);   // Excellent: green
    } else if (rssi >= -70) {
        return rgb565(100, 255, 100);  // Good: vibrant light green
    } else if (rssi >= -80) {
        return rgb565(255, 220, 0);   // Fair: yellow
    } else {
        return rgb565(255, 50, 50);   // Poor: red
    }
}

static void fb_draw_wifi_indicator(int x, int y, bool connected)
{
    if (!connected) {
        return; // Not connected, don't draw indicator
    }

    int rssi = -128;
    if (esp_wifi_sta_get_rssi(&rssi) != ESP_OK) {
        return; // Not connected, don't draw indicator
    }

    int active_bars = 0;
    if (rssi >= -60) active_bars = 4;
    else if (rssi >= -70) active_bars = 3;
    else if (rssi >= -80) active_bars = 2;
    else if (rssi >= -90) active_bars = 1;

    uint16_t inactive_col = rgb565(60, 60, 60);
    uint16_t active_col = get_wifi_rssi_color(rssi);

    // Draw 4 bars of increasing height
    for (int i = 0; i < 4; i++) {
        uint16_t col = (i < active_bars) ? active_col : inactive_col;
        int bar_h = (i + 1) * 4;
        fb_fill_rect(x + i * 5, y + 16 - bar_h, 3, bar_h, col);
    }
}

void bsp_display_show_lines(const char *title, const char *const *lines, int n_lines)
{
    if (!s_state_mutex) return;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);

    if (title) {
        strncpy(s_status_title, title, STATUS_TITLE_LEN - 1);
        s_status_title[STATUS_TITLE_LEN - 1] = '\0';
    } else {
        s_status_title[0] = '\0';
    }

    if (n_lines < 0) n_lines = 0;
    if (n_lines > MAX_STATUS_LINES) n_lines = MAX_STATUS_LINES;
    s_status_nlines = n_lines;
    for (int i = 0; i < n_lines; i++) {
        if (lines[i]) {
            strncpy(s_status_lines[i], lines[i], STATUS_LINE_LEN - 1);
            s_status_lines[i][STATUS_LINE_LEN - 1] = '\0';
        } else {
            s_status_lines[i][0] = '\0';
        }
    }

    s_status_dirty = true;
    xSemaphoreGive(s_state_mutex);
    if (s_display_task) xTaskNotifyGive(s_display_task);
}

/* ── Render helpers — called ONLY by display_task ───────────────────── */

/* Render the scrolling flow waveform with a fixed (static) vertical scale and
 * a left scale axis. Snapshots the flow ring buffer under the state mutex,
 * then draws without holding it so high-rate push_flow() callers are never
 * blocked for long. */
static void render_graph(void)
{
    if (!s_panel || !s_fb) return;

    const uint16_t bg        = rgb565(9, 11, 18);
    const uint16_t grid_col  = rgb565(28, 32, 46);
    const uint16_t zero_col  = rgb565(70, 78, 102);
    const uint16_t axis_col  = rgb565(40, 46, 64);
    const uint16_t flow_col  = rgb565(54, 247, 160);   /* sharp mint line */
    const uint16_t glow_col  = rgb565(20, 205, 132);   /* soft glow */
    const uint16_t fill_col  = rgb565(28, 150, 104);   /* area under curve */
    const uint16_t label_col = rgb565(122, 134, 158);
    const uint16_t unit_col  = rgb565(86, 96, 120);

    if (!s_flow_buf || !s_flow_local || !s_flow_yf) return;

    int n, head;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    n = s_flow_count;
    head = s_flow_head;
    memcpy(s_flow_local, s_flow_buf, FLOW_BUF_SIZE * sizeof(float));
    xSemaphoreGive(s_state_mutex);

    fb_clear(bg);

    const int mid_y  = (GRAPH_PLOT_TOP + GRAPH_PLOT_BOT) / 2;
    const int half_h = (GRAPH_PLOT_BOT - GRAPH_PLOT_TOP) / 2;
    const float scale = (float)half_h / GRAPH_FULL_SCALE;  /* px per L/min */

    /* ── Grid + left scale axis ──────────────────────────────────────── */
    const int tick_step = 50;  /* L/min between labelled ticks */
    int nticks = (int)(GRAPH_FULL_SCALE / tick_step);
    if (nticks < 1) nticks = 1;
    for (int t = -nticks; t <= nticks; t++) {
        int gy = mid_y + (int)lroundf(t * tick_step * scale);
        if (gy < GRAPH_PLOT_TOP || gy > GRAPH_PLOT_BOT) continue;
        if (t == 0) {
            for (int x = GRAPH_PLOT_X0; x < LCD_H_RES; x++)
                s_fb[gy * LCD_H_RES + x] = zero_col;
        } else {
            for (int x = GRAPH_PLOT_X0; x < LCD_H_RES; x += 5)
                s_fb[gy * LCD_H_RES + x] = grid_col;
        }
        char tb[8];
        snprintf(tb, sizeof(tb), "%d", t < 0 ? -t * tick_step : t * tick_step);
        int tw = str_width_aa(&roboto_body, tb);
        int tx = GRAPH_AXIS_W - 4 - tw;
        if (tx < 1) tx = 1;
        fb_draw_string_aa(tx, gy - roboto_body.height / 2, &roboto_body, tb, label_col);
    }

    /* Vertical time gridlines (anchored to the right/newest edge). */
    for (int x = LCD_H_RES - 1; x >= GRAPH_PLOT_X0; x -= 42)
        for (int y = GRAPH_PLOT_TOP; y <= GRAPH_PLOT_BOT; y += 5)
            s_fb[y * LCD_H_RES + x] = grid_col;

    /* Axis separator + unit label. */
    for (int y = GRAPH_PLOT_TOP; y <= GRAPH_PLOT_BOT; y++)
        s_fb[y * LCD_H_RES + (GRAPH_AXIS_W - 1)] = axis_col;
    fb_draw_string_aa(GRAPH_AXIS_W + 3, 1, &roboto_body, "L/m", unit_col);

    /* ── Waveform (static scale, right-aligned newest sample) ────────── */
    int m = n;
    if (m > GRAPH_PLOT_W) m = GRAPH_PLOT_W;
    int start = (head - m + FLOW_BUF_SIZE) % FLOW_BUF_SIZE;
    int xbase = LCD_H_RES - m;

    float *yf = s_flow_yf;
    for (int j = 0; j < m; j++) {
        float val = s_flow_local[(start + j) % FLOW_BUF_SIZE];
        float y = mid_y - val * scale;  /* positive flow (inhale) → up */
        if (y < GRAPH_PLOT_TOP) y = GRAPH_PLOT_TOP;   /* static hard cut */
        if (y > GRAPH_PLOT_BOT) y = GRAPH_PLOT_BOT;
        yf[xbase + j] = y;
    }

    /* Translucent area fill between the curve and the zero line. */
    for (int j = 0; j < m; j++) {
        int x = xbase + j;
        int y0 = (int)(yf[x] < mid_y ? yf[x] : mid_y);
        int y1 = (int)(yf[x] < mid_y ? mid_y : yf[x]);
        for (int y = y0; y <= y1; y++)
            fb_blend(x, y, fill_col, 30);
    }

    /* Soft glow pass, then the sharp antialiased line on top. */
    for (int j = 1; j < m; j++) {
        int x = xbase + j;
        fb_draw_line_aa(x - 1, yf[x - 1], x, yf[x], 4.5f, glow_col, 45);
    }
    for (int j = 1; j < m; j++) {
        int x = xbase + j;
        fb_draw_line_aa(x - 1, yf[x - 1], x, yf[x], 2.0f, flow_col, 255);
    }

    lcd_flush();
}

/* Draw battery percentage with a small outline icon and optional charging bolt.
 * Layout: [12×9px battery outline] [N% text]
 * x,y is the top-left of the battery outline. */
static void fb_draw_battery_indicator(int x, int y, int percent, bool charging)
{
    uint16_t frame_col = rgb565(180, 180, 180);
    uint16_t bolt_col = rgb565(255, 200, 0);

    /* Text color based on charge level */
    uint16_t text_col;
    if (percent < 0) {
        text_col = rgb565(100, 100, 100);   /* unknown */
    } else if (percent <= 15) {
        text_col = rgb565(255, 60, 60);      /* red */
    } else if (percent <= 30) {
        text_col = rgb565(255, 180, 0);      /* orange */
    } else {
        text_col = rgb565(80, 220, 100);     /* green */
    }

    /* Battery outline: 10px wide × 8px tall body + 2px terminal nub */
    fb_fill_rect(x, y + 1, 10, 8, frame_col);        /* outer frame */
    fb_fill_rect(x + 1, y + 2, 8, 6, rgb565(0, 0, 0)); /* inner cavity */
    fb_fill_rect(x + 10, y + 3, 2, 4, frame_col);    /* terminal nub */

    /* Charging bolt inside the outline */
    if (charging) {
        fb_fill_rect(x + 4, y + 2, 2, 3, bolt_col);
        fb_fill_rect(x + 3, y + 3, 4, 1, bolt_col);
        fb_fill_rect(x + 2, y + 4, 6, 1, bolt_col);
        fb_fill_rect(x + 3, y + 5, 4, 1, bolt_col);
        fb_fill_rect(x + 4, y + 6, 2, 2, bolt_col);
    }

    /* Percentage text to the right of the outline */
    if (percent >= 0) {
        char pct_str[16];
        snprintf(pct_str, sizeof(pct_str), "%d%%", percent);
        fb_draw_string_aa(x + 15, y - 3, &roboto_body, pct_str, text_col);
    }
}

/* \u2550\u2550 Animated brand badge (status screen, title lockup) \u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550\u2550
 *
 * Procedural re-creation of the animated emblem from the web UI
 * (assets/svg/logo-small.svg \u2014 also served as /favicon.svg and embedded in
 * /logo.svg), miniaturised to title size and placed to the LEFT of the
 * status title as a lockup, mirroring the web header brand.
 *
 *   SVG element              | SMIL animation                  | Device implementation
 *   -------------------------+---------------------------------+------------------------------------------------------
 *   gradient disc + rim ring | static                          | baked once into an offscreen sprite
 *   accent ring (r=216u)     | opacity 0.15\u21920.38\u21920.15, 4.8 s   | plotted circle, pulsing alpha + halo
 *   waveform                 | translate x \u2212400 u / 4 s        | per-column sine spans drifting left (period \u2248 5.3 s)
 *   waveform glow            | feGaussianBlur 3.8\u21927.2, 4.8 s   | wider low-alpha span pass, pulsing
 *   echo wave                | opacity 0.10\u21920.26\u21920.10, 4.8 s   | thin second span pass (culled below ~1 px stroke)
 *   crescent moon            | opacity 0.68\u21921\u21920.68, 6 s        | two-circle coverage mask, breathing alpha
 *   3 stars                  | staggered opacity pulses        | radial-coverage dots (culled below ~1 px radius)
 *
 * Layout contract: render_status() centres the [emblem + gap + title]
 * group, so the badge shifts ONLY the title (right by half of emblem+gap
 * compared to plain centring); every other element keeps its exact
 * coordinates. When there is no title \u2014 or it is too wide to share its row
 * with the emblem \u2014 the badge is skipped and the legacy centred-title
 * layout is used.
 *
 * Animation pacing: the phase comes from a logical clock that advances by
 * the real inter-frame interval CLAMPED to ~1.25 nominal frames. In steady
 * state the animation runs at exactly real-time speed; if a render is late
 * or interrupted (notification burst, mode switch), the animation briefly
 * slows instead of jumping forward \u2014 smooth and consistent, never
 * stuttering to catch up.
 *
 * Scale awareness: stroke widths derive from the SVG geometry (with a
 * legibility floor), and elements that would be smaller than ~1 px at the
 * chosen LOGO_SIZE (stars, echo wave) are culled rather than shimmered.
 */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define LOGO_SIZE       36                      /* emblem diameter \u2248 title glyph height (roboto_title: 33) */
#define LOGO_SCALE      (LOGO_SIZE / 512.0f)    /* SVG unit \u2192 device px */
#define LOGO_GAP        7                       /* gap between emblem and title text */
#define LOGO_DROP_PX    4                       /* emblem nudged below the title's visual mid */

#define LOGO_DIM_PCT    100     /* foreground badge: faithful web colours (knob kept for tuning) */

/* Geometry carried over from the SVG viewBox (values in SVG units). */
#define LOGO_R_DISCU    248     /* outer disc edge          */
#define LOGO_R_RIMU     236     /* rim ring radius          */
#define LOGO_RIM_HALFWU 4       /* rim stroke half-width    */
#define LOGO_R_RINGU    216     /* accent ring radius       */
#define LOGO_R_CLIPU    205     /* waveform clip circle     */
#define LOGO_WAVE_BASEU 22      /* wave baseline below ctr  */
#define LOGO_WAVE_AMPU  80      /* crest/trough amplitude   */
#define LOGO_WAVE_LENU  400     /* wavelength               */
/* Leftward drift, in SVG units per second. Holding the UNIT rate constant
 * preserves the animation's temporal rhythm at any icon size (crest-to-crest
 * period = λ/v = 400/75 ≈ 5.3 s); the pixel speed simply scales with
 * LOGO_SIZE (31 px/s at 212 px, 5.3 px/s at 36 px). Slightly quicker than
 * the web original (100 u/s would be the faithful rate) per user preference. */
#define LOGO_WAVE_SPEEDU 75

/* Baked static emblem: RGB565 colours + per-pixel coverage for the gradient
 * disc and rim ring (allocated once; a few KB).  If allocation fails the
 * badge silently stays off and the title falls back to plain centring. */
static uint16_t *s_logo_rgb = NULL;
static uint8_t  *s_logo_a   = NULL;
static bool      s_logo_ok  = false;

/* Logical animation clock (render-task-private). */
static int64_t  s_logo_last_us = -1;    /* last frame's esp_timer reading */
static float    s_logo_anim_t  = 0.0f;  /* smoothed animation time (s)    */

/* Per-column scratch for the wave spans (render-task-private). */
#define LOGO_SPAN_MAX   180     /* covers any LOGO_SIZE up to 212 px */
static float s_logo_y_main[LOGO_SPAN_MAX];
static float s_logo_y_echo[LOGO_SPAN_MAX];
static bool  s_logo_col_ok[LOGO_SPAN_MAX];

/* Triangle ramp 0\u21921\u21920 over period_s \u2014 matches SMIL values="lo;hi;lo" with an
 * optional begin offset \u2014 evaluated at logical time t seconds. */
static inline float logo_pulse(float t, float period_s, float begin_s)
{
    float ph = (t - begin_s) / period_s;
    ph -= floorf(ph);
    return 1.0f - fabsf(2.0f * ph - 1.0f);
}

/* Map an opacity in [lo,hi] as authored in the SVG through the global
 * dimmer to an 8-bit blend alpha. */
static inline uint8_t logo_alpha(float lo, float hi, float k)
{
    float o = (lo + (hi - lo) * k) * (LOGO_DIM_PCT / 100.0f);
    if (o <= 0.0f) return 0;
    if (o >= 1.0f) return 255;
    return (uint8_t)(o * 255.0f + 0.5f);
}

static inline float logo_clamp01(float v)
{
    return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

/* Vertical wave gradient sampled by the wave's vertical position across its
 * full peak-to-peak travel \u2014 same colour stops and offsets (0 / 40 % / 100 %)
 * as the SVG's waveGrad: #A5F3FC \u2192 #22D3EE \u2192 #818CF8. */
static uint16_t logo_wave_color(float rel /* px from emblem centre */)
{
    const float tt = logo_clamp01((rel - (LOGO_WAVE_BASEU - LOGO_WAVE_AMPU) * LOGO_SCALE) /
                                  (2.0f * LOGO_WAVE_AMPU * LOGO_SCALE));
    uint8_t r, g, b;
    if (tt < 0.40f) {
        const float u = tt / 0.40f;
        r = (uint8_t)(0xA5 + (0x22 - 0xA5) * u);
        g = (uint8_t)(0xF3 + (0xD3 - 0xF3) * u);
        b = (uint8_t)(0xFC + (0xEE - 0xFC) * u);
    } else {
        const float u = (tt - 0.40f) / 0.60f;
        r = (uint8_t)(0x22 + (0x81 - 0x22) * u);
        g = (uint8_t)(0xD3 + (0x8C - 0xD3) * u);
        b = (uint8_t)(0xEE + (0xF8 - 0xEE) * u);
    }
    return rgb565(r, g, b);
}

static void logo_init(void)
{
    const size_t n = (size_t)LOGO_SIZE * LOGO_SIZE;
    s_logo_rgb = heap_caps_malloc(n * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    s_logo_a   = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
    if (!s_logo_rgb || !s_logo_a) {
        ESP_LOGW(TAG, "badge sprite alloc failed \\u2014 animated emblem disabled");
        if (s_logo_rgb) { heap_caps_free(s_logo_rgb); s_logo_rgb = NULL; }
        if (s_logo_a)   { heap_caps_free(s_logo_a);   s_logo_a   = NULL; }
        return;
    }

    const float c = LOGO_SIZE * 0.5f;   /* disc centre inside the sprite */
    for (int y = 0; y < LOGO_SIZE; y++) {
        for (int x = 0; x < LOGO_SIZE; x++) {
            const int i = y * LOGO_SIZE + x;
            const float dx = x + 0.5f - c;
            const float dy = y + 0.5f - c;
            const float dist = sqrtf(dx * dx + dy * dy);

            float cov = LOGO_R_DISCU * LOGO_SCALE + 0.5f - dist;  /* AA disc edge */
            if (cov <= 0.0f) {
                s_logo_a[i] = 0;
                continue;
            }
            if (cov > 1.0f) cov = 1.0f;

            /* Diagonal linear gradient #0B1D36 \u2192 #122B4A (SVG bgGrad). */
            const float gt = (float)(x + y) / (float)(2 * (LOGO_SIZE - 1));
            float r = 0x0B + (0x12 - 0x0B) * gt;
            float g = 0x1D + (0x2B - 0x1D) * gt;
            float b = 0x36 + (0x4A - 0x36) * gt;

            /* Rim ring #1E3A5F @ 55 % (SVG r=236, stroke-width=8). */
            const float rc = LOGO_RIM_HALFWU * LOGO_SCALE + 0.5f -
                             fabsf(dist - LOGO_R_RIMU * LOGO_SCALE);
            if (rc > 0.0f) {
                const float ro = (rc > 1.0f ? 1.0f : rc) * 0.55f;
                r += (0x1E - r) * ro;
                g += (0x3A - g) * ro;
                b += (0x5F - b) * ro;
            }

            s_logo_rgb[i] = rgb565((uint8_t)r, (uint8_t)g, (uint8_t)b);
            s_logo_a[i] = (uint8_t)(cov * 255.0f + 0.5f);
        }
    }
    s_logo_ok = true;
    ESP_LOGI(TAG, "animated brand badge ready (%dx%d sprite)", LOGO_SIZE, LOGO_SIZE);
}

/* Blend one pixel COLUMN of a vertical span [y0f, y1f] with per-row exact
 * coverage.  Used to rasterise the badge waveforms: because the waves are
 * functions of x, per-column spans avoid the double-covered endpoint pixels
 * (and resulting alpha pile-up) that chained capsule segments would produce,
 * and keep brightness independent of slope. */
static void logo_vspan(int x, float y0f, float y1f, uint16_t col, uint8_t a)
{
    if (a == 0 || x < 0 || x >= LCD_H_RES) return;
    const int y_lo = (int)floorf(y0f);
    const int y_hi = (int)ceilf(y1f);
    for (int y = y_lo; y < y_hi; y++) {
        if (y < 0 || y >= LCD_V_RES) continue;
        float cov = fminf((float)y + 1.0f, y1f) - fmaxf((float)y, y0f);
        if (cov <= 0.0f) continue;
        if (cov > 1.0f) cov = 1.0f;
        fb_blend(x, y, col, (uint8_t)((float)a * cov + 0.5f));
    }
}

/* Draw one animation frame of the badge centred at framebuffer position
 * (cx, cy).  Called from render_status() only (render-task context), right
 * after fb_clear(). */
static void logo_render_frame(int cx, int cy)
{
    if (!s_logo_ok) return;

    /* Logical animation clock: advance by the real elapsed interval, but
     * never by more than ~1.25 nominal frames.  Steady state tracks wall
     * time exactly; a late/interrupted render slows the animation briefly
     * instead of jumping it forward. */
    const int64_t now = esp_timer_get_time();
    if (s_logo_last_us < 0) s_logo_last_us = now;
    float dt = (float)(now - s_logo_last_us) * 1e-6f;
    s_logo_last_us = now;
    if (dt < 0.0f) dt = 0.0f;
    const float max_step = (STATUS_FRAME_MS * 1.25f) / 1000.0f;
    if (dt > max_step) dt = max_step;
    s_logo_anim_t += dt;
    const float t = s_logo_anim_t;

    const float fx = (float)cx, fy = (float)cy;

    /* ── 1. Baked emblem (gradient disc + rim ring) ──────────────────── */
    const int ox = cx - LOGO_SIZE / 2, oy = cy - LOGO_SIZE / 2;
    if (ox < 0 || oy < 0 ||
        ox + LOGO_SIZE > LCD_H_RES || oy + LOGO_SIZE > LCD_V_RES) {
        return;                          /* out of range: draw nothing */
    }
    const unsigned dim = (255u * LOGO_DIM_PCT) / 100u;
    for (int y = 0; y < LOGO_SIZE; y++) {
        uint16_t *dst = &s_fb[(oy + y) * LCD_H_RES + ox];
        const uint16_t *src = &s_logo_rgb[y * LOGO_SIZE];
        const uint8_t *sa = &s_logo_a[y * LOGO_SIZE];
        for (int x = 0; x < LOGO_SIZE; x++) {
            if (!sa[x]) continue;
            const uint8_t fa = (uint8_t)(((unsigned)sa[x] * dim + 127u) / 255u);
            if (fa) dst[x] = blend565(dst[x], src[x], fa);
        }
    }

    /* ── 2. Accent ring, breathing (SVG opacity 0.15→0.38 over 4.8 s) ── */
    const uint16_t ring_col = rgb565(0x22, 0xD3, 0xEE);
    const uint8_t ring_a = logo_alpha(0.15f, 0.38f, logo_pulse(t, 4.8f, 0.0f));
    const uint8_t ring_halo = (uint8_t)(ring_a / 3);
    const float ring_r = LOGO_R_RINGU * LOGO_SCALE;
    /* Step count proportional to the circumference (~1.4 steps/px) so the
     * plotted ring gets even coverage without alpha pile-up at any size. */
    int steps = (int)(6.2831853f * ring_r * 1.4f);
    if (steps < 48) steps = 48;
    if (steps > 900) steps = 900;
    for (int i = 0; i < steps; i++) {
        const float th = (float)i * (2.0f * (float)M_PI / (float)steps);
        const float cs = cosf(th), sn = sinf(th);
        fb_blend((int)(fx + ring_r * cs),
                 (int)(fy + ring_r * sn), ring_col, ring_a);
        /* Faint inner/outer halo stands in for the SVG's ringGlow blur. */
        fb_blend((int)(fx + ring_r * cs - 0.7f),
                 (int)(fy + ring_r * sn - 0.7f), ring_col, ring_halo);
        fb_blend((int)(fx + ring_r * cs + 0.7f),
                 (int)(fy + ring_r * sn + 0.7f), ring_col, ring_halo);
    }

    /* ── 3. Waveforms ───────────────────────────────────────────────────
     * The SMIL translate becomes a phase term; the pattern drifts leftward
     * at LOGO_WAVE_SPEEDU units/s, preserving the web loop's period at any
     * icon size.  Stroke half-widths derive from the SVG strokes (main
     * 12 u, echo 3.6 u) with a legibility floor.  Points beyond the SVG's
     * r=205-unit clip circle are dropped, reproducing the clipPath. */
    const float ph0 = 2.0f * (float)M_PI * (LOGO_WAVE_SPEEDU * LOGO_SCALE * t) /
                      (LOGO_WAVE_LENU * LOGO_SCALE);
    const uint16_t echo_col = rgb565(0x67, 0xE8, 0xF9);
    const uint16_t glow_col = rgb565(0x22, 0xD3, 0xEE);
    const uint8_t echo_a = logo_alpha(0.10f, 0.26f, logo_pulse(t, 4.8f, 0.0f));
    const uint8_t glow_a = logo_alpha(0.07f, 0.15f, logo_pulse(t, 4.8f, 0.0f));
    const uint8_t core_a = logo_alpha(1.0f, 1.0f, 0.0f);

    float core_half = 6.0f * LOGO_SCALE;
    if (core_half < 0.9f) core_half = 0.9f;
    const float glow_half = core_half * 1.8f;
    const float echo_half = 1.8f * LOGO_SCALE;
    const bool draw_echo = (echo_half >= 0.55f);

    int wx0 = (int)(fx - LOGO_R_CLIPU * LOGO_SCALE) - 1;
    int wx1 = (int)(fx + LOGO_R_CLIPU * LOGO_SCALE) + 2;
    if (wx0 < 0) wx0 = 0;
    if (wx1 > LCD_H_RES) wx1 = LCD_H_RES;

    const float clip2 = (LOGO_R_CLIPU * LOGO_SCALE) * (LOGO_R_CLIPU * LOGO_SCALE);
    int npts = 0;
    for (int x = wx0; x < wx1; x++, npts++) {
        const float u = (float)x - fx;
        const float ang = ph0 + 2.0f * (float)M_PI * u / (LOGO_WAVE_LENU * LOGO_SCALE);
        const float s = sinf(ang);
        const float ym = fy + LOGO_WAVE_BASEU * LOGO_SCALE - LOGO_WAVE_AMPU * LOGO_SCALE * s;
        const float ye = fy + (LOGO_WAVE_BASEU + 11.0f) * LOGO_SCALE -
                         (58.5f * LOGO_SCALE) * s;
        const bool ok =
            (u * u + (ym - fy) * (ym - fy) <= clip2) &&
            (u * u + (ye - fy) * (ye - fy) <= clip2);
        s_logo_col_ok[npts] = ok;
        s_logo_y_main[npts] = ym;
        s_logo_y_echo[npts] = ye;
    }

    /* Soft glow pass (stands in for pulseGlow), then core, then echo — one
     * exact-coverage vertical span per column per pass. */
    for (int j = 0; j < npts; j++) {
        if (!s_logo_col_ok[j]) continue;
        const int x = wx0 + j;
        logo_vspan(x, s_logo_y_main[j] - glow_half, s_logo_y_main[j] + glow_half,
                   glow_col, glow_a);
        logo_vspan(x, s_logo_y_main[j] - core_half, s_logo_y_main[j] + core_half,
                   logo_wave_color(s_logo_y_main[j] - fy), core_a);
        if (draw_echo) {
            logo_vspan(x, s_logo_y_echo[j] - echo_half, s_logo_y_echo[j] + echo_half,
                       echo_col, echo_a);
        }
    }

    /* ── 4. Crescent moon (two-circle coverage mask), breathing ──────── */
    const uint16_t moon_col = rgb565(0xE0, 0xF2, 0xFE);
    const uint8_t moon_a = logo_alpha(0.68f, 1.0f, logo_pulse(t, 6.0f, 0.0f));
    {
        const float mcx = fx + 52.0f * LOGO_SCALE, mcy = fy - 144.0f * LOGO_SCALE;
        const float mr = 38.0f * LOGO_SCALE;
        const float qcx = fx + 72.0f * LOGO_SCALE, qcy = fy - 156.0f * LOGO_SCALE;
        const float qr = 31.0f * LOGO_SCALE;

        const int bx0 = (int)(mcx - mr) - 1, bx1 = (int)(mcx + mr) + 2;
        const int by0 = (int)(mcy - mr) - 1, by1 = (int)(mcy + mr) + 2;
        for (int py = by0; py <= by1; py++) {
            if (py < 0 || py >= LCD_V_RES) continue;
            for (int px = bx0; px <= bx1; px++) {
                if (px < 0 || px >= LCD_H_RES) continue;
                const float fxx = px + 0.5f, fyy = py + 0.5f;
                const float dmx = fxx - mcx, dmy = fyy - mcy;
                float cov = mr + 0.5f - sqrtf(dmx * dmx + dmy * dmy);
                if (cov <= 0.0f) continue;
                if (cov > 1.0f) cov = 1.0f;
                const float dqx = fxx - qcx, dqy = fyy - qcy;
                float cut = sqrtf(dqx * dqx + dqy * dqy) - qr + 0.5f;
                if (cut <= 0.0f) continue;   /* fully inside the cut-out circle */
                if (cut > 1.0f) cut = 1.0f;
                fb_blend(px, py, moon_col,
                         (uint8_t)((float)moon_a * cov * cut + 0.5f));
            }
        }
    }

    /* ── 5. Stars, staggered twinkle (culled below ~1 px radius) ─────── */
    static const struct {
        float dx, dy, r;         /* position relative to disc centre + radius (SVG units) */
        uint8_t cr, cg, cb;      /* fill colour */
        float lo, hi;            /* SVG opacity range */
        float period, begin;     /* twinkle timing (s) */
    } stars[3] = {
        { -114.0f, -104.0f, 6.5f, 0xA5, 0xB4, 0xFC, 0.28f, 1.00f, 2.7f, 0.0f },
        {  -88.0f, -138.0f, 5.0f, 0x67, 0xE8, 0xF9, 0.22f, 0.90f, 3.3f, 0.7f },
        {  134.0f,  -71.0f, 5.2f, 0x22, 0xD3, 0xEE, 0.22f, 0.95f, 3.0f, 1.2f },
    };
    for (int si = 0; si < 3; si++) {
        const float sr = stars[si].r * LOGO_SCALE;
        if (sr < 0.9f) continue;         /* sub-pixel at this icon size */
        const uint8_t a = logo_alpha(stars[si].lo, stars[si].hi,
                                     logo_pulse(t, stars[si].period, stars[si].begin));
        if (!a) continue;
        const float sx = fx + stars[si].dx * LOGO_SCALE;
        const float sy = fy + stars[si].dy * LOGO_SCALE;
        const int bx0 = (int)(sx - sr) - 1, bx1 = (int)(sx + sr) + 2;
        const int by0 = (int)(sy - sr) - 1, by1 = (int)(sy + sr) + 2;
        for (int py = by0; py <= by1; py++) {
            if (py < 0 || py >= LCD_V_RES) continue;
            for (int px = bx0; px <= bx1; px++) {
                if (px < 0 || px >= LCD_H_RES) continue;
                const float fxx = px + 0.5f - sx, fyy = py + 0.5f - sy;
                float cov = sr + 0.5f - sqrtf(fxx * fxx + fyy * fyy);
                if (cov <= 0.0f) continue;
                if (cov > 1.0f) cov = 1.0f;
                fb_blend(px, py, rgb565(stars[si].cr, stars[si].cg, stars[si].cb),
                         (uint8_t)((float)a * cov + 0.5f));
            }
        }
    }
}
/* Render the status screen. Snapshots content under the state mutex, then
 * draws without holding it. RSSI is read live each refresh. */
static void render_status(void)
{
    if (!s_panel || !s_fb) return;

    char title[STATUS_TITLE_LEN];
    char lines[MAX_STATUS_LINES][STATUS_LINE_LEN];
    int nlines;
    bool wifi;

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    memcpy(title, s_status_title, sizeof(title));
    memcpy(lines, s_status_lines, sizeof(lines));
    nlines = s_status_nlines;
    wifi = s_wifi_connected;
    bool as11_paired = s_as11_paired;
    int batt_pct = s_batt_percent;
    bool batt_chg = s_batt_charging;
    char notice[STATUS_LINE_LEN];
    memcpy(notice, s_notice, sizeof(notice));
    xSemaphoreGive(s_state_mutex);

    const uint16_t bg = rgb565(0, 0, 0);
    const uint16_t title_col = rgb565(0, 255, 120);
    const uint16_t text_col = rgb565(255, 255, 255);

    fb_clear(bg);

    /* ── Brand badge + title lockup placement ───────────────────────────
     * The emblem sits left of the title and the pair is centred as one
     * group, shifting the title right by half of emblem+gap. With no title
     * (or one too wide to share the row) the badge is skipped and the
     * legacy centred-title layout is used. Every other element keeps its
     * exact coordinates. */
    bool logo_show = false;
    int logo_cx = 0, logo_cy = 0;
    int title_x = -1;                       /* -1 → centred-title fallback */
    if (title[0]) {
        const int tw = str_width_aa(&roboto_title, title);
        const int lockup_w = LOGO_SIZE + LOGO_GAP + tw;
        if (lockup_w <= LCD_H_RES - 8) {
            int gx = (LCD_H_RES - lockup_w) / 2;
            if (gx < 4) gx = 4;
            logo_cx = gx + LOGO_SIZE / 2;
            logo_cy = 48 + roboto_title.ascender / 2 + LOGO_DROP_PX;   /* title mid, nudged down */
            title_x = gx + LOGO_SIZE + LOGO_GAP;
            logo_show = true;
        }
    }

    if (logo_show) {
        logo_render_frame(logo_cx, logo_cy);
    }

    /* Clock display (top-left) */
    time_t now = time(NULL);
    if (now > 1700000000) {  /* only show if NTP-synced (after ~Nov 2023) */
        struct tm tm_info;
        localtime_r(&now, &tm_info);
        char time_str[16];
        strftime(time_str, sizeof(time_str), "%H:%M", &tm_info);
        fb_draw_string_aa(6, 9, &roboto_body, time_str, rgb565(200, 210, 225));
    }

    fb_draw_wifi_indicator(218, 10, wifi);

    /* AS11 paired icon — small CPAP mask indicator to the left of WiFi bars */
    if (as11_paired) {
        int bx = 218 - 22;
        int by = 10;
        uint16_t icon_col = rgb565(100, 200, 255);
        /* Simple mask icon: rounded rectangle body + nose bridge */
        fb_fill_rect(bx, by + 4, 14, 10, icon_col);
        fb_fill_rect(bx + 5, by + 2, 4, 4, icon_col);
        /* Strap line */
        for (int dx = 0; dx < 14; dx += 3)
            fb_fill_rect(bx + dx, by + 14, 2, 2, icon_col);
    }

    /* Battery indicator — left of AS11 icon, right of clock area */
    if (batt_pct >= 0) {
        fb_draw_battery_indicator(130, 12, batt_pct, batt_chg);
    }

    int y = 48;
    if (title[0]) {
        int w = str_width_aa(&roboto_title, title);
        int x = (title_x >= 0) ? title_x : (LCD_H_RES - w) / 2;
        if (x < 4) x = 4;
        fb_draw_string_aa(x, y, &roboto_title, title, title_col);
        y += 40;
    } else {
        y = 60;
    }

    int line_h = roboto_body.height + 6;
    if (line_h < 18) line_h = 18;
    for (int i = 0; i < nlines; i++) {
        if (!lines[i][0]) continue;
        int w = str_width_aa(&roboto_body, lines[i]);
        int x = (LCD_H_RES - w) / 2;
        if (x < 4) x = 4;
        fb_draw_string_aa(x, y, &roboto_body, lines[i], text_col);
        y += line_h;
    }

    /* ── Persistent notice banner (bottom, amber) ──────────────────────
     * Drawn last and anchored to the bottom edge so it survives whatever
     * the status lines above happen to say. */
    if (notice[0]) {
        const uint16_t notice_col = rgb565(255, 190, 30);
        const uint16_t notice_bg  = rgb565(46, 34, 0);

        int band_h = roboto_body.height + 10;
        int band_y = LCD_V_RES - band_h;
        fb_fill_rect(0, band_y, LCD_H_RES, band_h, notice_bg);

        /* Warning triangle, drawn from primitives (the font has no glyph). */
        int tri_h = 11;
        int tri_x = 8;
        int tri_y = band_y + (band_h - tri_h) / 2;
        for (int row = 0; row < tri_h; row++) {
            int half = (row * 6) / tri_h;
            fb_fill_rect(tri_x + 6 - half, tri_y + row, half * 2 + 1, 1, notice_col);
        }
        /* Exclamation mark punched out of the triangle. */
        fb_fill_rect(tri_x + 6, tri_y + 4, 1, 4, notice_bg);
        fb_fill_rect(tri_x + 6, tri_y + 9, 1, 1, notice_bg);

        int tw = str_width_aa(&roboto_body, notice);
        int tx = tri_x + 16 + ((LCD_H_RES - tri_x - 16) - tw) / 2;
        if (tx < tri_x + 16) tx = tri_x + 16;
        fb_draw_string_aa(tx, band_y + 5, &roboto_body, notice, notice_col);
    }

    lcd_flush();
}

void bsp_display_set_notice(const char *text)
{
    if (!s_state_mutex) return;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    if (text && text[0]) {
        strncpy(s_notice, text, STATUS_LINE_LEN - 1);
        s_notice[STATUS_LINE_LEN - 1] = '\0';
    } else {
        s_notice[0] = '\0';
    }
    s_status_dirty = true;
    xSemaphoreGive(s_state_mutex);
    if (s_display_task) xTaskNotifyGive(s_display_task);
}

/* The single owner of the framebuffer and LCD panel. Renders the current
 * mode at a fixed cadence and on every mode/content change. */
static void display_task(void *arg)
{
    (void)arg;
    TickType_t last_render = 0;
    disp_mode_t last_mode = (disp_mode_t)-1;

    for (;;) {
        /* Block until woken by new data / a state change (push_flow,
         * set_therapy_active, show_lines, set_wifi_connected) or until the
         * status refresh interval elapses. This drives the graph at exactly
         * the data rate (25 Hz) with zero busy-polling; coalesced
         * notifications mean we never render more often than data arrives. */
        uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(STATUS_FRAME_MS));

        /* Service screenshot requests. The copy must happen inside this task:
         * it is the only writer of s_fb, which is what makes the image
         * tear-free even while the graph streams at 25 Hz. */
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        bool snap_req = s_snap_want;
        s_snap_want = false;
        xSemaphoreGive(s_state_mutex);
        if (snap_req && s_snap_buf) {
            memcpy(s_snap_buf, s_fb,
                   LCD_H_RES * LCD_V_RES * sizeof(uint16_t));
            xSemaphoreGive(s_snap_sem);
        }

        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        disp_mode_t mode = s_mode;
        bool dirty = s_status_dirty;
        s_status_dirty = false;
        xSemaphoreGive(s_state_mutex);

        bool mode_changed = (mode != last_mode);
        TickType_t now = xTaskGetTickCount();

        /* On any mode transition, hardware-reset the panel first so a wedged
         * ST7789 (frozen screen, ignoring SPI commands) is recovered before
         * the new frame is drawn. */
        if (mode_changed) {
            lcd_panel_hw_recover();
        }

        if (mode == DISP_MODE_GRAPH) {
            /* Redraw on new data or when first entering graph mode. */
            if (notified || mode_changed) {
                render_graph();
                last_render = now;
            }
        } else {
            if (mode_changed || dirty ||
                (now - last_render) >= pdMS_TO_TICKS(STATUS_FRAME_MS)) {
                render_status();
                last_render = now;
            }
        }
        last_mode = mode;
    }
}
