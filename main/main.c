#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp/esp-bsp.h"
#include "clicker.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_io_expander.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "lvgl.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "proximity.h"
#include "target.h"

#define STALE_MS 8000
#define HOLD_MS 12000
#define LIST_STALE_MS 15000
#define SWITCH_DB 8
#define MAX_DEVICES 24
#define SPARK_LEN 28
#define COL_BG 0x0C0B09
#define COL_INK 0xF3F0E8
#define COL_GHOST 0x3A3832
#define COL_MUTE 0x1C1B18
#define COL_ACCENT 0xF0C400
#define DOT_PX 7
#define DOT_PITCH 11
#define LINE_DOTS 24

typedef struct {
    ble_addr_t addr;
    char name[32];
    int8_t rssi;
    int64_t seen_us;
} device_t;

typedef struct {
    bool locked;
    bool sound_on;
    ble_addr_t addr;
    char name[32];
    float rssi;
    int previous_rssi;
    int64_t seen_us;
    int8_t spark[SPARK_LEN];
    uint8_t spark_len;
    uint8_t spark_head;
    uint8_t nearby;
} hunt_t;

static const char *TAG = "ble_finder";
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static hunt_t s_hunt = {.sound_on = true};
static device_t s_devices[MAX_DEVICES];
static size_t s_device_count;

static lv_obj_t *s_face;
static lv_obj_t *s_status_label;
static lv_obj_t *s_sound_btn;
static lv_obj_t *s_sound_label;
static char s_face_rssi[8] = "--";
static int s_face_fill;
static bool s_face_live;

/* 5x7, bit 4 is the left column. */
static const uint8_t k_digit[10][7] = {
    {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E},
    {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},
    {0x0E, 0x11, 0x01, 0x06, 0x08, 0x10, 0x1F},
    {0x0E, 0x11, 0x01, 0x06, 0x01, 0x11, 0x0E},
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02},
    {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},
    {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E},
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E},
    {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C},
};
static const uint8_t k_minus[7] = {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00};

static bool same_addr(const ble_addr_t *a, const ble_addr_t *b)
{
    return a->type == b->type && memcmp(a->val, b->val, sizeof(a->val)) == 0;
}

static bool parse_addr(const char *text, ble_addr_t *out)
{
    unsigned bytes[6];
    if (text[0] == '\0') {
        return false;
    }
    if (sscanf(text, "%02X:%02X:%02X:%02X:%02X:%02X",
               &bytes[0], &bytes[1], &bytes[2],
               &bytes[3], &bytes[4], &bytes[5]) != 6) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    for (int i = 0; i < 6; ++i) {
        out->val[5 - i] = (uint8_t)bytes[i];
    }
    return true;
}

static bool name_matches(const char *name)
{
    const char *needle = TARGET_NAME;
    if (needle[0] == '\0' || name[0] == '\0') {
        return false;
    }
    for (const char *p = name; *p != '\0'; ++p) {
        const char *a = p;
        const char *b = needle;
        while (*a != '\0' && *b != '\0'
               && tolower((unsigned char)*a) == tolower((unsigned char)*b)) {
            ++a;
            ++b;
        }
        if (*b == '\0') {
            return true;
        }
    }
    return false;
}

static bool configured_addr_matches(const ble_addr_t *addr)
{
    ble_addr_t configured;
    return parse_addr(TARGET_ADDR, &configured) && same_addr(addr, &configured);
}

static int strongest_first(const void *left, const void *right)
{
    return ((const device_t *)right)->rssi - ((const device_t *)left)->rssi;
}

static void remember_device(const struct ble_gap_disc_desc *disc)
{
    struct ble_hs_adv_fields fields = {0};
    const bool parsed = ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data) == 0;
    const int64_t now = esp_timer_get_time();

    taskENTER_CRITICAL(&s_lock);
    size_t index = s_device_count;
    for (size_t i = 0; i < s_device_count; ++i) {
        if (same_addr(&s_devices[i].addr, &disc->addr)) {
            index = i;
            break;
        }
    }
    if (index == s_device_count) {
        if (s_device_count < MAX_DEVICES) {
            ++s_device_count;
        } else {
            index = 0;
            for (size_t i = 1; i < MAX_DEVICES; ++i) {
                if (s_devices[i].seen_us < s_devices[index].seen_us) {
                    index = i;
                }
            }
        }
        s_devices[index] = (device_t){.addr = disc->addr};
    }

    device_t *device = &s_devices[index];
    device->rssi = disc->rssi;
    device->seen_us = now;
    if (parsed && fields.name_len > 0) {
        const size_t length = fields.name_len < sizeof(device->name) - 1
                                  ? fields.name_len
                                  : sizeof(device->name) - 1;
        memcpy(device->name, fields.name, length);
        device->name[length] = '\0';
    }
    taskEXIT_CRITICAL(&s_lock);
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    if (event->type == BLE_GAP_EVENT_DISC) {
        remember_device(&event->disc);
    }
    return 0;
}

static void start_scan(void)
{
    uint8_t own_addr_type;
    struct ble_gap_disc_params params = {
        .itvl = 0x30,
        .window = 0x30,
        .filter_policy = 0,
        .limited = 0,
        .passive = 0,
        .filter_duplicates = 0,
    };
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc == 0) {
        rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &params, gap_event, NULL);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE scan failed: %d", rc);
    }
}

static void ble_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) {
        start_scan();
    } else {
        ESP_LOGE(TAG, "BLE address init failed: %d", rc);
    }
}

static void ble_host_task(void *arg)
{
    (void)arg;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, uint32_t color)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_remove_flag(label, LV_OBJ_FLAG_SCROLLABLE);
    lv_label_set_long_mode(label, LV_LABEL_LONG_CLIP);
    lv_obj_set_style_text_font(label, font, LV_PART_MAIN);
    lv_obj_set_style_text_color(label, lv_color_hex(color), LV_PART_MAIN);
    return label;
}

static void draw_dot(lv_layer_t *layer, int cx, int cy, int size, uint32_t color)
{
    lv_draw_rect_dsc_t dsc;
    lv_draw_rect_dsc_init(&dsc);
    dsc.radius = LV_RADIUS_CIRCLE;
    dsc.bg_color = lv_color_hex(color);
    dsc.bg_opa = LV_OPA_COVER;
    dsc.border_width = 0;
    dsc.outline_width = 0;
    dsc.shadow_width = 0;
    lv_area_t area = {
        .x1 = (int32_t)(cx - size / 2),
        .y1 = (int32_t)(cy - size / 2),
        .x2 = (int32_t)(cx - size / 2 + size - 1),
        .y2 = (int32_t)(cy - size / 2 + size - 1),
    };
    lv_draw_rect(layer, &dsc, &area);
}

static const uint8_t *glyph_rows(char c)
{
    if (c >= '0' && c <= '9') {
        return k_digit[c - '0'];
    }
    if (c == '-') {
        return k_minus;
    }
    return NULL;
}

static void draw_glyph(lv_layer_t *layer, int x, int y, const uint8_t *rows)
{
    for (int row = 0; row < 7; ++row) {
        for (int col = 0; col < 5; ++col) {
            if (rows[row] & (0x10 >> col)) {
                draw_dot(layer, x + col * DOT_PITCH + DOT_PX / 2,
                         y + row * DOT_PITCH + DOT_PX / 2, DOT_PX, COL_INK);
            }
        }
    }
}

static void draw_face(lv_event_t *event)
{
    lv_obj_t *obj = lv_event_get_target_obj(event);
    lv_layer_t *layer = lv_event_get_layer(event);
    lv_area_t coords;
    lv_obj_get_coords(obj, &coords);

    const int width = lv_area_get_width(&coords);
    const char *text = s_face_rssi;
    int length = (int)strlen(text);
    if (length < 1) {
        text = "--";
        length = 2;
    }
    const int glyph_w = 5 * DOT_PITCH;
    const int total = length * glyph_w + (length - 1) * DOT_PITCH;
    int x = coords.x1 + (width - total) / 2;
    const int y = coords.y1 + 36;
    for (int i = 0; i < length; ++i) {
        const uint8_t *rows = glyph_rows(text[i]);
        if (rows != NULL) {
            draw_glyph(layer, x, y, rows);
        }
        x += glyph_w + DOT_PITCH;
    }

    const int line_pitch = 12;
    const int line_w = (LINE_DOTS - 1) * line_pitch;
    const int x0 = coords.x1 + (width - line_w) / 2;
    const int ly = y + 7 * DOT_PITCH + 36;
    const int lit = s_face_live ? (s_face_fill * LINE_DOTS + 99) / 100 : 0;
    for (int i = 0; i < LINE_DOTS; ++i) {
        uint32_t color = COL_GHOST;
        int size = 6;
        if (i < lit) {
            color = (i == lit - 1) ? COL_ACCENT : COL_INK;
            size = (i == lit - 1) ? 9 : 6;
        }
        draw_dot(layer, x0 + i * line_pitch, ly, size, color);
    }
}

static const char *ui_status(int rssi, bool stale)
{
    if (stale) {
        return "lost";
    }
    switch (proximity_band(rssi)) {
    case 0:
        return "at hand";
    case 1:
        return "same table";
    case 2:
        return "same room";
    case 3:
        return "far";
    default:
        return "very far";
    }
}

static void paint_sound(bool on)
{
    lv_label_set_text(s_sound_label, on ? "Mute" : "Sound");
    lv_obj_set_style_bg_color(s_sound_btn, lv_color_hex(on ? COL_MUTE : COL_ACCENT), LV_PART_MAIN);
    lv_obj_set_style_text_color(s_sound_label, lv_color_hex(on ? COL_INK : COL_BG), LV_PART_MAIN);
}

static void toggle_sound(lv_event_t *event)
{
    (void)event;
    taskENTER_CRITICAL(&s_lock);
    s_hunt.sound_on = !s_hunt.sound_on;
    const bool on = s_hunt.sound_on;
    taskEXIT_CRITICAL(&s_lock);
    clicker_set_enabled(on);
    paint_sound(on);
}

static void build_ui(void)
{
    lv_obj_t *screen = lv_screen_active();
    lv_obj_set_size(screen, 368, 448);
    lv_obj_set_style_bg_color(screen, lv_color_hex(COL_BG), LV_PART_MAIN);
    lv_obj_set_style_pad_all(screen, 0, LV_PART_MAIN);
    lv_obj_set_style_text_font(screen, &lv_font_unscii_16, LV_PART_MAIN);
    lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(screen, LV_SCROLLBAR_MODE_OFF);

    s_face = lv_obj_create(screen);
    lv_obj_set_size(s_face, 368, 180);
    lv_obj_align(s_face, LV_ALIGN_TOP_MID, 0, 88);
    lv_obj_set_style_bg_opa(s_face, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_face, 0, LV_PART_MAIN);
    lv_obj_set_style_outline_width(s_face, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(s_face, 0, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(s_face, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_face, 0, LV_PART_MAIN);
    lv_obj_remove_flag(s_face, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(s_face, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_face, draw_face, LV_EVENT_DRAW_MAIN, NULL);

    s_status_label = make_label(screen, &lv_font_unscii_16, COL_INK);
    lv_label_set_text(s_status_label, "scanning");
    lv_obj_align(s_status_label, LV_ALIGN_TOP_MID, 0, 292);

    s_sound_btn = lv_button_create(screen);
    lv_obj_set_size(s_sound_btn, 336, 80);
    lv_obj_align(s_sound_btn, LV_ALIGN_BOTTOM_MID, 0, -16);
    lv_obj_set_ext_click_area(s_sound_btn, 16);
    lv_obj_set_style_radius(s_sound_btn, 40, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(s_sound_btn, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_sound_btn, lv_color_hex(0x3A3832), LV_STATE_PRESSED);
    lv_obj_add_event_cb(s_sound_btn, toggle_sound, LV_EVENT_CLICKED, NULL);

    s_sound_label = lv_label_create(s_sound_btn);
    lv_obj_set_style_text_font(s_sound_label, &lv_font_unscii_16, LV_PART_MAIN);
    lv_obj_center(s_sound_label);
    paint_sound(true);
}

static void update_ui(lv_timer_t *timer)
{
    (void)timer;
    device_t devices[MAX_DEVICES];
    size_t count;
    hunt_t hunt;

    taskENTER_CRITICAL(&s_lock);
    count = s_device_count;
    memcpy(devices, s_devices, count * sizeof(*devices));
    hunt = s_hunt;
    taskEXIT_CRITICAL(&s_lock);

    const int64_t now = esp_timer_get_time();
    size_t fresh = 0;
    for (size_t i = 0; i < count; ++i) {
        if (now - devices[i].seen_us > LIST_STALE_MS * 1000LL) {
            continue;
        }
        if (TARGET_NAME[0] != '\0' && !name_matches(devices[i].name)
            && !configured_addr_matches(&devices[i].addr)) {
            continue;
        }
        devices[fresh++] = devices[i];
    }
    count = fresh;
    qsort(devices, count, sizeof(*devices), strongest_first);

    if (count == 0) {
        clicker_update(false, 0);
        snprintf(s_face_rssi, sizeof(s_face_rssi), "--");
        s_face_fill = 0;
        s_face_live = false;
        lv_label_set_text(s_status_label, "scanning");
        lv_obj_invalidate(s_face);
        return;
    }

    const device_t *best = &devices[0];
    if (hunt.locked) {
        const device_t *held = NULL;
        for (size_t i = 0; i < count; ++i) {
            if (same_addr(&devices[i].addr, &hunt.addr)) {
                held = &devices[i];
                break;
            }
        }
        if (held != NULL && now - held->seen_us <= HOLD_MS * 1000LL
            && devices[0].rssi < held->rssi + SWITCH_DB) {
            best = held;
        }
    }
    const bool same = hunt.locked && same_addr(&hunt.addr, &best->addr);
    const float rssi_f = same ? hunt.rssi * 0.75f + best->rssi * 0.25f : best->rssi;
    const int rssi = (int)rssi_f;
    const bool packet_fresh = now - best->seen_us <= STALE_MS * 1000LL;

    if (!same) {
        hunt.spark_len = 0;
        hunt.spark_head = 0;
        hunt.previous_rssi = 0;
    }
    if (hunt.spark_len < SPARK_LEN) {
        hunt.spark[hunt.spark_len++] = (int8_t)rssi;
    } else {
        hunt.spark[hunt.spark_head] = (int8_t)rssi;
        hunt.spark_head = (uint8_t)((hunt.spark_head + 1) % SPARK_LEN);
    }
    hunt.previous_rssi = rssi;
    hunt.locked = true;
    hunt.addr = best->addr;
    hunt.rssi = rssi_f;
    hunt.seen_us = best->seen_us;
    snprintf(hunt.name, sizeof(hunt.name), "%s",
             best->name[0] ? best->name : (TARGET_NAME[0] ? TARGET_NAME : "unnamed"));

    taskENTER_CRITICAL(&s_lock);
    s_hunt.locked = hunt.locked;
    s_hunt.addr = hunt.addr;
    s_hunt.rssi = hunt.rssi;
    s_hunt.seen_us = hunt.seen_us;
    memcpy(s_hunt.name, hunt.name, sizeof(hunt.name));
    memcpy(s_hunt.spark, hunt.spark, sizeof(hunt.spark));
    s_hunt.spark_len = hunt.spark_len;
    s_hunt.spark_head = hunt.spark_head;
    s_hunt.previous_rssi = hunt.previous_rssi;
    taskEXIT_CRITICAL(&s_lock);

    clicker_update(packet_fresh, rssi);

    snprintf(s_face_rssi, sizeof(s_face_rssi), "%d", rssi);
    s_face_fill = proximity_strength(rssi);
    s_face_live = packet_fresh;
    lv_label_set_text(s_status_label, ui_status(rssi, !packet_fresh));
    lv_obj_invalidate(s_face);
}

static void release_v2_panel_reset(void)
{
    if (bsp_i2c_init() != ESP_OK) {
        return;
    }
    if (i2c_master_probe(bsp_i2c_get_handle(), BSP_IO_EXPANDER_I2C_ADDRESS, 50) != ESP_OK) {
        ESP_LOGI(TAG, "No TCA9554, skip expander reset");
        return;
    }

    esp_io_expander_handle_t expander = bsp_io_expander_init();
    if (expander == NULL) {
        ESP_LOGE(TAG, "TCA9554 init failed");
        return;
    }

    const uint32_t reset_pins =
        IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1 | IO_EXPANDER_PIN_NUM_2;
    ESP_ERROR_CHECK(esp_io_expander_set_dir(expander, reset_pins, IO_EXPANDER_OUTPUT));
    ESP_ERROR_CHECK(esp_io_expander_set_level(expander, reset_pins, 1));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(esp_io_expander_set_level(expander, reset_pins, 0));
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_ERROR_CHECK(esp_io_expander_set_level(expander, reset_pins, 1));
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "V2 panel reset released via TCA9554");
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    release_v2_panel_reset();
    if (bsp_display_start() == NULL) {
        ESP_LOGE(TAG, "Display initialization failed");
        return;
    }
    ESP_ERROR_CHECK(bsp_display_backlight_on());
    if (bsp_display_lock(1000)) {
        build_ui();
        lv_timer_create(update_ui, 250, NULL);
        bsp_display_unlock();
    } else {
        ESP_LOGE(TAG, "LVGL lock failed, UI not built");
    }

    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);

    clicker_start();
}
