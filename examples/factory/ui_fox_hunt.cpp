/**
 * @file      ui_fox_hunt.cpp
 * @license   MIT
 *
 * BLE Fox Hunt — pick a nearby BLE device and home in on it by signal
 * strength. Built on the factory card-page framework (ui_create_app_page)
 * and NimBLE observer scanning, like the BLE Scanner app.
 *
 * A large "signal" panel shows the proximity band and smoothed RSSI, and
 * pulses in the band colour (FREEZING blue -> COLD cyan -> WARM amber ->
 * HOT red) in place of status LEDs. Tap its left/right half to lower/raise
 * feedback volume. Feedback is a mix of haptic pulses and beeps whose
 * cadence and pitch track the band. The target dropdown chooses which
 * device to hunt (or Auto = strongest). Smoothing, range tuning, feedback
 * type and volume are adjustable.
 */
#include "ui_define.h"

#ifndef EXCLUDE_FOX_HUNT

#include <algorithm>
#include <cstring>

#ifdef ARDUINO
#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#if CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ROLE_OBSERVER
#define FOX_HUNT_HAS_NIMBLE 1
#endif
#endif

#ifndef FOX_HUNT_HAS_NIMBLE
#define FOX_HUNT_HAS_NIMBLE 0
#endif

#if FOX_HUNT_HAS_NIMBLE
#include <string>
using std::string;
#endif

#include <math.h>

/* ── Tunables ── */
#define FOX_UI_PERIOD_MS        100
#define FOX_STALE_MS            4000
#define FOX_LIST_REBUILD_MS     1500
#define FOX_MAX_DEVICES         48
#define FOX_MAX_PICK            16
#define FOX_VOL_MAX             5
#define FOX_VOL_DEFAULT         3

typedef enum {
    FOX_BAND_NONE = 0,
    FOX_BAND_FREEZING,
    FOX_BAND_COLD,
    FOX_BAND_WARM,
    FOX_BAND_HOT,
} fox_band_t;

typedef enum {
    FOX_FB_OFF = 0,
    FOX_FB_HAPTIC,
    FOX_FB_BEEP,
    FOX_FB_BOTH,
} fox_fb_t;

#define FOX_HOT_MIN_DBM         (-60)
#define FOX_WARM_MIN_DBM        (-74)
#define FOX_COLD_MIN_DBM        (-88)

/* Persist within a session (reset on reboot). */
static uint8_t fox_volume = FOX_VOL_DEFAULT;
static int8_t  fox_range_tune = 0;
static uint8_t fox_smoothing_idx = 1;
static fox_fb_t fox_feedback = FOX_FB_BOTH;

/* ── UI ── */
static lv_obj_t *fox_page = NULL;        /* content from ui_create_app_page */
static lv_obj_t *fox_signal_panel = NULL;
static lv_obj_t *fox_band_label = NULL;
static lv_obj_t *fox_rssi_label = NULL;
static lv_obj_t *fox_vol_label = NULL;
static lv_obj_t *fox_sub_label = NULL;
static lv_obj_t *fox_target_dd = NULL;
static lv_obj_t *fox_vol_slider = NULL;
static lv_timer_t *fox_timer = NULL;

/* ── Runtime ── */
static fox_band_t fox_band = FOX_BAND_NONE;
static float   fox_smoothed = -100.0f;
static bool    fox_have_smoothed = false;
static uint32_t fox_pulse_phase_ms = 0;
static uint32_t fox_last_tick_ms = 0;
static uint32_t fox_next_list_ms = 0;
static bool    fox_crest_latched = false;
static bool    fox_has_codec = false;
static bool    fox_has_haptic = false;
static uint32_t fox_next_ui_ms = 0;      /* throttle display writes */
static int     fox_last_dbm = 999;
static int     fox_last_sub_raw = 999;
static bool    fox_last_have = false;

#define FOX_UI_REFRESH_MS       250      /* how often labels repaint */

#if FOX_HUNT_HAS_NIMBLE
typedef struct {
    string   address;
    uint8_t  address_type;
    string   name;
    int8_t   rssi;
    uint32_t last_seen_ms;
} fox_device_t;

static SemaphoreHandle_t fox_mutex = NULL;
static vector<fox_device_t> fox_devices;
static string fox_target_addr;
static uint8_t fox_target_type = 0;
static volatile bool fox_app_active = false;
static vector<std::pair<string, uint8_t>> fox_pick_map;

static bool fox_lock(TickType_t ticks)
{
    return fox_mutex && xSemaphoreTake(fox_mutex, ticks) == pdTRUE;
}
static void fox_unlock(void)
{
    if (fox_mutex) {
        xSemaphoreGive(fox_mutex);
    }
}

static void fox_prune_locked(uint32_t now)
{
    fox_devices.erase(
        remove_if(fox_devices.begin(), fox_devices.end(), [now](const fox_device_t &d) {
            return (uint32_t)(now - d.last_seen_ms) > (FOX_STALE_MS * 3);
        }),
        fox_devices.end());
}

static void fox_upsert(const NimBLEAdvertisedDevice *device)
{
    if (!device || !fox_app_active) {
        return;
    }
    fox_device_t in = {};
    in.address = device->getAddress().toString();
    in.address_type = device->getAddressType();
    in.name = device->haveName() ? device->getName() : "";
    in.rssi = device->getRSSI();
    in.last_seen_ms = millis();

    if (!fox_lock(0)) {
        return;
    }
    fox_prune_locked(in.last_seen_ms);
    auto it = find_if(fox_devices.begin(), fox_devices.end(), [&in](const fox_device_t &d) {
        return d.address == in.address && d.address_type == in.address_type;
    });
    if (it == fox_devices.end()) {
        if (fox_devices.size() < FOX_MAX_DEVICES) {
            fox_devices.push_back(in);
        }
    } else {
        it->rssi = in.rssi;
        it->last_seen_ms = in.last_seen_ms;
        if (!in.name.empty()) {
            it->name = in.name;
        }
    }
    fox_unlock();
}

class FoxScanCallbacks : public NimBLEScanCallbacks {
    void onDiscovered(const NimBLEAdvertisedDevice *d) override { fox_upsert(d); }
    void onResult(const NimBLEAdvertisedDevice *d) override { fox_upsert(d); }
};
static FoxScanCallbacks fox_scan_cb;

static bool fox_start_scan(void)
{
    if (!NimBLEDevice::isInitialized()) {
        if (!NimBLEDevice::init("LilyGo Fox Hunt")) {
            return false;
        }
    }
    NimBLEScan *scan = NimBLEDevice::getScan();
    if (!scan) {
        return false;
    }
    scan->setScanCallbacks(&fox_scan_cb, false);
    scan->setActiveScan(true);
    scan->setInterval(80);
    scan->setWindow(80);
    scan->setMaxResults(0);
    return scan->start(0, false, true);
}

static void fox_stop_scan(void)
{
    fox_app_active = false;
    if (NimBLEDevice::isInitialized()) {
        NimBLEScan *scan = NimBLEDevice::getScan();
        if (scan) {
            scan->setScanCallbacks(NULL, false);
            if (scan->isScanning()) {
                scan->stop();
            }
            scan->clearResults();
        }
    }
}

static bool fox_target_rssi(int8_t *rssi_out, string *name_out)
{
    bool found = false;
    int8_t best = -128;
    uint32_t now = millis();
    if (!fox_lock(pdMS_TO_TICKS(10))) {
        return false;
    }
    if (fox_target_addr.empty()) {
        for (const auto &d : fox_devices) {
            if ((uint32_t)(now - d.last_seen_ms) > FOX_STALE_MS) {
                continue;
            }
            if (!found || d.rssi > best) {
                best = d.rssi;
                found = true;
                if (name_out) {
                    *name_out = d.name.empty() ? d.address : d.name;
                }
            }
        }
    } else {
        for (const auto &d : fox_devices) {
            if (d.address == fox_target_addr && d.address_type == fox_target_type) {
                if ((uint32_t)(now - d.last_seen_ms) <= FOX_STALE_MS) {
                    best = d.rssi;
                    found = true;
                    if (name_out) {
                        *name_out = d.name.empty() ? d.address : d.name;
                    }
                }
                break;
            }
        }
    }
    fox_unlock();
    if (found && rssi_out) {
        *rssi_out = best;
    }
    return found;
}

static void fox_rebuild_target_list(void)
{
    if (!fox_target_dd || !fox_lock(pdMS_TO_TICKS(10))) {
        return;
    }
    vector<fox_device_t> snap = fox_devices;
    fox_unlock();

    uint32_t now = millis();
    snap.erase(remove_if(snap.begin(), snap.end(), [now](const fox_device_t &d) {
        return (uint32_t)(now - d.last_seen_ms) > FOX_STALE_MS;
    }), snap.end());
    sort(snap.begin(), snap.end(), [](const fox_device_t &a, const fox_device_t &b) {
        return a.rssi > b.rssi;
    });
    if (snap.size() > FOX_MAX_PICK) {
        snap.resize(FOX_MAX_PICK);
    }

    string opts = "Auto (strongest)";
    fox_pick_map.clear();
    for (const auto &d : snap) {
        string label = d.name.empty() ? d.address : d.name;
        if (label.size() > 18) {
            label = label.substr(0, 18);
        }
        opts += "\n";
        opts += label;
        fox_pick_map.push_back({ d.address, d.address_type });
    }
    lv_dropdown_set_options(fox_target_dd, opts.c_str());

    uint16_t sel = 0;
    if (!fox_target_addr.empty()) {
        for (size_t i = 0; i < fox_pick_map.size(); i++) {
            if (fox_pick_map[i].first == fox_target_addr &&
                    fox_pick_map[i].second == fox_target_type) {
                sel = (uint16_t)(i + 1);
                break;
            }
        }
    }
    lv_dropdown_set_selected(fox_target_dd, sel);
}
#endif /* FOX_HUNT_HAS_NIMBLE */

/* ── Band helpers ── */
static uint32_t fox_band_color(fox_band_t band)
{
    switch (band) {
    case FOX_BAND_HOT:      return 0xFF3B30;
    case FOX_BAND_WARM:     return 0xFFB800;
    case FOX_BAND_COLD:     return 0x00C8D4;
    case FOX_BAND_FREEZING: return 0x1E6CFF;
    default:                return 0x3A3A3A;
    }
}
static const char *fox_band_text(fox_band_t band)
{
    switch (band) {
    case FOX_BAND_HOT:      return "HOT";
    case FOX_BAND_WARM:     return "WARM";
    case FOX_BAND_COLD:     return "COLD";
    case FOX_BAND_FREEZING: return "FREEZING";
    default:                return "SEARCHING";
    }
}
static uint32_t fox_band_period(fox_band_t band)
{
    switch (band) {
    case FOX_BAND_HOT:      return 300;
    case FOX_BAND_WARM:     return 650;
    case FOX_BAND_COLD:     return 1100;
    case FOX_BAND_FREEZING: return 1600;
    default:                return 2200;
    }
}
static uint16_t fox_band_freq(fox_band_t band)
{
    switch (band) {
    case FOX_BAND_HOT:      return 1500;
    case FOX_BAND_WARM:     return 950;
    case FOX_BAND_COLD:     return 650;
    case FOX_BAND_FREEZING: return 420;
    default:                return 0;
    }
}
static fox_band_t fox_band_from_rssi(float smoothed)
{
    int adj = (int)lroundf(smoothed) + fox_range_tune;
    if (adj >= FOX_HOT_MIN_DBM)  return FOX_BAND_HOT;
    if (adj >= FOX_WARM_MIN_DBM) return FOX_BAND_WARM;
    if (adj >= FOX_COLD_MIN_DBM) return FOX_BAND_COLD;
    return FOX_BAND_FREEZING;
}
static float fox_smoothing_alpha(void)
{
    switch (fox_smoothing_idx) {
    case 0:  return 0.55f;
    case 2:  return 0.15f;
    default: return 0.30f;
    }
}
static const char *fox_fb_text(fox_fb_t fb)
{
    switch (fb) {
    case FOX_FB_HAPTIC: return "Haptic";
    case FOX_FB_BEEP:   return "Beep";
    case FOX_FB_BOTH:   return "Buzz+Beep";
    default:            return "Off";
    }
}
/* ui_create_card_* helpers return the row, not the widget; the widget is the
 * row's last child. Fetch it so we can drive the dropdown/slider directly. */
static lv_obj_t *fox_item_widget(lv_obj_t *row)
{
    if (!row) {
        return NULL;
    }
    uint32_t n = lv_obj_get_child_count(row);
    return n ? lv_obj_get_child(row, n - 1) : NULL;
}

static uint32_t fox_scale_color(uint32_t rgb, float b)
{
    if (b < 0.0f) {
        b = 0.0f;
    }
    if (b > 1.0f) {
        b = 1.0f;
    }
    uint32_t r = (uint32_t)(((rgb >> 16) & 0xFF) * b);
    uint32_t g = (uint32_t)(((rgb >> 8) & 0xFF) * b);
    uint32_t bl = (uint32_t)((rgb & 0xFF) * b);
    return (r << 16) | (g << 8) | bl;
}

static void fox_update_vol_label(void)
{
    if (fox_vol_label) {
        if (fox_volume == 0) {
            lv_label_set_text(fox_vol_label, LV_SYMBOL_MUTE " 0/5");
        } else {
            lv_label_set_text_fmt(fox_vol_label, LV_SYMBOL_VOLUME_MAX " %u/%u  %s",
                                  (unsigned)fox_volume, (unsigned)FOX_VOL_MAX,
                                  fox_fb_text(fox_feedback));
        }
    }
}

static void fox_emit_feedback(fox_band_t band)
{
    if (fox_volume == 0 || band == FOX_BAND_NONE) {
        return;
    }
    static bool skip = false;
    if (fox_volume <= 2) {
        skip = !skip;
        if (skip) {
            return;
        }
    }
#ifdef ARDUINO
    bool want_haptic = (fox_feedback == FOX_FB_HAPTIC || fox_feedback == FOX_FB_BOTH);
    bool want_beep   = (fox_feedback == FOX_FB_BEEP   || fox_feedback == FOX_FB_BOTH);
    if (want_haptic && fox_has_haptic) {
        hw_feedback();
    }
    if (want_beep && fox_has_codec) {
        uint16_t freq = fox_band_freq(band);
        uint16_t dur = (uint16_t)(20 + fox_volume * 8);
        if (freq) {
            hw_audio_beep(freq, dur);
        }
    }
#else
    (void)band;
#endif
}

/* ── Event handlers ── */
static void fox_target_cb(lv_event_t *e)
{
    (void)e;
#if FOX_HUNT_HAS_NIMBLE
    uint16_t sel = lv_dropdown_get_selected(fox_target_dd);
    if (sel == 0) {
        fox_target_addr.clear();
        fox_target_type = 0;
    } else if ((size_t)(sel - 1) < fox_pick_map.size()) {
        fox_target_addr = fox_pick_map[sel - 1].first;
        fox_target_type = fox_pick_map[sel - 1].second;
    }
    fox_have_smoothed = false;
#endif
}

static void fox_vol_slider_cb(lv_event_t *e)
{
    lv_obj_t *sl = lv_event_get_target_obj(e);
    int v = lv_slider_get_value(sl);
    if (v < 0) {
        v = 0;
    }
    if (v > FOX_VOL_MAX) {
        v = FOX_VOL_MAX;
    }
    fox_volume = (uint8_t)v;
    fox_update_vol_label();
}

static void fox_feedback_cb(lv_event_t *e)
{
    lv_obj_t *dd = lv_event_get_target_obj(e);
    fox_feedback = (fox_fb_t)lv_dropdown_get_selected(dd);
    fox_update_vol_label();
}

static void fox_smoothing_cb(lv_event_t *e)
{
    lv_obj_t *dd = lv_event_get_target_obj(e);
    fox_smoothing_idx = (uint8_t)lv_dropdown_get_selected(dd);
}

static void fox_range_cb(lv_event_t *e)
{
    lv_obj_t *sl = lv_event_get_target_obj(e);
    fox_range_tune = (int8_t)lv_slider_get_value(sl);
}

/* Tap the signal panel's left/right half to change feedback volume. */
static void fox_panel_click_cb(lv_event_t *e)
{
    (void)e;
    lv_indev_t *indev = lv_indev_active();
    if (!indev || lv_indev_get_type(indev) != LV_INDEV_TYPE_POINTER || !fox_signal_panel) {
        return;
    }
    lv_point_t p;
    lv_indev_get_point(indev, &p);
    lv_area_t a;
    lv_obj_get_coords(fox_signal_panel, &a);
    int32_t mid = (a.x1 + a.x2) / 2;
    if (p.x < mid) {
        if (fox_volume > 0) {
            fox_volume--;
        }
    } else {
        if (fox_volume < FOX_VOL_MAX) {
            fox_volume++;
        }
    }
    if (fox_vol_slider) {
        lv_slider_set_value(fox_vol_slider, fox_volume, LV_ANIM_OFF);
    }
    fox_update_vol_label();
#ifdef ARDUINO
    if (fox_volume > 0 && fox_has_haptic) {
        hw_feedback();
    }
#endif
}

static void fox_cleanup(void);

static void fox_back_cb(lv_event_t *e)
{
    (void)e;
    fox_cleanup();
    if (fox_page) {
        ui_destroy_app_page(fox_page);
        fox_page = NULL;
    }
    fox_signal_panel = NULL;
    fox_band_label = fox_rssi_label = fox_vol_label = fox_sub_label = NULL;
    fox_target_dd = fox_vol_slider = NULL;
    menu_show();
}

/* ── Main tick ── */
static void fox_tick_cb(lv_timer_t *timer)
{
    (void)timer;
    uint32_t now = millis();
    uint32_t dt = fox_last_tick_ms ? (now - fox_last_tick_ms) : FOX_UI_PERIOD_MS;
    fox_last_tick_ms = now;

    int8_t raw = 0;
    bool have = false;
    string name;

#if FOX_HUNT_HAS_NIMBLE
    have = fox_target_rssi(&raw, &name);
    if (fox_target_dd && now >= fox_next_list_ms && !lv_dropdown_is_open(fox_target_dd)) {
        fox_rebuild_target_list();
        fox_next_list_ms = now + FOX_LIST_REBUILD_MS;
    }
#endif

    fox_band_t band;
    if (have) {
        float alpha = fox_smoothing_alpha();
        if (!fox_have_smoothed) {
            fox_smoothed = raw;
            fox_have_smoothed = true;
        } else {
            fox_smoothed += alpha * ((float)raw - fox_smoothed);
        }
        band = fox_band_from_rssi(fox_smoothed);
    } else {
        fox_have_smoothed = false;
        band = FOX_BAND_NONE;
    }

    /* Recolour the big panel only on a band change (a large repaint — doing it
     * every tick tears this partial-refresh display). */
    if (band != fox_band) {
        fox_band = band;
        uint32_t color = fox_band_color(band);
        if (fox_signal_panel) {
            lv_obj_set_style_bg_color(fox_signal_panel, lv_color_hex(fox_scale_color(color, 0.80f)), 0);
            lv_obj_set_style_border_color(fox_signal_panel, lv_color_hex(color), 0);
        }
        if (fox_band_label) {
            lv_label_set_text(fox_band_label, fox_band_text(band));
        }
    }

    /* Throttle text updates (small areas only) to a gentle rate. */
    bool ui_due = (now >= fox_next_ui_ms);
    if (ui_due) {
        fox_next_ui_ms = now + FOX_UI_REFRESH_MS;
        int dbm = have ? (int)lroundf(fox_smoothed) : 999;
        if (fox_rssi_label && dbm != fox_last_dbm) {
            if (have) {
                lv_label_set_text_fmt(fox_rssi_label, "%d dBm", dbm);
            } else {
                lv_label_set_text(fox_rssi_label, "-- dBm");
            }
            fox_last_dbm = dbm;
        }
        if (fox_sub_label && ((int)raw != fox_last_sub_raw || have != fox_last_have)) {
            if (have) {
                lv_label_set_text_fmt(fox_sub_label, "%s  (raw %d)",
                                      name.empty() ? "target" : name.c_str(), (int)raw);
            } else {
                lv_label_set_text(fox_sub_label, "no signal - move around");
            }
            fox_last_sub_raw = (int)raw;
            fox_last_have = have;
        }
    }

    /* Feedback cadence tracks the band (no display work). */
    uint32_t period = fox_band_period(band);
    fox_pulse_phase_ms += dt;
    bool crest = false;
    if (fox_pulse_phase_ms >= period) {
        fox_pulse_phase_ms -= period;
        crest = true;
    }
    if (crest && !fox_crest_latched) {
        fox_emit_feedback(band);
    }
    fox_crest_latched = crest;
}

static void fox_cleanup(void)
{
    if (fox_timer) {
        lv_timer_delete(fox_timer);
        fox_timer = NULL;
    }
#if FOX_HUNT_HAS_NIMBLE
    fox_stop_scan();
    if (fox_lock(pdMS_TO_TICKS(20))) {
        fox_devices.clear();
        fox_unlock();
    }
    fox_pick_map.clear();
#endif
}

/* ── App entry ── */
void ui_fox_hunt_enter(lv_obj_t *parent)
{
    fox_band = FOX_BAND_NONE;
    fox_have_smoothed = false;
    fox_pulse_phase_ms = 0;
    fox_last_tick_ms = 0;
    fox_next_list_ms = 0;
    fox_crest_latched = false;
    fox_next_ui_ms = 0;
    fox_last_dbm = 999;
    fox_last_sub_raw = 999;
    fox_last_have = false;

#ifdef ARDUINO
    uint32_t online = hw_get_device_online();
    fox_has_codec = (online & HW_CODEC_ONLINE) != 0;
    fox_has_haptic = (online & HW_DRV_ONLINE) != 0;
#endif

    fox_page = ui_create_app_page(parent, "Fox Hunt", fox_back_cb);

    /* Big signal panel (the colour indicator). Tap left/right = volume. */
    fox_signal_panel = lv_obj_create(fox_page);
    lv_obj_set_width(fox_signal_panel, LV_PCT(100));
    lv_obj_set_height(fox_signal_panel, 132);
    lv_obj_remove_flag(fox_signal_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(fox_signal_panel, 14, 0);
    lv_obj_set_style_border_width(fox_signal_panel, 3, 0);
    lv_obj_set_style_border_color(fox_signal_panel, lv_color_hex(fox_band_color(FOX_BAND_NONE)), 0);
    lv_obj_set_style_bg_color(fox_signal_panel, lv_color_hex(fox_scale_color(fox_band_color(FOX_BAND_NONE), 0.55f)), 0);
    lv_obj_set_style_bg_opa(fox_signal_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(fox_signal_panel, 6, 0);
    lv_obj_add_flag(fox_signal_panel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(fox_signal_panel, fox_panel_click_cb, LV_EVENT_SHORT_CLICKED, NULL);

    fox_band_label = lv_label_create(fox_signal_panel);
    lv_label_set_text(fox_band_label, fox_band_text(FOX_BAND_NONE));
    lv_obj_set_style_text_font(fox_band_label, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(fox_band_label, lv_color_white(), 0);
    lv_obj_align(fox_band_label, LV_ALIGN_TOP_MID, 0, 2);

    fox_rssi_label = lv_label_create(fox_signal_panel);
    lv_label_set_text(fox_rssi_label, "-- dBm");
    lv_obj_set_style_text_font(fox_rssi_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(fox_rssi_label, lv_color_white(), 0);
    lv_obj_align(fox_rssi_label, LV_ALIGN_CENTER, 0, 10);

    fox_vol_label = lv_label_create(fox_signal_panel);
    lv_obj_set_style_text_font(fox_vol_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(fox_vol_label, lv_color_white(), 0);
    lv_obj_align(fox_vol_label, LV_ALIGN_BOTTOM_MID, 0, 0);

    /* Target */
    lv_obj_t *target_card = ui_create_card(fox_page, "Target");
    fox_target_dd = fox_item_widget(
        ui_create_card_dropdown(target_card, LV_SYMBOL_BLUETOOTH, "Hunt",
                                "Auto (strongest)", 0, fox_target_cb));
    fox_sub_label = lv_label_create(target_card);
    lv_label_set_text(fox_sub_label, "no signal - move around");
    lv_label_set_long_mode(fox_sub_label, LV_LABEL_LONG_DOT);
    lv_obj_set_width(fox_sub_label, LV_PCT(100));
    lv_obj_set_style_text_font(fox_sub_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(fox_sub_label, UI_COLOR_TEXT_SECONDARY, 0);

    /* Feedback */
    lv_obj_t *fb_card = ui_create_card(fox_page, "Feedback");
    fox_vol_slider = fox_item_widget(
        ui_create_card_slider(fb_card, LV_SYMBOL_VOLUME_MAX, "Volume",
                              0, FOX_VOL_MAX, fox_volume, fox_vol_slider_cb));
    ui_create_card_dropdown(fb_card, LV_SYMBOL_BELL, "Type",
                            "Off\nHaptic\nBeep\nBuzz+Beep", (uint8_t)fox_feedback, fox_feedback_cb);

    /* Tuning */
    lv_obj_t *tune_card = ui_create_card(fox_page, "Tuning");
    ui_create_card_dropdown(tune_card, LV_SYMBOL_SETTINGS, "Smoothing",
                            "Low\nMedium\nHigh", fox_smoothing_idx, fox_smoothing_cb);
    ui_create_card_slider(tune_card, LV_SYMBOL_SETTINGS, "Range dB",
                          -10, 10, fox_range_tune, fox_range_cb);

    fox_update_vol_label();

#if FOX_HUNT_HAS_NIMBLE
    if (!fox_mutex) {
        fox_mutex = xSemaphoreCreateMutex();
    }
    if (fox_lock(pdMS_TO_TICKS(20))) {
        fox_devices.clear();
        fox_unlock();
    }
    fox_target_addr.clear();
    fox_target_type = 0;
    fox_app_active = true;
    if (!fox_start_scan() && fox_sub_label) {
        lv_label_set_text(fox_sub_label, "BLE scan failed to start");
    }
    fox_timer = lv_timer_create(fox_tick_cb, FOX_UI_PERIOD_MS, NULL);
#else
    if (fox_sub_label) {
        lv_label_set_text(fox_sub_label, "BLE scan not available in this build");
    }
#endif
}

void ui_fox_hunt_exit(lv_obj_t *parent)
{
    (void)parent;
    fox_cleanup();
}

app_t ui_fox_hunt_main = {
    .setup_func_cb = ui_fox_hunt_enter,
    .exit_func_cb = ui_fox_hunt_exit,
    .user_data = nullptr,
};

#endif /* EXCLUDE_FOX_HUNT */
