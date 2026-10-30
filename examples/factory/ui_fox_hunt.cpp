/**
 * @file      ui_fox_hunt.cpp
 * @license   MIT
 *
 * BLE Fox Hunt — pick a nearby BLE beacon and home in on it by signal
 * strength. Built on NimBLE observer scanning, like the BLE Scanner app.
 *
 * Interface (round 240x240 touch display, few physical buttons):
 *   - A coloured glow around the edge of the screen replaces status LEDs.
 *     It fades inward and is tinted by the proximity band, pulsing faster
 *     the closer you are: FREEZING (blue) -> COLD (cyan) -> WARM (amber)
 *     -> HOT (red).
 *   - Tap the left third of the screen to lower feedback volume, the right
 *     third to raise it. 0 = silent (glow only).
 *   - Swipe in from the left or right bezel to go back.
 *   - A gear (top-right) opens smoothing / range / feedback settings.
 *
 * Feedback is a mix of haptic pulses and beeps, each firing on the glow
 * crest; cadence and pitch track the proximity band.
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
#define FOX_UI_PERIOD_MS        80       /* glow / feedback tick */
#define FOX_STALE_MS            4000     /* target lost after this */
#define FOX_LIST_REBUILD_MS     1500     /* target dropdown refresh */
#define FOX_MAX_DEVICES         48
#define FOX_MAX_PICK            16       /* devices listed in dropdown */
#define FOX_EDGE_MARGIN         26       /* bezel kept clear for swipe */
#define FOX_VOL_MAX             5
#define FOX_VOL_DEFAULT         3

/* Proximity bands, strongest last. */
typedef enum {
    FOX_BAND_NONE = 0,   /* no / stale signal */
    FOX_BAND_FREEZING,
    FOX_BAND_COLD,
    FOX_BAND_WARM,
    FOX_BAND_HOT,
} fox_band_t;

/* Feedback mix. */
typedef enum {
    FOX_FB_OFF = 0,
    FOX_FB_HAPTIC,
    FOX_FB_BEEP,
    FOX_FB_BOTH,
} fox_fb_t;

/* Default band edges in dBm (applied to the smoothed, range-adjusted RSSI). */
#define FOX_HOT_MIN_DBM         (-60)
#define FOX_WARM_MIN_DBM        (-74)
#define FOX_COLD_MIN_DBM        (-88)

/* ── Persist within a session (static), reset on reboot. ── */
static uint8_t fox_volume = FOX_VOL_DEFAULT;      /* 0..FOX_VOL_MAX */
static int8_t  fox_range_tune = 0;                /* -10..+10 dB */
static uint8_t fox_smoothing_idx = 1;             /* 0 Low, 1 Med, 2 High */
static fox_fb_t fox_feedback = FOX_FB_BOTH;

/* ── UI objects ── */
static lv_obj_t *fox_root = NULL;
static lv_obj_t *fox_glow[4] = { NULL, NULL, NULL, NULL }; /* top,bottom,left,right */
static lv_obj_t *fox_back_btn = NULL;
static lv_obj_t *fox_gear_btn = NULL;
static lv_obj_t *fox_target_dd = NULL;
static lv_obj_t *fox_band_label = NULL;
static lv_obj_t *fox_rssi_label = NULL;
static lv_obj_t *fox_sub_label = NULL;
static lv_obj_t *fox_vol_label = NULL;
static lv_obj_t *fox_fb_label = NULL;
static lv_obj_t *fox_settings = NULL;   /* modal overlay, NULL when closed */
static lv_timer_t *fox_timer = NULL;

/* ── Runtime state (UI thread) ── */
static fox_band_t fox_band = FOX_BAND_NONE;
static float   fox_smoothed = -100.0f;
static bool    fox_have_smoothed = false;
static uint32_t fox_pulse_phase_ms = 0;
static uint32_t fox_last_tick_ms = 0;
static uint32_t fox_next_list_ms = 0;
static bool    fox_crest_latched = false;
static bool    fox_has_codec = false;
static bool    fox_has_haptic = false;

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
static string fox_target_addr;           /* empty => auto (strongest) */
static uint8_t fox_target_type = 0;
static volatile bool fox_app_active = false;
static volatile bool fox_scan_running = false;

/* index in dropdown (minus the Auto row) -> device address/type */
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
    scan->setInterval(80);   /* 0.625ms units -> 50ms */
    scan->setWindow(80);     /* 100% duty, catch ~250ms beacons fast */
    scan->setMaxResults(0);  /* callback-only, no result cache */
    /* duration 0 => scan until stopped */
    bool ok = scan->start(0, false, true);
    fox_scan_running = ok;
    return ok;
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
    fox_scan_running = false;
}

/* Latest RSSI for the active target (or strongest in auto mode). Returns
 * false if nothing suitable has been seen recently. */
static bool fox_target_rssi(int8_t *rssi_out, string *name_out)
{
    bool found = false;
    int8_t best = -128;
    uint32_t now = millis();
    if (!fox_lock(pdMS_TO_TICKS(10))) {
        return false;
    }
    if (fox_target_addr.empty()) {
        /* auto: strongest fresh device */
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

/* Rebuild the "\n"-separated dropdown options from the device store. */
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

    /* keep the user's current selection highlighted */
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

/* ── Band helpers (available in both builds) ── */
static uint32_t fox_band_color(fox_band_t band)
{
    switch (band) {
    case FOX_BAND_HOT:      return 0xFF3B30;
    case FOX_BAND_WARM:     return 0xFFB800;
    case FOX_BAND_COLD:     return 0x00C8D4;
    case FOX_BAND_FREEZING: return 0x1E6CFF;
    default:                return 0x303030;
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

/* Pulse period (full cycle) in ms — faster when hotter. */
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
    case 0:  return 0.55f;  /* Low smoothing: fast, jumpy */
    case 2:  return 0.15f;  /* High smoothing: steady, laggy */
    default: return 0.30f;  /* Medium */
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

/* Paint one edge glow panel with a colour->transparent fade toward centre. */
static void fox_style_glow(lv_obj_t *panel, uint8_t edge, uint32_t color)
{
    lv_color_t c = lv_color_hex(color);
    lv_obj_set_style_bg_color(panel, c, 0);
    lv_obj_set_style_bg_grad_color(panel, c, 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_main_stop(panel, 0, 0);
    lv_obj_set_style_bg_grad_stop(panel, 255, 0);

    switch (edge) {
    case 0: /* top: opaque at top edge, fading down */
        lv_obj_set_style_bg_grad_dir(panel, LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_bg_main_opa(panel, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_grad_opa(panel, LV_OPA_TRANSP, 0);
        break;
    case 1: /* bottom: opaque at bottom edge, fading up */
        lv_obj_set_style_bg_grad_dir(panel, LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_bg_main_opa(panel, LV_OPA_TRANSP, 0);
        lv_obj_set_style_bg_grad_opa(panel, LV_OPA_COVER, 0);
        break;
    case 2: /* left: opaque at left edge, fading right */
        lv_obj_set_style_bg_grad_dir(panel, LV_GRAD_DIR_HOR, 0);
        lv_obj_set_style_bg_main_opa(panel, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_grad_opa(panel, LV_OPA_TRANSP, 0);
        break;
    default: /* right: opaque at right edge, fading left */
        lv_obj_set_style_bg_grad_dir(panel, LV_GRAD_DIR_HOR, 0);
        lv_obj_set_style_bg_main_opa(panel, LV_OPA_TRANSP, 0);
        lv_obj_set_style_bg_grad_opa(panel, LV_OPA_COVER, 0);
        break;
    }
}

static void fox_set_glow_color(uint32_t color)
{
    for (uint8_t i = 0; i < 4; i++) {
        if (fox_glow[i]) {
            lv_color_t c = lv_color_hex(color);
            lv_obj_set_style_bg_color(fox_glow[i], c, 0);
            lv_obj_set_style_bg_grad_color(fox_glow[i], c, 0);
        }
    }
}

static void fox_set_glow_opa(lv_opa_t opa)
{
    for (uint8_t i = 0; i < 4; i++) {
        if (fox_glow[i]) {
            lv_obj_set_style_opa(fox_glow[i], opa, 0);
        }
    }
}

static void fox_update_vol_label(void)
{
    if (fox_vol_label) {
        lv_label_set_text_fmt(fox_vol_label, LV_SYMBOL_VOLUME_MAX " %u/%u",
                              (unsigned)fox_volume, (unsigned)FOX_VOL_MAX);
    }
    if (fox_fb_label) {
        lv_label_set_text(fox_fb_label, fox_volume == 0 ? "Silent" : fox_fb_text(fox_feedback));
    }
}

/* Fire haptic/beep on a glow crest, honouring type + volume. */
static void fox_emit_feedback(fox_band_t band)
{
    if (fox_volume == 0 || band == FOX_BAND_NONE) {
        return;
    }
    /* At low volume, skip every other crest to thin the cadence out. */
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
        uint16_t dur = (uint16_t)(20 + fox_volume * 8); /* 28..60ms */
        if (freq) {
            hw_audio_beep(freq, dur);
        }
    }
#else
    (void)band;
#endif
}

/* ── Settings modal ── */
static void fox_settings_close_cb(lv_event_t *e);
static void fox_smoothing_cb(lv_event_t *e);
static void fox_range_cb(lv_event_t *e);
static void fox_feedback_cb(lv_event_t *e);

static void fox_open_settings(void)
{
    if (fox_settings || !fox_root) {
        return;
    }
    fox_settings = lv_obj_create(fox_root);
    lv_obj_set_size(fox_settings, 212, 224);
    lv_obj_center(fox_settings);
    lv_obj_set_style_bg_color(fox_settings, lv_color_hex(0x101010), 0);
    lv_obj_set_style_bg_opa(fox_settings, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(fox_settings, UI_COLOR_DIVIDER, 0);
    lv_obj_set_style_border_width(fox_settings, 1, 0);
    lv_obj_set_style_radius(fox_settings, 12, 0);
    lv_obj_set_style_pad_all(fox_settings, 10, 0);
    lv_obj_set_flex_flow(fox_settings, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(fox_settings, 8, 0);

    lv_obj_t *title = lv_label_create(fox_settings);
    lv_label_set_text(title, "Fox Hunt Settings");
    lv_obj_set_style_text_color(title, UI_COLOR_TEXT_PRIMARY, 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);

    lv_obj_t *l1 = lv_label_create(fox_settings);
    lv_label_set_text(l1, "Smoothing");
    lv_obj_set_style_text_color(l1, UI_COLOR_TEXT_SECONDARY, 0);
    lv_obj_set_style_text_font(l1, &lv_font_montserrat_12, 0);
    lv_obj_t *sm = lv_dropdown_create(fox_settings);
    lv_dropdown_set_options(sm, "Low\nMedium\nHigh");
    lv_dropdown_set_selected(sm, fox_smoothing_idx);
    lv_obj_set_width(sm, LV_PCT(100));
    lv_obj_add_event_cb(sm, fox_smoothing_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *l2 = lv_label_create(fox_settings);
    lv_label_set_text_fmt(l2, "Range tune: %+d dB", fox_range_tune);
    lv_obj_set_style_text_color(l2, UI_COLOR_TEXT_SECONDARY, 0);
    lv_obj_set_style_text_font(l2, &lv_font_montserrat_12, 0);
    lv_obj_t *rng = lv_slider_create(fox_settings);
    lv_slider_set_range(rng, -10, 10);
    lv_slider_set_value(rng, fox_range_tune, LV_ANIM_OFF);
    lv_obj_set_width(rng, LV_PCT(100));
    lv_obj_add_event_cb(rng, fox_range_cb, LV_EVENT_VALUE_CHANGED, l2);

    lv_obj_t *l3 = lv_label_create(fox_settings);
    lv_label_set_text(l3, "Feedback");
    lv_obj_set_style_text_color(l3, UI_COLOR_TEXT_SECONDARY, 0);
    lv_obj_set_style_text_font(l3, &lv_font_montserrat_12, 0);
    lv_obj_t *fb = lv_dropdown_create(fox_settings);
    lv_dropdown_set_options(fb, "Off\nHaptic\nBeep\nBuzz+Beep");
    lv_dropdown_set_selected(fb, (uint16_t)fox_feedback);
    lv_obj_set_width(fb, LV_PCT(100));
    lv_obj_add_event_cb(fb, fox_feedback_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *close = lv_btn_create(fox_settings);
    lv_obj_set_width(close, LV_PCT(100));
    lv_obj_set_style_bg_color(close, UI_COLOR_ACCENT, 0);
    lv_obj_add_event_cb(close, fox_settings_close_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *cl = lv_label_create(close);
    lv_label_set_text(cl, "Close");
    lv_obj_center(cl);
}

static void fox_settings_close_cb(lv_event_t *e)
{
    (void)e;
    if (fox_settings) {
        lv_obj_del(fox_settings);
        fox_settings = NULL;
    }
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
    lv_obj_t *lbl = (lv_obj_t *)lv_event_get_user_data(e);
    if (lbl) {
        lv_label_set_text_fmt(lbl, "Range tune: %+d dB", fox_range_tune);
    }
}

static void fox_feedback_cb(lv_event_t *e)
{
    lv_obj_t *dd = lv_event_get_target_obj(e);
    fox_feedback = (fox_fb_t)lv_dropdown_get_selected(dd);
}

/* ── Event handlers ── */
static void fox_gear_cb(lv_event_t *e)
{
    (void)e;
    fox_open_settings();
}

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
    fox_have_smoothed = false; /* restart smoothing for the new target */
#endif
}

/* Tap the left/right third (away from the bezel) to change volume. */
static void fox_root_click_cb(lv_event_t *e)
{
    if (fox_settings) {
        return; /* modal open — ignore background taps */
    }
    lv_indev_t *indev = lv_indev_active();
    if (!indev || lv_indev_get_type(indev) != LV_INDEV_TYPE_POINTER) {
        return;
    }
    lv_point_t p;
    lv_indev_get_point(indev, &p);

    int32_t w = lv_obj_get_width(fox_root);
    int32_t h = lv_obj_get_height(fox_root);
    if (w <= 0) {
        w = 240;
    }
    if (h <= 0) {
        h = 240;
    }
    /* keep the top bar and the bezel swipe margins out of it */
    if (p.y < 40 || p.y > h - 24) {
        return;
    }
    if (p.x < FOX_EDGE_MARGIN || p.x > w - FOX_EDGE_MARGIN) {
        return;
    }

    if (p.x < w / 2 - 8) {
        if (fox_volume > 0) {
            fox_volume--;
        }
    } else if (p.x > w / 2 + 8) {
        if (fox_volume < FOX_VOL_MAX) {
            fox_volume++;
        }
    } else {
        return;
    }
    fox_update_vol_label();
#ifdef ARDUINO
    if (fox_volume > 0 && fox_has_haptic) {
        hw_feedback(); /* confirm the change */
    }
#endif
}

static void fox_cleanup(void);

static void fox_back_cb(lv_event_t *e)
{
    (void)e;
    fox_cleanup();
    if (fox_root) {
        lv_obj_del(fox_root);
        fox_root = NULL;
    }
    for (uint8_t i = 0; i < 4; i++) {
        fox_glow[i] = NULL;
    }
    fox_back_btn = fox_gear_btn = fox_target_dd = NULL;
    fox_band_label = fox_rssi_label = fox_sub_label = NULL;
    fox_vol_label = fox_fb_label = fox_settings = NULL;
    menu_show();
}

/* ── Main tick: update smoothing, band, glow, feedback ── */
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

    if (fox_target_dd && now >= fox_next_list_ms &&
            !lv_dropdown_is_open(fox_target_dd)) {
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

    if (band != fox_band) {
        fox_band = band;
        fox_set_glow_color(fox_band_color(band));
        if (fox_band_label) {
            lv_label_set_text(fox_band_label, fox_band_text(band));
            lv_obj_set_style_text_color(fox_band_label, lv_color_hex(fox_band_color(band)), 0);
        }
    }

    if (fox_rssi_label) {
        if (have) {
            lv_label_set_text_fmt(fox_rssi_label, "%d dBm", (int)lroundf(fox_smoothed));
        } else {
            lv_label_set_text(fox_rssi_label, "-- dBm");
        }
    }
    if (fox_sub_label) {
        if (have) {
            lv_label_set_text_fmt(fox_sub_label, "%s  (raw %d)",
                                  name.empty() ? "target" : name.c_str(), (int)raw);
        } else {
            lv_label_set_text(fox_sub_label, "no signal — move around");
        }
    }

    /* Pulse the glow; cadence tracks the band. */
    uint32_t period = fox_band_period(band);
    fox_pulse_phase_ms += dt;
    bool crest = false;
    if (fox_pulse_phase_ms >= period) {
        fox_pulse_phase_ms -= period;
        crest = true;
    }
    float phase = (float)fox_pulse_phase_ms / (float)period;      /* 0..1 */
    float wave = 0.5f - 0.5f * cosf(phase * 2.0f * (float)M_PI);  /* 0..1, crest at phase 0/1 */

    lv_opa_t opa;
    if (band == FOX_BAND_NONE) {
        opa = (lv_opa_t)70; /* faint steady grey while searching */
    } else {
        /* brighter and deeper pulse the hotter we are */
        int base = 60 + (int)band * 20;      /* 80..140 */
        int amp = 60 + (int)band * 15;        /* 75..120 */
        int v = base + (int)(amp * wave);
        if (v > 255) {
            v = 255;
        }
        opa = (lv_opa_t)v;
    }
    fox_set_glow_opa(opa);

    /* Feedback on the crest. */
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

#ifdef ARDUINO
    uint32_t online = hw_get_device_online();
    fox_has_codec = (online & HW_CODEC_ONLINE) != 0;
    fox_has_haptic = (online & HW_DRV_ONLINE) != 0;
#endif

    /* Full-screen root (no card page — this is an immersive view). */
    fox_root = lv_obj_create(parent);
    lv_obj_set_size(fox_root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(fox_root, UI_COLOR_BG, 0);
    lv_obj_set_style_bg_opa(fox_root, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(fox_root, 0, 0);
    lv_obj_set_style_radius(fox_root, 0, 0);
    lv_obj_set_style_pad_all(fox_root, 0, 0);
    lv_obj_remove_flag(fox_root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(fox_root, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(fox_root, fox_root_click_cb, LV_EVENT_SHORT_CLICKED, NULL);

    int32_t sw = lv_display_get_horizontal_resolution(NULL);
    int32_t sh = lv_display_get_vertical_resolution(NULL);
    if (sw <= 0) {
        sw = 240;
    }
    if (sh <= 0) {
        sh = 240;
    }

    /* Edge glow panels (created first => behind everything, non-clickable). */
    const uint8_t edges[4] = { 0, 1, 2, 3 };
    const lv_align_t aligns[4] = { LV_ALIGN_TOP_MID, LV_ALIGN_BOTTOM_MID,
                                   LV_ALIGN_LEFT_MID, LV_ALIGN_RIGHT_MID };
    for (uint8_t i = 0; i < 4; i++) {
        lv_obj_t *g = lv_obj_create(fox_root);
        lv_obj_remove_flag(g, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_remove_flag(g, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_border_width(g, 0, 0);
        lv_obj_set_style_radius(g, 0, 0);
        lv_obj_set_style_pad_all(g, 0, 0);
        if (i < 2) {
            lv_obj_set_size(g, sw, (sh * 42) / 100);
        } else {
            lv_obj_set_size(g, (sw * 42) / 100, sh);
        }
        lv_obj_align(g, aligns[i], 0, 0);
        fox_style_glow(g, edges[i], fox_band_color(FOX_BAND_NONE));
        lv_obj_set_style_opa(g, 70, 0);
        fox_glow[i] = g;
    }

    /* Top bar: back, target dropdown, gear. */
    fox_back_btn = lv_btn_create(fox_root);
    lv_obj_set_size(fox_back_btn, 30, 30);
    lv_obj_align(fox_back_btn, LV_ALIGN_TOP_LEFT, 4, 4);
    lv_obj_set_style_radius(fox_back_btn, 15, 0);
    lv_obj_set_style_bg_color(fox_back_btn, lv_color_hex(0x202020), 0);
    lv_obj_set_style_border_width(fox_back_btn, 1, 0);
    lv_obj_set_style_border_color(fox_back_btn, UI_COLOR_DIVIDER, 0);
    lv_obj_add_event_cb(fox_back_btn, fox_back_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *bl = lv_label_create(fox_back_btn);
    lv_label_set_text(bl, LV_SYMBOL_LEFT);
    lv_obj_center(bl);

    fox_gear_btn = lv_btn_create(fox_root);
    lv_obj_set_size(fox_gear_btn, 30, 30);
    lv_obj_align(fox_gear_btn, LV_ALIGN_TOP_RIGHT, -4, 4);
    lv_obj_set_style_radius(fox_gear_btn, 15, 0);
    lv_obj_set_style_bg_color(fox_gear_btn, lv_color_hex(0x202020), 0);
    lv_obj_set_style_border_width(fox_gear_btn, 1, 0);
    lv_obj_set_style_border_color(fox_gear_btn, UI_COLOR_DIVIDER, 0);
    lv_obj_add_event_cb(fox_gear_btn, fox_gear_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *gl = lv_label_create(fox_gear_btn);
    lv_label_set_text(gl, LV_SYMBOL_SETTINGS);
    lv_obj_center(gl);

    fox_target_dd = lv_dropdown_create(fox_root);
    lv_dropdown_set_options(fox_target_dd, "Auto (strongest)");
    lv_obj_set_width(fox_target_dd, sw - 80);
    lv_obj_set_height(fox_target_dd, 30);
    lv_obj_align(fox_target_dd, LV_ALIGN_TOP_MID, 0, 4);
    lv_obj_set_style_pad_ver(fox_target_dd, 4, 0);
    lv_obj_set_style_text_font(fox_target_dd, &lv_font_montserrat_12, 0);
    lv_obj_add_event_cb(fox_target_dd, fox_target_cb, LV_EVENT_VALUE_CHANGED, NULL);

    /* Centre readout. */
    fox_band_label = lv_label_create(fox_root);
    lv_label_set_text(fox_band_label, fox_band_text(FOX_BAND_NONE));
    lv_obj_set_style_text_font(fox_band_label, &lv_font_montserrat_40, 0);
    lv_obj_set_style_text_color(fox_band_label, lv_color_hex(fox_band_color(FOX_BAND_NONE)), 0);
    lv_obj_align(fox_band_label, LV_ALIGN_CENTER, 0, -22);

    fox_rssi_label = lv_label_create(fox_root);
    lv_label_set_text(fox_rssi_label, "-- dBm");
    lv_obj_set_style_text_font(fox_rssi_label, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(fox_rssi_label, UI_COLOR_TEXT_PRIMARY, 0);
    lv_obj_align(fox_rssi_label, LV_ALIGN_CENTER, 0, 20);

    fox_sub_label = lv_label_create(fox_root);
    lv_label_set_text(fox_sub_label, "no signal — move around");
    lv_label_set_long_mode(fox_sub_label, LV_LABEL_LONG_DOT);
    lv_obj_set_width(fox_sub_label, sw - 2 * FOX_EDGE_MARGIN);
    lv_obj_set_style_text_align(fox_sub_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(fox_sub_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(fox_sub_label, UI_COLOR_TEXT_SECONDARY, 0);
    lv_obj_align(fox_sub_label, LV_ALIGN_CENTER, 0, 52);

    /* Bottom status: volume (left) and feedback type (right). */
    fox_vol_label = lv_label_create(fox_root);
    lv_obj_set_style_text_font(fox_vol_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(fox_vol_label, UI_COLOR_TEXT_PRIMARY, 0);
    lv_obj_align(fox_vol_label, LV_ALIGN_BOTTOM_LEFT, 10, -10);

    fox_fb_label = lv_label_create(fox_root);
    lv_obj_set_style_text_font(fox_fb_label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(fox_fb_label, UI_COLOR_TEXT_SECONDARY, 0);
    lv_obj_align(fox_fb_label, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    fox_update_vol_label();

    /* Bezel swipe to go back. */
    ui_enable_edge_swipe_back(fox_root, fox_back_btn);

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
    if (!fox_start_scan()) {
        if (fox_sub_label) {
            lv_label_set_text(fox_sub_label, "BLE scan failed to start");
        }
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
