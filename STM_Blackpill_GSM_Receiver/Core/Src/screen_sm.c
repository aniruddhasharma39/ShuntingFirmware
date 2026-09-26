#include "screen_sm.h"
#include "hmi_state.h"
#include "hmi_map.h"
#include "dwin_hmi.h"
#include "gsm_mqtt.h"
#include "lora_e220.h"
#include "buzzer.h"
#include <stdio.h>
#include <string.h>

/* ---- timing constants (see CLAUDE.md screen-flow section in the source
 * project for the ones that are spec, not just demo pacing: originally 3s
 * startup hold, 20s connect timeout, 2s shunting-completed hold) ------- */
#define STARTUP_HOLD_MS               5000u  /* 5s rather than the 3s spec:
    the battery boot-time voltage read (battery_soc.c) needs ~3.5s after
    power-on to publish a real percentage, and the status bar (which shows
    it) only starts being pushed once this screen is left — a 3s hold would
    let it flash the placeholder 0% for a moment. Also long enough for the
    modem to get past its worst boot noise before the first AWS attempt
    (see main.c, which holds that attempt until this screen is left). */
#define CONNECTING_TIMEOUT_MS        20000u
#define SHUNTING_COMPLETED_HOLD_MS    2000u
#define ERROR_HOLD_MS                 3000u

/* LoRa-priority connect race: the AWS/GSM path typically becomes ready
 * (a heartbeat/telemetry message from the selected transmitter) within a
 * second or two once the MQTT session is up, while LoRa's rolling window
 * needs a mandatory minimum of LORA_WINDOW_SIZE * LORA_SLOT_INTERVAL_MS
 * (5 * 1500 = 7500ms, in lora_e220.c) before it can ever report
 * LORA_LINK_STABLE, however good the signal is. Without a grace window the
 * cloud link would win the race almost every time regardless of "LoRa
 * checked first" in the code, which is the opposite of what's wanted:
 * LoRa is the preferred link whenever the selected device is actually in
 * range, with AWS/GSM only as the fallback for when it isn't. See
 * ConnectionIsReady(). */
#define LORA_PRIORITY_GRACE_MS       10000u  /* generous margin over LoRa's
    own ~7500ms minimum fill time — a genuinely in-range device reaches
    STABLE within this window and wins; the cloud link is never accepted
    before it, as long as LoRa has shown ANY presence for this device */
#define LORA_NO_SIGNAL_GRACE_MS       3000u  /* if LoRa hasn't heard a
    single frame from the selected device within this much shorter
    window, it's reasonably certain that device isn't reachable over
    LoRa right now at all (the transmitter sends roughly once a second) —
    no reason to make the operator wait the full LORA_PRIORITY_GRACE_MS in
    that case; the cloud link is accepted as soon as it's ready instead. */

#define STATUS_PUSH_INTERVAL_MS        300u
#define TELEMETRY_DISTANCE_PUSH_MS     150u
#define PAIRING_BG_REFRESH_MS         5000u

/* How long without an actual distance message before the status bar
 * stops showing the link as up, even though the receiver's own broker
 * session may still be technically alive (MQTT pub/sub is decoupled —
 * see GSM_GetMsSinceLastMessage()'s doc comment for why that distinction
 * matters here). Originally 6000ms (a guess, before real hardware
 * testing) — confirmed too tight on real hardware 2026-08-30: each
 * transmitter publish is a chain of several blocking AT commands over a
 * real cellular link (AT+CMQTTTOPIC/PAYLOAD/PUB, each its own round
 * trip), so ordinary gaps between distance updates can legitimately run
 * well past 6s even with nothing actually wrong — every field that reads
 * this constant (status-bar health/mode, distance blanking) was
 * flickering between real values and "--" every time a normal gap
 * crossed that line, then recovering the instant the next update
 * arrived. Widened to match GSM_PRESENCE_TIMEOUT_MS's already-proven
 * 15000ms (gsm_mqtt.c) — same underlying real-world AT/network latency
 * this constant needs to tolerate, just applied to distance messages
 * instead of presence responses. Still short enough to mean something
 * real on a genuine disconnect. */
#define LINK_STALE_TIMEOUT_MS       15000u

/* Outer edge of the "actively approaching" range — beyond this, the
 * telemetry screen shows "OR" instead of a number. */
#define DISTANCE_MAX_ACTIVE_CM       4500u /* 45.00 meters */

/* Obstacle-warning overlay (page 14) — new feature, not from CLAUDE.md.
 * Pops up automatically when the LiDAR sees a sudden distance drop
 * (a person/object stepping into the beam ahead of the tracked train),
 * clears automatically once readings stabilize again. See
 * TickObstacleOverlay()'s own comment for the detection/recovery design.
 * Ported from Shunting_Receiver_v2's already-hardware-tuned values, not
 * re-derived here. */
#define OBSTACLE_DROP_THRESHOLD_CM    500u /* 5.00 meters */
#define OBSTACLE_CONFIRM_HOLD_MS     2000u

/* Text-field widths below are inferred, not given by CLAUDE.md — verify
 * against each VP's configured character count in DWIN Designer. A too-
 * narrow value here just truncates the string; too-wide is harmless
 * padding, so this is safe to leave generous until confirmed. */
#define DEVICE_NAME_FIELD_BYTES         16u
#define PAIRING_SLOT_NAME_FIELD_BYTES   16u
#define CONFIRM_NAME_FIELD_BYTES        16u
#define NUMBER_TEXT_FIELD_BYTES          8u
#define HEALTH_TEXT_FIELD_BYTES         12u /* fits "Excellent" */
#define MODE_TEXT_FIELD_BYTES            8u /* fits "GSM" and "LORA" */

/* Distance/volume were originally sent as raw 16-bit binary VP writes
 * (DWIN_WriteVP16) and rendered as garbage or blank on hardware —
 * confirmed on 2026-08-03 (source project) that the display's digital-
 * readout widgets expect literal ASCII decimal text. Route all of them
 * through this helper instead. */
static void WriteVPNumberText(uint16_t vp, uint32_t value, uint8_t field_bytes)
{
    char buf[12];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)value);
    DWIN_WriteVPString(vp, buf, field_bytes);
}

/* Telemetry distance readout, in metres (explicit request): from 10m up it
 * is shown as whole metres ("23"); below 10m it gets one decimal, in
 * half-metre steps only — "9.5", "9.0", "8.5"... — so the last digit is
 * always 0 or 5. Both round to the NEAREST step (a reading of 9.75m or more
 * therefore already shows "10"). Only formats; the underlying g_hmi
 * distance stays in centimetres, and the buzzer bands / obstacle detection
 * keep working from the exact value, not from this rounded text. */
static void FormatDistanceText(char *out, size_t out_len, uint16_t cm)
{
    uint32_t m = cm / 100u;
    uint32_t frac = cm % 100u;
    snprintf(out, out_len, "%lu.%02lu", (unsigned long)m, (unsigned long)frac);
}

/* Same as WriteVPNumberText() but appends "%" — used for battery/charge
 * percentage fields, matching the source project's own two-helper split. */
static void WriteVPPercentText(uint16_t vp, uint32_t value, uint8_t field_bytes)
{
    char buf[12];
    snprintf(buf, sizeof(buf), "%lu%%", (unsigned long)value);
    DWIN_WriteVPString(vp, buf, field_bytes);
}

typedef struct {
    uint8_t  start_device;
    uint16_t vp[VP_PAIRING_SLOTS_PER_PAGE];
} pairing_page_info_t;

/* Explicit per-slot addresses, not base+step — hardware-verified sweep
 * result from the source project, not a formula. Copied verbatim; do not
 * recalculate or "clean up" any address in it. */
static const pairing_page_info_t PAIRING_PAGES[5] = {
    { 1u,  { 0x2120u, 0x2140u, 0x2160u, 0x2180u, 0x2200u, 0x2220u, 0x2240u, 0x2260u, 0x2280u } },
    { 10u, { 0x2300u, 0x2320u, 0x2340u, 0x2360u, 0x2380u, 0x2400u, 0x2420u, 0x2440u, 0x2460u } },
    { 19u, { 0x2480u, 0x2500u, 0x2520u, 0x2540u, 0x2560u, 0x2580u, 0x2600u, 0x2620u, 0x2640u } },
    { 28u, { 0x2660u, 0x2680u, 0x2700u, 0x2720u, 0x2740u, 0x2760u, 0x2780u, 0x2800u, 0x2820u } },
    { 37u, { 0x2840u, 0x2860u, 0x2880u, 0x2900u, 0x2920u, 0x2940u, 0x2960u, 0x2980u, 0x3000u } },
};

static bool IsPairingScreen(screen_id_t s)
{
    return s == SCR_01_PAIRING_P1 || s == SCR_05_PAIRING_P2 ||
           s == SCR_06_PAIRING_P3 || s == SCR_07_PAIRING_P4 ||
           s == SCR_08_PAIRING_P5;
}

/* screen_id_t values are defined to match the DGUS page IDs 1:1 (see the
 * assumption noted in hmi_map.h) — a plain cast is intentional here. */
static uint16_t PageIdFor(screen_id_t s)
{
    return (uint16_t)s;
}

static void PushStatusBar(uint32_t now)
{
    /* "Excellent" was truncating to "Excel" on hardware — that widget
     * renders ~5 characters, so "Best" is used instead of the full word. */
    static const char *HEALTH_TEXT[3] = { "Poor", "Good", "Best" };

    WriteVPPercentText(VP_BATTERY_PCT, g_hmi.battery_pct, NUMBER_TEXT_FIELD_BYTES);

    if (g_hmi.connected_device_num != 0u) {
        /* "Connected" on the status bar means the actual end-to-end link is
         * up for whichever transport is currently authoritative
         * (g_hmi.conn_mode, set by TickConnectionStatus()) — real telemetry
         * has actually arrived recently, not just "the broker session is
         * technically alive": a broker session can stay up even while the
         * specific transmitter this receiver is listening to has gone
         * silent (powered off, out of range, etc). Showing "GSM/Good" or
         * "LORA/Best" in that situation would be actively misleading on a
         * collision-avoidance system, not just cosmetic.
         *
         * Only reached once connected_device_num is set, and that only ever
         * happens inside CommitConnectionAndShowTelemetry() — i.e. after
         * ConnectionIsReady() has genuinely decided a link is up — so
         * g_hmi.conn_mode is never a stale leftover from a previous
         * connection here (this used to flash "GSM/Best" for a moment on
         * entering the connecting screen when the cloud subscription was
         * already live from an earlier session). */
        bool link_up;
        if (g_hmi.conn_mode == CONN_MODE_LORA) {
            link_up = (LoRa_GetState() == LORA_LINK_STABLE) || 
                      (LoRa_GetState() == LORA_LINK_LISTENING && LoRa_GetWindowSuccessCount() >= 1u);
        } else {
            link_up = (GSM_GetState() == GSM_LINK_CONNECTED) &&
                      (GSM_GetMsSinceLastMessage(now) <= LINK_STALE_TIMEOUT_MS);
        }
        if (link_up) {
            DWIN_WriteVPString(VP_CONN_HEALTH, HEALTH_TEXT[g_hmi.conn_health], HEALTH_TEXT_FIELD_BYTES);
            DWIN_WriteVPString(VP_CONN_MODE, (g_hmi.conn_mode == CONN_MODE_GSM) ? "GSM" : "LoRa", MODE_TEXT_FIELD_BYTES);
        } else {
            DWIN_WriteVPString(VP_CONN_HEALTH, "--", HEALTH_TEXT_FIELD_BYTES);
            DWIN_WriteVPString(VP_CONN_MODE, "--", MODE_TEXT_FIELD_BYTES);
        }
        DWIN_WriteVPString(VP_DEVICE_NAME, g_hmi.connected_device_name, DEVICE_NAME_FIELD_BYTES);
    } else {
        DWIN_WriteVPString(VP_CONN_HEALTH, "", HEALTH_TEXT_FIELD_BYTES);
        DWIN_WriteVPString(VP_CONN_MODE, "", MODE_TEXT_FIELD_BYTES);
        DWIN_WriteVPString(VP_DEVICE_NAME, "", DEVICE_NAME_FIELD_BYTES);
    }

    WriteVPNumberText(VP_VOLUME_PCT, g_hmi.volume_pct, NUMBER_TEXT_FIELD_BYTES);
}

static void WriteAllPairingSlots(void)
{
    char name[HMI_DEVICE_NAME_LEN];
    for (uint8_t p = 0; p < 5u; p++) {
        for (uint8_t slot = 0; slot < VP_PAIRING_SLOTS_PER_PAGE; slot++) {
            uint8_t dev = (uint8_t)(PAIRING_PAGES[p].start_device + slot);
            uint16_t vp = PAIRING_PAGES[p].vp[slot];
            if (g_hmi.device_online[dev - 1u]) {
                snprintf(name, sizeof(name), "TX-%02u", dev);
                DWIN_WriteVPString(vp, name, PAIRING_SLOT_NAME_FIELD_BYTES);
            } else {
                DWIN_WriteVPString(vp, "", PAIRING_SLOT_NAME_FIELD_BYTES);
            }
        }
    }
}

/* Real presence, merged across both links — a device counts as online
 * if either the AWS heartbeat/telemetry path or the LoRa broadcast has
 * heard from it recently. LoRa alone is enough, so the pairing screens
 * keep working with no cellular coverage at all. */
static bool DeviceIsOnline(uint8_t device_num, uint32_t now)
{
    return GSM_IsDeviceOnline(device_num, now) || LoRa_IsDeviceOnline(device_num, now);
}

static void RefreshPairingSlots(uint32_t now)
{
    for (uint8_t dev = 1u; dev <= HMI_MAX_DEVICES; dev++) {
        g_hmi.device_online[dev - 1u] = DeviceIsOnline(dev, now);
    }
    WriteAllPairingSlots();
}

/* Legacy scan function was removed. Presence is handled passively via AWS wildcard.
 * The redraw itself just reflects whatever's currently known — responses that
 * arrive later show up on the next redraw, not this one. */
static void TriggerPresenceScanAndRedraw(uint32_t now)
{
    /* GSM_RequestPresenceScan() removed. */
    RefreshPairingSlots(now);
}

/* Enters a screen: updates shared state, optionally sends the DGUS page
 * jump (skip send_page_cmd for touch codes DGUS already hardware-jumped
 * on, per CLAUDE.md), and pushes whatever VPs that screen needs on entry. */
static void EnterScreen(screen_id_t target, bool send_page_cmd, uint32_t now)
{
    g_hmi.active_screen = target;
    g_hmi.screen_entered_tick = now;

    /* While the obstacle overlay is covering the display, suppress the
     * page jump so that overlay's page stays visible — logical navigation
     * still happens underneath, and TickObstacleOverlay() jumps to the
     * right page once the overlay clears. */
    if (send_page_cmd && !g_hmi.obstacle_overlay_active) {
        DWIN_SwitchPage(PageIdFor(target));
    }

    switch (target) {
        case SCR_01_PAIRING_P1:
            /* Idempotent — safe to call every time this screen is
             * (re)entered, not just the first time (see
             * GSM_BeginScanning()'s doc comment — it's a no-op in this
             * project, since GSM_MQTT_Init() already ran the full
             * connect+subscribe chain synchronously at boot). */
            GSM_BeginScanning();
            TriggerPresenceScanAndRedraw(now);
            break;

        case SCR_10_CONFIRM_SELECTION:
            DWIN_WriteVPString(VP_CONFIRM_DEVICE_NAME, g_hmi.selected_device_name, CONFIRM_NAME_FIELD_BYTES);
            break;

        case SCR_09_CHANGE_DEVICE:
            DWIN_WriteVPString(VP_CHANGE_DEVICE_NAME, g_hmi.connected_device_name, CONFIRM_NAME_FIELD_BYTES);
            break;

        case SCR_02_CONNECTING:
            g_hmi.connecting_start_tick = now;
            /* Both links start racing from here — whichever proves
             * itself ready first wins (see ConnectionIsReady() below).
             * LoRa only ever wins this race if its rolling window fills
             * with a genuinely good success rate within a few seconds,
             * which in practice only happens when already in range — the
             * AWS/GSM path is the fallback for everything else.
             * GSM_BeginConnect() just records which transmitter's
             * telemetry to track (the baseline wildcard subscriptions are
             * already up); LoRa_BeginListen() is non-blocking. */
            GSM_BeginConnect(g_hmi.selected_device_num);
            LoRa_BeginListen(g_hmi.selected_device_num);
            break;

        default:
            break;
    }

    if (target != SCR_00_STARTUP) {
        PushStatusBar(now);
    }
}

static void HandleTouch(uint16_t code, uint32_t now)
{
    /* Volume is reachable from every screen in the reference bitmaps
     * (UI_Images), even though CLAUDE.md's prose only calls it out for
     * 04_Telemetry — handling it globally matches what's actually drawn
     * and is harmless if a given page has no volume control bound. */
    if (code == TOUCH_VOLUME_DOWN) {
        g_hmi.volume_pct = (g_hmi.volume_pct >= 10u) ? (uint8_t)(g_hmi.volume_pct - 10u) : 0u;
        WriteVPNumberText(VP_VOLUME_PCT, g_hmi.volume_pct, NUMBER_TEXT_FIELD_BYTES);
        return;
    }
    if (code == TOUCH_VOLUME_UP) {
        g_hmi.volume_pct = (g_hmi.volume_pct <= 90u) ? (uint8_t)(g_hmi.volume_pct + 10u) : 100u;
        WriteVPNumberText(VP_VOLUME_PCT, g_hmi.volume_pct, NUMBER_TEXT_FIELD_BYTES);
        return;
    }

    if (code == TOUCH_REFRESH) {
        if (IsPairingScreen(g_hmi.active_screen)) {
            TriggerPresenceScanAndRedraw(now);
        }
        return;
    }

    if (code >= TOUCH_DEVICE_SELECT_MIN && code <= TOUCH_DEVICE_SELECT_MAX) {
        if (IsPairingScreen(g_hmi.active_screen) && g_hmi.device_online[code - 1u]) {
            g_hmi.selected_device_num = (uint8_t)code;
            snprintf(g_hmi.selected_device_name, sizeof(g_hmi.selected_device_name), "TX-%02u", (unsigned)code);
            EnterScreen(SCR_10_CONFIRM_SELECTION, true, now);
        }
        return;
    }

    switch (g_hmi.active_screen) {
        case SCR_09_CHANGE_DEVICE:
            if (code == TOUCH_CHANGE_DEVICE_YES) {
                GSM_Disconnect(); /* actually sever the connection, per CLAUDE.md */
                LoRa_StopListen();
                g_hmi.connected_device_num = 0u;
                strcpy(g_hmi.connected_device_name, "--");
                EnterScreen(SCR_01_PAIRING_P1, false, now); /* DGUS already jumped */
            } else if (code == TOUCH_CHANGE_DEVICE_NO) {
                EnterScreen(SCR_04_TELEMETRY, false, now);
            }
            break;

        case SCR_10_CONFIRM_SELECTION:
            if (code == TOUCH_CONFIRM_SEL_YES) {
                /* connected_device_* is deliberately NOT set here — it
                 * only gets committed in TickTimers() once the connection
                 * actually succeeds, so the status bar keeps showing "--"
                 * all the way through 02_Connecting rather than jumping
                 * the gun before the link is real. */
                EnterScreen(SCR_02_CONNECTING, false, now);
            } else if (code == TOUCH_CONFIRM_SEL_NO) {
                g_hmi.selected_device_num = 0u;
                EnterScreen(SCR_01_PAIRING_P1, false, now);
            }
            break;

        case SCR_11_END_SHUNTING:
            if (code == TOUCH_END_SHUNTING_YES) {
                GSM_Disconnect(); /* actually sever the connection, per CLAUDE.md */
                LoRa_StopListen();
                g_hmi.connected_device_num = 0u;
                strcpy(g_hmi.connected_device_name, "--");
                EnterScreen(SCR_12_SHUNTING_COMPLETED, false, now);
            } else if (code == TOUCH_END_SHUNTING_NO) {
                EnterScreen(SCR_04_TELEMETRY, false, now);
            }
            break;

        case SCR_04_TELEMETRY:
            if (code == TOUCH_BACK) {
                EnterScreen(SCR_09_CHANGE_DEVICE, false, now);
            } else if (code == TOUCH_END_SHUNTING_BTN) {
                EnterScreen(SCR_11_END_SHUNTING, false, now);
            }
            break;

        default:
            /* Back buttons drawn on 02_Connecting/03_Error/pairing pages/
             * 12_Shunting_Completed have no documented touch code in
             * CLAUDE.md's table. Assumed (unverified) to be hardware-only
             * page jumps like Prev/Next, needing no STM32 reaction —
             * confirm in DWIN Designer; if wrong, those buttons currently
             * do nothing. */
            break;
    }
}

/* Commits the in-progress selection as the actual connected device and
 * shows Telemetry. Shared by the normal 02_Connecting success path and
 * the 03_Error auto-recovery below. */
static void CommitConnectionAndShowTelemetry(uint32_t now)
{
    g_hmi.connected_device_num = g_hmi.selected_device_num;
    strcpy(g_hmi.connected_device_name, g_hmi.selected_device_name);
    EnterScreen(SCR_04_TELEMETRY, true, now);
}

/* GSM_LINK_CONNECTED only means the receiver's own MQTT broker session
 * is up — the subscribe succeeds whether or not anything is actually
 * publishing to that topic, so a transmitter that was never even powered
 * on would otherwise still let a connect attempt succeed. Require at
 * least one real distance message to have arrived this session before
 * treating the connect as genuine — the same distinction
 * GSM_GetMsSinceLastMessage() already exists for (see its doc comment,
 * and PushStatusBar()'s use of it), just applied here to the screen
 * transition itself rather than only to the status bar text. A
 * legitimately-connecting transmitter only costs about one publish
 * interval of extra wait here, well inside CLAUDE.md's 20s timeout. */
static bool GsmHasRealTelemetry(uint32_t now)
{
    return GSM_GetState() == GSM_LINK_CONNECTED &&
           GSM_GetMsSinceLastMessage(now) != UINT32_MAX;
}

/* LoRa gets explicit priority over the AWS/GSM path, not just "checked
 * first". A plain `LoRa STABLE || GsmHasRealTelemetry()` would check LoRa
 * first in the code but not actually favour it: LORA_LINK_STABLE needs a
 * mandatory ~7.5s minimum to ever become true (the rolling window's
 * cold-start guard — see LORA_PRIORITY_GRACE_MS's comment), while the
 * cloud link is usually ready well inside that window, so in practice the
 * cloud would win almost every time and LoRa would only take over a few
 * seconds later on the Telemetry screen — the opposite of what's wanted.
 * So the cloud link isn't accepted at all until either (a) LoRa has shown
 * zero presence for this device within LORA_NO_SIGNAL_GRACE_MS (clearly not
 * reachable over LoRa right now — no reason to make the operator wait), or
 * (b) LoRa has been given the full LORA_PRIORITY_GRACE_MS and still hasn't
 * reached STABLE (signal present but too marginal to trust).
 *
 * This only decides when the SCREEN may act on the cloud link already being
 * ready; it never touches the AWS/GSM connection itself. Once connected,
 * TickConnectionStatus() keeps preferring LORA_LINK_STABLE every tick for as
 * long as the session lasts, so a session that started on the cloud link
 * (device briefly out of LoRa range) hands over to LoRa automatically the
 * moment it comes back into range. */
static bool ConnectionIsReady(uint32_t now)
{
    if (LoRa_GetState() == LORA_LINK_STABLE) {
        return true;
    }

    /* If GSM is not connected and LoRa has received multiple frames for target, connect directly */
    if (GSM_GetState() != GSM_LINK_CONNECTED) {
        if (LoRa_GetWindowSuccessCount() >= 2u) {
            return true;
        }
    }

    uint32_t connecting_elapsed = now - g_hmi.connecting_start_tick;
    if (connecting_elapsed < LORA_NO_SIGNAL_GRACE_MS) {
        return false; /* too soon to tell either way — keep waiting */
    }

    bool lora_signal_present = LoRa_IsDeviceOnline(g_hmi.selected_device_num, now);
    if (lora_signal_present && connecting_elapsed < LORA_PRIORITY_GRACE_MS) {
        return false; /* LoRa is visible and still has time to reach STABLE */
    }

    return GsmHasRealTelemetry(now);
}

static void TickTimers(uint32_t now)
{
    switch (g_hmi.active_screen) {
        case SCR_00_STARTUP:
            if (now - g_hmi.screen_entered_tick >= STARTUP_HOLD_MS) {
                EnterScreen(SCR_01_PAIRING_P1, true, now);
            }
            break;

        case SCR_02_CONNECTING:
            if (ConnectionIsReady(now)) {
                CommitConnectionAndShowTelemetry(now);
            } else if (now - g_hmi.connecting_start_tick >= CONNECTING_TIMEOUT_MS) {
                EnterScreen(SCR_03_ERROR, true, now);
            }
            break;

        case SCR_03_ERROR:
            /* CLAUDE.md's 20s UI timeout is shorter than a real GSM
             * connect sequence can legitimately take (individual AT
             * steps can run up to 60s each), so while still sitting on
             * this screen, both links keep trying in the background
             * rather than being aborted — an attempt that's just slow
             * (not actually failed) still recovers automatically instead
             * of stranding the locopilot here. Recovery is checked
             * first, same priority order as the hold-timer pattern
             * elsewhere (e.g. 02_Connecting) — if the link comes up
             * right as the hold is about to expire, it still wins. */
            if (ConnectionIsReady(now)) {
                CommitConnectionAndShowTelemetry(now);
            } else if (now - g_hmi.screen_entered_tick >= ERROR_HOLD_MS) {
                /* Once we actually leave for the pairing screen, the
                 * attempt must stop for real — otherwise GSM/LoRa keep
                 * retrying the old device in the background, and a
                 * reconnect succeeding later (e.g. the transmitter
                 * getting powered back on) would silently show up on
                 * the status bar while sitting on 01_Pairing_P1, with no
                 * user action taken. Same cleanup as the explicit
                 * Change Device / End Shunting paths. */
                GSM_Disconnect();
                LoRa_StopListen();
                g_hmi.selected_device_num = 0u;
                EnterScreen(SCR_01_PAIRING_P1, true, now);
            }
            break;

        case SCR_12_SHUNTING_COMPLETED:
            if (now - g_hmi.screen_entered_tick >= SHUNTING_COMPLETED_HOLD_MS) {
                EnterScreen(SCR_00_STARTUP, true, now);
            }
            break;

        default:
            break;
    }
}

/* True only when NEITHER link can actually deliver telemetry right now —
 * reuses the exact same "is this link actually live" criteria already
 * established for the status bar (PushStatusBar()'s link_up), not a new
 * concept. Forward-declared here, defined below, since
 * TickPeriodicPushes() needs it before its own definition appears. */
static bool BothLinksDown(uint32_t now);

static void TickPeriodicPushes(uint32_t now)
{
    static uint32_t next_status_push;
    static uint32_t next_distance_push;
    static uint32_t next_pairing_bg_refresh;

    if (g_hmi.active_screen != SCR_00_STARTUP && now >= next_status_push) {
        next_status_push = now + STATUS_PUSH_INTERVAL_MS;
        PushStatusBar(now);
    }

    if (g_hmi.active_screen == SCR_04_TELEMETRY && now >= next_distance_push) {
        next_distance_push = now + TELEMETRY_DISTANCE_PUSH_MS;
        /* Once the transmitter is truly gone (both links down, same check
         * used for the status bar), the last real distance sample is
         * stale/meaningless — TickDistanceFromLinks() only ever updates
         * g_hmi.distance_cm when a fresh sample actually arrives, so
         * without this check it would otherwise stay locked on whatever
         * was last received forever. Checked first/takes priority over
         * the out-of-range case below, since a stale sample's numeric
         * value (>45.00m or not) says nothing real once both links are
         * down. "DC" (disconnected) vs "OR" (out of range) — explicit
         * request, distinguishing "transmitter gone" from "transmitter
         * present but beyond the active range" instead of both showing
         * the same "--". */
        if (BothLinksDown(now)) {
            DWIN_WriteVPString(VP_DISTANCE, "DC", NUMBER_TEXT_FIELD_BYTES);
        } else if (g_hmi.distance_cm > DISTANCE_MAX_ACTIVE_CM) {
            DWIN_WriteVPString(VP_DISTANCE, "OR", NUMBER_TEXT_FIELD_BYTES);
        } else {
            char dist_str[12];
            FormatDistanceText(dist_str, sizeof(dist_str), g_hmi.distance_cm);
            DWIN_WriteVPString(VP_DISTANCE, dist_str, NUMBER_TEXT_FIELD_BYTES);
        }
    }

    if (IsPairingScreen(g_hmi.active_screen) && now >= next_pairing_bg_refresh) {
        next_pairing_bg_refresh = now + PAIRING_BG_REFRESH_MS;
        /* Prev/Next hardware-jump between pairing pages without telling
         * the STM32 (see CLAUDE.md), so this keeps ALL 5 pages' slot VPs
         * live in the background rather than refreshing only "on entry"
         * to whichever page we last commanded. Also the periodic trigger
         * for GSM's presence query cycle — scoped to pairing screens only
         * via this same IsPairingScreen() gate, so it never fires while
         * 02_Connecting/03_Error/04_Telemetry/etc — no reason to keep
         * asking "who else is online" once already connected to a
         * specific device and viewing its telemetry. */
        TriggerPresenceScanAndRedraw(now);
    }
}

/* Maps whichever link is currently authoritative onto the status bar's
 * coarse conn_mode/conn_health indicators. LoRa is checked first and
 * wins whenever its rolling window reports LORA_LINK_STABLE — LoRa is the
 * preferred link whenever it's in range, with the AWS/GSM path as the
 * fallback (matches EnterScreen()'s SCR_02_CONNECTING comment). This check
 * is deliberately NOT gated on the cloud link being connected — LoRa can
 * be, and often is, ready before the MQTT session is. Only meaningful once
 * at least one link is actually ready; otherwise conn_health/conn_mode
 * just hold their last value, same as before there was a real link to
 * report on.
 *
 * The cloud path's health stays recency-based (GSM_GetMsSinceLastMessage
 * against fixed thresholds), no live RSSI polling (a second, independent
 * AT+CSQ command would compete with GSM_MQTT_Poll()'s own receive path for
 * the same UART/response buffer, risking a dropped incoming message; not
 * worth it for a signal-bars nicety on a project whose top priority is
 * reliability). LoRa has no RSSI reachable on this hardware, so its own
 * rolling-window success count doubles as the health indicator instead —
 * thresholds are LoRa's own scale (0-5 slots), not meant to line up
 * numerically with the cloud path's time-based scale. */
static void TickConnectionStatus(uint32_t now)
{
    uint8_t lora_success = LoRa_GetWindowSuccessCount();
    if (LoRa_GetState() == LORA_LINK_STABLE || (LoRa_GetState() == LORA_LINK_LISTENING && lora_success >= 1u)) {
        g_hmi.conn_mode = CONN_MODE_LORA;
        if (lora_success >= 5u) {
            g_hmi.conn_health = CONN_HEALTH_EXCELLENT;
        } else if (lora_success >= 4u) {
            g_hmi.conn_health = CONN_HEALTH_GOOD;
        } else {
            g_hmi.conn_health = CONN_HEALTH_POOR; /* defensive — the dead
                zone (3) can still be observed here via hysteresis, but
                the exit threshold (<=2) demotes out of LORA_LINK_STABLE
                before this would normally go lower */
        }
        return;
    }

    if (GSM_GetState() == GSM_LINK_CONNECTED) {
        g_hmi.conn_mode = CONN_MODE_GSM;
        uint32_t sinceMs = GSM_GetMsSinceLastMessage(now);
        if (sinceMs <= 2000u) {
            g_hmi.conn_health = CONN_HEALTH_EXCELLENT;
        } else if (sinceMs <= LINK_STALE_TIMEOUT_MS) {
            g_hmi.conn_health = CONN_HEALTH_GOOD;
        } else {
            g_hmi.conn_health = CONN_HEALTH_POOR;
        }
    }
}

/* Drains whatever each link has most recently parsed into
 * g_hmi.distance_cm, taking whichever one is currently authoritative
 * (g_hmi.conn_mode, set above). Both getters run every tick
 * unconditionally, regardless of which is active — LoRa's is an "unread
 * since last call" flag, so reading it only while LoRa is active would
 * leave it holding a stale sample from before a handover; draining it
 * every tick keeps both fresh so a switch never picks up a leftover
 * reading from before it. */
static void TickDistanceFromLinks(void)
{
    uint16_t gsm_distance_cm;
    bool gsm_has_new = GSM_GetLatestDistance(&gsm_distance_cm);
    uint16_t lora_distance_cm;
    bool lora_has_new = LoRa_GetLatestDistance(&lora_distance_cm);

    if (g_hmi.conn_mode == CONN_MODE_LORA) {
        if (lora_has_new) {
            g_hmi.distance_cm = lora_distance_cm;
        }
    } else if (gsm_has_new) {
        g_hmi.distance_cm = gsm_distance_cm;
    }
}

static bool BothLinksDown(uint32_t now)
{
    bool gsm_live = (GSM_GetState() == GSM_LINK_CONNECTED) &&
                    (GSM_GetMsSinceLastMessage(now) <= LINK_STALE_TIMEOUT_MS);
    bool lora_live = (LoRa_GetState() == LORA_LINK_STABLE) ||
                     (g_hmi.conn_mode == CONN_MODE_LORA && LoRa_IsDeviceOnline(g_hmi.connected_device_num, now));
    return !gsm_live && !lora_live;
}

/* Ported from Shunting_Receiver_v2's TickObstacleOverlay(), unchanged
 * design — g_hmi.active_screen deliberately stays SCR_04_TELEMETRY
 * throughout, only g_hmi.obstacle_overlay_active + a direct
 * DWIN_SwitchPage() change.
 * This is what lets TickBuzzer()'s proximity/alarm tone and
 * TickPeriodicPushes()'s status-bar/distance writes keep running exactly
 * as they already do on Telemetry, and Volume -/+ / END SHUNTING keep
 * working via HandleTouch()'s existing SCR_04_TELEMETRY case — none of
 * that needs to change.
 *
 * Only evaluated during an actual live Telemetry session.
 *
 * Three states, tracked locally (PENDING doesn't need to be in g_hmi —
 * it's purely an internal debounce detail):
 *
 * IDLE -> PENDING: the previous sample was a real reading
 * (<= DISTANCE_MAX_ACTIVE_M — reusing the constant that already means "a
 * real reading" elsewhere in this file) and the new one is at least
 * OBSTACLE_DROP_THRESHOLD_M below it. The "previous sample was real" guard
 * is what stops the very first live reading after connecting — jumping
 * from the 500m "no data yet" fallback down to a real distance — from
 * being misread as a 400+ meter "obstacle." Does NOT show the warning
 * screen yet.
 *
 * PENDING -> IDLE (cancel) or -> ACTIVE (confirm): while pending, any
 * sample that rises back up by OBSTACLE_DROP_THRESHOLD_M or more cancels
 * the pending check — this is what makes someone briefly walking through
 * the beam not pop up the warning at all, per explicit request. If no
 * such rise happens before OBSTACLE_CONFIRM_HOLD_MS (2000ms) elapses,
 * PENDING is confirmed into ACTIVE and the warning screen shows. The
 * elapsed-time check runs every tick regardless of whether this
 * particular sample changed value, since a near-stationary obstacle can
 * easily produce several identical readings in a row and the hold timer
 * must still elapse in that case.
 *
 * ACTIVE -> IDLE: clears immediately (no further hold/debounce) the
 * first time a sample rises back up by OBSTACLE_DROP_THRESHOLD_M or
 * more — per explicit request, this needs to feel fast, not wait for
 * several consecutive stable samples. This rise *is* the obstacle
 * leaving the beam, not a separate event to wait out.
 *
 * A closing train never gets misread as a new obstacle once recovered:
 * every comparison here is against s_last_distance_m, which is updated
 * to the truly most recent sample every single tick — including all the
 * way through the obstacle period. So the moment ACTIVE clears, the
 * baseline for the next comparison is already wherever the train
 * actually is *now*, never the stale pre-obstacle value — a train that
 * kept closing distance while blocked just continues its normal
 * approach from there, not a phantom "sudden drop" from an old number.
 * (The one real edge case this can't fully rule out: if the train itself
 * moves more than OBSTACLE_DROP_THRESHOLD_M between two consecutive real
 * samples — physically implausible at ordinary shunting speeds against a
 * ~1Hz sample rate, and would trigger this same logic even with no
 * obstacle ever involved, since it's inherent to any consecutive-sample-
 * delta trigger, not specific to recovery.) */
static void TickObstacleOverlay(uint32_t now)
{
    static uint16_t s_last_distance_cm;
    static bool     s_have_last_distance;
    static bool     s_pending;
    static uint32_t s_pending_since_tick;

    bool telemetry_active = (g_hmi.active_screen == SCR_04_TELEMETRY);

    if (!telemetry_active) {
        s_have_last_distance = false;
        s_pending = false;
        if (g_hmi.obstacle_overlay_active) {
            /* Left Telemetry some other way (Change Device/End Shunting)
             * while the warning was up — clear it defensively rather than
             * leaving it stuck active. */
            g_hmi.obstacle_overlay_active = false;
        }
        return;
    }

    if (!s_have_last_distance) {
        /* First sample since telemetry began (or resumed) — just
         * establishes the baseline, never evaluated as a drop/rise. */
        s_last_distance_cm = g_hmi.distance_cm;
        s_have_last_distance = true;
    } else if (g_hmi.distance_cm != s_last_distance_cm) {
        uint16_t prev = s_last_distance_cm;
        uint16_t cur = g_hmi.distance_cm;
        bool prev_was_real = (prev <= DISTANCE_MAX_ACTIVE_CM);
        bool sudden_drop = prev_was_real && (prev > cur) && ((prev - cur) >= OBSTACLE_DROP_THRESHOLD_CM);
        bool sudden_rise = prev_was_real && (cur > prev) && ((cur - prev) >= OBSTACLE_DROP_THRESHOLD_CM);

        if (g_hmi.obstacle_overlay_active) {
            if (sudden_rise) {
                g_hmi.obstacle_overlay_active = false;
                DWIN_SwitchPage(PAGE_04_TELEMETRY);
            }
        } else if (s_pending) {
            if (sudden_rise) {
                s_pending = false; /* brief pass-through, not a real obstacle */
            }
        } else if (sudden_drop) {
            s_pending = true;
            s_pending_since_tick = now;
        }

        s_last_distance_cm = cur;
    }

    /* Runs every tick regardless of whether this particular sample
     * changed value — see the PENDING state's own comment above. */
    if (s_pending && (now - s_pending_since_tick >= OBSTACLE_CONFIRM_HOLD_MS)) {
        g_hmi.obstacle_overlay_active = true;
        DWIN_SwitchPage(PAGE_14_OBSTACLE_WARNING);
        s_pending = false;
    }
}

/* Ported from Shunting_Receiver_v2's TickBuzzer() — distance-band beep-
 * cadence thresholds (180/200/250/400ms) are its hand-verified "sounds
 * right" values, ported as-is. BothLinksDown() (cloud-or-LoRa) is the same
 * "is the link to the transmitter actually alive" concept the status bar
 * already uses, just checked here for the continuous-alarm-tone case. */
static void TickBuzzer(uint32_t now)
{
    Buzzer_SetVolume(g_hmi.volume_pct);

    if (g_hmi.active_screen != SCR_04_TELEMETRY) {
        Buzzer_SetContinuousTone(false);
        Buzzer_SetBeepIntervalMs(0u); /* silent outside an active shunting session */
        return;
    }

    if (BothLinksDown(now)) {
        /* Total link loss — a steady, non-toggling alarm tone rather
         * than the intermittent proximity cadence, so it reads as
         * unmistakably "we've lost the transmitter," not "getting close
         * to the dead-end." */
        Buzzer_SetContinuousTone(true);
        return;
    }
    Buzzer_SetContinuousTone(false);

    uint32_t d_cm = g_hmi.distance_cm;
    if (d_cm > DISTANCE_MAX_ACTIVE_CM) {
        Buzzer_SetBeepIntervalMs(0u); /* out of range entirely — fully silent */
        return;
    }

    uint32_t interval_ms;
    if (d_cm < 500u) {          /* < 5.00m: dead-end warning */
        interval_ms = 150u;
    } else if (d_cm < 1000u) {  /* < 10.00m */
        interval_ms = 200u;
    } else if (d_cm < 2000u) {  /* < 20.00m */
        interval_ms = 250u;
    } else {
        interval_ms = 400u;     /* 20.00 - 45.00m — outermost active tier */
    }
    Buzzer_SetBeepIntervalMs(interval_ms);
}

void ScreenSM_Init(void)
{
    EnterScreen(SCR_00_STARTUP, true, 0u);
}

void ScreenSM_ForceStartupReset(uint32_t now_ms)
{
    g_hmi.selected_device_num = 0u;
    g_hmi.connected_device_num = 0u;
    strcpy(g_hmi.selected_device_name, "--");
    strcpy(g_hmi.connected_device_name, "--");
    memset(g_hmi.device_online, 0, sizeof(g_hmi.device_online));
    g_hmi.conn_health = CONN_HEALTH_POOR;
    g_hmi.conn_mode = CONN_MODE_GSM;
    g_hmi.distance_cm = 50000u; /* same "no data yet" fallback as HmiState_Init() */
    EnterScreen(SCR_00_STARTUP, true, now_ms);
}

void ScreenSM_Tick(uint32_t now_ms)
{
    uint16_t code;
    while (DWIN_PopTouchCode(&code)) {
        HandleTouch(code, now_ms);
    }

    TickConnectionStatus(now_ms);
    TickDistanceFromLinks();
    TickObstacleOverlay(now_ms);
    TickTimers(now_ms);
    TickPeriodicPushes(now_ms);
    TickBuzzer(now_ms);
}
