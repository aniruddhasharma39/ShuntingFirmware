#include "screen_sm.h"
#include "hmi_state.h"
#include "hmi_map.h"
#include "dwin_hmi.h"
#include "gsm_mqtt.h"
#include "buzzer.h"
#include <stdio.h>
#include <string.h>

/* ---- timing constants (see CLAUDE.md screen-flow section in the source
 * project for the ones that are spec, not just demo pacing: 3s startup
 * hold, 20s connect timeout, 2s shunting-completed hold) --------------- */
#define STARTUP_HOLD_MS               3000u
#define CONNECTING_TIMEOUT_MS        20000u
#define SHUNTING_COMPLETED_HOLD_MS    2000u
#define ERROR_HOLD_MS                 3000u

#define STATUS_PUSH_INTERVAL_MS        300u
#define TELEMETRY_DISTANCE_PUSH_MS     150u
#define PAIRING_BG_REFRESH_MS         5000u
#define CHARGE_TEXT_PUSH_INTERVAL_MS  1000u

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
#define MODE_TEXT_FIELD_BYTES            8u /* fits "GSM" — this port is
                                                 GSM-only, but the field
                                                 stays in case a future
                                                 link type is added */
#define CHARGE_PCT_TEXT_FIELD_BYTES      8u

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
        /* "Connected" on the status bar means real telemetry has actually
         * arrived recently, not just "the broker session is technically
         * alive" — a broker session can stay up even while the specific
         * transmitter this receiver is listening to has gone silent
         * (powered off, out of range, etc). Showing "GSM/Good" in that
         * situation would be actively misleading on a collision-avoidance
         * system, not just cosmetic. */
        bool link_up = (GSM_GetState() == GSM_LINK_CONNECTED) &&
                       (GSM_GetMsSinceLastMessage(now) <= LINK_STALE_TIMEOUT_MS);
        if (link_up) {
            DWIN_WriteVPString(VP_CONN_HEALTH, HEALTH_TEXT[g_hmi.conn_health], HEALTH_TEXT_FIELD_BYTES);
            DWIN_WriteVPString(VP_CONN_MODE, "GSM", MODE_TEXT_FIELD_BYTES);
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
                snprintf(name, sizeof(name), "D-%02u", dev);
                DWIN_WriteVPString(vp, name, PAIRING_SLOT_NAME_FIELD_BYTES);
            } else {
                DWIN_WriteVPString(vp, "", PAIRING_SLOT_NAME_FIELD_BYTES);
            }
        }
    }
}

/* GSM-only presence (this port has no LoRa) — a device counts as online
 * if a presence response has been heard from it recently. */
static bool DeviceIsOnline(uint8_t device_num, uint32_t now)
{
    return GSM_IsDeviceOnline(device_num, now);
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

    /* While the charger or obstacle overlay is covering the display,
     * suppress the page jump so that overlay's page stays visible —
     * logical navigation still happens underneath, and
     * TickChargerOverlay()/TickObstacleOverlay() jump to the right page
     * once the overlay clears. */
    if (send_page_cmd && !g_hmi.charger_overlay_active && !g_hmi.obstacle_overlay_active) {
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
            /* GSM is the only link in this port — this call performs the
             * (blocking, bounded by the driver's own CMD_TIMEOUT/
             * URC_WAIT_TIMEOUT) per-device subscribe on top of the
             * already-up baseline session. */
            GSM_BeginConnect(g_hmi.selected_device_num);
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
            snprintf(g_hmi.selected_device_name, sizeof(g_hmi.selected_device_name), "D-%02u", (unsigned)code);
            EnterScreen(SCR_10_CONFIRM_SELECTION, true, now);
        }
        return;
    }

    switch (g_hmi.active_screen) {
        case SCR_09_CHANGE_DEVICE:
            if (code == TOUCH_CHANGE_DEVICE_YES) {
                GSM_Disconnect(); /* actually sever the connection, per CLAUDE.md */
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

static bool ConnectionIsReady(uint32_t now)
{
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
             * this screen, GSM keeps trying in the background rather
             * than being aborted — an attempt that's just slow (not
             * actually failed) still recovers automatically instead of
             * stranding the locopilot here. Recovery is checked first,
             * same priority order as the hold-timer pattern elsewhere
             * (e.g. 02_Connecting) — if the link comes up right as the
             * hold is about to expire, it still wins. */
            if (ConnectionIsReady(now)) {
                CommitConnectionAndShowTelemetry(now);
            } else if (now - g_hmi.screen_entered_tick >= ERROR_HOLD_MS) {
                /* Once we actually leave for the pairing screen, the
                 * attempt must stop for real — otherwise GSM keeps
                 * retrying the old device in the background, and a
                 * reconnect succeeding later (e.g. the transmitter
                 * getting powered back on) would silently show up on
                 * the status bar while sitting on 01_Pairing_P1, with no
                 * user action taken. Same cleanup as the explicit
                 * Change Device / End Shunting paths. */
                GSM_Disconnect();
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

/* True only when GSM can't actually deliver telemetry right now — reuses
 * the exact same "is this link actually live" criteria already
 * established for the status bar (PushStatusBar()'s link_up), not a new
 * concept. Forward-declared here, defined below, since
 * TickPeriodicPushes() needs it before its own definition appears. */
static bool GsmLinkDown(uint32_t now);

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
        /* Once the transmitter is truly gone (link down, same check used
         * for the status bar), the last real distance sample is stale/
         * meaningless — TickDistanceFromLinks() only ever updates
         * g_hmi.distance_m when a fresh sample actually arrives, so
         * without this check it would otherwise stay locked on whatever
         * was last received forever. Checked first/takes priority over
         * the out-of-range case below, since a stale sample's numeric
         * value (>44m or not) says nothing real once the link itself is
         * down. "DC" (disconnected) vs "OR" (out of range) — explicit
         * request, distinguishing "transmitter gone" from "transmitter
         * present but beyond the active range" instead of both showing
         * the same "--". */
        if (GsmLinkDown(now)) {
            DWIN_WriteVPString(VP_DISTANCE, "DC", NUMBER_TEXT_FIELD_BYTES);
        } else if (g_hmi.distance_cm > DISTANCE_MAX_ACTIVE_CM) {
            DWIN_WriteVPString(VP_DISTANCE, "OR", NUMBER_TEXT_FIELD_BYTES);
        } else {
            char dist_str[16];
            snprintf(dist_str, sizeof(dist_str), "%u.%02u", g_hmi.distance_cm / 100u, g_hmi.distance_cm % 100u);
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

/* GSM-only, recency-based link-health derivation — no live RSSI polling
 * (would need a second, independent AT+CSQ command competing with
 * GSM_MQTT_Poll()'s own receive path for the same UART/response buffer,
 * risking a dropped incoming message on collision; not worth it for a
 * signal-bars nicety on a project whose top priority is reliability).
 * Only meaningful once connected; otherwise conn_health just holds its
 * last value, same as before there was a real link to report on. */
static void TickConnectionStatus(uint32_t now)
{
    if (GSM_GetState() == GSM_LINK_CONNECTED) {
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

/* Drains whatever GSM has most recently parsed into g_hmi.distance_cm.
 * Unread-since-last-call flag on the GSM side, so this never picks up a
 * stale leftover reading from before a device change. */
static void TickDistanceFromLinks(void)
{
    uint16_t gsm_distance_cm;
    if (GSM_GetLatestDistance(&gsm_distance_cm)) {
        g_hmi.distance_cm = gsm_distance_cm;
    }
}

static bool GsmLinkDown(uint32_t now)
{
    bool gsm_live = (GSM_GetState() == GSM_LINK_CONNECTED) &&
                    (GSM_GetMsSinceLastMessage(now) <= LINK_STALE_TIMEOUT_MS);
    return !gsm_live;
}

/* Ported from Shunting_Receiver_v2's TickObstacleOverlay(), unchanged
 * design — same overlay technique as TickChargerOverlay():
 * g_hmi.active_screen deliberately stays SCR_04_TELEMETRY throughout,
 * only g_hmi.obstacle_overlay_active + a direct DWIN_SwitchPage() change.
 * This is what lets TickBuzzer()'s proximity/alarm tone and
 * TickPeriodicPushes()'s status-bar/distance writes keep running exactly
 * as they already do on Telemetry, and Volume -/+ / END SHUNTING keep
 * working via HandleTouch()'s existing SCR_04_TELEMETRY case — none of
 * that needs to change.
 *
 * Only evaluated during an actual live Telemetry session, and skipped
 * entirely while the charger overlay is up (charging already suspends
 * GSM polling, so no new samples would arrive anyway — this is a
 * defensive guard, not expected to matter in practice).
 *
 * Three states, tracked locally (PENDING doesn't need to be in g_hmi —
 * it's purely an internal debounce detail, same as e.g. charger_detect.c's
 * own debounce statics):
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

    bool telemetry_active = (g_hmi.active_screen == SCR_04_TELEMETRY) && !g_hmi.charger_overlay_active;

    if (!telemetry_active) {
        s_have_last_distance = false;
        s_pending = false;
        if (g_hmi.obstacle_overlay_active) {
            /* Left Telemetry some other way (Change Device/End Shunting/
             * charger plugged in) while the warning was up — clear it
             * defensively rather than leaving it stuck active. */
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

/* Ported from Shunting_Receiver_v2's TickChargerOverlay() — same overlay
 * technique used there: active_screen keeps advancing underneath (see
 * EnterScreen()'s charger_overlay_active guard) while the physical
 * display stays pinned on page 13. On unplug, this is a genuine fresh
 * start rather than a resumed session — a separate physical power switch
 * (outside firmware's control) stays off during charging on this
 * hardware, so there's no session to return to. Reinitializes GSM the
 * same way it would from a cold boot; no LoRa_Reset() here since this
 * port has no LoRa. */
static void TickChargerOverlay(uint32_t now)
{
    static uint32_t next_charge_text_push;

    if (g_hmi.charger_plugged && !g_hmi.charger_overlay_active) {
        g_hmi.charger_overlay_active = true;
        DWIN_SwitchPage(PAGE_13_CHARGER_PLUGGED);
        char buf[8];
        snprintf(buf, sizeof(buf), "%u", g_hmi.charge_pct);
        DWIN_WriteVPString(VP_CHARGE_PCT_TEXT, buf, CHARGE_PCT_TEXT_FIELD_BYTES);
        next_charge_text_push = now + CHARGE_TEXT_PUSH_INTERVAL_MS;
        return;
    }

    if (!g_hmi.charger_plugged && g_hmi.charger_overlay_active) {
        g_hmi.charger_overlay_active = false;
        GSM_Reset();
        ScreenSM_ForceStartupReset(now);
        return;
    }

    if (g_hmi.charger_overlay_active && now >= next_charge_text_push) {
        next_charge_text_push = now + CHARGE_TEXT_PUSH_INTERVAL_MS;
        char buf[8];
        snprintf(buf, sizeof(buf), "%u", g_hmi.charge_pct);
        DWIN_WriteVPString(VP_CHARGE_PCT_TEXT, buf, CHARGE_PCT_TEXT_FIELD_BYTES);
    }
}

/* Ported from Shunting_Receiver_v2's TickBuzzer() — distance-band beep-
 * cadence thresholds (180/200/250/400ms) are its hand-verified "sounds
 * right" values, ported as-is. Its BothLinksDown() (GSM-or-LoRa) becomes
 * GsmLinkDown() here since this port has no LoRa — same "is the link to
 * the transmitter actually alive" concept, just one link instead of two. */
static void TickBuzzer(uint32_t now)
{
    Buzzer_SetVolume(g_hmi.volume_pct);

    if (g_hmi.active_screen != SCR_04_TELEMETRY) {
        Buzzer_SetContinuousTone(false);
        Buzzer_SetBeepIntervalMs(0u); /* silent outside an active shunting session */
        return;
    }

    if (GsmLinkDown(now)) {
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
    TickChargerOverlay(now_ms);
    TickPeriodicPushes(now_ms);
    TickBuzzer(now_ms);
}
