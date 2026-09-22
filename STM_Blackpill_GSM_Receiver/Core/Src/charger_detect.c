#include "charger_detect.h"
#include "hmi_state.h"
#include "hmi_map.h"
#include "dwin_hmi.h"
#include <stdbool.h>

/* Threshold voltages ported from battey_pertest's finalized main.c
 * (CHARGER_DETECT_VOLTAGE / NEAR_FULL_DETECT_VOLTAGE there) — same
 * charger-indicator signal both boards read.
 * round(1.0f / 3.3f * 4095.0f) = 1241, round(2.0f / 3.3f * 4095.0f) = 2482. */
#define CHARGER_PRESENT_ADC_THRESHOLD_RAW    1241u
#define CHARGER_NEAR_FULL_ADC_THRESHOLD_RAW  2482u

/* Consecutive-hold time a new reading must persist before either flag
 * flips — placeholder pending bench tuning, same as LoRa's window /
 * buzzer's distance bands needed real-hardware tuning. battey_pertest's
 * own debounce is just a 5-sample oversample average with no hold time on
 * the digital flag itself; this accumulate-then-commit scheme is stronger
 * (matches battery_soc.c's EDV_DEBOUNCE_MS shape), which matters more here
 * since this flag now directly switches real power hardware, not just a
 * UI indicator. Measured against real elapsed time (HAL_GetTick()), not a
 * sample count — see UpdateDebouncedFlag(). */
#define CHARGER_DEBOUNCE_MS      150u

/* Re-assert period for DWIN_SwitchPage(PAGE_13_CHARGER_PLUGGED) even
 * after g_hmi.charger_overlay_active has already confirmed the switch —
 * see UpdateRelayMosfet()'s own comment for why this exists: on a
 * charger-triggered cold boot (this hardware can power the STM32 on via
 * the charger alone, bypassing the normal system switch), the DWIN
 * display's own power rail may still be settling while this ISR is
 * already firing — every attempt during the fast, every-tick retry
 * window before confirmation can be lost simply because the display
 * wasn't powered up enough yet to receive UART commands, and
 * charger_overlay_active confirms only that *software* processed the
 * transition, not that the display actually received anything. Slow
 * enough to coexist with TickChargerOverlay()'s own 1000ms percentage
 * refresh without starving it the way an every-tick repeat did. */
#define CHARGING_PAGE_HEARTBEAT_MS 1500u

/* ---- Relay control (ACTIVE-LOW relay module, PA8) ----
 * Ported verbatim from Shunting_Receiver_v2/battey_pertest: driving the
 * pin LOW turns the relay ON, HIGH turns it OFF — matches that board's
 * pin and polarity exactly, no conflict with anything already used in
 * this project. */
#define RELAY_ON()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
#define RELAY_OFF()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)

/* ---- MOSFET control (IRLZ44N, N-channel: HIGH = ON, LOW = OFF) ----
 * battey_pertest uses PA7 for this; Shunting_Receiver_v2 uses PA5 (PA7
 * already taken there). This project also uses PA5 for the same MOSFET
 * (confirmed by the user — the pre-existing hand-tuned "set PA5 HIGH at
 * boot" GPIO code in MX_GPIO_Init() was this same MOSFET's default-on
 * state, set before this driver existed). See charger_detect.h and
 * PROGRESS.md for the resulting boot-sequencing behavior. */
#define MOSFET_ON()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define MOSFET_OFF()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)

typedef enum { ADC_CH_PRESENT, ADC_CH_NEAR_FULL } adc_channel_t;

static ADC_HandleTypeDef *s_hadc;
static adc_channel_t s_next_channel;

static uint32_t s_present_disagree_since_ms;   /* 0 = not currently disagreeing */
static uint32_t s_near_full_disagree_since_ms; /* 0 = not currently disagreeing */

/* Debounced result of the PA6 (present-threshold) channel alone — by
 * explicit request, g_hmi.charger_plugged now also goes true whenever
 * PA7 (the near-full-threshold channel) senses voltage, not just PA6.
 * Each channel keeps its own independent debounce; only the final OR
 * combination in OnSample() is new. */
static bool s_present_flag;

static bool s_relay_mosfet_state_valid;
static bool s_last_applied_charger_plugged;
static uint32_t s_next_charging_heartbeat_ms; /* see CHARGING_PAGE_HEARTBEAT_MS */

/* Same accumulate-then-commit shape as battery_soc.c's EDV_DEBOUNCE_MS:
 * hold while a fresh sample disagrees with the currently-committed flag,
 * commit once that disagreement has held for CHARGER_DEBOUNCE_MS of REAL
 * elapsed time; any sample that agrees with the current flag resets it.
 * Tracks the tick disagreement actually started at and compares real
 * elapsed time against it, rather than accumulating a fixed per-call
 * increment — correct and prompt regardless of how often this function
 * happens to get called (relevant history: it used to be driven from the
 * main loop, where call frequency could vary a lot; now driven from a
 * fixed-period timer interrupt, so this matters less than it used to,
 * but the elapsed-time approach is still the more correct one to keep). */
static void UpdateDebouncedFlag(bool *flag, uint32_t *disagree_since_ms, bool raw_above_threshold, uint32_t now_ms)
{
    if (raw_above_threshold != *flag) {
        if (*disagree_since_ms == 0u) {
            *disagree_since_ms = now_ms;
        }
        if (now_ms - *disagree_since_ms >= CHARGER_DEBOUNCE_MS) {
            *flag = raw_above_threshold;
            *disagree_since_ms = 0u;
        }
    } else {
        *disagree_since_ms = 0u;
    }
}

/* g_hmi.charger_plugged ("detect charging") is true if EITHER sense
 * channel is above its own threshold: PA6 above the present threshold,
 * OR PA7 above the near-full threshold — by explicit request. Each
 * channel's own debounce timing is independent; this only combines the
 * two already-settled, independently-debounced flags with OR, recomputed
 * after every sample regardless of which channel it came from, so it's
 * always current. g_hmi.charger_near_full itself keeps its original,
 * separate meaning (used elsewhere for the charge_pct display gate and
 * battery_soc.c's confirmed-full snap) — this doesn't change what
 * near_full means, just adds it as a second way for charger_plugged to
 * become true. */
static void OnSample(adc_channel_t ch, uint32_t raw_counts, uint32_t now_ms)
{
    if (ch == ADC_CH_PRESENT) {
        UpdateDebouncedFlag(&s_present_flag, &s_present_disagree_since_ms,
                             raw_counts >= CHARGER_PRESENT_ADC_THRESHOLD_RAW, now_ms);
    } else {
        UpdateDebouncedFlag(&g_hmi.charger_near_full, &s_near_full_disagree_since_ms,
                             raw_counts >= CHARGER_NEAR_FULL_ADC_THRESHOLD_RAW, now_ms);
    }
    g_hmi.charger_plugged = s_present_flag || g_hmi.charger_near_full;
}

/* Drives the relay/MOSFET straight off the already-debounced
 * g_hmi.charger_plugged — no second debounce layer needed, the
 * CHARGER_DEBOUNCE_MS accumulate-then-commit above already keeps this
 * from chattering. GPIOs only get re-written on an actual change, same
 * "don't re-set on every tick" guard as battey_pertest's own
 * currentState pattern. Safe to call from interrupt context —
 * HAL_GPIO_WritePin() is a direct register write, no blocking.
 *
 * The DWIN screen follows the relay state too, by explicit request
 * (2026-08-30): "if relay turned ON, it should show charging screen, no
 * matter what." PA5 (the MOSFET here) is what actually turns the GSM
 * module's power on/off on this hardware, so relay/MOSFET switching
 * already happens instantly, right here in this ISR — but the physical
 * screen switch was originally left to screen_sm.c's
 * TickChargerOverlay(), which only runs from the main loop and can be
 * stuck for a long time behind an already-in-progress, deliberately
 * blocking GSM_MQTT_Poll() call. Moving the DWIN_SwitchPage() call into
 * this ISR fixed the *latency*, but every design after that had a real
 * problem, each found on real hardware:
 *   1. A single one-shot call on just the transition edge wasn't
 *      reliable — if that one attempt happened to lose dwin_hmi.c's
 *      rare tx-busy race against a concurrent main-loop DWIN write,
 *      nothing ever retried it, and the display could be stuck on the
 *      wrong page indefinitely even with the relay genuinely on.
 *   2. Repeating the switch on every single timer tick forever (to fix
 *      #1) broke the charge-percentage text instead: a DGUS page-switch
 *      command, even to the page already showing, appears to redraw the
 *      page fresh from its template — sent continuously, it was wiping
 *      TickChargerOverlay()'s own percentage text faster than its
 *      1000ms periodic refresh could keep up, so the field just showed
 *      its template placeholder ("--") permanently.
 *   3. Stopping entirely the moment g_hmi.charger_overlay_active
 *      confirms the main loop has caught up (to fix #2) turned out to
 *      have its own gap: charger_overlay_active only confirms that
 *      *software* processed the transition — not that the display
 *      actually received it. On this hardware, a charger-triggered cold
 *      boot (the charger itself can power the STM32 on, bypassing the
 *      normal system switch) means the DWIN display's own power rail
 *      may still be settling while this ISR is already firing — every
 *      fast-retry attempt during that startup window could be lost
 *      simply because the display wasn't powered up enough yet to
 *      receive UART commands, and once charger_overlay_active flipped
 *      true anyway (software doesn't know the display missed it), the
 *      ISR would go permanently silent for the rest of that charging
 *      session, leaving the screen stuck on whatever it showed before.
 * Fixed with two paces: fast (every tick) while unconfirmed, exactly as
 * in #3 — then, instead of going silent once confirmed, a much slower
 * heartbeat (CHARGING_PAGE_HEARTBEAT_MS) continues for as long as
 * charger_plugged stays true. The heartbeat is slow enough to coexist
 * with TickChargerOverlay()'s own 1000ms percentage refresh without
 * starving it the way #2's every-tick repeat did, while still
 * eventually reaching a display that wasn't ready during the initial
 * fast-retry window, and remaining resilient against any later
 * transient tx-busy loss too. Only applied to the charging (ON)
 * direction — the unplug direction doesn't have the same boot-time
 * "display might not be ready yet" concern (unplugging only happens
 * well into an already-running session, by which point the display has
 * obviously been up and responsive for a while), so it stays a simple
 * one-shot nudge until confirmed, unchanged. TickChargerOverlay() still
 * owns the full transition in both directions (charger_overlay_active
 * itself, the percentage text, and — unplug only — GSM_Reset()/
 * ScreenSM_ForceStartupReset()) from the main loop exactly as before;
 * this ISR only ever nudges the page, never touches that state
 * directly. */
static void UpdateRelayMosfet(uint32_t now_ms)
{
    bool changed = !s_relay_mosfet_state_valid || (g_hmi.charger_plugged != s_last_applied_charger_plugged);

    if (changed) {
        if (g_hmi.charger_plugged) {
            /* Charging present -> Relay ON, MOSFET OFF (battey_pertest logic) */
            RELAY_ON();
            MOSFET_OFF();
        } else {
            /* Charger absent -> Relay OFF, MOSFET ON */
            RELAY_OFF();
            MOSFET_ON();
        }
        s_last_applied_charger_plugged = g_hmi.charger_plugged;
        s_relay_mosfet_state_valid = true;
    }

    if (g_hmi.charger_plugged) {
        if (!g_hmi.charger_overlay_active || now_ms >= s_next_charging_heartbeat_ms) {
            DWIN_SwitchPage(PAGE_13_CHARGER_PLUGGED);
            s_next_charging_heartbeat_ms = now_ms + CHARGING_PAGE_HEARTBEAT_MS;
        }
    } else if (g_hmi.charger_overlay_active) {
        DWIN_SwitchPage(PAGE_00_STARTUP);
    }
}

void ChargerDetect_Init(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim)
{
    s_hadc = hadc;
    s_next_channel = ADC_CH_PRESENT;
    s_present_disagree_since_ms = 0u;
    s_near_full_disagree_since_ms = 0u;
    s_present_flag = false;

    g_hmi.charger_plugged = false;
    g_hmi.charger_near_full = false;
    g_hmi.charge_pct = (g_hmi.battery_pct < 99u) ? g_hmi.battery_pct : 99u;

    /* Safety default, same as battey_pertest's boot sequence: relay OFF
     * before anything else runs — matches PA8's own CubeMX-configured
     * boot state (High = off), so this is a harmless re-assertion.
     *
     * Deliberately NOT calling MOSFET_OFF() here, unlike the reference:
     * on this project, PA5 already starts HIGH (MOSFET ON) at very early
     * boot via the pre-existing hand-written GPIO code in MX_GPIO_Init()
     * — by explicit request, that HIGH state is left completely alone
     * through boot and only goes LOW once a genuinely-debounced
     * charger-present reading arrives. UpdateRelayMosfet() below still
     * applies the correct ON/OFF state from the very first timer
     * interrupt onward regardless. */
    RELAY_OFF();
    s_relay_mosfet_state_valid = false;

    /* Starts ChargerDetect_TimerCallback() firing periodically — see
     * charger_detect.h's top comment for why this module is
     * interrupt-driven rather than main-loop-tick-driven. */
    HAL_TIM_Base_Start_IT(htim);
}

void ChargerDetect_TimerCallback(void)
{
    uint32_t now_ms = HAL_GetTick();

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = (s_next_channel == ADC_CH_PRESENT) ? ADC_CHANNEL_6 : ADC_CHANNEL_7;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

    if (HAL_ADC_ConfigChannel(s_hadc, &sConfig) == HAL_OK &&
        HAL_ADC_Start(s_hadc) == HAL_OK) {
        /* Conversion completes in low single-digit microseconds at this
         * sampling time / ADC clock — a short bounded wait here is safe
         * inside an ISR (unlike the old main-loop version, which could
         * never block at all, per CLAUDE.md's CRITICAL CONSTRAINT on the
         * main loop specifically — that constraint doesn't apply to a
         * dedicated, low-duty-cycle timer ISR like this one). 2ms is
         * generous headroom, not an expected case; if it's ever hit, the
         * sample is just skipped for this cycle and retried next
         * interrupt on the same channel (s_next_channel only advances on
         * success). */
        if (HAL_ADC_PollForConversion(s_hadc, 2u) == HAL_OK) {
            uint32_t raw = HAL_ADC_GetValue(s_hadc);
            OnSample(s_next_channel, raw, now_ms);
            s_next_channel = (s_next_channel == ADC_CH_PRESENT) ? ADC_CH_NEAR_FULL : ADC_CH_PRESENT;
        }
        HAL_ADC_Stop(s_hadc);
    }

    /* Mirrors the real SoC from battery_soc.c every interrupt — this
     * module never computes its own percentage, only gates when 100% is
     * allowed to show. Held at 99% until charger_near_full trips, even if
     * the raw SoC already reads 100%, or is still far from it when
     * near-full trips first — either way this is purely a display gate,
     * per explicit request. */
    g_hmi.charge_pct = g_hmi.charger_near_full
        ? g_hmi.battery_pct
        : ((g_hmi.battery_pct < 99u) ? g_hmi.battery_pct : 99u);

    UpdateRelayMosfet(now_ms);
}
