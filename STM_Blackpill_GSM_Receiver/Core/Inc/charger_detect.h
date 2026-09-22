/* Charger-plugged detection AND charge-path switching for the receiver,
 * ported from Shunting_Receiver_v2 (itself ported from the finalized
 * battey_pertest reference), then adapted several times more by explicit
 * request for this project's specific hardware and architecture — see
 * PROGRESS.md for the full history.
 *
 * ARCHITECTURE (2026-08-30): unlike the reference, this module is
 * entirely timer-interrupt-driven, not main-loop-tick-driven. Real
 * hardware testing found charger-plugged detection noticeably slow and
 * inconsistent when it ran from the main loop, because this project's
 * proven GSM driver is deliberately blocking (that's the whole reason
 * this project uses it instead of Shunting_Receiver_v2's non-blocking
 * rewrite — see PROGRESS.md's `## Bug: Telemetry-screen ...` entry) and
 * shares the same single-threaded main loop: GSM_MQTT_Poll() routinely
 * blocks for up to ~100ms per call, and considerably longer during its
 * liveness check or a reconnect, which directly throttled how often ADC
 * sampling could run. Moving sampling onto its own hardware timer
 * interrupt (TIM3, see CubeMX setup notes in PROGRESS.md) makes charger
 * detection's responsiveness bounded purely by CHARGER_DEBOUNCE_MS,
 * completely independent of whatever GSM happens to be doing. This is a
 * genuine, deliberate divergence from the reference's architecture, not
 * just a pin/logic tweak — Shunting_Receiver_v2's own charger detection
 * stays main-loop-tick-driven there because its GSM driver never blocks
 * meaningfully, so it didn't need this.
 *
 * Two ADC sense channels, sampled one per timer interrupt (alternating):
 *   - "charger-present" threshold (~1.0V, PA6)  -> feeds g_hmi.charger_plugged
 *   - "near-full" threshold       (~2.0V, PA7)  -> g_hmi.charger_near_full,
 *     AND also feeds g_hmi.charger_plugged (by explicit request — see
 *     charger_detect.c's OnSample()). g_hmi.charger_near_full itself keeps
 *     its original, separate meaning (charge_pct display gate,
 *     battery_soc.c's confirmed-full snap) — it's just now ALSO one of
 *     two ways charger_plugged can become true, the other being PA6.
 *     g_hmi.charger_plugged = (PA6 above its threshold) OR (PA7 above
 *     its threshold), each independently debounced against real elapsed
 *     time (HAL_GetTick()), not a sample count.
 *
 * Two GPIO outputs, driven directly by g_hmi.charger_plugged, same
 * polarity/logic as battey_pertest:
 *   - Relay (ACTIVE-LOW module): ON (driven LOW) while charger present,
 *     OFF (driven HIGH) otherwise.
 *   - MOSFET (IRLZ44N, N-channel): OFF (driven LOW) while charger present,
 *     ON (driven HIGH) otherwise. battey_pertest puts this on PA7;
 *     Shunting_Receiver_v2 uses PA5 instead (PA7 already taken there).
 *     This project also uses PA5 for the same MOSFET — it already had a
 *     pre-existing, hand-tuned "set PA5 HIGH at boot" GPIO block (added
 *     before this driver existed, same default-ON intent). By explicit
 *     request, that HIGH state is left completely untouched through
 *     boot (ChargerDetect_Init() does NOT force it OFF the way the
 *     reference does) — PA5 stays HIGH continuously until the first
 *     genuinely-debounced charger-present reading, from either channel,
 *     switches it OFF. See PROGRESS.md.
 *
 * A single combined module, same ownership shape as battery_soc.c: reads
 * its own sensors, drives its own outputs, and writes directly into g_hmi
 * every interrupt, no separate app-logic-owner file. Also owns merging
 * the real INA226 SoC (g_hmi.battery_pct, "the ONLY place SoC logic
 * lives" per CLAUDE.md) into g_hmi.charge_pct for the 13_Charger_Plugged
 * display, capped at 99% until charger_near_full trips.
 *
 * Thread-safety note: g_hmi.charger_plugged/charger_near_full/charge_pct
 * are written here from interrupt context and read from the main loop
 * (screen_sm.c, main.c's GSM-suspend gate) and from battery_soc.c's own
 * main-loop tick. No lock is used — each field is a single bool/uint8_t,
 * whose reads/writes are inherently atomic on this architecture, and
 * these are continuously-valid state (not discrete events that could be
 * dropped), so a stale-by-one-interrupt-period read is harmless. This is
 * a lighter-weight case than the DWIN touch-code producer/consumer FIFO
 * (dwin_hmi.c), which exists because touch codes are discrete events
 * that must not be lost or merged.
 */
#ifndef CHARGER_DETECT_H
#define CHARGER_DETECT_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* One-time setup: binds the ADC + timer handles, seeds charge_pct from
 * whatever g_hmi.battery_pct already holds, and starts the periodic
 * sampling timer in interrupt mode (HAL_TIM_Base_Start_IT()). From this
 * point on, charger detection runs entirely from
 * ChargerDetect_TimerCallback(), fully independent of the main loop.
 * Call once at boot, after BatterySoc_Init(). */
void ChargerDetect_Init(ADC_HandleTypeDef *hadc, TIM_HandleTypeDef *htim);

/* Call from HAL_TIM_PeriodElapsedCallback() when htim is the timer bound
 * in ChargerDetect_Init() (see main.c's USER CODE BEGIN 4). Samples one
 * ADC channel (present/near-full alternate each call), updates
 * g_hmi.charger_plugged/charger_near_full/charge_pct, and drives the
 * relay/MOSFET — all synchronously within this ISR, bounded by a short
 * (2ms) worst-case wait on the ADC conversion itself. Not part of the
 * main-loop _Tick() convention used elsewhere in this project — never
 * call this from the main loop. */
void ChargerDetect_TimerCallback(void);

#endif /* CHARGER_DETECT_H */
