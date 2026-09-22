/* Non-blocking buzzer/PWM driver, ported from Shunting_Receiver_v2 (12V
 * piezo buzzer through an IRLZ44N MOSFET gate on TIM2 CH1 / PA0 — see
 * PROGRESS.md's `## Buzzer` section for the full port writeup). This
 * project runs the same 96MHz clock as Shunting_Receiver_v2
 * (HSE 25MHz -> PLLM=25/PLLN=192/PLLP=DIV2 -> 96MHz HCLK, APB1TimFreq =
 * 96MHz), so its TIM2 Prescaler=9/Period=9999 values apply directly here
 * too — no recomputation needed.
 *
 * A dumb output driver, like dwin_hmi.c — it knows PWM/timer hardware
 * only, not g_hmi or screens. screen_sm.c is the one place that reads
 * g_hmi.distance_m/volume_pct/active_screen and decides what to tell
 * this driver to do, the same way it already reads GSM state and
 * decides what to write to the DWIN display.
 */
#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/* One-time setup: binds the timer, starts PWM channel 1 muted. Call
 * once at boot. */
void Buzzer_Init(TIM_HandleTypeDef *htim);

/* Loudness, 0/10/.../100 — matches g_hmi.volume_pct's existing range
 * exactly. Maps through a hand-tuned lookup table (ported verbatim from
 * the reference) rather than a linear scale, since the raw PWM duty
 * cycle needed to sound e.g. "half as loud" isn't linear on this
 * hardware. Takes effect on the next beep-ON phase; safe to call every
 * tick regardless of whether the value actually changed. */
void Buzzer_SetVolume(uint8_t volume_pct);

/* Beep on/off cadence in ms — 0 means fully silent (PWM held at 0%).
 * screen_sm.c derives this from g_hmi.distance_m; smaller values beep
 * faster (closer to the dead-end). Has no effect while a continuous
 * tone is active (see Buzzer_SetContinuousTone()) — that takes over the
 * PWM output until turned off. */
void Buzzer_SetBeepIntervalMs(uint32_t interval_ms);

/* true: hold a steady, non-toggling tone at the current volume — used
 * for "the link to the transmitter is completely gone" (GSM down),
 * deliberately distinct from the intermittent proximity beep so it
 * can't be mistaken for "just getting close to the dead-end."
 * false: return control to Buzzer_SetBeepIntervalMs()'s cadence. */
void Buzzer_SetContinuousTone(bool on);

/* Call every main-loop iteration, unconditionally — same non-blocking
 * tick-function convention as every other _Tick/_Poll in this project.
 * Never blocks (no HAL_Delay) — just a HAL_GetTick() delta compare and,
 * at most, a single PWM compare-register write. */
void Buzzer_Tick(uint32_t now_ms);

#endif /* BUZZER_H */
