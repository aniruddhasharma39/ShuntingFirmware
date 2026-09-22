/* Battery State-of-Charge estimator for the INA226 + 10A/75mV shunt.
 * Ported from the working, hardware-tested implementation in the sibling
 * reference project Blackpill_Battery_Monitoring (see PROGRESS.md for the
 * full porting notes, including what was deliberately left out — USB
 * debug output and the reference's own DWIN-send helpers, neither of
 * which are needed here). Flash persistence (wear-leveled, sector 7) IS
 * ported — SoC survives a power cycle instead of re-estimating from a
 * single boot-time voltage sample every time.
 *
 * Owns g_hmi.battery_pct directly (writes it every tick), the same way
 * charger_detect.c owns the charger fields.
 */
#ifndef BATTERY_SOC_H
#define BATTERY_SOC_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* Initializes the INA226, loads any saved SoC from flash and resumes it
 * unconditionally, or — only if no valid record exists yet (genuine first
 * boot) — seeds from an open-circuit-voltage estimate instead. No
 * voltage-based disagreement check against a resumed value (removed —
 * see PROGRESS.md for why that was unreliable on this chemistry).
 * Blocking for ~500ms (sensor settle) — call once at boot, before the
 * main loop. */
void BatterySoc_Init(I2C_HandleTypeDef *hi2c);

/* Call every main-loop iteration; internally self-paced to update every
 * ~100ms, same non-blocking tick-function convention as charger_detect.c
 * and screen_sm.c. Also triggers INA226 bus recovery after repeated I2C
 * faults. */
void BatterySoc_Tick(uint32_t now_ms);

#endif /* BATTERY_SOC_H */
