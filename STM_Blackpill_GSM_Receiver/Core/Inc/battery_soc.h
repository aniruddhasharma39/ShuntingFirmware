/* Battery State-of-Charge estimator for the INA226 + 10A/75mV shunt, 3S
 * Li-ion (18650) pack, 4000mAh nominal.
 *
 * HOW IT WORKS (revised 2026-09-26): the SoC is tracked by Coulomb
 * counting (integrating the INA226's measured current) and PERSISTED to
 * flash (wear-leveled, sector 7) roughly every 1%, so it survives power
 * cycles. This pack is charged externally, off the board, so the firmware
 * can't see a charge happen; instead, each boot averages the bus voltage
 * for a few seconds and compares it with the voltage saved alongside the
 * last SoC record — a clear rise means the pack was recharged/swapped and
 * the SoC is re-seeded from voltage, otherwise the saved count is simply
 * resumed. (An earlier version re-derived SoC from voltage at EVERY boot;
 * because Li-ion voltage is nearly flat through mid-range, a real 10%
 * discharge barely moved it and the next boot re-seeded the old bucket.)
 * The displayed percentage is bucketed to the nearest 10% and only moves
 * to a new bucket after the underlying reading has implied it for several
 * continuous seconds. See battery_soc.c for the exact timings/thresholds
 * and PROGRESS.md for the full history.
 *
 * Owns g_hmi.battery_pct directly (writes it once the boot-time decision
 * completes, then again on every committed bucket change).
 */
#ifndef BATTERY_SOC_H
#define BATTERY_SOC_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* Initializes the INA226 (blocking ~500ms for sensor settle), loads the
 * last persisted SoC record from flash, and resets internal state so the
 * next few seconds of BatterySoc_Tick() calls perform the boot-time
 * voltage read and resume-or-reseed decision. g_hmi.battery_pct reads 0
 * until that completes — call once at boot, before the main loop. */
void BatterySoc_Init(I2C_HandleTypeDef *hi2c);

/* Call every main-loop iteration; internally self-paced to update every
 * ~100ms, same non-blocking tick-function convention as screen_sm.c.
 * Also triggers INA226 bus recovery after repeated I2C faults. Writes a
 * flash record roughly every 1% of SoC change. */
void BatterySoc_Tick(uint32_t now_ms);

#endif /* BATTERY_SOC_H */
