/* Thin INA226 register-level driver — no SoC logic here, just I2C
 * transport. Mirrors dwin_hmi.h's philosophy: a dumb driver that
 * battery_soc.c calls into, not a peer module. Ported from the working,
 * hardware-tested implementation in the sibling reference project
 * Blackpill_Battery_Monitoring (see PROGRESS.md for details).
 *
 * HARDENED (2026-09-26) beyond the reference, for failure modes that
 * reference never had to survive (it runs on a bench board with no GSM
 * modem powering up on the same ground at boot, and is power-cycled
 * along with the MCU):
 *   - I2C bus-clear (up to 9 SCL pulses + STOP) when a slave is holding
 *     SDA/SCL low — the standard fix for an INA226 left wedged
 *     mid-transaction by a noisy MCU reset/power event; HAL_I2C_DeInit/
 *     Init alone can't release a line the SLAVE is holding.
 *   - Address probe: scans 0x40-0x4F and prefers the device whose
 *     manufacturer-ID register (0xFE) reads 0x5449 ("TI"), so a module
 *     whose address solder-jumpers differ from the assumed 0x40 still
 *     works.
 *   - Finite write timeouts instead of HAL_MAX_DELAY, so a bad bus can
 *     never hang boot forever.
 *
 * The address probe is what actually fixed the receiver board (2026-09-26):
 * its INA226 module answers at 0x44, not the assumed 0x40 — see
 * PROGRESS.md.
 */
#ifndef INA226_H
#define INA226_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/* Clears a wedged bus if needed, probes for the device address, then
 * writes calibration (10A/75mV shunt) and config registers. Blocking but
 * bounded (a few ms typical; tens of ms worst case on a badly stuck bus).
 * Called once at boot and again from INA226_BusRecovery(). */
void INA226_Init(I2C_HandleTypeDef *hi2c);

/* Each returns true and writes *out on success; false (out left
 * untouched) on I2C failure — the caller (battery_soc.c) owns fault
 * counting and deciding when to call INA226_BusRecovery(), this module
 * doesn't track state across calls (apart from the selected address and
 * the diagnostic snapshot). */
bool INA226_ReadBusVoltage_V(I2C_HandleTypeDef *hi2c, float *out_volts);
bool INA226_ReadCurrent_A(I2C_HandleTypeDef *hi2c, float *out_amps);

/* De-init, release the bus (bus-clear), re-init the I2C peripheral and
 * re-run INA226_Init. Self contained — does not call into main.c's
 * generated MX_I2C1_Init(). */
void INA226_BusRecovery(I2C_HandleTypeDef *hi2c);

#endif /* INA226_H */
