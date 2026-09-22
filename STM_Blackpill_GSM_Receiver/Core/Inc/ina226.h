/* Thin INA226 register-level driver — no SoC logic here, just I2C
 * transport. Mirrors dwin_hmi.h's philosophy: a dumb driver that
 * battery_soc.c calls into, not a peer module. Ported from the working,
 * hardware-tested implementation in the sibling reference project
 * Blackpill_Battery_Monitoring (see PROGRESS.md for details).
 */
#ifndef INA226_H
#define INA226_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/* Writes calibration (10A/75mV shunt) and config registers. Blocking,
 * HAL_MAX_DELAY — matches the reference; only called once at boot and
 * again from INA226_BusRecovery(). */
void INA226_Init(I2C_HandleTypeDef *hi2c);

/* Each returns true and writes *out on success; false (out left
 * untouched) on I2C failure — the caller (battery_soc.c) owns fault
 * counting and deciding when to call INA226_BusRecovery(), this module
 * doesn't track state across calls. */
bool INA226_ReadBusVoltage_V(I2C_HandleTypeDef *hi2c, float *out_volts);
bool INA226_ReadCurrent_A(I2C_HandleTypeDef *hi2c, float *out_amps);

/* De-init/re-init the I2C peripheral and re-run INA226_Init. Self
 * contained — does not call into main.c's generated MX_I2C1_Init(). */
void INA226_BusRecovery(I2C_HandleTypeDef *hi2c);

#endif /* INA226_H */
