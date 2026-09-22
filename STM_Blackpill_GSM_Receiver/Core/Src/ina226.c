#include "ina226.h"

/* 7-bit device address 0x40 (A1=A0=GND), HAL wants it pre-shifted. */
#define INA226_ADDR          (0x40u << 1)

#define REG_CONFIG           0x00u
#define REG_BUS_VOLTAGE      0x02u
#define REG_CURRENT          0x04u
#define REG_CALIBRATION      0x05u

/* CAL = 0.00512 / (Current_LSB * R_SHUNT); for the 10A/75mV shunt in
 * CLAUDE.md's hardware spec (R_SHUNT = 75mV/10A = 7.5mOhm) with
 * Current_LSB = 0.2mA, CAL = 0.00512 / (0.0002 * 0.0075) = 3413 = 0x0D55.
 * Matches the reference project's tested value exactly. */
#define INA226_CAL_VALUE     0x0D55u
#define CURRENT_LSB_MA        0.2f
#define BUS_VOLTAGE_LSB_V      0.00125f /* datasheet-fixed, 1.25mV/bit */

/* Bits[14:12]=100 (fixed) | AVG=010 (16-sample avg) | VBUSCT=100 (1.1ms) |
 * VSHCT=100 (1.1ms) | MODE=111 (shunt+bus, continuous). Device free-runs;
 * the MCU just polls the latest averaged register value. */
#define INA226_CONFIG_VALUE  0x4527u

#define I2C_READ_TIMEOUT_MS     10u
#define I2C_BUS_RECOVERY_DELAY_MS 5u

static bool WriteReg16(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t value, uint32_t timeout)
{
    uint8_t data[2] = { (uint8_t)(value >> 8), (uint8_t)(value & 0xFFu) };
    return HAL_I2C_Mem_Write(hi2c, INA226_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                              data, sizeof(data), timeout) == HAL_OK;
}

static bool ReadReg16(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t *out)
{
    uint8_t data[2];
    if (HAL_I2C_Mem_Read(hi2c, INA226_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                          data, sizeof(data), I2C_READ_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    *out = (uint16_t)((data[0] << 8) | data[1]);
    return true;
}

void INA226_Init(I2C_HandleTypeDef *hi2c)
{
    WriteReg16(hi2c, REG_CALIBRATION, INA226_CAL_VALUE, HAL_MAX_DELAY);
    WriteReg16(hi2c, REG_CONFIG, INA226_CONFIG_VALUE, HAL_MAX_DELAY);
}

bool INA226_ReadBusVoltage_V(I2C_HandleTypeDef *hi2c, float *out_volts)
{
    uint16_t raw;
    if (!ReadReg16(hi2c, REG_BUS_VOLTAGE, &raw)) {
        return false;
    }
    *out_volts = (float)raw * BUS_VOLTAGE_LSB_V;
    return true;
}

bool INA226_ReadCurrent_A(I2C_HandleTypeDef *hi2c, float *out_amps)
{
    uint16_t raw;
    if (!ReadReg16(hi2c, REG_CURRENT, &raw)) {
        return false;
    }
    float raw_mA = (float)(int16_t)raw * CURRENT_LSB_MA;
    *out_amps = raw_mA / 1000.0f;
    return true;
}

void INA226_BusRecovery(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_DeInit(hi2c);
    HAL_Delay(I2C_BUS_RECOVERY_DELAY_MS);

    /* Mirrors the CubeMX-generated MX_I2C1_Init() this driver depends on
     * (see the CubeMX setup note in battery_soc.h) — kept in sync with
     * that config rather than calling it directly, since it's a static
     * function generated inside main.c. */
    hi2c->Init.ClockSpeed      = 400000u;
    hi2c->Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c->Init.OwnAddress1     = 0u;
    hi2c->Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c->Init.OwnAddress2     = 0u;
    hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(hi2c);

    INA226_Init(hi2c);
}
