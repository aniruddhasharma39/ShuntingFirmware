#include "ina226.h"

/* 7-bit base address 0x40 (A1=A0=GND). The driver probes 0x40-0x4F at
 * INA226_Init() time and adopts whichever address answers — see
 * ProbeAndSelectAddress(). HAL wants the address pre-shifted. */
#define INA226_ADDR_BASE_7BIT   0x40u
#define INA226_ADDR_SCAN_COUNT  16u

#define REG_CONFIG           0x00u
#define REG_BUS_VOLTAGE      0x02u
#define REG_CURRENT          0x04u
#define REG_CALIBRATION      0x05u
#define REG_MANUFACTURER_ID  0xFEu
#define MANUFACTURER_ID_TI   0x5449u /* "TI" */

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
/* Finite (the reference used HAL_MAX_DELAY): with a wedged bus the
 * START/SB wait inside HAL_I2C_Mem_Write honors this timeout and would
 * otherwise hang boot forever. */
#define I2C_WRITE_TIMEOUT_MS    20u
#define I2C_PROBE_TIMEOUT_MS     3u
#define I2C_BUS_RECOVERY_DELAY_MS 5u

/* I2C1 pins — mirror the CubeMX-generated HAL_I2C_MspInit() (PB8 = SCL,
 * PB9 = SDA); needed here only for the bit-banged bus-clear and the
 * line-level readout, kept in sync by hand like the peripheral params in
 * ReinitPeripheral() below. */
#define I2C_PORT             GPIOB
#define I2C_SCL_PIN          GPIO_PIN_8
#define I2C_SDA_PIN          GPIO_PIN_9

/* Runtime-selected 7-bit address. Defaults to 0x40 (A1=A0=GND) but is
 * replaced by ProbeAndSelectAddress(): the receiver board's INA226 module
 * turned out (2026-09-26, found via the on-display probe result) to be
 * strapped to 0x44 by its solder jumpers, not 0x40 — every read had been
 * going to an address nothing answered at. */
static uint8_t s_addr7 = INA226_ADDR_BASE_7BIT;

static bool WriteReg16(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t value, uint32_t timeout)
{
    uint8_t data[2] = { (uint8_t)(value >> 8), (uint8_t)(value & 0xFFu) };
    return HAL_I2C_Mem_Write(hi2c, (uint16_t)(s_addr7 << 1), reg, I2C_MEMADD_SIZE_8BIT,
                              data, sizeof(data), timeout) == HAL_OK;
}

static bool ReadReg16At(I2C_HandleTypeDef *hi2c, uint8_t addr7, uint8_t reg, uint16_t *out)
{
    uint8_t data[2];
    if (HAL_I2C_Mem_Read(hi2c, (uint16_t)(addr7 << 1), reg, I2C_MEMADD_SIZE_8BIT,
                          data, sizeof(data), I2C_READ_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    *out = (uint16_t)((data[0] << 8) | data[1]);
    return true;
}

static bool ReadReg16(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t *out)
{
    return ReadReg16At(hi2c, s_addr7, reg, out);
}

static void ShortDelay(void)
{
    for (volatile uint32_t i = 0u; i < 300u; i++) {
    }
}

/* Mirrors the CubeMX-generated MX_I2C1_Init() this driver depends on —
 * kept in sync with that config rather than calling it directly, since
 * it's a static function generated inside main.c. */
static void ReinitPeripheral(I2C_HandleTypeDef *hi2c)
{
    hi2c->Init.ClockSpeed      = 400000u;
    hi2c->Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c->Init.OwnAddress1     = 0u;
    hi2c->Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c->Init.OwnAddress2     = 0u;
    hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(hi2c);
}

/* Standard I2C bus-clear: releases the peripheral's hold on both pins,
 * then — if a slave is still pulling SDA low (an INA226 left mid-byte by
 * an MCU reset or a noise glitch waits forever for the clocks that never
 * came) — clocks SCL up to 9 times until it lets go, and finishes with a
 * START+STOP so its state machine is definitely idle. Leaves the pins
 * de-initialized; the caller re-inits the peripheral (which re-applies
 * the alternate-function config via HAL_I2C_MspInit()). */
static void ReleaseAndClearBus(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_DeInit(hi2c);
    HAL_Delay(I2C_BUS_RECOVERY_DELAY_MS);

    GPIO_InitTypeDef g = {0};
    g.Pin   = I2C_SCL_PIN | I2C_SDA_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(I2C_PORT, &g);

    HAL_GPIO_WritePin(I2C_PORT, I2C_SCL_PIN | I2C_SDA_PIN, GPIO_PIN_SET);
    ShortDelay();

    for (uint32_t i = 0u; i < 9u && HAL_GPIO_ReadPin(I2C_PORT, I2C_SDA_PIN) == GPIO_PIN_RESET; i++) {
        HAL_GPIO_WritePin(I2C_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
        ShortDelay();
        HAL_GPIO_WritePin(I2C_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
        ShortDelay();
    }

    HAL_GPIO_WritePin(I2C_PORT, I2C_SDA_PIN, GPIO_PIN_RESET); /* START */
    ShortDelay();
    HAL_GPIO_WritePin(I2C_PORT, I2C_SDA_PIN, GPIO_PIN_SET);   /* STOP  */
    ShortDelay();

    HAL_GPIO_DeInit(I2C_PORT, I2C_SCL_PIN | I2C_SDA_PIN);
}

/* Finds the INA226. Tries the currently-selected address first, then
 * scans 0x40-0x4F; the first device whose manufacturer-ID register reads
 * "TI" wins, otherwise the first device that ACKs at all (some clones
 * report a different ID). If nothing ACKs the selected address is left
 * unchanged, so a device that shows up later at the old address still
 * works. Bails out of the scan immediately on HAL_BUSY (bus stuck — a
 * scan can't learn anything until the bus is cleared). */
static void ProbeAndSelectAddress(I2C_HandleTypeDef *hi2c)
{
    if (HAL_GPIO_ReadPin(I2C_PORT, I2C_SDA_PIN) == GPIO_PIN_RESET ||
        HAL_GPIO_ReadPin(I2C_PORT, I2C_SCL_PIN) == GPIO_PIN_RESET) {
        ReleaseAndClearBus(hi2c);
        ReinitPeripheral(hi2c);
    }

    int first_ack = -1;
    for (uint32_t n = 0u; n <= INA226_ADDR_SCAN_COUNT; n++) {
        uint8_t cand = (n == 0u) ? s_addr7 : (uint8_t)(INA226_ADDR_BASE_7BIT + (n - 1u));
        if (n != 0u && cand == s_addr7) {
            continue;
        }

        HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(cand << 1), 1u, I2C_PROBE_TIMEOUT_MS);
        if (st == HAL_BUSY) {
            break;
        }
        if (st != HAL_OK) {
            continue;
        }

        uint16_t id = 0u;
        if (ReadReg16At(hi2c, cand, REG_MANUFACTURER_ID, &id) && id == MANUFACTURER_ID_TI) {
            s_addr7 = cand;
            return;
        }
        if (first_ack < 0) {
            first_ack = (int)cand;
        }
    }

    if (first_ack >= 0) {
        s_addr7 = (uint8_t)first_ack;
    }
}

void INA226_Init(I2C_HandleTypeDef *hi2c)
{
    ProbeAndSelectAddress(hi2c);
    WriteReg16(hi2c, REG_CALIBRATION, INA226_CAL_VALUE, I2C_WRITE_TIMEOUT_MS);
    WriteReg16(hi2c, REG_CONFIG, INA226_CONFIG_VALUE, I2C_WRITE_TIMEOUT_MS);
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
    ReleaseAndClearBus(hi2c);
    ReinitPeripheral(hi2c);
    INA226_Init(hi2c);
}
