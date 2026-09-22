#include "battery_soc.h"
#include "ina226.h"
#include "hmi_state.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

/* ---- pack constants, ported unchanged from Blackpill_Battery_Monitoring
 * (confirmed with the user: same physical pack/shunt as this receiver's
 * hardware spec, so these transfer as-is rather than being re-derived) */
#define NOMINAL_CAPACITY_AH     6.0
#define NOMINAL_CAPACITY_AS     (NOMINAL_CAPACITY_AH * 3600.0)

#define V_REST_FULL             13.50f /* lowered to catch settled surface charge */
#define V_REST_EMPTY            11.20f
#define REST_CURRENT_THRESHOLD   0.05f
#define REST_TIME_REQUIRED       300u  /* seconds of near-zero current before "at rest" */
#define CURRENT_NOISE_FLOOR_A    0.015f

#define CURRENT_OFFSET_INITIAL_A 0.0f
#define OFFSET_ADAPT_RATE        0.002f
#define OFFSET_MAX_DRIFT_A       0.6f

#define V_DISCONNECT_THRESHOLD   8.0f
#define V_LOAD_EMPTY_THRESHOLD  10.5f
#define EDV_DEBOUNCE_MS        3000u

/* If voltage > 13.7V and current has tapered, the pack is 100% full
 * (Valid Charge Termination). */
#define V_CHARGE_TERMINATION    13.70f
#define I_CHARGE_TAPER_A        -0.3f

#define CAPACITY_LEARN_ALPHA     0.3
#define CAPACITY_MIN_AS         (NOMINAL_CAPACITY_AS * 0.5)
#define CAPACITY_MAX_AS         (NOMINAL_CAPACITY_AS * 1.5)

#define EMA_ALPHA                0.05
#define TICK_INTERVAL_MS       100u
#define I2C_FAULT_RECOVERY_THRESHOLD 5u
#define BOOT_SETTLE_DELAY_MS    500u

/* How long after boot to ignore the EDV disconnect failsafe's "snap to
 * zero" conclusion — not the whole EDV block, just that one outcome.
 * Covers the brief window where the very first I2C transactions with the
 * INA226 can occasionally fail before the bus has fully settled: a
 * failed INA226_ReadBusVoltage_V() forces bus_V=0.0f, indistinguishable
 * in code from a genuine disconnect, and EDV_DEBOUNCE_MS (3s) of that
 * right after boot was enough to snap a perfectly healthy battery's
 * display to 0% — observed directly on hardware, not a hypothesis. A
 * real disconnect that persists past this window is still caught
 * normally; this only suppresses a false trigger in the first few
 * seconds after boot, at the cost of a real power-on disconnect taking
 * up to this much longer to display as 0%. */
#define EDV_BOOT_GRACE_MS      3000u

/* Genuine-first-boot fallback (no valid flash record at all yet) — a
 * fixed default instead of a voltage-based OCV estimate. This pack is
 * manually pre-charged to ~97% externally before a fresh unit is ever
 * first flashed, so a fixed value is actually more reliable than a
 * voltage read here: the same flat-discharge-curve unreliability that
 * justified removing the boot-time voltage re-seed check applies
 * equally to a first-boot voltage estimate, this is a consistent
 * follow-up to that fix, not a separate hack. The remaining ~5% gets
 * anchored to the real capacity the first time this unit is charged
 * through the system, via the normal charger_near_full + Coulomb-count
 * confirmed-full snap below — no special-casing needed for that. */
#define FRESH_UNIT_INITIAL_PCT   97.0f

/* Coulomb-count's own independent confirmation threshold for the
 * charger_near_full-triggered 100% snap — deliberately not 100%, since
 * Coulomb-count drift disagreeing with a voltage-confirmed full pack is
 * exactly the scenario this second signal exists to guard against.
 * Placeholder pending real-hardware tuning, same as every other timing/
 * threshold constant in this project. */
#define COULOMB_NEAR_FULL_FRACTION 0.90

typedef enum { CAL_EVENT_NONE = 0, CAL_EVENT_FULL, CAL_EVENT_EMPTY } cal_event_t;

/* ---- wear-leveled flash persistence (STM32F411, sector 7), ported
 * unchanged from the reference. Without this, BatterySoc_Init() has no
 * way to know the pack's actual SoC other than re-estimating it from a
 * single boot-time voltage sample every time — which is why SoC read
 * differently on every power cycle before this was added: a fresh OCV
 * guess depends on whatever load happens to be on the pack at that exact
 * instant, unlike resuming an exact Coulomb-counted value from flash. */
#define FLASH_RECORD_BASE_ADDR  0x08060000UL
#define FLASH_SECTOR_SIZE       (128UL * 1024UL)
#define FLASH_RECORD_SIZE       32UL
#define FLASH_RECORD_COUNT      (FLASH_SECTOR_SIZE / FLASH_RECORD_SIZE)
#define FLASH_RECORD_MAGIC_BASE 0x50C1A500UL

typedef struct {
    double   soc_as;
    double   capacity_as;
    uint32_t seq;
    uint32_t magic;
    uint8_t  reserved[8];
} soc_record_t;

static uint32_t s_next_slot_index;
static uint32_t s_next_seq = 1u;
static int      s_last_saved_percent = -1;

static uint32_t ComputeRecordMagic(uint32_t seq)
{
    return FLASH_RECORD_MAGIC_BASE ^ seq;
}

static bool LoadSocFromFlash(double *soc_as_out, double *capacity_as_out)
{
    uint32_t idx;
    bool found_valid = false;
    double last_soc = 0.0;
    double last_cap = NOMINAL_CAPACITY_AS;
    uint32_t last_seq = 0u;

    for (idx = 0; idx < FLASH_RECORD_COUNT; idx++) {
        const soc_record_t *rec = (const soc_record_t *)(FLASH_RECORD_BASE_ADDR + (idx * FLASH_RECORD_SIZE));

        if (rec->seq == 0xFFFFFFFFUL && rec->magic == 0xFFFFFFFFUL) {
            break; /* first never-written slot -- end of valid history */
        }

        if (rec->magic == ComputeRecordMagic(rec->seq) &&
            !isnan(rec->soc_as) && !isnan(rec->capacity_as) &&
            rec->soc_as >= 0.0 && rec->soc_as <= (NOMINAL_CAPACITY_AS * 1.5) &&
            rec->capacity_as >= CAPACITY_MIN_AS && rec->capacity_as <= CAPACITY_MAX_AS) {
            last_soc = rec->soc_as;
            last_cap = rec->capacity_as;
            last_seq = rec->seq;
            found_valid = true;
        }
    }

    s_next_slot_index = idx;
    s_next_seq = last_seq + 1u;

    if (found_valid) {
        *soc_as_out = last_soc;
        *capacity_as_out = last_cap;
        return true;
    }
    return false;
}

static void SaveSocToFlash(double soc_as, double capacity_as_val)
{
    HAL_FLASH_Unlock();

    if (s_next_slot_index >= FLASH_RECORD_COUNT) {
        FLASH_EraseInitTypeDef erase = {0};
        uint32_t sector_error = 0u;
        erase.TypeErase = FLASH_TYPEERASE_SECTORS;
        erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        erase.Sector = FLASH_SECTOR_7;
        erase.NbSectors = 1u;
        HAL_FLASHEx_Erase(&erase, &sector_error);
        s_next_slot_index = 0u;
    }

    soc_record_t rec;
    memset(&rec, 0xFF, sizeof(rec));
    rec.soc_as = soc_as;
    rec.capacity_as = capacity_as_val;
    rec.seq = s_next_seq;
    rec.magic = ComputeRecordMagic(s_next_seq);

    uint32_t addr = FLASH_RECORD_BASE_ADDR + (s_next_slot_index * FLASH_RECORD_SIZE);
    uint32_t words[FLASH_RECORD_SIZE / 4u];
    memcpy(words, &rec, sizeof(rec));

    for (uint32_t w = 0; w < (FLASH_RECORD_SIZE / 4u); w++) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + (w * 4u), words[w]);
    }

    HAL_FLASH_Lock();

    s_next_slot_index++;
    s_next_seq++;
}

static I2C_HandleTypeDef *s_hi2c;

static double s_current_SoC_As;
static double s_capacity_As;
static double s_displayed_SoC_percent;
static double s_lifetime_charge_odometer_As;
static double s_odometer_at_last_cal_event;
static cal_event_t s_last_cal_event_type;

static float   s_current_offset_A;
static float   s_current_A_last_good;
static uint8_t s_i2c_fault_count;

static uint32_t s_next_tick_ms;
static uint32_t s_last_integration_tick_ms;
static uint32_t s_edv_debounce_timer_ms;
static uint32_t s_boot_tick; /* see EDV_BOOT_GRACE_MS's own comment */

static uint8_t  s_subtick_counter; /* counts 100ms ticks up to 10, for the ~1s rest-check cadence */
static uint32_t s_rest_timer_s;
static uint8_t  s_rest_confirmed;

static bool s_last_confirmed_near_full; /* edge-detects (charger_near_full && coulomb_near_full) combined */

static float ApplyOffsetAndDeadband(float raw_current_A)
{
    float corrected = raw_current_A - s_current_offset_A;
    if (fabsf(corrected) < CURRENT_NOISE_FLOOR_A) {
        corrected = 0.0f;
    }
    return corrected;
}

void BatterySoc_Init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;
    INA226_Init(s_hi2c);
    HAL_Delay(BOOT_SETTLE_DELAY_MS); /* let the first 16-sample averaged conversion complete */

    s_current_offset_A = CURRENT_OFFSET_INITIAL_A;
    s_current_A_last_good = 0.0f;
    s_i2c_fault_count = 0u;

    double loaded_soc_As = 0.0;
    double loaded_capacity_As = NOMINAL_CAPACITY_AS;
    bool flash_valid = LoadSocFromFlash(&loaded_soc_As, &loaded_capacity_As);
    s_capacity_As = loaded_capacity_As;

    if (!flash_valid) {
        /* First boot, or no valid record yet — nothing to resume. Fixed
         * default, not a voltage estimate — see FRESH_UNIT_INITIAL_PCT's
         * own comment for why. */
        s_current_SoC_As = ((double)FRESH_UNIT_INITIAL_PCT / 100.0) * s_capacity_As;
    } else {
        /* Normal reboot: always resume the exact prior Coulomb-counted
         * value, unconditionally — no voltage-based disagreement check.
         * A voltage comparison against a resting-only, single-sample
         * estimate can't reliably tell "same pack, normal post-shutdown
         * voltage sag" from "different pack" on this chemistry: the OCV
         * table's middle rows span only ~0.25V for 45 percentage points,
         * so an ordinary sag reads as a large disagreement and used to
         * trigger an incorrect re-seed on nearly every boot. See
         * PROGRESS.md. Ongoing correction instead comes from the
         * rest-detection recalibration below and the charger_near_full
         * reset in BatterySoc_Tick(). */
        s_current_SoC_As = loaded_soc_As;
    }

    s_displayed_SoC_percent = (s_current_SoC_As / s_capacity_As) * 100.0;
    s_last_saved_percent = (int)(s_displayed_SoC_percent + 0.5);
    s_lifetime_charge_odometer_As = s_current_SoC_As;
    s_odometer_at_last_cal_event = 0.0;
    s_last_cal_event_type = CAL_EVENT_NONE;

    uint32_t now = HAL_GetTick();
    s_next_tick_ms = now;
    s_last_integration_tick_ms = now;
    s_boot_tick = now;
    s_edv_debounce_timer_ms = 0u;
    s_subtick_counter = 0u;
    s_rest_timer_s = 0u;
    s_rest_confirmed = 0u;
    s_last_confirmed_near_full = false;

    g_hmi.battery_pct = (uint8_t)s_last_saved_percent;
}

void BatterySoc_Tick(uint32_t now_ms)
{
    if (s_i2c_fault_count > I2C_FAULT_RECOVERY_THRESHOLD) {
        INA226_BusRecovery(s_hi2c);
        s_i2c_fault_count = 0u;
    }

    if (now_ms < s_next_tick_ms) {
        return;
    }
    s_next_tick_ms = now_ms + TICK_INTERVAL_MS;

    float raw_current_A;
    if (INA226_ReadCurrent_A(s_hi2c, &raw_current_A)) {
        s_i2c_fault_count = 0u;
    } else {
        s_i2c_fault_count++;
        /* Fallback matches the reference exactly: after Apply_Offset_And_
         * Deadband subtracts the offset back out, this nets to reusing
         * the last-known-good *corrected* current. */
        raw_current_A = s_current_A_last_good + s_current_offset_A;
    }
    float current_A = ApplyOffsetAndDeadband(raw_current_A);
    s_current_A_last_good = current_A;

    float bus_V;
    bool bus_v_valid = INA226_ReadBusVoltage_V(s_hi2c, &bus_V);
    if (bus_v_valid) {
        s_i2c_fault_count = 0u;
    } else {
        s_i2c_fault_count++;
        bus_V = 0.0f; /* placeholder only -- NOT fed into the EDV decision
                          below anymore; see bus_v_valid's use there. */
    }

    uint32_t now = HAL_GetTick();
    double dt_s = (double)(now - s_last_integration_tick_ms) / 1000.0;
    s_last_integration_tick_ms = now;

    /* 1. Base Coulomb integration */
    s_current_SoC_As -= ((double)current_A * dt_s);
    s_lifetime_charge_odometer_As -= ((double)current_A * dt_s);

    /* 2. Hardware failsafes: EDV (empty-detect voltage) and VCT (valid
     * charge termination) */
    uint8_t snap_to_zero = 0u;
    uint8_t snap_to_full = 0u;

    /* V_DISCONNECT_THRESHOLD (8.0V) is already below V_LOAD_EMPTY_THRESHOLD
     * (10.5V), so a single merged "low voltage" branch covers both — a
     * genuine disconnect and a merely-low reading both require the same
     * EDV_DEBOUNCE_MS of sustained low voltage before snapping to zero.
     *
     * Gated on bus_v_valid: a FAILED INA226_ReadBusVoltage_V() call is not
     * evidence of a low/disconnected pack and must never feed this
     * debounce. Before this gate, a failed read fell back to bus_V=0.0f
     * and was indistinguishable from a genuinely empty pack — so a run of
     * bad I2C reads (a few seconds' worth, e.g. right after boot before
     * the bus/sensor has settled, but not only then — any mid-operation
     * I2C glitch of the same duration could do it too) would snap SoC to
     * 0%. That snap has no fast way back (needs either a real charge-to-
     * full or 5 minutes of rest at a fully-charged voltage, see the rest-
     * detection block below), so one bad read streak meant a
     * *permanently* wrong 0% display, not just a momentary glitch —
     * observed directly on hardware (0% again after a routine reflash),
     * not a hypothesis. EDV_BOOT_GRACE_MS alone only delayed when this
     * could fire; it didn't stop a read failure from being misread as an
     * empty pack. Now a failed read this tick simply contributes no
     * evidence either way — the debounce timer holds at its last value
     * and a genuine empty/disconnected pack is still caught on the next
     * successful low read. */
    if (bus_v_valid) {
        if (bus_V < V_LOAD_EMPTY_THRESHOLD) {
            s_edv_debounce_timer_ms += TICK_INTERVAL_MS;
            if (s_edv_debounce_timer_ms >= EDV_DEBOUNCE_MS && (now - s_boot_tick) >= EDV_BOOT_GRACE_MS) {
                s_current_SoC_As = 0.0;
                snap_to_zero = 1u;
            }
        } else if (bus_V >= V_CHARGE_TERMINATION && current_A <= 0.0f && current_A >= I_CHARGE_TAPER_A) {
            s_current_SoC_As = s_capacity_As;
            s_edv_debounce_timer_ms = 0u;
            snap_to_full = 1u;
        } else {
            s_edv_debounce_timer_ms = 0u;
        }
    }

    /* 2b. "99% hold" — don't let SoC jump straight to 100% the instant
     * charge current merely dips; only once the charger has actually
     * confirmed tapered/terminated (prevents premature 100%). */
    if (s_current_SoC_As >= s_capacity_As) {
        if (current_A < -0.05f) {
            if (current_A < I_CHARGE_TAPER_A || bus_V < V_CHARGE_TERMINATION) {
                s_current_SoC_As = s_capacity_As * 0.994;
            } else {
                s_current_SoC_As = s_capacity_As;
            }
        } else {
            s_current_SoC_As = s_capacity_As;
        }
    }
    if (s_current_SoC_As < 0.0) {
        s_current_SoC_As = 0.0;
    }

    /* 2c. Charger-confirmed full: requires BOTH charger_near_full
     * (ADC-based, ~2.0V threshold, already debounced in
     * charger_detect.c) AND the Coulomb count independently already
     * being plausibly close to full (>= COULOMB_NEAR_FULL_FRACTION of
     * capacity) before committing an authoritative 100% snap. Voltage
     * alone isn't trusted — Coulomb-count drift disagreeing with a
     * voltage-confirmed full pack is exactly the scenario this second
     * signal exists to catch; forcing 100% off voltage alone would just
     * replace one wrong number with another. Edge-detects the COMBINED
     * condition, not each signal separately: while only one input is
     * true the AND stays false, and flips true at the exact instant the
     * second (later) one arrives — this fires exactly once, on
     * whichever signal becomes true last, with no need to track each
     * one's own edge individually. Placed after the 99%-hold block
     * above so a confirmed snap has the final say for this tick — if
     * charging current is still flowing when both conditions align
     * (plausible), the 99%-hold block above would otherwise immediately
     * clamp the freshly-set s_capacity_As back down to 99.4% in the
     * same tick. Coulomb counting resumes normally from this new
     * reference afterward, same as any other recalibration event in
     * this file. Reuses s_capacity_As as "100%" — that's what full
     * capacity already means throughout this file — not a new separate
     * constant. */
    bool coulomb_near_full = s_current_SoC_As >= (s_capacity_As * COULOMB_NEAR_FULL_FRACTION);
    bool confirmed_near_full = g_hmi.charger_near_full && coulomb_near_full;
    if (confirmed_near_full && !s_last_confirmed_near_full) {
        s_current_SoC_As = s_capacity_As;
        snap_to_full = 1u;
    }
    s_last_confirmed_near_full = confirmed_near_full;

    /* 3. Percentage + 4. EMA smoothing */
    double raw_SoC_percent = (s_current_SoC_As / s_capacity_As) * 100.0;
    if (snap_to_zero) {
        s_displayed_SoC_percent = 0.0;
    } else if (snap_to_full) {
        s_displayed_SoC_percent = 100.0;
    } else {
        s_displayed_SoC_percent = (raw_SoC_percent * EMA_ALPHA) + (s_displayed_SoC_percent * (1.0 - EMA_ALPHA));
    }

    int display_soc_int = (int)(s_displayed_SoC_percent + 0.5);
    g_hmi.battery_pct = (uint8_t)display_soc_int;

    /* Wear-leveled flash save — only when the rounded percent actually
     * changes, same gate the reference uses, so this isn't writing flash
     * every 100ms. This is what makes SoC survive a power cycle instead
     * of re-guessing from a single voltage sample on every boot. */
    if (display_soc_int != s_last_saved_percent) {
        SaveSocToFlash(s_current_SoC_As, s_capacity_As);
        s_last_saved_percent = display_soc_int;
    }

    /* 5. Rest detection -> current-sensor auto-zero + OCV recalibration +
     * capacity learning, checked roughly once per second (every 10th
     * 100ms tick). */
    float absolute_current = fabsf(current_A);
    if (++s_subtick_counter >= 10u) {
        s_subtick_counter = 0u;

        if (absolute_current < REST_CURRENT_THRESHOLD) {
            s_rest_timer_s++;
            if (s_rest_timer_s >= REST_TIME_REQUIRED) {
                s_rest_confirmed = 1u;
                s_rest_timer_s = REST_TIME_REQUIRED;
            }
        } else {
            s_rest_timer_s = 0u;
            s_rest_confirmed = 0u;
        }

        if (s_rest_confirmed) {
            s_current_offset_A += raw_current_A * OFFSET_ADAPT_RATE;
            if (s_current_offset_A > CURRENT_OFFSET_INITIAL_A + OFFSET_MAX_DRIFT_A) {
                s_current_offset_A = CURRENT_OFFSET_INITIAL_A + OFFSET_MAX_DRIFT_A;
            }
            if (s_current_offset_A < CURRENT_OFFSET_INITIAL_A - OFFSET_MAX_DRIFT_A) {
                s_current_offset_A = CURRENT_OFFSET_INITIAL_A - OFFSET_MAX_DRIFT_A;
            }

            cal_event_t this_event = CAL_EVENT_NONE;
            if (bus_V >= V_REST_FULL) {
                s_current_SoC_As = s_capacity_As;
                this_event = CAL_EVENT_FULL;
            } else if (bus_V <= V_REST_EMPTY && bus_V > V_DISCONNECT_THRESHOLD) {
                s_current_SoC_As = 0.0;
                this_event = CAL_EVENT_EMPTY;
            }

            if (this_event != CAL_EVENT_NONE) {
                if (s_last_cal_event_type != CAL_EVENT_NONE && this_event != s_last_cal_event_type) {
                    double measured_As = fabs(s_lifetime_charge_odometer_As - s_odometer_at_last_cal_event);
                    if (measured_As >= CAPACITY_MIN_AS && measured_As <= CAPACITY_MAX_AS) {
                        s_capacity_As = (CAPACITY_LEARN_ALPHA * measured_As) +
                                        ((1.0 - CAPACITY_LEARN_ALPHA) * s_capacity_As);
                    }
                }
                s_last_cal_event_type = this_event;
                s_odometer_at_last_cal_event = s_lifetime_charge_odometer_As;
            }
        }
    }
}
