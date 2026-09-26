#include "battery_soc.h"
#include "ina226.h"
#include "hmi_state.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* ---- pack constants (3S Li-ion / 18650, 4000mAh) ---- */
#define NOMINAL_CAPACITY_AH     4.0
#define NOMINAL_CAPACITY_AS     (NOMINAL_CAPACITY_AH * 3600.0)

#define CURRENT_NOISE_FLOOR_A    0.015f
#define CURRENT_OFFSET_INITIAL_A 0.0f
#define OFFSET_ADAPT_RATE        0.002f
#define OFFSET_MAX_DRIFT_A       0.6f
#define REST_CURRENT_THRESHOLD   0.05f
#define REST_TIME_REQUIRED       300u  /* seconds of near-zero current before "at rest" */

#define EMA_ALPHA                0.05
#define TICK_INTERVAL_MS       100u
#define I2C_FAULT_RECOVERY_THRESHOLD 5u
#define BOOT_SETTLE_DELAY_MS    500u

/* Bus recovery is rate-limited (2026-09-26): INA226_BusRecovery() now
 * does a full bus-clear + address probe, which is much heavier than the
 * old bare DeInit/Init, and a permanently dead sensor used to retrigger
 * it every ~3 ticks. */
#define I2C_RECOVERY_MIN_INTERVAL_MS 1000u

/* After this many consecutive ticks with BOTH register reads failing, the
 * tick rate backs off to BACKOFF_TICK_INTERVAL_MS (from TICK_INTERVAL_MS)
 * until a read succeeds again. A dead/absent sensor otherwise costs up to
 * ~25ms of blocking per failed read on a stuck bus (HAL's fixed BUSY-flag
 * wait) — on this shared, single-threaded main loop that would visibly
 * hitch the buzzer cadence, LoRa slot timing and DWIN touch response for
 * as long as the fault lasts. */
#define FAILED_TICKS_BEFORE_BACKOFF 20u
#define BACKOFF_TICK_INTERVAL_MS    1000u

/* Boot-time voltage read window — averaged over this long (not a single
 * sample) to ride out any brief noise/transient right at power-on. The
 * average is used to decide whether the pack was recharged/swapped since
 * the last session (see CommitBootSeed()) and, only if so, to seed a fresh
 * starting SoC from voltage. */
#define BOOT_VOLTAGE_SAMPLE_MS  3000u

/* Fallback if literally zero valid voltage samples were captured during
 * the boot window (I2C down the whole time) — a neutral "unknown, assume
 * half" default rather than falsely showing empty or full. It is shown
 * but NOT persisted, and replaced as soon as reads work (see
 * s_seed_is_fallback). */
#define BOOT_VOLTAGE_FALLBACK_PCT 50u

/* Recharge / pack-swap detection at boot (2026-09-26). This pack is
 * charged externally, off the board, so the firmware can't watch a charge
 * happen. Instead of re-deriving SoC from voltage at every boot — which
 * threw away the Coulomb count and, because a Li-ion pack's voltage is
 * nearly flat through the middle of its range, kept re-seeding the same
 * bucket after a real 10% discharge — the Coulomb-counted SoC is now
 * PERSISTED across power cycles and simply resumed. Voltage is used only
 * to notice that something changed while the board was off, by comparing
 * this boot's average bus voltage against the (filtered) bus voltage
 * saved with the last SoC record:
 *   - risen by at least RECHARGE_DETECT_RISE_V  -> pack was recharged (or
 *     swapped for a fuller one): re-seed from voltage.
 *   - fallen by at least PACK_SWAP_DROP_V       -> pack was swapped for an
 *     emptier one (or heavily self-discharged): re-seed from voltage (the
 *     conservative direction).
 *   - anything in between                       -> same charge state:
 *     resume the saved Coulomb count.
 * Comparing voltage against voltage from the SAME sensor (rather than
 * against an absolute voltage-to-percent table) cancels constant
 * bias/gain errors in the INA226's voltage reading — this board's reads
 * ~0.5V high — and sidesteps the flat-curve problem entirely. The 0.50V
 * rise threshold sits above the load-dependent sag differences between
 * "voltage at last shutdown, under a ~2.3A load" and "voltage at boot"
 * (roughly 0.1-0.4V depending on pack/wiring resistance), but below what
 * recharging a pack that was actually run down produces (0.6V+ even from
 * ~70% back to full). Deliberately conservative: a false "recharged"
 * verdict would OVERSTATE the charge (the dangerous direction), while a
 * top-up too small to clear the threshold goes undetected and merely
 * leaves the SoC reading lower than reality. Tune against real
 * hardware if a genuine recharge is missed. */
#define RECHARGE_DETECT_RISE_V   0.50f
#define PACK_SWAP_DROP_V         0.60f

/* Displayed SoC is bucketed to the nearest 10% and only allowed to move
 * to a new bucket once the underlying (EMA-smoothed) reading has implied
 * that SAME new bucket continuously for this long — by explicit request,
 * so an ordinary load transient (a current draw spike sagging the
 * reading for a second or two) can't flicker the display between
 * adjacent buckets. Same accumulate-then-commit debounce shape used
 * elsewhere in this project — see ApplyBucketHold(). */
#define BUCKET_HOLD_MS          5000u

/* Hard safety floor: if bus voltage is genuinely this low, the bucket is
 * forced toward 0% regardless of what Coulomb counting currently
 * believes — a cross-check against Coulomb-count drift, not a
 * replacement for it (still subject to the same BUCKET_HOLD_MS hold
 * before it's actually shown, same as any other bucket change). 3.00V/
 * cell for this 3S Li-ion pack, matching its standard safe discharge
 * floor. */
#define V_LOAD_EMPTY_THRESHOLD   9.0f

typedef struct {
    float   v;
    uint8_t pct;
} ocv_point_t;

/* Voltage-to-SoC table for this 3S Li-ion (18650) pack, used ONLY when a
 * fresh starting SoC has to be seeded from voltage (first ever boot, or a
 * detected recharge/swap). Simple linear map, by explicit request
 * (2026-09-25): 12.00V = 100%, 9.00V = 0% (matching V_LOAD_EMPTY_THRESHOLD's
 * safe-discharge floor). Li-ion is far flatter through the middle of its
 * range than this line suggests, so a voltage-derived seed is only
 * trustworthy near the top and bottom — which is exactly why it is no
 * longer used to re-seed on every boot. */
static const ocv_point_t OCV_TABLE[] = {
    {  9.00f,   0u },
    { 12.00f, 100u },
};
#define OCV_TABLE_LEN (sizeof(OCV_TABLE) / sizeof(OCV_TABLE[0]))

static uint8_t VoltageToPercent(float v)
{
    if (v <= OCV_TABLE[0].v) {
        return OCV_TABLE[0].pct;
    }
    if (v >= OCV_TABLE[OCV_TABLE_LEN - 1u].v) {
        return OCV_TABLE[OCV_TABLE_LEN - 1u].pct;
    }
    for (size_t i = 0; i + 1u < OCV_TABLE_LEN; i++) {
        if (v >= OCV_TABLE[i].v && v <= OCV_TABLE[i + 1u].v) {
            float span = OCV_TABLE[i + 1u].v - OCV_TABLE[i].v;
            float frac = (span > 0.0f) ? ((v - OCV_TABLE[i].v) / span) : 0.0f;
            float pct = (float)OCV_TABLE[i].pct +
                        frac * (float)(OCV_TABLE[i + 1u].pct - OCV_TABLE[i].pct);
            return (uint8_t)(pct + 0.5f);
        }
    }
    return OCV_TABLE[OCV_TABLE_LEN - 1u].pct; /* unreachable, keeps the compiler happy */
}

static uint8_t RoundToNearestBucket(double pct)
{
    if (pct < 0.0) {
        pct = 0.0;
    }
    if (pct > 100.0) {
        pct = 100.0;
    }
    int bucket = ((int)((pct / 10.0) + 0.5)) * 10;
    if (bucket > 100) {
        bucket = 100;
    }
    if (bucket < 0) {
        bucket = 0;
    }
    return (uint8_t)bucket;
}

/* ---- wear-leveled flash persistence (STM32F411, sector 7) ----
 * Restored 2026-09-26 (removed in the 2026-09-25 redesign, which turned
 * out to be a mistake — see the recharge-detection comment above). Same
 * scheme as before: 32-byte records appended one after another through
 * the 128KB sector; boot scans forward to the newest valid one; the
 * sector is only erased when full. Records are written whenever the
 * rounded SoC percent changes (~every 1%), so at most ~1% of progress is
 * lost to an abrupt power cut.
 *
 * FLASH_RECORD_MAGIC_BASE deliberately differs from the pre-redesign
 * format's (0x50C1A500): records left in this sector by older firmware
 * describe a different pack/capacity and must never be resurrected, and
 * this record now also carries the bus voltage. The firmware image itself
 * lives in the first sectors, far from sector 7. */
#define FLASH_RECORD_BASE_ADDR  0x08060000UL
#define FLASH_SECTOR_SIZE       (128UL * 1024UL)
#define FLASH_RECORD_SIZE       32UL
#define FLASH_RECORD_COUNT      (FLASH_SECTOR_SIZE / FLASH_RECORD_SIZE)
#define FLASH_RECORD_MAGIC_BASE 0x50C1A600UL

/* If the sector is this close to full at boot it is erased right then
 * (the newest record is immediately rewritten), so the ~1-2s stall of a
 * sector erase — which halts the whole CPU — happens at boot instead of
 * in the middle of operation. */
#define FLASH_NEARLY_FULL_MARGIN 16UL

typedef struct {
    double   soc_as;
    double   capacity_as; /* NOMINAL_CAPACITY_AS at save time — sanity check only */
    uint32_t seq;
    uint32_t magic;
    float    last_bus_V;  /* filtered bus voltage when this record was written */
    uint32_t reserved;
} soc_record_t;

_Static_assert(sizeof(soc_record_t) == FLASH_RECORD_SIZE, "soc_record_t must be exactly one flash record");

static uint32_t s_next_slot_index;
static uint32_t s_next_seq = 1u;

static uint32_t ComputeRecordMagic(uint32_t seq)
{
    return FLASH_RECORD_MAGIC_BASE ^ seq;
}

static bool LoadStateFromFlash(double *soc_as_out, float *last_bus_V_out)
{
    uint32_t idx;
    bool found_valid = false;
    double last_soc = 0.0;
    float last_v = 0.0f;
    uint32_t last_seq = 0u;

    for (idx = 0; idx < FLASH_RECORD_COUNT; idx++) {
        const soc_record_t *rec = (const soc_record_t *)(FLASH_RECORD_BASE_ADDR + (idx * FLASH_RECORD_SIZE));

        if (rec->seq == 0xFFFFFFFFUL && rec->magic == 0xFFFFFFFFUL) {
            break; /* first never-written slot -- end of history */
        }

        if (rec->magic == ComputeRecordMagic(rec->seq) &&
            !isnan(rec->soc_as) && rec->soc_as >= 0.0 && rec->soc_as <= (NOMINAL_CAPACITY_AS * 1.05) &&
            !isnan(rec->capacity_as) && fabs(rec->capacity_as - NOMINAL_CAPACITY_AS) < 1.0) {
            last_soc = rec->soc_as;
            last_v = rec->last_bus_V;
            last_seq = rec->seq;
            found_valid = true;
        }
    }

    s_next_slot_index = idx;
    s_next_seq = last_seq + 1u;

    if (found_valid) {
        *soc_as_out = last_soc;
        *last_bus_V_out = last_v;
        return true;
    }
    return false;
}

static void EraseSector(void)
{
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0u;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase.Sector = FLASH_SECTOR_7;
    erase.NbSectors = 1u;
    HAL_FLASHEx_Erase(&erase, &sector_error);
    HAL_FLASH_Lock();
    s_next_slot_index = 0u;
}

static void SaveStateToFlash(double soc_as, float last_bus_V)
{
    if (s_next_slot_index >= FLASH_RECORD_COUNT) {
        EraseSector(); /* fallback only — normally done at boot, see FLASH_NEARLY_FULL_MARGIN */
    }

    soc_record_t rec;
    memset(&rec, 0xFF, sizeof(rec));
    rec.soc_as = soc_as;
    rec.capacity_as = NOMINAL_CAPACITY_AS;
    rec.seq = s_next_seq;
    rec.magic = ComputeRecordMagic(s_next_seq);
    rec.last_bus_V = last_bus_V;

    uint32_t addr = FLASH_RECORD_BASE_ADDR + (s_next_slot_index * FLASH_RECORD_SIZE);
    uint32_t words[FLASH_RECORD_SIZE / 4u];
    memcpy(words, &rec, sizeof(rec));

    HAL_FLASH_Unlock();
    for (uint32_t w = 0; w < (FLASH_RECORD_SIZE / 4u); w++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + (w * 4u), words[w]) != HAL_OK) {
            break;
        }
    }
    HAL_FLASH_Lock();

    s_next_slot_index++; /* advance even on a failed program: never retry a dirty slot */
    s_next_seq++;
}

static I2C_HandleTypeDef *s_hi2c;

static double s_current_SoC_As;
static double s_displayed_SoC_percent; /* EMA-smoothed, NOT bucketed */

static float   s_current_offset_A;
static float   s_current_A_last_good;
static uint8_t s_i2c_fault_count;

static uint32_t s_next_tick_ms;
static uint32_t s_last_integration_tick_ms;
static uint32_t s_boot_tick;

static uint8_t  s_subtick_counter; /* counts 100ms ticks up to 10, for the ~1s rest-check cadence */
static uint32_t s_rest_timer_s;
static uint8_t  s_rest_confirmed;

/* Boot-time voltage-read state — see BOOT_VOLTAGE_SAMPLE_MS. */
static bool     s_boot_measurement_done;
static double   s_boot_voltage_sum;
static uint32_t s_boot_voltage_samples;

/* The record found in flash at boot, if any — compared against the boot
 * voltage average in CommitBootSeed(). */
static bool     s_saved_valid;
static double   s_saved_soc_As;
static float    s_saved_last_V;

/* Filtered bus voltage, written into each saved record. */
static float    s_bus_v_filt;
static int      s_last_saved_percent = -1;

/* True while the displayed starting SoC is only the neutral
 * BOOT_VOLTAGE_FALLBACK_PCT placeholder (no valid voltage sample landed
 * in the boot window, e.g. the sensor was wedged by a boot-time
 * transient). While set, nothing is persisted; once bus reads start
 * working, BatterySoc_Tick() re-runs the boot decision and replaces the
 * placeholder. */
static bool     s_seed_is_fallback;

static uint32_t s_last_recovery_ms;
static uint32_t s_failed_ticks;

/* Bucketed display + its own hold-time debounce — see BUCKET_HOLD_MS. */
static uint8_t  s_displayed_bucket;
static uint8_t  s_pending_bucket;
static uint32_t s_bucket_disagree_since_ms; /* 0 = not currently disagreeing */

static float ApplyOffsetAndDeadband(float raw_current_A)
{
    float corrected = raw_current_A - s_current_offset_A;
    if (fabsf(corrected) < CURRENT_NOISE_FLOOR_A) {
        corrected = 0.0f;
    }
    return corrected;
}

/* Same accumulate-then-commit shape as this project's other debounces:
 * only commits raw_bucket once it has been the SAME candidate value,
 * continuously, for BUCKET_HOLD_MS — a candidate that changes again
 * before committing restarts the hold from that new value, rather than
 * accumulating credit across different candidates. */
static void ApplyBucketHold(uint8_t raw_bucket, uint32_t now_ms)
{
    if (raw_bucket != s_displayed_bucket) {
        if (raw_bucket != s_pending_bucket || s_bucket_disagree_since_ms == 0u) {
            s_pending_bucket = raw_bucket;
            s_bucket_disagree_since_ms = now_ms;
        }
        if (now_ms - s_bucket_disagree_since_ms >= BUCKET_HOLD_MS) {
            s_displayed_bucket = raw_bucket;
            s_bucket_disagree_since_ms = 0u;
            g_hmi.battery_pct = s_displayed_bucket;
        }
    } else {
        s_bucket_disagree_since_ms = 0u;
    }
}

/* Commits a starting SoC (in amp-seconds): seeds the Coulomb count, the
 * smoothed percentage, and the displayed/pending bucket all at once, with
 * no hold delay (there's no previously-shown value to protect). */
static void CommitState(double soc_as)
{
    if (soc_as < 0.0) {
        soc_as = 0.0;
    }
    if (soc_as > NOMINAL_CAPACITY_AS) {
        soc_as = NOMINAL_CAPACITY_AS;
    }
    double pct = (soc_as / NOMINAL_CAPACITY_AS) * 100.0;
    uint8_t bucket = RoundToNearestBucket(pct);

    s_current_SoC_As = soc_as;
    s_displayed_SoC_percent = pct;
    s_displayed_bucket = bucket;
    s_pending_bucket = bucket;
    s_bucket_disagree_since_ms = 0u;
    g_hmi.battery_pct = bucket;
}

static void SaveState(void)
{
    SaveStateToFlash(s_current_SoC_As, s_bus_v_filt);
    s_last_saved_percent = (int)(s_displayed_SoC_percent + 0.5);
}

/* Decides the starting SoC from the boot-time voltage average: resume the
 * persisted Coulomb count if the pack looks like the same charge state as
 * last session, otherwise (first boot, recharge, pack swap) seed from
 * voltage. See RECHARGE_DETECT_RISE_V's comment for the reasoning. */
static void CommitBootSeed(float avg_V)
{
    bool resume = false;
    if (s_saved_valid && isfinite(s_saved_last_V) && s_saved_last_V > 5.0f) {
        float dv = avg_V - s_saved_last_V;
        resume = (dv < RECHARGE_DETECT_RISE_V) && (dv > -PACK_SWAP_DROP_V);
    }

    if (resume) {
        CommitState(s_saved_soc_As);
    } else {
        uint8_t bucket = RoundToNearestBucket((double)VoltageToPercent(avg_V));
        CommitState(((double)bucket / 100.0) * NOMINAL_CAPACITY_AS);
    }

    s_bus_v_filt = avg_V;
    SaveState();
}

void BatterySoc_Init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;
    INA226_Init(s_hi2c);
    HAL_Delay(BOOT_SETTLE_DELAY_MS); /* let the first 16-sample averaged conversion complete */

    s_saved_valid = LoadStateFromFlash(&s_saved_soc_As, &s_saved_last_V);
    if (s_next_slot_index + FLASH_NEARLY_FULL_MARGIN >= FLASH_RECORD_COUNT) {
        EraseSector();
        if (s_saved_valid) {
            SaveStateToFlash(s_saved_soc_As, s_saved_last_V); /* keep the state across the erase */
        }
    }

    s_current_offset_A = CURRENT_OFFSET_INITIAL_A;
    s_current_A_last_good = 0.0f;
    s_i2c_fault_count = 0u;

    s_current_SoC_As = 0.0;
    s_displayed_SoC_percent = 0.0;

    s_boot_measurement_done = false;
    s_boot_voltage_sum = 0.0;
    s_boot_voltage_samples = 0u;
    s_seed_is_fallback = false;
    s_failed_ticks = 0u;
    s_bus_v_filt = 0.0f;
    s_last_saved_percent = -1;

    s_displayed_bucket = 0u;
    s_pending_bucket = 0u;
    s_bucket_disagree_since_ms = 0u;

    uint32_t now = HAL_GetTick();
    s_last_recovery_ms = now - I2C_RECOVERY_MIN_INTERVAL_MS; /* allow an immediate first recovery */
    s_next_tick_ms = now;
    s_last_integration_tick_ms = now;
    s_boot_tick = now;
    s_subtick_counter = 0u;
    s_rest_timer_s = 0u;
    s_rest_confirmed = 0u;

    g_hmi.battery_pct = 0u; /* nothing meaningful yet — set for real once
                                the boot voltage read completes, a few
                                seconds into BatterySoc_Tick(). */
}

void BatterySoc_Tick(uint32_t now_ms)
{
    if (s_i2c_fault_count > I2C_FAULT_RECOVERY_THRESHOLD &&
        (now_ms - s_last_recovery_ms) >= I2C_RECOVERY_MIN_INTERVAL_MS) {
        INA226_BusRecovery(s_hi2c);
        s_i2c_fault_count = 0u;
        s_last_recovery_ms = now_ms;
    }

    if (now_ms < s_next_tick_ms) {
        return;
    }
    s_next_tick_ms = now_ms + TICK_INTERVAL_MS;

    bool current_valid = true;
    float raw_current_A;
    if (INA226_ReadCurrent_A(s_hi2c, &raw_current_A)) {
        s_i2c_fault_count = 0u;
    } else {
        current_valid = false;
        if (s_i2c_fault_count < 250u) {
            s_i2c_fault_count++;
        }
        /* Reuses the last-known-good *corrected* current, same fallback
         * shape as before. */
        raw_current_A = s_current_A_last_good + s_current_offset_A;
    }
    float current_A = ApplyOffsetAndDeadband(raw_current_A);
    s_current_A_last_good = current_A;

    float bus_V = 0.0f;
    bool bus_v_valid = INA226_ReadBusVoltage_V(s_hi2c, &bus_V);
    if (bus_v_valid) {
        s_i2c_fault_count = 0u;
        s_bus_v_filt = (s_bus_v_filt > 0.0f) ? ((0.9f * s_bus_v_filt) + (0.1f * bus_V)) : bus_V;
    } else if (s_i2c_fault_count < 250u) {
        s_i2c_fault_count++;
    }

    if (!current_valid && !bus_v_valid) {
        if (s_failed_ticks < 0xFFFFu) {
            s_failed_ticks++;
        }
        if (s_failed_ticks > FAILED_TICKS_BEFORE_BACKOFF) {
            s_next_tick_ms = now_ms + BACKOFF_TICK_INTERVAL_MS;
        }
    } else {
        s_failed_ticks = 0u;
    }

    uint32_t now = HAL_GetTick();

    /* ---- Boot-time voltage read. g_hmi.battery_pct stays at 0 until this
     * completes. */
    if (!s_boot_measurement_done) {
        s_last_integration_tick_ms = now; /* keep dt sane once integration starts below */

        if (bus_v_valid) {
            s_boot_voltage_sum += (double)bus_V;
            s_boot_voltage_samples++;
        }

        if ((now - s_boot_tick) >= BOOT_VOLTAGE_SAMPLE_MS) {
            if (s_boot_voltage_samples > 0u) {
                CommitBootSeed((float)(s_boot_voltage_sum / (double)s_boot_voltage_samples));
            } else {
                /* No valid read landed in the whole window — show the
                 * neutral placeholder (not persisted) and arm the re-seed
                 * below, so a boot-time transient doesn't leave a wrong
                 * percentage for the entire session. */
                CommitState((BOOT_VOLTAGE_FALLBACK_PCT / 100.0) * NOMINAL_CAPACITY_AS);
                s_seed_is_fallback = true;
                s_boot_voltage_sum = 0.0;
                s_boot_voltage_samples = 0u;
            }
            s_boot_measurement_done = true;
        }
        return;
    }

    /* Self-healing re-seed: the boot window ended with no valid voltage
     * sample (placeholder SoC in use). As soon as bus reads start
     * succeeding — e.g. after INA226_BusRecovery() cleared a wedged bus —
     * average BOOT_VOLTAGE_SAMPLE_MS worth of them and run the same boot
     * decision the window would have. */
    if (s_seed_is_fallback && bus_v_valid) {
        if (s_boot_voltage_samples == 0u) {
            s_boot_tick = now; /* window starts at the first good sample */
        }
        s_boot_voltage_sum += (double)bus_V;
        s_boot_voltage_samples++;
        if ((now - s_boot_tick) >= BOOT_VOLTAGE_SAMPLE_MS) {
            CommitBootSeed((float)(s_boot_voltage_sum / (double)s_boot_voltage_samples));
            s_seed_is_fallback = false;
        }
    }

    double dt_s = (double)(now - s_last_integration_tick_ms) / 1000.0;
    s_last_integration_tick_ms = now;

    /* Coulomb integration: positive current = discharging (confirmed on
     * this hardware, same as the reference project). This pack is never
     * seen charging by the board (charged externally). */
    s_current_SoC_As -= ((double)current_A * dt_s);
    if (s_current_SoC_As < 0.0) {
        s_current_SoC_As = 0.0;
    }
    if (s_current_SoC_As > NOMINAL_CAPACITY_AS) {
        s_current_SoC_As = NOMINAL_CAPACITY_AS;
    }

    double raw_SoC_percent = (s_current_SoC_As / NOMINAL_CAPACITY_AS) * 100.0;
    s_displayed_SoC_percent = (raw_SoC_percent * EMA_ALPHA) + (s_displayed_SoC_percent * (1.0 - EMA_ALPHA));

    /* Persist whenever the rounded percent changes (~every 1%) — never
     * while only the fallback placeholder is in use. */
    if (!s_seed_is_fallback && (int)(s_displayed_SoC_percent + 0.5) != s_last_saved_percent) {
        SaveState();
    }

    uint8_t raw_bucket = RoundToNearestBucket(s_displayed_SoC_percent);
    if (bus_v_valid && bus_V < V_LOAD_EMPTY_THRESHOLD) {
        raw_bucket = 0u; /* hard safety floor — see its own comment above */
    }
    ApplyBucketHold(raw_bucket, now_ms);

    /* Rest detection -> current-sensor auto-zero ONLY. Still checked
     * roughly once per second (every 10th 100ms tick), same cadence as
     * before. */
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
        }
    }
}
