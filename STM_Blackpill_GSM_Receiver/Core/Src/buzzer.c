#include "buzzer.h"
#include <stdbool.h>

/* Hand-tuned against the reference project's real 12V piezo buzzer +
 * IRLZ44N MOSFET — ported verbatim, NOT recomputed for this project's
 * hardware. Index = volume_pct / 10 (0-10), value = raw TIM2 CCR1
 * (0-9999, since Period=9999). Deliberately non-linear: below ~30-40%
 * the buzzer was near-silent, and above that it saturated (sounded "at
 * max" already) if scaled linearly — these values were picked by ear on
 * the reference hardware. May need retuning once heard on this
 * project's actual buzzer — see PROGRESS.md. */
static const uint16_t VOL_STEPS[11] = {
    0,      /*   0% - mute */
    6,      /*  10% */
    8,      /*  20% */
    11,     /*  30% */
    14,     /*  40% */
    17,     /*  50% */
    25,     /*  60% */
    40,     /*  70% - buzzer not yet saturated here */
    100,    /*  80% */
    1500,   /*  90% - pre-max jump */
    9999,   /* 100% - full 12V DC, MOSFET fully on */
};

static TIM_HandleTypeDef *s_htim;
static uint16_t s_current_pwm_value; /* the volume-scaled CCR value applied
    during a beep's ON phase — matches the reference's saved_pwm_value */

static uint32_t s_beep_interval_ms;  /* 0 = silent */
static uint32_t s_next_toggle_tick;
static bool     s_beep_on;
static bool     s_continuous;        /* true: Tick() ignores the cadence
    entirely and PWM is just held steady at s_current_pwm_value */

void Buzzer_Init(TIM_HandleTypeDef *htim)
{
    s_htim = htim;
    s_current_pwm_value = 0u;
    s_beep_interval_ms = 0u;
    s_beep_on = false;
    s_continuous = false;
    HAL_TIM_PWM_Start(s_htim, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, 0u);
}

void Buzzer_SetVolume(uint8_t volume_pct)
{
    uint8_t step = (uint8_t)((volume_pct > 100u ? 100u : volume_pct) / 10u);
    s_current_pwm_value = VOL_STEPS[step];
    if (s_continuous) {
        /* Keep Volume +/- responsive even while the continuous alarm
         * tone is sounding, instead of only taking effect once it ends. */
        __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, s_current_pwm_value);
    }
}

void Buzzer_SetBeepIntervalMs(uint32_t interval_ms)
{
    if (interval_ms != s_beep_interval_ms) {
        s_beep_interval_ms = interval_ms;
        /* Restart the cadence cleanly from silent rather than carrying
         * over s_next_toggle_tick from a different interval — avoids a
         * short mismatched-length first beep right at a band change. */
        s_beep_on = false;
        if (!s_continuous) {
            __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, 0u);
        }
    }
}

void Buzzer_SetContinuousTone(bool on)
{
    if (on != s_continuous) {
        s_continuous = on;
        if (on) {
            __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, s_current_pwm_value);
        } else {
            /* Hand back to Buzzer_Tick()'s cadence starting from OFF —
             * s_next_toggle_tick is stale (it wasn't advanced while
             * continuous was active), so the very next Tick() call will
             * see now_ms >= s_next_toggle_tick and immediately start the
             * cadence rather than waiting out a leftover interval. */
            s_beep_on = false;
            __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, 0u);
        }
    }
}

void Buzzer_Tick(uint32_t now_ms)
{
    if (s_continuous || s_beep_interval_ms == 0u) {
        return;
    }

    if (now_ms >= s_next_toggle_tick) {
        s_next_toggle_tick = now_ms + s_beep_interval_ms;
        s_beep_on = !s_beep_on;
        __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, s_beep_on ? s_current_pwm_value : 0u);
    }
}
