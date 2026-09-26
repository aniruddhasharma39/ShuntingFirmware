#include "lora_e220.h"
#include <stdlib.h>
#include <string.h>

/* ---- rolling-window handover-stability tuning ---
 * LORA_SLOT_INTERVAL_MS is deliberately LONGER than the transmitter's own
 * send cadence (lora_tx.c sends about once per second). With the two
 * boards' free-running clocks unsynchronized, a slot exactly as long as the
 * TX interval can occasionally catch a perfectly good transmission on the
 * wrong side of a slot boundary and miscount it as a "miss" with nothing
 * actually wrong on-air — real-hardware symptom was link health flapping
 * between Good and Best even at close range with a clean link. At 1500ms
 * (50% margin over the ~1000ms cadence) a slot mathematically cannot end up
 * with zero transmission opportunities, so it only registers a genuine miss
 * when the actual over-the-air reception failed. Trade-off: the 5-slot
 * window spans 7.5s (fall back after ~3 consecutive misses ~ 4.5s,
 * stabilize after ~4 consecutive hits ~ 6s) — a modest, deliberate cost in
 * handover reaction time; the AWS/GSM link stays available as fallback
 * throughout via screen_sm.c's ConnectionIsReady(). */
#define LORA_WINDOW_SIZE             5u   /* slots */
#define LORA_SLOT_INTERVAL_MS     1000u   /* matches lora_tx.c's ~1000ms send cadence */
#define LORA_ENTER_SUCCESS_COUNT     3u   /* >=60% successful -> promote to STABLE */
#define LORA_EXIT_SUCCESS_COUNT      1u   /* <=20% successful -> LoRa demote to LISTENING */
#define LORA_WINDOW_MASK         0x1Fu   /* low 5 bits of s_window_bits */

#define LORA_RX_BUF_SIZE   64u   /* one frame ("*D07:12345\r\n" is ~12 bytes) with headroom */

/* ---- presence tracking, for the pairing screens' "who's online" check.
 * 15000ms matches GSM presence timeout, giving plenty of margin during
 * blocking modem operations. */
#define LORA_MAX_DEVICES            45u
#define LORA_PRESENCE_TIMEOUT_MS 15000u

static UART_HandleTypeDef *s_huart;

static lora_link_state_t s_state = LORA_LINK_IDLE;
static uint8_t  s_target_device;         /* 1-45, only meaningful outside IDLE */

static uint16_t s_latest_distance_cm;
static bool     s_distance_ready;

/* Per-device presence, updated for EVERY validly-decoded frame regardless
 * of s_target_device — a completely separate concern from the rolling
 * window below (which only ever tracks the one currently-selected
 * device's signal quality for handover purposes). s_ever_seen guards
 * against a device that's never actually broadcast anything reading as
 * "online" just because now_ms - 0 happens to be small at boot. */
static uint32_t s_last_seen_tick[LORA_MAX_DEVICES];
static bool     s_ever_seen[LORA_MAX_DEVICES];

/* ---- rolling window state -----------------------------------------------
 * A slot counts as a "hit" if >=1 valid frame from s_target_device arrived
 * during it — evaluated once per LORA_SLOT_INTERVAL_MS tick, not once per
 * frame, so a slot with 3 frames and a slot with 1 frame count identically
 * and a burst in one slot can't paper over a silent later slot. */
static uint16_t s_window_bits;      /* low LORA_WINDOW_SIZE bits used */
static uint8_t  s_slots_recorded;   /* caps at LORA_WINDOW_SIZE; cold-start guard */
static bool     s_slot_hit;
static uint32_t s_next_slot_tick;
static uint8_t  s_window_success_count;

/* ---- interrupt-fed RX accumulation with double buffering so
 * newly incoming bytes do not corrupt an assembled frame awaiting LoRa_Poll. ---- */
static volatile bool s_frame_ready;
static uint16_t s_rx_len;
static char     s_rx_buf[LORA_RX_BUF_SIZE];
static char     s_completed_buf[LORA_RX_BUF_SIZE];
static uint8_t  s_rx_byte;

void LoRa_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == s_huart) {
        if (s_rx_byte == '*') {
            s_rx_len = 0u;
        }
        if (s_rx_len < (LORA_RX_BUF_SIZE - 1u)) {
            s_rx_buf[s_rx_len++] = (char)s_rx_byte;
            s_rx_buf[s_rx_len] = '\0';
        }
        if (s_rx_byte == '\n' || s_rx_len >= (LORA_RX_BUF_SIZE - 1u)) {
            memcpy(s_completed_buf, s_rx_buf, s_rx_len + 1u);
            s_frame_ready = true;
            s_rx_len = 0u;
        }
        HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1u);
    }
}

void LoRa_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == s_huart) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        s_rx_len = 0u; /* whatever was mid-assembly is suspect — resync on the next '*' */
        s_rx_buf[0] = '\0';
        HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1u);
    }
}

/* ---- frame parsing (main-context, called from LoRa_Poll) --------------- */

static void ParseFrame(const char *frame, uint32_t now)
{
    /* Wire format: "*D<NN>:<cm>" — see lora_e220.h's header comment. */
    const char *p = strchr(frame, '*');
    if (!p || p[1] != 'D') {
        return; /* not our frame shape — ignore malformed/unrelated payloads */
    }
    p += 2;

    char *end = NULL;
    long dev = strtol(p, &end, 10);
    if (end == p || dev < 1 || dev > (long)LORA_MAX_DEVICES || *end != ':') {
        return;
    }
    end++;

    char *end2 = NULL;
    long dist = strtol(end, &end2, 10);
    if (end2 == end || dist < 0 || dist > 65535) {
        return;
    }

    /* Presence: logged for whichever device this frame actually carries,
     * regardless of which one (if any) is currently selected — this is
     * what lets the pairing screens see every broadcasting transmitter,
     * not just the one being actively listened to. */
    s_last_seen_tick[dev - 1] = now;
    s_ever_seen[dev - 1] = true;

    if ((uint8_t)dev != s_target_device) {
        /* Another transmitter's broadcast, not the one (if any) actively
         * selected — not an error. This link is unaddressed at the radio
         * level (E220 fixed-point addressing isn't used), so every
         * receiver hears every transmitter; filtering for the
         * point-to-point telemetry/handover purpose below is entirely
         * client-side. */
        return;
    }

    s_latest_distance_cm = (uint16_t)dist;
    s_distance_ready = true;
    s_slot_hit = true; /* counts toward this slot's rolling-window success */
}

/* ---- public API ----------------------------------------------------------*/

void LoRa_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_rx_len = 0u;
    s_rx_buf[0] = '\0';
    s_completed_buf[0] = '\0';
    s_frame_ready = false;
    s_state = LORA_LINK_IDLE;
    s_distance_ready = false;
    memset(s_ever_seen, 0, sizeof(s_ever_seen));
    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1u);
}

void LoRa_BeginListen(uint8_t device_num)
{
    s_target_device = device_num;
    s_window_bits = 0u;
    s_slots_recorded = 0u;
    s_slot_hit = false;
    s_window_success_count = 0u;
    s_next_slot_tick = HAL_GetTick() + LORA_SLOT_INTERVAL_MS;
    s_distance_ready = false;
    s_state = LORA_LINK_LISTENING;
}

void LoRa_StopListen(void)
{
    s_state = LORA_LINK_IDLE;
}

lora_link_state_t LoRa_GetState(void)
{
    return s_state;
}

bool LoRa_GetLatestDistance(uint16_t *out_cm)
{
    if (!s_distance_ready) {
        return false;
    }
    *out_cm = s_latest_distance_cm;
    s_distance_ready = false;
    return true;
}

uint8_t LoRa_GetWindowSuccessCount(void)
{
    return s_window_success_count;
}

bool LoRa_IsDeviceOnline(uint8_t device_num, uint32_t now_ms)
{
    if (device_num < 1u || device_num > LORA_MAX_DEVICES) {
        return false;
    }
    uint8_t i = device_num - 1u;
    if (!s_ever_seen[i]) {
        return false;
    }
    return (now_ms - s_last_seen_tick[i]) <= LORA_PRESENCE_TIMEOUT_MS;
}

/* ---- the tick function ----------------------------------------------------
 * Two independent jobs each call: drain any frame the ISR has finished
 * assembling (fast, main-context parsing), and advance the rolling
 * window once per LORA_SLOT_INTERVAL_MS. Frame draining runs
 * unconditionally, IDLE or not — presence-logging (inside ParseFrame)
 * specifically needs to keep working before any device is selected,
 * since that's exactly when the pairing screens need it. Only the
 * rolling-window advance stays IDLE-gated, since there's no selected
 * device's stability to track yet. No HAL_Delay or wait loop anywhere. */
void LoRa_Poll(uint32_t now_ms)
{
    if (s_frame_ready) {
        s_frame_ready = false;
        ParseFrame(s_completed_buf, now_ms);
    }

    if (s_state == LORA_LINK_IDLE) {
        return;
    }

    if (now_ms >= s_next_slot_tick) {
        s_next_slot_tick = now_ms + LORA_SLOT_INTERVAL_MS;

        s_window_bits = (uint16_t)(((s_window_bits << 1) | (s_slot_hit ? 1u : 0u)) & LORA_WINDOW_MASK);
        s_slot_hit = false;
        if (s_slots_recorded < LORA_WINDOW_SIZE) {
            s_slots_recorded++;
        }

        uint8_t count = 0u;
        for (uint16_t bits = s_window_bits; bits != 0u; bits >>= 1) {
            count += (uint8_t)(bits & 1u);
        }
        s_window_success_count = count;

        if (s_state == LORA_LINK_LISTENING &&
            s_slots_recorded >= LORA_WINDOW_SIZE &&
            s_window_success_count >= LORA_ENTER_SUCCESS_COUNT) {
            s_state = LORA_LINK_STABLE;
        } else if (s_state == LORA_LINK_STABLE &&
                   s_window_success_count <= LORA_EXIT_SUCCESS_COUNT) {
            s_state = LORA_LINK_LISTENING;
        }
        /* else: dead zone (3/5) or cold-start guard not yet satisfied —
         * stay in whichever state is already active. */
    }
}
