#include "dwin_hmi.h"
#include "hmi_map.h"
#include <string.h>

/* ---- DGUS wire protocol constants ------------------------------------ */
#define DGUS_HDR1           0x5Au
#define DGUS_HDR2           0xA5u
#define DGUS_CMD_WRITE_VP    0x82u
#define DGUS_CMD_READ_VP     0x83u /* also the format of the display's
                                       auto-uploaded touch-code frames */

#define DWIN_TX_TIMEOUT_MS   20u   /* frames are <20 bytes @115200 baud;
                                       this is a fast, deterministic UART
                                       send, not the multi-second GSM
                                       blocking case CLAUDE.md warns about */

#define RX_FRAME_MAX_BYTES   64u   /* generous headroom over any frame this
                                       app actually sends/receives */
#define TOUCH_QUEUE_LEN       8u

/* Real messages here are a handful of bytes (page-jump payload, one VP
 * word, or a short text field); this caps the largest one we'll ever
 * build. Keeping it small matters because SendFrame() and its callers
 * each hold a stack buffer of this size, and this project's linker script
 * (STM32F411XX_FLASH.ld) only reserves _Min_Stack_Size = 0x400 (1KB) total
 * — a 255-byte buffer at two stack frames deep would burn half of that. */
#define DWIN_TX_MAX_DATA_BYTES 64u

static UART_HandleTypeDef *s_huart;

/* Guards against a real concurrency hazard added 2026-08-30: charger_detect.c
 * now calls DWIN_SwitchPage() directly from its TIM3 interrupt (see its own
 * comment), so SendFrame() can be entered from both the main loop and an
 * ISR now. HAL_UART_Transmit()'s own busy-check (huart->gState) has a
 * narrow, unprotected read-then-write window — if the ISR preempted the
 * main loop at exactly that instant, both contexts could see the
 * peripheral as "ready" and write to the UART data register concurrently,
 * corrupting the transmission. This flag is claimed inside a tiny
 * (nanoseconds — just the flag check-and-set, not the transmit itself)
 * IRQ-disabled critical section, so that race can't happen; the loser
 * silently drops its frame rather than corrupting the winner's. Safe for
 * DWIN's own RX interrupt too — the critical section is far shorter than
 * one UART byte-time at 115200 baud (~87us), so it can't cause a missed
 * incoming byte. */
static volatile bool s_tx_busy;

/* ---- byte-wise RX frame assembler ------------------------------------ */
typedef enum {
    RX_WAIT_H1 = 0,
    RX_WAIT_H2,
    RX_WAIT_LEN,
    RX_WAIT_DATA
} rx_state_t;

static volatile rx_state_t s_rx_state = RX_WAIT_H1;
static uint8_t  s_rx_byte;
static uint8_t  s_rx_frame[RX_FRAME_MAX_BYTES];
static uint8_t  s_rx_len;
static uint8_t  s_rx_idx;

/* single-producer (ISR) / single-consumer (main loop) ring buffer */
static volatile uint16_t s_touch_queue[TOUCH_QUEUE_LEN];
static volatile uint8_t  s_touch_head;
static volatile uint8_t  s_touch_tail;

static void RxQueuePush(uint16_t code)
{
    uint8_t next = (uint8_t)((s_touch_head + 1u) % TOUCH_QUEUE_LEN);
    if (next == s_touch_tail) {
        return; /* queue full: drop oldest-incoming rather than corrupt state */
    }
    s_touch_queue[s_touch_head] = code;
    s_touch_head = next;
}

static void HandleCompleteFrame(void)
{
    /* s_rx_frame[0] = CMD, followed by (s_rx_len - 1) data bytes */
    if (s_rx_len < 4u) {
        return;
    }
    uint8_t cmd = s_rx_frame[0];
    if (cmd != DGUS_CMD_READ_VP) {
        return;
    }
    uint16_t vp = (uint16_t)((s_rx_frame[1] << 8) | s_rx_frame[2]);
    uint8_t word_count = s_rx_frame[3];
    if (vp != VP_TOUCH_CODE || word_count < 1u || s_rx_len < 6u) {
        return;
    }
    uint16_t code = (uint16_t)((s_rx_frame[4] << 8) | s_rx_frame[5]);
    RxQueuePush(code);
}

static void RxFeedByte(uint8_t b)
{
    switch (s_rx_state) {
        case RX_WAIT_H1:
            if (b == DGUS_HDR1) {
                s_rx_state = RX_WAIT_H2;
            }
            break;

        case RX_WAIT_H2:
            if (b == DGUS_HDR2) {
                s_rx_state = RX_WAIT_LEN;
            } else if (b != DGUS_HDR1) {
                s_rx_state = RX_WAIT_H1;
            }
            break;

        case RX_WAIT_LEN:
            if (b == 0u || b > RX_FRAME_MAX_BYTES) {
                s_rx_state = RX_WAIT_H1; /* corrupt/oversized: resync */
            } else {
                s_rx_len = b;
                s_rx_idx = 0u;
                s_rx_state = RX_WAIT_DATA;
            }
            break;

        case RX_WAIT_DATA:
            s_rx_frame[s_rx_idx++] = b;
            if (s_rx_idx >= s_rx_len) {
                HandleCompleteFrame();
                s_rx_state = RX_WAIT_H1;
            }
            break;
    }
}

void DWIN_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == s_huart) {
        RxFeedByte(s_rx_byte);
        HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1u);
    }
}

/* ---- public API -------------------------------------------------------*/

void DWIN_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_rx_state = RX_WAIT_H1;
    s_touch_head = 0u;
    s_touch_tail = 0u;
    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1u);
}

static void SendFrame(uint8_t cmd, uint16_t vp_addr, const uint8_t *data, uint8_t data_len)
{
    uint8_t frame[6 + DWIN_TX_MAX_DATA_BYTES];
    if (data_len > DWIN_TX_MAX_DATA_BYTES) {
        data_len = DWIN_TX_MAX_DATA_BYTES;
    }
    uint8_t len = (uint8_t)(1u + 2u + data_len); /* cmd + addr + data */
    uint8_t idx = 0;

    frame[idx++] = DGUS_HDR1;
    frame[idx++] = DGUS_HDR2;
    frame[idx++] = len;
    frame[idx++] = cmd;
    frame[idx++] = (uint8_t)(vp_addr >> 8);
    frame[idx++] = (uint8_t)(vp_addr & 0xFFu);
    if (data_len > 0u) {
        memcpy(&frame[idx], data, data_len);
        idx = (uint8_t)(idx + data_len);
    }

    /* See s_tx_busy's own comment above: tiny critical section around
     * just the claim, not the transmit itself. */
    __disable_irq();
    bool already_busy = s_tx_busy;
    if (!already_busy) {
        s_tx_busy = true;
    }
    __enable_irq();

    if (already_busy) {
        /* Lost the race — whichever context already owns the UART wins;
         * this frame is silently dropped rather than corrupting the
         * winner's transmission. Acceptable: the only current caller
         * this can actually happen with is charger_detect.c's ISR-side
         * page-switch nudge, which is explicitly redundant with the
         * main loop's own TickChargerOverlay() sending the same command
         * shortly after — see charger_detect.c. */
        return;
    }

    HAL_UART_Transmit(s_huart, frame, idx, DWIN_TX_TIMEOUT_MS);
    s_tx_busy = false;
}

void DWIN_SwitchPage(uint16_t page_id)
{
    /* Standard DGUS II page-jump payload: 5A 01 00 <page_id_low> written
     * to VP_SYS_PIC_ID via the normal write-VP command. */
    uint8_t data[4] = { 0x5Au, 0x01u, 0x00u, (uint8_t)(page_id & 0xFFu) };
    SendFrame(DGUS_CMD_WRITE_VP, VP_SYS_PIC_ID, data, sizeof(data));
}

void DWIN_WriteVP16(uint16_t vp_addr, uint16_t value)
{
    uint8_t data[2] = { (uint8_t)(value >> 8), (uint8_t)(value & 0xFFu) };
    SendFrame(DGUS_CMD_WRITE_VP, vp_addr, data, sizeof(data));
}

void DWIN_WriteVPWords(uint16_t vp_addr, const uint16_t *words, uint8_t count)
{
    uint8_t data[DWIN_TX_MAX_DATA_BYTES];
    if (count > DWIN_TX_MAX_DATA_BYTES / 2u) {
        count = DWIN_TX_MAX_DATA_BYTES / 2u;
    }
    uint8_t len = (uint8_t)(count * 2u);
    for (uint8_t i = 0; i < count; i++) {
        data[i * 2]     = (uint8_t)(words[i] >> 8);
        data[i * 2 + 1] = (uint8_t)(words[i] & 0xFFu);
    }
    SendFrame(DGUS_CMD_WRITE_VP, vp_addr, data, len);
}

void DWIN_WriteVPString(uint16_t vp_addr, const char *str, uint8_t field_bytes)
{
    uint8_t data[DWIN_TX_MAX_DATA_BYTES];
    uint8_t padded = (uint8_t)((field_bytes + 1u) & ~1u); /* round up to even */
    if (padded > sizeof(data)) {
        padded = sizeof(data) & ~1u;
    }
    memset(data, 0, padded);
    size_t src_len = strlen(str);
    if (src_len > padded) {
        src_len = padded;
    }
    memcpy(data, str, src_len);
    SendFrame(DGUS_CMD_WRITE_VP, vp_addr, data, padded);
}

bool DWIN_PopTouchCode(uint16_t *code)
{
    if (s_touch_tail == s_touch_head) {
        return false;
    }
    *code = s_touch_queue[s_touch_tail];
    s_touch_tail = (uint8_t)((s_touch_tail + 1u) % TOUCH_QUEUE_LEN);
    return true;
}
