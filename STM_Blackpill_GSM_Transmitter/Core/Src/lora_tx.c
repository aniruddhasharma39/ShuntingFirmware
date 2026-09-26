#include "lora_tx.h"
#include "tf02pro.h"
#include "device_config.h"

/* If a previous frame is still marked in-flight this many broadcasts in a row
 * (seconds, at the 1Hz call rate), the UART/DMA transfer is presumed stuck
 * (e.g. a missed completion interrupt) and is aborted so broadcasting
 * resumes on the next tick. A healthy ~12-byte frame at 115200 baud takes
 * about 1ms, so this never triggers in normal operation. */
#define LORA_TX_BUSY_ABORT_TICKS   3u

static UART_HandleTypeDef *s_huart;

/* Device number carried in every frame — the "NN" of DEVICE_ID ("TX-NN").
 * 0 means DEVICE_ID could not be parsed; the receiver only accepts 1-45, so
 * frames tagged 0 are ignored there, which is safer than broadcasting under
 * a wrong identity. */
static uint8_t s_device_num;

/* Must be static, NOT a stack-local inside LoRaTx_Tick(): DMA reads this
 * buffer AFTER HAL_UART_Transmit_DMA() has already returned — the actual
 * byte clocking happens in hardware, asynchronously, while the rest of the
 * program (including whatever called LoRaTx_Tick()) keeps running. A stack
 * buffer would be invalid — quite possibly already overwritten by
 * something else's stack frame — by the time DMA gets around to reading
 * it. This buffer is only ever touched from LoRaTx_Tick(), and that's only
 * ever called from one place (TIM2's HAL_TIM_PeriodElapsedCallback(),
 * main.c), so there's no concurrent-access concern either — and it is only
 * rewritten once the previous transfer has finished (see LoRaTx_Tick()). */
static char s_buf[32];

/* "TX-NN" -> NN, or 0 if it isn't exactly that shape. */
static uint8_t ParseDeviceNum(const char *id)
{
    if (id[0] != 'T' || id[1] != 'X' || id[2] != '-') { return 0; }
    if (id[3] < '0' || id[3] > '9' || id[4] < '0' || id[4] > '9') { return 0; }
    if (id[5] != '\0') { return 0; }
    return (uint8_t)((id[3] - '0') * 10 + (id[4] - '0'));
}

/* Writes v (0-65535) into dst as plain decimal, no padding, no terminator;
 * returns the number of digits written (1-5). Hand-rolled on purpose: this
 * runs inside a timer interrupt, and formatting through the C library there
 * would pull its stack use and re-entrancy questions into the ISR for no
 * benefit. */
static uint8_t PutUInt(char *dst, uint32_t v)
{
    char tmp[5];
    uint8_t n = 0u;
    do {
        tmp[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v != 0u && n < (uint8_t)sizeof(tmp));
    for (uint8_t i = 0u; i < n; i++) {
        dst[i] = tmp[n - 1u - i];
    }
    return n;
}

void LoRaTx_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_device_num = ParseDeviceNum(DEVICE_ID);
}

void LoRaTx_Tick(void)
{
    static uint8_t busy_ticks;

    if (s_huart == NULL) {
        return;
    }

    /* Previous frame still in flight: don't touch the buffer DMA is reading.
     * Normally never happens (see LORA_TX_BUSY_ABORT_TICKS). */
    if (s_huart->gState != HAL_UART_STATE_READY) {
        if (++busy_ticks >= LORA_TX_BUSY_ABORT_TICKS) {
            busy_ticks = 0u;
            HAL_UART_AbortTransmit(s_huart);
        }
        return;
    }
    busy_ticks = 0u;

    /* Deliberately NOT gated on a "new reading available" check — this
     * broadcast frame IS the receiver's only LoRa presence signal for this
     * device. Must keep broadcasting continuously regardless of
     * connection/sensor state; a missing LiDAR reading should mean "0cm
     * until real data arrives," not "go silent and look offline."
     * TF02_GetDistance_cm() already defaults to 0 before the first valid
     * frame, so this falls through safely with no separate placeholder
     * needed. */
    uint16_t cm = TF02_GetDistance_cm();

    /* "*D<NN>:<cm>\r\n" — at most 12 bytes. */
    uint8_t n = 0u;
    s_buf[n++] = '*';
    s_buf[n++] = 'D';
    s_buf[n++] = (char)('0' + ((s_device_num / 10u) % 10u));
    s_buf[n++] = (char)('0' + (s_device_num % 10u));
    s_buf[n++] = ':';
    n = (uint8_t)(n + PutUInt(&s_buf[n], cm));
    s_buf[n++] = '\r';
    s_buf[n++] = '\n';

    /* Non-blocking: hands the frame to DMA and returns immediately — see
     * lora_tx.h's header comment for why. Return value deliberately ignored:
     * the state check above already covers the only realistic failure
     * (HAL_BUSY); anything else just means one skipped broadcast, retried
     * next second. */
    HAL_UART_Transmit_DMA(s_huart, (uint8_t *)s_buf, n);
}
