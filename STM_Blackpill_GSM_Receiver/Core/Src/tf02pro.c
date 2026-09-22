/* ============================================================
 * tf02pro.c
 * Place in Core/Src.
 * ============================================================ */
#include "tf02pro.h"

static UART_HandleTypeDef *lidarUart;
static uint8_t             rxByte;
static uint8_t             frame[9];
static uint8_t             frameIdx = 0;

static volatile uint16_t latestDistance = 0;
static volatile uint16_t latestStrength = 0;
static volatile uint8_t  dataReady = 0;

void TF02_Init(UART_HandleTypeDef *huart) {
    lidarUart = huart;
    frameIdx = 0;
    HAL_UART_Receive_IT(lidarUart, &rxByte, 1);
}

void TF02_RxCpltHandler(UART_HandleTypeDef *huart) {
    if (huart->Instance != lidarUart->Instance) return;

    if (frameIdx == 0) {
        if (rxByte == 0x59) frame[frameIdx++] = rxByte;
        // else: not a header byte, stay at 0 and keep looking
    } else if (frameIdx == 1) {
        if (rxByte == 0x59) {
            frame[frameIdx++] = rxByte;
        } else {
            frameIdx = 0;  // false start, resync
        }
    } else {
        frame[frameIdx++] = rxByte;
        if (frameIdx >= 9) {
            uint8_t sum = 0;
            for (int i = 0; i < 8; i++) sum += frame[i];
            if (sum == frame[8]) {
                latestDistance = (uint16_t)(frame[2] | (frame[3] << 8));
                latestStrength = (uint16_t)(frame[4] | (frame[5] << 8));
                dataReady = 1;
            }
            // if checksum failed, we just drop this frame silently and
            // resync on the next 0x59 0x59 - no need to error out, the
            // sensor sends another frame in a few milliseconds anyway.
            frameIdx = 0;
        }
    }

    // Re-arm for the next byte no matter what happened above.
    HAL_UART_Receive_IT(lidarUart, &rxByte, 1);
}

uint8_t TF02_GetLatest(uint16_t *distance_cm, uint16_t *strength) {
    if (!dataReady) return 0;
    *distance_cm = latestDistance;
    if (strength) *strength = latestStrength;
    dataReady = 0;
    return 1;
}
