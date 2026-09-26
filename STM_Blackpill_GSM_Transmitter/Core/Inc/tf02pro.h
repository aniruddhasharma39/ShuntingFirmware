/* ============================================================
 * tf02pro.h
 * Benewake TF02-Pro LiDAR driver - continuous frame reception.
 * The sensor streams 9-byte frames on its own (100Hz default,
 * 115200 baud) with no need to request readings.
 * Place in Core/Inc.
 * ============================================================ */
#ifndef TF02PRO_H
#define TF02PRO_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Call once at startup - starts continuous interrupt-driven reception. */
void TF02_Init(UART_HandleTypeDef *huart);

/* Call this from HAL_UART_RxCpltCallback() in main.c when huart->Instance
 * matches the one you passed to TF02_Init - see usage instructions. */
void TF02_RxCpltHandler(UART_HandleTypeDef *huart);

/* Call from your main loop as often as you like (it's just reading
 * variables, no blocking). Returns 1 if a NEW valid reading is available
 * since the last call, 0 if nothing new yet. distance_cm and strength are
 * only written when this returns 1. */
uint8_t TF02_GetLatest(uint16_t *distance_cm, uint16_t *strength);

/* Returns the most recently parsed distance unconditionally (0 before
 * the first valid frame), independent of TF02_GetLatest()'s own "new
 * since last call" flag — added for lora_tx.c, which needs the current
 * distance on every ~1Hz broadcast tick regardless of whether the
 * AWS telemetry path in main.c has already consumed the latest reading
 * via TF02_GetLatest(). Reading this never affects TF02_GetLatest()'s own
 * state, so the AWS path is unaffected. */
uint16_t TF02_GetDistance_cm(void);

#endif /* TF02PRO_H */
