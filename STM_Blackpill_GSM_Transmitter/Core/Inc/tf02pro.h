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

#endif /* TF02PRO_H */
