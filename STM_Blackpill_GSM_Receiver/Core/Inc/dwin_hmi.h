/* Low-level DWIN T5L DGUS II driver: frame TX and interrupt-driven frame RX.
 * This module knows nothing about screens or app state — it only speaks
 * the DGUS wire protocol on top of a HAL UART handle. screen_sm.c is the
 * main caller. As of 2026-08-30, charger_detect.c also calls
 * DWIN_SwitchPage() directly from its TIM3 interrupt, for an instant
 * plug-in screen switch that can't wait for the main loop (see
 * charger_detect.c and PROGRESS.md) — TX (SendFrame(), used by every
 * Write-style/SwitchPage function below) is safe to call from an ISR
 * concurrently with the main loop; see its own mutual-exclusion comment
 * in dwin_hmi.c. RX/DWIN_PopTouchCode() remain main-loop-only, unchanged.
 */
#ifndef DWIN_HMI_H
#define DWIN_HMI_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/* Bind the driver to the UART peripheral wired to the DWIN display and
 * arm the first byte-wise receive interrupt. Call once from main() after
 * MX_USARTx_UART_Init(). */
void DWIN_Init(UART_HandleTypeDef *huart);

/* Command the display to jump to a page (writes VP_SYS_PIC_ID). Do not
 * call this for touch codes that DGUS already hardware-jumped on its own
 * (see CLAUDE.md "Screen transitions already handled by DGUS hardware"). */
void DWIN_SwitchPage(uint16_t page_id);

/* Write a single 16-bit value to one VP word. */
void DWIN_WriteVP16(uint16_t vp_addr, uint16_t value);

/* Write `count` 16-bit words starting at vp_addr. */
void DWIN_WriteVPWords(uint16_t vp_addr, const uint16_t *words, uint8_t count);

/* Write an ASCII string into a text VP field, zero-padded/truncated to
 * exactly field_bytes bytes so leftover characters from a previous, longer
 * string are cleared. field_bytes is rounded up to an even number of
 * bytes internally (DGUS VPs are 16-bit words). */
void DWIN_WriteVPString(uint16_t vp_addr, const char *str, uint8_t field_bytes);

/* Pop the oldest queued touch code reported by the display. Returns true
 * and writes *code if one was available, false if the queue is empty.
 * Call repeatedly from the main loop (not from an ISR). */
bool DWIN_PopTouchCode(uint16_t *code);

/* Only the shared HAL_UART_RxCpltCallback dispatcher in main.c should
 * call this (there can be only one weak-override definition in the whole
 * program, so each UART-owning driver exposes a named handler instead of
 * defining HAL_UART_RxCpltCallback itself). No-ops if huart isn't the
 * one DWIN_Init() was given. */
void DWIN_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* DWIN_HMI_H */
