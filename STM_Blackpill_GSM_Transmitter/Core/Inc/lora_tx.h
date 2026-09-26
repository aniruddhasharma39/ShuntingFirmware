/* LoRa broadcast for the Ebyte E220-900T30D module, wired Normal/
 * transparent mode (M0/M1 hardwired to GND — no config-mode access, pure
 * opaque UART broadcast pipe, no firmware GPIO involvement). No
 * addressing at the radio level — every receiver in range hears every
 * transmitter's broadcasts; the device number embedded in the payload is
 * how the receiver's software tells transmitters apart.
 *
 * WHY THIS EXISTS: the AWS IoT path (aws_manager.c / gsm_mqtt.c) only works
 * with cellular coverage. This broadcast is the cellular-independent link the
 * receiver prefers whenever this transmitter is in range; AWS stays the
 * fallback. It is entirely additive — nothing in the AWS layer knows it
 * exists.
 *
 * Wire format: `*D<NN>:<cm>\r\n`, e.g. `*D02:12345\r\n` for TX-02 at 123.45m.
 * <NN> is parsed from DEVICE_ID ("TX-NN", device_config.h) once at
 * LoRaTx_Init(), so the same firmware image differs between units only by
 * that one define. The distance is sent in centimetres — the TF02-Pro's
 * native unit, and the same "distance_cm" unit the AWS telemetry carries —
 * so both links feed the receiver's g_hmi.distance_cm identically. Matches
 * STM_Blackpill_GSM_Receiver's lora_e220.c contract.
 *
 * TX-only: this device never listens on this UART, only the receiver
 * does.
 *
 * TIMER-DRIVEN, NOT POLLED. GSM's driver (gsm_mqtt.c) and AWS_Init() are
 * deliberately blocking and share the main loop with everything else —
 * AWS_Init() alone can hold it for minutes at boot with no SIM/coverage. A
 * broadcast paced from that loop would go silent (and the receiver would
 * see this transmitter as offline) exactly when the cellular side struggles.
 * So the send is paced by a dedicated hardware timer interrupt instead
 * (TIM2, 1Hz), and the interrupt itself stays short by handing the frame to
 * DMA rather than blocking on the UART.
 *
 * main.c starts TIM2 with HAL_TIM_Base_Start_IT() and calls LoRaTx_Tick()
 * directly from HAL_TIM_PeriodElapsedCallback(). Nothing else should call
 * LoRaTx_Tick(). */
#ifndef LORA_TX_H
#define LORA_TX_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* One-time setup: binds the UART and derives the device number from
 * DEVICE_ID. Call once at boot, before HAL_TIM_Base_Start_IT(&htim2) — see
 * main.c. */
void LoRaTx_Init(UART_HandleTypeDef *huart);

/* Call only from TIM2's HAL_TIM_PeriodElapsedCallback() (main.c) — never
 * from the main loop or anywhere else. Sends one frame via
 * HAL_UART_Transmit_DMA() (non-blocking — kicks off the transfer and
 * returns immediately, byte clocking happens in hardware afterward) and
 * returns. Broadcasts even before the first valid LiDAR reading arrives
 * (0cm placeholder) — this frame is the receiver's only LoRa presence
 * signal for this device, so it must never go silent regardless of
 * sensor state. */
void LoRaTx_Tick(void);

#endif /* LORA_TX_H */
