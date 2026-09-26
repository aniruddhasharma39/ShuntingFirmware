/* Cheap "is it even worth attempting AWS_Init() right now?" pre-check for
 * the A7670C modem, built ONLY on gsm_mqtt.h's public API (GSM_SetUART() and
 * GSM_SendAT()). gsm_mqtt.c / aws_manager.c are protected and must not be
 * edited, so this lives outside them.
 *
 * WHY IT EXISTS: AWS_Init() blocks for as long as its timeouts allow
 * (modem boot up to 40s, network registration up to 60s, provisioning up to
 * 5 minutes...). With no SIM inserted, or with the modem still booting, an
 * attempt can never succeed but still freezes the whole main loop for those
 * long stretches — on the receiver that stalls the LoRa link tracking, touch
 * handling and the buzzer; on the transmitter it stalls the lidar reads.
 * A missing SIM is detectable in well under two seconds with two ordinary
 * AT commands, so main.c asks this gate first and only spends a full AWS_Init()
 * attempt when it has a realistic chance of working. The gate never sends
 * anything that changes modem state (plain "AT" and "AT+CPIN?").
 *
 * Same idea as the "fail fast when there is no SIM" fix that was applied to
 * the standalone (non-AWS) GSM driver's fullReconnect(), done here in the
 * caller instead because fullReconnect() itself is off-limits.
 */
#ifndef MODEM_GATE_H
#define MODEM_GATE_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"

/* Returns true only if the modem answers "AT" and "AT+CPIN?" both come back
 * OK (i.e. the modem is up and a SIM is present). Blocks for at most about
 * 2 seconds in the worst case (modem silent), and ~100-300ms when the modem
 * is up. Safe to call repeatedly; assigns huart_modem as the driver's UART
 * exactly as AWS_Init() itself will. */
bool ModemGate_SimReady(UART_HandleTypeDef *huart_modem);

#endif /* MODEM_GATE_H */
