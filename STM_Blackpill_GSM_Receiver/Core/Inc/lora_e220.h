/* Non-blocking LoRa link driver for the Ebyte E220-900T30D module — the
 * receiver's local, cellular-independent link to a transmitter.
 *
 * WHY THIS EXISTS: the AWS IoT path (gsm_mqtt.c / aws_manager.c) only works
 * with cellular coverage and a registered SIM. LoRa gives the receiver a
 * direct radio link to the transmitter that keeps working with no network at
 * all, and is the PREFERRED link whenever the selected transmitter is in
 * range; AWS/GSM is the fallback. screen_sm.c owns that handover decision,
 * reading LoRa_GetState() alongside GSM_GetState(). This driver knows about
 * frames and device numbers only, not about g_hmi or screens.
 *
 * The driver never talks any module-config protocol: M0/M1 are GPIO-
 * controlled but main.c's MX_GPIO_Init() holds both permanently LOW
 * (Normal/transparent mode) and nothing switches them at runtime or at
 * boot (a boot-time configuration-mode driver was tried and removed — it
 * was the likely cause of the module's saved settings reverting to factory
 * defaults and of intermittent total link loss; the link works reliably
 * with the module left alone in Normal mode). So the module is a pure
 * opaque broadcast UART pipe, and this driver's job is entirely software:
 * byte-wise interrupt-driven framing, parsing a lightweight device-tagged
 * ASCII payload, filtering by the currently selected device number, and
 * tracking a rolling-window frame-reception success rate used as the
 * handover-stability signal. (No RSSI is read.) It also separately logs
 * per-device presence for every frame decoded (see LoRa_IsDeviceOnline()),
 * regardless of which device is selected — used by the pairing screens'
 * real "who's online" check.
 *
 * Wire format: `*D<NN>:<cm>\r\n`, e.g. `*D07:12345\r\n` for transmitter
 * TX-07 at 123.45m — centimetres, matching the "distance_cm" unit of the
 * AWS telemetry path so both links feed g_hmi.distance_cm identically.
 * The transmitter's lora_tx.c broadcasts this exact format (its NN comes
 * from DEVICE_ID, "TX-NN").
 */
#ifndef LORA_E220_H
#define LORA_E220_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef enum {
    LORA_LINK_IDLE = 0,      /* not listening for any device */
    LORA_LINK_LISTENING,     /* listening, rolling window hasn't (yet, or
                                 not currently) shown a high enough
                                 success rate to be trusted as the active
                                 telemetry source */
    LORA_LINK_STABLE,        /* rolling window success rate is high
                                 enough to use as the active telemetry
                                 source in place of the AWS/GSM link */
} lora_link_state_t;

/* One-time setup: binds the UART, arms interrupt-driven RX. Does not
 * start listening for any device. Call once at boot. */
void LoRa_Init(UART_HandleTypeDef *huart);

/* Starts (or restarts, e.g. after a device change) listening for
 * device_num's broadcast frames (1-45, matches the D-01..D-45 numbering
 * used by the pairing screens). Resets the rolling window. Call alongside
 * GSM_BeginConnect() — both links race, whichever proves ready first wins. */
void LoRa_BeginListen(uint8_t device_num);

/* User-initiated stop (device change / end shunting) — stops filtering,
 * back to LORA_LINK_IDLE. Call alongside GSM_Disconnect(). */
void LoRa_StopListen(void);

/* Call every main-loop iteration, unconditionally — same non-blocking
 * tick-function convention as BatterySoc_Tick()/ScreenSM_Tick()/etc.
 * Never blocks (no HAL_Delay, no wait loops). NOTE: it can only advance
 * while the main loop is actually iterating, so a long blocking call
 * elsewhere (an AWS_Init() attempt, say) pauses the rolling window — see
 * main.c for how the AWS attempts are throttled around an active session. */
void LoRa_Poll(uint32_t now_ms);

lora_link_state_t LoRa_GetState(void);

/* True and writes *out_cm if a telemetry reading for the currently
 * listened-to device has arrived since the last call. False otherwise;
 * *out_cm left untouched. Same "unread since last call" semantics as the
 * old GSM distance getter. */
bool LoRa_GetLatestDistance(uint16_t *out_cm);

/* Current rolling-window success count, 0-5 (enter LORA_LINK_STABLE at
 * >=4, fall back to LORA_LINK_LISTENING at <=2, dead zone at 3). This is
 * the signal-quality metric this driver actually uses. screen_sm.c maps
 * this count onto conn_health_t. */
uint8_t LoRa_GetWindowSuccessCount(void);

/* True if a validly-decoded frame carrying device_num (1-45) has been
 * heard within the last 5 seconds — regardless of whether that device is
 * the currently-selected one. Unlike the rolling window above (which
 * only ever tracks one selected device's signal quality), this runs for
 * every device all the time, even while LORA_LINK_IDLE — it's what lets
 * the pairing screens show real presence for every device before any one
 * of them has been chosen. */
bool LoRa_IsDeviceOnline(uint8_t device_num, uint32_t now_ms);

/* Only the shared HAL_UART_RxCpltCallback dispatcher in main.c should
 * call this — only one weak-override HAL_UART_RxCpltCallback can exist
 * program-wide. No-ops if huart isn't the one LoRa_Init() was given. */
void LoRa_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/* Only the shared HAL_UART_ErrorCallback in main.c should call this. HAL
 * stops the interrupt-driven receive after any UART error (overrun,
 * framing, noise) and never restarts it, which would silently kill the
 * LoRa link until reboot — this clears the error flags and re-arms RX.
 * No-ops if huart isn't the one LoRa_Init() was given. */
void LoRa_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif /* LORA_E220_H */
