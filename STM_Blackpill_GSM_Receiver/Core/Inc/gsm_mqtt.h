#ifndef GSM_MQTT_H
#define GSM_MQTT_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * GSM / MQTT LINK STATE
 * ============================================================ */

typedef enum
{
    GSM_LINK_DISCONNECTED = 0,
    GSM_LINK_SCANNING,
    GSM_LINK_CONNECTED
} gsm_link_state_t;


/* ============================================================
 * MQTT CONFIGURATION
 * ============================================================ */

typedef struct
{
    const char *client_id;
    const char *pub_topic;
    const char *sub_topic;
    const char *broker_url;
    const char *apn;

    /* AWS / TLS */
    uint8_t use_ssl;
    uint8_t ssl_ctx_index;

} GSM_MQTT_Config;


/* ============================================================
 * INCOMING MQTT CALLBACK
 * ============================================================ */

typedef void (*GSM_MQTT_IncomingCallback)(
    const char *topic,
    const char *payload
);


/* ============================================================
 * INITIALIZATION
 * ============================================================ */

uint8_t GSM_MQTT_Init(
    UART_HandleTypeDef *huartAT,
    const GSM_MQTT_Config *cfg
);

void GSM_SetUART(UART_HandleTypeDef *huartAT);

void dbg(const char *fmt, ...);
/* ============================================================
 * MQTT PUBLISH
 * ============================================================ */

/* Publish using configured pub_topic */
uint8_t GSM_MQTT_Publish(
    const char *message
);


/* Publish to an explicitly supplied topic */
uint8_t GSM_MQTT_PublishTopic(
    const char *topic,
    const char *message
);


/* ============================================================
 * MQTT SUBSCRIBE
 * ============================================================ */

uint8_t GSM_MQTT_SubscribeTopic(
    const char *topic
);


/* ============================================================
 * INCOMING MESSAGE CALLBACK
 * ============================================================ */

void GSM_SetIncomingCallback(
    GSM_MQTT_IncomingCallback callback
);


/* ============================================================
 * AT COMMAND ACCESS
 * ============================================================ */

/*
 * Send an AT command through the modem UART.
 *
 * Mainly used by AWS manager for:
 *   - certificate upload
 *   - TLS configuration
 *   - modem configuration
 */
uint8_t GSM_SendAT(
    const char *cmd,
    uint32_t timeout
);


/*
 * Send raw data after modem has issued a '>' prompt.
 */
uint8_t GSM_SendRawData(
    const char *data,
    uint16_t len,
    uint32_t timeout
);


/* ============================================================
 * POLLING
 * ============================================================ */

/*
 * Call continuously from main loop.
 *
 * Handles:
 *   - incoming MQTT messages
 *   - CMQTT connection loss
 *   - automatic modem/network/MQTT recovery
 */
void GSM_MQTT_Poll(void);


/* ============================================================
 * STATUS
 * ============================================================ */

uint8_t GSM_MQTT_IsConnected(void);

gsm_link_state_t GSM_GetState(void);

/* ============================================================
 * LEGACY STUBS FOR SCREEN_SM
 * ============================================================ */
void GSM_BeginConnect(uint8_t selected_device_num);
void GSM_BeginScanning(void);
void GSM_Disconnect(void);
bool GSM_GetLatestDistance(uint16_t *distance_out);
uint32_t GSM_GetMsSinceLastMessage(uint32_t now);
bool GSM_IsDeviceOnline(uint8_t device_num, uint32_t now);

void GSM_Reset(void);
void GSM_MarkDeviceOnline(uint8_t device_num);
void GSM_SetLatestDistance(uint8_t device_num, uint16_t distance_cm);


#ifdef __cplusplus
}
#endif

#endif /* GSM_MQTT_H */


