#ifndef __AWS_MANAGER_H
#define __AWS_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * ============================================================
 * AWS IoT Manager
 * ============================================================
 *
 * Handles:
 *   - Device identity
 *   - AWS IoT Fleet Provisioning
 *   - Claim certificate
 *   - Device certificate/key storage
 *   - AWS MQTT registration
 *   - Telemetry publishing
 *   - Command/config subscriptions
 *
 * ============================================================
 */


/* ------------------------------------------------------------
 * AWS INITIALIZATION
 * ------------------------------------------------------------ */

/*
 * Initialize AWS IoT.
 *
 * This function:
 *   1. Loads existing device certificate from STM32 flash
 *   2. If not found, performs Fleet Provisioning
 *   3. Configures modem TLS
 *   4. Connects to AWS IoT
 *
 * Returns:
 *   1 = success
 *   0 = failure
 */
uint8_t AWS_Init(UART_HandleTypeDef *huart, const char *apn);


/* ------------------------------------------------------------
 * AWS PERIODIC MANAGER
 * ------------------------------------------------------------ */

/*
 * Call continuously from main loop.
 *
 * Handles periodic telemetry publishing.
 */
void AWS_Manager_Tick(uint32_t now_ms);


/* ------------------------------------------------------------
 * DEVICE REGISTRATION
 * ------------------------------------------------------------ */

/*
 * Publish device registration/status information.
 */
uint8_t AWS_RegisterDevice(void);


/* ------------------------------------------------------------
 * TELEMETRY & OFFLINE QUEUE
 * ------------------------------------------------------------ */

typedef struct {
    uint32_t uptime_s;
    uint16_t distance_cm;
    uint8_t  selected_target_id;
    uint8_t  battery_pct;
    bool     is_charging;
    int8_t   gsm_rssi;
    char     link_state[16];
    bool     is_lora;
} OfflineTelemetry_t;

bool AWS_OfflineQueue_Push(const OfflineTelemetry_t *rec);
bool AWS_OfflineQueue_Pop(OfflineTelemetry_t *rec);
uint16_t AWS_OfflineQueue_Count(void);

/*
 * Publish device telemetry to AWS IoT with explicit isLoRa and uptime_s.
 */
uint8_t AWS_PublishTelemetryEx(
    uint16_t distance_cm,
    uint8_t selected_target_id,
    uint8_t battery_pct,
    bool is_charging,
    int8_t gsm_rssi,
    const char *link_state,
    bool is_lora,
    uint32_t uptime_s
);

/*
 * Publish device telemetry to AWS IoT.
 */
uint8_t AWS_PublishTelemetry(
    uint16_t distance_cm,
    uint8_t selected_target_id,
    uint8_t battery_pct,
    bool is_charging,
    int8_t gsm_rssi,
    const char *link_state
);


/* ------------------------------------------------------------
 * DEVICE INFORMATION
 * ------------------------------------------------------------ */

const char *AWS_GetDeviceId(void);

const char *AWS_GetSerialNumber(void);

const char *AWS_GetTopicTelemetry(void);

const char *AWS_GetTopicStatus(void);

const char *AWS_GetTopicCommands(void);

const char *AWS_GetTopicConfig(void);


/* ------------------------------------------------------------
 * AWS STATE
 * ------------------------------------------------------------ */

bool AWS_IsProvisioned(void);

bool AWS_IsRegistered(void);


/* ------------------------------------------------------------
 * CERTIFICATE STORAGE
 * ------------------------------------------------------------ */

/*
 * Check whether a valid provisioned certificate/key
 * is stored in STM32 internal flash.
 */
bool AWS_CertStorage_Exists(void);


/*
 * Load provisioned certificate and private key.
 */
bool AWS_CertStorage_Load(
    char *cert_pem,
    uint16_t max_cert_len,
    char *key_pem,
    uint16_t max_key_len
);


/*
 * Save provisioned certificate and private key.
 */
bool AWS_CertStorage_Save(
    const char *cert_pem,
    const char *key_pem
);


/*
 * Erase stored AWS credentials.
 */
void AWS_CertStorage_Erase(void);


#ifdef __cplusplus
}
#endif

#endif /* __AWS_MANAGER_H */
