#ifndef __DEVICE_SM_H
#define __DEVICE_SM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

typedef enum {
    DEV_STATE_BOOT,
    DEV_STATE_MODEM_STARTING,
    DEV_STATE_MODEM_READY,
    DEV_STATE_SIM_CHECK,
    DEV_STATE_NETWORK_SEARCH,
    DEV_STATE_NETWORK_READY,
    DEV_STATE_DATA_SETUP,
    DEV_STATE_DATA_READY,
    DEV_STATE_TLS_SETUP,
    DEV_STATE_TLS_READY,
    DEV_STATE_MQTT_CONNECTING,
    DEV_STATE_MQTT_CONNECTED,
    DEV_STATE_PROV_CHECK,
    DEV_STATE_PROV_SUBSCRIBE,
    DEV_STATE_PROV_REQUEST_CERT,
    DEV_STATE_PROV_WAIT_CERT,
    DEV_STATE_PROV_REGISTER,
    DEV_STATE_PROV_WAIT_THING,
    DEV_STATE_PROV_SAVE,
    DEV_STATE_PROV_RECONNECT,
    DEV_STATE_SUBSCRIBE,
    DEV_STATE_REGISTER_DEVICE,
    DEV_STATE_ONLINE,
    DEV_STATE_RECOVERY,
    DEV_STATE_ERROR
} DeviceState_t;

void DeviceSM_Init(UART_HandleTypeDef *huart);
void DeviceSM_Tick(void);
DeviceState_t DeviceSM_GetState(void);
const char* DeviceSM_GetStateString(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEVICE_SM_H */
