#include "device_sm.h"
#include "logger.h"
#include "device_config.h"
#include "aws_manager.h"
#include "gsm_mqtt.h"

static DeviceState_t currentState = DEV_STATE_BOOT;
static uint32_t stateEntryTime = 0;
// static uint32_t recoveryDelay = 30000; // start with 30s backoff

void DeviceSM_Init(UART_HandleTypeDef *huart)
{
    currentState = DEV_STATE_BOOT;
    stateEntryTime = HAL_GetTick();
    LOG_I("SM", "State Machine Initialized");
}

DeviceState_t DeviceSM_GetState(void)
{
    return currentState;
}

const char* DeviceSM_GetStateString(void)
{
    switch(currentState) {
        case DEV_STATE_BOOT: return "BOOT";
        case DEV_STATE_MODEM_STARTING: return "MODEM_STARTING";
        case DEV_STATE_MODEM_READY: return "MODEM_READY";
        case DEV_STATE_SIM_CHECK: return "SIM_CHECK";
        case DEV_STATE_NETWORK_SEARCH: return "NETWORK_SEARCH";
        case DEV_STATE_NETWORK_READY: return "NETWORK_READY";
        case DEV_STATE_DATA_SETUP: return "DATA_SETUP";
        case DEV_STATE_DATA_READY: return "DATA_READY";
        case DEV_STATE_TLS_SETUP: return "TLS_SETUP";
        case DEV_STATE_TLS_READY: return "TLS_READY";
        case DEV_STATE_MQTT_CONNECTING: return "MQTT_CONNECTING";
        case DEV_STATE_MQTT_CONNECTED: return "MQTT_CONNECTED";
        case DEV_STATE_PROV_CHECK: return "PROV_CHECK";
        case DEV_STATE_PROV_SUBSCRIBE: return "PROV_SUBSCRIBE";
        case DEV_STATE_PROV_REQUEST_CERT: return "PROV_REQUEST_CERT";
        case DEV_STATE_PROV_WAIT_CERT: return "PROV_WAIT_CERT";
        case DEV_STATE_PROV_REGISTER: return "PROV_REGISTER";
        case DEV_STATE_PROV_WAIT_THING: return "PROV_WAIT_THING";
        case DEV_STATE_PROV_SAVE: return "PROV_SAVE";
        case DEV_STATE_PROV_RECONNECT: return "PROV_RECONNECT";
        case DEV_STATE_SUBSCRIBE: return "SUBSCRIBE";
        case DEV_STATE_REGISTER_DEVICE: return "REGISTER_DEVICE";
        case DEV_STATE_ONLINE: return "ONLINE";
        case DEV_STATE_RECOVERY: return "RECOVERY";
        case DEV_STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

void DeviceSM_Tick(void)
{
    // Minimal stub implementation so that the new file exists as requested, 
    // without depending on the non-blocking methods that were removed from gsm_mqtt.h during revert
}
