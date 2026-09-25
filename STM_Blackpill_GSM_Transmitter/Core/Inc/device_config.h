#ifndef __DEVICE_CONFIG_H
#define __DEVICE_CONFIG_H

/* ===== DEVICE TYPE (choose ONE) ===== */
// #define DEVICE_TYPE_RECEIVER     1
#define DEVICE_TYPE_TRANSMITTER  1
#define DEVICE_TYPE_STR          "TRANSMITTER 45 MTR"

/* ===== DEVICE IDENTITY ===== */
#define DEVICE_ID              "TX-02"
#define DEVICE_SERIAL_NUMBER   "SN-TX-02"
#define DEVICE_NAME            "Shunting Transmitter Test"
#define DEVICE_HW_VERSION      "1.0"
#define DEVICE_FW_VERSION      "2.0.0"
#define DEVICE_MFG_DATE        "2026-09-01"

/* ===== NETWORK ===== */
#define DEVICE_APN             "airtelgprs.com"

/* ===== LOGGING ===== */
#define LOG_LEVEL              LOG_LEVEL_INFO
// Options: LOG_LEVEL_DEBUG, LOG_LEVEL_INFO, LOG_LEVEL_WARN, LOG_LEVEL_ERROR

/* ===== AWS CONFIGURATION ===== */
#define AWS_BROKER_URL         "tcp://a18ey7y7yi6gdo-ats.iot.ap-south-1.amazonaws.com:8883"
#define AWS_TEMPLATE_NAME      "ShuntingFleetTemplate"
#define MQTT_TOPIC_PREFIX      "devices"

/* ===== TIMEOUTS (ms) ===== */
#define TIMEOUT_MODEM_BOOT_MS       40000
#define TIMEOUT_SIM_READY_MS        10000
#define TIMEOUT_NETWORK_REG_MS      60000
#define TIMEOUT_APN_DATA_MS         15000
#define TIMEOUT_MQTT_CONNECT_MS     30000
#define TIMEOUT_PROVISIONING_MS     15000
#define TIMEOUT_OVERALL_PROV_MS     300000
#define TIMEOUT_FLASH_SAVE_MS       5000

/* ===== TELEMETRY ===== */
#define TELEMETRY_INTERVAL_MS  10000
#define STATUS_HEARTBEAT_INTERVAL_MS 20000

#endif /* __DEVICE_CONFIG_H */
