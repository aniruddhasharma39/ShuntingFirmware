#include "message.h"
#include "device_config.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

static uint32_t s_msgSeq = 0;

int Message_Build(char *buf, size_t buf_size, MessageType_t type, const char *data_json)
{
    const char *type_str = "UNKNOWN";
    switch (type) {
        case MSG_TYPE_TELEMETRY:   type_str = "TELEMETRY"; break;
        case MSG_TYPE_STATUS:      type_str = "STATUS"; break;
        case MSG_TYPE_REGISTRATION:type_str = "REGISTRATION"; break;
        case MSG_TYPE_HEARTBEAT:   type_str = "HEARTBEAT"; break;
        case MSG_TYPE_EVENT:       type_str = "EVENT"; break;
        case MSG_TYPE_ACK:         type_str = "ACK"; break;
    }

    uint32_t uptime = HAL_GetTick() / 1000;
    s_msgSeq++;

    return snprintf(buf, buf_size,
        "{"
        "\"msgId\":%lu,"
        "\"uptimeSec\":%lu,"
        "\"device\":{"
        "\"deviceId\":\"%s\","
#if defined(DEVICE_TYPE_TRANSMITTER)
        "\"deviceType\":\"TRANSMITTER\","
#else
        "\"deviceType\":\"RECEIVER\","
#endif
        "\"fwVersion\":\"%s\""
        "},"
        "\"msgType\":\"%s\","
        "\"seq\":%lu,"
        "\"data\":%s"
        "}",
        s_msgSeq,
        uptime,
        DEVICE_ID,
        DEVICE_FW_VERSION,
        type_str,
        s_msgSeq,
        data_json ? data_json : "{}"
    );
}
