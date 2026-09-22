#include "cmd_router.h"
#include "logger.h"
#include "aws_manager.h"
#include "device_config.h"
#include "stm32f4xx_hal.h"
#include <string.h>

void CmdRouter_Init(void)
{
    // Nothing for now
}

static const char *find_json_val_end(const char *str)
{
    while (*str) {
        if (*str == '\\') { str += 2; continue; }
        if (*str == '"') return str;
        str++;
    }
    return NULL;
}

void CmdRouter_Process(const char *topic, const char *payload)
{
    if (!topic || !payload) return;

    if (strstr(topic, "/commands")) {
        LOG_I("CMD", "Received command on topic: %s", topic);
        
        // Simple JSON parsing for {"command": "reboot"}
        const char *cmd_start = strstr(payload, "\"command\":\"");
        if (cmd_start) {
            cmd_start += 11;
            const char *cmd_end = find_json_val_end(cmd_start);
            if (cmd_end) {
                size_t len = cmd_end - cmd_start;
                if (strncmp(cmd_start, "reboot", len) == 0 && len == 6) {
                    LOG_W("CMD", "Executing reboot command");
                    HAL_NVIC_SystemReset();
                }
                else if (strncmp(cmd_start, "eraseCredentials", len) == 0 && len == 16) {
                    LOG_W("CMD", "Erasing credentials and rebooting");
                    AWS_CertStorage_Erase();
                    HAL_NVIC_SystemReset();
                }
                else if (strncmp(cmd_start, "identify", len) == 0 && len == 8) {
                    LOG_I("CMD", "Executing identify command");
                    // Assuming AWS_RegisterDevice is still available or we can just log
                    // AWS_RegisterDevice(); // Publishes to status topic
                }
                else if (strncmp(cmd_start, "setTelemetryInterval", len) == 0 && len == 20) {
                    LOG_I("CMD", "Executing setTelemetryInterval command (not persisted)");
                }
                else {
                    LOG_W("CMD", "Unknown command received");
                }
            }
        }
    }
}
