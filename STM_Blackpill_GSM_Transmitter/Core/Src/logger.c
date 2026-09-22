#include "logger.h"
#include "device_config.h"
#include <stdio.h>
#include <stdarg.h>

extern void dbg_raw(const char *fmt, ...); // Assuming usb_console has dbg_raw or similar

void log_print(int level, const char *tag, const char *fmt, ...)
{
#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

    if (level < LOG_LEVEL) {
        return;
    }

    const char *level_str = "U";
    switch (level) {
        case LOG_LEVEL_DEBUG: level_str = "D"; break;
        case LOG_LEVEL_INFO:  level_str = "I"; break;
        case LOG_LEVEL_WARN:  level_str = "W"; break;
        case LOG_LEVEL_ERROR: level_str = "E"; break;
    }

    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // Call existing dbg/printf mechanism
    printf("[%s] [%s] %s\r\n", level_str, tag, buf);
}
