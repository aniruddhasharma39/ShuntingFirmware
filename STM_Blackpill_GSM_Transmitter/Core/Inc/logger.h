#ifndef __LOGGER_H
#define __LOGGER_H

#ifdef __cplusplus
extern "C" {
#endif

#define LOG_LEVEL_DEBUG   0
#define LOG_LEVEL_INFO    1
#define LOG_LEVEL_WARN    2
#define LOG_LEVEL_ERROR   3

void log_print(int level, const char *tag, const char *fmt, ...);

#define LOG_D(tag, fmt, ...)  log_print(LOG_LEVEL_DEBUG, tag, fmt, ##__VA_ARGS__)
#define LOG_I(tag, fmt, ...)  log_print(LOG_LEVEL_INFO,  tag, fmt, ##__VA_ARGS__)
#define LOG_W(tag, fmt, ...)  log_print(LOG_LEVEL_WARN,  tag, fmt, ##__VA_ARGS__)
#define LOG_E(tag, fmt, ...)  log_print(LOG_LEVEL_ERROR, tag, fmt, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* __LOGGER_H */
