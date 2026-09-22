#ifndef __MESSAGE_H
#define __MESSAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

typedef enum {
    MSG_TYPE_TELEMETRY,
    MSG_TYPE_STATUS,
    MSG_TYPE_REGISTRATION,
    MSG_TYPE_HEARTBEAT,
    MSG_TYPE_EVENT,
    MSG_TYPE_ACK
} MessageType_t;

/* Build a JSON message with the standard envelope */
int Message_Build(
    char *buf, size_t buf_size,
    MessageType_t type,
    const char *data_json
);

#ifdef __cplusplus
}
#endif

#endif /* __MESSAGE_H */
