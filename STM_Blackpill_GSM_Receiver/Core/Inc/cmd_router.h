#ifndef __CMD_ROUTER_H
#define __CMD_ROUTER_H

#ifdef __cplusplus
extern "C" {
#endif

void CmdRouter_Init(void);
void CmdRouter_Process(const char *topic, const char *payload);

#ifdef __cplusplus
}
#endif

#endif /* __CMD_ROUTER_H */
