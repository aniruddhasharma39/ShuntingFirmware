#include "hmi_state.h"
#include <string.h>

hmi_state_t g_hmi;

void HmiState_Init(void)
{
    memset(&g_hmi, 0, sizeof(g_hmi));
    g_hmi.active_screen = SCR_00_STARTUP;
    g_hmi.battery_pct = 80u; /* transient fallback only — BatterySoc_Init()
                                 overwrites this with 0 immediately, then
                                 with a real boot-time reading a few
                                 seconds into the main loop (see
                                 battery_soc.c); never actually shown,
                                 since PushStatusBar() doesn't push
                                 VP_BATTERY_PCT while on SCR_00_STARTUP,
                                 which holds longer than that read takes */
    /* conn_health/conn_mode intentionally left at their zeroed defaults
     * (POOR / GSM) — screen_sm.c's PushStatusBar() only displays them
     * once a link is actually established, showing "--" until then, so
     * there's no real value in seeding them with specific enums here. */
    g_hmi.volume_pct = 50u;
    g_hmi.distance_cm = 50000u;
    strcpy(g_hmi.connected_device_name, "--");
    strcpy(g_hmi.selected_device_name, "--");
}
