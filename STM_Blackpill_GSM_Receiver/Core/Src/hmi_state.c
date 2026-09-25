#include "hmi_state.h"
#include <string.h>

hmi_state_t g_hmi;

void HmiState_Init(void)
{
    memset(&g_hmi, 0, sizeof(g_hmi));
    g_hmi.active_screen = SCR_00_STARTUP;
    g_hmi.battery_pct = 80u; /* transient fallback only — BatterySoc_Init()
                                 overwrites this with a real INA226-derived
                                 reading within milliseconds, before the
                                 main loop starts */
    /* conn_health intentionally left at its zeroed default —
     * screen_sm.c's PushStatusBar() only displays it once GSM_GetState()
     * is actually GSM_LINK_CONNECTED, showing "--" until then, so there's
     * no real value in seeding it with a specific enum here. */
    g_hmi.volume_pct = 50u;
    g_hmi.distance_cm = 50000u;
    strcpy(g_hmi.connected_device_name, "--");
    strcpy(g_hmi.selected_device_name, "--");
}
