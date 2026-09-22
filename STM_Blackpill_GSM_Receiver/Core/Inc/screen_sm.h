/* The 13-screen HMI state machine (GSM+DWIN only — no LoRa, no battery/
 * charger, no buzzer, no LiDAR-obstacle overlay), ported from
 * Shunting_Receiver_v2's 14-screen version. Owns page navigation and
 * drives the DWIN display via dwin_hmi.h based on timers and touch
 * codes, reading/writing the shared g_hmi struct.
 */
#ifndef SCREEN_SM_H
#define SCREEN_SM_H

#include <stdint.h>

/* Call once after DWIN_Init()/HmiState_Init(). Sends the initial page
 * switch to 00_Startup and starts its 3-second hold timer. */
void ScreenSM_Init(void);

/* Call every main-loop iteration. Drains queued touch codes, advances
 * timers/auto-transitions, and pushes any VP updates the current screen
 * needs (status bar, live distance, charger overlay, etc). Never blocks. */
void ScreenSM_Tick(uint32_t now_ms);

/* Forces navigation back to 00_Startup with a genuine fresh 3-second hold
 * and cleared device-selection/presence state — used by the
 * charger-unplug "fresh start" path (see TickChargerOverlay() in
 * screen_sm.c). Deliberately NOT the same as calling ScreenSM_Init()
 * again mid-session: that hardcodes now=0, which would make the startup
 * hold check see itself as already elapsed and skip straight to
 * 01_Pairing_P1. Does not touch g_hmi.volume_pct — nothing about a
 * charger fresh-start should reset the locopilot's buzzer volume. */
void ScreenSM_ForceStartupReset(uint32_t now_ms);

#endif /* SCREEN_SM_H */
