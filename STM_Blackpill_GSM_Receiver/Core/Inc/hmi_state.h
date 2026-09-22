/* Single shared state struct for the receiver HMI. Ported from
 * Shunting_Receiver_v2's hmi_state.h, trimmed to this project's scope (no
 * LoRa). GSM+DWIN+buzzer fields are owned by screen_sm.c; battery_soc.c
 * owns the real battery field; charger_detect.c owns the charger fields
 * — same ownership split as the source project. Nothing here talks to
 * hardware directly.
 */
#ifndef HMI_STATE_H
#define HMI_STATE_H

#include <stdint.h>
#include <stdbool.h>

#define HMI_MAX_DEVICES        45u
#define HMI_DEVICE_NAME_LEN     8u /* "D-45" + slack */

typedef enum {
    SCR_00_STARTUP = 0,
    SCR_01_PAIRING_P1,
    SCR_02_CONNECTING,
    SCR_03_ERROR,
    SCR_04_TELEMETRY,
    SCR_05_PAIRING_P2,
    SCR_06_PAIRING_P3,
    SCR_07_PAIRING_P4,
    SCR_08_PAIRING_P5,
    SCR_09_CHANGE_DEVICE,
    SCR_10_CONFIRM_SELECTION,
    SCR_11_END_SHUNTING,
    SCR_12_SHUNTING_COMPLETED,
    SCR_13_CHARGER_PLUGGED,
    SCR_14_OBSTACLE_WARNING,
    SCR_COUNT
} screen_id_t;

typedef enum {
    CONN_HEALTH_POOR = 0,
    CONN_HEALTH_GOOD = 1,
    CONN_HEALTH_EXCELLENT = 2
} conn_health_t;

typedef struct {
    /* -- navigation (owned by screen_sm.c) -- */
    screen_id_t active_screen;
    uint32_t    screen_entered_tick;
    uint32_t    connecting_start_tick;
    bool        charger_overlay_active;  /* true while page 13 is covering active_screen */
    bool        obstacle_overlay_active; /* true while page 14 is covering active_screen
                                             (active_screen itself stays SCR_04_TELEMETRY
                                             throughout, same overlay technique as charger) */

    /* -- pairing / device selection (owned by screen_sm.c) -- */
    uint8_t     selected_device_num;                    /* 1-45, 0 = none */
    char        selected_device_name[HMI_DEVICE_NAME_LEN];
    uint8_t     connected_device_num;                   /* 0 = not connected */
    char        connected_device_name[HMI_DEVICE_NAME_LEN];
    bool        device_online[HMI_MAX_DEVICES];          /* real presence,
                                                              from GSM_IsDeviceOnline() */

    /* -- battery, real (owned by battery_soc.c, INA226-derived) -- */
    uint8_t        battery_pct;

    /* -- GSM link telemetry, real (owned by screen_sm.c, sourced from
     * gsm_mqtt.c's GSM_GetLatestDistance()/GetState()) -- */
    uint16_t       distance_m;
    conn_health_t  conn_health;

    /* -- volume, real (owned by screen_sm.c, set directly by touch input) -- */
    uint8_t        volume_pct;

    /* -- charger, real (owned by charger_detect.c, dual-channel ADC
     * sensing) -- */
    bool    charger_plugged;    /* charger-present flag, ~1.0V threshold */
    bool    charger_near_full;  /* near-full flag, ~2.0V threshold */
    uint8_t charge_pct;         /* mirrors battery_pct, capped at 99 until
                                    charger_near_full is true */
} hmi_state_t;

extern hmi_state_t g_hmi;

void HmiState_Init(void);

#endif /* HMI_STATE_H */
