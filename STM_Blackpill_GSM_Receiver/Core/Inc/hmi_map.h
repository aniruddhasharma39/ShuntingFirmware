/* DWIN DGUS II memory map and touch-code map for the receiver HMI.
 * Ported from Shunting_Receiver_v2's hmi_map.h — trimmed to the GSM-only,
 * no-battery/no-charger/no-buzzer/no-LiDAR scope of this project. Do not
 * invent new addresses here; these are hardware-verified/spec values.
 */
#ifndef HMI_MAP_H
#define HMI_MAP_H

#include <stdint.h>

/* ---- VP addresses (host <-> display data) -------------------------- */
#define VP_DISTANCE            0x2100u /* distance readout, meters */
#define VP_BATTERY_PCT          0x2000u /* battery percentage */
#define VP_DEVICE_NAME          0x2020u /* connected device name (status bar) */
#define VP_CONN_HEALTH           0x2040u /* 0=Poor 1=Good 2=Excellent */
#define VP_CONN_MODE             0x2060u /* status-bar link-mode text field;
                                              this port is GSM-only, so the
                                              literal string "GSM" is always
                                              what gets written here (see
                                              screen_sm.c's PushStatusBar()) */
#define VP_VOLUME_PCT            0x2080u

/* Pairing-slot VP addresses are NOT a uniform base+step sequence — a
 * hardware sweep on 2026-08-03 (writing each candidate address's own hex
 * value as a label and reading off the display) found each page has one
 * anomalous 0x80 gap (4 steps instead of the usual 0x20) at a different
 * position per page, breaking any simple formula. The actual per-slot
 * addresses are an explicit lookup table in screen_sm.c (PAIRING_PAGES). */
#define VP_PAIRING_SLOTS_PER_PAGE 9u

#define VP_CONFIRM_DEVICE_NAME  0x3100u /* 10_Confirm_Selection device name text */
#define VP_CHANGE_DEVICE_NAME   0x3120u /* 09_Change_Device "Current Device:" text */
#define VP_CHARGE_PCT_TEXT      0x3140u /* 13_Charger_Plugged charging % text */

/* System VP used by the display firmware itself to jump pages (PIC_ID).
 * Written with the same "write VP" command as any other VP above. */
#define VP_SYS_PIC_ID           0x0084u

/* Touch-code report VP: DGUS "Return Key" controls auto-upload their code
 * word here; the STM32 only ever reads it, never writes it. */
#define VP_TOUCH_CODE            0x1000u

/* ---- Touch codes reported on VP_TOUCH_CODE -------------------------- */
#define TOUCH_DEVICE_SELECT_MIN  0x0001u /* device D-01 */
#define TOUCH_DEVICE_SELECT_MAX  0x002Du /* device D-45 */

#define TOUCH_CHANGE_DEVICE_YES   0x00C1u /* 09_Change_Device: YES -> (hw) 01_Pairing_P1 */
#define TOUCH_CHANGE_DEVICE_NO    0x00C2u /* 09_Change_Device: NO  -> (hw) 04_Telemetry */
#define TOUCH_CONFIRM_SEL_YES     0x00C3u /* 10_Confirm_Selection: YES -> (hw) 02_Connecting */
#define TOUCH_CONFIRM_SEL_NO      0x00C4u /* 10_Confirm_Selection: NO  -> (hw) 01_Pairing_P1 */
#define TOUCH_END_SHUNTING_YES    0x00C5u /* 11_End_Shunting: YES -> (hw) 12_Shunting_Completed */
#define TOUCH_END_SHUNTING_NO     0x00C6u /* 11_End_Shunting: NO  -> (hw) 04_Telemetry */

#define TOUCH_VOLUME_DOWN         0x00D0u
#define TOUCH_VOLUME_UP           0x00D1u
#define TOUCH_REFRESH             0x00A1u /* pairing pages only, no page change */
#define TOUCH_BACK                0x00EEu /* 04_Telemetry only -> (hw) 09_Change_Device */
#define TOUCH_END_SHUNTING_BTN    0x00EFu /* 04_Telemetry only -> (hw) 11_End_Shunting */

/* ---- DGUS page IDs (PIC_ID) ------------------------------------------
 * ASSUMPTION (unverified): page IDs equal the numeric prefix of the BMP
 * filenames in UI_Images (00..14), matching the order they were designed
 * in. This is the natural default when a DWIN Designer project is built
 * one page at a time from a numbered background set, but it must be
 * checked against the actual project's page list before flashing — a
 * wrong page ID here sends the display to the wrong screen entirely.
 *
 * Pages 00-14 exist in this port. Whether page 13 (Charger Plugged) and
 * page 14 (Obstacle Warning) are actually built on THIS board's own
 * physical display has not been separately confirmed for page 14 (page
 * 13 already confirmed working — see PROGRESS.md's charger-port entry)
 * — see PROGRESS.md's obstacle-warning entry. */
typedef enum {
    PAGE_00_STARTUP            = 0,
    PAGE_01_PAIRING_P1         = 1,
    PAGE_02_CONNECTING         = 2,
    PAGE_03_ERROR               = 3,
    PAGE_04_TELEMETRY           = 4,
    PAGE_05_PAIRING_P2          = 5,
    PAGE_06_PAIRING_P3          = 6,
    PAGE_07_PAIRING_P4          = 7,
    PAGE_08_PAIRING_P5          = 8,
    PAGE_09_CHANGE_DEVICE       = 9,
    PAGE_10_CONFIRM_SELECTION   = 10,
    PAGE_11_END_SHUNTING        = 11,
    PAGE_12_SHUNTING_COMPLETED  = 12,
    PAGE_13_CHARGER_PLUGGED     = 13,
    PAGE_14_OBSTACLE_WARNING    = 14
} dgus_page_id_t;

#endif /* HMI_MAP_H */
