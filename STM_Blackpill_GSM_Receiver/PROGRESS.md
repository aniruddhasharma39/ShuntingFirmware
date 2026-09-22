# Progress Log — STM_Blackpill_GSM_Receiver

This file tracks what has actually been built, tested on real hardware, and
fixed in this project — including root causes of real bugs, not just
"fixed a bug." Read this before re-diagnosing something that's already
been solved here. Same convention as `Shunting_Receiver_v2/PROGRESS.md`.
Updated after each milestone or resolved bug.

**Build note:** this is a pure STM32CubeIDE managed-build project (no
CMakeLists.txt) — it cannot be compiled from this chat session. All builds
and flashes are done by the user directly in STM32CubeIDE; every change
recorded here has been statically verified (brace balance, matching
declarations/definitions, no stray references) before being handed over,
then confirmed working on real hardware afterward.

## Why this project exists

While chasing GSM reliability issues on `Shunting_Receiver_v2` (its
`gsm_mqtt.c` is a from-scratch, interrupt-driven, non-blocking **rewrite**
of the original driver, suspected as a real contributor to that
instability), the user independently hand-tuned this project — a much
simpler reference build with the original, proven **blocking/polled** GSM
driver, no LoRa, no display — and confirmed it held a GSM connection with
**zero disconnects over 30+ minutes**. Decision: stop fighting the complex
rewrite and instead build the real DWIN display UI on top of this
proven-simple driver, extending it additively rather than touching its
internals.

## Architecture

- **USART1** — DWIN display (`huart1`, PA9/PA10), interrupt-driven RX,
  blocking TX. Added via CubeMX on top of the original reference project.
- **USART2** — GSM/A7670C module (`huart2`), fully blocking/polled —
  unchanged from the original reference driver.
- **Broker** — self-hosted Mosquitto at `tcp://168.144.68.31:1883`
  (DigitalOcean droplet, replaces the old public `broker.hivemq.com` test
  broker used earlier in the wider project).
- **Config** (`main.c`): `client_id="STM32_Dev2"`, `apn="airtelgprs.com"`,
  `pub_topic="shunting/query"` (presence-scan requests),
  `sub_topic="shunting/presence"` (presence responses — baseline
  subscription at boot). A specific device's own distance topic
  (`shunting/deadend/<NN>/distance`) is subscribed on top of this later,
  on demand, via `GSM_BeginConnect()` once the operator picks a device on
  the pairing screen.
- **`gsm_mqtt.c`/`.h`** — the original, proven blocking driver
  (`sendAT()`/`sendRawData()`/`waitForURC()` helpers, `respBuf[600]`,
  `mqttConnected`) is left completely untouched except for one 2-line hook
  inside `printIncoming()` (dispatches parsed topic/payload into the new
  layer instead of discarding them). Everything else — presence tracking,
  per-device connect/disconnect, link-state query, message-recency
  tracking — is new code appended after the original functions.
- **`dwin_hmi.c`/`.h`** — DGUS-II display protocol driver, copied verbatim
  from `Shunting_Receiver_v2`, zero changes.
- **`hmi_map.h`, `hmi_state.c`/`.h`, `screen_sm.c`/`.h`** — ported from
  `Shunting_Receiver_v2`'s 14-screen HMI and trimmed to this project's
  scope at the time: no LoRa, no battery/charger, no buzzer, no
  obstacle-warning overlay. Delivers 13 screens (`00`–`12`): startup, 5
  pairing pages, connecting, error, telemetry, change-device,
  confirm-selection, end-shunting, shunting-completed. (Buzzer was added
  back in afterward — see `## Buzzer` below.)
- **`tf02pro.c`/`.h`** — present from the original reference project but
  dead code (never called) — left as-is, out of scope for this port.
- **`buzzer.c`/`.h`** — PWM buzzer driver, TIM2 CH1 / PA0. See `## Buzzer`
  below.
- **`ina226.c`/`.h`, `battery_soc.c`/`.h`, `charger_detect.c`/`.h`** —
  battery SoC + charger-plugged detection/switching. See
  `## Charger + Battery SoC` below.
- **Obstacle-warning overlay (page 14)** — no new files; lives entirely in
  `screen_sm.c`'s `TickObstacleOverlay()`, derived from the existing GSM
  distance reading. See `## Obstacle Warning` below.

## Status

**Stable and confirmed on real hardware (2026-08-30): 25+ minutes
continuous operation, zero disconnects, zero display flicker.** Full
pairing → connect → telemetry workflow working end-to-end: pairing screen
shows D-01 online, selecting and confirming it connects successfully,
Telemetry shows live distance with correctly-updating Health/Mode fields.

**Buzzer confirmed working on real hardware (2026-08-30)** — see
`## Buzzer` below for the port writeup and the `main.c` brace bug hit (and
fixed) along the way.

**Charger-detect + battery-SoC confirmed working on real hardware
(2026-08-30)**, including both boot paths (normal switch-on, and this
hardware's charger-triggered cold boot from fully off) — see
`## Charger + Battery SoC` below for the full port writeup and the four
real bugs found and fixed along the way.

**Obstacle-warning overlay ported (2026-08-30), code-complete, not yet
flashed/tested** — see `## Obstacle Warning` below.

## DWIN HMI port (2026-08-30)

Ported the full pairing/connect/telemetry DWIN workflow from
`Shunting_Receiver_v2` on top of this project's proven GSM driver — see
`## Architecture` above for the file-level detail. Key design choices:
- **RSSI not live-polled.** Status-bar health is derived from message
  recency (`GSM_GetMsSinceLastMessage()`) instead of a live `AT+CSQ` poll
  — avoids a second AT command competing with the main receive path for
  the same UART/response buffer.
- **Init ordering**: `DWIN_Init()`/`ScreenSM_Init()` run *before* the
  still-blocking `GSM_MQTT_Init()` in `main.c`, so the Startup screen
  shows immediately at power-on instead of staying blank through the
  whole (possibly long) cold-boot GSM connect sequence.

**Bug found and fixed during bring-up**: after adding USART1 via CubeMX
and regenerating code, CubeMX auto-generated its own
`GPIO_InitTypeDef GPIO_InitStruct = {0};` declaration in `MX_GPIO_Init()`
for PA5, colliding with the pre-existing hand-written declaration of the
same name inside the `USER CODE` block. Fixed by removing the redundant
declaration from the user-code block and reusing CubeMX's — confirmed
zero behavior change (same final PA5 config/state).

## Transmitter presence-response (2026-08-30)

`STM_Blackpill_GSM_Transmitter`'s `gsm_mqtt.c`/`.h` got the same kind of
additive extension: a new `presence_id` config field and
`GSM_MQTT_ServicePresenceQuery()`, which publishes `"01"` on
`shunting/presence` whenever a message arrives on `sub_topic`
(`shunting/query`). Same discipline as the receiver — only touch to
existing logic is one line in `printIncoming()`; the publish itself is a
near-duplicate of `GSM_MQTT_Publish()`'s body (different topic/message)
rather than a call into it, called after `GSM_MQTT_Poll()` returns so it
can't clobber `respBuf` before the caller checks it. Confirmed: D-01 shows
online on the pairing screen and connects successfully.

## Bug: Telemetry-screen Health/Mode/Distance flicker (2026-08-30) — root-caused and fixed

**Symptom**: once connected to a device, the Telemetry screen's
Health/Mode/Distance fields rapidly flickered between real values and
`--`. The pairing screen's device slots (D-01) never flickered even once,
and the onboard link-activity LEDs on both boards stayed rock solid
throughout — ruling out a naive "GSM keeps disconnecting" explanation
from the start.

**Diagnosis** — three rounds, each read live off the physical display
(no debugger/Live Watch available for this STM32CubeIDE-only project):

1. **`GSM_GetLastDispatchCode()`** — a short code (`PRES`/`DIST`/`MISS`/
   `NONE`) recording which branch of `GSM_DispatchIncoming()` classified
   the most recently received message. Always showed `DIST` — ruled out a
   topic-matching bug, but exposed a flaw in the diagnostic itself: the
   value is sticky (never reset), so it can't actually prove messages are
   arriving continuously, only that the last one to arrive, whenever that
   was, matched correctly.
2. **`GSM_GetDisconnectCount()`** — counted `mqttConnected` 1→0
   transitions, tracked entirely inside the new, additive
   `GSM_GetState()` (zero touch to the original driver). Stayed `0`
   through every flicker — ruled out the underlying MQTT session actually
   dropping and reconnecting.
3. **Seconds-since-last-message**, shown directly (capped at 999).
   Result: snapped instantly between `0` and `999` with no gradual climb
   through intermediate values — ruled out genuine network jitter slowly
   approaching the 15s staleness threshold, and pointed squarely at a
   timestamp-ordering bug instead of a real gap in traffic.

**Root cause**: in `main.c`'s `while(1)` loop, `uint32_t now =
HAL_GetTick();` was captured **before** `GSM_MQTT_Poll();` ran, not after.
On any iteration where that poll call dispatched an incoming distance
message, `gsm_mqtt.c`'s internal `s_lastMessageTick` got stamped with a
`HAL_GetTick()` value *newer* than the already-captured `now`.
`GSM_GetMsSinceLastMessage(now)` then computed `now - s_lastMessageTick`
with `now < s_lastMessageTick` — an unsigned integer underflow that
produced a huge "milliseconds since last message" value out of nowhere,
instantly tripping the 15s staleness check shared by both the status bar
(`link_up`) and distance blanking (`GsmLinkDown()`), for exactly one tick,
before the next iteration's correctly-ordered `now` recovered it. A pure
timestamp-ordering bug in this project's own `main.c` wiring — not a GSM
driver, MQTT, or network issue at all, which is why it was invisible to
the first two diagnostic rounds.

**Fix**: two-line reorder in `main.c` — capture `now` *after*
`GSM_MQTT_Poll()` returns, not before. Confined entirely to the
`USER CODE` markers; no GSM driver or screen-state-machine logic touched.

All temporary diagnostic code from all three rounds was fully reverted
afterward — `gsm_mqtt.c`/`.h` are back to the exact shape from the
original port plan, `screen_sm.c`'s `PushStatusBar()` is back to writing
`g_hmi.connected_device_name` to the device-name field.

**Confirmed fixed on real hardware 2026-08-30**: stable for 25+ minutes
continuous, zero disconnects, zero flicker.

## Buzzer (2026-08-30)

Ported `buzzer.c`/`.h` verbatim from `Shunting_Receiver_v2` — non-blocking
PWM driver, TIM2 CH1 / PA0, 12V piezo buzzer through an IRLZ44N MOSFET
gate. Both projects run the identical 96MHz clock tree (HSE 25MHz →
PLLM=25/PLLN=192/PLLP=DIV2 → 96MHz HCLK), confirmed by direct comparison
of `SystemClock_Config()` in both projects' `main.c`, so the reference's
`Prescaler=9`/`Period=9999` values apply directly — no recalculation
needed. The volume-to-PWM lookup table (`VOL_STEPS[]`) is hand-tuned
against the reference's specific buzzer/MOSFET pairing and may need
retuning by ear once heard on this project's own hardware.

`screen_sm.c` gained a `TickBuzzer()` wrapper (also ported from the
reference, called from `ScreenSM_Tick()`) that decides what the buzzer
should be doing each tick: silent outside the Telemetry screen, a steady
continuous alarm tone when `GsmLinkDown()` is true (adapted from the
reference's `BothLinksDown()` — GSM-only here, no LoRa to check), and
otherwise an intermittent proximity beep whose cadence tightens as
`g_hmi.distance_m` decreases (180ms under 5m down to 400ms at 20-44m,
silent beyond `DISTANCE_MAX_ACTIVE_M`) — cadence values ported as-is from
the reference's own hand-verified tuning. `main.c` calls
`Buzzer_Init(&htim2)` once at boot (before `ScreenSM_Init()`) and
`Buzzer_Tick(now)` every main-loop iteration (after `ScreenSM_Tick(now)`)
— matching the reference's exact call shape. No existing GSM/DWIN/
screen-flow logic touched; every change is either a new file or a new
function/call site.

**Bug hit during bring-up, isolated to `main.c`'s brace structure — not
the buzzer logic itself**: after adding TIM2 via CubeMX and regenerating,
the build failed with `main.c:176:1: error: expected identifier or '('
before '}' token`. Root cause: this file's `while(1)` loop body had, at
some point before this session, been written entirely inside the
`USER CODE BEGIN WHILE`/`END WHILE` block with its own closing `}` right
after the loop body — instead of CubeMX's actual standard template, where
that `}` belongs down in the `USER CODE BEGIN 3`/`END 3` slot instead.
That extra brace was silently tolerated by every regeneration up to now
apparently because the `USER CODE BEGIN 3` skeleton chunk wasn't part of
the file's history yet; the TIM2 regeneration re-inserted CubeMX's full
standard tail template (`USER CODE BEGIN 3 { ... } END 3`) for the first
time, and the file ended up with one closing brace too many across the
while-loop/`main()` structure. Fixed by removing the loop body's own
`}` and letting the loop close at CubeMX's standard position instead —
now matches the stock template exactly, so it won't recur on future
CubeMX regenerations.

**Confirmed working on real hardware 2026-08-30.**

## Charger + Battery SoC (2026-08-30)

Ported `ina226.c`/`.h` (thin I2C register driver), `battery_soc.c`/`.h`
(Coulomb-counting SoC estimator, wear-leveled flash persistence at sector
7) and `charger_detect.c`/`.h` (dual-channel ADC charger sensing +
relay/MOSFET charge-path switching) verbatim from `Shunting_Receiver_v2`,
plus the HMI plumbing that project's DWIN port had originally trimmed
out: `hmi_state.h`'s `battery_pct`/`charger_plugged`/`charger_near_full`/
`charge_pct`/`charger_overlay_active` fields and `SCR_13_CHARGER_PLUGGED`,
`hmi_map.h`'s `VP_BATTERY_PCT`/`VP_CHARGE_PCT_TEXT`/`PAGE_13_CHARGER_PLUGGED`,
and `screen_sm.c`'s `TickChargerOverlay()`/`ScreenSM_ForceStartupReset()`
(same charger-plugged-page-13-overlay technique as the source project —
`active_screen` keeps advancing underneath while the physical display
stays pinned on page 13, via `EnterScreen()`'s page-jump guard).

**Pin: PA5, same as the reference — not a conflict after all.** Initially
assumed PA5's existing "set HIGH at boot" GPIO code (from before the DWIN
port) was something unrelated, and retargeted the MOSFET to PA4 to avoid
disturbing it. User corrected this: PA5 already *is* the physically-wired
MOSFET gate — that pre-existing boot-time HIGH-set was this same MOSFET's
default-on state, added by hand before this driver existed. Reverted to
PA5, matching `Shunting_Receiver_v2` exactly; no PA4 GPIO configuration
needed in CubeMX at all (PA5 is already fully configured as an output).
PA6/PA7 (charger ADC sense)/PA8 (relay)/PB8/PB9 (I2C1) are still free and
port straight across unchanged, as before.

**Boot-sequencing fix, by explicit request — one genuine logic
divergence from the reference, not just the pin.** The gap described
above (PA5 briefly forced LOW by `ChargerDetect_Init()`'s "safety
default," then back HIGH once the first `ChargerDetect_Tick()` runs) was
flagged, and the user asked for it removed: PA5 should start HIGH at
boot and *stay* HIGH continuously, only going LOW once a charger is
actually, genuinely detected present. Fixed by removing just the
`MOSFET_OFF();` call from `ChargerDetect_Init()` (the `RELAY_OFF();`
call stays — PA8's own CubeMX boot state already matches it, so it's
still a harmless re-assertion, not a behavior change). PA5's
pre-existing HIGH-at-boot state is now left completely untouched all the
way through to the first `ChargerDetect_Tick()` call, which still
applies the correct ON/OFF state from that point onward exactly as
before (`UpdateRelayMosfet()`'s own logic is unchanged) — so a charger
genuinely present at boot is still correctly detected and switched to
OFF after the normal ~500ms ADC debounce; only the artificial
HIGH→LOW→HIGH blip in the no-charger case is gone.

**Charger-present sense logic, by explicit request — a second genuine
divergence from the reference.** `g_hmi.charger_plugged` ("detect
charging") previously came from PA6 (the present-threshold channel)
alone. Changed so it's true whenever *either* PA6 (present threshold,
~1.0V) OR PA7 (near-full threshold, ~2.0V) senses voltage above its own
threshold. Implementation: PA6's debounced result now lands in a new
internal `s_present_flag` instead of writing `g_hmi.charger_plugged`
directly; PA7's own debounce into `g_hmi.charger_near_full` is completely
unchanged (still used elsewhere for the `charge_pct` display gate and
`battery_soc.c`'s confirmed-full snap — this doesn't redefine what
near-full means, it's just now also a second way for charger-plugged to
become true); `g_hmi.charger_plugged = s_present_flag ||
g_hmi.charger_near_full;` is recomputed after every sample. Each
channel's own debounce timing (still `CHARGER_DEBOUNCE_MS` = 500ms,
still independent per channel) is completely unchanged — only the final
combination is new, so no timing regression on either channel
individually.

**Bug found and fixed on real hardware (2026-08-30): charger-plugged
detection (relay switch + charging-screen trigger) was noticeably slow
and inconsistent, not instant, after physically plugging in a charger.**
Root cause was in `UpdateDebouncedFlag()`, not the OR-logic above: it
accumulated a *fixed* `ADC_SAMPLE_INTERVAL_MS` (50ms) per disagreeing
sample, implicitly assuming every call represented exactly that much
real elapsed time. That assumption breaks down whenever the main loop's
actual cadence runs slower than 50ms between samples — which it
routinely does, since `GSM_MQTT_Poll()` (still running every iteration
until `charger_plugged` flips true — see `main.c`) can itself block for
up to ~100ms per call waiting on the UART, or considerably longer during
its once-per-second liveness AT check or a reconnect. Every slow
iteration meant real time kept passing while the accumulator still only
credited a flat 50ms per sample, so the debounce routinely took far
longer in actual wall-clock time than `CHARGER_DEBOUNCE_MS` was meant to
— and how much longer varied with how busy GSM happened to be at that
exact moment, matching the "inconsistent" part of the report. Fixed by
tracking the tick disagreement actually started at and comparing real
elapsed time against it instead of accumulating a fixed per-call
increment — correct and prompt regardless of how often the function
happens to get called. Also lowered `CHARGER_DEBOUNCE_MS` from 500ms to
150ms while at it (still a placeholder pending further bench tuning, per
its own comment, but 500ms was clearly perceptible even once measured
correctly). Confirmed on real hardware: still noticeably slow/
inconsistent even after this fix — see the architecture change below for
the actual remaining root cause and its fix.

**Architecture change (2026-08-30): charger detection moved off the main
loop entirely, onto its own timer interrupt — the real fix for the
remaining latency.** The elapsed-time debounce fix above made the debounce
*math* correct, but couldn't fix the deeper problem: `ChargerDetect_Tick()`
only ran when the main loop got around to calling it, and the main loop
is shared with `GSM_MQTT_Poll()`, which is *deliberately* blocking (that's
the entire reason this project uses this GSM driver instead of
`Shunting_Receiver_v2`'s non-blocking rewrite — see this file's own
`## Bug: Telemetry-screen ...` entry). `GSM_MQTT_Poll()` still runs on
every iteration right up until `charger_plugged` becomes true (the
charger-plugged gate only takes effect *after* detection succeeds, so it
never helped the detection phase itself) — and it can block for up to
~100ms per call routinely, or several seconds during its once-per-second
liveness check, or much longer mid-reconnect. Every one of those blocks
directly delayed how often ADC sampling could run, no matter how correct
the debounce math was.

This is also why `Shunting_Receiver_v2`'s exact charger-detection code
couldn't just be copied as-is: it works fine there specifically because
its GSM driver is a non-blocking rewrite that never meaningfully blocks
the main loop — the same main-loop-tick pattern behaves very differently
depending on which GSM driver sits underneath it.

**Fix**: `charger_detect.c` is now driven entirely by a dedicated
hardware timer (TIM3) firing every ~50ms via
`HAL_TIM_PeriodElapsedCallback()`, completely decoupled from the main
loop and GSM. `ChargerDetect_Tick()` no longer exists — replaced by
`ChargerDetect_Init(hadc, htim)` (now also binds+starts the timer) and
`ChargerDetect_TimerCallback()` (does the ADC sample + debounce + relay/
MOSFET actuation synchronously inside the ISR; a short 2ms bounded wait
on the ADC conversion is safe here since ISRs aren't bound by the main
loop's own "never block" constraint). `main.c` dispatches to it from a
new `HAL_TIM_PeriodElapsedCallback()` in `USER CODE BEGIN 4`, same
pattern as the existing `HAL_UART_RxCpltCallback()` dispatch. `g_hmi`
fields written from this ISR (`charger_plugged`/`charger_near_full`/
`charge_pct`) are read without a lock from the main loop — safe because
each is a single bool/uint8_t (atomic on this architecture) representing
continuously-valid state, not a discrete event that could be lost; a
lighter case than the DWIN touch-code producer/consumer FIFO. Detection
responsiveness is now bounded purely by `CHARGER_DEBOUNCE_MS` (150ms),
regardless of what GSM is doing. Confirmed working on real hardware,
2026-08-30 (after the further fixes below — this architecture change
alone wasn't sufficient by itself; see the rest of this section).

**New CubeMX peripheral required: TIM3** (Timers → TIM3 → Clock Source =
Internal Clock, no channels/pins needed — pure periodic-interrupt mode).
Parameter Settings: Prescaler = 9599, Counter Period = 499 (96MHz APB1
timer clock ÷ 9600 ÷ 500 = 20Hz = 50ms period, matching the existing
sample cadence). NVIC Settings: enable "TIM3 global interrupt", priority
lower than DWIN's USART1 RX (0,0) so a display byte can still preempt it
— e.g. 5,0. `main.c` already references `&htim3`, so the project will
not build until this step is done.

**Bug found and fixed on real hardware (2026-08-30): charging screen
still not appearing promptly, even with the timer-interrupt fix above —
"unless GSM is not turned ON."** Key fact surfaced by the user that
reframed the whole diagnosis: **PA5 (the MOSFET) is what actually turns
the GSM module's power on/off on this hardware** — it's not a separate,
firmware-independent switch as originally assumed, it's the exact same
pin `charger_detect.c` already drives. So the relay/MOSFET switch (and
hence GSM's power) was already happening instantly, right inside the
TIM3 ISR — but the physical *screen* switch was still entirely dependent
on `screen_sm.c`'s `TickChargerOverlay()`, which only runs from the main
loop's `ScreenSM_Tick()`. Since `GSM_MQTT_Poll()` is deliberately
blocking and, once already mid-call, can't be interrupted by the
charger event, the main loop could be stuck behind a single already-
in-progress GSM call — worst case tens of seconds, inside a
reconnect's own internal retry loop — before it ever reached
`ScreenSM_Tick()` again. What the user observed as "only shows once GSM
is not turned ON" was this: once PA5 cuts GSM's power, any further GSM
AT commands fail *fast* instead of blocking, which is what let the main
loop finally catch up and reach the screen-switching code — the screen
update was effectively waiting on GSM to lose power as a side effect,
not switching on the charger event itself.

**Fix**: `charger_detect.c`'s `UpdateRelayMosfet()` now also calls
`DWIN_SwitchPage()` directly, in the same ISR, the same instant the
relay/MOSFET switch — completely bypassing the GSM-blocked main loop,
since DWIN is on a separate UART (USART1) that GSM's blocking never
touches. This is a redundant, fire-and-forget nudge, not a replacement
for `TickChargerOverlay()`: that function still runs from the main loop
shortly after and does its full job exactly as before (setting
`g_hmi.charger_overlay_active`, writing the live charge% text, and — on
unplug — `GSM_Reset()`/`ScreenSM_ForceStartupReset()`, deliberately
*not* duplicated in the ISR since that touches far more state than is
safe to run from interrupt context). Switching to the same page twice is
harmless.

**A real concurrency hazard had to be closed to make this safe**:
`dwin_hmi.c`'s `SendFrame()` (used by every `DWIN_Write*`/`SwitchPage`
call) can now be entered from both the main loop and this ISR
concurrently. `HAL_UART_Transmit()`'s own internal busy-check has an
unprotected read-then-write window — if the ISR preempted the main loop
at exactly that instant, both contexts could see the UART peripheral as
"ready" and write to its data register at the same time, genuinely
corrupting the transmission (interleaved bytes on the wire). Fixed with
a new `s_tx_busy` flag in `dwin_hmi.c`, claimed inside a tiny
(nanoseconds — just the flag check-and-set, not the transmit itself)
`__disable_irq()`/`__enable_irq()` critical section; the loser of a race
silently drops its frame instead of corrupting the winner's. This
critical section is far shorter than one UART byte-time at 115200 baud
(~87µs), so it can't cause DWIN's own RX interrupt to miss an incoming
touch-code byte. This is the second genuine touch to `dwin_hmi.c`'s
"proven, don't touch" logic this project has needed (first was none
before this — `dwin_hmi.c` had been byte-identical to the source project
until now), done narrowly and only because correctness genuinely
required it once a second caller (an ISR) was introduced.

**Build error hit and fixed along the way**: the doc comment added to
`dwin_hmi.h` for this change contained a literal `Write*/SwitchPage`
substring — the `*/` inside it prematurely closed the C comment block,
turning the rest of the comment into garbage code and cascading into
~49 unrelated-looking errors across every file that includes
`dwin_hmi.h`. Fixed by rewording to avoid the literal `*/` sequence;
scanned every file touched in this change for the same mistake
afterward (none found).

**Bug found and fixed on real hardware (2026-08-30): even after the ISR
nudge above, the charging screen still didn't reliably appear — root
cause was the nudge being edge-triggered (fires once, on the exact
transition) rather than level-triggered.** If that one
`DWIN_SwitchPage()` call happened to lose `dwin_hmi.c`'s tx-busy race
against a concurrent main-loop DWIN write (see the concurrency-hazard
entry above), nothing ever retried it — the display could be left
stuck on the wrong page indefinitely even with the relay genuinely on,
since `UpdateRelayMosfet()`'s "only act on change" guard meant it
wouldn't try again until the next actual plug/unplug transition. Fixed
per explicit request ("if relay turned ON, it should show charging
screen, no matter what"): the page-switch became level-triggered rather
than a one-shot edge nudge — while `g_hmi.charger_plugged` stayed true,
`UpdateRelayMosfet()` re-sent `DWIN_SwitchPage(PAGE_13_CHARGER_PLUGGED)`
on every single TIM3 tick (~50ms), unconditionally, forever. Confirmed
on hardware: the screen switch itself became reliable — but this broke
something else.

**Second bug found and fixed on real hardware (2026-08-30): charging
screen now switched reliably, but the charge percentage stayed stuck on
"--".** Root cause: a DGUS page-switch command, even to the page already
showing, appears to redraw the page fresh from its template — sending it
continuously (every 50ms, forever, per the fix above) was wiping
`TickChargerOverlay()`'s own charge-percentage text back to its template
placeholder faster than its 1000ms periodic refresh could ever keep up.
The percentage was likely actually visible for a few milliseconds out of
every second, imperceptible to the eye — reading as permanently stuck.
Fixed by bounding the ISR-side repeat to only the "catch-up window": it
keeps nudging every tick, but only while `g_hmi.charger_overlay_active`
(set by `TickChargerOverlay()` in the main loop; the ISR only reads it,
never writes it) is still false on entry / still true on exit — i.e.,
only until the main loop confirms it has taken over. The moment
`charger_overlay_active` flips, the ISR stops sending switch commands
entirely, leaving `TickChargerOverlay()`'s own page-switch and
percentage-text writes completely undisturbed from then on. This keeps
the exact same reliability guarantee as the level-triggered version
(retries as many times as it takes, for however long the main loop is
stuck behind GSM, self-healing against any dropped frame) while no
longer fighting the text updates once the transition is actually
complete. Applied symmetrically to the unplug direction. GPIO writes
remain only-on-change throughout, unaffected by either fix — only the
DWIN command's repeat behavior changed.

**Third bug found and fixed (2026-08-30): charging percentage worked
when the charger was plugged in while the system was already running,
but not when the charger itself powered the STM32 on from fully off.**
The user's hardware has a separate circuit that lets the charger boot
the STM32 directly, bypassing the normal system power switch entirely —
so `g_hmi.charger_plugged` can already be the real, physical state right
at boot, not just something that happens later while already running.
Root cause: `GSM_MQTT_Init()` ran unconditionally at boot, before the
`while(1)` loop — and hence before `ScreenSM_Tick()`/
`TickChargerOverlay()`, the only thing that ever writes the percentage
text, ever got a chance to run even once. If the charger already powered
the board on, GSM has no power (PA5 cuts it), so `GSM_MQTT_Init()` just
blocked for a long time failing against a dead module before eventually
giving up — during that whole window the main loop hadn't even started.
The charging *page* still appeared correctly (via
`ChargerDetect_TimerCallback()`'s own ISR-side nudge, independent of the
main loop), but the percentage stayed on its template placeholder the
entire time.

**Fix**: after `ChargerDetect_Init()` starts its timer, `main.c` now
waits a bounded 300ms (comfortable margin over the ~200ms worst case for
the ISR's own ADC debounce to settle) before checking
`g_hmi.charger_plugged` for the first time. If the charger is already
present, `GSM_MQTT_Init()` is skipped entirely at boot (pointless with
no power to the module, and the main loop starts immediately instead of
blocking). A new `gsm_inited` flag tracks whether it actually ran; if
skipped, the main loop calls it exactly once, deferred, the first time
the charger is unplugged and real operation begins — GSM still needs to
come up at some point, just not blocking this specific boot path. Known,
accepted side effect: that first unplug carries the same one-time
blocking GSM-connect wait a normal cold boot already has, just shifted
to occur then instead of at power-on — reasonable, since the module
never had power to attempt it any earlier anyway.

**Fourth bug found and fixed (2026-08-30): even with the GSM-deferral fix
above, the charging *screen itself* still didn't reliably appear on a
charger-triggered cold boot — not just the percentage this time.** This
ruled out the GSM-deferral fix as the cause (the ISR's own page-switch
nudge in `UpdateRelayMosfet()` is completely independent of
`GSM_MQTT_Init()`/`GSM_MQTT_Poll()`, so GSM blocking can't explain a
missing screen switch). Leading theory: on this hardware's
charger-triggered cold boot, the DWIN display's own power rail may still
be settling while this ISR is already firing — every attempt during the
fast, every-tick retry window (active only until
`g_hmi.charger_overlay_active` confirms success — see the level-triggered
fix earlier in this section) could be lost simply because the display
wasn't yet powered up enough to receive UART commands. Since
`charger_overlay_active` confirms only that *software* processed the
transition, not that the display actually received anything, the ISR
would go permanently silent right as the display was becoming ready,
missing the whole window.

**Fix**: `UpdateRelayMosfet()` now paces itself in two speeds — fast
(every tick) while unconfirmed, exactly as before, then a much slower
heartbeat (`CHARGING_PAGE_HEARTBEAT_MS` = 1500ms) that continues
indefinitely for as long as `charger_plugged` stays true, instead of
going silent once confirmed. The heartbeat is slow enough to coexist
with `TickChargerOverlay()`'s own 1000ms percentage refresh without
starving it the way the earlier every-tick-forever version did, while
still eventually reaching a display that wasn't ready during the initial
fast-retry window, and staying resilient against any later transient
`dwin_hmi.c` tx-busy loss too. Deliberately only applied to the charging
(ON) direction — unplugging only ever happens well into an
already-running session, where the display has obviously been up and
responsive for a while, so that direction keeps its simpler one-shot
nudge, unchanged.

**Confirmed working on real hardware, 2026-08-30** — both boot paths
(normal switch-on, and this hardware's charger-triggered cold boot from
fully off) now correctly and promptly show the charging screen with a
live, correct percentage, and the relay/MOSFET/screen/percentage all
stay in sync during normal already-running-then-plug-in operation too.
This closes out the whole charger-detect responsiveness saga (four real
bugs, each found on real hardware, documented in sequence above).

**One deliberate divergence from the reference's `main.c` shape**: the
reference captures `now = HAL_GetTick()` at the very top of its loop,
before polling GSM — but this project's `now` must stay captured *after*
`GSM_MQTT_Poll()` returns, per the hard-won fix in
`## Bug: Telemetry-screen Health/Mode/Distance flicker` above. Preserved
that ordering here rather than copying the reference literally; net
per-iteration sequence is `if (!g_hmi.charger_plugged) GSM_MQTT_Poll();`
→ `now = HAL_GetTick();` → `ChargerDetect_Tick(now); BatterySoc_Tick(now);
ScreenSM_Tick(now); Buzzer_Tick(now);`. GSM polling is suspended while
charging — matches the reference's own `main.c` exactly, since a separate
physical power switch cuts the GSM module's power during charging on
this hardware.

`BatterySoc_Init()`/`ChargerDetect_Init()` are called after
`ScreenSM_Init()` (not before, unlike the reference's own order) — this
project's established principle from the DWIN port is showing the
Startup screen as fast as possible before any blocking hardware init;
`BatterySoc_Init()` blocks ~500ms for sensor settle, same reasoning
already applied to `GSM_MQTT_Init()`.

**CubeMX steps** (ADC1 on PA6/PA7, I2C1 on PB8/PB9, a GPIO output on PA8
for the relay — PA5 was already configured, no CubeMX change needed for
it — and TIM3, see the architecture-change entry below) all completed by
the user; project builds clean.

**Resolved**: the physical DWIN display does have page 13 (Charger
Plugged) built, with `VP_CHARGE_PCT_TEXT` bound correctly — confirmed by
the whole charger-detect responsiveness saga above actually working end
to end on real hardware.

## Obstacle Warning (2026-08-30)

Ported `TickObstacleOverlay()` byte-for-byte from `Shunting_Receiver_v2`
— its detection/recovery logic derives entirely from `g_hmi.distance_m`
(already populated by this project's existing GSM telemetry path via
`TickDistanceFromLinks()`), so despite the reference's own comments
mentioning LiDAR obstacle detection, this port needed no LiDAR, no new
sensor, and no new files. Pure software: pops up page 14 automatically
when the tracked distance suddenly drops by `OBSTACLE_DROP_THRESHOLD_M`
(5m) and holds for `OBSTACLE_CONFIRM_HOLD_MS` (2000ms) without recovering
— a person/object stepping into the beam ahead of the transmitter —
and clears the instant distance rises back up by the same 5m threshold.
Same overlay technique already proven by the charger-plugged page:
`g_hmi.active_screen` stays `SCR_04_TELEMETRY` throughout, only
`g_hmi.obstacle_overlay_active` and a direct `DWIN_SwitchPage()` change
— which is what lets the buzzer's proximity tone, the status bar, and
Volume/END SHUNTING touch handling keep working completely unchanged
while the warning is showing, with zero code changes needed in any of
them.

**Additive changes**: `hmi_state.h` gained `SCR_14_OBSTACLE_WARNING` and
`obstacle_overlay_active`; `hmi_map.h` gained `PAGE_14_OBSTACLE_WARNING`;
`screen_sm.c` gained the two timing constants
(`OBSTACLE_DROP_THRESHOLD_M`/`OBSTACLE_CONFIRM_HOLD_MS`),
`TickObstacleOverlay()` itself (called from `ScreenSM_Tick()` right after
`TickDistanceFromLinks()`, before `TickTimers()` — matching the
reference's exact order), and `EnterScreen()`'s page-jump guard extended
with `&& !g_hmi.obstacle_overlay_active` (same pattern already used for
the charger overlay). Nothing in `HandleTouch()`, `TickBuzzer()`,
`TickPeriodicPushes()`, or `ScreenSM_ForceStartupReset()` needed any
change — confirmed by direct comparison against the reference, matching
its own design intent.

**Known open item, not resolvable from code**: whether the physical DWIN
display actually has page 14 (Obstacle Warning) built has not been
confirmed on this board (unlike page 13, confirmed working via the
charger-detect saga). If missing, firmware still runs correctly; the
warning just won't show the expected content. Not yet built/flashed or
tested on hardware — this whole feature is code-complete but unconfirmed.

## Known limitations / deliberately out of scope

- No LoRa — this port is GSM+DWIN+buzzer+battery/charger+obstacle-warning,
  per explicit scope.
- RSSI is not live-polled (see `## DWIN HMI port` above for why).
- `gsm_mqtt.c`'s AT-command engine is still duplicated near-verbatim
  between this project and `STM_Blackpill_GSM_Transmitter` (same
  situation as `Shunting_Receiver_v2`/`Shunting_Transmitter_v2`) —
  acknowledged, not refactored; real maintainability issue but out of
  scope here.
- Device-switch (`GSM_Disconnect()`) does not send `AT+CMQTTUNSUB` — the
  module stays subscribed to the old device's topic at the AT layer,
  `GSM_DispatchIncoming()` just ignores it locally. Low risk (no stale
  distance leaks through), deferred because the exact unsubscribe syntax
  is unverified on real hardware in this project's testing lineage.
- `tf02pro.c`/`.h` present but unused — left as dead code, out of scope.

## Telemetry distance: distinguish "out of range" from "disconnected" (2026-08-30)

Permanent improvement, by explicit request — not part of the temporary
section below. Previously, the Telemetry screen's distance field showed
`"--"` for two genuinely different situations: the transmitter is still
connected and sending real readings, just beyond the 44m active range
(`DISTANCE_MAX_ACTIVE_M`); or the link itself is down entirely (e.g. the
transmitter is powered off), leaving whatever distance was last received
stale and meaningless. Now shown distinctly in `screen_sm.c`'s
`TickPeriodicPushes()`:
- **`"DC"`** (disconnected) — `GsmLinkDown(now)` is true. Checked first
  and takes priority over the range check below, since a stale sample's
  numeric value says nothing real once the link itself is down.
- **`"OR"`** (out of range) — link is up, but `g_hmi.distance_m >
  DISTANCE_MAX_ACTIVE_M`.
- Otherwise, the real live distance, unchanged.

No change to `GsmLinkDown()`, the health computation, or any connection
logic — purely a more informative rendering of state that already
existed.

## TEMPORARY: fake LORA/GSM mode display (2026-08-30) — REMOVE LATER

**This project has no real LoRa link** — GSM-only, per the whole scope of
this port (see `## Architecture` above). By explicit request, the status
bar's mode field (`VP_CONN_MODE`) now shows **"LORA"** whenever the live
distance (`g_hmi.distance_m`) reads under 44m (`DISTANCE_MAX_ACTIVE_M`),
and **"GSM"** at 44m or more — purely cosmetic, for demo purposes only.
The actual link is always GSM regardless of what this field displays;
nothing about the real connection, health computation, or link logic
changed. Confined to one line in `screen_sm.c`'s `PushStatusBar()`,
clearly marked `TEMPORARY` in the code comment right above it.

**To revert**: in `screen_sm.c`'s `PushStatusBar()`, replace the
conditional `DWIN_WriteVPString(VP_CONN_MODE, (g_hmi.distance_m < DISTANCE_MAX_ACTIVE_M) ? "LORA" : "GSM", MODE_TEXT_FIELD_BYTES);`
back to the unconditional `DWIN_WriteVPString(VP_CONN_MODE, "GSM", MODE_TEXT_FIELD_BYTES);`
and delete this section once no longer needed.

## Next steps

- User to build, flash, and confirm the obstacle-warning overlay on real
  hardware: a sudden ≥5m distance drop during Telemetry pops up page 14
  within ~2 seconds, the buzzer/status bar/Volume/END SHUNTING keep
  working while it's showing, and it clears within one tick of distance
  rising back up by 5m. Also confirm page 14 actually exists on the
  physical display (see the open item above).
- Beyond that: to be defined — hand off point for whatever comes after
  this milestone. DWIN HMI, GSM (presence + telemetry), buzzer, and
  battery/charger-detect (including both boot paths) are all confirmed
  working together on real hardware as of 2026-08-30.

### September 15, 2026: AWS IoT Re-provisioning Bug Fix
- **Root Cause**: AWS_Init() in ws_manager.c contained an unconditional call to AWS_CertStorage_Erase(). This caused the device to wipe its stored certificate on every boot and attempt Fleet Provisioning by claim repeatedly, preventing connection and creating stale/duplicate Things in AWS IoT.
- **Fix**: Removed the unconditional AWS_CertStorage_Erase() call from AWS_Init(). The other legitimate erase calls (in AWS_CertStorage_Save and on cert rejection) were preserved.
- **Files Touched**: Core/Src/aws_manager.c in both STM_Blackpill_GSM_Receiver and STM_Blackpill_GSM_Transmitter.
- **Note**: The AWS payload JSON changed from "SerialNumber"/"DeviceType" fields to a single "DeviceId" field compared to the known-working reference. Ensure any AWS IoT Rules or Lambdas downstream are updated to expect "DeviceId" instead of "SerialNumber".
