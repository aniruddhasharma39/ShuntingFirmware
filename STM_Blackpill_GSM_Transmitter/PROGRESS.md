# Progress Log — STM_Blackpill_GSM_Transmitter

### September 26, 2026: LoRa broadcast added alongside AWS IoT Core

Ported from the standalone (plain-MQTT) working transmitter. **Everything the
firmware team marked off-limits is untouched** (`aws_manager.c/h`,
`gsm_mqtt.c/h`, `device_config.h`, AWS_Init()'s body and arguments,
AWS_PublishTelemetry(), topics, payloads, certificates, flash sector 6). The
change is purely additive.

- **LoRa broadcast** (`lora_tx.c/h`, new): frame `*D<NN>:<cm>\r\n` once per
  second on USART1 (PA9/PA10) to the E220 (M0/M1 hardwired to GND, Normal
  mode). `<NN>` is parsed from `DEVICE_ID` ("TX-NN") at `LoRaTx_Init()`, so
  units differ only by that one define; the distance is in centimetres (the
  TF02-Pro's native unit, same as the AWS telemetry) and is sent even before
  the first valid reading (0 cm) because the frame is also the receiver's
  presence signal. The receiver's `lora_e220.c` parses exactly this format.
- **Timer + DMA driven, independent of the main loop**: TIM2 (1 Hz) calls
  `LoRaTx_Tick()` from `HAL_TIM_PeriodElapsedCallback()`, which starts a
  non-blocking `HAL_UART_Transmit_DMA()` (DMA2 Stream7, static frame buffer).
  A blocking `AWS_Init()` / GSM call therefore can never delay or skip a
  broadcast. `TF02_Init()`, `LoRaTx_Init()` and the timer now start *before*
  the boot-time `AWS_Init()` attempt (they used to come after it), so LoRa is
  on the air from the first second even with no SIM/coverage.
- **SIM check** (replaces the standalone `fullReconnect()` fix, which cannot
  be ported because `gsm_mqtt.c` is protected): `modem_gate.c` sends plain
  `AT` + `AT+CPIN?` (~2 s worst case) and `AWS_Init()` is only attempted when
  the modem answers and a SIM is present — at boot and in the existing
  non-blocking 5 s retry. `AWS_Init()` itself is called exactly as before.
- `tf02pro.c/h`: added `TF02_GetDistance_cm()` (non-consuming read of the
  latest distance) so the LoRa timer never steals the "new reading" flag that
  `TF02_GetLatest()` gives the AWS publish path.
- Build/peripheral changes: `HAL_TIM_MODULE_ENABLED` turned on in
  `stm32f4xx_hal_conf.h`; HAL TIM driver sources/headers added under
  `Drivers/`; TIM2, USART1 (+DMA2 Stream7 TX) generated code added to
  `main.c`, `stm32f4xx_hal_msp.c`, `stm32f4xx_it.c`; `.ioc` updated to match.
  Flash use ~42 KB; nothing near sector 6 (0x08040000, AWS credentials).

#### Pre-flash audit follow-ups (same day)

- `LoRaTx_Tick()` builds its frame by hand (no `snprintf` inside the timer
  interrupt) and only touches the DMA buffer once the previous transfer has
  finished; if a transfer ever stays in flight for 3 consecutive broadcasts it
  is aborted so the broadcast recovers by itself.
- The unit's LoRa number is the `NN` of `DEVICE_ID` in `device_config.h`
  (currently "TX-02", so the receiver lists it as D-02).
- **GPIO LoRa Power & Pin Config**: Initialized PA1/PA4 (M1/M0 LOW for Normal Mode)
  and PA5/PA8 (Power gate / Enable HIGH) in `MX_GPIO_Init` so the E220 module is powered
  and transmitting reliably.
- **Offline Telemetry Queueing & isLoRa Flag**: Added `OfflineTelemetry_t` FIFO queue
  (128 capacity) in `aws_manager.c/h`. When GSM is disconnected, LiDAR readings are
  stored offline with their timestamp (`uptime_s`), and automatically drained and
  published to AWS IoT once GSM reconnects. Payload format now carries `"isLoRa": true/false`.

