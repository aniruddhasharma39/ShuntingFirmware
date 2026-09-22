# ShuntingFirmware

Firmware repository for the Shunting System built for STM32 Blackpill (STM32F411CEU6) and SIMCom A7670C GSM/LTE module with AWS IoT Core integration and DWIN HMI display.

## Projects
- **STM_Blackpill_GSM_Receiver**: Receiver unit firmware with DWIN DGUS-II HMI display driver, PWM proximity buzzer, INA226 battery fuel gauge, hardware charger detection, and AWS IoT Core MQTT client (Fleet Provisioning, status heartbeat, live telemetry, and transmitter distance reception).
- **STM_Blackpill_GSM_Transmitter**: Transmitter unit firmware with TF02-Pro LiDAR distance sensor driver and AWS IoT Core MQTT telemetry & presence publisher.

## Development Environment
- STM32CubeIDE
- Toolchain: GNU Arm Embedded (arm-none-eabi-gcc)
- Hardware: STM32F411CEU6 (Blackpill), SIMCom A7670C, TF02-Pro LiDAR, DWIN DGUS-II Display, INA226
