/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gsm_mqtt.h"
#include "aws_manager.h"
#include "hmi_state.h"
#include "hmi_map.h"
#include "dwin_hmi.h"
#include "screen_sm.h"
#include "buzzer.h"
#include "battery_soc.h"
#include "lora_e220.h"
#include "modem_gate.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* AWS_Init() retry pacing (see the main loop): 5s is the MASTER PROMPT 9.2
 * base interval, doubled after each failed attempt up to the cap. */
#define AWS_RETRY_BASE_MS    5000U
#define AWS_RETRY_MAX_MS    60000U
/* Reconnect pacing after a dropped session. Must exceed GSM_MQTT_Poll()'s own
 * internal 5s reconnect throttle (strict ">") or a call could be swallowed. */
#define AWS_RECONNECT_BASE_MS  6000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdbool.h>
bool g_system_initialized = false;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();

  /* USER CODE BEGIN 2 */
  HmiState_Init();
  DWIN_Init(&huart1);
  Buzzer_Init(&htim2);  /* must run before ScreenSM_Init()/the main loop
                            start calling ScreenSM_Tick() -> TickBuzzer(),
                            which assumes the timer handle is already
                            bound. Matches Shunting_Receiver_v2's relative
                            ordering (Buzzer_Init() before ScreenSM_Init()). */
  ScreenSM_Init();   /* sends the 00_Startup page-jump now, before any of
                         this project's blocking hardware inits, so the
                         display shows something immediately at power-on.
                         AWS_Init() is NOT called here any more — it
                         moved into the main loop (see the retry block
                         below) so it can never hold the Startup screen up
                         or freeze LoRa/touch handling for its whole
                         (possibly minutes-long) connect sequence. */
  BatterySoc_Init(&hi2c1);       /* after ScreenSM_Init() too, for the same
                                     reason — this blocks ~500ms for sensor
                                     settle, and the Startup screen should
                                     already be showing before any of this
                                     project's blocking hardware inits run. */
  /* No boot-time E220 configuration-mode switch: M0/M1 (PA4/PA1) are held
   * LOW (Normal/transparent mode) by MX_GPIO_Init() and nothing ever
   * changes them. A boot-time configuration write was tried in the
   * standalone project and removed — it was the likely cause of the
   * module's saved settings reverting to factory defaults and of an
   * intermittent total link loss; the link runs reliably (1.5km tested)
   * with the module simply left in Normal mode. */
  LoRa_Init(&huart6);   /* arms interrupt-driven RX on the LoRa UART
                            (USART6, PA11/PA12). Doesn't talk to the module,
                            so it's harmless to call before PA5 (the MOSFET
                            powering the E220 + GSM modem) has settled. */

  /* AWS IoT Core / GSM bring-up is retried from inside the main loop,
   * never blocked on here: AWS_Init() can run for minutes (modem boot,
   * network registration, fleet provisioning), and LoRa_Poll(), the touch
   * handling and the buzzer all live in that same loop. Same non-blocking
   * retry pattern the transmitter already uses; AWS_Init() itself is called
   * exactly as before (same arguments, never modified — see MASTER PROMPT
   * section 9). */
  bool     aws_inited         = false;
  uint32_t lastAwsRetryTick   = 0;
  uint32_t awsRetryIntervalMs = AWS_RETRY_BASE_MS;
  uint32_t lastAwsReconnectTick   = 0;
  uint32_t awsReconnectIntervalMs = AWS_RECONNECT_BASE_MS;
  g_system_initialized = true;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      LoRa_Poll(HAL_GetTick());   /* first, every iteration, independent of
                                      GSM/AWS state — LoRa presence tracking
                                      and the rolling link window must never
                                      wait behind the cellular bring-up
                                      below. Its own timestamp, deliberately
                                      NOT the `now` captured further down
                                      (which must stay AFTER
                                      GSM_MQTT_Poll() — see that capture's
                                      own comment). */

      if (!aws_inited) {
          /* Non-blocking retry, MASTER PROMPT 9.2 shape, with three
           * additions on top of the plain fixed 5s timer:
           *  1. Not before the Startup screen has finished — a single
           *     AWS_Init() call has no yield points, so attempting it on
           *     the first iteration would freeze the loop (and with it
           *     ScreenSM_Tick()'s hold timer) until it returns.
           *  2. Not while a LoRa session is live (a transmitter selected:
           *     LISTENING/STABLE) — a failed attempt can block for minutes,
           *     which would stall LoRa_Poll() and the obstacle buzzer
           *     mid-shunt. AWS then simply comes up the next time the
           *     receiver is back on the pairing screens.
           *  3. ModemGate_SimReady() first: two ordinary AT commands (~2s
           *     worst case) that skip a hopeless attempt outright when the
           *     modem is silent or there is no SIM. Replaces the standalone
           *     project's fullReconnect() SIM check, which can't live here
           *     because gsm_mqtt.c is protected.
           * Every failed attempt doubles the wait (5s -> 10s -> ... -> 60s
           * cap) so an uncovered or SIM-less unit doesn't keep freezing the
           * loop; the wait is measured from the END of the attempt so a
           * long attempt is always followed by real loop time. */
          if ((g_hmi.active_screen != SCR_00_STARTUP) &&
              (LoRa_GetState() == LORA_LINK_IDLE) &&
              ((HAL_GetTick() - lastAwsRetryTick) >= awsRetryIntervalMs)) {
              if (ModemGate_SimReady(&huart2)) {
                  aws_inited = AWS_Init(&huart2, "airtelgprs.com");
              }
              lastAwsRetryTick = HAL_GetTick();
              if (aws_inited) {
                  awsRetryIntervalMs = AWS_RETRY_BASE_MS;
              } else if (awsRetryIntervalMs < AWS_RETRY_MAX_MS) {
                  awsRetryIntervalMs *= 2U;
                  if (awsRetryIntervalMs > AWS_RETRY_MAX_MS) {
                      awsRetryIntervalMs = AWS_RETRY_MAX_MS;
                  }
              }
          }
      } else {
          if (GSM_MQTT_IsConnected()) {
              GSM_MQTT_Poll();
              awsReconnectIntervalMs = AWS_RECONNECT_BASE_MS;
          } else if ((g_hmi.active_screen != SCR_00_STARTUP) &&
                     (LoRa_GetState() == LORA_LINK_IDLE) &&
                     ((HAL_GetTick() - lastAwsReconnectTick) >= awsReconnectIntervalMs)) {
              /* Session dropped (coverage loss, failed publish, modem
               * reset...). With no MQTT session GSM_MQTT_Poll() does nothing
               * but run its own reconnect — fullReconnect(), a blocking
               * sequence that can hold the loop for a minute or more (network
               * wait, TLS connect...) and, when it fails, is retried
               * again straight away. Called unconditionally that would mean
               * one loop iteration per minute for as long as the outage
               * lasts: no LoRa tracking, no touch handling, no buzzer.
               * Same rules as the first-connect retry above: only from the
               * pairing screens (never while a transmitter is selected —
               * LoRa alone must keep working through a cellular outage),
               * only when the modem is up with a SIM, and backing off
               * 6s -> 12s -> ... -> 60s after each failure. */
              if (ModemGate_SimReady(&huart2)) {
                  GSM_MQTT_Poll();   /* not connected: runs its reconnect */
              }
              lastAwsReconnectTick = HAL_GetTick();
              if (GSM_MQTT_IsConnected()) {
                  awsReconnectIntervalMs = AWS_RECONNECT_BASE_MS;
              } else {
                  awsReconnectIntervalMs *= 2U;
                  if (awsReconnectIntervalMs > AWS_RETRY_MAX_MS) {
                      awsReconnectIntervalMs = AWS_RETRY_MAX_MS;
                  }
              }
          }
          AWS_Manager_Tick(HAL_GetTick());
      }
      /* `now` must be captured AFTER GSM_MQTT_Poll() returns, not before —
       * that call can dispatch an incoming message partway through and
       * set gsm_mqtt.c's internal s_lastMessageTick to a HAL_GetTick()
       * value newer than a `now` captured at the top of the loop. Root
       * cause of the Telemetry-screen Health/Mode/Distance flicker
       * (2026-08-30): GSM_GetMsSinceLastMessage(now) computed
       * `now - s_lastMessageTick` with now < s_lastMessageTick on any
       * iteration where a message arrived mid-poll — unsigned
       * underflow produced a huge "milliseconds since last message"
       * value out of nowhere, instantly (not gradually) tripping the
       * 15s staleness check and flipping the status bar/distance to
       * "--" for exactly one tick before the next iteration's fresh,
       * correctly-ordered `now` recovered it. */
      uint32_t now = HAL_GetTick();
      BatterySoc_Tick(now);
      ScreenSM_Tick(now);
      Buzzer_Tick(now);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9599;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level for PC13 (Blue LED, active low) */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pins : PA1 PA4 PA5 PA8 (PA1/PA4 = E220 M1/M0, held LOW =
    Normal mode, never driven anywhere else) */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin   = GPIO_PIN_5;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    DWIN_UART_RxCpltCallback(huart);
    LoRa_UART_RxCpltCallback(huart);
    /* GSM stays fully blocking/polled in this project (unchanged, proven
     * behavior) — it has no *_UART_RxCpltCallback of its own, so no
     * dispatch fan-out is needed here for it. USART1 (DWIN), USART2 (GSM)
     * and USART6 (LoRa) are electrically and logically independent; no
     * NVIC/priority interaction between them. Each dispatched function
     * checks huart against its own stored handle and no-ops if it doesn't
     * match, so calling both unconditionally here is safe. */
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        DWIN_UART_RxCpltCallback(huart);
    }
    LoRa_UART_ErrorCallback(huart);   /* no-ops unless huart is USART6 */
}

/* __io_putchar was declared `extern ... __attribute__((weak))` in
 * syscalls.c with no definition anywhere in this project. An unresolved
 * weak symbol resolves to address 0, so any code path that actually
 * reaches real printf()/vprintf() (e.g. logger.c's log_print(), used by
 * cmd_router.c/device_sm.c if those are ever wired into main()) would
 * call through a null function pointer and HardFault the MCU. This is a
 * safe no-op: it transmits nothing (this project's USART1 is already
 * dedicated to the DWIN display and must not be shared with a debug
 * console), but makes printf() callable without crashing. aws_manager.c's
 * own `#define printf dbg` and gsm_mqtt.c's dbg() are unaffected either
 * way (dbg() never reaches this path at all). */
int __io_putchar(int ch)
{
    (void)ch;
    return ch;
}

void HAL_Delay(uint32_t Delay)
{
    uint32_t tickstart = HAL_GetTick();
    uint32_t wait = Delay;
    if (wait < HAL_MAX_DELAY)
    {
        wait += (uint32_t)(uwTickFreq);
    }
    while ((HAL_GetTick() - tickstart) < wait)
    {
        extern bool g_system_initialized;
        if (g_system_initialized) {
            LoRa_Poll(HAL_GetTick());
            ScreenSM_Tick(HAL_GetTick());
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
