/* ============================================================
 * gsm_mqtt.c
 *
 * Shared A7670C AT-command + native MQTT driver for STM32 HAL.
 * Uses the A7670C native AT+CMQTT command family.
 * ============================================================ */

#include "gsm_mqtt.h"
#include "main.h"
#include "aws_manager.h"
#include "device_config.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define RESP_BUF_SIZE       8192U
#define RX_CHUNK_TIMEOUT    600U
#define CMD_TIMEOUT         3000U
#define LONG_TIMEOUT        15000U
#define VERY_LONG_TIMEOUT   60000U
#define URC_WAIT_TIMEOUT    5000U

/* Set to 1 for complete AT traffic. */
#define GSM_MQTT_VERBOSE_AT 1

static UART_HandleTypeDef *hAT = NULL;
static GSM_MQTT_Config gcfg;
static uint8_t mqttConnected = 0U;

static char respBuf[RESP_BUF_SIZE];

/* Large enough for AWS Fleet Provisioning JSON responses. */
static char incomingTopic[256];
static char incomingPayload[RESP_BUF_SIZE];

static GSM_MQTT_IncomingCallback s_incomingCallback = NULL;

/* ---------- Link-activity LED ---------- */

#define LINK_LED_PORT       GPIOC
#define LINK_LED_PIN        GPIO_PIN_13
#define LED_ACTIVE_LOW      1
#define LINK_ACTIVITY_MS    1000U

static uint32_t s_lastMessageTick = 0U;
/* s_lastDistance and DEVICE_PRESENCE_TIMEOUT_MS removed due to warnings/redefinition */
static uint32_t s_device_last_seen[16] = {0};

static uint32_t lastRxTick = 0U;
static uint32_t lastPublishTick = 0U;

static void linkLed_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = LINK_LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(LINK_LED_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(
        LINK_LED_PORT,
        LINK_LED_PIN,
        LED_ACTIVE_LOW ? GPIO_PIN_SET : GPIO_PIN_RESET
    );
}

static void linkLed_Update(void)
{
    GPIO_PinState onState =
        LED_ACTIVE_LOW ? GPIO_PIN_RESET : GPIO_PIN_SET;

    GPIO_PinState offState =
        LED_ACTIVE_LOW ? GPIO_PIN_SET : GPIO_PIN_RESET;

    HAL_GPIO_WritePin(
        LINK_LED_PORT,
        LINK_LED_PIN,
        mqttConnected ? onState : offState
    );
}

/* ---------- USB debug helper ---------- */

void dbg(const char *fmt, ...)
{
    char line[1024];

    va_list args;
    va_start(args, fmt);

    int len = vsnprintf(line, sizeof(line), fmt, args);

    va_end(args);

    if (len <= 0)
        return;

    if (len >= (int)sizeof(line))
        len = (int)sizeof(line) - 1;

    // CDC_Transmit_FS removed for compatibility
    (void)len;
}

static void clearUartErrors(void)
{
    if (hAT == NULL)
        return;

    if (__HAL_UART_GET_FLAG(hAT, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_OREFLAG(hAT);
    }
    if (__HAL_UART_GET_FLAG(hAT, UART_FLAG_FE))
    {
        __HAL_UART_CLEAR_FEFLAG(hAT);
    }
    if (__HAL_UART_GET_FLAG(hAT, UART_FLAG_NE))
    {
        __HAL_UART_CLEAR_NEFLAG(hAT);
    }
    if (__HAL_UART_GET_FLAG(hAT, UART_FLAG_PE))
    {
        __HAL_UART_CLEAR_PEFLAG(hAT);
    }
}

/* ---------- Low-level AT command ---------- */

static uint8_t sendAT_internal(
    const char *cmd,
    uint32_t timeout,
    const char *expected_urc
)
{
    if (hAT == NULL || cmd == NULL)
    {
        dbg("[AT ERR] Cannot send '%s' - UART not initialized!\r\n", cmd ? cmd : "NULL");
        return 0U;
    }

    dbg("[AT >>] %s\r\n", cmd);

    clearUartErrors();

    /* Drain stale bytes from RX FIFO/DR before sending new command */
    while (__HAL_UART_GET_FLAG(hAT, UART_FLAG_RXNE))
    {
        volatile uint32_t dummy = hAT->Instance->DR;
        (void)dummy;
    }

    memset(respBuf, 0, sizeof(respBuf));

    uint16_t total = 0U;

    HAL_UART_Transmit(
        hAT,
        (uint8_t *)cmd,
        (uint16_t)strlen(cmd),
        1000U
    );

    HAL_UART_Transmit(
        hAT,
        (uint8_t *)"\r\n",
        2U,
        200U
    );

    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout)
    {
        uint16_t chunkLen = 0U;

        /* Safe here specifically because this loop cannot run longer
         * than `timeout` (max 60000ms, AT+CMQTTSTART) — the exit
         * condition above is self-contained and doesn't depend on
         * anything external. This is not "proof the modem responded",
         * just "this already-bounded wait hasn't finished yet". */


        clearUartErrors();

        HAL_StatusTypeDef st = HAL_UARTEx_ReceiveToIdle(
            hAT,
            (uint8_t *)respBuf + total,
            (uint16_t)(sizeof(respBuf) - total - 1U),
            &chunkLen,
            100U
        );
        (void)st;

        total = (uint16_t)(total + chunkLen);
        respBuf[total] = '\0';

        if (strstr(respBuf, "ERROR") != NULL)
        {
            break;
        }

        if (expected_urc != NULL)
        {
            /* Check if expected asynchronous URC has arrived */
            if (strstr(respBuf, expected_urc) != NULL)
            {
                break;
            }
        }
        else
        {
            if (strstr(respBuf, "OK") != NULL ||
                strstr(respBuf, ">") != NULL)
            {
                break;
            }
        }

        if (total >= sizeof(respBuf) - 2U)
            break;
    }

    if (total > 0U)
    {
        dbg("[AT <<] %s\r\n", respBuf);
    }
    else
    {
        dbg("[AT <<] (NO RESPONSE / TIMEOUT %lums)\r\n", (unsigned long)timeout);
    }

    if (strstr(respBuf, "ERROR") != NULL)
        return 0U;

    if (expected_urc != NULL)
    {
        return (strstr(respBuf, expected_urc) != NULL) ? 1U : 0U;
    }

    if (strstr(respBuf, "OK") != NULL ||
        strstr(respBuf, ">") != NULL)
        return 1U;

    return 0U;
}

static uint8_t sendAT(const char *cmd, uint32_t timeout)
{
    return sendAT_internal(cmd, timeout, NULL);
}

static uint8_t sendAT_ExpectURC(
    const char *cmd,
    uint32_t timeout,
    const char *expected_urc
)
{
    return sendAT_internal(cmd, timeout, expected_urc);
}

/* ---------- Raw data after a '>' prompt ---------- */

static uint8_t sendRawData(
    const char *data,
    uint16_t len,
    uint32_t timeout
)
{
    if (hAT == NULL || data == NULL)
    {
        dbg("[RAW ERR] Cannot send raw data - UART not initialized!\r\n");
        return 0U;
    }

    dbg("[RAW >>] (%u bytes)\r\n", len);

    clearUartErrors();

    memset(respBuf, 0, sizeof(respBuf));

    HAL_UART_Transmit(
        hAT,
        (uint8_t *)data,
        len,
        2000U
    );

    uint16_t total = 0U;
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout)
    {
        uint16_t chunkLen = 0U;

        /* See the matching comment in sendAT_internal() above — safe
         * because this loop is self-bounded by `timeout`. */


        clearUartErrors();

        HAL_StatusTypeDef st = HAL_UARTEx_ReceiveToIdle(
            hAT,
            (uint8_t *)respBuf + total,
            (uint16_t)(sizeof(respBuf) - total - 1U),
            &chunkLen,
            100U
        );
        (void)st;

        total = (uint16_t)(total + chunkLen);
        respBuf[total] = '\0';

        if (strstr(respBuf, "OK") != NULL ||
            strstr(respBuf, "ERROR") != NULL)
        {
            break;
        }

        if (total >= sizeof(respBuf) - 2U)
            break;
    }

    if (total > 0U)
    {
        dbg("[RAW <<] %s\r\n", respBuf);
    }
    else
    {
        dbg("[RAW <<] (NO RESPONSE / TIMEOUT %lums)\r\n", (unsigned long)timeout);
    }

    return (strstr(respBuf, "OK") != NULL) ? 1U : 0U;
}

/* Public wrappers used by the AWS manager. */
void GSM_SetUART(UART_HandleTypeDef *huartAT)
{
    hAT = huartAT;
}

uint8_t GSM_SendAT(const char *cmd, uint32_t timeout)
{
    return sendAT(cmd, timeout);
}

uint8_t GSM_SendRawData(
    const char *data,
    uint16_t len,
    uint32_t timeout
)
{
    return sendRawData(data, len, timeout);
}

/* ---------- Wait for asynchronous CMQTT result ---------- */

static uint8_t waitForURC(
    const char *marker,
    uint32_t timeout
)
{
    if (marker == NULL || hAT == NULL)
        return 0U;

    /* If marker is already present in respBuf, return success immediately */
    if (strstr(respBuf, marker) != NULL)
    {
        return 1U;
    }

    dbg("[URC WAIT] Waiting for '%s' (timeout %lums)...\r\n", marker, (unsigned long)timeout);

    clearUartErrors();

    uint16_t total = (uint16_t)strlen(respBuf);
    uint32_t start = HAL_GetTick();

    while (strstr(respBuf, marker) == NULL &&
           (HAL_GetTick() - start) < timeout &&
           total < sizeof(respBuf) - 2U)
    {
        uint16_t more = 0U;

        /* Safe for the same reason as sendAT_internal()'s loop above:
         * self-bounded by `timeout` (all call sites use 2000ms here). */


        clearUartErrors();

        HAL_UARTEx_ReceiveToIdle(
            hAT,
            (uint8_t *)respBuf + total,
            (uint16_t)(sizeof(respBuf) - total - 1U),
            &more,
            100U
        );

        if (more > 0U)
        {
            total = (uint16_t)(total + more);
            respBuf[total] = '\0';
        }
    }

    dbg("[URC <<] %s\r\n", respBuf);

    return (strstr(respBuf, marker) != NULL) ? 1U : 0U;
}

/* ---------- Connection state ---------- */

static void markDisconnected(void)
{
    if (mqttConnected)
        dbg("[STATUS] Disconnected. Will reconnect automatically.\r\n");

    mqttConnected = 0U;
    lastRxTick = 0U;
    lastPublishTick = 0U;
    
    /* Turn OFF Blue LED (Active Low on Blackpill) */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

/* ---------- Network ---------- */

static uint8_t waitForNetwork(void)
{
    dbg("[STATUS] Checking network registration...\r\n");
    for (int i = 0; i < 30; i++)
    {
        /* Check LTE Registration */
        sendAT("AT+CEREG?", CMD_TIMEOUT);
        if (strstr(respBuf, "+CEREG: 0,1") != NULL ||
            strstr(respBuf, "+CEREG: 0,5") != NULL ||
            strstr(respBuf, "+CEREG: 1,1") != NULL ||
            strstr(respBuf, "+CEREG: 1,5") != NULL)
        {
            dbg("[STATUS] Network registered! (+CEREG ok)\r\n");
            return 1U;
        }

        /* Check GPRS Registration as fallback */
        sendAT("AT+CGREG?", CMD_TIMEOUT);
        if (strstr(respBuf, "+CGREG: 0,1") != NULL ||
            strstr(respBuf, "+CGREG: 0,5") != NULL ||
            strstr(respBuf, "+CGREG: 1,1") != NULL ||
            strstr(respBuf, "+CGREG: 1,5") != NULL)
        {
            dbg("[STATUS] Network registered! (+CGREG ok)\r\n");
            return 1U;
        }

        HAL_Delay(2000U);
    }

    dbg("[STATUS] Network registration timed out.\r\n");
    return 0U;
}

/* ---------- MQTT connect ---------- */

static uint8_t connectMQTT(void)
{
    static char cmd[300];

    /*
     * Full MQTT service teardown + restart.
     * The A7670C MQTT stack gets stuck in a half-open state after
     * a failed TLS handshake. AT+CMQTTSTOP is the only way to
     * fully reset its internal state machine.
     */
    sendAT("AT+CMQTTDISC=0,120", CMD_TIMEOUT);
    sendAT("AT+CMQTTREL=0", CMD_TIMEOUT);
    sendAT("AT+CMQTTSTOP", CMD_TIMEOUT);
    HAL_Delay(1000);

    /* Start native MQTT service. */
    sendAT("AT+CMQTTSTART", VERY_LONG_TIMEOUT);

    if (strstr(respBuf, "OK") == NULL &&
        strstr(respBuf, "+CMQTTSTART: 0") == NULL)
    {
        /* It may already be running. Continue. */
        dbg("[INFO] CMQTTSTART already active or failed, continuing.\r\n");
    }

    /* Clean old client session. Errors here are harmless. */
    sendAT("AT+CMQTTDISC=0,120", CMD_TIMEOUT);
    sendAT("AT+CMQTTREL=0", CMD_TIMEOUT);

    /* Acquire MQTT client. */
    if (gcfg.use_ssl)
    {
        snprintf(
            cmd,
            sizeof(cmd),
            "AT+CMQTTACCQ=0,\"%s\",1",
            gcfg.client_id
        );
    }
    else
    {
        snprintf(
            cmd,
            sizeof(cmd),
            "AT+CMQTTACCQ=0,\"%s\"",
            gcfg.client_id
        );
    }

    if (!sendAT(cmd, CMD_TIMEOUT))
    {
        dbg("[STATUS] Client acquire failed.\r\n");
        return 0U;
    }

    /* AWS IoT MQTT uses TLS. */
    if (gcfg.use_ssl)
    {
        snprintf(
            cmd,
            sizeof(cmd),
            "AT+CMQTTSSLCFG=0,%u",
            (unsigned)gcfg.ssl_ctx_index
        );

        if (!sendAT(cmd, CMD_TIMEOUT))
        {
            dbg("[STATUS] MQTT SSL context setup failed.\r\n");
            return 0U;
        }

        dbg(
            "[STATUS] MQTT TLS enabled. SSL context: %u\r\n",
            (unsigned)gcfg.ssl_ctx_index
        );
    }

    /* Connect broker. */
    snprintf(
        cmd,
        sizeof(cmd),
        "AT+CMQTTCONNECT=0,\"%s\",60,1",
        gcfg.broker_url
    );

    if (!sendAT_ExpectURC(cmd, LONG_TIMEOUT, "+CMQTTCONNECT:"))
    {
        dbg("[STATUS] MQTT CONNECT command failed.\r\n");
        return 0U;
    }

    waitForURC("+CMQTTCONNECT:", 2000U);

    if (strstr(respBuf, "+CMQTTCONNECT: 0,0") == NULL)
    {
        dbg("[STATUS] Broker connection rejected.\r\n");
        dbg("[STATUS] MQTT response: %s\r\n", respBuf);
        return 0U;
    }

    /*
     * Optional initial subscription.
     * AWS provisioning starts with sub_topic == NULL, so this
     * block is intentionally skipped in that case.
     */
    if (gcfg.sub_topic != NULL &&
        gcfg.sub_topic[0] != '\0')
    {
        uint16_t topicLen =
            (uint16_t)strlen(gcfg.sub_topic);

        snprintf(
            cmd,
            sizeof(cmd),
            "AT+CMQTTSUBTOPIC=0,%u,0",
            (unsigned)topicLen
        );

        if (!sendAT(cmd, CMD_TIMEOUT))
            return 0U;

        if (!sendRawData(
                gcfg.sub_topic,
                topicLen,
                CMD_TIMEOUT))
        {
            return 0U;
        }

        if (!sendAT_ExpectURC("AT+CMQTTSUB=0", URC_WAIT_TIMEOUT, "+CMQTTSUB:"))
            return 0U;

        waitForURC("+CMQTTSUB:", 2000U);

        if (strstr(respBuf, "+CMQTTSUB: 0,0") == NULL)
        {
            dbg("[STATUS] Subscribe failed.\r\n");
            return 0U;
        }

        dbg("[STATUS] Initial topic subscribed.\r\n");
    }

    mqttConnected = 1U;
    lastRxTick = 0U;
    lastPublishTick = 0U;
    
    /* Turn ON Blue LED (Active Low on Blackpill) to indicate connection */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    dbg("[STATUS] Connected. Ready to send/receive.\r\n");

    return 1U;
}

/* ---------- Module/reconnect ---------- */

static uint8_t moduleAlive(void)
{
    return sendAT("AT", 2000U);
}

static uint8_t fullReconnect(void)
{
    sendAT("ATE0", CMD_TIMEOUT);

    if (!waitForNetwork())
        return 0U;

    char cmd[128];

    snprintf(
        cmd,
        sizeof(cmd),
        "AT+CGDCONT=1,\"IP\",\"%s\"",
        gcfg.apn
    );

    if (!sendAT(cmd, CMD_TIMEOUT))
        return 0U;

    return connectMQTT();
}

/* ---------- Public initialization ---------- */

uint8_t GSM_MQTT_Init(
    UART_HandleTypeDef *huartAT,
    const GSM_MQTT_Config *cfg
)
{
    if (huartAT == NULL || cfg == NULL)
        return 0U;

    hAT = huartAT;
    gcfg = *cfg;

    mqttConnected = 0U;
    lastRxTick = 0U;
    lastPublishTick = 0U;

    memset(respBuf, 0, sizeof(respBuf));
    memset(incomingTopic, 0, sizeof(incomingTopic));
    memset(incomingPayload, 0, sizeof(incomingPayload));

    linkLed_Init();

    /* Give USB CDC time to enumerate. */
    HAL_Delay(2000U);



    dbg("\r\n[STATUS] Starting up...\r\n");

    /* Give A7670C time to boot. */
    HAL_Delay(3000U);



    sendAT("AT", CMD_TIMEOUT);
    sendAT("ATE0", CMD_TIMEOUT);
    sendAT("AT+CPIN?", CMD_TIMEOUT);
    sendAT("AT+CSQ", CMD_TIMEOUT);

    dbg("[STATUS] Connecting to network...\r\n");

    if (!waitForNetwork())
    {
        dbg("[STATUS] Network connection FAILED. Check SIM/signal.\r\n");
        return 0U;
    }

    char cmd[128];

    snprintf(
        cmd,
        sizeof(cmd),
        "AT+CGDCONT=1,\"IP\",\"%s\"",
        gcfg.apn
    );

    if (!sendAT(cmd, CMD_TIMEOUT))
    {
        dbg("[STATUS] APN setup FAILED.\r\n");
        return 0U;
    }

    if (!connectMQTT())
    {
        dbg("[STATUS] NOT connected. Check the log above.\r\n");
        return 0U;
    }

    return 1U;
}

/* ---------- Generic MQTT publish ---------- */

uint8_t GSM_MQTT_PublishTopic(
    const char *topic,
    const char *message
)
{
    if (!mqttConnected ||
        topic == NULL ||
        message == NULL)
    {
        return 0U;
    }

    size_t topicSize = strlen(topic);
    size_t messageSize = strlen(message);

    if (topicSize > 65535U || messageSize > 65535U)
        return 0U;

    char cmd[64];

    uint16_t topicLen = (uint16_t)topicSize;
    uint16_t msgLen = (uint16_t)messageSize;

    /* Topic. */
    snprintf(
        cmd,
        sizeof(cmd),
        "AT+CMQTTTOPIC=0,%u",
        (unsigned)topicLen
    );

    if (!sendAT(cmd, CMD_TIMEOUT))
    {
        markDisconnected();
        return 0U;
    }

    if (!sendRawData(topic, topicLen, CMD_TIMEOUT))
    {
        markDisconnected();
        return 0U;
    }

    /* Payload. */
    snprintf(
        cmd,
        sizeof(cmd),
        "AT+CMQTTPAYLOAD=0,%u",
        (unsigned)msgLen
    );

    if (!sendAT(cmd, CMD_TIMEOUT))
    {
        markDisconnected();
        return 0U;
    }

    if (!sendRawData(message, msgLen, CMD_TIMEOUT))
    {
        markDisconnected();
        return 0U;
    }

    /*
     * QoS 0 for low-latency telemetry.
     * A7670C syntax:
     * AT+CMQTTPUB=<client>,<qos>,<timeout>
     */
    if (!sendAT_ExpectURC("AT+CMQTTPUB=0,0,60", URC_WAIT_TIMEOUT, "+CMQTTPUB:"))
    {
        markDisconnected();
        return 0U;
    }

    waitForURC("+CMQTTPUB:", 2000U);

    if (strstr(respBuf, "+CMQTTPUB: 0,0") == NULL)
    {
        markDisconnected();
        return 0U;
    }

    lastPublishTick = HAL_GetTick();

    dbg("[SENT] %s\r\n", message);

    return 1U;
}

uint8_t GSM_MQTT_Publish(const char *message)
{
    if (gcfg.pub_topic == NULL ||
        gcfg.pub_topic[0] == '\0')
    {
        return 0U;
    }

    return GSM_MQTT_PublishTopic(
        gcfg.pub_topic,
        message
    );
}

/* ---------- Dynamic MQTT subscribe ---------- */

uint8_t GSM_MQTT_SubscribeTopic(const char *topic)
{
    if (!mqttConnected ||
        topic == NULL ||
        topic[0] == '\0')
    {
        return 0U;
    }

    size_t topicSize = strlen(topic);

    if (topicSize > 65535U)
        return 0U;

    uint16_t topicLen = (uint16_t)topicSize;
    char cmd[64];

    snprintf(
        cmd,
        sizeof(cmd),
        "AT+CMQTTSUBTOPIC=0,%u,0",
        (unsigned)topicLen
    );

    if (!sendAT(cmd, CMD_TIMEOUT))
    {
        markDisconnected();
        return 0U;
    }

    if (!sendRawData(
            topic,
            topicLen,
            CMD_TIMEOUT))
    {
        markDisconnected();
        return 0U;
    }

    if (!sendAT_ExpectURC("AT+CMQTTSUB=0", URC_WAIT_TIMEOUT, "+CMQTTSUB:"))
    {
        markDisconnected();
        return 0U;
    }

    waitForURC("+CMQTTSUB:", 2000U);

    if (strstr(respBuf, "+CMQTTSUB: 0,0") == NULL)
    {
        dbg("[MQTT] Subscribe rejected: %s\r\n", topic);
        return 0U;
    }

    dbg("[MQTT] Subscribed: %s\r\n", topic);

    return 1U;
}

/* ---------- Incoming MQTT message ---------- */

static void printIncoming(const char *buf)
{
    if (buf == NULL)
        return;

    const char *topicMark =
        strstr(buf, "+CMQTTRXTOPIC:");

    const char *payloadMark =
        strstr(buf, "+CMQTTRXPAYLOAD:");

    if (topicMark == NULL ||
        payloadMark == NULL)
    {
        return;
    }

    const char *topicStart =
        strchr(topicMark, '\n');

    const char *payloadStart =
        strchr(payloadMark, '\n');

    if (topicStart == NULL ||
        payloadStart == NULL)
    {
        return;
    }

    topicStart++;
    payloadStart++;

    memset(incomingTopic, 0, sizeof(incomingTopic));
    memset(incomingPayload, 0, sizeof(incomingPayload));

    /* Topic ends at CR/LF. */
    const char *topicEnd =
        strchr(topicStart, '\r');

    if (topicEnd == NULL)
        topicEnd = strchr(topicStart, '\n');

    size_t topicLen = 0U;

    if (topicEnd != NULL)
        topicLen = (size_t)(topicEnd - topicStart);

    if (topicLen >= sizeof(incomingTopic))
        topicLen = sizeof(incomingTopic) - 1U;

    memcpy(
        incomingTopic,
        topicStart,
        topicLen
    );

    incomingTopic[topicLen] = '\0';

    /* Payload ends at CMQTTRXEND. */
    const char *payloadEnd =
        strstr(payloadStart, "+CMQTTRXEND");

    size_t payloadLen = 0U;

    if (payloadEnd != NULL)
        payloadLen = (size_t)(payloadEnd - payloadStart);

    while (payloadLen > 0U &&
           (payloadStart[payloadLen - 1U] == '\r' ||
            payloadStart[payloadLen - 1U] == '\n'))
    {
        payloadLen--;
    }

    if (payloadLen >= sizeof(incomingPayload))
        payloadLen = sizeof(incomingPayload) - 1U;

    memcpy(
        incomingPayload,
        payloadStart,
        payloadLen
    );

    incomingPayload[payloadLen] = '\0';

    dbg("[RECEIVED TOPIC] %s\r\n", incomingTopic);
    dbg("[RECEIVED PAYLOAD] %s\r\n", incomingPayload);

    /*
     * Critical for AWS Fleet Provisioning:
     * pass the complete topic + JSON payload to aws_manager.c.
     */
    if (s_incomingCallback != NULL)
    {
        s_incomingCallback(
            incomingTopic,
            incomingPayload
        );
    }
}

void GSM_SetIncomingCallback(
    GSM_MQTT_IncomingCallback callback
)
{
    s_incomingCallback = callback;
}

/* ---------- Poll ---------- */

void GSM_MQTT_Poll(void)
{
    linkLed_Update();

    /*
     * If disconnected, periodically check the module and reconnect.
     */
    if (!mqttConnected)
    {
        static uint32_t lastAttempt = 0U;

        if ((HAL_GetTick() - lastAttempt) > 5000U)
        {
            lastAttempt = HAL_GetTick();

            if (!moduleAlive())
            {
                dbg("[STATUS] Waiting for module...\r\n");
            }
            else
            {
                dbg("[STATUS] Module detected, reconnecting...\r\n");

                if (!fullReconnect())
                {
                    dbg(
                        "[STATUS] Reconnect failed, retrying in 5s.\r\n"
                    );
                }
            }
        }

        return;
    }

    /*
     * Periodic local liveness check (every 30 seconds).
     * This detects an actual modem power loss even if the modem cannot
     * send +CMQTTCONNLOST.
     */
    static uint32_t lastLivenessCheck = 0U;

    if ((HAL_GetTick() - lastLivenessCheck) > 30000U)
    {
        lastLivenessCheck = HAL_GetTick();

        if (!moduleAlive())
        {
            markDisconnected();
            return;
        }
    }

    /*
     * Read unsolicited modem data.
     */
    uint16_t chunkLen = 0U;

    clearUartErrors();

    HAL_StatusTypeDef st = HAL_UARTEx_ReceiveToIdle(
        hAT,
        (uint8_t *)respBuf,
        (uint16_t)(sizeof(respBuf) - 1U),
        &chunkLen,
        100U
    );

    (void)st;

    if (chunkLen == 0U)
        return;

    respBuf[chunkLen] = '\0';

    if (strstr(respBuf, "+CMQTTRXSTART") == NULL)
    {
        dbg("[URC] %s\r\n", respBuf);
    }

    if (strstr(respBuf, "+CMQTTRXSTART") != NULL)
    {
        uint16_t total = chunkLen;
        uint32_t start = HAL_GetTick();
        uint32_t lastDataTick = HAL_GetTick();

        while (strstr(respBuf, "+CMQTTRXEND") == NULL &&
               (HAL_GetTick() - start) < 15000U &&
               total < sizeof(respBuf) - 2U)
        {
            uint16_t more = 0U;

            HAL_UARTEx_ReceiveToIdle(
                hAT,
                (uint8_t *)respBuf + total,
                (uint16_t)(sizeof(respBuf) - total - 1U),
                &more,
                500U
            );

            if (more > 0U)
            {
                total = (uint16_t)(total + more);
                respBuf[total] = '\0';
                lastDataTick = HAL_GetTick();
            }
            else
            {
                /* Only break if no new data has been received for 3 seconds */
                if ((HAL_GetTick() - lastDataTick) > 3000U)
                    break;
            }
        }

        printIncoming(respBuf);

        if (strstr(respBuf, "+CMQTTRXEND") != NULL)
            lastRxTick = HAL_GetTick();
    }

    if (strstr(respBuf, "+CMQTTCONNLOST") != NULL)
    {
        markDisconnected();
    }
}

/* ============================================================
 * LEGACY STUBS & AWS INTEGRATION FOR SCREEN_SM
 * ============================================================ */

static uint8_t  s_connected_device_num = 0U;
static uint16_t s_device_distance[16] = {0};
static uint16_t s_latest_distance = 500U;
static bool     s_has_latest_distance = false;

void GSM_BeginConnect(uint8_t selected_device_num)
{
    s_connected_device_num = selected_device_num;
    if (selected_device_num < 16 && s_device_distance[selected_device_num] > 0)
    {
        s_latest_distance = s_device_distance[selected_device_num];
        s_has_latest_distance = true;
    }
}

void GSM_BeginScanning(void)
{
    /* Handled via AWS telemetry topics */
}

void GSM_Disconnect(void)
{
    s_connected_device_num = 0U;
    s_has_latest_distance = false;
}

void GSM_SetLatestDistance(uint8_t device_num, uint16_t distance_m)
{
    if (device_num < 16 && device_num > 0)
    {
        s_device_distance[device_num] = distance_m;
        s_lastMessageTick = HAL_GetTick();

        if (s_connected_device_num == 0 || s_connected_device_num == device_num)
        {
            s_latest_distance = distance_m;
            s_has_latest_distance = true;
        }
    }
}

bool GSM_GetLatestDistance(uint16_t *distance_out)
{
    if (s_has_latest_distance && distance_out != NULL)
    {
        *distance_out = s_latest_distance;
        return true;
    }
    return false;
}

uint32_t GSM_GetMsSinceLastMessage(uint32_t now)
{
    /* Return actual time since last valid message to avoid false-online inferences */
    if (s_lastMessageTick == 0U) return UINT32_MAX;
    if (now >= s_lastMessageTick) {
        return now - s_lastMessageTick;
    } else {
        return (UINT32_MAX - s_lastMessageTick) + now + 1;
    }
}

bool GSM_IsDeviceOnline(uint8_t device_num, uint32_t now)
{
    if (device_num >= 16 || device_num == 0) return false;
    /* Device is online if seen within last DEVICE_PRESENCE_TIMEOUT_MS */
    if (now - s_device_last_seen[device_num] < DEVICE_PRESENCE_TIMEOUT_MS) return true;
    return false;
}

void GSM_MarkDeviceOnline(uint8_t device_num)
{
    if (device_num < 16 && device_num > 0) {
        s_device_last_seen[device_num] = HAL_GetTick();
    }
}

void GSM_LegacyPresenceScan_NoOp(void)
{
    /* Legacy shunting/query protocol has been completely replaced by AWS IoT wildcard 
     * subscriptions to devices/+/status, which passively monitors continuous heartbeats. 
     * This function remains only to fulfill API compatibility without breaking UI code. */
}

void GSM_Reset(void)
{
    /* No-op */
}

/* ---------- Public state ---------- */

uint8_t GSM_MQTT_IsConnected(void)
{
    return mqttConnected;
}

gsm_link_state_t GSM_GetState(void)
{
    if (mqttConnected)
        return GSM_LINK_CONNECTED;

    return GSM_LINK_DISCONNECTED;
}


