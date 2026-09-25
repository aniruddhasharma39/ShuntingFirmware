/*
 * ============================================================
 * aws_manager.c
 *
 * AWS IoT Fleet Provisioning Manager
 *
 * STM32F411CEU6 + A7670C
 *
 * ============================================================
 */

#include "aws_manager.h"
#include "gsm_mqtt.h"
#include "device_config.h"



#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>

#define printf dbg


/* ============================================================
 * FLASH STORAGE
 * ============================================================ */

#define AWS_FLASH_STORAGE_ADDR    0x08040000U
#define AWS_FLASH_SECTOR          FLASH_SECTOR_6
#define AWS_CREDENTIALS_MAGIC     0x41575343U



typedef struct
{
    uint32_t magic;

    uint16_t version;

    uint16_t cert_len;

    uint16_t key_len;

    uint16_t cert_id_len;

    uint32_t crc;

    char certificate_id[68];

    char cert_pem[2048];

    char priv_key_pem[2048];

} AWS_StoredCredentials_t;


/* ============================================================
 * DEVICE IDENTITY
 * ============================================================ */

static char s_serialNumber[25];

static char s_deviceId[32];

static char s_topicTelemetry[64];

static char s_topicStatus[64];

static char s_topicCommands[64];

static char s_topicConfig[64];


/*
 * Set to 1:
 *   Automatically generate ID from STM32 unique ID.
 *
 * Set to 0:
 *   Use manually defined ID below.
 */
#define AWS_USE_AUTO_DEVICE_ID 0


#if !AWS_USE_AUTO_DEVICE_ID

/* Was previously a second, separately-hardcoded identity pair here
 * ("0003" / "SHN-TX-TEST-01"), completely uncoordinated with
 * device_config.h's own DEVICE_ID ("DVC-10245" as shipped) — the two
 * files disagreed on this unit's identity. device_config.h is now the
 * single place to change per physical unit before each build; nothing
 * else needs editing for identity. */
static const char *MANUAL_DEVICE_ID =
    DEVICE_ID;

static const char *MANUAL_SERIAL_NUMBER =
    DEVICE_SERIAL_NUMBER;

#endif


/* static const char *DEVICE_DESCRIPTION =
    "Transmitter_Test_Device_using_Receiver_Template"; */


static void AWS_RestoreSubscriptions(void);


/* ============================================================
 * AWS ROOT CA
 * ============================================================ */

static const char *AWS_ROOT_CA =
"-----BEGIN CERTIFICATE-----\n"
"MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF\n"
"ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n"
"b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL\n"
"MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv\n"
"b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj\n"
"ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM\n"
"9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw\n"
"IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6\n"
"VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L\n"
"93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm\n"
"jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC\n"
"AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA\n"
"A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI\n"
"U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs\n"
"N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv\n"
"o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU\n"
"5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy\n"
"rqXRfboQnoZsG4q5WTP468SQvvG5\n"
"-----END CERTIFICATE-----\n";


/* ============================================================
 * AWS CLAIM CERTIFICATE
 *
 * KEEP YOUR EXISTING CLAIM CERTIFICATE HERE.
 *
 * Do NOT use the example text below as a real certificate.
 * ============================================================ */

static const char *AWS_CLAIM_CERT =
"-----BEGIN CERTIFICATE-----\n"
"MIIDWTCCAkGgAwIBAgIUQ3c6sW/TyJvc/Ikfxj9/ci0i9l4wDQYJKoZIhvcNAQEL\n"
"BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g\n"
"SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI2MDkxOTEyMzQw\n"
"NVoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0\n"
"ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAODuVcL21l0dHqj+BItS\n"
"TosbaEnbKia3HuGjnUo60l9kUUDDSLDqUUgr6faCrr/pu0eJmb+cbsDgka1NyN9l\n"
"g4CwCe4M+QxJiBiZenp7y8O3iKNv1v3nk3xFcfX7JdLkLhIooIuju9fz1a0GZQIX\n"
"3+lu1yTL4Sq3tvfRft6Ux9snc9LOTdSzPzUSeygJhJ8/6gVokRGcBJ4dH4oiO3W4\n"
"uFSzzkRPSRDXDbEWQkOYcoCIC7JoTWceXMmzHdDKcuqdOYk38yhDIqxtxO81//J1\n"
"eGayBEgPvMGBWovjPJKIGlZ7KPgXtaZ56HUSIf5NHmVhZ6sE3lsZWAw4fOP9YYUX\n"
"1KcCAwEAAaNgMF4wHwYDVR0jBBgwFoAUkfvSpY2hxqlaM9sTJgt/0akHgTEwHQYD\n"
"VR0OBBYEFDf3FpkasTJhKxyT7WA/owBD3CHaMAwGA1UdEwEB/wQCMAAwDgYDVR0P\n"
"AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQCpyu9jWJT2oyeCygNw/dRGAOyE\n"
"lzf2iPOrZe3gnGtZJxNbB1eD1eLPFyp2DMMDI3gfJnr+0wkYyxMYmBo2vg0LWQ8Y\n"
"Roe1j43xNOzlCAZGe4o3EM/u+5rZkrWQlWaESRdXryRHH77wYNyyrokF5lCg86hL\n"
"S8wKdpyRVxDPYSwL0DwEwLv6GoAgAZiq47r8598mfY2FMuu6L5zakSznFirk9RdF\n"
"+2nW49jsKs261zUFD9MJIO9DHY+0WA2MSEL4/7IDDTdfXGLYW7kfnV2jfW/gc1H3\n"
"Mp5iPoCzEv/bwG+oxHkA6C0tE6U7bwIl+BY2wht//3mHu0pZ+UtUdzRqnSqJ\n"
"-----END CERTIFICATE-----\n";


/* ============================================================
 * AWS CLAIM PRIVATE KEY
 *
 * KEEP YOUR EXISTING CLAIM PRIVATE KEY HERE.
 *
 * Do NOT publish this key or commit it to a public repository.
 * ============================================================ */

static const char *AWS_CLAIM_KEY =
"-----BEGIN RSA PRIVATE KEY-----\n"
"MIIEpAIBAAKCAQEA4O5VwvbWXR0eqP4Ei1JOixtoSdsqJrce4aOdSjrSX2RRQMNI\n"
"sOpRSCvp9oKuv+m7R4mZv5xuwOCRrU3I32WDgLAJ7gz5DEmIGJl6envLw7eIo2/W\n"
"/eeTfEVx9fsl0uQuEiigi6O71/PVrQZlAhff6W7XJMvhKre299F+3pTH2ydz0s5N\n"
"1LM/NRJ7KAmEnz/qBWiREZwEnh0fiiI7dbi4VLPORE9JENcNsRZCQ5hygIgLsmhN\n"
"Zx5cybMd0Mpy6p05iTfzKEMirG3E7zX/8nV4ZrIESA+8wYFai+M8kogaVnso+Be1\n"
"pnnodRIh/k0eZWFnqwTeWxlYDDh84/1hhRfUpwIDAQABAoIBADD9eAxourO5He5s\n"
"tQyrNpQxufssEvgvtjgq7J04Ro2kSdYzMmfmASIY/nJEcE84VyPzolsLdUvpHZGD\n"
"eZa+g2/J57/Db5LviQbilryKrgzIsqf2Ofs/Lu5IKdQLiWdvb2FhG+aOGOGmKAnx\n"
"dJqKHKi5DX7kMPye2ukK5fkQqp7ehinzHmo8Q4Kq1FUVYrZFo6ADrYfog95W4m+W\n"
"Vg/Gp11SrBDXJSfrOk0k2GyeIIZwsuDD+U8gLwKJ+SJP0M+rxMZgWAmMmlgIN85U\n"
"N7F08SOeysh5t9Jzo8qIExjSR8X4zscJJWoLrI5nLBxoaxIH176C0z83G6BFKRSa\n"
"pBxLWWECgYEA//1NgD8ybIHVGYR113Iko6lLUJr3IRst6klg+GFlmVgx/gph0lxS\n"
"NtUhRTmmSdycVIhw9O3N550VirOFhzzuFmg4Ta/8cK1buoCOJz47hffz6FglJJyo\n"
"An8IO8ze/yb6t7ZMxdm1agrsywDqnDr7xKWEqnK2FG/3adhdZI6c3ekCgYEA4PC0\n"
"e/4fQ0cNMNuugq1VWWmmR/ezkDUVkwsCOlNDPw3U03/HjL7qBeCZdRp4ysKIAsAZ\n"
"q9m9iL77IMHtvODJowAxpBKliTOSvjxdxpiGFkq7H+6LWw4nP3oCLdgJcNZR6AX/\n"
"8ZJso4nAbg+0Xy9Uo4Rus+zLF0l0F6QTMBiHtA8CgYEAi138s7W0Xh7RRfM3tPQP\n"
"voqM1nWH1h5WAReyE4fzKGk7znMYjs50dCXU1ztrrQrOkbc2yCIv18lN0RWrvNUP\n"
"SmOzQ5hsd2vqZL9YpUTcYzN7NyPGFQi076b5dJU1UvSFyz8UzSKaAgGwqVT2Zdg2\n"
"+fijIakSOE49BQMm1XhPLwECgYEApG3wUU9DRSfUMan02Fuir1i2j5c3cKwNWE7M\n"
"0xv4ZUhUEkMu+Oi6I4+6PcsbD4TCbdhFK8VgcmzBIIICqnZbj0beAsUKss+7bkqs\n"
"djcalYMNZNs4jVg8Qn+Kxj5NMGnKt+Ri4xEZTzasLwhV3tq3cUymq+nlz+EG9x0e\n"
"VaMo1+sCgYB5xmaYHJqOLYoQ0P8QdhV38xgbnTeLEJJwsKsyBrJnsyeJCbHYYe1a\n"
"0X6SvgDAbnU17Ukv0STnXS448ueQ03Ync2nZuZp+BNs1DfzTozE5A2ZOXqDG1hGV\n"
"at0IgXwCfFCQUXYMF1jcPeEScuAECt+kF6u0PXFPEAX3O3HOKtKWMQ==\n"
"-----END RSA PRIVATE KEY-----\n";


/* ============================================================
 * STATE
 * ============================================================ */

static bool s_isProvisioned = false;

static bool s_isRegistered = false;

static uint32_t s_lastTelemetryTick = 0;
static uint32_t s_lastHeartbeatTick = 0;
static bool s_wasConnected = false;


/* ============================================================
 * PROVISIONING RESPONSE DATA
 * ============================================================ */

static volatile bool s_provisioningResponseReceived = false;

static char s_provisioningCert[2048];

static char s_provisioningKey[2048];

static char s_provisioningToken[1024];

static char s_provisioningThingName[64];


/* ============================================================
 * DEVICE IDENTITY INITIALIZATION
 * ============================================================ */

static void InitIdentity(void)
{
#if AWS_USE_AUTO_DEVICE_ID

    uint32_t w0 = HAL_GetUIDw0();

    uint32_t w1 = HAL_GetUIDw1();

    uint32_t w2 = HAL_GetUIDw2();


    snprintf(
        s_serialNumber,
        sizeof(s_serialNumber),
        "%08X%08X%08X",
        (unsigned int)w2,
        (unsigned int)w1,
        (unsigned int)w0
    );


    snprintf(
        s_deviceId,
        sizeof(s_deviceId),
        "shunting-rx-%08X",
        (unsigned int)w0
    );

#else

    strncpy(
        s_serialNumber,
        MANUAL_SERIAL_NUMBER,
        sizeof(s_serialNumber) - 1
    );

    s_serialNumber[sizeof(s_serialNumber) - 1] = '\0';


    strncpy(
        s_deviceId,
        MANUAL_DEVICE_ID,
        sizeof(s_deviceId) - 1
    );

    s_deviceId[sizeof(s_deviceId) - 1] = '\0';

#endif


    snprintf(
        s_topicTelemetry,
        sizeof(s_topicTelemetry),
        "devices/%s/telemetry",
        s_deviceId
    );


    snprintf(
        s_topicStatus,
        sizeof(s_topicStatus),
        "devices/%s/status",
        s_deviceId
    );


    snprintf(
        s_topicCommands,
        sizeof(s_topicCommands),
        "devices/%s/commands",
        s_deviceId
    );


    snprintf(
        s_topicConfig,
        sizeof(s_topicConfig),
        "devices/%s/config",
        s_deviceId
    );
}


/* ============================================================
 * GETTERS
 * ============================================================ */

const char *AWS_GetDeviceId(void)
{
    return s_deviceId;
}


const char *AWS_GetSerialNumber(void)
{
    return s_serialNumber;
}


const char *AWS_GetTopicTelemetry(void)
{
    return s_topicTelemetry;
}


const char *AWS_GetTopicStatus(void)
{
    return s_topicStatus;
}


const char *AWS_GetTopicCommands(void)
{
    return s_topicCommands;
}


const char *AWS_GetTopicConfig(void)
{
    return s_topicConfig;
}


bool AWS_IsProvisioned(void)
{
    return s_isProvisioned;
}


bool AWS_IsRegistered(void)
{
    return s_isRegistered;
}


/* ============================================================
 * CRC32
 * ============================================================ */

static uint32_t CalculateCRC32(
    const uint8_t *data,
    size_t length
)
{
    uint32_t crc = 0xFFFFFFFFU;


    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];


        for (size_t j = 0; j < 8; j++)
        {
            if (crc & 1U)
            {
                crc =
                    (crc >> 1) ^
                    0xEDB88320U;
            }
            else
            {
                crc >>= 1;
            }
        }
    }


    return ~crc;
}


/* ============================================================
 * CHECK STORED CERTIFICATE
 * ============================================================ */

bool AWS_CertStorage_Exists(void)
{
    AWS_StoredCredentials_t *storage =
        (AWS_StoredCredentials_t *)AWS_FLASH_STORAGE_ADDR;


    if (storage->magic != AWS_CREDENTIALS_MAGIC)
    {
        return false;
    }


    uint32_t crc =
        CalculateCRC32(
            (const uint8_t *)storage->certificate_id,
            sizeof(AWS_StoredCredentials_t) -
            offsetof(
                AWS_StoredCredentials_t,
                certificate_id
            )
        );


    return storage->crc == crc;
}


/* ============================================================
 * LOAD STORED CERTIFICATE
 * ============================================================ */

bool AWS_CertStorage_Load(
    char *cert_pem,
    uint16_t max_cert_len,
    char *key_pem,
    uint16_t max_key_len
)
{
    if (!AWS_CertStorage_Exists())
    {
        return false;
    }


    AWS_StoredCredentials_t *storage =
        (AWS_StoredCredentials_t *)AWS_FLASH_STORAGE_ADDR;


    if (storage->cert_len >= max_cert_len)
    {
        return false;
    }


    if (storage->key_len >= max_key_len)
    {
        return false;
    }


    strcpy(
        cert_pem,
        storage->cert_pem
    );


    strcpy(
        key_pem,
        storage->priv_key_pem
    );


    return true;
}


/* ============================================================
 * ERASE CERTIFICATE
 * ============================================================ */

void AWS_CertStorage_Erase(void)
{
    FLASH_EraseInitTypeDef eraseInit;

    uint32_t sectorError;


    eraseInit.TypeErase =
        FLASH_TYPEERASE_SECTORS;

    eraseInit.VoltageRange =
        FLASH_VOLTAGE_RANGE_3;

    eraseInit.Sector =
        AWS_FLASH_SECTOR;

    eraseInit.NbSectors =
        1;


    HAL_FLASH_Unlock();


    HAL_FLASHEx_Erase(
        &eraseInit,
        &sectorError
    );


    HAL_FLASH_Lock();
}


/* ============================================================
 * SAVE CERTIFICATE
 * ============================================================ */

bool AWS_CertStorage_Save(
    const char *cert_pem,
    const char *key_pem
)
{
    if (cert_pem == NULL ||
        key_pem == NULL)
    {
        return false;
    }


    AWS_StoredCredentials_t creds;

    memset(
        &creds,
        0,
        sizeof(creds)
    );


    creds.magic =
        AWS_CREDENTIALS_MAGIC;


    creds.version = 1;


    creds.cert_len =
        (uint16_t)strlen(cert_pem);


    creds.key_len =
        (uint16_t)strlen(key_pem);


    if (creds.cert_len >=
        sizeof(creds.cert_pem))
    {
        return false;
    }


    if (creds.key_len >=
        sizeof(creds.priv_key_pem))
    {
        return false;
    }


    strncpy(
        creds.cert_pem,
        cert_pem,
        sizeof(creds.cert_pem) - 1
    );


    strncpy(
        creds.priv_key_pem,
        key_pem,
        sizeof(creds.priv_key_pem) - 1
    );


    creds.crc =
        CalculateCRC32(
            (const uint8_t *)creds.certificate_id,
            sizeof(AWS_StoredCredentials_t) -
            offsetof(
                AWS_StoredCredentials_t,
                certificate_id
            )
        );


    AWS_CertStorage_Erase();


    HAL_FLASH_Unlock();


    uint32_t *data =
        (uint32_t *)&creds;


    for (
        size_t i = 0;
        i < sizeof(AWS_StoredCredentials_t) / 4;
        i++
    )
    {
        HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_WORD,
            AWS_FLASH_STORAGE_ADDR + i * 4,
            data[i]
        );
    }


    HAL_FLASH_Lock();


    return true;
}


/* ============================================================
 * MODEM CERTIFICATE DOWNLOAD
 * ============================================================ */

static bool DownloadCertToModem(
    const char *filename,
    const char *data
)
{
    if (filename == NULL ||
        data == NULL)
    {
        return false;
    }


    char cmd[128];


    uint16_t len =
        (uint16_t)strlen(data);


    snprintf(
        cmd,
        sizeof(cmd),
        "AT+CCERTDOWN=\"%s\",%u",
        filename,
        len
    );


    if (!GSM_SendAT(
            cmd,
            3000))
    {
        return false;
    }


    if (!GSM_SendRawData(
            data,
            len,
            5000))
    {
        return false;
    }


    return true;
}


/* ============================================================
 * MODEM SSL CONFIGURATION
 * ============================================================ */

static bool ConfigureModemSSL(void)
{
    if (!GSM_SendAT(
            "AT+CSSLCFG=\"sslversion\",0,4",
            3000))
    {
        return false;
    }


    if (!GSM_SendAT(
            "AT+CSSLCFG=\"authmode\",0,2",
            3000))
    {
        return false;
    }


    if (!GSM_SendAT(
            "AT+CSSLCFG=\"cacert\",0,\"root_ca.pem\"",
            3000))
    {
        return false;
    }


    if (!GSM_SendAT(
            "AT+CSSLCFG=\"clientcert\",0,\"clientcert.pem\"",
            3000))
    {
        return false;
    }


    if (!GSM_SendAT(
            "AT+CSSLCFG=\"clientkey\",0,\"clientkey.pem\"",
            3000))
    {
        return false;
    }


    return true;
}


/* ============================================================
 * PROVISIONING CALLBACK
 * ============================================================ */

/* Helper to locate the closing unescaped quote of a JSON string value */
static const char *find_json_val_end(const char *str)
{
    while (*str)
    {
        if (*str == '\\')
        {
            if (*(str + 1) != '\0')
                str += 2;
            else
                str++;
            continue;
        }
        if (*str == '"')
        {
            return str;
        }
        str++;
    }
    return NULL;
}

static void AWS_IncomingCallback(
    const char *topic,
    const char *payload
)
{
    if (topic == NULL ||
        payload == NULL)
    {
        return;
    }

    /* --------------------------------------------------------
     * CREATE CERTIFICATE ACCEPTED
     * -------------------------------------------------------- */

    if (strstr(
            topic,
            "$aws/certificates/create/json/accepted"
        ))
    {
        dbg("[AWS] Provisioning response received. Parsing keys & cert...\r\n");

        /* Reset provisioning buffers */
        s_provisioningCert[0] = '\0';
        s_provisioningKey[0] = '\0';
        s_provisioningToken[0] = '\0';

        /* Certificate PEM */
        const char *cert_start = strstr(payload, "\"certificatePem\":\"");
        if (cert_start)
        {
            cert_start += strlen("\"certificatePem\":\"");
            const char *cert_end = find_json_val_end(cert_start);
            if (cert_end)
            {
                size_t len = (size_t)(cert_end - cert_start);
                if (len < sizeof(s_provisioningCert))
                {
                    strncpy(s_provisioningCert, cert_start, len);
                    s_provisioningCert[len] = '\0';

                    char *dst = s_provisioningCert;
                    const char *src = s_provisioningCert;
                    while (*src)
                    {
                        if (*src == '\\' && *(src + 1) == 'n')
                        {
                            *dst++ = '\n';
                            src += 2;
                        }
                        else
                        {
                            *dst++ = *src++;
                        }
                    }
                    *dst = '\0';
                }
            }
        }

        /* Private Key */
        const char *key_start = strstr(payload, "\"privateKey\":\"");
        if (key_start)
        {
            key_start += strlen("\"privateKey\":\"");
            const char *key_end = find_json_val_end(key_start);
            if (key_end)
            {
                size_t len = (size_t)(key_end - key_start);
                if (len < sizeof(s_provisioningKey))
                {
                    strncpy(s_provisioningKey, key_start, len);
                    s_provisioningKey[len] = '\0';

                    char *dst = s_provisioningKey;
                    const char *src = s_provisioningKey;
                    while (*src)
                    {
                        if (*src == '\\' && *(src + 1) == 'n')
                        {
                            *dst++ = '\n';
                            src += 2;
                        }
                        else
                        {
                            *dst++ = *src++;
                        }
                    }
                    *dst = '\0';
                }
            }
        }

        /* Ownership Token */
        const char *token_start = strstr(payload, "\"certificateOwnershipToken\":\"");
        if (token_start)
        {
            token_start += strlen("\"certificateOwnershipToken\":\"");
            const char *token_end = find_json_val_end(token_start);
            if (token_end)
            {
                size_t len = (size_t)(token_end - token_start);
                if (len < sizeof(s_provisioningToken))
                {
                    strncpy(s_provisioningToken, token_start, len);
                    s_provisioningToken[len] = '\0';
                }
            }
        }

        dbg("[AWS] Extracted cert=%u bytes, key=%u bytes, token=%u bytes\r\n",
            (unsigned)strlen(s_provisioningCert),
            (unsigned)strlen(s_provisioningKey),
            (unsigned)strlen(s_provisioningToken));

        s_provisioningResponseReceived = true;
    }

    /* --------------------------------------------------------
     * CREATE CERTIFICATE REJECTED
     * -------------------------------------------------------- */

    else if (
        strstr(
            topic,
            "$aws/certificates/create/json/rejected"
        )
    )
    {
        dbg("[AWS] Provisioning rejected by AWS IoT!\r\n");
        s_provisioningResponseReceived =
            true;
    }

    /* --------------------------------------------------------
     * PROVISION ACCEPTED
     * -------------------------------------------------------- */

    else if (
        strstr(
            topic,
            "/provision/json/accepted"
        )
    )
    {
        const char *thing_start = strstr(payload, "\"thingName\"");
        if (thing_start)
        {
            thing_start += strlen("\"thingName\"");
            while (*thing_start == ':' || *thing_start == ' ' || *thing_start == '\t' || *thing_start == '\"')
            {
                if (*thing_start == '\"') {
                    thing_start++;
                    break;
                }
                thing_start++;
            }
            const char *thing_end = find_json_val_end(thing_start);
            if (thing_end)
            {
                size_t len = (size_t)(thing_end - thing_start);
                if (len < sizeof(s_provisioningThingName))
                {
                    strncpy(s_provisioningThingName, thing_start, len);
                    s_provisioningThingName[len] = '\0';
                }
            }
        }

        if (strlen(s_provisioningThingName) == 0)
        {
            strncpy(s_provisioningThingName, DEVICE_ID, sizeof(s_provisioningThingName) - 1);
            s_provisioningThingName[sizeof(s_provisioningThingName) - 1] = '\0';
        }

        dbg("[AWS] Thing provisioned: '%s'\r\n", s_provisioningThingName);
        s_provisioningResponseReceived = true;
    }

    /* --------------------------------------------------------
     * PROVISION REJECTED
     * -------------------------------------------------------- */

    else if (
        strstr(
            topic,
            "/provision/json/rejected"
        )
    )
    {
        dbg("[AWS] RegisterThing rejected by AWS IoT!\r\n");
        s_provisioningResponseReceived =
            true;
    }
}


/* ============================================================
 * AWS INITIALIZATION
 * ============================================================ */

uint8_t AWS_Init(
    UART_HandleTypeDef *huart,
    const char *apn
)
{
    if (huart == NULL ||
        apn == NULL)
    {
        return 0;
    }

    /* Assign modem UART handle to gsm_mqtt driver */
    GSM_SetUART(huart);

    /* Verify modem responsiveness and autobaud */
    dbg("[AWS] Checking modem responsiveness on USART1 (PA9/PA10)...\r\n");
    uint8_t modem_ok = 0;
    for (int i = 0; i < 8; i++)
    {
        if (GSM_SendAT("AT", 1000))
        {
            modem_ok = 1;
            break;
        }
        HAL_Delay(300);
    }

    if (!modem_ok)
    {
        dbg("[AWS] ERROR: Modem not responding to AT!\r\n");
        dbg("[AWS] Check wiring: PA9=TX -> A7670C RX, PA10=RX <- A7670C TX, Common GND, and 3.8-4.2V/5V Power.\r\n");
        return 0;
    }

    GSM_SendAT("ATE0", 1000);
    dbg("[AWS] Modem communication OK.\r\n");


    /* --------------------------------------------------------
     * Identity
     * -------------------------------------------------------- */

    InitIdentity();


    GSM_SetIncomingCallback(
        AWS_IncomingCallback
    );


    /* --------------------------------------------------------
     * Check stored credentials
     * -------------------------------------------------------- */

    // NOTE: Do NOT unconditionally erase stored certificates here.
    // AWS_CertStorage_Exists()/AWS_CertStorage_Load() just below already
    // handle first-boot provisioning (no cert on flash yet), and
    // AWS_CertStorage_Erase() is still correctly called later in this
    // file if the broker actually rejects a stored cert (see the
    // "Stored device cert rejected by broker" branch). Erasing here on
    // every boot forced Fleet Provisioning by claim on every power-cycle
    // instead of reusing the already-issued permanent device
    // certificate, which is why RegisterThing kept getting rejected
    // (AWS IoT does not expect the same Thing to re-provision itself
    // over and over) and the device never reached a durable connection.
    bool use_claim = false;


    static char dev_cert[2048];
    memset(dev_cert, 0, sizeof(dev_cert));

    static char dev_key[2048];
    memset(dev_key, 0, sizeof(dev_key));


    if (
        AWS_CertStorage_Exists() &&
        AWS_CertStorage_Load(
            dev_cert,
            sizeof(dev_cert),
            dev_key,
            sizeof(dev_key)
        )
    )
    {
        s_isProvisioned = true;

        printf(
            "[AWS] Stored device certificate found.\r\n"
        );
    }
    else
    {
        s_isProvisioned = false;

        use_claim = true;

        printf(
            "[AWS] No stored certificate. Starting provisioning.\r\n"
        );
    }


    /* --------------------------------------------------------
     * Download Root CA
     * -------------------------------------------------------- */

    if (!DownloadCertToModem(
            "root_ca.pem",
            AWS_ROOT_CA
        ))
    {
        printf(
            "[AWS] Root CA download failed.\r\n"
        );

        return 0;
    }


    /* --------------------------------------------------------
     * Download appropriate client certificate
     * -------------------------------------------------------- */

    if (use_claim)
    {
        if (!DownloadCertToModem(
                "clientcert.pem",
                AWS_CLAIM_CERT
            ))
        {
            printf(
                "[AWS] Claim certificate download failed.\r\n"
            );

            return 0;
        }


        if (!DownloadCertToModem(
                "clientkey.pem",
                AWS_CLAIM_KEY
            ))
        {
            printf(
                "[AWS] Claim private key download failed.\r\n"
            );

            return 0;
        }
    }
    else
    {
        if (!DownloadCertToModem(
                "clientcert.pem",
                dev_cert
            ))
        {
            printf(
                "[AWS] Device certificate download failed.\r\n"
            );

            return 0;
        }


        if (!DownloadCertToModem(
                "clientkey.pem",
                dev_key
            ))
        {
            printf(
                "[AWS] Device private key download failed.\r\n"
            );

            return 0;
        }
    }


    /* --------------------------------------------------------
     * TLS configuration
     * -------------------------------------------------------- */

    if (!ConfigureModemSSL())
    {
        printf(
            "[AWS] TLS configuration failed.\r\n"
        );

        return 0;
    }


    printf(
        "[AWS] TLS configured.\r\n"
    );


    /* --------------------------------------------------------
     * MQTT configuration
     *
     * IMPORTANT:
     *
     * sub_topic = NULL
     *
     * The AWS manager will create the required
     * provisioning subscriptions itself.
     * -------------------------------------------------------- */

    GSM_MQTT_Config cfg;


    memset(
        &cfg,
        0,
        sizeof(cfg)
    );


    cfg.client_id =
        s_deviceId;


    cfg.pub_topic =
        s_topicStatus;


    cfg.sub_topic =
        NULL;


    cfg.broker_url =
        AWS_BROKER_URL;


    cfg.apn =
        apn;


    cfg.use_ssl =
        1;


    cfg.ssl_ctx_index =
        0;


    printf(
        "[AWS] Connecting to AWS IoT....\r\n"
    );


    if (!GSM_MQTT_Init(
            huart,
            &cfg
        ))
    {
        printf(
            "[AWS] MQTT initialization failed.\r\n"
        );

        /*
         * If we were using stored device credentials and the
         * MQTT/TLS connection failed, the certificate is likely
         * invalid (orphaned from a failed provisioning, or the
         * Thing was deleted from AWS IoT console).
         *
         * Erase the stale credentials so the next retry will
         * fall through to claim-cert provisioning.
         */
        if (!use_claim)
        {
            printf(
                "[AWS] Stored device cert rejected by broker. "
                "Erasing stale credentials.\r\n"
            );

            AWS_CertStorage_Erase();
            s_isProvisioned = false;
        }

        return 0;
    }


    printf(
        "[AWS] MQTT connected.\r\n"
    );


    /* ========================================================
     * FLEET PROVISIONING
     * ======================================================== */

    if (use_claim)
    {
        printf(
            "[AWS] Starting Fleet Provisioning...\r\n"
        );


        memset(
            s_provisioningCert,
            0,
            sizeof(s_provisioningCert)
        );


        memset(
            s_provisioningKey,
            0,
            sizeof(s_provisioningKey)
        );


        memset(
            s_provisioningToken,
            0,
            sizeof(s_provisioningToken)
        );


        memset(
            s_provisioningThingName,
            0,
            sizeof(s_provisioningThingName)
        );


        /* ----------------------------------------------------
         * Subscribe to certificate creation responses
         * ---------------------------------------------------- */

        if (!GSM_MQTT_SubscribeTopic(
                "$aws/certificates/create/json/accepted"
            ))
        {
            printf(
                "[AWS] Certificate accepted subscription failed.\r\n"
            );

            return 0;
        }


        if (!GSM_MQTT_SubscribeTopic(
                "$aws/certificates/create/json/rejected"
            ))
        {
            printf(
                "[AWS] Certificate rejected subscription failed.\r\n"
            );

            return 0;
        }


        s_provisioningResponseReceived =
            false;


        /* ----------------------------------------------------
         * Request new certificate
         * ---------------------------------------------------- */

        printf(
            "[AWS] Requesting device certificate...\r\n"
        );


        if (!GSM_MQTT_PublishTopic(
                "$aws/certificates/create/json",
                "{}"
            ))
        {
            printf(
                "[AWS] Certificate request publish failed.\r\n"
            );

            return 0;
        }


        /* ----------------------------------------------------
         * Wait for AWS response
         * ---------------------------------------------------- */

        uint32_t start =
            HAL_GetTick();


        while (
            !s_provisioningResponseReceived &&
            (HAL_GetTick() - start) < TIMEOUT_PROVISIONING_MS
        )
        {
            /* Safe: this loop is self-bounded to TIMEOUT_PROVISIONING_MS above. */

            GSM_MQTT_Poll();
        }


        if (
            !s_provisioningResponseReceived ||
            strlen(s_provisioningCert) == 0 ||
            strlen(s_provisioningKey) == 0 ||
            strlen(s_provisioningToken) == 0
        )
        {
            printf(
                "[AWS] Certificate provisioning response failed (cert=%u, key=%u, token=%u).\r\n",
                (unsigned)strlen(s_provisioningCert),
                (unsigned)strlen(s_provisioningKey),
                (unsigned)strlen(s_provisioningToken)
            );

            return 0;
        }


        printf(
            "[AWS] Device certificate received.\r\n"
        );


        /* ----------------------------------------------------
         * Subscribe to provisioning accepted topic
         * ---------------------------------------------------- */

        char sub_topic[160];


        snprintf(
            sub_topic,
            sizeof(sub_topic),
            "$aws/provisioning-templates/%s/provision/json/accepted",
            AWS_TEMPLATE_NAME
        );


        if (!GSM_MQTT_SubscribeTopic(
                sub_topic
            ))
        {
            printf(
                "[AWS] Provision accepted subscription failed.\r\n"
            );

            return 0;
        }


        /* ----------------------------------------------------
         * Subscribe to provisioning rejected topic
         * ---------------------------------------------------- */

        snprintf(
            sub_topic,
            sizeof(sub_topic),
            "$aws/provisioning-templates/%s/provision/json/rejected",
            AWS_TEMPLATE_NAME
        );


        GSM_MQTT_SubscribeTopic(
            sub_topic
        );


        /* ----------------------------------------------------
         * Build provisioning payload
         * ---------------------------------------------------- */

        static char payload[1400];


        snprintf(
            payload,
            sizeof(payload),
            "{"
            "\"certificateOwnershipToken\":\"%s\","
            "\"parameters\":{"
                "\"DeviceId\":\"%s\","
                "\"DeviceType\":\"%s\","
                "\"SerialNumber\":\"%s\""
            "}"
            "}",
            s_provisioningToken,
            DEVICE_ID,
            DEVICE_TYPE_STR,
            DEVICE_SERIAL_NUMBER
        );


        /* ----------------------------------------------------
         * Provision device
         * ---------------------------------------------------- */

        char pub_topic[160];


        snprintf(
            pub_topic,
            sizeof(pub_topic),
            "$aws/provisioning-templates/%s/provision/json",
            AWS_TEMPLATE_NAME
        );


        s_provisioningResponseReceived =
            false;


        printf(
            "[AWS] Registering device with provisioning template...\r\n"
        );


        if (!GSM_MQTT_PublishTopic(
                pub_topic,
                payload
            ))
        {
            printf(
                "[AWS] Provision publish failed.\r\n"
            );

            return 0;
        }


        /* ----------------------------------------------------
         * Wait for provisioning result
         * ---------------------------------------------------- */

        start =
            HAL_GetTick();


        while (
            !s_provisioningResponseReceived &&
            (HAL_GetTick() - start) < TIMEOUT_PROVISIONING_MS
        )
        {
            /* Safe: this loop is self-bounded to TIMEOUT_PROVISIONING_MS above. */

            GSM_MQTT_Poll();
        }


        if (
            !s_provisioningResponseReceived ||
            strlen(s_provisioningThingName) == 0
        )
        {
            printf(
                "[AWS] Device provisioning failed.\r\n"
            );

            return 0;
        }


        printf(
            "[AWS] Device provisioned successfully.\r\n"
        );


        /* ----------------------------------------------------
         * Save device certificate/key to STM32 flash
         * ---------------------------------------------------- */

        if (!AWS_CertStorage_Save(
                s_provisioningCert,
                s_provisioningKey
            ))
        {
            printf(
                "[AWS] Failed to save credentials to flash.\r\n"
            );

            return 0;
        }


        s_isProvisioned =
            true;


        /* ----------------------------------------------------
         * Disconnect claim MQTT session
         * ---------------------------------------------------- */

        GSM_SendAT(
            "AT+CMQTTDISC=0,120",
            3000
        );


        GSM_SendAT(
            "AT+CMQTTREL=0",
            3000
        );


        /* ----------------------------------------------------
         * Replace claim certificate with device certificate
         * ---------------------------------------------------- */

        if (!DownloadCertToModem(
                "clientcert.pem",
                s_provisioningCert
            ))
        {
            printf(
                "[AWS] Device certificate modem download failed.\r\n"
            );

            return 0;
        }


        if (!DownloadCertToModem(
                "clientkey.pem",
                s_provisioningKey
            ))
        {
            printf(
                "[AWS] Device key modem download failed.\r\n"
            );

            return 0;
        }


        /* ----------------------------------------------------
         * Reconfigure TLS
         * ---------------------------------------------------- */

        if (!ConfigureModemSSL())
        {
            printf(
                "[AWS] Device TLS configuration failed.\r\n"
            );

            return 0;
        }


        /* ----------------------------------------------------
         * Reconnect using device certificate
         * ---------------------------------------------------- */

        if (!GSM_MQTT_Init(
                huart,
                &cfg
            ))
        {
            printf(
                "[AWS] Device MQTT reconnect failed.\r\n"
            );

            return 0;
        }


        printf(
            "[AWS] Connected using device certificate.\r\n"
        );
    }


    /* ========================================================
     * NORMAL PROVISIONED DEVICE
     * ======================================================== */

    if (s_isProvisioned)
    {
        printf(
            "[AWS] Device is provisioned.\r\n"
        );


        /* ----------------------------------------------------
         * Register device
         * ---------------------------------------------------- */

        AWS_RegisterDevice();


        /* ----------------------------------------------------
         * Restore application subscriptions
         * ---------------------------------------------------- */
        AWS_RestoreSubscriptions();
    }


    s_lastTelemetryTick =
        HAL_GetTick();
    s_lastHeartbeatTick = 
        HAL_GetTick();


    return 1;
}


/* ============================================================
 * RESTORE SUBSCRIPTIONS
 * ============================================================ */

static void AWS_RestoreSubscriptions(void)
{
    /* ----------------------------------------------------
     * Subscribe to commands
     * ---------------------------------------------------- */
    if (!GSM_MQTT_SubscribeTopic(s_topicCommands))
    {
        printf("[AWS] Command subscription failed.\r\n");
    }

    /* ----------------------------------------------------
     * Subscribe to configuration
     * ---------------------------------------------------- */
    if (!GSM_MQTT_SubscribeTopic(s_topicConfig))
    {
        printf("[AWS] Config subscription failed.\r\n");
    }
}

/* ============================================================
 * PUBLISH HEARTBEAT
 * ============================================================ */

static void AWS_PublishHeartbeat(void)
{
    char payload[128];
    snprintf(
        payload,
        sizeof(payload),
        "{\"deviceId\":\"%s\",\"status\":\"ONLINE\"}",
        s_deviceId
    );

    if (GSM_MQTT_PublishTopic(s_topicStatus, payload))
    {
        printf("[STATUS] Heartbeat published.\r\n");
    }
}


uint8_t AWS_RegisterDevice(void)
{
    static char reg_payload[1400];
    snprintf(
        reg_payload,
        sizeof(reg_payload),
        "{"
        "\"device\":{"
            "\"deviceId\":\"%s\","
            "\"deviceName\":\"%s\","
            "\"serialNumber\":\"%s\","
            "\"productType\":\"%s\","
            "\"hardwareVersion\":\"%s\","
            "\"firmwareVersion\":\"%s\","
            "\"manufacturingDate\":\"%s\","
            "\"sensors\":["
                "{"
                    "\"sensorId\":\"S_LIDAR_01\","
                    "\"sensorName\":\"TF02-Pro LiDAR\","
                    "\"sensorType\":\"Distance Sensor\","
                    "\"purpose\":\"Measures approach distance\","
                    "\"manufacturer\":\"Benewake\","
                    "\"parameters\":["
                        "{\"parameterId\":\"DIST\",\"parameterName\":\"distance\",\"displayName\":\"Distance\",\"unit\":\"cm\",\"dataType\":\"integer\",\"minValue\":10,\"maxValue\":4000}"
                    "],"
                    "\"health\":{\"status\":\"ONLINE\",\"lastReadingTimestamp\":null,\"errorCode\":null}"
                "},"
                "{"
                    "\"sensorId\":\"S_BATT_01\","
                    "\"sensorName\":\"Battery Fuel Gauge\","
                    "\"sensorType\":\"Power Monitor\","
                    "\"purpose\":\"Tracks internal battery\","
                    "\"manufacturer\":\"TI\","
                    "\"parameters\":["
                        "{\"parameterId\":\"BATT_PCT\",\"parameterName\":\"battery_level\",\"displayName\":\"Battery Percentage\",\"unit\":\"%%\",\"dataType\":\"integer\",\"minValue\":0,\"maxValue\":100}"
                    "],"
                    "\"health\":{\"status\":\"ONLINE\",\"lastReadingTimestamp\":null,\"errorCode\":null}"
                "}"
            "]"
        "}"
        "}",
        DEVICE_ID, DEVICE_NAME, DEVICE_SERIAL_NUMBER, DEVICE_TYPE_STR,
        DEVICE_HW_VERSION, DEVICE_FW_VERSION, DEVICE_MFG_DATE
    );
    char reg_topic[128];
    snprintf(reg_topic, sizeof(reg_topic), "devices/%s/register", DEVICE_ID);
    GSM_MQTT_PublishTopic(reg_topic, reg_payload);

    char cfg_topic[128];
    snprintf(cfg_topic, sizeof(cfg_topic), "devices/%s/config", DEVICE_ID);
    return GSM_MQTT_PublishTopic(cfg_topic, reg_payload);
}


/* ============================================================
 * TELEMETRY
 * ============================================================ */

uint8_t AWS_PublishTelemetry(
    uint16_t distance_cm,
    uint8_t selected_target_id,
    uint8_t battery_pct,
    bool is_charging,
    int8_t gsm_rssi,
    const char *link_state
)
{
    char telem_payload[512];
    snprintf(
        telem_payload,
        sizeof(telem_payload),
        "{"
        "\"deviceId\":\"%s\","
        "\"productType\":\"%s\","
        "\"uptime_s\":%lu,"
        "\"readings\":{"
            "\"distance_cm\":%u,"
            "\"selected_target_id\":%u"
        "},"
        "\"diagnostics\":{"
            "\"battery_pct\":%u,"
            "\"is_charging\":%s,"
            "\"gsm_rssi\":%d,"
            "\"link_state\":\"%s\","
            "\"status\":\"ONLINE\""
        "}"
        "}",
        DEVICE_ID,
        DEVICE_TYPE_STR,
        (unsigned long)(HAL_GetTick() / 1000U),
        distance_cm,
        selected_target_id,
        battery_pct,
        is_charging ? "true" : "false",
        gsm_rssi,
        link_state ? link_state : "CONNECTED"
    );
    char telem_topic[128];
    snprintf(telem_topic, sizeof(telem_topic), "devices/%s/telemetry", DEVICE_ID);
    return GSM_MQTT_PublishTopic(telem_topic, telem_payload);
}


/* ============================================================
 * PERIODIC AWS MANAGER
 * ============================================================ */

void AWS_Manager_Tick(
    uint32_t now_ms
)
{
    if (!s_isProvisioned)
    {
        return;
    }


    bool isConnected = GSM_MQTT_IsConnected();

    if (isConnected && !s_wasConnected)
    {
        printf("[AWS] MQTT reconnected - restoring subscriptions\r\n");
        AWS_RestoreSubscriptions();
        
        AWS_PublishHeartbeat();
        s_lastHeartbeatTick = now_ms;
    }
    s_wasConnected = isConnected;


    if (!isConnected)
    {
        return;
    }


    /* --------------------------------------------------------
     * Heartbeat every STATUS_HEARTBEAT_INTERVAL_MS
     * -------------------------------------------------------- */

    if (
        now_ms -
        s_lastHeartbeatTick >=
        STATUS_HEARTBEAT_INTERVAL_MS
    )
    {
        s_lastHeartbeatTick =
            now_ms;

        AWS_PublishHeartbeat();
    }
}


