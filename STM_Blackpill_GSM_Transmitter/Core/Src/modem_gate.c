#include "modem_gate.h"
#include "gsm_mqtt.h"

/* "AT" is answered within ~100ms by a booted modem; the wait only runs its
 * full length when the modem is silent (still booting, or unpowered).
 * "AT+CPIN?" may take a little longer while the SIM initializes. */
#define MODEM_GATE_AT_TIMEOUT_MS      600u
#define MODEM_GATE_CPIN_TIMEOUT_MS   1500u

bool ModemGate_SimReady(UART_HandleTypeDef *huart_modem)
{
    GSM_SetUART(huart_modem);

    if (!GSM_SendAT("AT", MODEM_GATE_AT_TIMEOUT_MS)) {
        return false; /* modem not up yet */
    }

    /* "+CPIN: READY ... OK" when a SIM is present and unlocked (also OK
     * while it asks for a PIN); "+CME ERROR: 10" (SIM not inserted) or
     * "+CME ERROR: 14" (SIM busy, still initializing) otherwise — sendAT()
     * turns any "ERROR" or timeout into 0. */
    if (!GSM_SendAT("AT+CPIN?", MODEM_GATE_CPIN_TIMEOUT_MS)) {
        return false;
    }

    return true;
}
