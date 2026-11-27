#include "protocol.h"
#include "UART.h"

// ===== ESTADOS DEL PARSER =====
typedef enum { ESP_INICIO, ESP_CMD, ESP_D1, ESP_D2, ESP_CHK } estado_t;

static estado_t estado  = ESP_INICIO;
static trama_t  rx;
static uint16_t errores = 0;

static uint8_t cmd_valido(uint8_t c)
{
	return (c == CMD_MOVER) || (c == CMD_PINZA) || (c == CMD_MODO) || (c == CMD_EMERG);
}

uint8_t proto_poll(trama_t *t)
{
    while (uart_available())
    {
        uint8_t b = uart_read();

        switch (estado)
        {
            case ESP_INICIO:                         // --- (1) busca el byte de inicio ---
                if (b == PROTO_INICIO) estado = ESP_CMD;
                break;

            case ESP_CMD:                            // --- (2) comando ---
                if (cmd_valido(b)) { rx.cmd = b; estado = ESP_D1; }
                else if (b != PROTO_INICIO) estado = ESP_INICIO;   // otro 0xAA: puede ser un inicio nuevo
                break;

            case ESP_D1:                             // --- (3) primer dato ---
                rx.d1  = b;
                estado = ESP_D2;
                break;

            case ESP_D2:                             // --- (4) segundo dato ---
                rx.d2  = b;
                estado = ESP_CHK;
                break;

            case ESP_CHK:                            // --- (5) verifica checksum ---
                estado = ESP_INICIO;
                if (b == (rx.cmd ^ rx.d1 ^ rx.d2))
                {
                    *t = rx;
                    return 1;                        // devuelve de a una trama por llamada
                }
                errores++;
                break;
        }
    }
    return 0;
}

uint16_t proto_errores(void)
{
    return errores;
}

void proto_send(uint8_t cmd, uint8_t d1, uint8_t d2) {
	uart_send(PROTO_INICIO);
	uart_send(cmd);
	uart_send(d1);
	uart_send(d2);
	uart_send(cmd ^ d1 ^ d2);
}