#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

// ===== TRAMA: [0xAA][CMD][D1][D2][CHK], CHK = CMD ^ D1 ^ D2 =====
#define PROTO_INICIO  0xAA

#define CMD_MOVER     0x01    // D1 = v + 100, D2 = w + 100 (0..200)
#define CMD_PINZA     0x02    // D1: 0 = abrir, 1 = cerrar; D2 = velocidad 0..100 (se manda mientras se mantiene el botón)
#define CMD_MODO      0x03    // D1: 0 = lento, 1 = rápido (no se usa: la velocidad la da el deslizador)
#define CMD_EMERG     0x04    // D1: 1 = activar, 0 = rearmar (extensión propia de la tabla)
#define CMD_TELEM     0x10    // robot -> app: D1 = batería %, D2 = estado (bits)
#define EST_EMERG     0x01    // bit 0: emergencia enclavada
#define EST_IR        0x02    // bit 1: IR detecta borde
#define EST_PINZA     0x04    // bit 2: pinza en movimiento
typedef struct
{
	uint8_t cmd;
	uint8_t d1;
	uint8_t d2;
} trama_t;

// ===== FUNCIONES =====
uint8_t  proto_poll(trama_t *t);   // procesa los bytes del buffer; 1 si completó una trama válida
uint16_t proto_errores(void);      // tramas descartadas por checksum
void     proto_send(uint8_t cmd, uint8_t d1, uint8_t d2);   // envía [0xAA][CMD][D1][D2][CHK]

#endif