//==========================================================
// trama.c - Protocolo de tramas sobre uart_rb
//==========================================================

#include "trama.h"                       // Formato, códigos y prototipos del protocolo
#include "uart_rb.h"                     // Buffers de recepción y transmisión
#include "tiempo.h"                      // millis() para detectar tramas incompletas

#define TRAMA_HEX 8                      // Dígitos hex entre START y STOP (2 cmd + 4 dato + 2 checksum)

//===== Estado del receptor =====
static uint8_t  buf[TRAMA_HEX];          // Dígitos hex recibidos de la trama en curso
static uint8_t  idx = 0;                 // Cantidad de dígitos ya guardados
static uint8_t  recibiendo = 0;          // 1 = se recibió START y falta el STOP
static uint32_t t_ultimo = 0;            // Instante del último carácter de la trama en curso

//==========================================================
// Conversión hexadecimal
//==========================================================
static uint8_t hex_a_nibble(uint8_t c)   // Convierte un carácter hex a su valor (16 = inválido)
{
	if (c >= '0' && c <= '9') return c - '0';        // Dígitos 0 a 9
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;   // Letras mayúsculas A a F
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;   // Letras minúsculas a a f
	return 16;                           // Cualquier otro carácter no es hexadecimal
}                                        // Fin de hex_a_nibble

static uint8_t hex_a_byte(const uint8_t *p)  // Convierte dos caracteres hex a un byte
{
	return (hex_a_nibble(p[0]) << 4) | hex_a_nibble(p[1]);  // Nibble alto desplazado más nibble bajo
}                                        // Fin de hex_a_byte

static uint8_t calcular_checksum(uint8_t cmd, uint16_t dato)  // Checksum del protocolo
{
	return (uint8_t)(cmd + (dato >> 8) + (dato & 0xFF));     // Suma de los tres bytes, módulo 256 (el uint8_t descarta el acarreo)
}                                        // Fin de calcular_checksum

//==========================================================
// Recepción (fuera de las ISR, desde el lazo principal)
//==========================================================
int8_t trama_recibir(trama_t *t)         // Procesa los bytes pendientes y devuelve el resultado
{
	uint8_t c;                           // Byte leído del buffer de recepción
	uint8_t chk;                         // Checksum recibido

	if (recibiendo && (uint32_t)(millis() - t_ultimo) > TRAMA_TIMEOUT_MS) {  // Si la trama quedó cortada demasiado tiempo...
		recibiendo = 0;                  // ...se abandona
		return -ERR_INCOMPLETA;          // ...y se informa como incompleta
	}                                    // Fin del chequeo de timeout

	while (uart_leer(&c)) {              // Mientras haya bytes recibidos
		if (c == TRAMA_START) {          // Llegó un START
			uint8_t cortada = recibiendo;    // Recuerda si había una trama sin terminar
			recibiendo = 1;              // Empieza una trama nueva
			idx = 0;                     // Sin dígitos guardados
			t_ultimo = millis();         // Marca el instante de inicio
			if (cortada) return -ERR_INCOMPLETA;  // La anterior quedó incompleta (la nueva sigue en curso)
			continue;                    // Pasa al siguiente byte
		}                                // Fin del tratamiento de START

		if (!recibiendo) continue;       // Bytes fuera de una trama (por ejemplo \r\n) se ignoran
		t_ultimo = millis();             // Actualiza el instante del último carácter

		if (c == TRAMA_STOP) {           // Llegó el STOP
			recibiendo = 0;              // La trama terminó
			if (idx != TRAMA_HEX) return -ERR_FORMATO;   // Cantidad de dígitos incorrecta
			t->cmd  = hex_a_byte(&buf[0]);               // Dígitos 0-1: comando
			t->dato = ((uint16_t)hex_a_byte(&buf[2]) << 8) | hex_a_byte(&buf[4]);  // Dígitos 2-5: dato de 16 bits
			chk     = hex_a_byte(&buf[6]);               // Dígitos 6-7: checksum recibido
			if (chk != calcular_checksum(t->cmd, t->dato)) return -ERR_CHECKSUM;  // No coincide con el calculado
			return TRAMA_OK;             // Trama válida
		}                                // Fin del tratamiento de STOP

		if (hex_a_nibble(c) > 15 || idx >= TRAMA_HEX) {  // Carácter no hexadecimal o demasiados dígitos
			recibiendo = 0;              // Se descarta la trama
			return -ERR_FORMATO;         // Error de formato
		}                                // Fin del chequeo de formato
		buf[idx++] = c;                  // Guarda el dígito y avanza
	}                                    // Fin de la lectura de bytes

	return TRAMA_NADA;                   // No hay trama completa todavía
}                                        // Fin de trama_recibir

//==========================================================
// Transmisión
//==========================================================
uint8_t trama_enviar(uint8_t cmd, uint16_t dato)  // Arma y encola una trama completa
{
	if (uart_tx_libre() < 10) return 0;  // Sin lugar para los 10 caracteres: no se envía a medias

	uart_send(TRAMA_START);              // '<'
	uart_print_hex(cmd);                 // Comando (2 dígitos)
	uart_print_hex(dato >> 8);           // Dato, byte alto (2 dígitos)
	uart_print_hex(dato & 0xFF);         // Dato, byte bajo (2 dígitos)
	uart_print_hex(calcular_checksum(cmd, dato));  // Checksum (2 dígitos)
	uart_send(TRAMA_STOP);               // '>'
	return 1;                            // Trama encolada
}                                        // Fin de trama_enviar

uint8_t trama_enviar_error(uint16_t codigo)  // Envía una trama de error
{
	return trama_enviar(RESP_ERROR, codigo); // Comando 0xEE con el código como dato
}                                        // Fin de trama_enviar_error