//==================== MAPEO DE PINES ====================
// PB2 -> SS (SDA del RC522 / SS del SPI Debugger en simulación)
// PB3 -> MOSI   PB4 <- MISO   PB5 -> SCK
// PB1 -> RST del RC522
// PD0 <- RX / PD1 -> TX (Virtual Terminal, 9600 baudios)
//=========================================================

#define SIMULACION      // comentar para hardware real con RC522

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>

#include "UART.c"
#include "SPI.c"
#include "RC522.c"

//==================== LECTURA DE TARJETA ====================
uint8_t uid_valido(const uint8_t *uid)
{
	uint8_t i, ceros = 0, unos = 0;
	for (i = 0; i < 4; i++) {
		if (uid[i] == 0x00) ceros++;
		if (uid[i] == 0xFF) unos++;
	}
	return (ceros < 4) && (unos < 4);               // 00000000 / FFFFFFFF = sin tarjeta
}

uint8_t leer_tarjeta(uint8_t *uid)
{
	uint8_t buf[16];
	uint8_t i;

	memset(buf, 0, sizeof(buf));
	#ifdef SIMULACION
	// --- UID cargado manualmente en el SPI Debugger (modo Slave) ---
	SS_LOW();
	for (i = 0; i < 4; i++) buf[i] = spi_transfer(0x00);
	SS_HIGH();
	#else
	// --- Lectura real con la librería de la cátedra ---
	mfrc522_standard(buf);
	#endif
	memcpy(uid, buf, 4);
	return uid_valido(uid);
}

//==================== PROGRAMA PRINCIPAL ====================
int main(void)
{
	uint8_t uid[4], ultimo[4];
	uint8_t presente = 0, fallos = 0;

	// ---Inicialización ---
	uart_init(103);                                 // 9600 baudios a 16 MHz
	spi_init();
	#ifdef SIMULACION
	SPSR &= ~(1 << SPI2X);                          // anular el doble de velocidad
	SPCR |= (1 << SPR1) | (1 << SPR0);              // fosc/128 = 125 kHz para el SPI Debugger
	#endif
	#ifndef SIMULACION
	mfrc522_resetPinInit();
	mfrc522_init();
	#endif
	uart_print("Cajero listo\r\n");

	while (1) {
		// ---Lectura de tarjeta cada 200 ms ---
		if (leer_tarjeta(uid)) {
			fallos = 0;
			if (!presente || memcmp(uid, ultimo, 4) != 0) {
				uart_print("UID: ");
				uart_print_hex_array(uid, 4);
				memcpy(ultimo, uid, 4);
			}
			presente = 1;
			} else if (++fallos >= 2) {                 // 2 fallos seguidos = tarjeta retirada
			presente = 0;
		}
		_delay_ms(200);
	}
}