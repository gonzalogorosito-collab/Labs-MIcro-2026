//==================== MAPEO DE PINES ====================
// PB2 -> SS (SDA del RC522 / SS del SPI Debugger en simulación)
// PB3 -> MOSI   PB4 <- MISO   PB5 -> SCK
// PB1 -> RST del RC522
// PD0 <- RX / PD1 -> TX (Virtual Terminal, 9600 baudios)
// PD4..PD7 -> Filas F1..F4 del teclado (salidas)
// PD2, PD3, PB0, PC3 <- Columnas C1..C4 del teclado (entradas con pull-up)
// PC4 <-> SDA / PC5 -> SCL (LCD 16x2 por PCF8574, dirección 0x27)
// PC0 -> LED verde   PC1 -> LED rojo   PC2 -> Buzzer activo
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

#define PCF8574  0x27
#include "twi_lcd.h"

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

//==================== TECLADO 4x4 ====================
const char teclas[4][4] = {
	{ '1', '2', '3', 'A' },
	{ '4', '5', '6', 'B' },
	{ '7', '8', '9', 'C' },
	{ '*', '0', '#', 'D' }
};

void teclado_init(void)
{
	// ---Filas PD4..PD7 como salidas en alto (inactivas) ---
	DDRD  |= 0xF0;
	PORTD |= 0xF0;

	// ---Columnas como entradas con pull-up ---
	DDRD  &= ~((1 << PD2) | (1 << PD3));
	PORTD |=  (1 << PD2) | (1 << PD3);
	DDRB  &= ~(1 << PB0);
	PORTB |=  (1 << PB0);
	DDRC  &= ~(1 << PC3);
	PORTC |=  (1 << PC3);
}

uint8_t leer_columnas(void)
{
	uint8_t c = 0;
	if (!(PIND & (1 << PD2))) c |= 0x01;
	if (!(PIND & (1 << PD3))) c |= 0x02;
	if (!(PINB & (1 << PB0))) c |= 0x04;
	if (!(PINC & (1 << PC3))) c |= 0x08;
	return c;
}

char teclado_escanear(void)
{
	uint8_t f, c, cols;

	for (f = 0; f < 4; f++) {
		PORTD |= 0xF0;
		PORTD &= ~(1 << (PD4 + f));                 // fila f en bajo
		_delay_us(5);
		cols = leer_columnas();
		if (cols) {
			PORTD |= 0xF0;
			for (c = 0; c < 4; c++)
			if (cols & (1 << c)) return teclas[f][c];
		}
	}
	PORTD |= 0xF0;
	return 0;
}

char teclado_leer(void)
{
	static char estable = 0;
	char k = teclado_escanear();

	if (k != estable) {
		_delay_ms(20);                              // antirrebote
		if (teclado_escanear() == k) {
			estable = k;
			return k;                               // devuelve la tecla solo al presionar
		}
	}
	return 0;
}

//==================== LCD ====================
void lcd_linea(uint8_t fila, const char *txt)
{
	uint8_t i = 0;
	twi_lcd_cmd(fila == 0 ? 0x80 : 0xC0);
	while (txt[i] && i < 16) twi_lcd_dwr(txt[i++]);
	while (i++ < 16) twi_lcd_dwr(' ');              // completar con espacios
}

//==================== INDICADORES ====================
void indicadores_init(void)
{
	DDRC  |=  (1 << PC0) | (1 << PC1) | (1 << PC2);
	PORTC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2));
}

void led_verde(uint8_t on) { if (on) PORTC |= (1 << PC0); else PORTC &= ~(1 << PC0); }
void led_rojo(uint8_t on)  { if (on) PORTC |= (1 << PC1); else PORTC &= ~(1 << PC1); }
void buzzer(uint8_t on)    { if (on) PORTC |= (1 << PC2); else PORTC &= ~(1 << PC2); }

//==================== PROGRAMA PRINCIPAL ====================
int main(void)
{
	const char h[] = "0123456789ABCDEF";
	uint8_t uid[4], ultimo[4];
	uint8_t presente = 0, fallos = 0, ciclos = 0, i;
	char tecla;
	char txt[17];

	// ---Inicialización (twi_init primero: pisa DDRC/PORTC) ---
	twi_init();
	uart_init(103);                                 // 9600 baudios a 16 MHz
	spi_init();
	#ifdef SIMULACION
	SPSR &= ~(1 << SPI2X);                          // anular el doble de velocidad
	SPCR |= (1 << SPR1) | (1 << SPR0);              // fosc/128 = 125 kHz para el SPI Debugger
	#else
	mfrc522_resetPinInit();
	mfrc522_init();
	#endif
	teclado_init();
	indicadores_init();
	twi_lcd_init();
	_delay_ms(2);                                   // esperar fin del clear (1,52 ms)
	twi_lcd_cmd(0x0C);                              // display on, cursor off

	lcd_linea(0, "Acerque tarjeta");
	lcd_linea(1, "");
	uart_print("Cajero listo\r\n");

	while (1) {
		// ---Teclado: en cada ciclo de 10 ms ---
		tecla = teclado_leer();
		if (tecla) {
			strcpy(txt, "Tecla: ");
			txt[7] = tecla;
			txt[8] = '\0';
			lcd_linea(0, txt);

			if (tecla == '#') { led_verde(1); _delay_ms(300); led_verde(0); }
			if (tecla == '*') { led_rojo(1);  _delay_ms(300); led_rojo(0);  }
			if (tecla == 'D') { buzzer(1);    _delay_ms(300); buzzer(0);    }
		}

		// ---Tarjeta: cada 200 ms ---
		if (++ciclos >= 20) {
			ciclos = 0;
			if (leer_tarjeta(uid)) {
				fallos = 0;
				if (!presente || memcmp(uid, ultimo, 4) != 0) {
					strcpy(txt, "UID: ");
					for (i = 0; i < 4; i++) {
						txt[5 + 2 * i] = h[uid[i] >> 4];
						txt[6 + 2 * i] = h[uid[i] & 0x0F];
					}
					txt[13] = '\0';
					lcd_linea(1, txt);
					memcpy(ultimo, uid, 4);
				}
				presente = 1;
				} else if (++fallos >= 2) {             // 2 fallos seguidos = tarjeta retirada
				presente = 0;
			}
		}

		_delay_ms(10);
	}
}