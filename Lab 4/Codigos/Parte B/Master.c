//===============================================================
// Problema B - Maestro: decisiones, sensores e interfaz
//===============================================================
#define F_CPU 16000000UL
#define PCF8574 0x27

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "twi_lcd.h"

//===== Mapeo de pines =====
// PB2 SS   -> SS esclavo
// PB3 MOSI -> MOSI esclavo
// PB4 MISO <- MISO esclavo
// PB5 SCK  -> SCK esclavo
// PC4 SDA / PC5 SCL -> LCD I2C (PCF8574)
// PD3 Boton Start/Stop (pull-up interno, activo en bajo)
// PC2 ADC2 potenciometro (commit 2)
// PC3 ADC3 LDR (commit 3)
// PD2 DHT11 (commit 4)

//===== Protocolo SPI: [CMD][VALOR][respuesta] =====
#define CMD_START   0x01
#define CMD_STOP    0x02
#define CMD_PISTON  0x03
#define CMD_VEL     0x04
#define CMD_EMERG   0x05
#define RESP_ACK    0x06
#define RESP_NACK   0x15
#define REINTENTOS  3

typedef enum { PARADO, MARCHA, EMERGENCIA } estado_t;

volatile uint16_t ms = 0;

estado_t estado = PARADO;
uint8_t ultimo_cmd = 0;
uint8_t ultimo_resp = 0;
uint8_t actualizar_lcd = 1;

//===== Base de tiempo: Timer0 CTC 1 ms =====
ISR(TIMER0_COMPA_vect)
{
	ms++;
}

void timer0_init(void)
{
	TCCR0A = (1 << WGM01);
	OCR0A = 249;                              // 16 MHz / 64 / 250 = 1 kHz
	TCCR0B = (1 << CS01) | (1 << CS00);
	TIMSK0 = (1 << OCIE0A);
}

uint16_t millis(void)
{
	uint16_t t;
	cli();
	t = ms;
	sei();
	return t;
}

//===== SPI maestro =====
void spi_init_maestro(void)
{
	DDRB |= (1 << PB2) | (1 << PB3) | (1 << PB5);
	DDRB &= ~(1 << PB4);
	PORTB |= (1 << PB2) | (1 << PB4);         // SS en alto, pull-up en MISO
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);   // modo 0, fosc/16 = 1 MHz
}

uint8_t spi_transferir(uint8_t dato)
{
	SPDR = dato;
	while (!(SPSR & (1 << SPIF)));
	return SPDR;
}

uint8_t spi_enviar_trama(uint8_t cmd, uint8_t valor)
{
	uint8_t resp;
	PORTB &= ~(1 << PB2);
	_delay_us(20);
	spi_transferir(cmd);
	_delay_us(20);
	spi_transferir(valor);
	_delay_us(20);
	resp = spi_transferir(0x00);
	PORTB |= (1 << PB2);
	return resp;
}

uint8_t enviar_comando(uint8_t cmd, uint8_t valor)
{
	uint8_t i;
	uint8_t resp = RESP_NACK;
	for (i = 0; i < REINTENTOS; i++)
	{
		resp = spi_enviar_trama(cmd, valor);
		if (resp == RESP_ACK) break;
		_delay_ms(2);
	}
	ultimo_cmd = cmd;
	ultimo_resp = resp;
	actualizar_lcd = 1;
	return (resp == RESP_ACK);
}

//===== Boton con antirrebote (llamar cada 10 ms) =====
uint8_t leer_boton(void)
{
	static uint8_t ultimo = 1, estable = 1, cuenta = 0;
	uint8_t lectura = (PIND >> PD3) & 1;

	if (lectura != ultimo)
	{
		ultimo = lectura;
		cuenta = 0;
		return 0;
	}
	if (cuenta < 3)
	{
		cuenta++;
		if (cuenta == 3 && lectura != estable)
		{
			estable = lectura;
			if (estable == 0) return 1;       // flanco de presion confirmado
		}
	}
	return 0;
}

//===== LCD =====
const char *nombre_cmd(uint8_t c)
{
	switch (c)
	{
		case CMD_START:  return "START";
		case CMD_STOP:   return "STOP";
		case CMD_PISTON: return "PISTON";
		case CMD_VEL:    return "VEL";
		case CMD_EMERG:  return "EMERG";
		default:         return "------";
	}
}

const char *nombre_estado(estado_t e)
{
	switch (e)
	{
		case MARCHA:     return "MAR";
		case EMERGENCIA: return "EMR";
		default:         return "PAR";
	}
}

void lcd_refrescar(void)
{
	char linea[17];
	const char *resp = "";

	if (ultimo_cmd != 0) resp = (ultimo_resp == RESP_ACK) ? "ACK" : "NACK";

	twi_lcd_cmd(0x80);
	snprintf(linea, sizeof(linea), "%-16s", "T:-- V:--- P:000");
	twi_lcd_msg(linea);

	twi_lcd_cmd(0xC0);
	snprintf(linea, sizeof(linea), "%s %-6s %-5s", nombre_estado(estado), nombre_cmd(ultimo_cmd), resp);
	twi_lcd_msg(linea);
}

//===== Programa principal =====
int main(void)
{
	uint16_t ahora;
	uint16_t t_boton = 0;

	DDRD &= ~(1 << PD3);
	PORTD |= (1 << PD3);

	spi_init_maestro();
	timer0_init();
	twi_init();
	twi_lcd_init();
	twi_lcd_cmd(0x0C);                        // cursor apagado
	twi_lcd_clear();
	_delay_ms(2);
	sei();

	enviar_comando(CMD_STOP, 0);              // estado conocido al arrancar

	while (1)
	{
		ahora = millis();

		// --- (1) Boton Start/Stop y transiciones de estado ---
		if ((uint16_t)(ahora - t_boton) >= 10)
		{
			t_boton = ahora;
			if (leer_boton())
			{
				switch (estado)
				{
					case PARADO:
					if (enviar_comando(CMD_START, 0)) estado = MARCHA;
					break;
					case MARCHA:
					if (enviar_comando(CMD_STOP, 0)) estado = PARADO;
					break;
					case EMERGENCIA:
					break;
				}
				actualizar_lcd = 1;
			}
		}

		// --- (2) Refresco del LCD solo ante cambios ---
		if (actualizar_lcd)
		{
			actualizar_lcd = 0;
			lcd_refrescar();
		}
	}
}