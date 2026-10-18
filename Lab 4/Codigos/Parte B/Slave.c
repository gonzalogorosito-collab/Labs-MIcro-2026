//===============================================================
// Problema B - Esclavo: ejecucion de ordenes sobre actuadores
//===============================================================
#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>

//===== Mapeo de pines =====
// PB2 SS   <- SS maestro
// PB3 MOSI <- MOSI maestro
// PB4 MISO -> MISO maestro
// PB5 SCK  <- SCK maestro
// PD4 LED verde (marcha)
// PD5 LED rojo (falla / emergencia)
// PD6 OC0A -> ENA L298N (PWM banda)
// PD7      -> IN1 L298N
// PB0      -> IN2 L298N
// PB1 OC1A -> servo SG90 (commit 3)
// PD3 buzzer activo (commit 4)

//===== Protocolo SPI: [CMD][VALOR][respuesta] =====
#define CMD_START   0x01
#define CMD_STOP    0x02
#define CMD_PISTON  0x03
#define CMD_VEL     0x04
#define CMD_EMERG   0x05
#define RESP_ACK    0x06
#define RESP_NACK   0x15

#define PWM_MIN     80                        // duty minimo para que arranque el motor

volatile uint8_t indice = 0;
volatile uint8_t cmd_rx, valor_rx;
volatile uint8_t trama_lista = 0;
volatile uint8_t cmd_nuevo, valor_nuevo;

uint8_t en_marcha = 0;
uint8_t velocidad = 0;

uint8_t trama_valida(uint8_t c, uint8_t v)
{
	if (c < CMD_START || c > CMD_EMERG) return 0;
	if (c == CMD_VEL && v > 100) return 0;
	return 1;
}

//===== SPI esclavo =====
void spi_init_esclavo(void)
{
	DDRB |= (1 << PB4);
	DDRB &= ~((1 << PB2) | (1 << PB3) | (1 << PB5));
	SPCR = (1 << SPE) | (1 << SPIE);          // esclavo, modo 0, con interrupcion
	SPDR = 0x00;
}

ISR(SPI_STC_vect)
{
	uint8_t dato = SPDR;

	switch (indice)
	{
		case 0:                               // byte 1: comando
		cmd_rx = dato;
		SPDR = 0x00;
		indice = 1;
		break;
		case 1:                               // byte 2: valor -> preparar respuesta
		valor_rx = dato;
		if (trama_valida(cmd_rx, valor_rx))
		{
			SPDR = RESP_ACK;
			cmd_nuevo = cmd_rx;
			valor_nuevo = valor_rx;
			trama_lista = 1;
		}
		else
		{
			SPDR = RESP_NACK;
		}
		indice = 2;
		break;
		default:                              // byte 3: respuesta ya enviada
		SPDR = 0x00;
		indice = 0;
		break;
	}
}

//===== Motor de la banda: Timer0 Fast PWM en OC0A (~976 Hz) =====
void motor_init(void)
{
	DDRD |= (1 << PD6) | (1 << PD7);
	DDRB |= (1 << PB0);
	PORTD |= (1 << PD7);                      // sentido fijo: IN1 = 1, IN2 = 0
	PORTB &= ~(1 << PB0);
	PORTD &= ~(1 << PD6);
	TCCR0A = (1 << WGM01) | (1 << WGM00);
	TCCR0B = (1 << CS01) | (1 << CS00);       // 16 MHz / 64 / 256
}

void motor_aplicar(void)
{
	if (!en_marcha || velocidad == 0)
	{
		TCCR0A &= ~(1 << COM0A1);             // OC0A desconectado
		PORTD &= ~(1 << PD6);
	}
	else
	{
		OCR0A = PWM_MIN + (uint8_t)(((uint16_t)velocidad * (255 - PWM_MIN)) / 100);
		TCCR0A |= (1 << COM0A1);
	}
}

//===== Ejecucion de comandos =====
void ejecutar(uint8_t c, uint8_t v)
{
	switch (c)
	{
		case CMD_START:
		en_marcha = 1;
		PORTD |= (1 << PD4);
		PORTD &= ~(1 << PD5);
		motor_aplicar();
		break;
		case CMD_STOP:
		en_marcha = 0;
		PORTD &= ~((1 << PD4) | (1 << PD5));
		motor_aplicar();
		break;
		case CMD_VEL:
		velocidad = v;
		motor_aplicar();
		break;
		default:
		break;
	}
}

//===== Programa principal =====
int main(void)
{
	uint8_t c, v;

	DDRD |= (1 << PD4) | (1 << PD5);
	PORTD &= ~((1 << PD4) | (1 << PD5));

	motor_init();
	spi_init_esclavo();
	sei();

	while (1)
	{
		// --- (1) Resincronizacion: con SS en alto la trama arranca de cero ---
		cli();
		if ((PINB & (1 << PB2)) && indice) indice = 0;
		sei();

		// --- (2) Ejecucion de la trama recibida ---
		if (trama_lista)
		{
			cli();
			c = cmd_nuevo;
			v = valor_nuevo;
			trama_lista = 0;
			sei();
			ejecutar(c, v);
		}
	}
}