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

//#define SIMULACION      // comentar para hardware real con RC522

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <stdint.h>
#include <stdlib.h>
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

	memset(buf, 0, sizeof(buf));
	#ifdef SIMULACION
	// --- UID cargado manualmente en el SPI Debugger (modo Slave) ---
	uint8_t trama[8], i, j;

	SS_LOW();
	for (i = 0; i < 8; i++) trama[i] = spi_transfer(0x00);
	SS_HIGH();

	// El esclavo puede entregar los datos con retraso: el UID empieza en el primer byte != 0xFF
	for (i = 0; i <= 4 && trama[i] == 0xFF; i++);
	if (i > 4) return 0;                            // sin datos en la ventana
	for (j = 0; j < 4; j++) {
		if (trama[i + j] == 0xFF) return 0;         // UID cortado entre dos lecturas
		buf[j] = trama[i + j];
	}
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
	
//==================== EEPROM ====================
#define EE_MAGIC_DIR    0x00
#define EE_MAGIC        0xA5
#define EE_CUENTAS_DIR  0x10
#define N_CUENTAS       3
#define SIN_CUENTA      0xFF

typedef struct {
	uint8_t  uid[4];
	char     pin[4];
	uint32_t saldo;
	uint8_t  bloqueada;
} cuenta_t;

const cuenta_t cuentas_defecto[N_CUENTAS] = {
	#ifdef SIMULACION
	{ { 0xA1, 0xB2, 0xC3, 0xD4 }, { '1', '2', '3', '4' }, 1000, 0 },
	{ { 0x11, 0x22, 0x33, 0x44 }, { '5', '6', '7', '8' }, 2500, 0 },
	{ { 0x5A, 0x6B, 0x7C, 0x8D }, { '0', '0', '0', '0' },  500, 0 }
	#else
	{ { 0xC3, 0x7A, 0x44, 0x29 }, { '1', '2', '3', '4' }, 1000, 0 },
	{ { 0xC9, 0xC2, 0x5E, 0x12 }, { '5', '6', '7', '8' }, 2500, 0 },
	{ { 0x96, 0x15, 0xE3, 0xB8 }, { '0', '0', '0', '0' },  500, 0 }
	#endif
};

#define EE_DIR(i)  ((void *)(EE_CUENTAS_DIR + (i) * sizeof(cuenta_t)))

void eeprom_cargar_defecto(void)
{
	uint8_t i;
	for (i = 0; i < N_CUENTAS; i++)
	eeprom_update_block(&cuentas_defecto[i], EE_DIR(i), sizeof(cuenta_t));
	eeprom_update_byte((uint8_t *)EE_MAGIC_DIR, EE_MAGIC);   // último: confirma carga completa
}

void eeprom_init(uint8_t forzar)
{
	if (forzar || eeprom_read_byte((uint8_t *)EE_MAGIC_DIR) != EE_MAGIC)
	eeprom_cargar_defecto();
}

void cuenta_leer(uint8_t i, cuenta_t *c)
{
	eeprom_read_block(c, EE_DIR(i), sizeof(cuenta_t));
}

void cuenta_guardar(uint8_t i, const cuenta_t *c)
{
	eeprom_update_block(c, EE_DIR(i), sizeof(cuenta_t));
}

uint8_t cuenta_buscar(const uint8_t *uid)
{
	cuenta_t c;
	uint8_t i;
	for (i = 0; i < N_CUENTAS; i++) {
		cuenta_leer(i, &c);
		if (memcmp(c.uid, uid, 4) == 0) return i;
	}
	return SIN_CUENTA;
}
	
void u32_a_texto(uint32_t v, char *s)
{
	char tmp[11];
	uint8_t i = 0;

	do {
		tmp[i++] = '0' + (v % 10);                  // dígitos de menor a mayor peso
		v /= 10;
	} while (v);
	while (i) *s++ = tmp[--i];                      // invertir el orden
	*s = '\0';
}

//==================== MÁQUINA DE ESTADOS ====================
typedef enum { E_ESPERA, E_PIN, E_MENU, E_SALDO, E_MONTO, E_MENSAJE } estado_t;

#define TICK_MS         10
#define T_MENSAJE       (2000 / TICK_MS)            // 2 s
#define T_INACTIVIDAD   (30000 / TICK_MS)           // 30 s
#define MAX_INTENTOS    3
#define MAX_DIG_MONTO   5

#define IND_NINGUNO     0x00
#define IND_VERDE       0x01
#define IND_ROJO        0x02
#define IND_BUZZER      0x04

estado_t estado, estado_sig;
uint8_t  cuenta = SIN_CUENTA;
cuenta_t c;
char     entrada[6];
uint8_t  n_entrada;
uint8_t  intentos;
char     operacion;                                 // 'B' = retiro, 'C' = depósito
uint16_t t_mensaje, t_inactivo;

void indicadores_off(void)
{
	led_verde(0);
	led_rojo(0);
	buzzer(0);
}

void pantalla(void)
{
	char txt[17], num[11];
	uint8_t i;

	switch (estado) {
		case E_ESPERA:
		lcd_linea(0, "Bienvenido");
		lcd_linea(1, "Acerque tarjeta");
		break;
		case E_PIN:
		lcd_linea(0, "Ingrese PIN:");
		for (i = 0; i < n_entrada; i++) txt[i] = '*';
		txt[i] = '\0';
		lcd_linea(1, txt);
		break;
		case E_MENU:
		lcd_linea(0, "A:Saldo B:Retiro");
		lcd_linea(1, "C:Depos. D:Salir");
		break;
		case E_SALDO:
		u32_a_texto(c.saldo, num);
		strcpy(txt, "Saldo: $");
		strcat(txt, num);
		lcd_linea(0, txt);
		lcd_linea(1, "#:Volver");
		break;
		case E_MONTO:
		lcd_linea(0, operacion == 'B' ? "Monto retiro:" : "Monto deposito:");
		txt[0] = '$';
		memcpy(txt + 1, entrada, n_entrada);
		txt[n_entrada + 1] = '\0';
		lcd_linea(1, txt);
		break;
		default:
		break;
	}
}

void ir_a(estado_t e)
{
	estado = e;
	n_entrada = 0;
	t_inactivo = 0;
	pantalla();
}

void mensaje(const char *l1, const char *l2, uint8_t ind, estado_t sig)
{
	lcd_linea(0, l1);
	lcd_linea(1, l2);
	if (ind & IND_VERDE)  led_verde(1);
	if (ind & IND_ROJO)   led_rojo(1);
	if (ind & IND_BUZZER) buzzer(1);
	estado_sig = sig;
	t_mensaje = T_MENSAJE;
	estado = E_MENSAJE;
}

uint32_t texto_a_u32(const char *s, uint8_t n)
{
	uint32_t v = 0;
	uint8_t i;
	for (i = 0; i < n; i++) v = v * 10 + (s[i] - '0');
	return v;
}

void procesar_tarjeta(const uint8_t *uid)
{
	cuenta = cuenta_buscar(uid);
	if (cuenta == SIN_CUENTA) {
		mensaje("Tarjeta no", "registrada", IND_ROJO, E_ESPERA);
		return;
	}
	cuenta_leer(cuenta, &c);
	if (c.bloqueada) {
		mensaje("Tarjeta", "bloqueada", IND_ROJO, E_ESPERA);
		return;
	}
	intentos = 0;
	ir_a(E_PIN);
}

void verificar_pin(void)
{
	char txt[17];

	// --- (1) PIN correcto ---
	if (memcmp(entrada, c.pin, 4) == 0) {
		intentos = 0;
		mensaje("PIN correcto", "", IND_VERDE, E_MENU);
		return;
	}

	// --- (2) Tercer intento fallido: bloqueo persistente ---
	if (++intentos >= MAX_INTENTOS) {
		c.bloqueada = 1;
		cuenta_guardar(cuenta, &c);
		mensaje("Tarjeta", "bloqueada", IND_ROJO | IND_BUZZER, E_ESPERA);
		return;
	}

	// --- (3) Intento fallido: volver a pedir PIN ---
	strcpy(txt, "Intento ");
	txt[8]  = '0' + intentos;
	txt[9]  = '/';
	txt[10] = '0' + MAX_INTENTOS;
	txt[11] = '\0';
	mensaje("PIN incorrecto", txt, IND_ROJO, E_PIN);
}

void ejecutar_operacion(void)
{
	uint32_t monto = texto_a_u32(entrada, n_entrada);
	char txt[17], num[11];

	// --- (1) Validar monto ---
	if (monto % 10) {
		mensaje("Monto invalido", "Multiplo de 10", IND_ROJO, E_MENU);
		return;
	}

	// --- (2) Retiro o depósito ---
	if (operacion == 'B') {
		if (monto > c.saldo) {
			mensaje("Saldo", "insuficiente", IND_ROJO, E_MENU);
			return;
		}
		c.saldo -= monto;
		} else {
		c.saldo += monto;
	}

	// --- (3) Guardar en EEPROM y confirmar ---
	cuenta_guardar(cuenta, &c);
	u32_a_texto(c.saldo, num);
	strcpy(txt, "Saldo: $");
	strcat(txt, num);
	mensaje("Operacion OK", txt, IND_VERDE, E_MENU);
}

//==================== PROGRAMA PRINCIPAL ====================
int main(void)
{
	uint8_t uid[4], ultimo[4];
	uint8_t presente = 0, fallos = 0, ciclos = 0;
	uint8_t reset_fabrica;
	char tecla;

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

	// ---EEPROM: D presionada al encender = reset de fábrica ---
	reset_fabrica = (teclado_escanear() == 'D');
	eeprom_init(reset_fabrica);

	twi_lcd_init();
	_delay_ms(2);                                   // esperar fin del clear (1,52 ms)
	twi_lcd_cmd(0x0C);                              // display on, cursor off

	if (reset_fabrica) {
		lcd_linea(0, "Reset de fabrica");
		lcd_linea(1, "Cuentas cargadas");
		_delay_ms(1500);
	}
	ir_a(E_ESPERA);
	uart_print("Cajero listo\r\n");

	while (1) {
		tecla = teclado_leer();
		if (tecla) t_inactivo = 0;

		switch (estado) {
			// ---ESPERA: lectura de tarjeta cada 200 ms ---
			case E_ESPERA:
			if (++ciclos >= 20) {
				ciclos = 0;
				if (leer_tarjeta(uid)) {
					fallos = 0;
					if (!presente || memcmp(uid, ultimo, 4) != 0) {
						memcpy(ultimo, uid, 4);
						procesar_tarjeta(uid);
					}
					presente = 1;
					} else if (++fallos >= 2) {     // 2 fallos seguidos = tarjeta retirada
					presente = 0;
				}
			}
			break;

			// ---PIN: 4 dígitos, * borra, # confirma, D cancela ---
			case E_PIN:
			if (tecla >= '0' && tecla <= '9' && n_entrada < 4) {
				entrada[n_entrada++] = tecla;
				pantalla();
				} else if (tecla == '*' && n_entrada > 0) {
				n_entrada--;
				pantalla();
				} else if (tecla == '#' && n_entrada == 4) {
				verificar_pin();
				} else if (tecla == 'D') {
				ir_a(E_ESPERA);
			}
			break;

			// ---MENÚ: A saldo, B retiro, C depósito, D salir ---
			case E_MENU:
			if (tecla == 'A') {
				ir_a(E_SALDO);
				} else if (tecla == 'B' || tecla == 'C') {
				operacion = tecla;
				ir_a(E_MONTO);
				} else if (tecla == 'D') {
				mensaje("Gracias", "Retire tarjeta", IND_NINGUNO, E_ESPERA);
			}
			break;

			// ---SALDO: # o D vuelve al menú ---
			case E_SALDO:
			if (tecla == '#' || tecla == 'D') ir_a(E_MENU);
			break;

			// ---MONTO: hasta 5 dígitos, * borra, # confirma, D vuelve ---
			case E_MONTO:
			if (tecla >= '0' && tecla <= '9' && n_entrada < MAX_DIG_MONTO) {
				entrada[n_entrada++] = tecla;
				pantalla();
				} else if (tecla == '*' && n_entrada > 0) {
				n_entrada--;
				pantalla();
				} else if (tecla == '#' && n_entrada > 0) {
				ejecutar_operacion();
				} else if (tecla == 'D') {
				ir_a(E_MENU);
			}
			break;

			// ---MENSAJE: esperar y pasar al estado siguiente ---
			case E_MENSAJE:
			if (--t_mensaje == 0) {
				indicadores_off();
				ir_a(estado_sig);
			}
			break;
		}

		// ---Timeout de inactividad durante la sesión ---
		if (estado == E_PIN || estado == E_MENU || estado == E_SALDO || estado == E_MONTO) {
			if (++t_inactivo >= T_INACTIVIDAD)
			mensaje("Tiempo agotado", "Sesion cerrada", IND_ROJO, E_ESPERA);
		}

		_delay_ms(TICK_MS);
	}
}