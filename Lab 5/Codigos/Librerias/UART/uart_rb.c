//==========================================================
// uart_rb.c - USART0 por interrupciones con buffers circulares
//==========================================================

#include "uart_rb.h"                     // Prototipos y configuración de esta librería
#include <avr/interrupt.h>               // Macro ISR()
#include <util/atomic.h>                 // Macro ATOMIC_BLOCK

#define RX_MASK (UART_RX_TAM - 1)        // Máscara para dar la vuelta al buffer RX (índice & máscara)
#define TX_MASK (UART_TX_TAM - 1)        // Máscara para dar la vuelta al buffer TX

//===== Buffers circulares =====
static volatile uint8_t rx_buf[UART_RX_TAM];  // Bytes recibidos aún no leídos
static volatile uint8_t rx_head = 0;     // Próxima posición donde escribe la ISR de RX
static volatile uint8_t rx_tail = 0;     // Próxima posición donde lee el programa principal

static volatile uint8_t tx_buf[UART_TX_TAM];  // Bytes pendientes de envío
static volatile uint8_t tx_head = 0;     // Próxima posición donde escribe el programa principal
static volatile uint8_t tx_tail = 0;     // Próxima posición donde lee la ISR de UDRE
static volatile uint8_t tx_activo = 0;   // 1 = se escribió al menos un byte en UDR0 sin confirmar su salida

static volatile uint8_t errores = 0;     // Contador de errores de comunicación (satura en 255)

//==========================================================
// Inicialización y control
//==========================================================
void uart_init(unsigned int ubrr)        // ubrr = 103 para 9600 bps a 16 MHz
{
	UBRR0H = (uint8_t)(ubrr >> 8);       // Parte alta del divisor de baudios
	UBRR0L = (uint8_t)ubrr;              // Parte baja del divisor de baudios
	UCSR0A = 0;                          // Velocidad normal (U2X0 = 0), sin multiprocesador
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // Formato 8N1: 8 bits de datos, sin paridad, 1 stop
	uart_habilitar(1);                   // Habilita receptor, transmisor e interrupción de recepción
}                                        // Fin de uart_init

void uart_habilitar(uint8_t on)          // Enciende o apaga la USART
{
	if (on) {                            // Si se pide encender...
		UCSR0B |= (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);  // ...habilita RX, TX e interrupción de RX
		if (tx_head != tx_tail) UCSR0B |= (1 << UDRIE0);        // Si quedó algo por enviar, reanuda el envío
	} else {                             // Si se pide apagar...
		UCSR0B &= ~((1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) | (1 << UDRIE0));  // ...apaga RX, TX y sus interrupciones
		tx_activo = 0;                   // No queda ningún byte en curso
	}                                    // Fin de la selección encendido/apagado
}                                        // Fin de uart_habilitar

//==========================================================
// Rutinas de interrupción
//==========================================================
ISR(USART_RX_vect)                       // Se ejecuta cuando llega un byte completo
{
	uint8_t estado = UCSR0A;             // Lee los flags de error ANTES de leer UDR0 (lo exige el datasheet)
	uint8_t dato   = UDR0;               // Lee el byte recibido (esto borra RXC0)
	uint8_t sig    = (rx_head + 1) & RX_MASK;  // Posición siguiente del buffer

	if (estado & ((1 << FE0) | (1 << DOR0))) {  // Error de trama (stop inválido) o byte perdido por sobreescritura
		if (errores < 255) errores++;    // Cuenta el error
		return;                          // Descarta el byte dañado
	}                                    // Fin del chequeo de errores
	if (sig == rx_tail) {                // Si el buffer está lleno...
		if (errores < 255) errores++;    // ...cuenta el error
		return;                          // ...y descarta el byte
	}                                    // Fin del chequeo de buffer lleno
	rx_buf[rx_head] = dato;              // Guarda el byte en el buffer
	rx_head = sig;                       // Avanza la cabeza
}                                        // Fin de la ISR de recepción

ISR(USART_UDRE_vect)                     // Se ejecuta cuando UDR0 queda libre para otro byte
{
	if (tx_head == tx_tail) {            // Si no queda nada por enviar...
		UCSR0B &= ~(1 << UDRIE0);        // ...deshabilita esta interrupción (si no, se repetiría sin fin)
		return;                          // Sale de la ISR
	}                                    // Fin del chequeo de buffer vacío
	UCSR0A = (UCSR0A & ((1 << U2X0) | (1 << MPCM0))) | (1 << TXC0);  // Borra TXC0 (escribiendo 1) sin tocar los flags de error
	UDR0 = tx_buf[tx_tail];              // Envía el byte más antiguo del buffer
	tx_tail = (tx_tail + 1) & TX_MASK;   // Avanza la cola
	tx_activo = 1;                       // Hay un byte saliendo por el pin TX
}                                        // Fin de la ISR de transmisión

//==========================================================
// Recepción
//==========================================================
uint8_t uart_disponible(void)            // Bytes esperando a ser leídos
{
	return (rx_head - rx_tail) & RX_MASK;    // Diferencia entre cabeza y cola, con vuelta del buffer
}                                        // Fin de uart_disponible

uint8_t uart_leer(uint8_t *c)            // Saca un byte del buffer de recepción
{
	if (rx_head == rx_tail) return 0;    // Buffer vacío: no hay dato
	*c = rx_buf[rx_tail];                // Copia el byte más antiguo
	rx_tail = (rx_tail + 1) & RX_MASK;   // Avanza la cola (solo la modifica el programa principal)
	return 1;                            // Había dato
}                                        // Fin de uart_leer

void uart_vaciar_rx(void)                // Descarta lo recibido
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { rx_tail = rx_head; }  // Iguala cola y cabeza sin que la ISR intervenga
}                                        // Fin de uart_vaciar_rx

//==========================================================
// Transmisión
//==========================================================
uint8_t uart_send(char c)                // Encola un byte para enviar
{
	uint8_t sig = (tx_head + 1) & TX_MASK;   // Posición siguiente del buffer
	if (sig == tx_tail) {                // Si el buffer está lleno...
		if (errores < 255) errores++;    // ...cuenta el error
		return 0;                        // ...y descarta el byte en lugar de esperar
	}                                    // Fin del chequeo de buffer lleno
	tx_buf[tx_head] = (uint8_t)c;        // Guarda el byte
	tx_head = sig;                       // Avanza la cabeza
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {  // Bloque sin interrupciones (la ISR también modifica UCSR0B)
		if (UCSR0B & (1 << TXEN0)) UCSR0B |= (1 << UDRIE0);  // Si el transmisor está encendido, activa el envío
	}                                    // Fin del bloque atómico
	return 1;                            // Byte encolado
}                                        // Fin de uart_send

void uart_print(const char *s)           // Encola una cadena
{
	while (*s) uart_send(*s++);          // Encola carácter por carácter hasta el '\0'
}                                        // Fin de uart_print

void uart_print_hex(uint8_t val)         // Encola un byte en hexadecimal (sin sprintf)
{
	const char hex[] = "0123456789ABCDEF";   // Tabla de dígitos hexadecimales
	uart_send(hex[val >> 4]);            // Nibble alto
	uart_send(hex[val & 0x0F]);          // Nibble bajo
}                                        // Fin de uart_print_hex

void uart_print_u16(uint16_t val)        // Encola un número en decimal (sin sprintf)
{
	char txt[6];                         // Hasta 5 dígitos (65535) + '\0'
	uint8_t i = 5;                       // Se llena de atrás hacia adelante
	txt[i] = '\0';                       // Terminador de la cadena
	do {                                 // Al menos un dígito (para el valor 0)
		txt[--i] = '0' + (val % 10);     // Último dígito decimal convertido a ASCII
		val /= 10;                       // Descarta ese dígito
	} while (val);                       // Repite mientras queden dígitos
	uart_print(&txt[i]);                 // Encola desde el primer dígito significativo
}                                        // Fin de uart_print_u16

uint8_t uart_tx_libre(void)              // Lugares libres en el buffer de transmisión
{
	return (tx_tail - tx_head - 1) & TX_MASK;  // Capacidad menos ocupados (se reserva un lugar para distinguir lleno de vacío)
}                                        // Fin de uart_tx_libre

uint8_t uart_tx_terminado(void)          // Indica si ya se puede apagar la USART sin cortar un envío
{
	if (tx_head != tx_tail) return 0;    // Quedan bytes en el buffer
	if (tx_activo && !(UCSR0A & (1 << TXC0))) return 0;  // El último byte todavía está saliendo por el pin
	tx_activo = 0;                       // Todo salió
	return 1;                            // Se puede apagar
}                                        // Fin de uart_tx_terminado

//==========================================================
// Diagnóstico
//==========================================================
uint8_t uart_errores(void)               // Devuelve el contador de errores
{
	return errores;                      // Lectura de 1 byte: no necesita bloque atómico
}                                        // Fin de uart_errores