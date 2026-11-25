#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer.h"

// ===== VARIABLES =====
// volatile: la modifica la ISR, así que el compilador debe leerla siempre de memoria
static volatile uint32_t contador_ms = 0;

// ===== INTERRUPCIÓN =====
// Se ejecuta cada vez que TCNT2 llega a OCR2A (cada 1 ms)
ISR(TIMER2_COMPA_vect) {
	contador_ms++;
}

// ===== FUNCIONES PÚBLICAS =====

void timer_init(void) {
	// --- (1) Modo CTC: el contador vuelve a 0 al llegar a OCR2A ---
	TCCR2A = (1 << WGM21);

	// --- (2) Período: 16 MHz / 64 / (249 + 1) = 1000 Hz -> 1 ms ---
	OCR2A = 249;

	// --- (3) Habilita la interrupción por comparación con OCR2A ---
	TIMSK2 = (1 << OCIE2A);

	// --- (4) Prescaler 64 (en Timer2: CS22 = 1) y arranca el timer ---
	TCCR2B = (1 << CS22);
}

uint32_t millis(void) {
	uint32_t t;
	uint8_t sreg = SREG;     // --- (1) guarda el estado de las interrupciones ---
	cli();                   // --- (2) las deshabilita: el AVR lee los 4 bytes de a uno
	t = contador_ms;         //         y la ISR podría cambiar el valor a mitad de lectura
	SREG = sreg;             // --- (3) restaura el estado anterior ---
	return t;
}