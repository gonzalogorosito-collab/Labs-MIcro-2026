#include "pins.h"
#include "ir.h"

// ===== ESTADO =====
static uint8_t  estado   = 0;    // último estado aceptado
static uint8_t  contador = 0;    // lecturas seguidas distintas al estado
static uint8_t  flanco   = 0;    // pendiente de consumir con ir_flanco()
static uint32_t ultimo   = 0;    // ms de la última muestra

// Lee el pin y lo normaliza: 1 = borde
static uint8_t leer(void) {
	uint8_t pin = (IR_PINR >> IR_BIT) & 1;
	return IR_DETECTA_ALTO ? pin : !pin;
}

void ir_init(void) {
	IR_DDR  &= ~(1 << IR_BIT);       // --- (1) entrada ---
	IR_PORT |=  (1 << IR_BIT);       // --- (2) pull-up interno ---
	estado = leer();                 // --- (3) arranca con el estado real: no dispara al encender ---
}

void ir_update(uint32_t ahora) {
	// --- (1) Una muestra por milisegundo ---
	if (ahora == ultimo) return;
	ultimo = ahora;

	// --- (2) Filtro: el cambio se acepta tras IR_MUESTRAS lecturas iguales ---
	if (leer() == estado) {
		contador = 0;
		return;
	}
	if (++contador >= IR_MUESTRAS) {
		estado   = !estado;
		contador = 0;
		if (estado) flanco = 1;      // --- (3) solo el paso piso -> borde genera flanco ---
	}
}

uint8_t ir_detecta(void) {
	return estado;
}

uint8_t ir_flanco(void) {
	uint8_t f = flanco;
	flanco = 0;
	return f;
}