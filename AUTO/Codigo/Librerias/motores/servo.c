#include "pins.h"
#include "servo.h"
#include "timer.h"

// ===== VARIABLES =====
static uint16_t actual_us;                   // ancho de pulso que se está generando
static uint16_t destino_us;                  // ancho de pulso al que se quiere llegar
static uint32_t ultimo_paso = 0;             // millis() del último paso del barrido

// ===== FUNCIONES INTERNAS =====

// Escribe el ancho de pulso en el timer: 1 tick = 0,5 µs -> OCR1A = µs * 2
// En modo PWM, OCR1A tiene doble buffer: el valor nuevo se aplica al inicio del
// próximo período, así que nunca sale un pulso cortado
static void escribir_pulso(uint16_t us) {
	OCR1A = us * 2;
}

// ===== FUNCIONES PÚBLICAS =====

void servo_init(void) {
	// --- (1) D9 como salida ---
	SERVO_DDR |= (1 << SERVO_BIT);

	// --- (2) Fast PWM modo 14 (TOP = ICR1), salida OC1A no invertida ---
	TCCR1A = (1 << COM1A1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12);

	// --- (3) Período: 16 MHz / 8 = 2 MHz -> 0,5 µs por tick; 40000 ticks = 20 ms ---
	ICR1 = 39999;

	// --- (4) Posición inicial (el servo salta a ella al encender) ---
	actual_us  = SERVO_INICIAL;
	destino_us = SERVO_INICIAL;
	escribir_pulso(actual_us);

	// --- (5) Prescaler 8 y arranca el timer ---
	TCCR1B |= (1 << CS11);
}

void servo_open(void)  { servo_set_us(SERVO_ABIERTO_US); }
void servo_close(void) { servo_set_us(SERVO_CERRADO_US); }

void servo_set_us(uint16_t us) {
	if (us < SERVO_MIN_US) us = SERVO_MIN_US;
	if (us > SERVO_MAX_US) us = SERVO_MAX_US;
	destino_us = us;
}

uint16_t servo_get_us(void) {
	return destino_us;
}

void servo_hold(void) {
	destino_us = actual_us;
}

uint8_t servo_moving(void) {
	return actual_us != destino_us;
}

void servo_update(void) {
	// --- (1) Nada que hacer si ya llegó o si no pasó un período ---
	if (actual_us == destino_us) return;
	if (millis() - ultimo_paso < SERVO_PERIODO_MS) return;
	ultimo_paso = millis();

	// --- (2) Avanza un paso hacia el destino sin pasarse ---
	if (actual_us < destino_us) {
		if (destino_us - actual_us > SERVO_PASO_US) actual_us += SERVO_PASO_US;
		else                                        actual_us  = destino_us;
	} else {
		if (actual_us - destino_us > SERVO_PASO_US) actual_us -= SERVO_PASO_US;
		else                                        actual_us  = destino_us;
	}

	// --- (3) Actualiza el pulso ---
	escribir_pulso(actual_us);
}