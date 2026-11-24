#include "pins.h"
#include "motors.h"

// ===== FUNCIONES INTERNAS =====

// Limita un valor al rango -255..255
static int16_t limitar(int16_t v) {
	if (v > 255)  return 255;
	if (v < -255) return -255;
	return v;
}

// Motor izquierdo: IN1/IN2 dan el sentido, OCR0A (D6) el duty
static void motor_izq(int16_t v) {
	v = limitar(v);
	if (v > 0) {                         // --- (1) adelante ---
		IN1_PORT |=  (1 << IN1_BIT);
		IN2_PORT &= ~(1 << IN2_BIT);
		OCR0A = (uint8_t)v;
	} else if (v < 0) {                  // --- (2) atrás ---
		IN1_PORT &= ~(1 << IN1_BIT);
		IN2_PORT |=  (1 << IN2_BIT);
		OCR0A = (uint8_t)(-v);
	} else {                             // --- (3) parado: IN1 = IN2 = 0 ---
		IN1_PORT &= ~(1 << IN1_BIT);
		IN2_PORT &= ~(1 << IN2_BIT);
		OCR0A = 0;
	}
}

// Motor derecho: IN3/IN4 dan el sentido, OCR0B (D5) el duty
static void motor_der(int16_t v) {
	v = limitar(v);
	if (v > 0) {                         // --- (1) adelante ---
		IN3_PORT |=  (1 << IN3_BIT);
		IN4_PORT &= ~(1 << IN4_BIT);
		OCR0B = (uint8_t)v;
	} else if (v < 0) {                  // --- (2) atrás ---
		IN3_PORT &= ~(1 << IN3_BIT);
		IN4_PORT |=  (1 << IN4_BIT);
		OCR0B = (uint8_t)(-v);
	} else {                             // --- (3) parado: IN3 = IN4 = 0 ---
		IN3_PORT &= ~(1 << IN3_BIT);
		IN4_PORT &= ~(1 << IN4_BIT);
		OCR0B = 0;
	}
}

// ===== FUNCIONES PÚBLICAS =====

void motors_init(void) {
	// --- (1) Pines como salida ---
	ENA_DDR |= (1 << ENA_BIT);
	ENB_DDR |= (1 << ENB_BIT);
	IN1_DDR |= (1 << IN1_BIT);
	IN2_DDR |= (1 << IN2_BIT);
	IN3_DDR |= (1 << IN3_BIT);
	IN4_DDR |= (1 << IN4_BIT);

	// --- (2) Timer0 en Fast PWM (TOP = 0xFF), salidas OC0A y OC0B no invertidas ---
	TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);

	// --- (3) Prescaler 64 -> f = 16 MHz / (64 * 256) = 976,6 Hz ---
	TCCR0B = (1 << CS01) | (1 << CS00);

	// --- (4) Arranca parado ---
	motors_stop();
}

void motors_set(int16_t izq, int16_t der) {
	motor_izq(izq);
	motor_der(der);
}

void motors_stop(void) {
	motor_izq(0);
	motor_der(0);
}