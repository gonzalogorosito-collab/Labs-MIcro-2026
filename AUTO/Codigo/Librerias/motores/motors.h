#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>

// ===== CALIBRACIÓN =====
#define PWM_MIN  100    // duty mínimo con el que arrancan los motores (a calibrar)

// ===== FUNCIONES =====
void motors_init(void);
void motors_set(int16_t izq, int16_t der);   // -255..255: signo = sentido, módulo = duty
void motors_stop(void);

#endif