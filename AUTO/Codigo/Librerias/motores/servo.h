#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

// ===== CALIBRACIÓN (anchos de pulso en µs) =====
#define SERVO_ABIERTO_US   1000              // a calibrar con la pinza armada
#define SERVO_CERRADO_US   2000              // a calibrar con la pinza armada
#define SERVO_INICIAL      SERVO_CERRADO_US  // posición al encender: la que ocupa menos lugar
#define SERVO_MIN_US       500               // límites de seguridad del SG90
#define SERVO_MAX_US       2500

// ===== BARRIDO SUAVE =====
#define SERVO_PERIODO_MS   20                // cada cuánto avanza el barrido (= período del servo)
#define SERVO_PASO_US      40                // cuánto avanza en cada paso -> 1000 µs en 500 ms

// ===== FUNCIONES =====
void     servo_init(void);
void     servo_open(void);
void     servo_close(void);
void     servo_set_us(uint16_t us);          // fija la posición destino (limitada a MIN..MAX)
uint16_t servo_get_us(void);                 // posición destino actual
void     servo_hold(void);                   // detiene el barrido en la posición actual
uint8_t  servo_moving(void);                 // 1 si el barrido no terminó
void     servo_update(void);                 // llamar en cada vuelta del bucle principal

#endif