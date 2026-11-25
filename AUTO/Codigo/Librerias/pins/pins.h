// ===== MAPEO DE PINES =====
// D0/D1  (PD0/PD1) -> USART: RX/TX del HC-06, 9600 baud
// D6     (PD6/OC0A)-> ENA (PWM motor izquierdo, Timer0)
// D5     (PD5/OC0B)-> ENB (PWM motor derecho, Timer0)
// D7     (PD7)     -> IN1 (motor izquierdo)
// D8     (PB0)     -> IN2 (motor izquierdo)
// D12    (PB4)     -> IN3 (motor derecho)
// D13    (PB5)     -> IN4 (motor derecho)
// D9     (PB1/OC1A)-> servo SG90 (Timer1, parte 3)
// D4     (PD4)     -> sensor IR al piso (parte 4)
// A1     (ADC1)    -> lectura de batería (parte 4)
// Timer2           -> tick de 1 ms (parte 2)

#ifndef PINS_H
#define PINS_H

#include <avr/io.h>

// ===== MOTOR IZQUIERDO (canal A del L298N) =====
#define ENA_DDR   DDRD
#define ENA_BIT   PD6

#define IN1_DDR   DDRD
#define IN1_PORT  PORTD
#define IN1_BIT   PD7

#define IN2_DDR   DDRB
#define IN2_PORT  PORTB
#define IN2_BIT   PB0

// ===== MOTOR DERECHO (canal B del L298N) =====
#define ENB_DDR   DDRD
#define ENB_BIT   PD5

#define IN3_DDR   DDRB
#define IN3_PORT  PORTB
#define IN3_BIT   PB4

#define IN4_DDR   DDRB
#define IN4_PORT  PORTB
#define IN4_BIT   PB5

#endif