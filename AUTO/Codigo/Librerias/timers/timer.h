#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

// ===== FUNCIONES =====
void     timer_init(void);   // Timer2 en CTC: interrupción cada 1 ms
uint32_t millis(void);       // milisegundos desde el arranque

#endif