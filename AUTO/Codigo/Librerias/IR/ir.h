#ifndef IR_H
#define IR_H

#include <stdint.h>

// ===== CONFIGURACIÓN =====
#define IR_DETECTA_ALTO  1    // 1: OUT = 1 sin reflejo (borde); 0 si el módulo funciona al revés
#define IR_MUESTRAS      5    // lecturas iguales seguidas (una por ms) para aceptar un cambio

// ===== FUNCIONES =====
void    ir_init(void);
void    ir_update(uint32_t ahora);   // llamar en cada vuelta del bucle con millis()
uint8_t ir_detecta(void);            // estado filtrado: 1 = borde
uint8_t ir_flanco(void);             // 1 una sola vez al pasar de piso a borde

#endif