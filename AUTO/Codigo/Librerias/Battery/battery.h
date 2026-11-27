#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>

// ===== CONFIGURACIÓN =====
#define BAT_CANAL       1       // A1 (ADC1)
#define BAT_VREF_MV     5000    // AVcc: medir el 5 V real del Arduino con el tester
#define BAT_DIVISOR     2       // 10 kΩ / 10 kΩ: Vbat = 2 · VA1
#define BAT_MV_VACIA    6400    // 2S: 3,2 V por celda = 0 %
#define BAT_MV_LLENA    8400    // 2S: 4,2 V por celda = 100 %
#define BAT_PERIODO_MS  100     // cada cuánto se mide

// ===== FUNCIONES =====
void     battery_init(void);
void     battery_update(uint32_t ahora);   // llamar en cada vuelta del bucle con millis()
uint16_t battery_mv(void);                 // tensión filtrada en mV
uint8_t  battery_pct(void);                // 0..100

#endif