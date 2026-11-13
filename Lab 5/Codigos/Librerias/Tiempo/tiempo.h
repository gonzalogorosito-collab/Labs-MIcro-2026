//==========================================================
// tiempo.h - Base de tiempos, ADC periódico y planificador
// Laboratorio 5 - Tecnologías de Microprocesamiento
//==========================================================
// Timer0 : CTC, base de tiempos de 1 ms (millis)
// Timer1 : CTC, interrupción cada 100 ms -> dispara el ADC
// ADC    : conversión por interrupción, resultado en buffer
// Planificador cooperativo por tabla de tareas (sin bloqueo)
//==========================================================

#ifndef TIEMPO_H                         // Evita incluir este archivo dos veces
#define TIEMPO_H                         // Marca el archivo como ya incluido

#ifndef F_CPU                            // Si la frecuencia no fue definida antes...
#define F_CPU 16000000UL                 // ...se fija en 16 MHz (cristal del Arduino Uno)
#endif                                   // Fin de la definición condicional de F_CPU

#include <avr/io.h>                      // Nombres de registros y bits del ATmega328P
#include <stdint.h>                      // Tipos enteros de tamaño fijo (uint8_t, uint16_t, uint32_t)

//===== Configuración =====
#define TIEMPO_ADC_CANAL  2              // Canal ADC2 (pin PC2 / A2); ADC0 y ADC1 los pisa twi_init()

//===== Tarea del planificador =====
typedef struct {                         // Estructura que describe una tarea periódica
	uint16_t periodo_ms;                 // Cada cuántos ms debe ejecutarse la tarea
	uint32_t ultimo_ms;                  // Instante (en ms) de la última ejecución
	void   (*funcion)(void);             // Puntero a la función que realiza la tarea
	uint8_t  activa;                     // 1 = la tarea se ejecuta, 0 = está suspendida
} tarea_t;                               // Nombre del tipo de dato

//===== Base de tiempos =====
void     tiempo_init(void);              // Configura Timer0, Timer1 y ADC
uint32_t millis(void);                   // Devuelve los ms transcurridos desde el arranque
void     tiempo_sumar_ms(uint32_t ms);   // Suma ms al contador (compensa el tiempo en Power-Down)
void     tiempo_base_rapida(void);       // Base de 1 ms con Timer0 (estado ACTIVO)
void     tiempo_base_lenta(void);        // Base de 100 ms solo con Timer1 (estado STANDBY)

//===== ADC periódico =====
void     adc_habilitar(uint8_t on);      // 1 = Timer1 dispara conversiones, 0 = no las dispara
uint8_t  adc_nuevo_dato(uint16_t *valor);// Devuelve 1 si hay conversión nueva y copia el valor
uint16_t adc_ultimo(void);               // Devuelve el último valor convertido (0 a 1023)

//===== Planificador cooperativo =====
void     planificador_ejecutar(tarea_t *tareas, uint8_t n);  // Ejecuta las tareas que vencieron
void     planificador_reiniciar(tarea_t *tareas, uint8_t n); // Pone todas las tareas en "ahora"

#endif                                   // Fin de la protección contra inclusión doble