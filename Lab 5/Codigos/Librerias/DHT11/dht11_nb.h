//==========================================================
// dht11_nb.h - Lectura del DHT11 sin bloquear el arranque de 18 ms
// Laboratorio 5 - Tecnologías de Microprocesamiento
//==========================================================
// 1) dht_iniciar_lectura(): pone la línea en bajo y vuelve enseguida
// 2) dht_tarea(): a los 20 ms (contados con millis) suelta la línea
//    y lee los 40 bits midiendo cada pulso con Timer2 (0,5 us/cuenta)
//    Esa ráfaga dura ~4 ms y es el único tramo que ocupa la CPU
// Timer2 solo tiene reloj durante la ráfaga (TCCR2B = 0 el resto)
//==========================================================

#ifndef DHT11_NB_H                       // Evita incluir este archivo dos veces
#define DHT11_NB_H                       // Marca el archivo como ya incluido

#include <avr/io.h>                      // Registros de puertos y de Timer2
#include <stdint.h>                      // Tipos enteros de tamaño fijo

//===== Configuración del pin =====
#define DHT_DDR       DDRD               // Registro de dirección del puerto del sensor
#define DHT_PORT      PORTD              // Registro de salida del puerto del sensor
#define DHT_PINR      PIND               // Registro de lectura del puerto del sensor
#define DHT_BIT       PD4                // Pin de datos del DHT11 (D4)

//===== Tiempos =====
#define DHT_ARRANQUE_MS     20           // Pulso bajo de inicio (el sensor pide al menos 18 ms)
#define DHT_INTERVALO_MIN   1000         // Mínimo entre lecturas según la hoja de datos (1 s)
#define DHT_TIMEOUT_TICKS   200          // 100 us máximo por nivel (200 cuentas de 0,5 us)
#define DHT_UMBRAL_TICKS    80           // 40 us: un pulso alto más largo es un "1" (0: ~27 us, 1: ~70 us)

//===== Resultados =====
#define DHT_OK              0            // Lectura correcta
#define DHT_ERR_RESPUESTA   1            // El sensor no respondió al pulso de inicio
#define DHT_ERR_TIMEOUT     2            // Un bit no llegó a tiempo
#define DHT_ERR_CHECKSUM    3            // Los datos no coinciden con el byte de verificación
#define DHT_SIN_DATO        4            // Todavía no se hizo ninguna lectura

//===== Funciones =====
void    dht_init(void);                  // Pin en entrada con pull-up y Timer2 detenido
uint8_t dht_iniciar_lectura(void);       // 1 = arrancó una lectura; 0 = ocupado o falta 1 s desde la anterior
uint8_t dht_tarea(void);                 // Llamar en el lazo principal: 1 = terminó una lectura (ver dht_resultado)
uint8_t dht_ocupado(void);               // 1 = hay una lectura en curso
uint8_t dht_resultado(uint8_t *temp, uint8_t *hum);  // Últimos valores válidos y código de la última lectura

#endif                                   // Fin de la protección contra inclusión doble