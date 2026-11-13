//==========================================================
// energia.h - Modos sleep, PRR, Watchdog Timer y causa de reset
// Laboratorio 5 - Tecnologías de Microprocesamiento
//==========================================================
// Sleep : Idle, ADC Noise Reduction y Power-Down (con BOD apagado)
// PRR   : máscaras explícitas por periférico (nunca 0xFF, bit 4 reservado)
// WDT   : modo interrupción, modo reset y modo combinado
// Reset : copia de MCUSR tomada antes de main() (.init3)
// Reloj : prescaler del sistema mediante CLKPR
//==========================================================

#ifndef ENERGIA_H                        // Evita incluir este archivo dos veces
#define ENERGIA_H                        // Marca el archivo como ya incluido

#ifndef F_CPU                            // Si la frecuencia no fue definida antes...
#define F_CPU 16000000UL                 // ...se fija en 16 MHz
#endif                                   // Fin de la definición condicional de F_CPU

#include <avr/io.h>                      // Registros y bits del ATmega328P
#include <avr/sleep.h>                   // Constantes SLEEP_MODE_* y macros de sleep
#include <avr/power.h>                   // Tipo clock_div_t para el prescaler del reloj
#include <avr/wdt.h>                     // Macro wdt_reset() (instrucción WDR)
#include <stdint.h>                      // Tipos enteros de tamaño fijo

//===== Modos sleep (valores de SMCR definidos en avr/sleep.h) =====
#define DORMIR_IDLE       SLEEP_MODE_IDLE      // CPU detenida; timers, USART, ADC, TWI siguen
#define DORMIR_ADC        SLEEP_MODE_ADC       // ADC Noise Reduction: solo ADC, TWI, WDT, INT y Timer2 asíncrono
#define DORMIR_POWER_DOWN SLEEP_MODE_PWR_DOWN  // Oscilador detenido: solo INT0/INT1 (nivel), PCINT, TWI dirección, WDT

//===== Máscaras de PRR (una por periférico) =====
#define PRR_ADC     (1 << PRADC)         // 0x01 - ADC (apagar ADEN antes)
#define PRR_USART0  (1 << PRUSART0)      // 0x02 - USART0 (apagar RX/TX antes)
#define PRR_SPI     (1 << PRSPI)         // 0x04 - SPI
#define PRR_TIM1    (1 << PRTIM1)        // 0x08 - Timer/Counter1
#define PRR_TIM0    (1 << PRTIM0)        // 0x20 - Timer/Counter0
#define PRR_TIM2    (1 << PRTIM2)        // 0x40 - Timer/Counter2
#define PRR_TWI     (1 << PRTWI)         // 0x80 - TWI (I2C)
#define PRR_VALIDOS (PRR_ADC | PRR_USART0 | PRR_SPI | PRR_TIM1 | PRR_TIM0 | PRR_TIM2 | PRR_TWI)  // 0xEF: todos menos el bit 4 reservado

//===== Intervalos del WDT (bits WDP3..WDP0 de WDTCSR) =====
#define WDT_PRESC(n)  (((n) & 0x07) | (((n) & 0x08) << 2))  // WDP2..0 en bits 2..0, WDP3 en el bit 5
#define WDT_16MS      WDT_PRESC(0)       // 2K ciclos del oscilador de 128 kHz  -> ~16 ms
#define WDT_32MS      WDT_PRESC(1)       // 4K ciclos   -> ~32 ms
#define WDT_64MS      WDT_PRESC(2)       // 8K ciclos   -> ~64 ms
#define WDT_125MS     WDT_PRESC(3)       // 16K ciclos  -> ~0,125 s
#define WDT_250MS     WDT_PRESC(4)       // 32K ciclos  -> ~0,25 s
#define WDT_500MS     WDT_PRESC(5)       // 64K ciclos  -> ~0,5 s
#define WDT_1S        WDT_PRESC(6)       // 128K ciclos -> ~1 s
#define WDT_2S        WDT_PRESC(7)       // 256K ciclos -> ~2 s
#define WDT_4S        WDT_PRESC(8)       // 512K ciclos -> ~4 s
#define WDT_8S        WDT_PRESC(9)       // 1024K ciclos -> ~8 s

//===== Causa del último reset (bits de MCUSR) =====
#define RESET_POR     (1 << PORF)        // Encendido (Power-On Reset)
#define RESET_EXT     (1 << EXTRF)       // Pin RESET (botón de la placa o programación)
#define RESET_BOR     (1 << BORF)        // Caída de tensión (Brown-Out)
#define RESET_WDT     (1 << WDRF)        // Watchdog Timer

//===== General =====
void     energia_init(void);             // Apaga lo que el sistema nunca usa (comparador analógico, SPI)

//===== Sleep =====
void     energia_dormir(uint8_t modo);   // Idle o ADC Noise Reduction hasta la próxima interrupción
void     energia_power_down(void);       // Power-Down con BOD apagado; INT0/INT1 pasan a nivel bajo mientras duerme

//===== PRR =====
void     energia_prr_apagar(uint8_t mascara);    // Detiene el reloj de los periféricos indicados
void     energia_prr_encender(uint8_t mascara);  // Vuelve a dar reloj a los periféricos indicados
uint8_t  energia_prr_estado(void);               // Valor actual de PRR (1 = periférico apagado)

//===== Watchdog Timer =====
void     wdt_modo_interrupcion(uint8_t intervalo);  // Solo interrupción: despierta, nunca reinicia
void     wdt_modo_reset(uint8_t intervalo);         // Solo reset: reinicia si no se lo alimenta a tiempo
void     wdt_modo_combinado(uint8_t intervalo);     // Primero interrupción, al siguiente vencimiento reset
void     wdt_apagar(void);                          // Detiene el WDT por completo
#define  wdt_alimentar()  wdt_reset()               // Reinicia la cuenta del WDT (instrucción WDR)
uint8_t  wdt_disparo(void);              // 1 si hubo interrupción del WDT desde la última consulta
uint16_t wdt_eventos(void);              // Cantidad total de interrupciones del WDT
uint16_t wdt_periodo_ms(void);           // Período nominal configurado, en ms

//===== Causa de reset =====
uint8_t  energia_causa_reset(void);      // Copia de MCUSR del arranque (usar con RESET_*)

//===== Reloj del sistema =====
void     energia_reloj_div(clock_div_t div);  // clock_div_1, clock_div_2, clock_div_8, clock_div_64, clock_div_128

#endif                                   // Fin de la protección contra inclusión doble