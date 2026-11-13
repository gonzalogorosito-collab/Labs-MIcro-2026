//==========================================================
// energia.c - Modos sleep, PRR, Watchdog Timer y causa de reset
//==========================================================

#include "energia.h"                     // Prototipos, máscaras e intervalos de esta librería
#include <avr/interrupt.h>               // Macro ISR(), sei() y cli()
#include <util/atomic.h>                 // Macro ATOMIC_BLOCK

//===== Copia de MCUSR =====
uint8_t mcusr_copia __attribute__((section(".noinit")));  // En .noinit: el arranque de C no la borra

//===== Variables del WDT =====
static volatile uint8_t  wdt_flag = 0;   // 1 = hubo interrupción del WDT sin consultar
static volatile uint16_t wdt_cuenta = 0; // Interrupciones del WDT desde el arranque
static uint16_t          wdt_ms = 0;     // Período nominal configurado (ms)

//==========================================================
// Arranque: se ejecuta ANTES de main() y antes de inicializar variables
//==========================================================
void energia_arranque(void) __attribute__((naked, used, section(".init3")));  // Sin prólogo, se conserva aunque nadie la llame, va en .init3
void energia_arranque(void)              // Captura la causa de reset y apaga el WDT
{
	mcusr_copia = MCUSR;                 // Guarda qué provocó este reset
	MCUSR = 0;                           // Borra los flags (WDRF en 1 impediría apagar el WDT)
	WDTCSR = (1 << WDCE) | (1 << WDE);   // Secuencia temporizada: habilita el cambio (4 ciclos para el siguiente)
	WDTCSR = 0;                          // Apaga el WDT: tras un reset por WDT sigue activo en 16 ms y reiniciaría en bucle
}                                        // Fin de energia_arranque

//==========================================================
// General
//==========================================================
void energia_init(void)                  // Apaga recursos que este sistema no usa nunca
{
	ACSR  |= (1 << ACD);                 // Apaga el comparador analógico (no está en PRR)
	DIDR1 |= (1 << AIN1D) | (1 << AIN0D);    // Desconecta la entrada digital de AIN0/AIN1 (PD6/PD7)
	energia_prr_apagar(PRR_SPI);         // El SPI no se usa en este laboratorio
}                                        // Fin de energia_init

//==========================================================
// Sleep
//==========================================================
void energia_dormir(uint8_t modo)        // Duerme en Idle o ADC Noise Reduction
{
	set_sleep_mode(modo);                // Escribe SM2..SM0 en SMCR
	cli();                               // Sin interrupciones mientras se prepara el sleep
	sleep_enable();                      // SE = 1: habilita la instrucción SLEEP
	sei();                               // La instrucción siguiente a SEI siempre se ejecuta antes de atender una interrupción...
	sleep_cpu();                         // ...por eso SLEEP no se pierde aunque haya una interrupción pendiente
	sleep_disable();                     // SE = 0 al despertar (evita dormir por accidente)
}                                        // Fin de energia_dormir

void energia_power_down(void)            // Duerme en Power-Down hasta INT0/INT1 (nivel bajo), PCINT o WDT
{
	uint8_t eicra = EICRA;               // Guarda la configuración de flancos de INT0/INT1

	EICRA &= ~((1 << ISC11) | (1 << ISC10) | (1 << ISC01) | (1 << ISC00));  // INT0 e INT1 por nivel bajo: lo único que detectan sin reloj
	set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Selecciona Power-Down
	cli();                               // Sin interrupciones mientras se prepara el sleep
	sleep_enable();                      // SE = 1
	sleep_bod_disable();                 // Apaga el BOD durante el sueño (secuencia temporizada BODS/BODSE)
	sei();                               // Habilita interrupciones (SLEEP se ejecuta igual, ver energia_dormir)
	sleep_cpu();                         // Duerme; al despertar primero corre la ISR y luego sigue acá
	sleep_disable();                     // SE = 0
	EICRA = eicra;                       // Restituye la detección por flanco
	EIFR  = (1 << INTF1) | (1 << INTF0); // Borra flancos que el cambio de modo pudo haber marcado
}                                        // Fin de energia_power_down

//==========================================================
// PRR
//==========================================================
void energia_prr_apagar(uint8_t mascara) // Detiene el reloj de los periféricos indicados
{
	PRR |= (mascara & PRR_VALIDOS);      // Solo bits definidos: el bit 4 nunca se toca
}                                        // Fin de energia_prr_apagar

void energia_prr_encender(uint8_t mascara)   // Devuelve el reloj a los periféricos indicados
{
	PRR &= ~(mascara & PRR_VALIDOS);     // El periférico vuelve con la configuración que tenía al apagarse
}                                        // Fin de energia_prr_encender

uint8_t energia_prr_estado(void)         // Lectura de PRR
{
	return PRR;                          // Bit en 1 = periférico sin reloj
}                                        // Fin de energia_prr_estado

//==========================================================
// Watchdog Timer
//==========================================================
static void wdt_configurar(uint8_t control)  // Escribe WDTCSR con la secuencia temporizada
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {  // Una interrupción en medio rompería los 4 ciclos de la secuencia
		wdt_reset();                     // Arranca la nueva configuración con la cuenta en cero
		MCUSR &= ~(1 << WDRF);           // WDRF en 1 fuerza WDE en 1 (no se podría pasar a solo interrupción)
		WDTCSR = (1 << WDCE) | (1 << WDE);   // Paso 1: habilita el cambio de configuración
		WDTCSR = control;                // Paso 2 (dentro de 4 ciclos): modo e intervalo
	}                                    // Fin del bloque atómico
}                                        // Fin de wdt_configurar

static uint16_t wdt_calcular_ms(uint8_t intervalo)   // Período nominal a partir de los bits WDP
{
	static const uint16_t nominal[10] = {16, 32, 64, 125, 250, 500, 1000, 2000, 4000, 8000};  // Tabla del datasheet (VCC = 5 V)
	uint8_t n = (intervalo & 0x07) | ((intervalo >> 2) & 0x08);  // Reconstruye el índice 0..9 (WDP3 vuelve al bit 3)
	return nominal[n];                   // Período nominal en ms
}                                        // Fin de wdt_calcular_ms

void wdt_modo_interrupcion(uint8_t intervalo)    // WDIE = 1, WDE = 0
{
	wdt_ms = wdt_calcular_ms(intervalo); // Recuerda el período nominal
	wdt_configurar((1 << WDIE) | intervalo);     // Solo interrupción: el WDT nunca reinicia en este modo
}                                        // Fin de wdt_modo_interrupcion

void wdt_modo_reset(uint8_t intervalo)   // WDIE = 0, WDE = 1
{
	wdt_ms = wdt_calcular_ms(intervalo); // Recuerda el período nominal
	wdt_configurar((1 << WDE) | intervalo);      // Solo reset: si no se alimenta a tiempo, reinicia el micro
}                                        // Fin de wdt_modo_reset

void wdt_modo_combinado(uint8_t intervalo)   // WDIE = 1, WDE = 1
{
	wdt_ms = wdt_calcular_ms(intervalo); // Recuerda el período nominal
	wdt_configurar((1 << WDIE) | (1 << WDE) | intervalo);  // 1er vencimiento: ISR (el hardware borra WDIE); 2do: reset
}                                        // Fin de wdt_modo_combinado

void wdt_apagar(void)                    // Detiene el WDT
{
	wdt_ms = 0;                          // Sin período configurado
	wdt_configurar(0);                   // WDIE = 0 y WDE = 0
}                                        // Fin de wdt_apagar

ISR(WDT_vect)                            // Vencimiento del WDT en modo interrupción o combinado
{
	wdt_flag = 1;                        // Avisa al programa principal
	if (wdt_cuenta < 0xFFFF) wdt_cuenta++;   // Registra el evento (satura en 65535)
}                                        // Fin de la ISR del WDT

uint8_t wdt_disparo(void)                // Consulta y borra la bandera del WDT
{
	uint8_t f;                           // Copia local
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {  // Bloque sin interrupciones
		f = wdt_flag;                    // Copia la bandera
		wdt_flag = 0;                    // La borra
	}                                    // Fin del bloque atómico
	return f;                            // 1 = hubo interrupción
}                                        // Fin de wdt_disparo

uint16_t wdt_eventos(void)               // Total de interrupciones del WDT
{
	uint16_t n;                          // Copia local
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { n = wdt_cuenta; }  // Lectura de 2 bytes sin interrupciones
	return n;                            // Devuelve la copia
}                                        // Fin de wdt_eventos

uint16_t wdt_periodo_ms(void)            // Período nominal configurado
{
	return wdt_ms;                       // 0 si el WDT está apagado
}                                        // Fin de wdt_periodo_ms

//==========================================================
// Causa de reset
//==========================================================
uint8_t energia_causa_reset(void)        // Flags de MCUSR capturados en .init3
{
	return mcusr_copia;                  // Comparar con RESET_POR, RESET_EXT, RESET_BOR, RESET_WDT
}                                        // Fin de energia_causa_reset

//==========================================================
// Reloj del sistema
//==========================================================
void energia_reloj_div(clock_div_t div)  // Cambia el prescaler del reloj (CLKPR)
{
	clock_prescale_set(div);             // Secuencia temporizada CLKPCE + CLKPS (la hace avr/power.h sin interrupciones)
}                                        // Fin de energia_reloj_div (F_CPU NO cambia: timers, UART y _delay quedan mal escalados)