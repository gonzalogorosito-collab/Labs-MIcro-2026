//==========================================================
// tiempo.c - Base de tiempos, ADC periódico y planificador
//==========================================================

#include "tiempo.h"                      // Prototipos y configuración de esta librería
#include <avr/interrupt.h>               // Macro ISR() y funciones sei()/cli()
#include <util/atomic.h>                 // Macro ATOMIC_BLOCK para lecturas sin interrupciones

//===== Variables compartidas con las ISR =====
static volatile uint32_t ms_ticks   = 0; // Contador de ms desde el arranque (lo incrementa la ISR)
static volatile uint8_t  base_lenta = 0; // 1 = el tiempo avanza de a 100 ms desde Timer1
static volatile uint8_t  adc_activo = 1; // 1 = Timer1 solicita conversiones al ADC
static volatile uint8_t  adc_flag   = 0; // 1 = hay un resultado nuevo sin consultar
static volatile uint16_t adc_valor  = 0; // Último resultado de conversión (10 bits)

//==========================================================
// Inicialización
//==========================================================
void tiempo_init(void)                   // Configura los tres periféricos de la librería
{
	// --- (1) Timer0: CTC, N = 64, OCR0A = 249 -> 16 MHz / (64 * 250) = 1 kHz ---
	TCCR0A = (1 << WGM01);               // Modo CTC: el contador vuelve a 0 al llegar a OCR0A
	TCCR0B = (1 << CS01) | (1 << CS00);  // Prescaler 64: el timer cuenta a 250 kHz (4 us por cuenta)
	OCR0A  = 249;                        // 250 cuentas (0 a 249) x 4 us = 1 ms
	TIMSK0 = (1 << OCIE0A);              // Habilita la interrupción por comparación con OCR0A

	// --- (2) Timer1: CTC, N = 256, OCR1A = 6249 -> 16 MHz / (256 * 6250) = 10 Hz ---
	TCCR1A = 0;                          // Sin salidas PWM ni modos de forma de onda en el registro A
	TCCR1B = (1 << WGM12) | (1 << CS12); // Modo CTC con tope OCR1A y prescaler 256 (16 us por cuenta)
	OCR1A  = 6249;                       // 6250 cuentas (0 a 6249) x 16 us = 100 ms
	TIMSK1 = (1 << OCIE1A);              // Habilita la interrupción por comparación con OCR1A

	// --- (3) ADC: referencia AVcc, canal configurado, N = 128 (125 kHz), con interrupción ---
	ADMUX  = (1 << REFS0) | (TIEMPO_ADC_CANAL & 0x0F);  // Referencia AVcc (5 V) y selección del canal
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADC encendido, interrupción, prescaler 128
	DIDR0 |= (1 << TIEMPO_ADC_CANAL);    // Desconecta la entrada digital del pin analógico (menos consumo)
}                                        // Fin de tiempo_init

//==========================================================
// Rutinas de interrupción (trabajo mínimo)
//==========================================================
ISR(TIMER0_COMPA_vect)                   // Se ejecuta cada 1 ms (Timer0 llegó a OCR0A)
{
	ms_ticks++;                          // Avanza la base de tiempos 1 ms
}                                        // Fin de la ISR de Timer0

ISR(TIMER1_COMPA_vect)                   // Se ejecuta cada 100 ms (Timer1 llegó a OCR1A)
{
	if (base_lenta) ms_ticks += 100;     // En STANDBY Timer0 no interrumpe: el tiempo avanza de a 100 ms
	if (adc_activo && (ADCSRA & (1 << ADEN))) ADCSRA |= (1 << ADSC);  // Si el ADC está en uso y encendido, inicia una conversión
}                                        // Fin de la ISR de Timer1

ISR(ADC_vect)                            // Se ejecuta al terminar una conversión del ADC
{
	adc_valor = ADC;                     // Guarda el resultado de 10 bits (ADCL + ADCH)
	adc_flag  = 1;                       // Avisa al programa principal que hay un dato nuevo
}                                        // Fin de la ISR del ADC

//==========================================================
// Base de tiempos
//==========================================================
uint32_t millis(void)                    // Devuelve el tiempo actual en ms
{
	uint32_t t;                          // Copia local del contador
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { t = ms_ticks; }  // Lee los 4 bytes sin que una ISR lo cambie a la mitad
	return t;                            // Devuelve la copia
}                                        // Fin de millis

void tiempo_sumar_ms(uint32_t ms)        // Suma un tiempo al contador (por ejemplo, el dormido)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { ms_ticks += ms; }  // Modifica el contador sin interrupciones de por medio
}                                        // Fin de tiempo_sumar_ms

void tiempo_base_rapida(void)            // Vuelve a la base de 1 ms (estado ACTIVO)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {  // Bloque sin interrupciones
		base_lenta = 0;                  // Timer1 deja de sumar 100 ms
		TCNT0  = 0;                      // Reinicia la cuenta de Timer0
		TIFR0  = (1 << OCF0A);           // Borra una comparación pendiente (se borra escribiendo 1)
		TIMSK0 |= (1 << OCIE0A);         // Vuelve a habilitar la interrupción de 1 ms
	}                                    // Fin del bloque atómico
}                                        // Fin de tiempo_base_rapida

void tiempo_base_lenta(void)             // Pasa a la base de 100 ms (estado STANDBY)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {  // Bloque sin interrupciones
		TIMSK0 &= ~(1 << OCIE0A);        // Timer0 deja de despertar a la CPU cada 1 ms
		base_lenta = 1;                  // Timer1 pasa a sumar 100 ms en cada interrupción
	}                                    // Fin del bloque atómico
}                                        // Fin de tiempo_base_lenta

//==========================================================
// ADC periódico
//==========================================================
void adc_habilitar(uint8_t on)           // Activa o desactiva el disparo periódico del ADC
{
	adc_activo = on;                     // La ISR de Timer1 consulta esta variable
}                                        // Fin de adc_habilitar

uint8_t adc_nuevo_dato(uint16_t *valor)  // Consulta si hay una conversión nueva
{
	uint8_t hay;                         // Copia local de la bandera
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {  // Bloque sin interrupciones
		hay = adc_flag;                  // Copia la bandera
		adc_flag = 0;                    // La borra: el dato ya fue consultado
		*valor = adc_valor;              // Copia el último valor convertido
	}                                    // Fin del bloque atómico
	return hay;                          // 1 = el valor es nuevo, 0 = es el mismo de antes
}                                        // Fin de adc_nuevo_dato

uint16_t adc_ultimo(void)                // Devuelve el último valor sin tocar la bandera
{
	uint16_t v;                          // Copia local del valor
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = adc_valor; }  // Lee los 2 bytes sin interrupciones de por medio
	return v;                            // Devuelve la copia
}                                        // Fin de adc_ultimo

//==========================================================
// Planificador cooperativo
//==========================================================
void planificador_ejecutar(tarea_t *tareas, uint8_t n)   // Recorre la tabla y ejecuta las tareas vencidas
{
	uint32_t ahora = millis();           // Toma el tiempo actual una sola vez por pasada

	for (uint8_t i = 0; i < n; i++) {    // Recorre las n tareas de la tabla
		if (tareas[i].activa && (uint32_t)(ahora - tareas[i].ultimo_ms) >= tareas[i].periodo_ms) {  // Activa y con el período cumplido (la resta es segura ante el desborde)
			tareas[i].ultimo_ms += tareas[i].periodo_ms;                         // Agenda la próxima ejecución sin acumular error
			if ((uint32_t)(ahora - tareas[i].ultimo_ms) >= tareas[i].periodo_ms) // Si sigue atrasada más de un período (por ejemplo, tras dormir)...
				tareas[i].ultimo_ms = ahora;                                     // ...se resincroniza para no ejecutarla varias veces seguidas
			tareas[i].funcion();         // Ejecuta la tarea (debe ser corta y no bloquear)
		}                                // Fin de la condición de ejecución
	}                                    // Fin del recorrido de la tabla
}                                        // Fin de planificador_ejecutar

void planificador_reiniciar(tarea_t *tareas, uint8_t n)  // Toma el instante actual como punto de partida
{
	uint32_t ahora = millis();           // Tiempo actual
	for (uint8_t i = 0; i < n; i++) tareas[i].ultimo_ms = ahora;  // Todas las tareas cuentan su período desde ahora
}                                        // Fin de planificador_reiniciar