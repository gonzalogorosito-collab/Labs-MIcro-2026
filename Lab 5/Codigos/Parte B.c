// Conexiones:
//   D2 (INT0) pulsador a GND (hace sonar el buzzer)
//   D9 buzzer activo (actuador 2)
//   D13 LED: parpadea cada 200 ms (actuador 1)
//==========================================================

#define F_CPU 16000000UL                 

#include <avr/io.h>                      // Nombres de registros y bits del ATmega328P
#include <avr/interrupt.h>               // Macro ISR() y funciones sei()/cli()
#include <stdint.h>                      // Tipos enteros de tamaño fijo (uint8_t, uint16_t, uint32_t)
#include "tiempo.h"                      // millis(), ADC periódico y planificador de tareas

//==========================================================
// Configuración
//==========================================================
#define T_LED_MS        200              // El LED cambia de estado cada 200 ms (Problema A)
#define T_BEEP_MS       150UL            // Duración del beep del buzzer al apretar el pulsador (Problema A)
#define ANTIREBOTE_MS   300UL            // Tiempo en que se ignoran nuevos flancos de INT0 (rebotes del pulsador)

//===== Pines (todos en PORTB salvo los pulsadores en PORTD) =====
#define LED_ACT   PB5                    // D13: LED actuador 1
#define BUZZER    PB1                    // D9: buzzer actuador 2
#define BOTON     PD2                    // D2: pulsador local (INT0)

//==========================================================
// Variables globales
//==========================================================
static uint32_t t_ultimo_int0 = 0;       // Instante del último flanco de INT0 aceptado (antirrebote)
static volatile uint8_t flag_int0 = 0;   // La ISR de INT0 lo pone en 1; el lazo principal lo atiende
static uint8_t  led = 1;                 // Actuador 1: 1 = el LED parpadea (por defecto), 0 = LED apagado
static uint8_t  beep_activo = 0;         // 1 = el buzzer está sonando por el beep del pulsador
static uint32_t beep_fin = 0;            // Instante (ms) en que termina el beep
static uint8_t  buzzer = 0;              // Estado deseado del buzzer (actuador 2)

//==========================================================
// Interrupción externa INT0 (pulsador local)
//==========================================================
ISR(INT0_vect)                           // Se ejecuta al apretar el pulsador
{
	flag_int0 = 1;                       // Solo avisa: el trabajo se hace en el lazo principal, no en la ISR
}                                        // Fin de la ISR de INT0

//==========================================================
// Funciones auxiliares
//==========================================================
static void aplicar_actuadores(void)     // Lleva los pines al estado deseado de LED y buzzer
{
	if (!led) PORTB &= ~(1 << LED_ACT);  // Parpadeo desactivado: el LED queda apagado (si está activado lo mueve la tarea_led)
	if (buzzer || beep_activo) PORTB |= (1 << BUZZER); else PORTB &= ~(1 << BUZZER);   // Buzzer encendido (beep) o apagado
}                                        // Fin de aplicar_actuadores

//==========================================================
// Tareas periódicas (las ejecuta el planificador)
//==========================================================
static void tarea_led(void)              // Cada 200 ms: hace parpadear el LED (tarea periódica del Problema A)
{
	if (led) PORTB ^= (1 << LED_ACT);    // Si el parpadeo está activado, invierte el LED
	else PORTB &= ~(1 << LED_ACT);       // Si no, lo deja apagado
}                                        // Fin de tarea_led

//===== Tabla de tareas: { período en ms, último instante, función, activa } =====
static tarea_t tareas[] = {
	{ T_LED_MS, 0, tarea_led, 1 },       // T_LED: cada 200 ms (parpadeo del LED)
};
#define N_TAREAS (sizeof(tareas) / sizeof(tareas[0]))   // Cantidad de tareas de la tabla

static void atender_int0(void)           // Procesa el flanco del pulsador (fuera de la ISR)
{
	if (!flag_int0) return;              // No hubo pulsación
	flag_int0 = 0;                       // Consume el aviso
	uint32_t ahora = millis();           // Instante actual
	if ((uint32_t)(ahora - t_ultimo_int0) < ANTIREBOTE_MS) return;   // Rebote del pulsador: se ignora
	t_ultimo_int0 = ahora;               // Recuerda el instante de esta pulsación
	beep_activo = 1;                     // La pulsación activa el buzzer (requisito 5 del Problema A)
	beep_fin = millis() + T_BEEP_MS;     // Instante en que el beep debe terminar
	aplicar_actuadores();                // Enciende el buzzer
}                                        // Fin de atender_int0

static void beep_tarea(void)             // Apaga el buzzer cuando termina el beep (sin bloquear)
{
	if (beep_activo && (int32_t)(millis() - beep_fin) >= 0) {   // Si está sonando y ya pasó su tiempo...
		beep_activo = 0;                 // ...el beep termina...
		aplicar_actuadores();            // ...y el buzzer vuelve a su valor deseado
	}                                    // Fin del control del beep
}                                        // Fin de beep_tarea

//==========================================================
// Programa principal
//==========================================================
int main(void)                           // Punto de entrada
{
	DDRB |= (1 << LED_ACT) | (1 << BUZZER);  // LED y buzzer como salidas
	DDRD &= ~(1 << BOTON);              // Pulsador como entrada
	PORTD |= (1 << BOTON);              // Con resistencia de pull-up interna (apretado = 0)
	EICRA = (EICRA & ~(1 << ISC00)) | (1 << ISC01);  // INT0 por flanco descendente
	EIFR = (1 << INTF0);                 // Borra un posible flag pendiente
	EIMSK |= (1 << INT0);                // Habilita INT0

	tiempo_init();                       // Timer0 (1 ms), Timer1 (100 ms) y ADC
	sei();                               // Habilita las interrupciones globales
	t_ultimo_int0 = millis() - ANTIREBOTE_MS;    // Permite aceptar la primera pulsación de D2

	for (;;) {                           // Lazo principal
		atender_int0();                  // Procesa el pulsador D2
		beep_tarea();                    // Termina el beep del buzzer cuando corresponde
		planificador_ejecutar(tareas, N_TAREAS);     // Ejecuta las tareas periódicas vencidas
	}                                    // Fin del lazo principal
}                                        // Fin de main
