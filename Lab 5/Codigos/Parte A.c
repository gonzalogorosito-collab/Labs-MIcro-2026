//==========================================================
// Pines usados:
//   PB5 (D13) LED              PB1 Buzzer
//   PB0 Pin de prueba          PD0 / PD1 HC-05
//   PD2 (D2)  Pulsador INT0    PD3 (D3) Pulsador de emergencia INT1
//   PD4 (D4)  Sensor DHT11 (dht11_nb.h)
//   PC2 (A2)  Potenciómetro (tiempo.h)   PC4/PC5 LCD I2C
// Recursos: Timer0, Timer1, ADC (tiempo.c), Timer2 (DHT11), INT0, INT1
//==========================================================

#ifndef F_CPU                                // Si nadie definió la frecuencia del micro...
#define F_CPU 16000000UL                     // ...la definimos: 16 MHz (16.000.000 ciclos por segundo)
#endif                                       // Fin del "si no estaba definida"

#include <avr/io.h>                          // Nombres de los registros del micro (PORTB, DDRD, etc.)
#include <avr/interrupt.h>                   // Para usar interrupciones: ISR() y sei()
#include <util/atomic.h>                     // Para leer variables sin que una interrupción las cambie a la mitad
#include <stdint.h>                          // Tipos de número con tamaño fijo (uint8_t, uint32_t, etc.)
#include "tiempo.h"                          // Reloj millis(), ADC cada 100 ms y planificador de tareas
#include "lcd_nb.h"                          // Pantalla LCD que no frena el programa
#include "dht11_nb.h"                        // Sensor de temperatura y humedad que no frena el programa

//===== Hardware =====
#define LED_BIT        PB5                    // El LED está en el pin PB5 (D13)
#define BUZZER_BIT     PB1                    // El buzzer está en el pin PB1
#define TEST_BIT       PB0                    // Pin de prueba para el osciloscopio en PB0

//===== Tiempos del sistema (en milisegundos) =====
#define T_LED_MS       200                    // El LED cambia de estado cada 200 ms
#define T_LCD_MS       500                    // La pantalla se actualiza cada 500 ms
#define T_DHT_MS       2000                   // El sensor DHT11 se lee cada 2 segundos
#define T_STANDBY_MS   10000UL                // 10 s sin actividad: pasa de ACTIVO a STANDBY
#define T_APAGADO_MS   30000UL                // 30 s sin actividad (desde la última): pasa de STANDBY a APAGADO
#define T_REBOTE_MS    50                     // Ignora pulsaciones repetidas en menos de 50 ms (rebote del botón)
#define T_BEEP_MS      150                    // El buzzer suena durante 150 ms

//===== Estados de la FSM =====
typedef enum {                                 // Lista de los estados posibles (cada uno es un número)
	E_ACTIVO = 0,                            // Estado 0: funciona todo normalmente
	E_STANDBY,                               // Estado 1: en espera, muestra "Esperando..."
	E_APAGADO,                               // Estado 2: apagado, se desactiva lo que no se necesita
	E_SEGURO,                                // Estado 3: estado seguro (por emergencia)
	N_ESTADOS                                // Cantidad total de estados (vale 4)
} estado_t;                                  // Nombre del tipo: estado_t

#define CUALQUIERA     0xFF                  // Valor especial: la transición vale desde cualquier estado

//===== Tabla de transiciones =====
typedef struct {                               // Cada fila de la tabla tiene tres datos:
	uint8_t  origen;                         // Desde qué estado se evalúa (o CUALQUIERA)
	uint8_t  (*condicion)(void);             // Función que dice si hay que cambiar de estado (1 = sí)
	estado_t destino;                        // A qué estado se pasa
} transicion_t;                              // Nombre del tipo: transicion_t

//===== Variables globales =====
static estado_t estado_actual = E_ACTIVO;    // Estado en el que está el sistema ahora
static uint32_t ultima_actividad = 0;        // Momento (en ms) de la última vez que el usuario hizo algo

static volatile uint8_t  isr_boton = 0;      // La interrupción INT0 pone 1 cuando se aprieta el pulsador
static volatile uint8_t  isr_emerg = 0;      // La interrupción INT1 pone 1 cuando se aprieta emergencia
static volatile uint32_t t_boton = 0;        // Momento de la última pulsación válida de INT0
static volatile uint32_t t_emerg = 0;        // Momento de la última pulsación válida de INT1
static uint8_t ev_boton = 0;                 // 1 si en esta vuelta del programa se apretó el pulsador
static uint8_t ev_emerg = 0;                 // 1 si en esta vuelta se apretó emergencia

static uint8_t  beep_activo = 0;             // 1 si el buzzer está sonando
static uint32_t beep_fin = 0;                // Momento en que el buzzer debe dejar de sonar

static uint8_t  luz_apagar_pendiente = 0;    // 1 = falta apagar la luz de la pantalla cuando termine de escribirse "APAGADO"

//==========================================================
// Interrupciones de los pulsadores (solo avisan que pasó algo)
//==========================================================
ISR(INT0_vect)                               // Se ejecuta sola cuando se aprieta el pulsador de D2
{
	uint32_t ahora = millis();               // Guarda la hora actual
	if ((uint32_t)(ahora - t_boton) >= T_REBOTE_MS) {  // Si pasaron al menos 50 ms desde la pulsación anterior...
		isr_boton = 1;                       // ...avisa al programa principal que hubo una pulsación
		t_boton = ahora;                     // ...y guarda la hora de esta pulsación
	}                                        // Fin del filtro de rebote
}                                            // Fin de la interrupción INT0

ISR(INT1_vect)                               // Se ejecuta sola cuando se aprieta el pulsador de D3 (emergencia)
{
	uint32_t ahora = millis();               // Guarda la hora actual
	if ((uint32_t)(ahora - t_emerg) >= T_REBOTE_MS) {  // Si pasaron al menos 50 ms desde la pulsación anterior...
		isr_emerg = 1;                       // ...avisa al programa principal que hubo una emergencia
		t_emerg = ahora;                     // ...y guarda la hora de esta pulsación
	}                                        // Fin del filtro de rebote
}                                            // Fin de la interrupción INT1

//==========================================================
// Buzzer (suena un rato y se apaga solo, sin frenar el programa)
//==========================================================
static void buzzer_encender(void)            // Hace sonar el buzzer
{
	PORTB |= (1 << BUZZER_BIT);              // Pone el pin del buzzer en 1 (empieza a sonar)
	beep_fin = millis() + T_BEEP_MS;         // Calcula cuándo tiene que dejar de sonar
	beep_activo = 1;                         // Anota que el buzzer está sonando
}                                            // Fin de buzzer_encender

static void buzzer_apagar(void)              // Corta el sonido del buzzer
{
	PORTB &= ~(1 << BUZZER_BIT);             // Pone el pin del buzzer en 0 (deja de sonar)
	beep_activo = 0;                         // Anota que el buzzer ya no suena
}                                            // Fin de buzzer_apagar

static void buzzer_tarea(void)               // Se llama en cada vuelta del programa
{
	if (beep_activo && (int32_t)(millis() - beep_fin) >= 0)  // Si está sonando y ya pasó su tiempo...
	buzzer_apagar();                         // ...lo apaga
}                                            // Fin de buzzer_tarea

//==========================================================
// Luz de la pantalla (se apaga recién cuando terminó de escribirse "APAGADO")
//==========================================================
static void luz_tarea(void)                  // Se llama en cada vuelta del programa
{
	if (luz_apagar_pendiente && !lcd_nb_ocupado()) {  // Si hay que apagar la luz y la pantalla ya terminó de escribir...
		lcd_nb_backlight(0);                 // ...apaga la luz de la pantalla y deja de usar el bus I2C
		luz_apagar_pendiente = 0;            // Ya no queda nada pendiente
	}                                        // Fin de la revisión
}                                            // Fin de luz_tarea

//==========================================================
// Tareas periódicas (cortas y sin esperas)
//==========================================================
static void tarea_led(void)                  // Se ejecuta cada 200 ms
{
	PORTB ^= (1 << LED_BIT);                 // Invierte el LED: si estaba prendido lo apaga y al revés
}                                            // Fin de tarea_led

static void tarea_dht(void)                  // Se ejecuta cada 2 s
{
	dht_iniciar_lectura();                   // Le pide una medición al sensor (la termina dht_tarea() en el lazo principal)
}                                            // Fin de tarea_dht

static void tarea_lcd(void)                  // Se ejecuta cada 500 ms (solo en ACTIVO)
{
	uint8_t t, h;                            // Variables para temperatura (t) y humedad (h)
	uint8_t cod = dht_resultado(&t, &h);     // Pide los últimos valores del sensor y si la lectura salió bien

	lcd_nb_linea(0, "ACTIVO  T:   C");        // Fila de arriba: texto fijo con temperatura
	lcd_nb_numero(0, 10, t, 2);              // Escribe la temperatura en la columna 10 (2 dígitos)
	if (cod != DHT_OK) lcd_nb_texto(0, 15, "!");  // Si el sensor falló, muestra un "!" al final de la fila
	lcd_nb_linea(1, "ADC:    ");            // Fila de abajo: solo muestra el valor del ADC
	lcd_nb_numero(1, 4, adc_ultimo(), 4);    // Escribe el último valor del ADC (0 a 1023) en la columna 4
}                                            // Fin de tarea_lcd

//===== Tabla de tareas del planificador =====
#define T_LED   0                            // Posición de la tarea del LED en la tabla
#define T_LCD   1                            // Posición de la tarea de la pantalla en la tabla
#define T_DHT   2                            // Posición de la tarea del sensor en la tabla

static tarea_t tareas[] = {                  // Cada fila: { cada cuántos ms, última vez, función, activa (1) o no (0) }
{ T_LED_MS, 0, tarea_led, 1 },           // El LED cambia cada 200 ms
{ T_LCD_MS, 0, tarea_lcd, 1 },           // La pantalla se actualiza cada 500 ms
{ T_DHT_MS, 0, tarea_dht, 1 },           // El sensor se lee cada 2 s
};                                           // Fin de la tabla de tareas
#define N_TAREAS (sizeof(tareas) / sizeof(tareas[0]))  // Cantidad de tareas que hay en la tabla

static void tareas_activar(uint8_t led, uint8_t lcd, uint8_t dht)  // Prende (1) o apaga (0) cada tarea
{
	tareas[T_LED].activa = led;              // 1 = el LED parpadea, 0 = no
	tareas[T_LCD].activa = lcd;              // 1 = se actualiza la pantalla, 0 = no
	tareas[T_DHT].activa = dht;              // 1 = se lee el sensor, 0 = no
	planificador_reiniciar(tareas, N_TAREAS);// Todas las tareas empiezan a contar su tiempo desde este momento
}                                            // Fin de tareas_activar

//==========================================================
// Qué se hace al entrar en cada estado
//==========================================================
static void entrar_activo(void)              // Al entrar en ACTIVO
{
	luz_apagar_pendiente = 0;                // Cancela el apagado de la luz si todavía no se había hecho
	lcd_nb_backlight(1);                     // Enciende la luz de la pantalla (por si venía de APAGADO)
	adc_habilitar(1);                        // Vuelve a activar las lecturas del ADC
	ultima_actividad = millis();             // Reinicia el contador de inactividad
	tareas_activar(1, 1, 1);                 // Activa las tres tareas: LED, pantalla y sensor
	tarea_lcd();                             // Dibuja la pantalla ahora mismo, sin esperar los 500 ms
}                                            // Fin de entrar_activo

static void entrar_standby(void)             // Al entrar en STANDBY
{
	PORTB &= ~(1 << LED_BIT);                // Apaga el LED
	tareas_activar(0, 0, 0);                 // Desactiva las tres tareas
	lcd_nb_linea(0, "Esperando...");         // Muestra el mensaje que pide la letra
	lcd_nb_linea(1, "");                     // Deja la fila de abajo en blanco
}                                            // Fin de entrar_standby

static void entrar_apagado(void)             // Al entrar en APAGADO
{
	PORTB &= ~(1 << LED_BIT);                // Apaga el LED
	tareas_activar(0, 0, 0);                 // Desactiva las tres tareas
	adc_habilitar(0);                        // Deja de pedir lecturas al ADC
	lcd_nb_linea(0, "APAGADO");              // Escribe "APAGADO" en la fila de arriba
	lcd_nb_linea(1, "");                     // Deja la fila de abajo en blanco
	luz_apagar_pendiente = 1;                // Anota que hay que apagar la luz cuando la pantalla termine de escribir
}                                            // Fin de entrar_apagado

static void entrar_seguro(void)              // Al entrar en ESTADO SEGURO
{
	PORTB &= ~(1 << LED_BIT);                // Apaga el LED
	buzzer_apagar();                         // Apaga el buzzer
	luz_apagar_pendiente = 0;                // Cancela el apagado de la luz si todavía no se había hecho
	tareas_activar(0, 0, 0);                 // Desactiva las tres tareas
	lcd_nb_backlight(1);                     // Enciende la luz de la pantalla para que se vea el aviso
	lcd_nb_linea(0, "ESTADO SEGURO");        // Muestra el aviso en la fila de arriba
	lcd_nb_linea(1, "Pulse p/ rearmar");     // Muestra cómo salir ("p/" quiere decir "para")
}                                            // Fin de entrar_seguro

static void (*const entrada[N_ESTADOS])(void) = {  // Tabla: a cada estado le corresponde su función de entrada
	[E_ACTIVO]  = entrar_activo,             // Al entrar en ACTIVO se ejecuta entrar_activo
	[E_STANDBY] = entrar_standby,            // Al entrar en STANDBY se ejecuta entrar_standby
	[E_APAGADO] = entrar_apagado,            // Al entrar en APAGADO se ejecuta entrar_apagado
	[E_SEGURO]  = entrar_seguro,             // Al entrar en ESTADO SEGURO se ejecuta entrar_seguro
};                                           // Fin de la tabla de entradas