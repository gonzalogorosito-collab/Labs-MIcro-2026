// Conexiones:
//   D0 (RX) <- TXD del HC-05
//   D1 (TX) -> RXD del HC-05 (con divisor de 1k/2k, el HC-05 es de 3,3 V)
//   D2 (INT0) pulsador a GND (hace sonar el buzzer)
//   D4 DATA del DHT11
//   D9 buzzer activo (actuador 2)
//   D13 LED: parpadea cada 200 ms (actuador 1: la aplicación activa o desactiva el parpadeo)
//   A2 potenciómetro o sensor analógico (ADC2)
//   A4 (SDA) y A5 (SCL) a la LCD I2C
//==========================================================

#define F_CPU 16000000UL                 

#include <avr/io.h>                      // Nombres de registros y bits del ATmega328P
#include <avr/interrupt.h>               // Macro ISR() y funciones sei()/cli()
#include <stdint.h>                      // Tipos enteros de tamaño fijo (uint8_t, uint16_t, uint32_t)
#include <stddef.h>                      // Define NULL para las tablas de punteros
#include "tiempo.h"                      // millis(), ADC periódico y planificador de tareas
#include "uart_rb.h"                     // USART por interrupciones con buffers circulares
#include "trama.h"                       // Protocolo de tramas para el Bluetooth
#include "lcd_nb.h"                      // LCD 16x2 I2C sin bloqueo
#include "dht11_nb.h"                    // Sensor DHT11 sin bloqueo

//==========================================================
// Configuración
//==========================================================
#define TEMP_ALARMA     35               // Temperatura (°C) a partir de la cual se registra una alarma
#define T_LED_MS        200              // El LED cambia de estado cada 200 ms (Problema A)
#define T_BEEP_MS       150UL            // Duración del beep del buzzer al apretar el pulsador (Problema A)
#define ANTIREBOTE_MS   300UL            // Tiempo en que se ignoran nuevos flancos de INT0 (rebotes del pulsador)

//===== Pines (todos en PORTB salvo los pulsadores en PORTD) =====
#define LED_ACT   PB5                    // D13: LED actuador 1
#define BUZZER    PB1                    // D9: buzzer actuador 2
#define BOTON     PD2                    // D2: pulsador local (INT0)

//===== Códigos de evento (byte alto del DATO de RESP_EVENTO; el byte bajo es un valor) =====
#define EV_TEMP_ALTA     1               // Alarma: temperatura sobre el límite (valor = °C)

//==========================================================
// Variables globales
//==========================================================
static uint32_t t_ultimo_int0 = 0;       // Instante del último flanco de INT0 aceptado (antirrebote)
static volatile uint8_t flag_int0 = 0;   // La ISR de INT0 lo pone en 1; el lazo principal lo atiende
static uint8_t  led = 1;                 // Actuador 1: 1 = el LED parpadea (por defecto), 0 = LED apagado
static uint8_t  beep_activo = 0;         // 1 = el buzzer está sonando por el beep del pulsador
static uint32_t beep_fin = 0;            // Instante (ms) en que termina el beep
static uint8_t  buzzer = 0;              // Estado deseado del buzzer (actuador 2)
static uint8_t  temperatura = 0;         // Última temperatura válida del DHT11 (°C)
static uint16_t alarmas_total = 0;       // Cantidad de alarmas registradas desde el arranque
static uint8_t  ultima_alarma = 0;       // Código de la última alarma registrada
static uint8_t  alarma_temp_activa = 0;  // 1 = la alarma de temperatura ya fue avisada (evita repetirla)

//===== Tipo de función de comando, para la tabla =====
typedef void (*cmd_t)(uint16_t);         // Puntero a función de comando: recibe el DATO de la trama

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
static void enviar(uint8_t cmd, uint16_t dato)   // Envía una trama a la aplicación
{
	trama_enviar(cmd, dato);             // Arma y encola la trama (con checksum)
}                                        // Fin de enviar

static void evento(uint8_t codigo, uint8_t valor)    // Envía una notificación de evento a la aplicación
{
	enviar(RESP_EVENTO, ((uint16_t)codigo << 8) | valor);   // Byte alto = código, byte bajo = valor
}                                        // Fin de evento

static void registrar_alarma(uint8_t codigo, uint8_t valor)  // Guarda una alarma y la notifica
{
	alarmas_total++;                     // Cuenta una alarma más
	ultima_alarma = codigo;              // Recuerda cuál fue la última
	evento(codigo, valor);               // Avisa a la aplicación si hay comunicación
}                                        // Fin de registrar_alarma

static void aplicar_actuadores(void)     // Lleva los pines al estado deseado de LED y buzzer
{
	if (!led) PORTB &= ~(1 << LED_ACT);  // Parpadeo desactivado: el LED queda apagado (si está activado lo mueve la tarea_led)
	if (buzzer || beep_activo) PORTB |= (1 << BUZZER); else PORTB &= ~(1 << BUZZER);   // Buzzer encendido (comando o beep) o apagado
}                                        // Fin de aplicar_actuadores

//==========================================================
// Tareas periódicas (las ejecuta el planificador)
//==========================================================
static void tarea_led(void)              // Cada 200 ms: hace parpadear el LED (tarea periódica del Problema A)
{
	if (led) PORTB ^= (1 << LED_ACT);    // Si el parpadeo está activado, invierte el LED
	else PORTB &= ~(1 << LED_ACT);       // Si no, lo deja apagado
}                                        // Fin de tarea_led

static void tarea_dht(void)              // Cada 2 s: pide una lectura al DHT11
{
	dht_iniciar_lectura();               // Baja la línea y vuelve enseguida (no bloquea)
}                                        // Fin de tarea_dht

static void tarea_sensores(void)         // Cada 500 ms: controla la temperatura
{
	if (temperatura > TEMP_ALARMA) {     // Temperatura sobre el límite
		if (!alarma_temp_activa) {       // Si todavía no se avisó...
			alarma_temp_activa = 1;      // ...se marca como avisada...
			registrar_alarma(EV_TEMP_ALTA, temperatura);  // ...y se registra la alarma una sola vez
		}                                // Fin del aviso único
		} else {                             // Temperatura normal
		alarma_temp_activa = 0;          // Se rearma el aviso para la próxima vez
	}                                    // Fin del control de temperatura
}                                        // Fin de tarea_sensores

static void tarea_lcd(void)              // Cada 500 ms: actualiza el buffer de la LCD
{
	lcd_nb_linea(0, "Sensores");      // Fila 0: título
	lcd_nb_linea(1, "ADC      T   C"); // Fila 1: plantilla con las etiquetas (solo ADC y temperatura)
	lcd_nb_numero(1, 4, adc_ultimo(), 4);        // Valor del ADC (columnas 4 a 7)
	lcd_nb_numero(1, 11, temperatura, 2);        // Temperatura (columnas 11 y 12)
}                                        // Fin de tarea_lcd

//===== Tabla de tareas: { período en ms, último instante, función, activa } =====
static tarea_t tareas[] = {
	{ 2000, 0, tarea_dht,     1 },       // T_DHT: cada 2 s (el DHT11 pide al menos 1 s entre lecturas)
	{ 500,  0, tarea_sensores, 1 },      // T_SENSORES: cada 500 ms
	{ 500,  0, tarea_lcd,     1 },       // T_LCD: cada 500 ms (no en cada vuelta, para no refrescar sin parar)
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
		aplicar_actuadores();            // ...y el buzzer vuelve al valor pedido por la aplicación
	}                                    // Fin del control del beep
}                                        // Fin de beep_tarea

//==========================================================
// Comandos Bluetooth (se validan antes de modificar el sistema)
//==========================================================
static void cmd_sensores(uint16_t dato)  // 0x02: pide las lecturas (ADC y temperatura)
{
	(void)dato;                          // El dato no se usa
	enviar(RESP_ADC, adc_ultimo());      // Último valor del ADC
	enviar(RESP_TEMP, temperatura);      // Última temperatura válida
}                                        // Fin de cmd_sensores

static void cmd_actuador(uint16_t dato)  // 0x04: byte alto = 1 LED (parpadeo) / 2 buzzer; byte bajo = 0 apagar / 1 encender
{
	uint8_t cual = dato >> 8;            // Qué actuador
	uint8_t valor = dato & 0xFF;         // Qué valor
	if (valor > 1 || cual < 1 || cual > 2) {   // Validación: valor 0/1, actuador 1/2
		trama_enviar_error(ERR_RECHAZADO);   // Comando no permitido
		return;                          // No modifica nada
	}                                    // Fin de la validación
	if (cual == 1) led = valor; else buzzer = valor;     // Guarda el estado deseado
	aplicar_actuadores();                // Lo aplica a los pines
	enviar(RESP_ACK(CMD_ACTUADOR), dato);    // Confirma
}                                        // Fin de cmd_actuador

static void cmd_alarmas(uint16_t dato)   // 0x06: consulta los eventos de alarma
{
	(void)dato;                          // El dato no se usa
	enviar(RESP_ACK(CMD_ALARMAS), alarmas_total);    // Cantidad de alarmas
	evento(ultima_alarma, 0);            // Código de la última alarma
}                                        // Fin de cmd_alarmas

//===== Tabla de comandos: la posición es el código del comando (NULL = no existe) =====
static const cmd_t tabla_cmd[8] = {
	NULL,                                // 0x00: no existe
	NULL,                                // 0x01: no implementado
	cmd_sensores,                        // 0x02
	NULL,                                // 0x03: no implementado
	cmd_actuador,                        // 0x04
	NULL,                                // 0x05: no implementado
	cmd_alarmas,                         // 0x06
	NULL,                                // 0x07: no implementado
};

static void procesar_trama(void)         // Lee una trama (si hay) y la despacha, fuera de las ISR
{
	trama_t t;                           // Trama recibida
	int8_t r = trama_recibir(&t);        // Revisa el buffer de recepción
	if (r == TRAMA_NADA) return;         // Todavía no hay una trama completa
	if (r < 0) {                         // Error de recepción (checksum, formato o trama incompleta)
		trama_enviar_error((uint16_t)(-r));      // Informa el error a la aplicación
		return;                          // No se cambia nada
	}                                    // Fin del tratamiento de errores
	if (t.cmd >= 8 || tabla_cmd[t.cmd] == NULL) {    // Comando desconocido
		trama_enviar_error(ERR_DESCONOCIDO);     // Informa el error
		return;                          // No se cambia nada
	}                                    // Fin del tratamiento de comandos desconocidos
	tabla_cmd[t.cmd](t.dato);            // Ejecuta el comando (sin switch-case)
}                                        // Fin de procesar_trama

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
	ADCSRA |= (1 << ADEN);               // Enciende el ADC
	adc_habilitar(1);                    // Timer1 pide las conversiones del ADC
	dht_init();                          // Configura Timer2 y el pin del sensor
	uart_init(UART_UBRR(9600));          // USART a 9600 bps, 8N1, con interrupciones y buffers circulares
	sei();                               // Habilita las interrupciones globales
	lcd_nb_init();                       // Inicia la LCD (bloquea ~1 s)
	t_ultimo_int0 = millis() - ANTIREBOTE_MS;    // Permite aceptar la primera pulsación de D2

	for (;;) {                           // Lazo principal
		atender_int0();                  // Procesa el pulsador D2
		beep_tarea();                    // Termina el beep del buzzer cuando corresponde
		procesar_trama();                // Procesa comandos Bluetooth (fuera de las ISR)
		if (dht_tarea()) {               // Si terminó una lectura del DHT11...
			uint8_t t, h;                // Valores leídos (la humedad no se usa en este sistema)
			if (dht_resultado(&t, &h) == DHT_OK) {   // ...y fue correcta...
				temperatura = t;         // ...se guarda la temperatura
			}                            // Fin de la lectura correcta
		}                                // Fin de la atención del DHT11
		planificador_ejecutar(tareas, N_TAREAS);     // Ejecuta las tareas periódicas vencidas
		lcd_nb_tarea();                  // Envía a la LCD unos pocos caracteres pendientes
	}                                    // Fin del lazo principal
}                                        // Fin de main
