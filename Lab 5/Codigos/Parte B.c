// Conexiones:
//   D0 (RX) <- TXD del HC-05
//   D1 (TX) -> RXD del HC-05 (con divisor de 1k/2k, el HC-05 es de 3,3 V)
//   D2 (INT0) pulsador a GND (despierta y hace sonar el buzzer)
//   D4 DATA del DHT11
//   D9 buzzer activo (actuador 2)
//   D8 pin de prueba (osciloscopio): pulso en cada despertar del WDT (Prueba A)
//   D13 LED: parpadea cada 200 ms en ACTIVO (actuador 1: la aplicación activa o desactiva el parpadeo)
//   A2 potenciómetro o sensor analógico (ADC2)
//   A4 (SDA) y A5 (SCL) a la LCD I2C
//==========================================================

#define F_CPU 16000000UL                 

#include <avr/io.h>                      // Nombres de registros y bits del ATmega328P
#include <avr/interrupt.h>               // Macro ISR() y funciones sei()/cli()
#include <stdint.h>                      // Tipos enteros de tamaño fijo (uint8_t, uint16_t, uint32_t)
#include <stddef.h>                      // Define NULL para las tablas de punteros
#include "tiempo.h"                      // millis(), ADC periódico y planificador de tareas
#include "energia.h"                     // Sleep, PRR, Watchdog y causa de reset
#include "uart_rb.h"                     // USART por interrupciones con buffers circulares
#include "trama.h"                       // Protocolo de tramas para el Bluetooth
#include "lcd_nb.h"                      // LCD 16x2 I2C sin bloqueo
#include "dht11_nb.h"                    // Sensor DHT11 sin bloqueo

//==========================================================
// Configuración
//==========================================================
#define T_STANDBY_MS    10000UL          // Inactividad para pasar de ACTIVO a STANDBY: 10 s
#define T_APAGADO_MS    30000UL          // Inactividad (desde el último evento válido) para pasar a APAGADO: 30 s
#define TEMP_ALARMA     35               // Temperatura (°C) a partir de la cual se registra una alarma
#define T_LED_MS        200              // El LED cambia de estado cada 200 ms (Problema A)
#define T_BEEP_MS       150UL            // Duración del beep del buzzer al apretar el pulsador (Problema A)
#define ANTIREBOTE_MS   300UL            // Tiempo en que se ignoran nuevos flancos de INT0 (rebotes del pulsador)

//===== Pines (todos en PORTB salvo los pulsadores en PORTD) =====
#define LED_ACT   PB5                    // D13: LED actuador 1
#define BUZZER    PB1                    // D9: buzzer actuador 2
#define LED_WDT   PB0                    // D8: pin de prueba para el osciloscopio (pulso en cada despertar del WDT)
#define BOTON     PD2                    // D2: pulsador local (INT0)

//===== Estados =====
#define E_ACTIVO   0                     // Estado ACTIVO: todo funcionando
#define E_STANDBY  1                     // Estado STANDBY: Idle entre tareas
#define E_APAGADO  2                     // Estado APAGADO: Power-Down
#define N_ESTADOS  3                     // Cantidad de estados

//===== Códigos de evento (byte alto del DATO de RESP_EVENTO; el byte bajo es un valor) =====
#define EV_TEMP_ALTA     1               // Alarma: temperatura sobre el límite (valor = °C)
#define EV_WDT_DESPERTAR 5               // Informativo: despertar por WDT (valor = número de evento)
#define EV_CAMBIO_ESTADO 6               // Informativo: cambio de estado (valor = estado nuevo)

//===== Índices de la tabla de tareas =====
#define T_DHT       0                    // Tarea que inicia la lectura del DHT11
#define T_SENSORES  1                    // Tarea que toma el ADC y controla las alarmas
#define T_LCD       2                    // Tarea que actualiza el texto de la LCD
#define T_LED       3                    // Tarea que hace parpadear el LED cada 200 ms

//==========================================================
// Variables globales
//==========================================================
static uint8_t  estado = E_ACTIVO;       // Estado actual de la máquina de estados
static uint32_t t_actividad = 0;         // Instante (ms) del último evento válido (pulsador o trama válida)
static uint32_t t_ultimo_int0 = 0;       // Instante del último flanco de INT0 aceptado (antirrebote)
static volatile uint8_t flag_int0 = 0;   // La ISR de INT0 lo pone en 1; el lazo principal lo atiende
static uint8_t  led = 1;                 // Actuador 1: 1 = el LED parpadea en ACTIVO (por defecto), 0 = LED apagado
static uint8_t  beep_activo = 0;         // 1 = el buzzer está sonando por el beep del pulsador
static uint32_t beep_fin = 0;            // Instante (ms) en que termina el beep
static uint8_t  buzzer = 0;              // Estado deseado del buzzer (actuador 2)
static uint8_t  led_guardado = 0;        // Copia del LED al entrar a un estado de bajo consumo
static uint8_t  buzzer_guardado = 0;     // Copia del buzzer al entrar a un estado de bajo consumo
static uint8_t  hay_contexto = 0;        // 1 = hay una copia guardada para restaurar al volver a ACTIVO
static uint8_t  temperatura = 0;         // Última temperatura válida del DHT11 (°C)
static uint16_t alarmas_total = 0;       // Cantidad de alarmas registradas desde el arranque
static uint8_t  ultima_alarma = 0;       // Código de la última alarma registrada
static uint8_t  alarma_temp_activa = 0;  // 1 = la alarma de temperatura ya fue avisada (evita repetirla)

//===== Tipo de función sin argumentos, para las tablas =====
typedef void (*fn_t)(void);              // Puntero a función que no recibe ni devuelve nada
typedef void (*cmd_t)(uint16_t);         // Puntero a función de comando: recibe el DATO de la trama

//===== Prototipos (las tablas necesitan conocer las funciones antes de definirlas) =====
static void entrar_activo(void);         // Acciones al entrar a ACTIVO
static void entrar_standby(void);        // Acciones al entrar a STANDBY
static void entrar_apagado(void);        // Acciones al entrar a APAGADO
static void ejecutar_activo(void);       // Trabajo de cada vuelta en ACTIVO
static void ejecutar_standby(void);      // Trabajo de cada vuelta en STANDBY
static void ejecutar_apagado(void);      // Trabajo de cada vuelta en APAGADO
static void dormir_nada(void);           // No duerme (ACTIVO)
static void dormir_idle(void);           // Duerme en Idle (STANDBY)
static void dormir_power_down(void);     // Duerme en Power-Down (APAGADO)
static void cambiar_estado(uint8_t nuevo);   // Transición entre estados

//===== Tablas de la máquina de estados (una posición por estado) =====
static const fn_t tabla_entrar[N_ESTADOS]   = { entrar_activo, entrar_standby, entrar_apagado };      // Al entrar
static const fn_t tabla_ejecutar[N_ESTADOS] = { ejecutar_activo, ejecutar_standby, ejecutar_apagado }; // En cada vuelta
static const fn_t tabla_dormir[N_ESTADOS]   = { dormir_nada, dormir_idle, dormir_power_down };            // Al final de cada vuelta

//==========================================================
// Interrupción externa INT0 (pulsador local)
//==========================================================
ISR(INT0_vect)                           // Se ejecuta al apretar el pulsador (también despierta de Power-Down)
{
	flag_int0 = 1;                       // Solo avisa: el trabajo se hace en el lazo principal, no en la ISR
}                                        // Fin de la ISR de INT0

//==========================================================
// Funciones auxiliares
//==========================================================
static void enviar(uint8_t cmd, uint16_t dato)   // Envía una trama solo si la USART tiene reloj
{
	if (energia_prr_estado() & PRR_USART0) return;   // USART apagada por PRR: no se puede transmitir
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

static void guardar_contexto(void)       // Guarda el estado necesario para continuar al despertar
{
	led_guardado = led;                  // Copia el LED (las variables quedan en la SRAM, que se conserva al dormir)
	buzzer_guardado = buzzer;            // Copia el buzzer
	hay_contexto = 1;                    // Marca que hay algo para restaurar
}                                        // Fin de guardar_contexto

static void tareas_activar(uint8_t dht, uint8_t sensores, uint8_t lcd, uint8_t ledt);  // Prototipo (usa la tabla de tareas)

static void usart_dormir(void)           // Apaga la USART respetando el orden obligatorio
{
	while (!uart_tx_terminado());        // 1) Espera que salga todo lo que estaba en el buffer de transmisión
	uart_habilitar(0);                   // 2) Apaga el receptor y el transmisor
	energia_prr_apagar(PRR_USART0);      // 3) Recién ahora se quita el reloj con PRR
}                                        // Fin de usart_dormir

static void adc_dormir(void)             // Apaga el ADC respetando el orden obligatorio
{
	adc_habilitar(0);                    // 1) Timer1 deja de pedir conversiones
	ADCSRA &= ~(1 << ADEN);              // 2) Apaga el ADC
	energia_prr_apagar(PRR_ADC);         // 3) Quita el reloj del ADC
}                                        // Fin de adc_dormir

static void restaurar_perifericos(void)  // Vuelve a dejar todo disponible (orden inverso al de apagado)
{
	energia_prr_encender(PRR_TIM0 | PRR_TIM1 | PRR_TIM2 | PRR_TWI | PRR_ADC | PRR_USART0);  // Primero se da reloj a todos
	uart_habilitar(1);                   // Después se enciende la USART (RX, TX e interrupción de RX)
	ADCSRA |= (1 << ADEN);               // Se enciende el ADC
	adc_habilitar(1);                    // Timer1 vuelve a pedir conversiones
	tiempo_base_rapida();                // Vuelve la base de 1 ms con Timer0
	lcd_nb_backlight(1);                 // Se enciende la luz de la LCD y se refresca el texto
	wdt_modo_reset(WDT_1S);              // El WDT vuelve a supervisar en modo reset (1 s)
}                                        // Fin de restaurar_perifericos

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

static const char *const nombre_estado[N_ESTADOS] = { "ACTIVO", "STANDBY", "APAGADO" };  // Textos para la LCD

static void tarea_lcd(void)              // Cada 500 ms: actualiza el buffer de la LCD
{
	lcd_nb_linea(0, nombre_estado[estado]);      // Fila 0: nombre del estado
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

static void tareas_activar(uint8_t dht, uint8_t sensores, uint8_t lcd, uint8_t ledt)  // Activa o suspende cada tarea
{
	tareas[T_DHT].activa = dht;          // Tarea del DHT11
	tareas[T_SENSORES].activa = sensores;    // Tarea de sensores
	tareas[T_LCD].activa = lcd;          // Tarea de la LCD
	tareas[T_LED].activa = ledt;         // Tarea del parpadeo del LED
	planificador_reiniciar(tareas, N_TAREAS);    // Todas cuentan su período desde ahora
}                                        // Fin de tareas_activar

//==========================================================
// Máquina de estados: entrada a cada estado
//==========================================================
static void entrar_activo(void)          // ACTIVO: tareas periódicas, sensores, LCD y Bluetooth
{
	restaurar_perifericos();             // Todos los periféricos que ACTIVO necesita vuelven a funcionar
	dht_init();                          // Timer2 vuelve a tener reloj: se reconfigura el sensor
	if (hay_contexto) {                  // Si veníamos de STANDBY o APAGADO...
		led = led_guardado;              // ...se recupera el LED...
		buzzer = buzzer_guardado;        // ...y el buzzer como estaban
		hay_contexto = 0;                // La copia ya fue usada
	}                                    // Fin de la restauración del contexto
	aplicar_actuadores();                // Los pines reflejan el estado deseado
	t_actividad = millis();              // Reinicia la cuenta de inactividad
	tareas_activar(1, 1, 1, 1);          // En ACTIVO corren las cuatro tareas (incluido el parpadeo del LED)
	tarea_lcd();                         // Escribe el texto ya, sin esperar 500 ms
}                                        // Fin de entrar_activo

static void entrar_standby(void)         // STANDBY: Idle entre tareas, LCD con "Esperando..."
{
	guardar_contexto();                  // Guarda el estado necesario para continuar
	while (dht_ocupado()) dht_tarea();   // Si hay una lectura del DHT11 en curso, espera que termine (máx. 20 ms)
	PORTB &= ~(1 << LED_ACT);            // El LED queda apagado mientras dura STANDBY
	tareas_activar(0, 0, 0, 0);          // En STANDBY no corre ninguna tarea periódica
	adc_dormir();                        // El ADC no se necesita en STANDBY
	tiempo_base_lenta();                 // Timer0 deja de interrumpir cada 1 ms: el tiempo avanza de a 100 ms con Timer1
	energia_prr_apagar(PRR_TIM0 | PRR_TIM2);     // Timer0 (ya sin uso) y Timer2 (solo el DHT11) sin reloj
	lcd_nb_linea(0, "Esperando...");     // Mensaje pedido en la letra
	lcd_nb_linea(1, "");                 // Fila 1 en blanco
}                                        // Fin de entrar_standby

static void entrar_apagado(void)         // APAGADO: se apaga todo lo que no hace falta y se duerme en Power-Down
{
	guardar_contexto();                  // Guarda el estado necesario para continuar
	buzzer = 0;                          // Apaga el buzzer
	beep_activo = 0;                     // Cancela un beep en curso
	PORTB &= ~((1 << LED_ACT) | (1 << BUZZER));  // LED y buzzer quedan en bajo mientras dura el sueño
	evento(EV_CAMBIO_ESTADO, E_APAGADO); // Avisa a la aplicación antes de cortar la USART
	while (dht_ocupado()) dht_tarea();   // Termina una lectura del DHT11 en curso
	tareas_activar(0, 0, 0, 0);          // Ninguna tarea periódica
	usart_dormir();                      // USART apagada por PRR: en Power-Down no recibe de todos modos
	adc_dormir();                        // ADC apagado
	lcd_nb_linea(0, "APAGADO");          // Último texto de la LCD
	lcd_nb_linea(1, "");                 // Fila 1 en blanco
	while (lcd_nb_ocupado()) lcd_nb_tarea();     // Termina de enviar el texto antes de apagar la luz
	lcd_nb_backlight(0);                 // Apaga la retroiluminación y deja de usar el bus
	energia_prr_apagar(PRR_TWI);         // TWI sin reloj (después de apagar la LCD)
	tiempo_base_lenta();                 // Timer0 sin interrupción
	energia_prr_apagar(PRR_TIM0 | PRR_TIM1 | PRR_TIM2);  // Los tres timers sin reloj: el tiempo se compensa con tiempo_sumar_ms()
	wdt_modo_interrupcion(WDT_8S);       // WDT en modo interrupción, 8 s: despierta de Power-Down (nunca reinicia)
}                                        // Fin de entrar_apagado

//==========================================================
// Máquina de estados: trabajo de cada vuelta
//==========================================================
static void ejecutar_activo(void)        // ACTIVO: controla la inactividad
{
	if ((uint32_t)(millis() - t_actividad) >= T_STANDBY_MS) cambiar_estado(E_STANDBY);  // 10 s sin actividad: STANDBY
}                                        // Fin de ejecutar_activo

static void ejecutar_standby(void)       // STANDBY: controla cuándo pasar a APAGADO
{
	if ((uint32_t)(millis() - t_actividad) >= T_APAGADO_MS) cambiar_estado(E_APAGADO);  // 30 s desde el último evento válido
}                                        // Fin de ejecutar_standby

static void lectura_rapida_wdt(void)     // Prueba A: lectura rápida tras despertar por el WDT
{
	PORTB |= (1 << LED_WDT);             // Enciende el LED indicador (se apaga al terminar)
	energia_prr_encender(PRR_ADC);       // 1) Primero se da reloj al ADC: no se asume que siga operativo
	ADCSRA |= (1 << ADEN);               // 2) Se enciende el ADC
	ADCSRA |= (1 << ADSC);               // 3) Se inicia una conversión
	while (ADCSRA & (1 << ADSC)) { }     // 4) Espera ~104 us hasta que termine (bloqueo muy corto y acotado)
	uint16_t v = ADC;                    // 5) Lee el resultado de 10 bits
	ADCSRA &= ~(1 << ADEN);              // 6) Apaga el ADC...
	energia_prr_apagar(PRR_ADC);         //    ...y le quita el reloj
	energia_prr_encender(PRR_USART0);    // Para informar, se da reloj a la USART un momento
	uart_habilitar(1);                   // Se enciende la USART
	evento(EV_WDT_DESPERTAR, wdt_eventos() & 0xFF);  // Registra el evento (número de despertar)
	enviar(RESP_ADC, v);                 // Envía la lectura
	usart_dormir();                      // Espera el fin del envío y vuelve a apagar la USART
	PORTB &= ~(1 << LED_WDT);            // Apaga el LED indicador
}                                        // Fin de lectura_rapida_wdt

static void ejecutar_apagado(void)       // APAGADO: atiende los despertares del WDT
{
	if (wdt_disparo()) {                 // Si el despertar fue por el WDT...
		tiempo_sumar_ms(wdt_periodo_ms());   // ...se compensa el tiempo dormido (los timers estaban detenidos)
		lectura_rapida_wdt();            // ...se hace la lectura rápida y se informa
	}                                    // Fin del despertar por WDT (después de esto vuelve a Power-Down)
}                                        // Fin de ejecutar_apagado

//==========================================================
// Máquina de estados: dormir al final de cada vuelta
//==========================================================
static void dormir_nada(void)            // ACTIVO no duerme
{
}                                        // Fin de dormir_nada

static void dormir_idle(void)            // STANDBY: la CPU se detiene hasta la próxima interrupción
{
	energia_dormir(DORMIR_IDLE);         // Despierta con Timer1 (cada 100 ms), la USART (comando) o INT0 (pulsador)
}                                        // Fin de dormir_idle

static void dormir_power_down(void)      // APAGADO: sueño profundo
{
	energia_power_down();                // Despierta solo con INT0 (nivel bajo), PCINT o WDT
}                                        // Fin de dormir_power_down

//==========================================================
// Transiciones
//==========================================================
static void cambiar_estado(uint8_t nuevo)    // Cambia de estado y ejecuta las acciones de entrada
{
	if (nuevo >= N_ESTADOS) return;      // Estado inexistente: se ignora
	estado = nuevo;                      // Registra el estado nuevo
	tabla_entrar[nuevo]();               // Ejecuta la función de entrada de ese estado (sin switch-case)
	if (nuevo != E_APAGADO) evento(EV_CAMBIO_ESTADO, nuevo);  // Notifica (APAGADO ya notificó antes de cortar la USART)
}                                        // Fin de cambiar_estado

static void atender_int0(void)           // Procesa el flanco del pulsador (fuera de la ISR)
{
	if (!flag_int0) return;              // No hubo pulsación
	flag_int0 = 0;                       // Consume el aviso
	uint32_t ahora = millis();           // Instante actual
	if ((uint32_t)(ahora - t_ultimo_int0) < ANTIREBOTE_MS) return;   // Rebote del pulsador: se ignora
	t_ultimo_int0 = ahora;               // Recuerda el instante de esta pulsación
	t_actividad = ahora;                 // Es un evento válido: reinicia la inactividad
	if (estado == E_STANDBY || estado == E_APAGADO) cambiar_estado(E_ACTIVO);   // Despierta y vuelve a ACTIVO
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
static void cmd_estado(uint16_t dato)    // 0x01: consulta el estado
{
	(void)dato;                          // El dato no se usa
	enviar(RESP_ACK(CMD_ESTADO), estado);    // Responde con el número de estado
}                                        // Fin de cmd_estado

static void cmd_sensores(uint16_t dato)  // 0x02: pide las lecturas (ADC y temperatura)
{
	(void)dato;                          // El dato no se usa
	enviar(RESP_ADC, adc_ultimo());      // Último valor del ADC
	enviar(RESP_TEMP, temperatura);      // Última temperatura válida
}                                        // Fin de cmd_sensores

static void cmd_cambiar(uint16_t dato)   // 0x03: cambia entre ACTIVO (0), STANDBY (1) y APAGADO (2)
{
	if (dato > E_APAGADO) { trama_enviar_error(ERR_RECHAZADO); return; }     // Solo se permiten los 3 estados normales
	enviar(RESP_ACK(CMD_CAMBIAR), dato); // Confirma antes de cambiar (APAGADO corta la USART)
	cambiar_estado((uint8_t)dato);       // Hace la transición
}                                        // Fin de cmd_cambiar

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
	cmd_estado,                          // 0x01
	cmd_sensores,                        // 0x02
	cmd_cambiar,                         // 0x03
	cmd_actuador,                        // 0x04
	NULL,                                // 0x05: no implementado
	cmd_alarmas,                         // 0x06
	NULL,                                // 0x07: no implementado
};

static void procesar_trama(void)         // Lee una trama (si hay) y la despacha, fuera de las ISR
{
	if (energia_prr_estado() & PRR_USART0) return;   // USART apagada: no hay nada que leer
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
	uint8_t es_consulta = (t.cmd == CMD_ESTADO || t.cmd == CMD_SENSORES || t.cmd == CMD_ALARMAS);   // Consultas: la app las repite sola cada pocos segundos
	if (!es_consulta) {                  // Solo las órdenes de la app cuentan como actividad (las consultas automáticas no)
		t_actividad = millis();          // Una orden válida es un evento válido: reinicia la inactividad
		if (estado == E_STANDBY && t.cmd != CMD_CAMBIAR) cambiar_estado(E_ACTIVO);   // Una orden despierta de STANDBY (CMD_CAMBIAR hace su propia transición)
	}                                    // Fin del tratamiento de actividad
	tabla_cmd[t.cmd](t.dato);            // Ejecuta el comando (sin switch-case)
}                                        // Fin de procesar_trama

//==========================================================
// Programa principal
//==========================================================
int main(void)                           // Punto de entrada
{
	DDRB |= (1 << LED_ACT) | (1 << BUZZER) | (1 << LED_WDT);     // LED, buzzer y LED del WDT como salidas
	DDRD &= ~(1 << BOTON);              // Pulsador como entrada
	PORTD |= (1 << BOTON);              // Con resistencia de pull-up interna (apretado = 0)
	EICRA = (EICRA & ~(1 << ISC00)) | (1 << ISC01);  // INT0 por flanco descendente
	EIFR = (1 << INTF0);                 // Borra un posible flag pendiente
	EIMSK |= (1 << INT0);                // Habilita INT0

	energia_init();                      // Apaga el comparador analógico y el SPI (no se usan)
	tiempo_init();                       // Timer0 (1 ms), Timer1 (100 ms) y ADC
	uart_init(UART_UBRR(9600));          // USART a 9600 bps, 8N1, con interrupciones y buffers circulares
	sei();                               // Habilita las interrupciones globales
	lcd_nb_init();                       // Inicia la LCD (bloquea ~1 s; el WDT todavía no está armado)
	t_ultimo_int0 = millis() - ANTIREBOTE_MS;    // Permite aceptar la primera pulsación de D2

	cambiar_estado(E_ACTIVO);            // Arranca en ACTIVO (con el LED parpadeando)

	for (;;) {                           // Lazo principal
		if (estado != E_APAGADO) wdt_alimentar();    // Supervisión: reinicia la cuenta del WDT (en APAGADO el WDT es solo despertador)
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
		tabla_ejecutar[estado]();        // Trabajo propio del estado actual
		lcd_nb_tarea();                  // Envía a la LCD unos pocos caracteres pendientes
		tabla_dormir[estado]();          // Duerme según el estado (ACTIVO no duerme)
	}                                    // Fin del lazo principal
}                                        // Fin de main
