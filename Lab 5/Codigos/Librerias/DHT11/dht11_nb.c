//==========================================================
// dht11_nb.c - Lectura del DHT11 sin bloquear el arranque de 18 ms
//==========================================================

#include "dht11_nb.h"                    // Pin, tiempos y prototipos de esta librería
#include "tiempo.h"                      // millis() para el pulso de inicio y el intervalo mínimo

#define DHT_MASK  (1 << DHT_BIT)         // Máscara del pin dentro del puerto

//===== Estado de la librería =====
static uint8_t  en_arranque = 0;         // 1 = la línea está en bajo esperando los 20 ms
static uint32_t t_inicio = 0;            // Instante en que se bajó la línea
static uint32_t t_ultima = 0;            // Instante de la última lectura terminada
static uint8_t  temperatura = 0;         // Última temperatura válida (°C)
static uint8_t  humedad = 0;             // Última humedad válida (%)
static uint8_t  resultado = DHT_SIN_DATO;    // Código de la última lectura

//==========================================================
// Inicialización
//==========================================================
void dht_init(void)                      // Deja el sensor en reposo
{
	DHT_DDR  &= ~DHT_MASK;               // Pin como entrada (el sensor maneja la línea)
	DHT_PORT |=  DHT_MASK;               // Pull-up interno (además de la resistencia del módulo)
	TCCR2A = 0;                          // Timer2 en modo normal, sin salidas
	TCCR2B = 0;                          // Timer2 detenido hasta que se lea
	t_ultima = millis() - DHT_INTERVALO_MIN;     // Permite leer apenas arranca
}                                        // Fin de dht_init

//==========================================================
// Inicio de lectura (no bloquea)
//==========================================================
uint8_t dht_iniciar_lectura(void)        // Baja la línea y vuelve enseguida
{
	if (en_arranque) return 0;           // Ya hay una lectura en curso
	if ((uint32_t)(millis() - t_ultima) < DHT_INTERVALO_MIN) return 0;  // Respeta 1 s entre lecturas
	DHT_PORT &= ~DHT_MASK;               // Nivel bajo (primero, para no dar un pulso alto)
	DHT_DDR  |=  DHT_MASK;               // Pin como salida: la línea queda en 0
	t_inicio = millis();                 // Empieza a contar los 20 ms
	en_arranque = 1;                     // Lectura en curso
	return 1;                            // Lectura iniciada
}                                        // Fin de dht_iniciar_lectura

//==========================================================
// Medición de pulsos con Timer2
//==========================================================
static uint8_t esperar_nivel(uint8_t nivel)  // Espera que la línea tome "nivel"; devuelve las cuentas o 0xFF
{
	TCNT2 = 0;                           // Empieza a medir desde cero
	while (((DHT_PINR & DHT_MASK) ? 1 : 0) != nivel) {   // Mientras la línea no tenga el nivel buscado
		if (TCNT2 >= DHT_TIMEOUT_TICKS) return 0xFF;     // Más de 100 us: el sensor no respondió
	}                                    // Fin de la espera
	return TCNT2;                        // Duración del nivel anterior en cuentas de 0,5 us
}                                        // Fin de esperar_nivel

static uint8_t leer_bits(uint8_t *d)     // Ráfaga de ~4 ms: respuesta del sensor + 40 bits
{
	uint8_t t;                           // Duración del pulso alto de cada bit

	DHT_DDR  &= ~DHT_MASK;               // Suelta la línea (entrada)...
	DHT_PORT |=  DHT_MASK;               // ...con pull-up: sube a 1 y el sensor toma el control
	TCCR2B = (1 << CS21);                // Arranca Timer2 con prescaler 8: 2 MHz -> 0,5 us por cuenta

	if (esperar_nivel(0) == 0xFF) return DHT_ERR_RESPUESTA;   // El sensor baja la línea (20-40 us)
	if (esperar_nivel(1) == 0xFF) return DHT_ERR_RESPUESTA;   // Fin de su pulso bajo de 80 us
	if (esperar_nivel(0) == 0xFF) return DHT_ERR_RESPUESTA;   // Fin de su pulso alto de 80 us

	for (uint8_t i = 0; i < 40; i++) {   // 40 bits: humedad (2 bytes), temperatura (2), checksum (1)
		if (esperar_nivel(1) == 0xFF) return DHT_ERR_TIMEOUT; // Fin del bajo de 50 us que separa los bits
		t = esperar_nivel(0);            // Mide el pulso alto
		if (t == 0xFF) return DHT_ERR_TIMEOUT;                // Pulso alto demasiado largo
		d[i / 8] <<= 1;                  // Hace lugar para el bit nuevo (MSB primero)
		if (t > DHT_UMBRAL_TICKS) d[i / 8] |= 1;              // Alto mayor a 40 us: el bit es 1
	}                                    // Fin de la lectura de bits

	if ((uint8_t)(d[0] + d[1] + d[2] + d[3]) != d[4]) return DHT_ERR_CHECKSUM;  // Verificación del sensor
	return DHT_OK;                       // Datos válidos
}                                        // Fin de leer_bits

//==========================================================
// Tarea del lazo principal
//==========================================================
uint8_t dht_tarea(void)                  // Completa la lectura cuando pasaron los 20 ms
{
	uint8_t d[5] = {0, 0, 0, 0, 0};      // Bytes recibidos

	if (!en_arranque) return 0;          // No hay lectura en curso
	if ((uint32_t)(millis() - t_inicio) < DHT_ARRANQUE_MS) return 0;  // Todavía no pasaron los 20 ms

	resultado = leer_bits(d);            // Ráfaga acotada de ~4 ms (las interrupciones siguen habilitadas)
	TCCR2B = 0;                          // Detiene Timer2 (sin reloj, no consume)
	DHT_DDR  &= ~DHT_MASK;               // Asegura la línea en entrada...
	DHT_PORT |=  DHT_MASK;               // ...con pull-up, también si hubo error

	if (resultado == DHT_OK) {           // Si la lectura fue válida...
		humedad     = d[0];              // ...byte 0: humedad entera (%)
		temperatura = d[2];              // ...byte 2: temperatura entera (°C)
	}                                    // Fin de la actualización de valores
	t_ultima = millis();                 // Cuenta el intervalo mínimo desde ahora
	en_arranque = 0;                     // Lectura terminada
	return 1;                            // Hay resultado nuevo
}                                        // Fin de dht_tarea

uint8_t dht_ocupado(void)                // Lectura en curso
{
	return en_arranque;                  // 1 mientras la línea está en bajo esperando
}                                        // Fin de dht_ocupado

uint8_t dht_resultado(uint8_t *temp, uint8_t *hum)   // Entrega los últimos valores válidos
{
	*temp = temperatura;                 // Temperatura (°C)
	*hum  = humedad;                     // Humedad (%)
	return resultado;                    // DHT_OK o el código de error de la última lectura
}                                        // Fin de dht_resultado