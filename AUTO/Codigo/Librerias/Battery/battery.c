#include <avr/io.h>
#include "battery.h"

// ===== ESTADO =====
static uint16_t mv_filtrado = 0;
static uint32_t ultima      = 0;

// Conversión simple (~104 µs con prescaler 128)
static uint16_t leer_adc(void) {
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

static uint16_t medir_mv(void) {
	return (uint32_t)leer_adc() * BAT_VREF_MV * BAT_DIVISOR / 1023;
}

void battery_init(void) {
	// --- (1) Referencia AVcc y canal de la batería ---
	ADMUX = (1 << REFS0) | BAT_CANAL;

	// --- (2) ADC habilitado, prescaler 128 -> 125 kHz ---
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

	// --- (3) Desactiva la entrada digital del pin analógico ---
	DIDR0 = (1 << BAT_CANAL);

	// --- (4) La primera conversión se descarta; arranca el filtro con una medida real ---
	leer_adc();
	mv_filtrado = medir_mv();
}

void battery_update(uint32_t ahora) {
	if (ahora - ultima < BAT_PERIODO_MS) return;
	ultima = ahora;

	// Promedio móvil exponencial (7/8 anterior + 1/8 nueva): suaviza las caídas por los motores
	mv_filtrado = ((uint32_t)mv_filtrado * 7 + medir_mv()) / 8;
}

uint16_t battery_mv(void) {
	return mv_filtrado;
}

uint8_t battery_pct(void) {
	if (mv_filtrado <= BAT_MV_VACIA) return 0;
	if (mv_filtrado >= BAT_MV_LLENA) return 100;
	return (uint32_t)(mv_filtrado - BAT_MV_VACIA) * 100 / (BAT_MV_LLENA - BAT_MV_VACIA);
}