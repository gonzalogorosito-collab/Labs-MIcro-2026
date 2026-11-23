// ===== MAPEO DE PINES =====
// D0/D1  (PD0/PD1) -> USART: RX/TX del HC-05, 9600 baud
// D6     (PD6/OC0A)-> ENA (PWM motor izquierdo, Timer0)
// D5     (PD5/OC0B)-> ENB (PWM motor derecho, Timer0)
// D7     (PD7)     -> IN1 (motor izquierdo)
// D8     (PB0)     -> IN2 (motor izquierdo)
// D12    (PB4)     -> IN3 (motor derecho)
// D13    (PB5)     -> IN4 (motor derecho)
// D9     (PB1/OC1A)-> servo SG90 (Timer1)
// D4     (PD4)     -> sensor IR al piso
// A1     (ADC1)    -> lectura de batería
// Timer2           -> tick de 1 ms

#include "UART.h"      // define F_CPU
#include <avr/interrupt.h>
#include "pins.h"
#include "protocol.h"
#include "motors.h"
#include "timer.h"
#include "servo.h"

// ===== CONFIGURACIÓN =====
#define BAUD           9600
#define UBRR_VALOR     (F_CPU / 16 / BAUD - 1)   // 103 para 9600 baud a 16 MHz
#define FAILSAFE_MS    300                       // tiempo máximo sin tramas en movimiento
#define ECO_BYTE       0x55                      // respuesta a cada trama válida (latencia)
#define CALIBRAR_SERVO 1                         // 1: habilita PINZA con D1 = 2 (+) y D1 = 3 (−)
#define PASO_CAL_US    50                        // paso de calibración del servo

// ===== ESTADO =====
static uint8_t  emergencia = 0;    // 1 si está enclavada la parada de emergencia
static uint8_t  parado     = 1;    // 1 si los motores están frenados (habilita la pinza)
static uint32_t ultimo_cmd = 0;    // millis() de la última trama válida

// ===== FUNCIONES =====

// Limita un valor al rango de motors_set()
static int16_t limitar(int16_t x) {
	if (x >  255) return  255;
	if (x < -255) return -255;
	return x;
}

// Escala −100..+100 a −255..+255
static int16_t escalar(int16_t x) {
	return (int16_t)((int32_t)x * 255 / 100);
}

// Mezcla diferencial: v > 0 avanza, v < 0 retrocede; w > 0 gira a la izquierda, w < 0 a la derecha
static void mover(int16_t v, int16_t w) {
	// --- (1) Orden de quieto ---
	if (v == 0 && w == 0) {
		motors_stop();
		parado = 1;
		return;
	}

	// --- (2) La pinza termina su barrido antes de arrancar ---
	if (servo_moving()) {
		motors_stop();
		parado = 1;
		return;                              // la próxima trama MOVER (50 ms) arranca sola
	}

	// --- (3) Aplica la mezcla a los motores ---
	motors_set(limitar(escalar(v - w)), limitar(escalar(v + w)));
	parado = 0;
}

#if CALIBRAR_SERVO
// Envía un número en decimal seguido de '\n'
static void enviar_numero(uint16_t n) {
	char buf[5];                             // uint16_t tiene hasta 5 dígitos (65535)
	uint8_t i = 0;

	// --- (1) Extrae los dígitos de menor a mayor peso (n % 10 = último dígito) ---
	do {
		buf[i++] = '0' + (n % 10);
		n /= 10;
	} while (n > 0);

	// --- (2) Los envía en orden inverso (de mayor a menor peso) ---
	while (i > 0) {
		uart_send(buf[--i]);
	}
	uart_send('\n');
}
#endif

// Ejecuta una trama válida
static void procesar(const trama_t *t) {
	// --- (1) Emergencia enclavada: solo se acepta el rearme ---
	if (emergencia) {
		if (t->cmd == CMD_EMERG && t->d1 == 0) {
			emergencia = 0;                  // queda frenado hasta el próximo MOVER
			parado     = 1;
		}
		return;
	}

	// --- (2) Comandos ---
	switch (t->cmd) {
		case CMD_EMERG:                      // emergencia: frena, congela la pinza y enclava
		if (t->d1 == 1) {
			emergencia = 1;
			motors_stop();
			servo_hold();
			parado = 1;
		}
		break;

		case CMD_MOVER:                      // movimiento: D1 = v + 100, D2 = w + 100
		mover((int16_t)t->d1 - 100, (int16_t)t->d2 - 100);
		break;

		case CMD_PINZA:                      // pinza: solo con el auto parado
		if (!parado) break;
		switch (t->d1) {
			case 0: servo_open();  break;
			case 1: servo_close(); break;
			#if CALIBRAR_SERVO
			case 2: servo_set_us(servo_get_us() + PASO_CAL_US); enviar_numero(servo_get_us()); break;
			case 3: servo_set_us(servo_get_us() - PASO_CAL_US); enviar_numero(servo_get_us()); break;
			#endif
			default: break;
		}
		break;

		case CMD_MODO:                       // no se usa: la velocidad la da el deslizador
		default:
		break;
	}
}

// ===== PROGRAMA PRINCIPAL =====
int main(void) {
	// --- (1) Inicialización ---
	motors_init();
	motors_stop();
	servo_init();
	uart_init(UBRR_VALOR);
	timer_init();
	sei();                                   // habilita interrupciones (RX de la UART y tick de 1 ms)

	trama_t t;

	// --- (2) Bucle principal ---
	while (1) {
		// --- (2a) Recepción: el parser arma las tramas desde el buffer circular ---
		while (proto_poll(&t)) {
			uart_send(ECO_BYTE);             // eco apenas llega: mide solo la latencia del enlace
			ultimo_cmd = millis();
			procesar(&t);
		}

		// --- (2b) Failsafe: en movimiento y sin tramas durante FAILSAFE_MS -> frena ---
		// la resta sin signo da el tiempo correcto aunque millis() desborde
		if (!parado && (millis() - ultimo_cmd) >= FAILSAFE_MS) {
			motors_stop();
			parado = 1;
		}

		// --- (2c) Barrido suave de la pinza ---
		servo_update();
	}
}	