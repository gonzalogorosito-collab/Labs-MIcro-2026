// ===== MAPEO DE PINES =====
// D0/D1  (PD0/PD1) -> USART: RX/TX del HC-06, 9600 baud
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
#include "motors.h"
#include "timer.h"
#include "servo.h"

// ===== CONFIGURACIÓN =====
#define BAUD           9600
#define UBRR_VALOR     (F_CPU / 16 / BAUD - 1)   // 103 para 9600 baud a 16 MHz
#define VEL_INICIAL    5                         // nivel de velocidad al encender (0..9)
#define GIRO_PCT       70                        // giros al 70 % de la velocidad actual
#define FAILSAFE_MS    400                       // tiempo máximo sin comandos en movimiento
#define CALIBRAR_SERVO 1                         // 1: habilita los comandos + y - (calibración)
#define PASO_CAL_US    50                        // paso de calibración del servo

// ===== ESTADO =====
static uint8_t  nivel           = VEL_INICIAL; // nivel de velocidad actual (0..9)
static char     movimiento      = 'P';         // último movimiento: W, A, S, D o P
static uint8_t  esperando_nivel = 0;           // 1 si llegó una 'V' y falta el dígito
static uint8_t  emergencia      = 0;           // 1 si está enclavada la parada de emergencia
static uint32_t ultimo_cmd      = 0;           // millis() del último comando válido

// ===== FUNCIONES =====

// Duty para un nivel: PWM_MIN + n*(255-PWM_MIN)/9  ->  V0 = PWM_MIN, V9 = 255
static uint8_t duty_de_nivel(uint8_t n) {
	return PWM_MIN + (uint16_t)n * (255 - PWM_MIN) / 9;
}

// Duty de giro: 70 % del duty actual, nunca menos que PWM_MIN
static uint8_t duty_giro(void) {
	uint8_t d = (uint16_t)duty_de_nivel(nivel) * GIRO_PCT / 100;
	if (d < PWM_MIN) d = PWM_MIN;
	return d;
}

// Aplica a los motores el movimiento guardado con el nivel actual
static void aplicar_movimiento(void) {
	int16_t d = duty_de_nivel(nivel);
	int16_t g = duty_giro();

	switch (movimiento) {
		case 'W': motors_set( d,  d); break;   // adelante
		case 'S': motors_set(-d, -d); break;   // atrás
		case 'A': motors_set(-g,  g); break;   // gira a la izquierda en el lugar
		case 'D': motors_set( g, -g); break;   // gira a la derecha en el lugar
		default:  motors_stop();      break;   // P
	}
}

// Envía un carácter seguido de '\n'
static void eco(char c) {
	uart_send(c);
	uart_send('\n');
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

// Interpreta un carácter recibido
static void procesar(char c) {
	// --- (1) Emergencia enclavada: solo se acepta R ---
	if (emergencia) {
		if (c == 'R') {
			emergencia      = 0;
			esperando_nivel = 0;
			movimiento      = 'P';           // queda frenado; el nivel se conserva
			ultimo_cmd      = millis();
			eco('R');
		}
		return;                              // todo lo demás se ignora en silencio
	}

	// --- (2) Dígito que completa un comando V ---
	if (esperando_nivel) {
		esperando_nivel = 0;
		if (c >= '0' && c <= '9') {
			nivel = c - '0';
			aplicar_movimiento();            // el cambio se aplica en el momento
			ultimo_cmd = millis();
			uart_send('V');
			eco(c);
			return;
		}
		// si no es dígito, la V se descarta y c se procesa como comando normal
	}

	// --- (3) Comandos de un carácter ---
	switch (c) {
		case 'W': case 'A': case 'S': case 'D': case 'P':
		ultimo_cmd = millis();
		if (c != 'P' && servo_moving()) {  // la pinza termina su barrido antes de arrancar
			eco('X');
			break;
		}
		movimiento = c;
		aplicar_movimiento();
		eco(c);
		break;

		case 'V':
		esperando_nivel = 1;
		break;

		case 'O': case 'C':                  // pinza: solo con el auto parado
		ultimo_cmd = millis();
		if (movimiento == 'P') {
			if (c == 'O') servo_open();
			else          servo_close();
			eco(c);
			} else {
			eco('X');
		}
		break;

		#if CALIBRAR_SERVO
		case '+': case '-':                  // calibración: mueve la pinza de a PASO_CAL_US
		ultimo_cmd = millis();
		if (movimiento == 'P') {
			if (c == '+') servo_set_us(servo_get_us() + PASO_CAL_US);
			else          servo_set_us(servo_get_us() - PASO_CAL_US);
			enviar_numero(servo_get_us());
			} else {
			eco('X');
		}
		break;
		#endif

		case 'E':                            // emergencia: frena, congela la pinza y enclava
		emergencia = 1;
		movimiento = 'P';
		motors_stop();
		servo_hold();
		eco('E');
		break;

		default:                             // '\r', '\n', espacios, R sin emergencia y desconocidos
		break;
	}
}

// ===== PROGRAMA PRINCIPAL =====
int main(void) {
	// --- (1) Inicialización ---
	motors_init();
	servo_init();
	uart_init(UBRR_VALOR);
	timer_init();
	sei();                                   // habilita las interrupciones globales (tick de 1 ms)

	// --- (2) Bucle principal ---
	while (1) {
		// --- (2a) Recepción: uart_receive() es bloqueante, solo se llama si hay un byte ---
		if (UCSR0A & (1 << RXC0)) {
			procesar(uart_receive());
		}

		// --- (2b) Failsafe: en movimiento y sin comandos durante FAILSAFE_MS -> frena ---
		// la resta sin signo da el tiempo correcto aunque millis() desborde
		if (movimiento != 'P' && (millis() - ultimo_cmd) >= FAILSAFE_MS) {
			movimiento = 'P';
			motors_stop();
			eco('F');
		}

		// --- (2c) Barrido suave de la pinza ---
		servo_update();
	}
}