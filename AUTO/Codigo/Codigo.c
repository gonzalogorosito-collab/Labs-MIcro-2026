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
#include "pins.h"
#include "motors.h"

// ===== CONFIGURACIÓN =====
#define BAUD         9600
#define UBRR_VALOR   (F_CPU / 16 / BAUD - 1)   // 103 para 9600 baud a 16 MHz
#define VEL_INICIAL  5                         // nivel de velocidad al encender (0..9)
#define GIRO_PCT     70                        // giros al 70 % de la velocidad actual

// ===== ESTADO =====
static uint8_t nivel           = VEL_INICIAL;  // nivel de velocidad actual (0..9)
static char    movimiento      = 'P';          // último movimiento: W, A, S, D o P
static uint8_t esperando_nivel = 0;            // 1 si llegó una 'V' y falta el dígito

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

// Interpreta un carácter recibido
static void procesar(char c) {
	// --- (1) Dígito que completa un comando V ---
	if (esperando_nivel) {
		esperando_nivel = 0;
		if (c >= '0' && c <= '9') {
			nivel = c - '0';
			aplicar_movimiento();              // el cambio se aplica en el momento
			uart_send('V');
			eco(c);
			return;
		}
		// si no es dígito, la V se descarta y c se procesa como comando normal
	}

	// --- (2) Comandos de un carácter ---
	switch (c) {
		case 'W': case 'A': case 'S': case 'D': case 'P':
		movimiento = c;
		aplicar_movimiento();
		eco(c);
		break;

		case 'V':
		esperando_nivel = 1;
		break;

		case 'O': case 'C':                    // pinza: solo con el auto parado
		if (movimiento == 'P') {
			// servo_open() / servo_close() más adelante
			eco(c);
			} else {
			eco('X');
		}
		break;

		case 'E':                              // emergencia (enclavamiento próximamente)
		movimiento = 'P';
		motors_stop();
		eco('E');
		break;

		default:                               // '\r', '\n', espacios y desconocidos
		break;
	}
}

// ===== PROGRAMA PRINCIPAL =====
int main(void) {
	// --- (1) Inicialización ---
	motors_init();
	uart_init(UBRR_VALOR);


	// --- (2) Bucle principal ---
	while (1) {
		// uart_receive() es bloqueante: solo se llama si ya hay un byte (RXC0 = 1)
		if (UCSR0A & (1 << RXC0)) {
			procesar(uart_receive());
		}
	}
}