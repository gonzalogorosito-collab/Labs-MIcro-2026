#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define PCF8574 0x27          // Dirección I2C del expansor para la pantalla LCD
#include "twi_lcd.h"          // Incluye la LCD I2C y automáticamente el archivo twi.c / twi.h

#define BAUD 9600
#define MY_UBRR ((F_CPU / 16 / BAUD) - 1)

// Definición de pines físicos según los requerimientos del sistema
#define MOVIMIENTO_PIN PD3  // Botón de movimiento en Pin 3 (con resistencia pull-down externa a GND)
#define LED_ARMADO     PD4  // LED verde de sistema armado (Pin 4)
#define LED_ALARMA     PD5  // LED rojo de alarma (Pin 5)
#define BUZZER         PD6  // Buzzer activo / sirena (Pin 6)

// Pulsadores locales para control sin Bluetooth
#define BTN_ARMAR    PB0  // Alterna armar / desarmar (comandos A / D)
#define BTN_SILENCIO PB1  // Silencia la sirena
#define BTN_MODO     PB2  // Alterna modo automático / manual
#define BTN_FORZAR   PB3  // Fuerza la alarma en modo manual
#define BTN_MASK     ((1 << BTN_ARMAR) | (1 << BTN_SILENCIO) | (1 << BTN_MODO) | (1 << BTN_FORZAR))

// Inicialización del puerto serie por hardware para el módulo Bluetooth HC-05
void USART_Init(void) {
	UBRR0H = (MY_UBRR >> 8);
	UBRR0L = MY_UBRR;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void USART_Transmit(unsigned char data) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

void USART_Send_String(const char *str) {
	while (*str) {
		USART_Transmit(*str++);
	}
}

// Inicialización del Conversor Analógico-Digital (Canal ADC2 / Pin PC2 para la LDR)
void ADC_Init(void) {
	ADMUX = (1 << REFS0); // Referencia AVCC
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t ADC_Read(uint8_t channel) {
	channel &= 0x07;
	ADMUX = (ADMUX & 0xF8) | channel;
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

// Variables de estado globales con volatile
volatile uint8_t sistema_armado = 1;      // 1: Armado, 0: Desarmado
volatile uint8_t modo_auto = 1;            // 1: Modo Automático, 0: Modo Manual
volatile uint8_t alarma_activa = 0;
volatile uint8_t sirena_silenciada = 0;

// Variables de control de estado para evitar parpadeos en la LCD
int8_t ultimo_estado_lcd = -1; 
uint8_t ventana_abierta = 0; 

int main(void) {
	// Configuración de E/S en Puerto D
	DDRD |= (1 << LED_ARMADO) | (1 << LED_ALARMA) | (1 << BUZZER);
	
	// Configurar PD3 como entrada (con resistencia pull-down externa a GND)
	DDRD &= ~(1 << MOVIMIENTO_PIN);
	PORTD &= ~(1 << MOVIMIENTO_PIN); 

	DDRB &= ~BTN_MASK;  // Pulsadores como entradas
	PORTB |= BTN_MASK;  // Pull-up interna para los botones de control

	twi_init();          
	twi_lcd_init();     
	
	twi_lcd_clear();
	_delay_ms(10);
	twi_lcd_msg("INICIO");
	_delay_ms(1500);    
	twi_lcd_clear();
	_delay_ms(10);
	twi_lcd_msg("ARMADO");
	ultimo_estado_lcd = 0;
	
	USART_Init();      
	ADC_Init();        
	sei();               

	USART_Send_String("SISTEMA LISTO\r\n");

	while(1) {
		// 1. Lectura de la LDR con Histéresis limpia:
		uint16_t val_ldr = ADC_Read(2);
		if (val_ldr > 450) {
			ventana_abierta = 1; 
		} else if (val_ldr < 250) {
			ventana_abierta = 0; 
		}
		
		// 2. Lectura del botón de movimiento en Pin 3 (PD3)
		uint8_t movimiento = ((PIND & (1 << MOVIMIENTO_PIN)) != 0);

		_delay_ms(50);
	}
}