//==========================================================
// uart_rb.h - USART0 por interrupciones con buffers circulares
// Laboratorio 5 - Tecnologías de Microprocesamiento
//==========================================================
// RX : la ISR guarda cada byte recibido en un buffer circular
// TX : uart_send() deja el byte en un buffer y la ISR UDRE lo envía
// Ninguna función espera a la USART: el programa nunca se bloquea
// Reemplaza a UART.c/h de la cátedra (mismos nombres: no usar ambas)
//==========================================================

#ifndef UART_RB_H                        // Evita incluir este archivo dos veces
#define UART_RB_H                        // Marca el archivo como ya incluido

#ifndef F_CPU                            // Si la frecuencia no fue definida antes...
#define F_CPU 16000000UL                 // ...se fija en 16 MHz
#endif                                   // Fin de la definición condicional de F_CPU

#include <avr/io.h>                      // Registros y bits del ATmega328P
#include <stdint.h>                      // Tipos enteros de tamaño fijo

//===== Configuración =====
#define UART_UBRR(baud)  ((F_CPU / 16UL / (baud)) - 1)  // Valor de UBRR0 en modo normal (U2X0 = 0)
#define UART_RX_TAM      64              // Tamaño del buffer de recepción (potencia de 2)
#define UART_TX_TAM      128             // Tamaño del buffer de transmisión (potencia de 2)

//===== Inicialización y control =====
void     uart_init(unsigned int ubrr);   // Configura 8N1, habilita RX/TX e interrupción de recepción
void     uart_habilitar(uint8_t on);     // 1 = USART en uso, 0 = RX/TX apagados (antes de usar PRR)

//===== Recepción =====
uint8_t  uart_disponible(void);          // Cantidad de bytes esperando en el buffer de recepción
uint8_t  uart_leer(uint8_t *c);          // Saca un byte del buffer: 1 = había dato, 0 = vacío
void     uart_vaciar_rx(void);           // Descarta todo lo recibido

//===== Transmisión =====
uint8_t  uart_send(char c);              // Encola un byte: 1 = encolado, 0 = buffer lleno (se descarta)
void     uart_print(const char *s);      // Encola una cadena terminada en '\0'
void     uart_print_hex(uint8_t val);    // Encola un byte como dos dígitos hexadecimales
void     uart_print_u16(uint16_t val);   // Encola un número sin signo en decimal
uint8_t  uart_tx_libre(void);            // Lugares libres en el buffer de transmisión
uint8_t  uart_tx_terminado(void);        // 1 = buffer vacío y último byte ya salió por el pin

//===== Diagnóstico =====
uint8_t  uart_errores(void);             // Cantidad de errores (trama, sobreescritura, buffer lleno)

#endif                                   // Fin de la protección contra inclusión doble