//==========================================================
// lcd_nb.h - LCD 16x2 I2C (PCF8574) sin bloqueo, sobre twi_lcd.h
// Laboratorio 5 - Tecnologías de Microprocesamiento
//==========================================================
// El programa escribe en un buffer de 2x16 en RAM (instantáneo)
// lcd_nb_tarea() pasa el buffer a la LCD de a pocos caracteres
// por llamada, así ninguna llamada bloquea más de ~1,5 ms
// Usa la librería de la cátedra: twi_lcd.h + twi.h (+ twi.c)
//==========================================================

#ifndef LCD_NB_H                         // Evita incluir este archivo dos veces
#define LCD_NB_H                         // Marca el archivo como ya incluido

#include <stdint.h>                      // Tipos enteros de tamaño fijo

//===== Configuración =====
#ifndef LCD_DIR                          // Si no se definió la dirección antes...
#define LCD_DIR  0x27                    // ...módulo PCF8574T típico (Proteus con A2..A0 a GND: 0x20)
#endif                                   // Fin de la definición condicional de la dirección
#define LCD_CHARS_POR_PASADA  4          // Caracteres enviados por cada llamada a lcd_nb_tarea()

//===== Funciones =====
void    lcd_nb_init(void);               // Inicializa TWI y LCD (bloquea ~1 s, solo al arrancar)
void    lcd_nb_limpiar(void);            // Llena el buffer con espacios
void    lcd_nb_texto(uint8_t fila, uint8_t col, const char *s);  // Escribe texto en el buffer desde (fila, col)
void    lcd_nb_linea(uint8_t fila, const char *s);               // Escribe la fila completa (rellena con espacios)
void    lcd_nb_numero(uint8_t fila, uint8_t col, uint16_t v, uint8_t ancho);  // Número alineado a la derecha
void    lcd_nb_tarea(void);              // Envía hasta LCD_CHARS_POR_PASADA caracteres pendientes
uint8_t lcd_nb_ocupado(void);            // 1 = hay contenido del buffer sin enviar a la LCD
void    lcd_nb_backlight(uint8_t on);    // 0 = apaga la luz y deja de escribir; 1 = la enciende y refresca

#endif                                   // Fin de la protección contra inclusión doble