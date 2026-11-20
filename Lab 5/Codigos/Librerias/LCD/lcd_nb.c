//==========================================================
// lcd_nb.c - LCD 16x2 I2C (PCF8574) sin bloqueo, sobre twi_lcd.h
//==========================================================
// IMPORTANTE: twi_lcd.h incluye twi.c y define funciones dentro
// del header. Solo este archivo debe incluirlo, y twi.c NO debe
// agregarse al proyecto como archivo a compilar.
//==========================================================

#include "lcd_nb.h"                      // Prototipos y configuración de esta librería
#include <avr/io.h>                      // Registros DDRC y PORTC
#define PCF8574 LCD_DIR                  // Dirección que usa la librería de la cátedra (no la define ella)
#include "twi_lcd.h"                     // Librería de la cátedra (incluye twi.h y twi.c)

#define LCD_FIN  34                      // Posiciones por refresco: 1 comando + 16 caracteres por fila

//===== Estado de la librería =====
static char    buf[2][16];               // Copia en RAM de lo que debe mostrar la LCD
static uint8_t sucio = 0;                // 1 = el buffer cambió y hay que refrescar
static uint8_t pos = LCD_FIN;            // Próxima posición a enviar (LCD_FIN = sin refresco en curso)
static uint8_t luz = 0;                  // 1 = retroiluminación encendida y LCD en uso

//==========================================================
// Inicialización
//==========================================================
void lcd_nb_init(void)                   // Prepara el bus I2C y la LCD
{
	uint8_t ddrc  = DDRC;                // Guarda la configuración de PORTC...
	uint8_t portc = PORTC;               // ...porque twi_init() pisa PC0 y PC1 (DDRC = PORTC = 0x03)

	twi_init();                          // TWI a 400 kHz (librería de la cátedra)
	DDRC  = (DDRC  & ~0x03) | (ddrc  & 0x03);   // Devuelve PC0/PC1 a como estaban (ADC0/ADC1 libres)
	PORTC = (PORTC & ~0x03) | (portc & 0x03);   // Ídem para las resistencias de pull-up
	twi_lcd_init();                      // Secuencia de inicio en 4 bits (bloquea ~1 s por el mensaje inicial)
	twi_lcd_cmd(0x0C);                   // Display encendido, sin cursor ni parpadeo
	lcd_nb_limpiar();                    // Buffer en blanco
	luz = 1;                             // Luz encendida (la encienden los comandos de la librería)
}                                        // Fin de lcd_nb_init

//==========================================================
// Escritura en el buffer (no toca el bus I2C)
//==========================================================
void lcd_nb_limpiar(void)                // Borra el buffer
{
	for (uint8_t f = 0; f < 2; f++)      // Recorre las dos filas
		for (uint8_t c = 0; c < 16; c++) // Recorre las 16 columnas
			buf[f][c] = ' ';             // Espacio en cada posición
	sucio = 1;                           // Pide refresco (no se usa el comando clear: tarda 1,52 ms)
}                                        // Fin de lcd_nb_limpiar

void lcd_nb_texto(uint8_t fila, uint8_t col, const char *s)  // Texto desde (fila, col)
{
	if (fila > 1) return;                // Solo hay filas 0 y 1
	while (*s && col < 16) buf[fila][col++] = *s++;  // Copia hasta el fin del texto o de la fila
	sucio = 1;                           // Pide refresco
}                                        // Fin de lcd_nb_texto

void lcd_nb_linea(uint8_t fila, const char *s)   // Fila completa
{
	uint8_t col = 0;                     // Columna actual
	if (fila > 1) return;                // Solo hay filas 0 y 1
	while (*s && col < 16) buf[fila][col++] = *s++;  // Copia el texto
	while (col < 16) buf[fila][col++] = ' ';         // Rellena con espacios lo que sobra
	sucio = 1;                           // Pide refresco
}                                        // Fin de lcd_nb_linea

void lcd_nb_numero(uint8_t fila, uint8_t col, uint16_t v, uint8_t ancho)  // Número alineado a la derecha
{
	if (fila > 1 || ancho == 0 || ancho > 5 || col + ancho > 16) return;  // Fuera de rango: no escribe
	for (int8_t i = ancho - 1; i >= 0; i--) {    // Del último dígito al primero
		buf[fila][col + i] = (v || i == ancho - 1) ? ('0' + v % 10) : ' ';  // Dígito, o espacio a la izquierda
		v /= 10;                         // Descarta el dígito ya escrito
	}                                    // Fin del recorrido de dígitos
	sucio = 1;                           // Pide refresco
}                                        // Fin de lcd_nb_numero

//==========================================================
// Envío progresivo a la LCD
//==========================================================
void lcd_nb_tarea(void)                  // Se llama en cada vuelta del lazo principal
{
	uint8_t n = LCD_CHARS_POR_PASADA;    // Cupo de envíos en esta llamada

	if (!luz) return;                    // LCD apagada: no se usa el bus (puede estar apagado por PRR)
	if (pos >= LCD_FIN) {                // No hay refresco en curso...
		if (!sucio) return;              // ...y nada cambió: no hay trabajo
		sucio = 0;                       // Toma el contenido actual
		pos = 0;                         // Empieza un refresco nuevo
	}                                    // Fin del arranque de refresco

	while (n-- && pos < LCD_FIN) {       // Mientras quede cupo y refresco pendiente
		if (pos == 0)        twi_lcd_cmd(0x80);  // Posición 0: cursor a fila 0, columna 0
		else if (pos == 17)  twi_lcd_cmd(0xC0);  // Posición 17: cursor a fila 1, columna 0
		else twi_lcd_dwr(buf[pos / 17][(pos % 17) - 1]);  // 1..16 = fila 0, 18..33 = fila 1
		pos++;                           // Avanza a la siguiente posición
	}                                    // Fin del envío de esta pasada
}                                        // Fin de lcd_nb_tarea

uint8_t lcd_nb_ocupado(void)             // Hay algo sin mostrar
{
	return (pos < LCD_FIN) || sucio;     // Refresco en curso o buffer modificado
}                                        // Fin de lcd_nb_ocupado

//==========================================================
// Retroiluminación
//==========================================================
void lcd_nb_backlight(uint8_t on)        // Enciende o apaga la luz (P3 del PCF8574)
{
	if (on) {                            // Encender (antes: PRR del TWI ya habilitado)
		luz = 1;                         // La tarea vuelve a escribir
		PCF8574_write(0x08);             // P3 = 1: luz encendida, EN = 0 (no se escribe nada en la LCD)
		sucio = 1;                       // Refresca todo el contenido
	} else {                             // Apagar
		PCF8574_write(0x00);             // P3 = 0: luz apagada
		luz = 0;                         // La tarea deja de usar el bus (después se puede apagar el TWI por PRR)
		pos = LCD_FIN;                   // Cancela un refresco a medias...
		sucio = 1;                       // ...y lo repite completo al volver
	}                                    // Fin de la selección
}                                        // Fin de lcd_nb_backlight