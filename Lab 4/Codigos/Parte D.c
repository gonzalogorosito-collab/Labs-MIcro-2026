#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdint.h>

/* Mapeo de pines
 * PD6        -> DIN matriz WS2812B 8x8
 * PC0 (ADC0) -> VRx joystick
 * PC1 (ADC1) -> VRy joystick
 * PD2        -> SW joystick (pull-up interno, activo en bajo)
 * PB1 (OC1A) -> Buzzer
 * PD1 (TXD)  -> Terminal virtual (depuracion)
 */

/* Parametros ajustables  */
#define WS_PORT        PORTD
#define WS_PIN         PD6
#define SERPENTINA     0        /* 1: las filas impares van en sentido contrario (matriz en zigzag) */
#define INVERTIR_X     0        /* 1: invierte izquierda/derecha del joystick */
#define INVERTIR_Y     0        /* 1: invierte arriba/abajo del joystick */
#define BRILLO         40       /* intensidad maxima de cada canal (0..255), con 64 LEDs hay que ser bajo */
#define REFRESCO_MS    100      /* reenvia la imagen cada tanto aunque no cambie (el simulador la puede perder) */
#define DEBUG          1       /* 1: imprime la secuencia por UART (fila,columna,color) para comprobar el juego */

#define CANAL_X        0        /* ADC0 = PC0 */
#define CANAL_Y        1        /* ADC1 = PC1 */
#define ZONA_MUERTA    150      /* cuentas de ADC alrededor del centro (512) que se ignoran */
#define PASO_CURSOR_MS 200      /* tiempo entre pasos del cursor si el joystick se mantiene */
#define T_PULSO_LARGO  1000     /* pulsacion larga del boton para iniciar el juego */

#define MAX_NIVEL      20       /* niveles para ganar */
#define T_ENCENDIDO    500      /* ms que se ilumina cada elemento de la secuencia */
#define T_PAUSA        200      /* ms de pausa entre elementos */
#define T_INICIAL      600      /* ms antes del primer elemento */
#define T_FLASH        180      /* ms de confirmacion al acertar una posicion */
#define T_RECORD       2000     /* ms que se muestra el record al encender */

/*Colores*/
typedef struct { uint8_t r, g, b; } color_t;

#define V BRILLO
static const color_t PALETA[6] = {
    {V, 0, 0},   /* 0 rojo     */
    {0, V, 0},   /* 1 verde    */
    {0, 0, V},   /* 2 azul     */
    {V, V, 0},   /* 3 amarillo */
    {0, V, V},   /* 4 cian     */
    {V, 0, V}    /* 5 magenta  */
};
#define COLORES_JUEGO   4       /* la secuencia usa solo los 4 primeros */
#define COLORES_CURSOR  6

/*Maquina de estados  */
typedef enum {
    ST_RECORD,      /* al encender: muestra el record */
    ST_LIBRE,       /* modo libre: cursor movil, el boton cambia el color */
    ST_MUESTRA,     /* el sistema reproduce la secuencia */
    ST_JUGADOR,     /* el jugador repite la secuencia */
    ST_ACIERTO,     /* animacion de nivel superado */
    ST_ERROR        /* animacion de fallo */
} estado_t;

typedef enum { BTN_NADA, BTN_PRESIONADO, BTN_CORTO, BTN_LARGO } evbtn_t;

typedef struct { uint8_t f, c, col; } elem_t;

static estado_t estado;
static volatile uint32_t ms;                /* contador de milisegundos (Timer0) */

static color_t fb[64];                      /* imagen de la matriz */
static uint8_t cambio;                      /* 1 = hay que enviar la imagen a la matriz */

static elem_t  seq[MAX_NIVEL];              /* secuencia actual */
static uint8_t nivel;                       /* cantidad de elementos de la secuencia */
static uint8_t completados;                 /* niveles superados en la partida */
static uint8_t record;                      /* record leido de la EEPROM */

static uint8_t cf = 3, cc = 3;              /* posicion del cursor (fila, columna) */
static uint8_t col_cursor = 3;              /* color del cursor */

static uint16_t rng = 0xACE1;               /* estado del generador pseudoaleatorio */

/* Tiempo: Timer0 en CTC, una interrupcion por milisegundo
 f = 16 MHz / (64 * (1 + 249)) = 1 kHz */
ISR(TIMER0_COMPA_vect)
{
    ms++;
}

static void timer0_init(void)
{
    TCCR0A = (1 << WGM01);
    OCR0A  = 249;
    TIMSK0 = (1 << OCIE0A);
    TCCR0B = (1 << CS01) | (1 << CS00);     /* prescaler 64 */
}

static uint32_t millis(void)
{
    uint32_t t;
    uint8_t s = SREG;
    cli();
    t = ms;
    SREG = s;
    return t;
}

/* Matriz WS2812B*/
#define WS_BIT                                  \
    "out  %[p], %[hi]\n\t"                      \
    "nop\n\tnop\n\tnop\n\tnop\n\t"              \
    "sbrs %[b], 7\n\t"                          \
    "out  %[p], %[lo]\n\t"                      \
    "nop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\t" \
    "out  %[p], %[lo]\n\t"                      \
    "lsl  %[b]\n\t"                             \
    "nop\n\tnop\n\tnop\n\tnop\n\tnop\n\t"

static inline void ws_byte(uint8_t b, uint8_t hi, uint8_t lo)
{
    asm volatile(
        WS_BIT WS_BIT WS_BIT WS_BIT
        WS_BIT WS_BIT WS_BIT WS_BIT
        : [b] "+r" (b)
        : [p] "I" (_SFR_IO_ADDR(WS_PORT)), [hi] "r" (hi), [lo] "r" (lo)
    );
}

static void ws_init(void)
{
    DDRD  |= (1 << WS_PIN);
    WS_PORT &= ~(1 << WS_PIN);
}

static void ws_show(void)
{
    uint8_t hi = WS_PORT | (1 << WS_PIN);
    uint8_t lo = WS_PORT & ~(1 << WS_PIN);
    uint8_t s = SREG;
    cli();                                  /* la temporizacion no admite interrupciones.*/
    for (uint8_t i = 0; i < 64; i++) {
        ws_byte(fb[i].g, hi, lo);
        ws_byte(fb[i].r, hi, lo);
        ws_byte(fb[i].b, hi, lo);
    }
    SREG = s;
    _delay_us(80);                          /* nivel bajo > 50 us: la matriz carga la imagen */
}

/* Posicion (fila, columna) indice del LED en la cadena */
static uint8_t indice(uint8_t f, uint8_t c)
{
#if SERPENTINA
    return (uint8_t)(f * 8 + ((f & 1) ? (7 - c) : c));
#else
    return (uint8_t)(f * 8 + c);
#endif
}

static void limpiar(void)
{
    for (uint8_t i = 0; i < 64; i++) { fb[i].r = 0; fb[i].g = 0; fb[i].b = 0; }
    cambio = 1;
}

static void llenar(color_t col)
{
    for (uint8_t i = 0; i < 64; i++) fb[i] = col;
    cambio = 1;
}

static void poner(uint8_t f, uint8_t c, color_t col)
{
    fb[indice(f, c)] = col;
    cambio = 1;
}

/* main */
int main(void)
{
    ws_init();
    timer0_init();
    sei();

    limpiar();
    for (uint8_t f = 0; f < 8; f++)         /* patron de prueba: triangulo, cada fila de un color */
        for (uint8_t c = 0; c <= f; c++)
            poner(f, c, PALETA[f % 6]);

    uint32_t t_refresco = 0;

    for (;;) {
        uint32_t ahora = millis();

        if (cambio || (ahora - t_refresco >= REFRESCO_MS)) {    /* se envia si hubo cambios o cada REFRESCO_MS */
            ws_show();
            cambio = 0;
            t_refresco = ahora;
        }
    }
}