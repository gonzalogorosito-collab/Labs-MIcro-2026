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

/* ADC para el joystick (referencia AVcc, prescaler 128 = 125 kHz)*/
static void adc_init(void)
{
    ADMUX  = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

static uint16_t adc_leer(uint8_t canal)
{
    ADMUX = (1 << REFS0) | (canal & 0x07);
    ADCSRA |= (1 << ADSC);                  /* primera conversion: se descarta */
    while (ADCSRA & (1 << ADSC));
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
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

static void dibujar_cursor(void)
{
    limpiar();
    poner(cf, cc, PALETA[col_cursor]);
}

/* Boton del joystick: antirrebote cada 10 ms y eventos*/
static evbtn_t btn_evento(void)
{
    static uint8_t ult = 0, est = 0, largo = 0;
    static uint32_t t_lec = 0, t_pres = 0;
    uint32_t ahora = millis();

    if (ahora - t_lec >= 10) {
        t_lec = ahora;
        uint8_t lec = !(PIND & (1 << PD2));         /* activo en bajo */
        uint8_t prev = ult;
        ult = lec;
        if (lec == prev && lec != est) {            /* cambio estable */
            est = lec;
            if (est) { t_pres = ahora; largo = 0; return BTN_PRESIONADO; }
            if (!largo) return BTN_CORTO;           /* soltado antes del tiempo largo */
        }
    }
    if (est && !largo && (ahora - t_pres >= T_PULSO_LARGO)) {
        largo = 1;
        return BTN_LARGO;
    }
    return BTN_NADA;
}

/* Cursor con joystick: ejes independientes, zona muerta y limites [0..7] */
static void mover_cursor(void)
{
    static uint8_t activo = 0;
    static uint32_t t_paso = 0;

    int16_t vx = (int16_t)adc_leer(CANAL_X) - 512;
    int16_t vy = (int16_t)adc_leer(CANAL_Y) - 512;
#if INVERTIR_X
    vx = -vx;
#endif
#if INVERTIR_Y
    vy = -vy;
#endif
    int16_t ax = (vx < 0) ? -vx : vx;
    int16_t ay = (vy < 0) ? -vy : vy;

    if (ax < ZONA_MUERTA && ay < ZONA_MUERTA) { activo = 0; return; }

    uint32_t ahora = millis();
    if (activo && (ahora - t_paso < PASO_CURSOR_MS)) return;
    activo = 1;
    t_paso = ahora;

    if (ax >= ZONA_MUERTA) {                        /* cada eje se mueve por separado */
        if (vx > 0) { if (cc < 7) cc++; } else { if (cc > 0) cc--; }
    }
    if (ay >= ZONA_MUERTA) {
        if (vy > 0) { if (cf < 7) cf++; } else { if (cf > 0) cf--; }
    }
    dibujar_cursor();
}

/* main */
int main(void)
{
    DDRD  &= ~(1 << PD2);                   /* boton del joystick con pull-up interno */
    PORTD |= (1 << PD2);

    ws_init();
    adc_init();
    timer0_init();
    sei();

    dibujar_cursor();
    uint32_t t_refresco = 0;

    for (;;) {
        evbtn_t ev = btn_evento();
        uint32_t ahora = millis();

        mover_cursor();
        if (ev == BTN_CORTO) {                      /* cambia el color del cursor */
            col_cursor = (col_cursor + 1) % COLORES_CURSOR;
            dibujar_cursor();
        }

        if (cambio || (ahora - t_refresco >= REFRESCO_MS)) {    /* se envia si hubo cambios o cada REFRESCO_MS */
            ws_show();
            cambio = 0;
            t_refresco = ahora;
        }
    }
}