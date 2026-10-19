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

/* Un tono distinto para cada color del juego (Hz) */
static const uint16_t TONOS[COLORES_JUEGO] = { 330, 392, 440, 523 };
#define TONO_ERROR      110

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

/* Buzzer: Timer1 Fast PWM (modo 14), OC1A = PB1
 ICR1 = F_CPU / (8 * f) - 1 ; ciclo de trabajo 50 % */
static void tono(uint16_t f)
{
    if (f == 0) {
        TCCR1A = 0;
        TCCR1B = 0;
        PORTB &= ~(1 << PB1);
        return;
    }
    uint32_t top = (F_CPU / (8UL * f)) - 1;
    DDRB |= (1 << PB1);
    ICR1  = (uint16_t)top;
    OCR1A = (uint16_t)(top / 2);
    TCNT1 = 0;
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);   /* prescaler 8 */
}

/*UART 9600 8N1 (solo transmision, para depuracion) */
static void uart_init(void)
{
    UBRR0  = 103;                           /* 16 MHz / (16 * 9600) - 1 */
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart_tx(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

static void uart_puts_P(const char *s)
{
    char c;
    while ((c = pgm_read_byte(s++))) uart_tx(c);
}

static void uart_uint(uint8_t n)
{
    if (n >= 100) uart_tx('0' + n / 100);
    if (n >= 10)  uart_tx('0' + (n / 10) % 10);
    uart_tx('0' + n % 10);
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

/* Numeros aleatorios */
static uint16_t rnd(void)
{
    rng ^= rng << 7;
    rng ^= rng >> 9;
    rng ^= rng << 8;
    return rng;
}

/* Agrega un elemento a la secuencia, distinto de posicion al anterior */
static void agregar_elemento(void)
{
    elem_t e;
    do {
        e.f = rnd() & 7;
        e.c = rnd() & 7;
    } while (nivel > 0 && e.f == seq[nivel - 1].f && e.c == seq[nivel - 1].c);
    e.col = rnd() % COLORES_JUEGO;
    seq[nivel++] = e;
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

/* Variables y rutinas de cada estado */
static uint32_t t_estado;
static uint8_t  fase;
static uint8_t  mu_idx, mu_on;              /* ST_MUESTRA */
static uint8_t  idx, flash;                 /* ST_JUGADOR */
static uint32_t t_flash;

static const uint16_t DUR_ACIERTO[4] = { 250, 150, 250, 150 };
static const uint16_t DUR_ERROR[6]   = { 250, 120, 250, 120, 250, 120 };

static void iniciar_libre(void)
{
    estado = ST_LIBRE;
    cf = 3; cc = 3;
    dibujar_cursor();
}

static void iniciar_muestra(void)
{
    estado = ST_MUESTRA;
    mu_idx = 0;
    mu_on = 0;
    t_estado = millis();
    limpiar();
    uart_puts_P(PSTR("Nivel "));
    uart_uint(nivel);
    uart_puts_P(PSTR("\r\n"));
#if DEBUG
    for (uint8_t i = 0; i < nivel; i++) {               /* fila, columna y color (0 rojo, 1 verde, 2 azul, 3 amarillo) */
        uart_puts_P(PSTR("  ("));
        uart_uint(seq[i].f);
        uart_puts_P(PSTR(","));
        uart_uint(seq[i].c);
        uart_puts_P(PSTR(") color "));
        uart_uint(seq[i].col);
        uart_puts_P(PSTR("\r\n"));
    }
#endif
}

static void iniciar_jugador(void)
{
    estado = ST_JUGADOR;
    idx = 0;
    flash = 0;
    cf = 3; cc = 3;
    dibujar_cursor();
}

static void aplicar_acierto(void)
{
    if (fase == 0)      { llenar(PALETA[1]); tono(523); }
    else if (fase == 2) { llenar(PALETA[1]); tono(659); }
    else                { limpiar(); tono(0); }
}

static void iniciar_acierto(void)
{
    estado = ST_ACIERTO;
    fase = 0;
    t_estado = millis();
    aplicar_acierto();
}

static void aplicar_error(void)
{
    if ((fase & 1) == 0) { llenar(PALETA[0]); tono(TONO_ERROR); }
    else                 { limpiar(); tono(0); }
}

static void iniciar_error(void)
{
    estado = ST_ERROR;
    fase = 0;
    t_estado = millis();
    aplicar_error();
}

/* Actualiza el record y vuelve al modo libre */
static void terminar_partida(void)
{
    if (completados > record) {
        record = completados;
    }
    uart_puts_P(PSTR("Fin. Niveles: "));
    uart_uint(completados);
    uart_puts_P(PSTR("  Record: "));
    uart_uint(record);
    uart_puts_P(PSTR("\r\n"));
    iniciar_libre();
}

/* main */
int main(void)
{
    DDRD  &= ~(1 << PD2);                   /* boton del joystick con pull-up interno */
    PORTD |= (1 << PD2);

    ws_init();
    adc_init();
    uart_init();
    timer0_init();
    sei();

    /* semilla del generador: ruido del joystick y tiempo */
    rng ^= (uint16_t)(adc_leer(CANAL_X) * 31u + adc_leer(CANAL_Y));
    if (rng == 0) rng = 0xACE1;

    _delay_ms(100);                         /* deja estabilizar la linea serie antes del primer texto */
    uart_puts_P(PSTR("Simon Dice\r\n"));

    iniciar_libre();
    uint32_t t_refresco = 0;

    for (;;) {
        evbtn_t ev = btn_evento();
        uint32_t ahora = millis();

        switch (estado) {

        case ST_LIBRE:
            mover_cursor();
            if (ev == BTN_CORTO) {                      /* color aleatorio para el cursor */
                uint8_t nuevo;
                do { nuevo = rnd() % COLORES_CURSOR; } while (nuevo == col_cursor);
                col_cursor = nuevo;
                dibujar_cursor();
            } else if (ev == BTN_LARGO) {               /* empieza la partida */
                rng ^= (uint16_t)ahora;
                if (rng == 0) rng = 0xACE1;
                nivel = 0;
                completados = 0;
                agregar_elemento();
                iniciar_muestra();
            }
            break;

        case ST_MUESTRA:
            if (!mu_on) {
                if (ahora - t_estado >= (mu_idx == 0 ? T_INICIAL : T_PAUSA)) {
                    limpiar();
                    poner(seq[mu_idx].f, seq[mu_idx].c, PALETA[seq[mu_idx].col]);
                    tono(TONOS[seq[mu_idx].col]);
                    mu_on = 1;
                    t_estado = ahora;
                }
            } else if (ahora - t_estado >= T_ENCENDIDO) {
                limpiar();
                tono(0);
                mu_on = 0;
                t_estado = ahora;
                mu_idx++;
                if (mu_idx >= nivel) iniciar_jugador();
            }
            break;

        case ST_JUGADOR:
            if (flash) {                                /* confirmacion de una posicion correcta */
                if (ahora - t_flash >= T_FLASH) {
                    flash = 0;
                    tono(0);
                    idx++;
                    if (idx >= nivel) {
                        completados = nivel;
                        iniciar_acierto();
                    } else {
                        dibujar_cursor();
                    }
                }
                break;
            }
            mover_cursor();
            if (ev == BTN_PRESIONADO) {
                if (cf == seq[idx].f && cc == seq[idx].c) {
                    limpiar();
                    poner(cf, cc, PALETA[seq[idx].col]);
                    tono(TONOS[seq[idx].col]);
                    flash = 1;
                    t_flash = ahora;
                } else {
                    iniciar_error();
                }
            }
            break;

        case ST_ACIERTO:
            if (ahora - t_estado >= DUR_ACIERTO[fase]) {
                t_estado = ahora;
                fase++;
                if (fase >= 4) {
                    tono(0);
                    if (nivel >= MAX_NIVEL) {
                        terminar_partida();             /* juego completo */
                    } else {
                        agregar_elemento();
                        iniciar_muestra();
                    }
                } else {
                    aplicar_acierto();
                }
            }
            break;

        case ST_ERROR:
            if (ahora - t_estado >= DUR_ERROR[fase]) {
                t_estado = ahora;
                fase++;
                if (fase >= 6) {
                    tono(0);
                    terminar_partida();
                } else {
                    aplicar_error();
                }
            }
            break;
        }

        if (cambio || (ahora - t_refresco >= REFRESCO_MS)) {    /* se envia si hubo cambios o cada REFRESCO_MS */
            ws_show();
            cambio = 0;
            t_refresco = ahora;
        }
    }
}