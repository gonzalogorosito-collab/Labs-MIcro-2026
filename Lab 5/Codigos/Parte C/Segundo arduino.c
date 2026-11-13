/*
 * LABORATORIO 5 - PROBLEMA C
 * Segundo Arduino usado solamente como registrador.
 *
 * Conexion:
 *   A0  <- salida del LM358 por una resistencia de 1 k
 *   GND <- pin 4 del LM358 (GND de referencia)
 *
 * No conectar el pin 5 V de este Arduino al circuito medido.
 */
#include <Arduino.h>                                  // Funciones de Arduino: analogRead, micros, Serial

const uint8_t PIN_MEDICION = A0;                      // Pin analógico donde llega la salida del amplificador (por la resistencia de 1 k)
const unsigned long PERIODO_MUESTREO_US = 5000UL;     // Período de muestreo: 5000 us = 5 ms -> 200 muestras por segundo
unsigned long proxima_muestra = 0UL;                  // Instante (en us) en que corresponde tomar la próxima muestra

void setup(void)                                      // Se ejecuta una sola vez al encender o resetear
{
    Serial.begin(115200);                             // Abre el puerto serie a 115200 baudios (debe coincidir con BAUDIOS en Python)

    /* Referencia ADC = alimentacion USB del segundo Arduino. */
    analogReference(DEFAULT);                         // El ADC usa como referencia los 5 V de este Arduino (valor que Python llama VREF_ADC_V)

    /* Se descarta la primera conversion despues de configurar el ADC. */
    (void)analogRead(PIN_MEDICION);                   // Lectura de prueba; el (void) indica que el resultado se descarta a propósito

    proxima_muestra = micros();                       // La primera muestra se toma de inmediato: el instante actual en us
}

void loop(void)                                       // Se repite indefinidamente
{
    unsigned long ahora = micros();                   // Tiempo actual en microsegundos desde el arranque

    if ((long)(ahora - proxima_muestra) >= 0) {       // ¿Ya llegó el momento de muestrear? La conversión a long evita errores cuando micros() da la vuelta (cada ~71 min)
        int lectura_adc = analogRead(PIN_MEDICION);   // Convierte A0 a un número de 0 a 1023

        /* Se envia el dato crudo para poder recalibrarlo despues. */
        Serial.print(ahora);                          // Envía el instante de la muestra (us); Python lo usa como eje de tiempo
        Serial.print(',');                            // Separador entre los dos datos
        Serial.println(lectura_adc);                  // Envía el valor del ADC y termina la línea: formato "micros,adc"

        proxima_muestra += PERIODO_MUESTREO_US;       // Programa la siguiente muestra sumando un período al objetivo anterior (así no se acumula deriva)

        /* Si alguna operacion demoro demasiado, se retoma desde el presente. */
        if ((long)(ahora - proxima_muestra) >         // Si el sistema quedó muy atrasado respecto del objetivo...
            (long)(PERIODO_MUESTREO_US * 4UL)) {      // ...por más de unos 4 períodos (unos 20 ms)
            proxima_muestra = ahora + PERIODO_MUESTREO_US; // ...se reinicia el calendario desde ahora, en vez de enviar muestras atrasadas a ráfaga
        }                                             // Fin del control de atraso
    }                                                 // Fin de la condición de muestreo
}                                                     // Fin de loop
