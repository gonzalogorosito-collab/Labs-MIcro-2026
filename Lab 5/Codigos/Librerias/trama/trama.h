//==========================================================
// trama.h - Protocolo de tramas sobre uart_rb (Bluetooth HC-05)
// Laboratorio 5 - Tecnologías de Microprocesamiento
//==========================================================
// Formato ASCII de largo fijo (10 caracteres):
//
//   <  C C  D D D D  K K  >
//   |  |    |        |    +-- STOP  : '>'
//   |  |    |        +------- CHECKSUM: 2 dígitos hex
//   |  |    +---------------- DATO  : 4 dígitos hex (16 bits)
//   |  +--------------------- COMANDO: 2 dígitos hex
//   +------------------------ START : '<'
//
// CHECKSUM = (COMANDO + DATO_alto + DATO_bajo) mod 256
// Ejemplo: consultar estado   -> <01000001>
//          pasar a STANDBY    -> <03000104>
//==========================================================

#ifndef TRAMA_H                          // Evita incluir este archivo dos veces
#define TRAMA_H                          // Marca el archivo como ya incluido

#include <stdint.h>                      // Tipos enteros de tamaño fijo

//===== Delimitadores y tiempos =====
#define TRAMA_START       '<'            // Carácter de inicio de trama
#define TRAMA_STOP        '>'            // Carácter de fin de trama
#define TRAMA_TIMEOUT_MS  100            // Tiempo máximo entre caracteres antes de declarar trama incompleta

//===== Comandos App -> ATmega =====
#define CMD_ESTADO        0x01           // Consultar estado (DATO = 0)
#define CMD_SENSORES      0x02           // Solicitar lecturas (DATO = 0)
#define CMD_CAMBIAR       0x03           // Cambiar de estado (DATO = 0 ACTIVO, 1 STANDBY, 2 APAGADO)
#define CMD_ACTUADOR      0x04           // Actuador (DATO alto = 1 LED / 2 buzzer, DATO bajo = 0 apagar / 1 encender)
#define CMD_EMERGENCIA    0x05           // Pasar a estado seguro (DATO = 0)
#define CMD_ALARMAS       0x06           // Consultar eventos de alarma (DATO = 0)
#define CMD_REARME        0x07           // Pedir salida del estado seguro (se valida en el firmware)

//===== Respuestas ATmega -> App =====
#define RESP_ACK(cmd)     ((cmd) | 0x80) // Respuesta afirmativa: comando con el bit 7 en 1 (0x81, 0x82, ...)
#define RESP_ADC          0x82           // Lectura del ADC (DATO = 0 a 1023)
#define RESP_TEMP         0x83           // Temperatura del DHT11 (DATO = °C)
#define RESP_HUM          0x84           // Humedad del DHT11 (DATO = %)
#define RESP_EVENTO       0x90           // Notificación espontánea (DATO = código de evento)
#define RESP_ERROR        0xEE           // Error (DATO = código de error)

//===== Códigos de error (DATO de RESP_ERROR) =====
#define ERR_CHECKSUM      0x0001         // El checksum no coincide
#define ERR_FORMATO       0x0002         // Carácter no hexadecimal o largo incorrecto
#define ERR_INCOMPLETA    0x0003         // Trama cortada (timeout o nuevo START antes del STOP)
#define ERR_DESCONOCIDO   0x0004         // Comando que el firmware no reconoce
#define ERR_RECHAZADO     0x0005         // Comando válido pero no permitido en el estado actual

//===== Resultado de trama_recibir() =====
#define TRAMA_NADA        0              // Todavía no hay una trama completa
#define TRAMA_OK          1              // Trama válida en *t

//===== Trama decodificada =====
typedef struct {                         // Contenido útil de una trama
	uint8_t  cmd;                        // Código de comando o respuesta
	uint16_t dato;                       // Dato de 16 bits
} trama_t;                               // Nombre del tipo

//===== Funciones =====
int8_t  trama_recibir(trama_t *t);       // TRAMA_NADA, TRAMA_OK o -código de error (negativo)
uint8_t trama_enviar(uint8_t cmd, uint16_t dato);  // 1 = encolada completa, 0 = sin lugar (no se envía a medias)
uint8_t trama_enviar_error(uint16_t codigo);       // Atajo para enviar RESP_ERROR

#endif                                   // Fin de la protección contra inclusión doble