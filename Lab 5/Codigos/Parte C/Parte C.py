# 1.... Lectura del puerto serie, calibración del offset y cálculo de corriente
"""
LABORATORIO 5 - PROBLEMA C
Recibe muestras de un segundo Arduino, calibra el offset del amplificador
y muestra por consola la corriente calculada.
"""
from __future__ import annotations                      # Permite usar anotaciones de tipos modernas (tuple[int, int] | None) sin errores
import statistics                                       # Media y desviación estándar
import time                                             # Esperas y cronómetro
import serial                                           # Comunicación con el puerto serie (pyserial)
from serial.tools import list_ports                     # Lista los puertos serie disponibles

# ----------------------- DATOS DEL MONTAJE -----------------------
BAUDIOS = 115200                                        # Velocidad del puerto serie (igual que Serial.begin del Arduino)

# Medir con multimetro el pin 5 V del SEGUNDO Arduino y corregir este valor.
VREF_ADC_V = 5.110                                      # Tensión de referencia del ADC (los 5 V reales del segundo Arduino)

# Divisor 1k (serie) + 10k (a GND) entre la salida del LM358 y A0.
# A0 recibe 10/11 de la salida, por eso Python la multiplica por 11/10 = 1,1.
FACTOR_DIVISOR = 1.1                                    # Corrige el divisor para recuperar la tensión de salida del amplificador

# Resistencia shunt (en el retorno, entre los dos GND).
R_SHUNT_OHM = 4.7                                       # Valor de la shunt en ohmios

# Ganancia del amplificador no inversor: G = 1 + Rf/Rg = 1 + 1k/1k = 2.
GANANCIA = 2.0                                          # Ganancia del LM358

# Duracion de la calibracion inicial del offset.
CALIBRACION_S = 3.0                                     # Segundos que se promedia para calibrar


def elegir_puerto() -> str:                             # Devuelve el nombre del puerto serie elegido
    """Muestra los puertos disponibles y permite elegir uno."""
    puertos = list(list_ports.comports())               # Lista de puertos serie detectados
    if not puertos:                                     # Si no hay ninguno...
        raise RuntimeError("No se encontro ningun puerto serie.")  # ...se aborta con un mensaje
    print("\nPuertos disponibles:")                     # Encabezado de la lista
    for numero, puerto in enumerate(puertos, start=1):  # Recorre los puertos numerándolos desde 1
        print(f"  {numero}. {puerto.device} - {puerto.description}")  # Muestra número, nombre y descripción
    if len(puertos) == 1:                               # Si hay un solo puerto...
        print(f"\nSe usara {puertos[0].device}.")       # ...avisa cuál se usa
        return puertos[0].device                        # ...y lo devuelve sin preguntar
    while True:                                         # Si hay varios, pregunta hasta recibir una opción válida
        try:                                            # Protege contra texto que no sea número
            opcion = int(input("Elegir numero de puerto: "))  # Lee el número que escribe el usuario
            if 1 <= opcion <= len(puertos):             # Si está dentro del rango de la lista...
                return puertos[opcion - 1].device       # ...devuelve ese puerto
        except ValueError:                              # Si no era un número...
            pass                                        # ...no hace nada y vuelve a preguntar
        print("Opcion invalida.")                       # Aviso de opción inválida


def leer_linea(ser: serial.Serial) -> tuple[int, int] | None:  # Devuelve (micros, adc) o None si la línea no sirve
    """Lee una linea con formato micros,adc."""
    try:                                                # Cualquier error de formato se trata abajo
        texto = ser.readline().decode("ascii", errors="ignore").strip()  # Lee una línea, la pasa a texto y quita espacios y saltos
        if not texto:                                   # Si llegó vacía (timeout)...
            return None                                 # ...no hay dato
        tiempo_us, adc = texto.split(",", maxsplit=1)   # Separa "micros,adc" en dos partes por la coma
        adc_entero = int(adc)                           # Convierte el valor del ADC a entero
        if not 0 <= adc_entero <= 1023:                 # Un ADC de 10 bits solo da de 0 a 1023
            return None                                 # Si está fuera de rango, se descarta la línea
        return int(tiempo_us), adc_entero               # Devuelve el tiempo en us y el valor del ADC
    except (ValueError, UnicodeError):                  # Si la línea estaba cortada o con basura...
        return None                                     # ...se descarta


def calibrar_offset(ser: serial.Serial) -> float:       # Mide el offset del amplificador y lo devuelve en voltios
    """Calcula el offset del amplificador con el shunt puenteado."""
    print("\nCALIBRACION DEL OFFSET")                   # Título de la calibración
    print("1. Mantener el sistema encendido.")          # Instrucción 1
    print("2. Puentear momentaneamente las dos patas del shunt.")  # Instrucción 2: sin caída en la shunt, la salida debe ser solo el offset
    input("3. Presionar ENTER cuando el puente este colocado...")  # Espera a que el usuario confirme
    ser.reset_input_buffer()                            # Descarta datos viejos acumulados en el puerto
    muestras_v = []                                     # Aquí se guardan las tensiones medidas
    fin = time.monotonic() + CALIBRACION_S              # Instante en que termina la calibración
    while time.monotonic() < fin:                       # Durante CALIBRACION_S segundos...
        dato = leer_linea(ser)                          # ...lee una muestra
        if dato is None:                                # Si la línea no sirvió...
            continue                                    # ...pasa a la siguiente
        _, adc = dato                                   # Se ignora el tiempo, solo interesa el ADC
        voltaje_a0 = adc * VREF_ADC_V / 1023.0          # Convierte la cuenta del ADC a voltios en A0
        muestras_v.append(voltaje_a0 * FACTOR_DIVISOR)  # Corrige el divisor y guarda la tensión de salida del amplificador
    if len(muestras_v) < 20:                            # Si llegaron muy pocas muestras...
        raise RuntimeError("No se recibieron suficientes muestras para calibrar.")  # ...no se puede calibrar
    offset = statistics.fmean(muestras_v)               # Offset = promedio de la tensión con la shunt puenteada
    ruido = statistics.pstdev(muestras_v)               # Dispersión de esas muestras (ruido)
    print(f"Offset medido: {offset:.6f} V")             # Muestra el offset
    print(f"Desviacion durante calibracion: {ruido:.6f} V")  # Muestra el ruido
    input("Retirar el puente del shunt y presionar ENTER para comenzar...")  # Espera a que se quite el puente
    ser.reset_input_buffer()                            # Descarta lo recibido durante la espera
    return offset                                       # Devuelve el offset para restarlo después


def main() -> None:                                     # Programa principal
    puerto = elegir_puerto()                            # El usuario elige el puerto del segundo Arduino
    print(f"\nAbriendo {puerto} a {BAUDIOS} baudios...")  # Aviso de apertura
    ser = serial.Serial(puerto, BAUDIOS, timeout=0.2)   # Abre el puerto; readline espera como máximo 0,2 s
    time.sleep(2.0)  # El segundo Arduino se reinicia al abrir el puerto.
    ser.reset_input_buffer()                            # Descarta lo que haya llegado durante el reinicio
    offset_v = calibrar_offset(ser)                     # Calibra el offset del amplificador

    print("\nLeyendo corriente. Presionar Ctrl+C para terminar.")  # Aviso de inicio de la lectura
    ultimo_aviso = time.monotonic()                     # Instante de la última línea impresa (se imprime cada 0,5 s para no saturar la consola)
    try:                                                # Permite terminar con Ctrl+C sin mostrar un error
        while True:                                     # Lee indefinidamente
            dato = leer_linea(ser)                      # Lee una muestra del puerto
            if dato is None:                            # Si la línea no sirvió...
                continue                                # ...pasa a la siguiente
            us_crudo, adc = dato                        # Separa el tiempo del Arduino y el valor del ADC

            voltaje_a0 = adc * VREF_ADC_V / 1023.0      # Cuenta del ADC a voltios en A0
            vout = voltaje_a0 * FACTOR_DIVISOR          # Compensa el divisor: tensión de salida del amplificador
            vshunt = (vout - offset_v) / GANANCIA       # Resta el offset y divide por la ganancia: tensión real sobre la shunt
            corriente_mA = vshunt / R_SHUNT_OHM * 1000.0  # Ley de Ohm: I = V / R, pasada a mA

            if time.monotonic() - ultimo_aviso >= 0.5:  # Si pasaron 0,5 s desde la última impresión...
                ultimo_aviso = time.monotonic()         # ...actualiza el instante
                print(f"t={us_crudo / 1_000_000.0:8.3f} s  ADC={adc:4d}  I={corriente_mA:7.2f} mA")  # ...muestra tiempo, ADC y corriente
    except KeyboardInterrupt:                           # Si se aprieta Ctrl+C...
        print("\nLectura terminada.")                  # ...avisa que terminó
    finally:                                            # Pase lo que pase...
        ser.close()                                     # ...se cierra el puerto serie


if __name__ == "__main__":                              # Solo si el archivo se ejecuta directamente
    main()                                              # Arranca el programa
