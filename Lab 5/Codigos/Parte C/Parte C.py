# 3.... Resumen por estado, carga integrada, guardado final y aviso de saturación del ADC
"""
LABORATORIO 5 - PROBLEMA C
Recibe muestras de un segundo Arduino, calcula corriente, dibuja una grafica
en vivo y guarda los resultados para el informe.
Teclas dentro de la ventana de la grafica:
    1 -> ACTIVO
    2 -> STANDBY
    3 -> APAGADO
    4 -> EMERGENCIA
    Q -> terminar y guardar
"""
from __future__ import annotations                      # Permite usar anotaciones de tipos modernas (tuple[int, int] | None) sin errores
import csv                                              # Escritura de los archivos CSV
import queue                                            # Cola para pasar datos del hilo lector al hilo principal
import statistics                                       # Media y desviación estándar
import threading                                        # Hilo que lee el puerto serie sin trabar la gráfica
import time                                             # Esperas y cronómetro
from collections import deque                           # Lista con extremos rápidos, para la ventana móvil de la gráfica
from datetime import datetime                           # Fecha y hora para nombrar los archivos
from pathlib import Path                                # Manejo de rutas de archivos
import matplotlib.pyplot as plt                         # Gráfica
import numpy as np                                      # Cálculos numéricos con arreglos
import serial                                           # Comunicación con el puerto serie (pyserial)
from matplotlib.animation import FuncAnimation          # Actualiza la gráfica periódicamente
from serial.tools import list_ports                     # Lista los puertos serie disponibles

# Se anulan atajos de Matplotlib que interfieren con el marcado de estados.
plt.rcParams["keymap.save"] = []                        # Quita el atajo de guardar figura
plt.rcParams["keymap.pan"] = []                         # Quita el atajo de desplazar (pan)

# ----------------------- DATOS DEL MONTAJE -----------------------
BAUDIOS = 115200                                        # Velocidad del puerto serie (igual que Serial.begin del Arduino)
VENTANA_GRAFICA_S = 60.0                                # Segundos que se muestran en la gráfica en vivo

# Medir con multimetro el pin 5 V del SEGUNDO Arduino y corregir este valor.
VREF_ADC_V = 5.110                                      # Tensión de referencia del ADC (los 5 V reales del segundo Arduino)

# Divisor 1k (serie) + 10k (a GND) entre la salida del LM358 y A0.
# A0 recibe 10/11 de la salida, por eso Python la multiplica por 11/10 = 1,1.
FACTOR_DIVISOR = 1.1                                    # Corrige el divisor para recuperar la tensión de salida del amplificador

# Resistencia shunt (en el retorno, entre los dos GND).
R_SHUNT_OHM = 4.7                                       # Valor de la shunt en ohmios

# Ganancia del amplificador no inversor: G = 1 + Rf/Rg = 1 + 1k/1k = 2.
GANANCIA = 2.0                                          # Ganancia del LM358

# Tension que alimenta el sistema medido (fuente de banco).
TENSION_ALIMENTACION_V = 5.0                            # Se usa para calcular potencia y energía

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


def crear_archivos() -> tuple[Path, Path, Path]:        # Devuelve las rutas de CSV, resumen y gráfica
    marca = datetime.now().strftime("%Y%m%d_%H%M%S")    # Fecha y hora actuales, usadas en los nombres
    carpeta = Path(__file__).resolve().parent / "mediciones"  # Carpeta "mediciones" junto al script
    carpeta.mkdir(parents=True, exist_ok=True)          # La crea si no existe
    return (                                            # Devuelve las tres rutas
        carpeta / f"medicion_{marca}.csv",              # Datos muestra por muestra
        carpeta / f"resumen_{marca}.csv",               # Estadísticas por estado
        carpeta / f"grafica_{marca}.png",               # Imagen de la gráfica
    )


def calcular_resumen(                                   # Calcula las estadísticas por estado
    tiempos: list[float],                               # Lista de tiempos (s)
    corrientes: list[float],                            # Lista de corrientes (mA)
    estados: list[str],                                 # Estado asignado a cada muestra
) -> list[dict[str, float | int | str]]:                # Devuelve una lista de diccionarios, uno por estado
    """Calcula estadisticas y carga integrada para cada estado."""
    nombres = ["TOTAL", "ACTIVO", "STANDBY", "APAGADO", "EMERGENCIA"]  # Filas del resumen
    resumen = []                                        # Resultado acumulado
    for nombre in nombres:                              # Procesa cada estado
        if nombre == "TOTAL":                           # Para el total...
            indices = list(range(len(tiempos)))         # ...se usan todas las muestras
        else:                                           # Para un estado concreto...
            indices = [i for i, estado in enumerate(estados) if estado == nombre]  # ...solo las muestras con ese estado
        if not indices:                                 # Si ese estado nunca apareció...
            continue                                    # ...se omite
        valores = [corrientes[i] for i in indices]      # Corrientes de esas muestras
        duracion = 0.0                                  # Duración acumulada (s)
        carga_mAh = 0.0                                 # Carga acumulada (mAh)
        for i in range(1, len(tiempos)):                # Recorre los intervalos entre muestras consecutivas
            mismo_estado = (                            # El intervalo cuenta si ambas muestras son del estado
                nombre == "TOTAL"                       # (el total cuenta todos los intervalos)
                or (estados[i - 1] == nombre and estados[i] == nombre)  # o ambas muestras del estado
            )
            if not mismo_estado:                        # Si el intervalo cruza dos estados...
                continue                                # ...no se cuenta
            dt_s = tiempos[i] - tiempos[i - 1]          # Duración del intervalo (s)
            if dt_s <= 0.0 or dt_s > 1.0:               # Se descartan tiempos no válidos o saltos de más de 1 s
                continue                                # Pasa al siguiente intervalo
            corriente_media_intervalo = (corrientes[i - 1] + corrientes[i]) / 2.0  # Promedio de las dos muestras (regla del trapecio)
            carga_mAh += corriente_media_intervalo * dt_s / 3600.0  # mA * s / 3600 = mAh
            duracion += dt_s                            # Suma la duración del intervalo
        corriente_media = statistics.fmean(valores)     # Corriente promedio del estado
        resumen.append(                                 # Agrega la fila al resumen
            {
                "estado": nombre,                       # Nombre del estado
                "muestras": len(valores),               # Cantidad de muestras
                "duracion_s": duracion,                 # Duración integrada (s)
                "corriente_min_mA": min(valores),       # Corriente mínima
                "corriente_max_mA": max(valores),       # Corriente máxima
                "corriente_media_mA": corriente_media,  # Corriente promedio
                "desviacion_mA": statistics.pstdev(valores),  # Desviación estándar
                "carga_mAh": carga_mAh,                 # Carga consumida
                "potencia_media_mW": TENSION_ALIMENTACION_V * corriente_media,  # P = V * I (V en V, I en mA -> mW)
                "energia_mWh": TENSION_ALIMENTACION_V * carga_mAh,  # E = V * Q (mWh)
            }
        )
    return resumen                                      # Devuelve todas las filas


def main() -> None:                                     # Programa principal
    puerto = elegir_puerto()                            # El usuario elige el puerto del segundo Arduino
    print(f"\nAbriendo {puerto} a {BAUDIOS} baudios...")  # Aviso de apertura
    ser = serial.Serial(puerto, BAUDIOS, timeout=0.2)   # Abre el puerto; readline espera como máximo 0,2 s
    time.sleep(2.0)  # El segundo Arduino se reinicia al abrir el puerto.
    ser.reset_input_buffer()                            # Descarta lo que haya llegado durante el reinicio
    offset_v = calibrar_offset(ser)                     # Calibra el offset del amplificador

    archivo_csv, archivo_resumen, archivo_png = crear_archivos()  # Rutas de salida
    print(f"\nCSV: {archivo_csv}")                      # Muestra dónde se guardará el CSV
    print("Teclas: 1=ACTIVO, 2=STANDBY, 3=APAGADO, 4=EMERGENCIA, Q=salir")  # Recordatorio de teclas

    cola: queue.Queue[tuple[int, int]] = queue.Queue()  # Cola de muestras (micros, adc) entre hilos
    ejecutando = threading.Event()                      # Bandera para indicar al hilo lector si debe seguir
    ejecutando.set()                                    # La bandera arranca activada

    def lector_serial() -> None:                        # Función que corre en el hilo lector
        while ejecutando.is_set():                      # Mientras no se pida terminar...
            dato = leer_linea(ser)                      # ...lee una línea del puerto
            if dato is not None:                        # Si es válida...
                cola.put(dato)                          # ...la pone en la cola

    hilo = threading.Thread(target=lector_serial, daemon=True)  # Hilo lector; daemon=True no impide cerrar el programa
    hilo.start()                                        # Arranca el hilo

    # Datos completos para CSV y resumen.
    todos_tiempos: list[float] = []                     # Tiempo de cada muestra (s)
    todos_adc: list[int] = []                           # Valor crudo del ADC
    todos_vout: list[float] = []                        # Tensión de salida del amplificador (V)
    todos_vshunt: list[float] = []                      # Tensión sobre la shunt (V)
    todas_corrientes: list[float] = []                  # Corriente (mA)
    todos_estados: list[str] = []                       # Estado de cada muestra

    # Ventana movil para que la grafica siga siendo rapida.
    graf_t = deque()                                    # Tiempos de la ventana visible
    graf_i = deque()                                    # Corrientes de la ventana visible

    estado_actual = "ACTIVO"                            # El estado inicial es ACTIVO
    cambios_estado: list[tuple[float, str]] = [(0.0, estado_actual)]  # Registro de los cambios de estado (tiempo, nombre)
    primer_us: int | None = None                        # Primer instante recibido (para que el tiempo empiece en 0)
    ultimo_us: int | None = None                        # Último instante recibido (para detectar la vuelta de micros)
    vueltas_micros = 0                                  # Acumula los 2**32 us que se pierden cuando micros() da la vuelta
    finalizado = False                                  # Evita guardar dos veces al cerrar

    csv_f = archivo_csv.open("w", newline="", encoding="utf-8")  # Abre el CSV de datos para escribir
    escritor = csv.writer(csv_f)                        # Escritor de filas CSV
    escritor.writerow(                                  # Escribe la fila de encabezados
        [
            "tiempo_s",                                 # Tiempo desde el inicio (s)
            "micros_arduino",                           # micros() original del Arduino
            "adc_raw",                                  # Cuenta cruda del ADC
            "vout_amplificador_V",                      # Salida del amplificador
            "vshunt_V",                                 # Tensión en la shunt
            "corriente_mA",                             # Corriente calculada
            "estado",                                   # Estado marcado con el teclado
        ]
    )

    fig, ax = plt.subplots(figsize=(12, 6))             # Crea la figura y los ejes
    fig.canvas.manager.set_window_title("LAB 5 - Consumo de corriente")  # Título de la ventana
    linea_cruda, = ax.plot([], [], color="0.75", linewidth=0.7, label="Corriente")  # Curva gris con cada muestra
    linea_suave, = ax.plot([], [], color="#0066CC", linewidth=2.0, label="Promedio movil")  # Curva azul suavizada
    texto_info = ax.text(                               # Recuadro de texto con el estado en vivo
        0.02,                                           # Posición horizontal (fracción del eje)
        0.96,                                           # Posición vertical (fracción del eje)
        "",                                             # Texto inicial vacío
        transform=ax.transAxes,                         # Las posiciones son relativas al eje, no a los datos
        va="top",                                       # Anclado por arriba
        bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "0.75"},  # Fondo blanco semitransparente
    )
    ax.set_title("Corriente del sistema en funcion del tiempo")  # Título de la gráfica
    ax.set_xlabel("Tiempo (s)")                         # Eje X con unidad
    ax.set_ylabel("Corriente (mA)")                     # Eje Y con unidad
    ax.grid(True, alpha=0.3)                            # Cuadrícula suave
    ax.legend(loc="upper right")                        # Leyenda arriba a la derecha

    def marcar_estado(nuevo_estado: str) -> None:       # Registra un cambio de estado
        nonlocal estado_actual                          # Permite modificar la variable de main
        estado_actual = nuevo_estado                    # Guarda el nuevo estado
        tiempo = todos_tiempos[-1] if todos_tiempos else 0.0  # Instante de la última muestra (o 0 si aún no hay)
        cambios_estado.append((tiempo, nuevo_estado))   # Anota el cambio
        ax.axvline(tiempo, color="black", linestyle="--", alpha=0.45)  # Línea vertical punteada en el cambio
        ax.text(                                        # Etiqueta con el nombre del estado
            tiempo,                                     # En el instante del cambio
            ax.get_ylim()[1],                           # En el borde superior de la gráfica
            nuevo_estado,                               # Texto: nombre del estado
            rotation=90,                                # Girado 90 grados
            va="top",                                   # Anclado por arriba
            ha="right",                                 # A la izquierda de la línea
            fontsize=8,                                 # Letra chica
        )
        print(f"Estado marcado: {nuevo_estado} en t={tiempo:.3f} s")  # Confirma en consola

    def al_presionar_tecla(event) -> None:              # Se llama al apretar una tecla en la ventana
        tecla = (event.key or "").lower()               # Nombre de la tecla en minúscula
        mapa = {                                        # Tecla -> estado
            "1": "ACTIVO",
            "2": "STANDBY",
            "3": "APAGADO",
            "4": "EMERGENCIA",
        }
        if tecla in mapa:                               # Si es una de las teclas 1 a 4...
            marcar_estado(mapa[tecla])                  # ...marca ese estado
        elif tecla == "q":                              # Si es Q...
            plt.close(fig)                              # ...cierra la ventana (eso dispara el guardado)

    fig.canvas.mpl_connect("key_press_event", al_presionar_tecla)  # Conecta el teclado con la función

    def procesar_muestra(us_crudo: int, adc: int) -> None:  # Convierte una muestra cruda en corriente y la guarda
        nonlocal primer_us, ultimo_us, vueltas_micros   # Variables de main que se modifican
        if ultimo_us is not None and us_crudo < ultimo_us:  # Si micros() es menor que el anterior, dio la vuelta
            vueltas_micros += 2**32                     # Se suma una vuelta completa (micros() es de 32 bits)
        ultimo_us = us_crudo                            # Recuerda el último valor
        us_extendido = us_crudo + vueltas_micros        # Tiempo continuo sin saltos
        if primer_us is None:                           # En la primera muestra...
            primer_us = us_extendido                    # ...se guarda el origen del tiempo
        tiempo_s = (us_extendido - primer_us) / 1_000_000.0  # Tiempo en segundos desde la primera muestra

        voltaje_a0 = adc * VREF_ADC_V / 1023.0          # Cuenta del ADC a voltios en A0
        vout = voltaje_a0 * FACTOR_DIVISOR              # Compensa el divisor: tensión de salida del amplificador
        vshunt = (vout - offset_v) / GANANCIA           # Resta el offset y divide por la ganancia: tensión real sobre la shunt
        corriente_mA = vshunt / R_SHUNT_OHM * 1000.0    # Ley de Ohm: I = V / R, pasada a mA

        todos_tiempos.append(tiempo_s)                  # Guarda el tiempo
        todos_adc.append(adc)                           # Guarda el valor crudo
        todos_vout.append(vout)                         # Guarda la salida del amplificador
        todos_vshunt.append(vshunt)                     # Guarda la tensión de la shunt
        todas_corrientes.append(corriente_mA)           # Guarda la corriente
        todos_estados.append(estado_actual)             # Guarda el estado vigente

        escritor.writerow(                              # Escribe la muestra en el CSV
            [
                f"{tiempo_s:.6f}",                      # Tiempo con 6 decimales
                us_crudo,                               # micros() original
                adc,                                    # ADC crudo
                f"{vout:.6f}",                          # Vout con 6 decimales
                f"{vshunt:.9f}",                        # Vshunt con 9 decimales
                f"{corriente_mA:.6f}",                  # Corriente con 6 decimales
                estado_actual,                          # Estado
            ]
        )

        graf_t.append(tiempo_s)                         # Agrega el punto a la ventana visible
        graf_i.append(corriente_mA)                     # Agrega la corriente a la ventana visible
        while graf_t and tiempo_s - graf_t[0] > VENTANA_GRAFICA_S:  # Mientras haya puntos más viejos que la ventana...
            graf_t.popleft()                            # ...elimina el tiempo más viejo
            graf_i.popleft()                            # ...y su corriente

    def actualizar(_):                                  # Se llama cada 100 ms para refrescar la gráfica
        cantidad = 0                                    # Muestras procesadas en esta pasada
        while cantidad < 1000:                          # Procesa hasta 1000 muestras por actualización
            try:                                        # Intenta sacar una muestra de la cola
                us_crudo, adc = cola.get_nowait()       # Sin esperar si está vacía
            except queue.Empty:                         # Si no hay más muestras...
                break                                   # ...termina el bucle
            procesar_muestra(us_crudo, adc)             # Convierte y guarda la muestra
            cantidad += 1                               # Cuenta la muestra

        if not graf_t:                                  # Si aún no hay nada para dibujar...
            return linea_cruda, linea_suave, texto_info  # ...no hace nada

        t = np.asarray(graf_t, dtype=float)             # Ventana de tiempos como arreglo
        corriente = np.asarray(graf_i, dtype=float)     # Ventana de corrientes como arreglo

        # Promedio movil de 0,25 s para visualizar sin modificar el CSV.
        puntos = min(50, len(corriente))                # 50 muestras = 0,25 s a 200 muestras/s
        if puntos >= 3:                                 # Con al menos 3 puntos se puede suavizar
            kernel = np.ones(puntos) / puntos           # Ventana de promedio simple
            suave = np.convolve(corriente, kernel, mode="valid")  # Aplica el promedio móvil
            t_suave = t[puntos - 1 :]                   # Tiempos correspondientes a la curva suavizada
        else:                                           # Con muy pocos puntos...
            suave = corriente                           # ...no se suaviza
            t_suave = t                                 # ...y se usan los mismos tiempos

        linea_cruda.set_data(t, corriente)              # Actualiza la curva gris
        linea_suave.set_data(t_suave, suave)            # Actualiza la curva azul

        izquierda = max(0.0, t[-1] - VENTANA_GRAFICA_S)  # Borde izquierdo del eje X
        derecha = max(VENTANA_GRAFICA_S, t[-1] + 1.0)   # Borde derecho del eje X
        ax.set_xlim(izquierda, derecha)                 # Aplica los límites del eje X

        minimo = float(np.min(corriente))               # Corriente mínima visible
        maximo = float(np.max(corriente))               # Corriente máxima visible
        margen = max(2.0, (maximo - minimo) * 0.15)     # Margen vertical: 15 % del rango, al menos 2 mA
        ax.set_ylim(min(minimo - margen, 0.0), maximo + margen)  # Límites del eje Y, siempre incluyendo el 0

        media_corta = float(np.mean(corriente[-min(200, len(corriente)) :]))  # Promedio de las últimas 200 muestras = 1 s
        voltaje_a0_actual = todos_adc[-1] * VREF_ADC_V / 1023.0  # Tensión actual en A0
        aviso = ""                                      # Texto de advertencia (vacío por defecto)
        if voltaje_a0_actual >= 4.75:                   # Si A0 está cerca de los 5 V...
            aviso = "\nADVERTENCIA: A0 cerca de 5 V"    # ...avisa: el amplificador puede estar saturando
        texto_info.set_text(                            # Actualiza el recuadro de información
            f"Estado: {estado_actual}\n"                # Estado actual
            f"Corriente: {corriente[-1]:.2f} mA\n"      # Última corriente
            f"Media 1 s: {media_corta:.2f} mA\n"        # Media del último segundo
            f"A0: {voltaje_a0_actual:.3f} V\n"          # Tensión en A0
            f"Offset LM358: {offset_v:.4f} V"           # Offset calibrado
            f"{aviso}"                                  # Advertencia, si corresponde
        )

        if len(todos_tiempos) % 200 == 0:               # Aproximadamente cada 200 muestras...
            csv_f.flush()                               # ...fuerza la escritura al disco para no perder datos
        return linea_cruda, linea_suave, texto_info     # Devuelve los elementos actualizados

    animacion = FuncAnimation(                          # Llama a actualizar() periódicamente
        fig,                                            # Figura a animar
        actualizar,                                     # Función de actualización
        interval=100,                                   # Cada 100 ms
        blit=False,                                     # Redibuja toda la figura
        cache_frame_data=False,                         # No acumula cuadros en memoria
    )
    _ = animacion                                       # Mantiene una referencia para que Python no elimine la animación

    def finalizar() -> None:                            # Cierra todo y guarda los resultados
        nonlocal finalizado                             # Variable de main que se modifica
        if finalizado:                                  # Si ya se ejecutó...
            return                                      # ...no lo repite
        finalizado = True                               # Marca que ya se ejecutó
        ejecutando.clear()                              # Pide al hilo lector que termine
        hilo.join(timeout=1.0)                          # Espera hasta 1 s a que termine
        ser.close()                                     # Cierra el puerto serie
        csv_f.flush()                                   # Vuelca lo pendiente al CSV
        csv_f.close()                                   # Cierra el CSV
        if not todos_tiempos:                           # Si no llegó ninguna muestra...
            print("No se guardaron muestras.")          # ...avisa
            return                                      # ...y termina
        resumen = calcular_resumen(todos_tiempos, todas_corrientes, todos_estados)  # Calcula las estadísticas
        with archivo_resumen.open("w", newline="", encoding="utf-8") as f:  # Abre el CSV del resumen
            campos = list(resumen[0].keys())            # Nombres de columnas
            writer = csv.DictWriter(f, fieldnames=campos)  # Escritor basado en diccionarios
            writer.writeheader()                        # Escribe los encabezados
            writer.writerows(resumen)                   # Escribe una fila por estado
        fig.savefig(archivo_png, dpi=180, bbox_inches="tight")  # Guarda la gráfica como imagen
        print("\nArchivos guardados:")                  # Lista de archivos generados
        print(f"  Datos:   {archivo_csv}")              # CSV de datos
        print(f"  Resumen: {archivo_resumen}")          # CSV de resumen
        print(f"  Grafica: {archivo_png}")              # Imagen
        print("\nResumen:")                             # Resumen en consola
        for fila in resumen:                            # Una línea por estado
            print(
                f"  {fila['estado']:<10} "              # Nombre alineado a 10 caracteres
                f"media={fila['corriente_media_mA']:.2f} mA, "  # Corriente media
                f"min={fila['corriente_min_mA']:.2f} mA, "      # Corriente mínima
                f"max={fila['corriente_max_mA']:.2f} mA"        # Corriente máxima
            )

    fig.canvas.mpl_connect("close_event", lambda _event: finalizar())  # Al cerrar la ventana se guarda todo

    try:                                                # Muestra la ventana y espera hasta que se cierre
        plt.tight_layout()                              # Ajusta los márgenes de la figura
        plt.show()                                      # Abre la ventana y bloquea hasta cerrarla
    finally:                                            # Pase lo que pase...
        finalizar()                                     # ...se guardan los datos


if __name__ == "__main__":                              # Solo si el archivo se ejecuta directamente
    main()                                              # Arranca el programa
