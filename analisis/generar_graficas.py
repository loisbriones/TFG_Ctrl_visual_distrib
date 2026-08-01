#!/usr/bin/env python3
"""
Genera las gráficas de análisis de una carrera a partir de la carpeta de datos
de una prueba (bag mcap + cachés JSON de trayectoria + logs de carrera).

Gráficas generadas (las 4 primeras replican las figuras 5.10, 5.12, 5.16 y
5.17 de la memoria de Mario López Cea):

  1. Detección de derrapes: dist_derrape frente a índice de mensaje, con el
     umbral y los eventos de derrape marcados.
  2. Valor de derrape en cada punto del recorrido (3D, una por cámara).
  3. Relación entre los derrapes y la velocidad instantánea (px/fotograma).
  4. Trayectorias de la etiqueta frontal y trasera (una por cámara), unidas
     en cada instante por el eje del vehículo.
  5. Trayectoria guardada por el controlador, un color por cámara.

Uso (dentro de un entorno ROS2 con image_processor_pkg compilado; ver run.sh):

  python3 generar_graficas.py <carpeta_datos> [--coche car1] [--salida DIR] [--show]
  python3 generar_graficas.py <carpeta_datos> --vueltas    # modo por vueltas

Con --vueltas no se generan PNG: la figura interactiva se sirve EN EL
NAVEGADOR (backend WebAgg de matplotlib; el script imprime la dirección,
http://localhost:8988) con las gráficas 1-4 de una sola vuelta, y se cambia
de vuelta con las flechas del teclado tras hacer clic sobre la figura. La
gráfica 5 no aparece porque la ruta del controlador es fija entre vueltas.

============================================================
CÓMO ESTÁ ORGANIZADO EL SCRIPT
============================================================

1. Constantes del análisis y colores (justo debajo de los imports).
2. Localización de ficheros dentro de la carpeta de datos.
3. Lectura del bag con rosbag2_py (la API oficial de ROS2) y emparejado
   de la telemetría con las posiciones que la originaron.
4. Las cinco funciones grafica_N_*: cada una lleva encima un comentario
   grande que explica TODOS los parámetros de matplotlib que usa (y algunos
   extra que no usa pero que pueden hacer falta), para poder ajustar las
   figuras editando directamente los valores.
5. Modo interactivo por vueltas (--vueltas): las gráficas 1-4 de una vuelta
   en una sola ventana, cambiando de vuelta con el teclado.
6. Resúmenes por consola (tiempos por vuelta del bag y logs de carrera).
7. main(): orquesta todo lo anterior.
"""

import argparse
import bisect
import json
import math
import re
import sys
from pathlib import Path

import matplotlib

# Backend "Agg" = dibujar en memoria y guardar a fichero, sin necesidad de
# pantalla. Imprescindible dentro del contenedor Docker (no hay servidor X).
# Si se pasa --show, main() lo cambia a "TkAgg" para abrir ventanas.
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

# API oficial de ROS2 para leer bags:
#  - rosbag2_py: abre el bag y devuelve los mensajes serializados (bytes CDR)
#  - deserialize_message: convierte esos bytes en un objeto mensaje de Python
#  - get_message: dado el nombre del tipo ("image_processor_pkg/msg/CarLocation")
#    devuelve la clase Python generada al compilar el paquete con colcon
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# ---------------------------------------------------------------------------
# Parámetros del análisis
# ---------------------------------------------------------------------------
# Umbral de distancia perpendicular (px) para considerar derrape.
# Debe coincidir con EstrategiaPerfil.umbral_derrape (AlgoritmoVelocidad.py):
# aquí solo se usa para DIBUJAR la línea de umbral de la gráfica 1, no cambia
# nada del análisis. Si algún día se cambia en el algoritmo, cambiarlo aquí
# también para que la figura siga contando la verdad.
UMBRAL_DERRAPE = 8.0

# Desfase máximo (s) admitido al emparejar un mensaje de telemetría con el
# mensaje de posición que lo originó (se emparejan por tiempo de grabación,
# ver emparejar_telemetria). Subirlo empareja más mensajes pero con más riesgo
# de asociar una telemetría a una posición que no es la suya; bajarlo deja
# más telemetrías sin emparejar (no salen en las gráficas 2 y 3).
MAX_DESFASE_EMPAREJADO = 0.2

# Filtros de la velocidad instantánea (gráfica 3): dos posiciones consecutivas
# de la misma cámara solo definen una velocidad si están próximas en tiempo y
# en espacio. Si el hueco temporal es mayor es que se perdieron mensajes (la
# distancia recorrida ya no equivale a "px por fotograma"); si el salto en px
# es enorme es una detección falsa o el coche reapareció por otro lado del
# encuadre. En ambos casos ese par de puntos se descarta.
MAX_DT_VELOCIDAD = 0.25  # s (a 30 Hz lo normal son ~0.033 s entre mensajes)
MAX_SALTO_VELOCIDAD = 100.0  # px

# ---------------------------------------------------------------------------
# Colores de las series (tomados de las figuras gnuplot de la memoria de
# Mario, para que las gráficas de esta memoria se parezcan a las suyas).
# Se pueden cambiar por cualquier color matplotlib: nombre ("red", "black"),
# hexadecimal ("#RRGGBB") o abreviatura ("k" = negro, "m" = magenta...).
# ---------------------------------------------------------------------------
COLOR_DIST = "#7EB6DE"  # azul claro de la serie dist_derrape (gráfica 1)
COLOR_UMBRAL = "black"  # línea horizontal del umbral (gráfica 1)
COLOR_DERRAPE = "#009E73"  # verde de los eventos/serie de derrape (gráficas 1 y 3)
COLOR_VELOCIDAD = "#9400D3"  # violeta de la velocidad instantánea (gráfica 3)
COLOR_TRAY_3D = "#9400D3"  # línea "Posicion" de la gráfica 3D (gráfica 2)
COLOR_ETIQ_TRASERA = "#CC00CC"  # magenta de la etiqueta trasera (gráfica 4)
COLOR_ETIQ_DELANTERA = "#009E73"  # verde de la etiqueta delantera (gráfica 4)
COLOR_EJE_VEHICULO = "0.55"  # gris para la recta que une frontal y trasera (gráfica 4)

# Paleta categórica de orden fijo para la gráfica 5 (un color por cámara).
# Es la paleta Okabe-Ito, segura para daltonismo: la cámara 1ª por orden
# alfabético siempre recibe el primer color, la 2ª el segundo, etc. Si algún
# día hay más cámaras que colores, se reutilizan cíclicamente (operador %).
PALETA_CAMARAS = ["#0072B2", "#E69F00", "#009E73", "#CC79A7", "#D55E00", "#56B4E9"]


# ---------------------------------------------------------------------------
# Localización de los ficheros dentro de la carpeta de datos
# ---------------------------------------------------------------------------
def encontrar_bag(carpeta: Path) -> Path:
    """Devuelve la carpeta del bag (la que contiene metadata.yaml).

    Un bag de rosbag2 es una CARPETA con un metadata.yaml y uno o más .mcap,
    así que basta con buscar el metadata.yaml de forma recursiva (rglob) por
    si el bag está en una subcarpeta (p. ej. PRUEBAS/X/001_LUNES_13_08/)."""
    metadatas = sorted(carpeta.rglob("metadata.yaml"))
    if not metadatas:
        sys.exit(f"ERROR: no se encontró ningún bag (metadata.yaml) en {carpeta}")
    if len(metadatas) > 1:
        # Si hubiera varios bags se avisa y se usa el primero por orden
        # alfabético; para analizar otro, pasar directamente su carpeta.
        print(f"AVISO: hay {len(metadatas)} bags; se usa {metadatas[0].parent}")
    return metadatas[0].parent


def cargar_trayectoria_controlador(carpeta: Path):
    """Carga cache_trayectoria_controller.json: {camara_id: [[x, y], ...]}.

    Es la caché que CarControllerNode escribe al terminar la calibración
    (ver procesar_trayectorias en CarControllerNode.py): la ruta "limpia"
    (nodos separados >= 15 px) que ve cada cámara, en píxeles de esa cámara.
    La usan la gráfica 2 (línea "Posicion" en z=0) y la gráfica 5.
    Devuelve {} si el fichero no está (esas partes se omiten con un aviso)."""
    ficheros = sorted(carpeta.rglob("cache_trayectoria_controller.json"))
    if not ficheros:
        return {}
    with open(ficheros[0]) as f:
        datos = json.load(f)
    # Se convierte cada lista de puntos a array numpy (N, 2) para poder
    # indexar por columnas: puntos[:, 0] = todas las x, puntos[:, 1] = las y
    return {cam: np.asarray(puntos, dtype=float) for cam, puntos in datos.items()}


# ---------------------------------------------------------------------------
# Lectura del bag con la API oficial de ROS2 (rosbag2_py)
# ---------------------------------------------------------------------------
def _decodificadores_mcap(bag_dir: Path):
    """Construye un decodificador por topic a partir de las definiciones de
    mensaje EMBEBIDAS en el propio fichero mcap.

    Es el plan B de leer_bag: los mensajes se deserializan normalmente con
    las clases compiladas del paquete (deserialize_message), pero si los
    .msg del código han cambiado DESPUÉS de grabar el bag (p. ej. añadir un
    campo a CarLocation), la clase actual ya no coincide con los bytes
    grabados y la deserialización revienta. El formato mcap guarda dentro
    del fichero el texto completo de la definición de cada tipo tal y como
    era al grabar, así que la librería mcap-ros2-support puede decodificar
    cualquier bag antiguo con SU definición original, sin depender del
    estado actual del código.

    Los objetos decodificados exponen los campos con la misma notación que
    los mensajes normales (msg.front.center.x), por lo que el resto del
    script no nota la diferencia.

    Devuelve {nombre_topic: funcion(bytes) -> mensaje decodificado}."""
    # Se importa aquí y no arriba: solo hace falta para bags "antiguos" y
    # así el script sigue funcionando aunque la librería no esté instalada
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    decodificadores = {}
    fabrica = DecoderFactory()
    # Un bag puede estar repartido en varios .mcap (rosbag2 los rota por
    # tamaño); cada uno lleva su propio índice con los esquemas
    for fichero in sorted(bag_dir.glob("*.mcap")):
        with open(fichero, "rb") as f:
            resumen = make_reader(f).get_summary()
            if resumen is None:
                continue  # mcap sin índice (grabación cortada): se ignora
            for canal in resumen.channels.values():
                esquema = resumen.schemas[canal.schema_id]
                # decoder_for devuelve la función que convierte los bytes
                # CDR en un objeto con los campos del esquema embebido
                decodificadores[canal.topic] = fabrica.decoder_for("cdr", esquema)
    return decodificadores


def leer_bag(bag_dir: Path, coche: str):
    """Lee del bag las posiciones, la telemetría del controlador y los tiempos
    por vuelta del coche indicado.

    Devuelve tres listas de diccionarios (posiciones, telemetria, vueltas).
    Los timestamps "t" son los de GRABACIÓN del bag en nanosegundos: el
    instante en el que el grabador recibió cada mensaje. Como todos los
    mensajes los graba la misma máquina, todos comparten reloj y se pueden
    comparar entre topics (clave para el emparejado posterior)."""
    # El metadata.yaml dice con qué plugin de almacenamiento se grabó el bag
    # (aquí siempre "mcap"); se lee en vez de suponerlo por robustez.
    with open(bag_dir / "metadata.yaml") as f:
        metadata = yaml.safe_load(f)["rosbag2_bagfile_information"]
    storage_id = metadata.get("storage_identifier", "mcap")

    # Patrón estándar de lectura con rosbag2_py: reader secuencial que
    # devuelve los mensajes de todos los topics mezclados en orden de tiempo.
    # ConverterOptions("cdr", "cdr") = no convertir el formato de serialización
    # (CDR es el formato binario nativo de DDS/ROS2).
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )

    # Topics que interesan, construidos a partir del nombre del coche
    topic_pos = f"/{coche}/position"  # CarLocation de las cámaras
    topic_tel = f"/telemetria/{coche}/car_control"  # CarControlTelemetry
    topic_lap = f"/telemetria/{coche}/time_per_lap"  # TimePerLap
    interesantes = [topic_pos, topic_tel, topic_lap]

    # get_all_topics_and_types() da el nombre del tipo de cada topic tal cual
    # quedó grabado ("image_processor_pkg/msg/CarLocation")
    tipos = {t.name: t.type for t in reader.get_all_topics_and_types()}
    faltan = [t for t in interesantes if t not in tipos]
    if topic_pos in faltan:
        sys.exit(f"ERROR: el bag no contiene {topic_pos} (topics: {sorted(tipos)})")
    if faltan:
        print(f"AVISO: el bag no contiene {faltan}")
    # Filtro de topics: el reader se salta todo lo demás. Importante para no
    # deserializar las imágenes de /camara_XX/camara_debug (lo más pesado del bag)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in interesantes if t in tipos]))

    # Clase Python de cada tipo de mensaje (necesita el paquete compilado y
    # el install/setup.bash cargado; run.sh se encarga de ambas cosas)
    clases = {t: get_message(tipos[t]) for t in interesantes if t in tipos}

    # Decodificadores de respaldo para bags grabados con definiciones de
    # mensaje antiguas (ver _decodificadores_mcap). Se construyen solo si
    # hacen falta, la primera vez que una deserialización falla.
    decodificadores = None

    posiciones, telemetria, vueltas = [], [], []
    # Bucle estándar: has_next/read_next hasta agotar el bag. Cada read_next
    # devuelve (nombre del topic, bytes serializados, timestamp de grabación)
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        msg = None
        if clases[topic] is not None:
            try:
                msg = deserialize_message(data, clases[topic])
            except Exception:
                # La definición actual del .msg no coincide con la del bag
                # (cambió después de grabar): este topic pasa a decodificarse
                # con el esquema embebido en el mcap de aquí en adelante
                clases[topic] = None
                print(f"AVISO: la definición actual de {tipos[topic]} no coincide con "
                      f"la grabada en el bag; {topic} se decodifica con el esquema "
                      "embebido en el mcap")
        if msg is None:
            if decodificadores is None:
                decodificadores = _decodificadores_mcap(bag_dir)
            msg = decodificadores[topic](data)
        if topic == topic_pos:
            # Posición del coche vista por UNA cámara: centros de las dos
            # pegatinas en píxeles de esa cámara. (0, 0) = no detectada.
            posiciones.append(
                {
                    "t": t_ns,
                    "camara": msg.camara_id,
                    "fx": msg.front.center.x,
                    "fy": msg.front.center.y,
                    "bx": msg.back.center.x,
                    "by": msg.back.center.y,
                }
            )
        elif topic == topic_tel:
            # Telemetría que publica CarControllerNode por cada posición
            # procesada en modo carrera: la distancia perpendicular de la
            # pegatina trasera a la trayectoria y si hay derrape en curso
            telemetria.append(
                {
                    "t": t_ns,
                    "dist": msg.dist_derrape,
                    "derrapando": msg.estado_derrapando,
                }
            )
        elif topic == topic_lap:
            # Un mensaje por vuelta completada, con su tiempo. El timestamp
            # de grabación "t" marca el FIN de la vuelta (el mensaje se
            # publica al cruzar la línea de meta), así que la vuelta ocupa
            # la ventana [t - tiempo*1e9, t]: es lo que usa el modo
            # --vueltas para repartir los demás mensajes entre vueltas.
            vueltas.append({"numero": msg.lap_number, "tiempo": msg.lap_time, "t": t_ns})

    return posiciones, telemetria, vueltas


def emparejar_telemetria(telemetria, posiciones_validas):
    """Asocia cada mensaje de telemetría con el mensaje de posición (válido)
    anterior más cercano en tiempo de grabación.

    CarControlTelemetry no lleva camara_id ni las
    coordenadas del coche, solo dist_derrape. Para pintar ese valor SOBRE el
    circuito (gráfica 2) o junto a la velocidad (gráfica 3) hay que saber a
    qué posición corresponde. Como el controlador publica una telemetría
    inmediatamente después de procesar cada posición, y el bag graba ambos
    topics con el mismo reloj, la posición que originó una telemetría es la
    última posición grabada antes que ella (a milisegundos de distancia).

    Los tiempos de posiciones_validas están ordenados (el bag
    se lee en orden), así que bisect_right hace una búsqueda binaria O(log n)
    del hueco donde caería el tiempo de la telemetría; el índice anterior es
    la última posición con t <= t_telemetria.

    Devuelve una lista paralela a `telemetria` con el índice de la posición
    emparejada en posiciones_validas, o None si la posición más cercana está
    a más de MAX_DESFASE_EMPAREJADO segundos (mensaje de posición perdido por
    el QoS best-effort: mejor descartar que emparejar mal)."""
    tiempos = [p["t"] for p in posiciones_validas]
    max_dt_ns = int(MAX_DESFASE_EMPAREJADO * 1e9)
    indices = []
    for m in telemetria:
        i = bisect.bisect_right(tiempos, m["t"]) - 1
        if i >= 0 and m["t"] - tiempos[i] <= max_dt_ns:
            indices.append(i)
        else:
            indices.append(None)
    return indices


# ---------------------------------------------------------------------------
# Aspecto general de las gráficas
# ---------------------------------------------------------------------------
def estilo_gnuplot():
    """Aproxima el aspecto de las figuras gnuplot de la memoria de Mario.

    rcParams son los valores POR DEFECTO globales de matplotlib: afectan a
    todas las figuras que se creen después de llamar a esta función, salvo
    que una llamada concreta los sobreescriba (p. ej. un lw= en un plot()
    manda sobre lines.linewidth). Qué controla cada clave:

      figure.facecolor    color de fondo de la figura (fuera de los ejes)
      axes.linewidth      grosor del recuadro de los ejes (px). Subirlo
                          engorda el marco de la gráfica
      font.size           tamaño base de TODO el texto (ticks, leyenda,
                          etiquetas). Para la memoria puede interesar 10-11
      legend.frameon      True = la leyenda lleva recuadro (como gnuplot)
      legend.framealpha   opacidad del fondo del recuadro (1 = opaco)
      legend.edgecolor    color del borde del recuadro de la leyenda
      legend.fancybox     False = esquinas rectas (True las redondea)
      xtick/ytick.direction  "in" = las marcas de los ejes apuntan hacia
                          dentro (estilo gnuplot); "out" hacia fuera
      xtick.top / ytick.right  repetir las marcas en los bordes superior y
                          derecho (gnuplot las pone en los 4 lados)
      lines.linewidth     grosor por defecto de las líneas (se sobreescribe
                          con lw= en cada plot)
      savefig.dpi         resolución de los PNG (puntos por pulgada). El
                          tamaño en píxeles del PNG = figsize * dpi; para
                          imprimir en la memoria con más calidad subir a
                          200-300 (los ficheros pesan más)
    """
    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.linewidth": 0.8,
            "font.size": 9,
            "legend.frameon": True,
            "legend.framealpha": 1.0,
            "legend.edgecolor": "black",
            "legend.fancybox": False,
            "xtick.direction": "in",
            "ytick.direction": "in",
            "xtick.top": True,
            "ytick.right": True,
            "lines.linewidth": 0.8,
            "savefig.dpi": 150,
        }
    )


def guardar(fig, salida: Path, nombre: str, show: bool):
    """Guarda la figura como PNG en la carpeta de salida (creándola si no
    existe) y, si se pidió --show, la muestra también en una ventana.

    bbox_inches="tight" recorta los márgenes blancos sobrantes alrededor de
    la figura. plt.close(fig) libera la memoria de la figura: importa cuando
    se generan muchas (matplotlib avisa a partir de ~20 figuras abiertas)."""
    salida.mkdir(parents=True, exist_ok=True)
    ruta = salida / nombre
    fig.savefig(ruta, bbox_inches="tight")
    print(f"  -> {ruta}")
    if show:
        plt.show()
    plt.close(fig)


# ===========================================================================
# GRÁFICA 1: detección de derrapes (réplica de la figura 5.10 de Mario)
# ===========================================================================
def grafica_1_deteccion_derrapes(telemetria, salida, show):
    """Réplica de la figura 5.10: dist_derrape por mensaje, umbral y eventos."""
    dist = np.array([m["dist"] for m in telemetria])
    derrapando = np.array([m["derrapando"] for m in telemetria])
    x = np.arange(len(dist))

    fig, ax = plt.subplots(figsize=(7.5, 5))
    ax.axhline(UMBRAL_DERRAPE, color=COLOR_UMBRAL, lw=0.8, label="Umbral")
    ax.plot(
        x[derrapando],
        np.full(derrapando.sum(), UMBRAL_DERRAPE),
        linestyle="none",
        marker="*",
        markersize=7,
        color=COLOR_DERRAPE,
        label="Derrape",
    )
    ax.plot(x, dist, color=COLOR_DIST, lw=0.6, label=r"Dist$_{derrape}$")
    ax.set_ylabel("Distancia de derrape")
    ax.set_xlim(0, len(dist) if len(dist) else 1)
    ax.set_ylim(bottom=0)
    ax.legend(loc="upper right")
    guardar(fig, salida, "1_deteccion_derrapes.png", show)


# ===========================================================================
# GRÁFICA 2: valor de derrape en cada punto del recorrido, en 3D
# ===========================================================================
def grafica_2_derrape_recorrido(telemetria, pos_validas, indices, trayectorias, salida, show):
    """Réplica de la figura 5.12 (una por cámara): scatter 3D del valor de
    derrape sobre la posición del coche + trayectoria del controlador en z=0."""
    por_camara = {}
    for m, i in zip(telemetria, indices):
        if i is None:
            continue
        p = pos_validas[i]
        por_camara.setdefault(p["camara"], []).append((p["fx"], p["fy"], m["dist"]))

    for camara, puntos in sorted(por_camara.items()):
        xs, ys, zs = (np.array(v, dtype=float) for v in zip(*puntos))

        fig = plt.figure(figsize=(7.5, 6))
        ax = fig.add_subplot(projection="3d")
        tray = trayectorias.get(camara)
        if tray is not None and len(tray):
            ax.plot(
                tray[:, 0], tray[:, 1], zs=0, color=COLOR_TRAY_3D, lw=1, label="Posicion"
            )
        sc = ax.scatter(
            xs, ys, zs, c=zs, cmap="gnuplot", marker="*", s=10, vmin=0,
            label=r"Dist$_{derrape}$",
        )
        fig.colorbar(sc, ax=ax, shrink=0.55, pad=0.1)
        ax.set_zlim(bottom=0)
        ax.view_init(elev=35, azim=-60)
        ax.legend(loc="upper right")
        guardar(fig, salida, f"2_derrape_recorrido_{camara}.png", show)

    if not por_camara:
        print("  (sin datos emparejados: no se genera la gráfica 2)")


def _serie_velocidad(pos_validas):
    """Calcula la serie de velocidad instantánea en px/fotograma."""
    ultimo_por_camara = {}
    vel_x, vel_y = [], []
    for idx, p in enumerate(pos_validas):
        previo = ultimo_por_camara.get(p["camara"])
        if previo is not None:
            dt = (p["t"] - previo["t"]) / 1e9  # ns -> s
            salto = math.hypot(p["fx"] - previo["fx"], p["fy"] - previo["fy"])
            if dt <= MAX_DT_VELOCIDAD and salto <= MAX_SALTO_VELOCIDAD:
                vel_x.append(idx)
                vel_y.append(salto)
        ultimo_por_camara[p["camara"]] = p
    return vel_x, vel_y


# ===========================================================================
# GRÁFICA 3: relación entre los derrapes y la velocidad
# ===========================================================================
def grafica_3_derrapes_velocidad(telemetria, pos_validas, indices, salida, show):
    """Réplica de la figura 5.16: velocidad instantánea frente a derrapes."""
    vel_x, vel_y = _serie_velocidad(pos_validas)
    der_x = [i for m, i in zip(telemetria, indices) if i is not None]
    der_y = [m["dist"] for m, i in zip(telemetria, indices) if i is not None]

    fig, ax = plt.subplots(figsize=(7.5, 5))
    ax.plot(
        vel_x, vel_y, color=COLOR_VELOCIDAD, lw=0.5, marker="+", markersize=3,
        label="Velocidad",
    )
    ax.plot(
        der_x, der_y, color=COLOR_DERRAPE, lw=0.5, marker="x", markersize=3,
        label="Derrapes",
    )
    ax.set_xlim(0, len(pos_validas) if pos_validas else 1)
    ax.set_ylim(bottom=0)
    ax.legend(loc="upper right")
    guardar(fig, salida, "3_derrapes_velocidad.png", show)


def _cortar_saltos(puntos, salto_max=60.0):
    """Inserta NaN donde hay un salto grande entre puntos consecutivos."""
    if len(puntos) < 2:
        return puntos
    saltos = np.hypot(*np.diff(puntos, axis=0).T) > salto_max
    filas = []
    for i, p in enumerate(puntos):
        if i > 0 and saltos[i - 1]:
            filas.append([np.nan, np.nan])
        filas.append(p)
    return np.array(filas)


def _segmentos_eje_vehiculo(puntos):
    """Genera las series (x, y) separadas por NaN para unir con una recta
    la etiqueta frontal y trasera de cada instante en que ambas sean válidas."""
    x, y = [], []
    for p in puntos:
        if (p["fx"], p["fy"]) != (0, 0) and (p["bx"], p["by"]) != (0, 0):
            x.extend([p["fx"], p["bx"], np.nan])
            y.extend([p["fy"], p["by"], np.nan])
    return x, y


# ===========================================================================
# GRÁFICA 4: trayectorias de la etiqueta frontal y trasera
# (réplica de la figura 5.17 de Mario; una por cámara)
# ===========================================================================
# Qué muestra: el recorrido completo de las dos pegatinas durante toda la
# grabación, superpuesto vuelta sobre vuelta, en coordenadas de imagen de la
# cámara. Donde la trasera (magenta) se separa de la delantera (verde) es
# donde el coche abre la cola: las zonas de derrape se ven a simple vista.
# A diferencia de la figura de Mario, los puntos se dibujan SUELTOS (sin
# línea que los una en el tiempo): con decenas de vueltas superpuestas la nube
# de puntos se lee mucho mejor y las zonas más transitadas se ven más densas.
# Además, se une con un segmento recto la etiqueta frontal y trasera de cada
# instante (el eje longitudinal del vehículo), permitiendo ver el ángulo de
# derrape en cada posición.
#
# PARÁMETROS USADOS:
#
#   figsize=(7.5, 5.5)    tamaño de la figura en pulgadas.
#   linestyle="none"      SOLO puntos para las etiquetas, sin línea que los una.
#   marker="."            punto pequeño ("." es más fino que "o").
#   markersize=2          tamaño del punto; con miles de posiciones conviene pequeño.
#   color=...             magenta para trasera, verde para delantera y gris para
#                         los segmentos que unen ambas en cada instante.
#   lw=0.4, alpha=0.5     línea fina y semitransparente para que la superposición
#                         de cientos de segmentos no emborrone la figura.
#   label=...             leyendas en notación matemática.
#   ax.invert_yaxis()     INVIERTE el eje Y para coordenadas de imagen.
#   ax.set_aspect("equal", adjustable="datalim")
#                         misma escala en X e Y para no deformar el circuito.
# ===========================================================================
def grafica_4_trayectorias_etiquetas(posiciones, salida, show):
    """Réplica de la figura 5.17 (una por cámara): posiciones de las dos
    etiquetas y recta de unión en cada instante, en coordenadas de imagen."""
    por_camara = {}
    for p in posiciones:
        por_camara.setdefault(p["camara"], []).append(p)

    for camara, puntos in sorted(por_camara.items()):
        front = np.array(
            [(p["fx"], p["fy"]) for p in puntos if (p["fx"], p["fy"]) != (0, 0)], dtype=float
        )
        back = np.array(
            [(p["bx"], p["by"]) for p in puntos if (p["bx"], p["by"]) != (0, 0)], dtype=float
        )
        eje_x, eje_y = _segmentos_eje_vehiculo(puntos)

        fig, ax = plt.subplots(figsize=(7.5, 5.5))
        # Dibujamos las rectas de unión primero para que los puntos queden encima
        if eje_x:
            ax.plot(eje_x, eje_y, color=COLOR_EJE_VEHICULO, lw=0.4, alpha=0.5,
                    label=r"Eje$_{vehículo}$")
        if len(back):
            ax.plot(back[:, 0], back[:, 1], linestyle="none", marker=".",
                    markersize=2, color=COLOR_ETIQ_TRASERA, label=r"Etiq$_{Trasera}$")
        if len(front):
            ax.plot(front[:, 0], front[:, 1], linestyle="none", marker=".",
                    markersize=2, color=COLOR_ETIQ_DELANTERA, label=r"Etiq$_{Delantera}$")
        ax.invert_yaxis()
        ax.set_aspect("equal", adjustable="datalim")
        ax.legend(loc="upper right", markerscale=4)
        guardar(fig, salida, f"4_trayectorias_etiquetas_{camara}.png", show)


# ===========================================================================
# GRÁFICA 5: trayectoria que tiene guardada el controlador
# ===========================================================================
def grafica_5_trayectoria_controlador(trayectorias, salida, show):
    """Gráfica nueva: los puntos de trayectoria que guarda el controlador."""
    if not trayectorias:
        print("  (sin cache_trayectoria_controller.json: no se genera la gráfica 5)")
        return

    fig, ax = plt.subplots(figsize=(7.5, 5.5))
    for i, (camara, puntos) in enumerate(sorted(trayectorias.items())):
        color = PALETA_CAMARAS[i % len(PALETA_CAMARAS)]
        ax.plot(
            puntos[:, 0], puntos[:, 1], linestyle="none", marker="o", markersize=3,
            color=color, label=camara,
        )
    ax.invert_yaxis()
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="upper right")
    guardar(fig, salida, "5_trayectoria_controlador.png", show)


# ===========================================================================
# MODO INTERACTIVO POR VUELTAS (--vueltas)
# ===========================================================================
def _preparar_arco(trayectorias, pos_validas):
    """Construye el eje X "circuito estirado" de los paneles 1 y 3 del visor."""
    if not trayectorias:
        return None
    orden = []
    for p in pos_validas:
        if p["camara"] in trayectorias and p["camara"] not in orden:
            orden.append(p["camara"])
    for cam in sorted(trayectorias):
        if cam not in orden:
            orden.append(cam)

    arco = {"camaras": {}, "spans": [], "cortes_tramo": []}
    offset = 0.0
    for cam in orden:
        puntos = np.asarray(trayectorias[cam], dtype=float)
        if len(puntos) < 2:
            continue
        dif = np.diff(puntos, axis=0)
        dist = np.hypot(dif[:, 0], dif[:, 1])
        umbral_salto = max(4.0 * float(np.median(dist)), 60.0)
        tramos = np.zeros(len(puntos), dtype=int)
        s = np.zeros(len(puntos))
        for i in range(1, len(puntos)):
            if dist[i - 1] > umbral_salto:
                tramos[i] = tramos[i - 1] + 1
            else:
                tramos[i] = tramos[i - 1]
                s[i] = s[i - 1] + dist[i - 1]
        long_tramos = {int(t): float(s[tramos == t].max()) for t in np.unique(tramos)}
        cerrada = False
        if tramos[-1] == 0:
            cierre = float(np.hypot(*(puntos[-1] - puntos[0])))
            if cierre <= umbral_salto:
                cerrada = True
                long_tramos[0] += cierre
        inicio_cam = offset
        offsets = {}
        for t in sorted(long_tramos):
            offsets[t] = offset
            if t > 0:
                arco["cortes_tramo"].append(offset)
            offset += long_tramos[t]

        seg_a, seg_b, seg_s = [], [], []
        for i in range(len(puntos) - 1):
            if tramos[i] == tramos[i + 1]:
                seg_a.append(i)
                seg_b.append(i + 1)
                seg_s.append(offsets[int(tramos[i])] + s[i])
        if cerrada and len(puntos) > 1:
            seg_a.append(len(puntos) - 1)
            seg_b.append(0)
            seg_s.append(offsets[0] + s[-1])
        A = puntos[seg_a]
        AB = puntos[seg_b] - A
        l2 = (AB ** 2).sum(axis=1)
        validos = l2 > 0.0

        arco["camaras"][cam] = {
            "seg_A": A[validos],
            "seg_AB": AB[validos],
            "seg_l2": l2[validos],
            "seg_len": np.sqrt(l2[validos]),
            "seg_s": np.asarray(seg_s)[validos],
            "cerrada": cerrada,
            "long_cerrada": long_tramos.get(0, 0.0),
            "offset0": offsets[0],
        }
        arco["spans"].append((inicio_cam, offset, cam))
    if not arco["camaras"]:
        return None
    arco["total"] = offset
    return arco


def _proyectar_arco(info, punto):
    """Proyecta un punto (x, y) sobre TODOS los segmentos de la trayectoria."""
    p = np.asarray(punto, dtype=float)
    AP = p - info["seg_A"]
    t = np.clip((AP * info["seg_AB"]).sum(axis=1) / info["seg_l2"], 0.0, 1.0)
    proyeccion = info["seg_A"] + t[:, None] * info["seg_AB"]
    dist = np.hypot(*(p - proyeccion).T)
    s = info["seg_s"] + t * info["seg_len"]
    if info["cerrada"]:
        s = info["offset0"] + (s - info["offset0"]) % info["long_cerrada"]
    return dist, s


def _serie_s(arco, pos_validas):
    """Calcula la s global de la etiqueta delantera de cada posición válida."""
    s_pos = np.full(len(pos_validas), np.nan)
    previo = {}
    for i, p in enumerate(pos_validas):
        info = arco["camaras"].get(p["camara"])
        if info is None or not len(info["seg_l2"]):
            continue
        dist, s = _proyectar_arco(info, (p["fx"], p["fy"]))
        d_min = float(dist.min())
        if d_min > 80.0:
            continue
        ant = previo.get(p["camara"])
        if ant is not None and (p["t"] - ant["t"]) / 1e9 <= MAX_DT_VELOCIDAD:
            aceptable = dist <= d_min + 20.0
            dif = np.abs(s - ant["s"])
            if info["cerrada"]:
                dif = np.minimum(dif, info["long_cerrada"] - dif)
            dif[~aceptable] = np.inf
            j = int(np.argmin(dif))
        else:
            j = int(np.argmin(dist))
        s_pos[i] = float(s[j])
        previo[p["camara"]] = {"s": s_pos[i], "t": p["t"]}
    return s_pos


def _cortar_saltos_serie(x, y, salto_max=60.0):
    """Inserta NaN en una serie (x, y) allí donde x salta más de salto_max."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if len(x) < 2:
        return x, y
    cortes = np.where(np.abs(np.diff(x)) > salto_max)[0]
    return np.insert(x, cortes + 1, np.nan), np.insert(y, cortes + 1, np.nan)


def _marcar_camaras(ax, arco):
    """Rotula el eje "circuito estirado" de un panel."""
    trans = ax.get_xaxis_transform()
    for ini, fin, cam in arco["spans"]:
        if ini > 0:
            ax.axvline(ini, color="0.55", lw=0.6)
        ax.text((ini + fin) / 2, 0.02, cam, transform=trans,
                ha="center", va="bottom", fontsize=6, color="0.4")
    for x in arco["cortes_tramo"]:
        ax.axvline(x, color="0.75", lw=0.5, linestyle=":")


def _preparar_datos_vueltas(posiciones, pos_validas, telemetria, indices, vueltas, trayectorias):
    """Precalcula todo lo que _dibujar_vuelta necesita, UNA sola vez."""
    datos = {
        "vueltas": vueltas,
        "trayectorias": trayectorias,
        "posiciones": posiciones,
        "dist": np.array([m["dist"] for m in telemetria]),
        "derrapando": np.array([m["derrapando"] for m in telemetria], dtype=bool),
    }

    t_tel = [m["t"] for m in telemetria]
    t_pos = [p["t"] for p in pos_validas]
    t_todas = [p["t"] for p in posiciones]
    rangos = []
    for v in vueltas:
        fin = v["t"]
        ini = fin - int(v["tiempo"] * 1e9)
        rangos.append({
            "tel": (bisect.bisect_left(t_tel, ini), bisect.bisect_right(t_tel, fin)),
            "pos": (bisect.bisect_left(t_pos, ini), bisect.bisect_right(t_pos, fin)),
            "todas": (bisect.bisect_left(t_todas, ini), bisect.bisect_right(t_todas, fin)),
        })
    datos["rangos"] = rangos

    vel_x, vel_y = _serie_velocidad(pos_validas)
    datos["vel_x"] = np.array(vel_x, dtype=int)
    datos["vel_y"] = np.array(vel_y, dtype=float)
    datos["der_x"] = np.array(
        [i for m, i in zip(telemetria, indices) if i is not None], dtype=int
    )
    datos["der_y"] = np.array(
        [m["dist"] for m, i in zip(telemetria, indices) if i is not None], dtype=float
    )

    datos["arco"] = _preparar_arco(trayectorias, pos_validas)
    if datos["arco"] is not None:
        s_pos = _serie_s(datos["arco"], pos_validas)
        datos["s_pos"] = s_pos
        datos["s_tel"] = np.array(
            [s_pos[i] if i is not None else np.nan for i in indices]
        )
        datos["cam_pos"] = np.array([p["camara"] for p in pos_validas])
        datos["cam_tel"] = np.array(
            [pos_validas[i]["camara"] if i is not None else "" for i in indices]
        )

    pares = {}
    for m, i in zip(telemetria, indices):
        if i is None:
            continue
        p = pos_validas[i]
        pares.setdefault(p["camara"], []).append((i, p["fx"], p["fy"], m["dist"]))
    datos["pares"] = {c: np.array(v, dtype=float) for c, v in pares.items()}

    datos["camaras"] = sorted({p["camara"] for p in posiciones})

    margen = 15.0  # px
    encuadres = {}
    for cam in datos["camaras"]:
        xs, ys = [], []
        for p in posiciones:
            if p["camara"] != cam:
                continue
            for px, py in ((p["fx"], p["fy"]), (p["bx"], p["by"])):
                if (px, py) != (0, 0):
                    xs.append(px)
                    ys.append(py)
        if xs:
            encuadres[cam] = (min(xs) - margen, max(xs) + margen,
                              min(ys) - margen, max(ys) + margen)
    datos["encuadres"] = encuadres

    datos["max_dist"] = max(float(datos["dist"].max()) if len(datos["dist"]) else 0.0,
                            UMBRAL_DERRAPE)
    datos["max_vel"] = max(datos["vel_y"], default=1.0)
    return datos


def _dibujar_vuelta(fig, datos, k):
    """Borra la figura y dibuja los paneles de la vuelta k."""
    v = datos["vueltas"][k]
    a_tel, b_tel = datos["rangos"][k]["tel"]
    a_pos, b_pos = datos["rangos"][k]["pos"]
    a_tod, b_tod = datos["rangos"][k]["todas"]

    fig.clf()
    camaras = datos["camaras"]
    gs = fig.add_gridspec(1 + len(camaras), 2, hspace=0.45, wspace=0.3,
                          left=0.07, right=0.97, top=0.9, bottom=0.07)
    fig.suptitle(
        f"Vuelta {v['numero']} ({k + 1}/{len(datos['vueltas'])}) — {v['tiempo']:.3f} s"
        "      [clic en la figura y:  ←/→ ±1   ↑/↓ ±5   Inicio/Fin]",
        fontsize=10,
    )

    arco = datos["arco"]

    # --- Panel de la gráfica 1: detección de derrapes ---
    ax = fig.add_subplot(gs[0, 0])
    dist = datos["dist"][a_tel:b_tel]
    derr = datos["derrapando"][a_tel:b_tel]
    ax.axhline(UMBRAL_DERRAPE, color=COLOR_UMBRAL, lw=0.8, label="Umbral")
    if arco is not None:
        x = datos["s_tel"][a_tel:b_tel]
        cams_v = datos["cam_tel"][a_tel:b_tel]
        ax.plot(x[derr], np.full(int(derr.sum()), UMBRAL_DERRAPE), linestyle="none",
                marker="*", markersize=7, color=COLOR_DERRAPE, label="Derrape")
        etiqueta = r"Dist$_{derrape}$"
        for cam in camaras:
            en_cam = cams_v == cam
            if not en_cam.any():
                continue
            xc, yc = _cortar_saltos_serie(x[en_cam], dist[en_cam])
            ax.plot(xc, yc, color=COLOR_DIST, lw=0.6, marker=".", markersize=3,
                    label=etiqueta)
            etiqueta = None
        ax.set_xlim(0, arco["total"])
        ax.set_xlabel("Posición en el circuito, s (px de trayectoria)", fontsize=8)
        _marcar_camaras(ax, arco)
    else:
        x = np.arange(a_tel, b_tel)
        ax.plot(x[derr], np.full(int(derr.sum()), UMBRAL_DERRAPE), linestyle="none",
                marker="*", markersize=7, color=COLOR_DERRAPE, label="Derrape")
        ax.plot(x, dist, color=COLOR_DIST, lw=0.6, label=r"Dist$_{derrape}$")
        ax.set_xlim(a_tel, max(b_tel, a_tel + 1))
    ax.set_ylim(0, datos["max_dist"] * 1.05)
    ax.set_title("Detección de derrapes", fontsize=9)
    ax.set_ylabel("Distancia de derrape")
    ax.legend(loc="upper right", fontsize=7)

    # --- Panel de la gráfica 3: derrapes frente a velocidad ---
    ax = fig.add_subplot(gs[0, 1])
    i0, i1 = bisect.bisect_left(datos["vel_x"], a_pos), bisect.bisect_left(datos["vel_x"], b_pos)
    j0, j1 = bisect.bisect_left(datos["der_x"], a_pos), bisect.bisect_left(datos["der_x"], b_pos)
    if arco is not None:
        vx = datos["vel_x"][i0:i1]
        vs, vy, vcam = datos["s_pos"][vx], datos["vel_y"][i0:i1], datos["cam_pos"][vx]
        etiqueta = "Velocidad"
        for cam in camaras:
            en_cam = vcam == cam
            if not en_cam.any():
                continue
            xc, yc = _cortar_saltos_serie(vs[en_cam], vy[en_cam])
            ax.plot(xc, yc, color=COLOR_VELOCIDAD, lw=0.5, marker="+", markersize=3,
                    label=etiqueta)
            etiqueta = None
        dx = datos["der_x"][j0:j1]
        ds, dy, dcam = datos["s_pos"][dx], datos["der_y"][j0:j1], datos["cam_pos"][dx]
        etiqueta = "Derrapes"
        for cam in camaras:
            en_cam = dcam == cam
            if not en_cam.any():
                continue
            xc, yc = _cortar_saltos_serie(ds[en_cam], dy[en_cam])
            ax.plot(xc, yc, color=COLOR_DERRAPE, lw=0.5, marker="x", markersize=3,
                    label=etiqueta)
            etiqueta = None
        ax.set_xlim(0, arco["total"])
        ax.set_xlabel("Posición en el circuito, s (px de trayectoria)", fontsize=8)
        _marcar_camaras(ax, arco)
    else:
        ax.plot(datos["vel_x"][i0:i1], datos["vel_y"][i0:i1], color=COLOR_VELOCIDAD,
                lw=0.5, marker="+", markersize=3, label="Velocidad")
        ax.plot(datos["der_x"][j0:j1], datos["der_y"][j0:j1], color=COLOR_DERRAPE,
                lw=0.5, marker="x", markersize=3, label="Derrapes")
        ax.set_xlim(a_pos, max(b_pos, a_pos + 1))
    ax.set_ylim(0, max(datos["max_vel"], datos["max_dist"]) * 1.05)
    ax.set_title("Derrapes frente a velocidad", fontsize=9)
    ax.legend(loc="upper right", fontsize=7)

    # --- Una fila por cámara: gráfica 2 (3D) y gráfica 4 (etiquetas) ---
    for fila, cam in enumerate(camaras, start=1):
        encuadre = datos["encuadres"].get(cam)

        # Gráfica 2 de la vuelta: scatter 3D
        ax = fig.add_subplot(gs[fila, 0], projection="3d")
        tray = datos["trayectorias"].get(cam)
        if tray is not None and len(tray):
            ax.plot(tray[:, 0], tray[:, 1], zs=0, color=COLOR_TRAY_3D, lw=1)
        pares = datos["pares"].get(cam)
        if pares is not None and len(pares):
            i0 = int(np.searchsorted(pares[:, 0], a_pos))
            i1 = int(np.searchsorted(pares[:, 0], b_pos))
            sub = pares[i0:i1]
            if len(sub):
                ax.scatter(sub[:, 1], sub[:, 2], sub[:, 3], c=sub[:, 3],
                           cmap="gnuplot", marker="*", s=10,
                           vmin=0, vmax=datos["max_dist"])
        if encuadre:
            ax.set_xlim(encuadre[0], encuadre[1])
            ax.set_ylim(encuadre[2], encuadre[3])
        ax.set_zlim(0, datos["max_dist"] * 1.05)
        ax.view_init(elev=35, azim=-60)
        ax.set_title(f"Derrape en el recorrido — {cam}", fontsize=9)

        # Gráfica 4 de la vuelta: etiquetas unidas con el eje del vehículo
        ax = fig.add_subplot(gs[fila, 1])
        puntos = [p for p in datos["posiciones"][a_tod:b_tod] if p["camara"] == cam]
        front = np.array([(p["fx"], p["fy"]) for p in puntos
                          if (p["fx"], p["fy"]) != (0, 0)], dtype=float)
        back = np.array([(p["bx"], p["by"]) for p in puntos
                         if (p["bx"], p["by"]) != (0, 0)], dtype=float)
        eje_x, eje_y = _segmentos_eje_vehiculo(puntos)

        if eje_x:
            ax.plot(eje_x, eje_y, color=COLOR_EJE_VEHICULO, lw=0.6, alpha=0.6,
                    label=r"Eje$_{vehículo}$")
        if len(back):
            ax.plot(back[:, 0], back[:, 1], linestyle="none", marker=".",
                    markersize=3, color=COLOR_ETIQ_TRASERA, label=r"Etiq$_{Trasera}$")
        if len(front):
            ax.plot(front[:, 0], front[:, 1], linestyle="none", marker=".",
                    markersize=3, color=COLOR_ETIQ_DELANTERA, label=r"Etiq$_{Delantera}$")
        if encuadre:
            ax.set_xlim(encuadre[0], encuadre[1])
            ax.set_ylim(encuadre[3], encuadre[2])
        else:
            ax.invert_yaxis()
        ax.set_aspect("equal", adjustable="box")
        ax.set_title(f"Trayectorias de las etiquetas — {cam}", fontsize=9)
        ax.legend(loc="upper right", fontsize=7, markerscale=3)


def modo_por_vueltas(posiciones, pos_validas, telemetria, indices, vueltas, trayectorias):
    """Abre la ventana interactiva y la mantiene hasta que se cierre."""
    if not vueltas:
        print("El bag no contiene vueltas (TimePerLap): no hay nada que recorrer.")
        return

    datos = _preparar_datos_vueltas(
        posiciones, pos_validas, telemetria, indices, vueltas, trayectorias
    )

    for tecla, mapa in (("left", "keymap.back"), ("right", "keymap.forward"),
                        ("home", "keymap.home")):
        if tecla in plt.rcParams[mapa]:
            plt.rcParams[mapa].remove(tecla)

    fig = plt.figure(figsize=(13, 4.2 * (1 + len(datos["camaras"]))))
    estado = {"k": 0}

    def _on_key(evento):
        saltos = {"right": 1, "left": -1, "up": 5, "down": -5}
        if evento.key in saltos:
            estado["k"] = (estado["k"] + saltos[evento.key]) % len(vueltas)
        elif evento.key == "home":
            estado["k"] = 0
        elif evento.key == "end":
            estado["k"] = len(vueltas) - 1
        else:
            return
        _dibujar_vuelta(fig, datos, estado["k"])
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("key_press_event", _on_key)
    _dibujar_vuelta(fig, datos, estado["k"])
    if matplotlib.get_backend().lower() == "webagg":
        print(f"\n>> Abre http://localhost:{matplotlib.rcParams['webagg.port']} "
              "en el navegador.")
        print(">> HAZ CLIC SOBRE LA FIGURA (para que reciba el teclado) y usa: "
              "←/→ cambia de vuelta, ↑/↓ salta 5, Inicio/Fin primera/última.")
        print(">> Con la lupa de la barra de herramientas puedes hacer zoom en "
              "cualquier panel (cambiar de vuelta lo resetea).")
        print(">> Para terminar: Ctrl+C en esta terminal.")
    else:
        print("\nModo por vueltas: ←/→ cambia de vuelta, ↑/↓ salta 5, "
              "Inicio/Fin primera/última, s guarda PNG, q cierra.")
    plt.show()


# ---------------------------------------------------------------------------
# Resúmenes por consola (logs de carrera + tiempos por vuelta del bag)
# ---------------------------------------------------------------------------
def resumen_logs(carpeta: Path):
    """Imprime un resumen de cada derrapesLog_*.txt que haya en la carpeta."""
    logs = sorted(carpeta.rglob("derrapesLog_*.txt"))
    if not logs:
        print("No hay logs de carrera (derrapesLog_*.txt) en la carpeta.")
        return
    for log in logs:
        texto = log.read_text(errors="replace")
        limpias = re.findall(r"Vuelta (\d+) limpia", texto)
        nuevas = re.findall(r"nueva zona de derrape \[([\d.]+), ([\d.]+)\] en tramo (\d+)", texto)
        repetidos = re.findall(r"Anticipación aumentada a ([\d.]+)", texto)
        trayectoria = re.search(r"Trayectoria cargada: .*", texto)

        print(f"\n--- Resumen de {log.name} ---")
        if trayectoria:
            print(f"  {trayectoria.group(0)}")
        print(f"  Vueltas limpias: {len(limpias)}"
              + (f" ({', '.join(limpias)})" if limpias else ""))
        print(f"  Zonas de derrape nuevas: {len(nuevas)}")
        for s_ini, s_fin, tramo in nuevas:
            print(f"    tramo {tramo}: [{s_ini}, {s_fin}]")
        print(f"  Derrapes repetidos en zona: {len(repetidos)}"
              + (f" (anticipación final: {repetidos[-1]} px)" if repetidos else ""))


def resumen_vueltas(vueltas):
    """Imprime la tabla de tiempos por vuelta leída del bag."""
    if not vueltas:
        print("El bag no contiene tiempos por vuelta.")
        return
    tiempos = [v["tiempo"] for v in vueltas]
    print(f"\n--- Tiempos por vuelta ({len(vueltas)} vueltas) ---")
    for v in vueltas:
        print(f"  vuelta {v['numero']:>3}: {v['tiempo']:.3f} s")
    print(f"  mejor: {min(tiempos):.3f} s | media: {sum(tiempos) / len(tiempos):.3f} s"
          f" | peor: {max(tiempos):.3f} s")


# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("carpeta", type=Path,
                        help="Carpeta con el bag, los JSON de trayectoria y los logs")
    parser.add_argument("--coche", default="car1", help="Nombre del coche (def: car1)")
    parser.add_argument("--salida", type=Path, default=None,
                        help="Carpeta de salida de los PNG (def: <carpeta>/graficas)")
    parser.add_argument("--show", action="store_true", help="Además, mostrar en pantalla")
    parser.add_argument("--vueltas", action="store_true",
                        help="Modo interactivo: recorrer las gráficas vuelta a vuelta "
                             "en una ventana (flechas del teclado). No genera PNG.")
    args = parser.parse_args()

    carpeta = args.carpeta.resolve()
    if not carpeta.is_dir():
        sys.exit(f"ERROR: {carpeta} no es una carpeta")
    salida = args.salida or carpeta / "graficas"
    if args.show:
        matplotlib.use("TkAgg", force=True)
    if args.vueltas:
        matplotlib.use("WebAgg", force=True)
        matplotlib.rcParams.update(
            {
                "webagg.address": "0.0.0.0",
                "webagg.port": 8988,
                "webagg.open_in_browser": False,
            }
        )

    bag_dir = encontrar_bag(carpeta)
    print(f"Bag: {bag_dir}")
    posiciones, telemetria, vueltas = leer_bag(bag_dir, args.coche)
    print(f"Mensajes: {len(posiciones)} posiciones, {len(telemetria)} telemetrías, "
          f"{len(vueltas)} vueltas")

    trayectorias = cargar_trayectoria_controlador(carpeta)
    if trayectorias:
        print("Trayectoria del controlador: "
              + ", ".join(f"{c} ({len(p)} puntos)" for c, p in sorted(trayectorias.items())))

    pos_validas = [
        p for p in posiciones
        if (p["fx"], p["fy"]) != (0, 0) and (p["bx"], p["by"]) != (0, 0)
    ]
    indices = emparejar_telemetria(telemetria, pos_validas)
    emparejadas = sum(1 for i in indices if i is not None)
    print(f"Telemetrías emparejadas con su posición: {emparejadas}/{len(telemetria)}")

    if args.vueltas:
        estilo_gnuplot()
        resumen_vueltas(vueltas)
        modo_por_vueltas(posiciones, pos_validas, telemetria, indices, vueltas, trayectorias)
        return

    estilo_gnuplot()
    print("\nGenerando gráficas:")
    grafica_1_deteccion_derrapes(telemetria, salida, args.show)
    grafica_2_derrape_recorrido(telemetria, pos_validas, indices, trayectorias, salida, args.show)
    grafica_3_derrapes_velocidad(telemetria, pos_validas, indices, salida, args.show)
    grafica_4_trayectorias_etiquetas(posiciones, salida, args.show)
    grafica_5_trayectoria_controlador(trayectorias, salida, args.show)

    resumen_vueltas(vueltas)
    resumen_logs(carpeta)


if __name__ == "__main__":
    main()
