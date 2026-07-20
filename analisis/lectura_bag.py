#!/usr/bin/env python3
"""
Lectura del bag mcap para el dashboard unificado (analisis.py).

encontrar_bag, cargar_trayectoria_controlador, _decodificadores_mcap y
emparejar_telemetria están movidos TAL CUAL de generar_graficas.py; leer_bag
es el mismo patrón ampliado con las imágenes de debug de las cámaras y
repartir_por_vueltas es nuevo (asigna cada mensaje a su vuelta usando las
ventanas de tiempo de /telemetria/<coche>/time_per_lap).

Los timestamps "t" son los de GRABACIÓN del bag en nanosegundos: el instante
en que el grabador recibió cada mensaje. Todos los topics comparten reloj
(los graba la misma máquina), así que se pueden comparar entre sí.
"""

import bisect
import json
import re
import sys
from pathlib import Path

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

# Desfase máximo (s) admitido al emparejar un mensaje de telemetría con el
# mensaje de posición que lo originó (se emparejan por tiempo de grabación,
# ver emparejar_telemetria). Subirlo empareja más mensajes pero con más riesgo
# de asociar una telemetría a una posición que no es la suya; bajarlo deja
# más telemetrías sin emparejar.
MAX_DESFASE_EMPAREJADO = 0.2


# ---------------------------------------------------------------------------
# Localización de los ficheros dentro de la carpeta de datos
# ---------------------------------------------------------------------------
def encontrar_bag(carpeta: Path) -> Path:
    """Devuelve la carpeta del bag (la que contiene metadata.yaml).

    Un bag de rosbag2 es una CARPETA con un metadata.yaml y uno o más .mcap,
    así que basta con buscar el metadata.yaml de forma recursiva (rglob) por
    si el bag está en una subcarpeta (p. ej. PRUEBAS/X/001_LUNES_13_08/)."""
    if (carpeta / "metadata.yaml").is_file():
        return carpeta
    metadatas = sorted(carpeta.rglob("metadata.yaml"))
    if not metadatas:
        sys.exit(f"ERROR: no se encontró ningún bag (metadata.yaml) en {carpeta}")
    if len(metadatas) > 1:
        # Si hubiera varios bags se avisa y se usa el primero por orden
        # alfabético; para analizar otro, pasar directamente su carpeta.
        print(f"AVISO: hay {len(metadatas)} bags; se usa {metadatas[0].parent}")
    return metadatas[0].parent


def cargar_trayectoria_controlador(carpeta: Path):
    """Carga cache_trayectoria_controller.json: {camara_id: array (N, 2)}.

    Es la caché que CarControllerNode escribe al terminar la calibración:
    la ruta "limpia" (nodos separados >= 15 px) que ve cada cámara, en
    píxeles de esa cámara. Devuelve {} si el fichero no está."""
    ficheros = sorted(carpeta.rglob("cache_trayectoria_controller.json"))
    if not ficheros:
        return {}
    with open(ficheros[0]) as f:
        datos = json.load(f)
    return {cam: np.asarray(puntos, dtype=float) for cam, puntos in datos.items()}


# ---------------------------------------------------------------------------
# Plan B de decodificación para bags grabados con .msg que luego cambiaron
# ---------------------------------------------------------------------------
def _decodificadores_mcap(bag_dir: Path):
    """Construye un decodificador por topic a partir de las definiciones de
    mensaje EMBEBIDAS en el propio fichero mcap.

    Los mensajes se deserializan normalmente con las clases compiladas del
    paquete (deserialize_message), pero si los .msg del código han cambiado
    DESPUÉS de grabar el bag, la clase actual ya no coincide con los bytes
    grabados y la deserialización revienta. El formato mcap guarda dentro
    del fichero el texto completo de la definición de cada tipo tal y como
    era al grabar, así que mcap-ros2-support puede decodificar cualquier bag
    antiguo con SU definición original. Los objetos decodificados exponen
    los campos con la misma notación (msg.front.center.x), por lo que el
    resto del código no nota la diferencia.

    Devuelve {nombre_topic: funcion(bytes) -> mensaje decodificado}."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    decodificadores = {}
    fabrica = DecoderFactory()
    for fichero in sorted(bag_dir.glob("*.mcap")):
        with open(fichero, "rb") as f:
            resumen = make_reader(f).get_summary()
            if resumen is None:
                continue  # mcap sin índice (grabación cortada): se ignora
            for canal in resumen.channels.values():
                esquema = resumen.schemas[canal.schema_id]
                decodificadores[canal.topic] = fabrica.decoder_for("cdr", esquema)
    return decodificadores


# ---------------------------------------------------------------------------
# Lectura del bag
# ---------------------------------------------------------------------------
def leer_bag(bag_dir: Path, coche: str, con_imagenes: bool = False):
    """Lee del bag todo lo que usa el dashboard para el coche indicado.

    Devuelve un dict con:
      posiciones  [{t, camara, fx, fy, bx, by}]   /coche/position
      telemetria  [{t, dist, derrapando}]         /telemetria/coche/car_control
      vueltas     [{numero, tiempo, t}]           /telemetria/coche/time_per_lap
                  ("t" marca el FIN de la vuelta: el mensaje se publica al
                  cruzar meta, así que la vuelta ocupa [t - tiempo*1e9, t])
      imagenes    {camara: [(t, bytes jpeg)]}     /camara_XX/camara_debug
                  (solo si con_imagenes=True: es lo más pesado del bag)
    """
    with open(bag_dir / "metadata.yaml") as f:
        metadata = yaml.safe_load(f)["rosbag2_bagfile_information"]
    storage_id = metadata.get("storage_identifier", "mcap")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )

    topic_pos = f"/{coche}/position"
    topic_tel = f"/telemetria/{coche}/car_control"
    topic_lap = f"/telemetria/{coche}/time_per_lap"
    tipos = {t.name: t.type for t in reader.get_all_topics_and_types()}
    # Los topics de imagen se descubren por patrón (hay uno por cámara)
    topics_img = sorted(t for t in tipos if re.fullmatch(r"/camara_[^/]+/camara_debug", t))

    interesantes = [topic_pos, topic_tel, topic_lap] + (topics_img if con_imagenes else [])
    faltan = [t for t in [topic_pos, topic_tel, topic_lap] if t not in tipos]
    if topic_pos in faltan:
        sys.exit(f"ERROR: el bag no contiene {topic_pos} (topics: {sorted(tipos)})")
    if faltan:
        print(f"AVISO: el bag no contiene {faltan}")
    # Filtro de topics: el reader se salta todo lo demás (incluidos /rosout y
    # compañía que entran con la grabación --all, y las imágenes si no se piden)
    presentes = [t for t in interesantes if t in tipos]
    reader.set_filter(rosbag2_py.StorageFilter(topics=presentes))

    clases = {t: get_message(tipos[t]) for t in presentes}
    decodificadores = None  # se construyen solo si una deserialización falla

    posiciones, telemetria, vueltas = [], [], []
    imagenes = {}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        # Las imágenes se guardan tal cual: el campo data de CompressedImage
        # es el JPEG completo y está al final del mensaje serializado; se
        # deserializa igual (es barato: sensor_msgs sí está instalado)
        msg = None
        if clases[topic] is not None:
            try:
                msg = deserialize_message(data, clases[topic])
            except Exception:
                # La definición actual del .msg no coincide con la del bag
                # (cambió después de grabar): este topic pasa a decodificarse
                # con el esquema embebido en el mcap de aquí en adelante
                clases[topic] = None
                print(f"AVISO: la definición actual de {tipos[topic]} no coincide "
                      f"con la grabada en el bag; {topic} se decodifica con el "
                      "esquema embebido en el mcap")
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
            telemetria.append(
                {
                    "t": t_ns,
                    "dist": msg.dist_derrape,
                    "derrapando": msg.estado_derrapando,
                }
            )
        elif topic == topic_lap:
            vueltas.append({"numero": msg.lap_number, "tiempo": msg.lap_time, "t": t_ns})
        else:  # imagen de debug: el nombre de cámara es el primer tramo del topic
            camara = topic.split("/")[1]
            imagenes.setdefault(camara, []).append((t_ns, bytes(msg.data)))

    return {
        "posiciones": posiciones,
        "telemetria": telemetria,
        "vueltas": vueltas,
        "imagenes": imagenes,
    }


def emparejar_telemetria(telemetria, posiciones_validas):
    """Asocia cada mensaje de telemetría con el mensaje de posición (válido)
    anterior más cercano en tiempo de grabación.

    CarControlTelemetry no lleva camara_id ni las coordenadas del coche, solo
    dist_derrape. Como el controlador publica una telemetría inmediatamente
    después de procesar cada posición, y el bag graba ambos topics con el
    mismo reloj, la posición que originó una telemetría es la última posición
    grabada antes que ella (a milisegundos de distancia). bisect_right hace
    la búsqueda binaria sobre los tiempos (ordenados: el bag se lee en orden).

    Devuelve una lista paralela a `telemetria` con el índice de la posición
    emparejada en posiciones_validas, o None si la más cercana está a más de
    MAX_DESFASE_EMPAREJADO segundos (mensaje perdido por el QoS best-effort:
    mejor descartar que emparejar mal)."""
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


def repartir_por_vueltas(tiempos_ns, vueltas):
    """Número de vuelta de cada timestamp, usando las ventanas de tiempo de
    los mensajes time_per_lap: la vuelta `numero` termina en su "t" y empieza
    tiempo*1e9 antes.

    Devuelve una lista paralela a tiempos_ns con el número de vuelta o None
    si el instante no cae en ninguna vuelta (calibración, huecos entre el
    debounce de meta, o después de la última vuelta cronometrada)."""
    if not vueltas:
        return [None] * len(tiempos_ns)
    finales = [v["t"] for v in vueltas]  # ordenados: el bag se lee en orden
    resultado = []
    for t in tiempos_ns:
        i = bisect.bisect_left(finales, t)
        if i >= len(vueltas):
            resultado.append(None)
            continue
        v = vueltas[i]
        inicio = v["t"] - int(v["tiempo"] * 1e9)
        resultado.append(v["numero"] if t >= inicio else None)
    return resultado
