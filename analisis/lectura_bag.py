#!/usr/bin/env python3
"""
Lectura del bag mcap 

Saca del bag las cuatro series que usan las figuras (posiciones, telemetria,
tiempos de vuelta y ordenes de PWM)

Los timestamps "t" son los de grabacion del bag en nanosegundos: el instante
en que el grabador recibio cada mensaje. Todos los topics comparten reloj
"""

import bisect
import sys
from pathlib import Path

import yaml

# API oficial de ROS2 para leer bags:
#  - rosbag2_py: abre el bag y devuelve los mensajes serializados (bytes CDR)
#  - deserialize_message: convierte esos bytes en un objeto mensaje de Python
#  - get_message: dado el nombre del tipo ("image_processor_pkg/msg/CarLocation")
#    devuelve la clase Python generada al compilar el paquete con colcon
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Desfase maximo (s) admitido al emparejar un mensaje de telemetria con el
# mensaje de posicion que lo origino. 
# Subirlo empareja mas mensajes pero con mas riesgo
# de asociar una telemetria a una posicion que no es la suya
# Bajarlo deja mas telemetrias sin emparejar
MAX_DESFASE_EMPAREJADO = 0.2

def encontrar_bag(carpeta: Path) -> Path:
    """Devuelve la carpeta del bag (la que contiene metadata.yaml)"""

    if (carpeta / "metadata.yaml").is_file():
        return carpeta

    # Busqueda recursiva en los subdirectorios para buscar en los que hay metadata.yaml
    metadatas = sorted(carpeta.rglob("metadata.yaml"))
    if not metadatas:
        sys.exit(f"ERROR: no se encontró ningún bag (metadata.yaml) en {carpeta}")
    if len(metadatas) > 1:
        # Si hubiera varios bags se avisa y se usa el primero por orden
        # alfabetico; para analizar otro, pasar directamente su carpeta
        print(f"AVISO: hay {len(metadatas)} bags; se usa {metadatas[0].parent}")
    return metadatas[0].parent



def _decodificadores_mcap(bag_dir: Path):
    """Construye un decodificador por topic a partir de las definiciones de
    mensaje embebidas en el propio fichero mcap.

    Los mensajes se deserializan normalmente con las clases compiladas del
    paquete (deserialize_message), pero si los .msg del codigo han cambiado
    despues de grabar el bag, la clase actual ya no coincide con los bytes
    grabados y la deserializacion revienta. El formato mcap guarda dentro
    del fichero el texto completo de la definicion de cada tipo tal y como
    era al grabar, asi que mcap-ros2-support puede decodificar cualquier bag
    antiguo con su definicion original. Los objetos decodificados exponen
    los campos con la misma notacion (msg.front.center.x), por lo que el
    resto del codigo no nota la diferencia.

    Devuelve {nombre_topic: funcion(bytes) -> mensaje decodificado}"""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    decodificadores = {}
    fabrica = DecoderFactory()
    for fichero in sorted(bag_dir.glob("*.mcap")):
        with open(fichero, "rb") as f:
            resumen = make_reader(f).get_summary()

            if resumen is None:
                continue            

            # Creamos un decodificador para cada topic que guardado en el mcap 
            for canal in resumen.channels.values():
                esquema = resumen.schemas[canal.schema_id]
                decodificadores[canal.topic] = fabrica.decoder_for("cdr", esquema)

    return decodificadores



def leer_bag(bag_dir: Path, coche: str):
    """Lee del bag todo lo que usa el dashboard para el coche indicado.

    Devuelve un dict con:
      posiciones  [{t, camara, fx, fy, bx, by}]   /coche/position
      telemetria  [{t, dist, derrapando}]         /telemetria/coche/car_control
      vueltas     [{numero, tiempo, t}]           /telemetria/coche/time_per_lap
                  ("t" marca el fin de la vuelta: el mensaje se publica al
                  cruzar meta, asi que la vuelta ocupa [t - tiempo*1e9, t])
      pwm         [{t, pwm}]                      /coche/pwd

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
    topic_pwm = f"/{coche}/pwd"

    # Diccionario que guarda todos los topics con su tipo y nombre que hay en el mcap
    tipos = {t.name: t.type for t in reader.get_all_topics_and_types()}

    interesantes = [topic_pos, topic_tel, topic_lap, topic_pwm]
    
    # Comprobamos que estan los 4 que nos interesan
    faltan = [t for t in interesantes if t not in tipos]
    # Si falta alguno damos error
    if topic_pos in faltan:
        sys.exit(f"ERROR: el bag no contiene {topic_pos} (topics: {sorted(tipos)})")
    if faltan:
        print(f"AVISO: el bag no contiene {faltan}")

    # Generamos un filtro para quedarnos solo con los mensajes que nos interesan 
    # El fichero se tiene que leer de manera secuencial mensaje a mensaje
    presentes = [t for t in interesantes if t in tipos]
    reader.set_filter(rosbag2_py.StorageFilter(topics=presentes))

    clases = {t: get_message(tipos[t]) for t in presentes}
    decodificadores = None  # se construyen solo si una deserializacion falla

    posiciones, telemetria, vueltas, pwm = [], [], [], []
    while reader.has_next():
        """Leer el siguiente mensaje"""
        topic, data, t_ns = reader.read_next()
        msg = None
        # Compromaos que el topic es el que queremos
        if clases[topic] is not None:
            try:
                # Cogemos el contenido del mensaje
                msg = deserialize_message(data, clases[topic])
            except Exception:
                # La definicion actual del .msg no coincide con la del bag
                # este topic pasa a decodificarse con el esquema embebido en el mcap
                clases[topic] = None
                print(f"AVISO: la definición actual de {tipos[topic]} no coincide "
                      f"con la grabada en el bag; {topic} se decodifica con el "
                      "esquema embebido en el mcap")
        if msg is None:
            if decodificadores is None:
                decodificadores = _decodificadores_mcap(bag_dir)
            msg = decodificadores[topic](data)

        
        """Comprobar para el mensaje actual a cual del los 4 topics corresponde"""
        if topic == topic_pos:
            # Posicion del coche vista por una camara: centros de las dos
            posiciones.append(
                {
                    "t": t_ns,
                    "camara": msg.camara_id,
                    "fx": msg.front.center.x,
                    "fy": msg.front.center.y,
                    "bx": msg.back.center.x,
                    "by": msg.back.center.y,
                    "n_frame": getattr(msg, "n_frame", None),
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
        elif topic == topic_pwm:
            # El carril (msg.carril) no se guarda ya se filtro por coche al
            # elegir el topic. Solo se hace para un coche de cada vez 
            pwm.append({"t": t_ns, "pwm": int(msg.pwm)})

    return {
        "posiciones": posiciones,
        "telemetria": telemetria,
        "vueltas": vueltas,
        "pwm": pwm,
    }


def emparejar_telemetria(telemetria, posiciones_validas):
    """Asocia cada mensaje de telemetria con el mensaje de posicion (valido)
    anterior mas cercano en tiempo de grabacion.

    CarControlTelemetry no lleva camara_id ni las coordenadas del coche, solo
    dist_derrape. Como el controlador publica una telemetria inmediatamente
    despues de procesar cada posicion, y el bag graba ambos topics con el
    mismo reloj, la posicion que origino una telemetria es la ultima posicion
    grabada antes que ella (a milisegundos de distancia). bisect_right hace
    la busqueda binaria sobre los tiempos (ordenados: el bag se lee en orden).

    Devuelve una lista paralela a `telemetria` con el indice de la posicion
    emparejada en posiciones_validas, o None si la mas cercana esta a mas de
    MAX_DESFASE_EMPAREJADO segundos"""

    # Ordenados porque el bag se lee en orden
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


def pwm_en_instantes(tiempos_ns, pwm):
    """PWM aplicado en cada instante de `tiempos_ns`. El del ultimo mensaje de
    /coche/pwd anterior o igual a ese instante, porque una orden sigue en vigor
    hasta que llega la siguiente.

    Devuelve una lista paralela a tiempos_ns con el valor entero, o None si
    ese instante es anterior a la primera orden grabada"""
    if not pwm:
        return [None] * len(tiempos_ns)

    # Ordenados porque el bag se lee en orden
    tiempos_pwm = [m["t"] for m in pwm]  
    resultado = []
    for t in tiempos_ns:
        i = bisect.bisect_right(tiempos_pwm, t) - 1
        resultado.append(pwm[i]["pwm"] if i >= 0 else None)
    return resultado


def repartir_por_vueltas(tiempos_ns, vueltas):
    """Numero de vuelta de cada timestamp, usando las ventanas de tiempo de
    los mensajes time_per_lap.

    Devuelve una lista paralela a tiempos_ns con el numero de vuelta o None
    si el instante no cae en ninguna vuelta"""

    if not vueltas:
        return [None] * len(tiempos_ns)

    finales = [v["t"] for v in vueltas]  
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
