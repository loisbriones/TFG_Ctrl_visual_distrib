#!/usr/bin/python3
"""
Nodo de VISUALIZACION EN DIRECTO de la carrera.

Se suscribe a los topics que ya publica la arquitectura, mantiene el estado de
la carrera en memoria y lo sirve por HTTP a un navegador. No publica NADA en
ROS y no toca a nadie: se puede levantar y tirar en mitad de una carrera sin
que el resto del sistema se entere.

Lo que se ve (referencia: la pantalla de "directo" de una retransmision de F1):
tabla de clasificacion con los tiempos coloreados, plano del circuito con los
coches como puntos, historico de eventos y una barra de salud del sistema.

Como llega al navegador
-----------------------
Servidor HTTP de la libreria estandar (mismo patron que analisis/analisis.py) y
push por SSE (Server-Sent Events). SSE y no WebSocket a proposito: la imagen del
proyecto (python/Dockerfile) es ros:humble-ros-base sin un solo pip install, y
un WebSocket necesitaria una libreria externa. SSE es un
"Content-Type: text/event-stream" y escribir "data: {...}\n\n" cada vez que hay
algo nuevo; el navegador lo lee con EventSource y reconecta el solo si se cae.

La pagina son ficheros normales en web/ (index.html, estilos.css y panel.js, mas
camaras.html y camaras.js del mosaico), no un string dentro de este fichero: asi
se pueden abrir y editar con resaltado de sintaxis y aviso de errores. Ademas se
leen del disco en cada peticion, asi que retocar el CSS o el JS y recargar el
navegador basta.

Las imagenes: la pagina /camaras
--------------------------------
Segunda pagina, en el mismo servidor, con un mosaico de lo que ve cada camara.
Es la herramienta de ANTES de la carrera: se levantan solo los nodos camara, se
abre esta pagina y se van encuadrando hasta que entre todas se vea el circuito
entero, sin tener que ir maquina por maquina enchufando el USB.

Las camaras ya publican /camara_XX/camara_debug con la imagen COMPRIMIDA en
JPEG (CompressedImage), asi que aqui no se decodifica nada: los bytes de
msg.data se le pasan tal cual a un <img> del navegador con el formato MJPEG
(multipart/x-mixed-replace, el mismo de toda la vida de las camaras IP). Sin
cv_bridge, sin OpenCV y sin una sola linea de JavaScript de video: el <img> lo
hace todo.

Y la suscripcion es BAJO DEMANDA. Las imagenes son el grueso del trafico de la
red -por eso el panel de datos nunca se suscribio a ellas- asi que cada camara
lleva la cuenta de cuantos navegadores la estan mirando ('visores'): el hilo
HTTP solo toca ese contador y es ajustar_subs_imagen, en el hilo de ROS, quien
da de alta la suscripcion cuando pasa de 0 y la destruye cuando vuelve a 0. Con
la pagina cerrada -o con esa camara en pausa- el trafico es exactamente el
mismo que antes de que existiera esta pagina.

El mapa y las varias camaras
----------------------------
Cada camara tiene su propio sistema de pixeles (su imagen) y NO hay ninguna
calibracion entre ellas: no se puede saber desde aqui como encajan dos trozos
de circuito vistos por dos camaras distintas. analisis/figuras.py resolvio esto
con traslaciones ajustadas a mano (OFFSETS_CAMARAS); aqui se hace lo mismo pero
el ajuste lo hace la persona arrastrando cada camara con el raton, y el
navegador se acuerda de la posicion. El boton "copiar offsets" de la pagina
vuelca el resultado con el formato de OFFSETS_CAMARAS para pegarlo alli y que
el analisis posterior use el mismo encaje.

El TRAZADO del circuito no se le pide a nadie: se dibuja acumulando las
posiciones de la pegatina delantera que ya llegan por /carX/position. Durante la
vuelta de calibracion el circuito se dibuja solo. Asi el panel no depende de
cache_trayectoria_controller.json, que vive dentro del contenedor del cerebro y
no existe al reproducir un bag.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from image_processor_pkg.msg import (
    CarLocation,
    SpeedCarril,
    FinishLine,
    TimePerLap,
    CarControlTelemetry,
)

from ament_index_python.packages import get_package_share_directory
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from collections import deque
import numpy as np
import threading
import json
import time
import os
import re

# La pagina son tres ficheros normales (HTML, CSS y JS) en la carpeta web/ del
# paquete, no un string dentro de este fichero: asi el editor da resaltado,
# autocompletado y avisos de error al tocarlos, que dentro de un string de
# Python no habia. install(DIRECTORY ... web) del CMakeLists los deja en el
# share del paquete, que es donde los busca get_package_share_directory
DIR_WEB = os.path.join(get_package_share_directory("image_processor_pkg"), "web")

# Lista blanca de lo que el servidor esta dispuesto a servir, con su tipo. Es
# una lista y no una traduccion de URL a ruta a proposito: asi no existe la
# posibilidad de pedirle un fichero que no sea uno de estos tres
FICHEROS_WEB = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/index.html": ("index.html", "text/html; charset=utf-8"),
    "/estilos.css": ("estilos.css", "text/css; charset=utf-8"),
    "/panel.js": ("panel.js", "application/javascript; charset=utf-8"),
    # Segunda pagina: el mosaico de lo que ve cada camara (ver "Las imagenes"
    # mas abajo). Comparte estilos.css con el panel
    "/camaras": ("camaras.html", "text/html; charset=utf-8"),
    "/camaras.html": ("camaras.html", "text/html; charset=utf-8"),
    "/camaras.js": ("camaras.js", "application/javascript; charset=utf-8"),
}


# Puerto del panel. 8988 ya lo usa el dashboard de analisis (analisis/analisis.py),
# asi que el directo va en el siguiente para poder tener los dos abiertos a la
# vez: el bag de la carrera de antes y la carrera de ahora
PUERTO = 8990

# Cada cuanto se le manda una instantanea al navegador. 10 Hz es mas que de
# sobra para el ojo (las camaras publican a ~30 Hz) y mantiene el trafico en
# unos pocos cientos de bytes por envio, porque solo van los datos que cambian
PERIODO_ENVIO = 0.1

# Ritmo maximo al que se le manda una imagen al navegador en /camaras. Mismo
# criterio que PERIODO_ENVIO: 10 imagenes por segundo sobran para encuadrar una
# camara, y con cuatro mosaicos abiertos evita cargar al navegador con 120
# JPEG/s que no le aportan nada al ojo
PERIODO_IMAGEN = 0.1

# Segundos sin recibir ni un JPEG tras los que se cierra el stream de una
# camara. Es el caso de una camara con 'camera.debug: False': existe en el grafo
# pero no publica imagenes, y sin esto el hilo del navegador se quedaria dando
# vueltas para siempre. La pagina lo reintenta sola en su siguiente sondeo
SIN_IMAGEN = 10.0

# Segundos sin recibir /carX/position para dar el coche por perdido y pintarlo
# en gris. Generoso a proposito respecto al timeout_camara_activa del
# controlador (0.3 s): aqui no se decide nada, solo se avisa a quien mira
TIMEOUT_SIN_SENAL = 2.0

# La linea de meta se publica UNA sola vez al arrancar la camara, asi que hay
# que suscribirse con el mismo QoS latcheado que usa el CarControllerNode para
# recibirla aunque el panel se levante a mitad de carrera
QOS_FINISH_LINE = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
)

# Color de cada coche en el mapa y en la tabla, por orden de aparicion. NO son
# los colores de sus pegatinas: las pegatinas son verde/azul en TODOS los
# coches (lo que cambia entre coches es la pareja, no un color "del coche"),
# asi que pintar el punto del color de la pegatina no distinguiria nada
COLORES_COCHES = ["#e6194b", "#4363d8", "#3cb44b", "#f58231", "#911eb4"]

# Los coches viven en /carX/... y las camaras en /camara_XX/...; el panel no
# necesita que nadie le diga cuantos hay, los descubre mirando el grafo (mismo
# criterio que analisis/lectura_bag.py al recorrer los topics de un bag)
RE_COCHE = re.compile(r"^/(car[^/]+)/position$")
RE_CAMARA = re.compile(r"^/(camara_[^/]+)/camara_debug$")

# La unica ruta del servidor que no es fija: /video/camara_01 es el MJPEG de esa
# camara. El nombre se valida contra self.camaras antes de usarlo, asi que de
# aqui no puede salir nada que no sea una camara que ya existe
RE_RUTA_VIDEO = re.compile(r"^/video/(camara_[^/]+)$")


class VisualizacionNode(Node):
    def __init__(self):
        super().__init__("visualizacion")

        # Rango de PWM del perfil: es la escala de la barra de velocidad de la
        # tabla. Sin esto habria que pintarla sobre 0-255 y el recorrido real
        # (65-80) quedaria en una rayita invisible
        self.declare_parameter("controller.minimum_speed", 65)
        self.v_min = float(self.get_parameter("controller.minimum_speed").value)
        self.declare_parameter("controller.maximum_speed", 80)
        self.v_max = float(self.get_parameter("controller.maximum_speed").value)

        # Mismo adelgazado que aplica el controlador a la trayectoria base: un
        # punto nuevo solo si esta a mas de esta distancia de los que ya hay
        self.declare_parameter("controller.distancia_nodos_trayectoria", 15.0)
        self.paso_trazado = float(
            self.get_parameter("controller.distancia_nodos_trayectoria").value
        )

        # Todo el estado vive aqui dentro y lo tocan dos mundos: los callbacks
        # de ROS (un solo hilo, spin normal) y los hilos del servidor HTTP (uno
        # por navegador conectado). El lock es lo unico que los separa
        self.lock = threading.Lock()

        self.coches = {}  # nombre -> dict con todo lo del coche
        self.camaras = {}  # camara_id -> dict con trazado, meta y ritmo
        self.subs = {}  # topic -> suscripcion, para no darla de alta dos veces

        # Los eventos se numeran para poder mandarle a cada navegador solo los
        # que le faltan. El deque acotado es la memoria del panel: quien entre
        # tarde vera los ultimos 200, no la carrera entera
        self.eventos = deque(maxlen=200)
        self.n_evento = 0

        # Mejor vuelta de la sesion, de cualquier coche: es la que se pinta en
        # morado en la tabla (el verde es el mejor personal de cada uno)
        self.mejor_absoluto = None

        self.t_arranque = time.monotonic()

        # /modo_calibracion global es el nombre ANTIGUO (una sola calibracion
        # para todo el sistema); hoy es /carX/modo_calibracion, uno por coche.
        # Se aceptan los dos para que el panel valga tambien reproduciendo
        # bags viejos, donde solo existe el global
        self.crear_sub(
            "/modo_calibracion", Bool, lambda m: self.cb_calibracion(m, None), 10
        )

        # La meta se suscribe ya, sin esperar a descubrir ningun coche: se
        # publica una sola vez al arrancar la camara. El QoS latcheado la
        # entrega igualmente a quien llegue tarde, pero cuanto antes este la
        # suscripcion menos depende el panel de eso (reproduciendo un bag, el
        # mensaje sale al principio y no se vuelve a repetir)
        self.crear_sub(
            "/finish_line_position", FinishLine, self.cb_meta, QOS_FINISH_LINE
        )

        # El grafo de ROS se mira cada 2 s en vez de una sola vez al arrancar:
        # las camaras estan en Raspberrys que pueden encenderse despues, y con
        # ros2 bag play los publicadores aparecen cuando arranca la
        # reproduccion, casi siempre despues que el panel
        self.create_timer(2.0, self.descubrir)
        self.create_timer(0.5, self.vigilar_senal)
        self.create_timer(0.5, self.ajustar_subs_imagen)

        hilo = threading.Thread(target=self.servir, daemon=True)
        hilo.start()

        self.get_logger().info(
            f"🖥️  Panel de carrera en http://localhost:{PUERTO}  "
            f"(esperando a que publique alguien)"
        )
        self.get_logger().info(
            f"📷 Camaras en directo en http://localhost:{PUERTO}/camaras"
        )

    # -----------------------------------------------------------------
    # Descubrimiento: quien hay en la carrera
    # -----------------------------------------------------------------
    def crear_sub(self, topic, tipo, callback, qos):
        if topic in self.subs:
            return
        self.subs[topic] = self.create_subscription(tipo, topic, callback, qos)

    def descubrir(self):
        for topic, _tipos in self.get_topic_names_and_types():
            m = RE_COCHE.match(topic)
            if m:
                self.alta_coche(m.group(1))
                continue
            m = RE_CAMARA.match(topic)
            if m:
                # Del topic de imagenes solo se coge, aqui, el NOMBRE de la
                # camara: con eso la barra de salud ya puede avisar de que la
                # camara esta ahi pero no ve el coche. La suscripcion a las
                # imagenes no se crea aqui, porque son el grueso del trafico de
                # la red: la crea ajustar_subs_imagen, y solo mientras alguien
                # las este mirando en /camaras
                self.alta_camara(m.group(1))

    def alta_coche(self, coche):
        with self.lock:
            if coche in self.coches:
                return
            self.coches[coche] = {
                "nombre": coche,
                "color": COLORES_COCHES[len(self.coches) % len(COLORES_COCHES)],
                "carril": None,
                "vuelta": 0,
                "ultimo_tiempo": None,
                "tiempo_anterior": None,
                "mejor_tiempo": None,
                "mejor_vuelta": None,
                # Instante del ultimo cruce de meta: desempata la clasificacion
                # entre dos coches que van por la misma vuelta
                "t_cruce": None,
                "pwm": None,
                "derrapando": False,
                "dist_derrape": 0.0,
                "camara": None,
                "front": None,
                "back": None,
                "pipeline_ms": None,
                "calibrando": None,
                "t_posicion": None,
                "sin_senal": False,
            }
        self.evento("coche", f"🏎️ {coche} en la sesión", coche)

        self.crear_sub(
            f"/{coche}/position",
            CarLocation,
            lambda m, c=coche: self.cb_posicion(m, c),
            qos_profile_sensor_data,
        )
        self.crear_sub(
            f"/telemetria/{coche}/car_control",
            CarControlTelemetry,
            lambda m, c=coche: self.cb_telemetria(m, c),
            qos_profile_sensor_data,
        )
        self.crear_sub(
            f"/telemetria/{coche}/time_per_lap",
            TimePerLap,
            lambda m, c=coche: self.cb_vuelta(m, c),
            qos_profile_sensor_data,
        )
        # En modo manual el controlador NO publica pwd (conduce una persona):
        # la suscripcion se queda sin emisor y la columna PWM en "—". Es el
        # comportamiento correcto, no un fallo
        self.crear_sub(
            f"/{coche}/pwd",
            SpeedCarril,
            lambda m, c=coche: self.cb_pwm(m, c),
            qos_profile_sensor_data,
        )
        self.crear_sub(
            f"/{coche}/modo_calibracion",
            Bool,
            lambda m, c=coche: self.cb_calibracion(m, c),
            10,
        )

    def alta_camara(self, camara):
        with self.lock:
            if camara in self.camaras:
                return
            self.camaras[camara] = {
                "id": camara,
                # Un trazado POR COCHE, no uno por camara: cada coche va por su
                # carril y los dos carriles son dos curvas paralelas separadas.
                # Metiendolos en la misma lista, el trazado saltaba de un carril
                # al otro y el circuito salia dibujado en zigzag. Separados, el
                # mapa enseña ademas la linea de carrera de cada uno
                "trazos": {},  # coche -> {"puntos": [[x, y], ...], "np": (N,2)}
                "meta": None,
                # Marcas de tiempo de los ultimos mensajes, para el ritmo en Hz
                "marcas": deque(maxlen=30),
                # --- imagen en directo de /camaras ---
                # Solo el ULTIMO JPEG recibido, no una cola: si el navegador va
                # mas lento que la camara lo que quiere ver es lo de ahora, no
                # el atasco de hace un segundo. 'seq' es lo que le dice a cada
                # navegador si esto que hay guardado ya se lo mando o no
                "jpeg": None,
                "seq": 0,
                "t_imagen": 0.0,
                # Cuantos navegadores estan mirando esta camara ahora mismo. Lo
                # suben y bajan los hilos HTTP; ajustar_subs_imagen lo lee para
                # decidir si hace falta estar suscrito a sus imagenes
                "visores": 0,
            }
        self.evento("camara", f"📷 {camara} conectada")

    def ajustar_subs_imagen(self):
        """Da de alta y de baja las suscripciones a las imagenes de las camaras
        segun quien las este mirando en /camaras.

        Corre en el hilo de ROS (es un timer), que es el mismo sitio desde el
        que descubrir() crea las suscripciones de siempre: los hilos del
        servidor HTTP se limitan a subir y bajar el contador 'visores' y no
        tocan el grafo. Medio segundo de periodo para que al abrir la pagina la
        imagen aparezca casi al momento; descubrir() va a 2 s porque una camara
        que se enciende tarde no tiene ninguna prisa."""
        with self.lock:
            miradas = {c for c, d in self.camaras.items() if d["visores"] > 0}
            camaras = list(self.camaras.items())

        for camara, datos in camaras:
            topic = f"/{camara}/camara_debug"
            if camara in miradas:
                # crear_sub no hace nada si ya existe, asi que esto se puede
                # llamar cada medio segundo sin comprobar nada mas
                self.crear_sub(
                    topic,
                    CompressedImage,
                    lambda m, c=camara: self.cb_imagen(m, c),
                    # El publisher es BEST_EFFORT (qos_profile_sensor_data): con
                    # el QoS fiable por defecto no llegaria ni una sola imagen,
                    # y encima sin ningun error, simplemente en silencio
                    qos_profile_sensor_data,
                )
            elif topic in self.subs:
                self.destroy_subscription(self.subs.pop(topic))
                # Se tira tambien la ultima imagen: si dentro de un rato alguien
                # vuelve a mirar, mejor un hueco que un fotograma de hace horas
                with self.lock:
                    datos["jpeg"] = None
                    datos["t_imagen"] = 0.0

    # -----------------------------------------------------------------
    # Callbacks de ROS
    # -----------------------------------------------------------------
    def cb_imagen(self, msg: CompressedImage, camara):
        # msg.data ya es un JPEG (lo comprime la camara en _tarea_debug con
        # calidad 70): se guarda tal cual, sin decodificar, porque lo unico que
        # se va a hacer con el es escribirlo en un socket
        with self.lock:
            datos = self.camaras.get(camara)
            if datos is None:
                return
            datos["jpeg"] = bytes(msg.data)
            datos["seq"] += 1
            datos["t_imagen"] = time.monotonic()

    def cb_posicion(self, msg: CarLocation, coche):
        camara = msg.camara_id
        if not camara:
            return
        self.alta_camara(camara)

        fx, fy = float(msg.front.center.x), float(msg.front.center.y)
        bx, by = float(msg.back.center.x), float(msg.back.center.y)

        ahora = time.monotonic()
        cambio_camara = None

        with self.lock:
            cam = self.camaras[camara]
            cam["marcas"].append(ahora)

            c = self.coches.get(coche)
            if c is None:
                return

            # (0,0) es como la camara dice "no la he encontrado" (ver
            # publish_car_position): no es una posicion, es la ausencia de una
            if fx == 0 and fy == 0:
                return

            if c["camara"] != camara:
                cambio_camara = (c["camara"], camara)
                c["camara"] = camara

            c["front"] = [fx, fy]
            c["back"] = None if (bx == 0 and by == 0) else [bx, by]
            c["t_posicion"] = ahora
            c["sin_senal"] = False

            self.acumular_trazado(cam, coche, fx, fy)

        if cambio_camara and cambio_camara[0] is not None:
            self.evento(
                "camara",
                f"👁️ {coche}: {cambio_camara[0]} → {cambio_camara[1]}",
                coche,
            )

    def acumular_trazado(self, cam, coche, x, y):
        """Va dibujando el circuito con las posiciones por las que pasa el coche.

        El punto entra solo si esta lejos de TODOS los que ya hay, no solo del
        ultimo: con la regla del ultimo (que es la del controlador, porque el
        recorre una unica vuelta) el trazado seguiria creciendo vuelta tras
        vuelta pintando encima de si mismo. Asi converge: cuando el coche
        completa la primera vuelta el circuito esta dibujado y ya no entra
        nada mas.

        Y el punto se INTERCALA junto a su vecino mas cercano en vez de
        añadirse al final. Añadir al final funciona mientras el coche va
        dibujando su primera vuelta, pero los pocos puntos que entran despues
        (el coche pasa un poco por fuera y descubre un hueco) quedarian al
        final de la lista, y como el mapa une los puntos en orden, cada uno de
        esos puntos tardios pintaba una raya de un lado a otro del circuito.
        """
        p = np.array([[x, y]], dtype=np.float32)
        trazo = cam["trazos"].setdefault(
            coche, {"puntos": [], "np": np.zeros((0, 2), dtype=np.float32)}
        )
        pts = trazo["puntos"]
        if not pts:
            pts.append([x, y])
            trazo["np"] = p
            return

        d = np.linalg.norm(trazo["np"] - p, axis=1)
        j = int(np.argmin(d))
        if d[j] < self.paso_trazado:
            return  # ese trozo de circuito ya esta dibujado

        # Donde va: pegado a su vecino mas cercano, del lado que corresponda.
        # En los extremos la referencia es la longitud del tramo: si el punto
        # nuevo esta mas cerca del penultimo de lo que lo esta el ultimo,
        # entonces cae ENTRE los dos; si no, el trazado crece por ahi
        n = len(pts)
        if n == 1:
            k = 1
        elif j == n - 1:
            k = j if d[j - 1] < np.linalg.norm(
                trazo["np"][j] - trazo["np"][j - 1]
            ) else n
        elif j == 0:
            k = 1 if d[1] < np.linalg.norm(trazo["np"][0] - trazo["np"][1]) else 0
        else:
            k = j if d[j - 1] < d[j + 1] else j + 1

        pts.insert(k, [x, y])
        trazo["np"] = np.insert(trazo["np"], k, p, axis=0)

    def cb_telemetria(self, msg: CarControlTelemetry, coche):
        derrape_nuevo = False
        with self.lock:
            c = self.coches.get(coche)
            if c is None:
                return
            derrape_nuevo = msg.estado_derrapando and not c["derrapando"]
            c["derrapando"] = bool(msg.estado_derrapando)
            c["dist_derrape"] = float(msg.dist_derrape)
            c["pipeline_ms"] = float(msg.pipeline_time) * 1000.0
            camara = c["camara"]

        # Solo el flanco: mientras el derrape dura llegan 30 mensajes por
        # segundo diciendo lo mismo y el historico seria ilegible
        if derrape_nuevo:
            self.evento(
                "derrape",
                f"⚠️ {coche} derrapa en {camara or 'desconocida'} "
                f"({msg.dist_derrape:.0f} px)",
                coche,
            )

    def cb_vuelta(self, msg: TimePerLap, coche):
        texto = None
        with self.lock:
            c = self.coches.get(coche)
            if c is None:
                return
            t = float(msg.lap_time)
            c["tiempo_anterior"] = c["ultimo_tiempo"]
            c["ultimo_tiempo"] = t
            c["vuelta"] = int(msg.lap_number)
            c["t_cruce"] = time.monotonic()

            mejor_personal = c["mejor_tiempo"] is None or t < c["mejor_tiempo"]
            if mejor_personal:
                c["mejor_tiempo"] = t
                c["mejor_vuelta"] = int(msg.lap_number)

            mejor_sesion = self.mejor_absoluto is None or t < self.mejor_absoluto
            if mejor_sesion:
                self.mejor_absoluto = t

            texto = f"🏁 {coche} vuelta {msg.lap_number} · {t:.3f} s"
            if c["tiempo_anterior"] is not None:
                delta = t - c["tiempo_anterior"]
                texto += f"  {'▼' if delta < 0 else '▲'} {delta:+.3f}"
            if mejor_sesion:
                texto += "  🏆 mejor de la sesión"
            elif mejor_personal:
                texto += "  ⭐ mejor personal"

        self.evento("vuelta", texto, coche)

    def cb_pwm(self, msg: SpeedCarril, coche):
        with self.lock:
            c = self.coches.get(coche)
            if c is None:
                return
            c["pwm"] = int(msg.pwm)
            c["carril"] = msg.carril

    def cb_meta(self, msg: FinishLine):
        self.alta_camara(msg.camara_id)
        with self.lock:
            self.camaras[msg.camara_id]["meta"] = [
                [float(msg.finish_line.start.x), float(msg.finish_line.start.y)],
                [float(msg.finish_line.end.x), float(msg.finish_line.end.y)],
            ]

    def cb_calibracion(self, msg: Bool, coche):
        """coche = None cuando llega por el topic global de los bags antiguos."""
        afectados = []
        with self.lock:
            objetivo = self.coches.values() if coche is None else (
                [self.coches[coche]] if coche in self.coches else []
            )
            for c in objetivo:
                if c["calibrando"] != msg.data:
                    c["calibrando"] = bool(msg.data)
                    afectados.append(c["nombre"])

        for nombre in afectados:
            if msg.data:
                self.evento("calibracion", f"📐 {nombre}: calibrando…", nombre)
            else:
                self.evento(
                    "calibracion", f"✅ {nombre}: calibración terminada", nombre
                )

    def vigilar_senal(self):
        """Avisa de los coches que dejan de verse (una Raspberry caida, el coche
        fuera de la pista o simplemente el final de la reproduccion de un bag)."""
        perdidos = []
        ahora = time.monotonic()
        with self.lock:
            for c in self.coches.values():
                if c["t_posicion"] is None or c["sin_senal"]:
                    continue
                if ahora - c["t_posicion"] > TIMEOUT_SIN_SENAL:
                    c["sin_senal"] = True
                    perdidos.append(c["nombre"])
        for nombre in perdidos:
            self.evento("senal", f"📡 {nombre}: sin señal", nombre)

    # -----------------------------------------------------------------
    # Eventos: el "race control" del panel
    # -----------------------------------------------------------------
    def evento(self, tipo, texto, coche=None):
        # El reloj es RELATIVO al arranque del panel, no la hora del reloj: asi
        # la pagina se lee igual en carrera que reproduciendo un bag grabado
        # hace meses, donde la hora absoluta de los mensajes no dice nada
        t = time.monotonic() - self.t_arranque
        with self.lock:
            self.n_evento += 1
            self.eventos.append(
                {
                    "n": self.n_evento,
                    "reloj": f"{int(t // 60):02d}:{int(t % 60):02d}",
                    "tipo": tipo,
                    "coche": coche,
                    "texto": texto,
                }
            )

    # -----------------------------------------------------------------
    # Instantanea para el navegador
    # -----------------------------------------------------------------
    def instantanea(self, enviado):
        """Estado actual en JSON.

        `enviado` es la memoria de UNA conexion: cuantos puntos de trazado y
        que evento fue el ultimo que se le mando. Asi cada envio lleva solo lo
        que le falta a ese navegador, y el trazado (que despues de la primera
        vuelta ya no cambia) no viaja diez veces por segundo.
        """
        ahora = time.monotonic()
        with self.lock:
            coches = []
            for c in self.coches.values():
                sin_senal = (
                    c["t_posicion"] is None or ahora - c["t_posicion"] > TIMEOUT_SIN_SENAL
                )
                delta = None
                if c["ultimo_tiempo"] is not None and c["tiempo_anterior"] is not None:
                    delta = c["ultimo_tiempo"] - c["tiempo_anterior"]
                coches.append(
                    {
                        "nombre": c["nombre"],
                        "color": c["color"],
                        "carril": c["carril"],
                        "vuelta": c["vuelta"],
                        "ultimo_tiempo": c["ultimo_tiempo"],
                        "delta": delta,
                        "mejor_tiempo": c["mejor_tiempo"],
                        "mejor_vuelta": c["mejor_vuelta"],
                        # Para pintar de morado la mejor vuelta de la sesion, como
                        # en una retransmision: el verde queda para el mejor
                        # personal de cada coche
                        "es_mejor_sesion": (
                            c["mejor_tiempo"] is not None
                            and c["mejor_tiempo"] == self.mejor_absoluto
                        ),
                        "pwm": c["pwm"],
                        "derrapando": c["derrapando"],
                        "dist_derrape": round(c["dist_derrape"], 1),
                        "camara": c["camara"],
                        "front": c["front"],
                        "back": c["back"],
                        "pipeline_ms": (
                            None if c["pipeline_ms"] is None
                            else round(c["pipeline_ms"], 1)
                        ),
                        "calibrando": c["calibrando"],
                        "sin_senal": sin_senal,
                        # Desempate de la clasificacion: a igualdad de vueltas
                        # va delante quien cruzo meta antes
                        "t_cruce": c["t_cruce"],
                    }
                )

            camaras = []
            for cam in self.camaras.values():
                # Cada trazado va entero, pero SOLO cuando ha cambiado. Mandar
                # solo los puntos nuevos no vale porque no se añaden al final:
                # se intercalan (ver acumular_trazado), y el navegador acabaria
                # con la lista desordenada. Como el trazado deja de crecer en
                # cuanto el coche completa una vuelta, esto son un par de
                # kilobytes durante la primera vuelta y nada el resto de la
                # carrera
                trazos = {}
                for nombre, trazo in cam["trazos"].items():
                    clave = (cam["id"], nombre)
                    if enviado["puntos"].get(clave) != len(trazo["puntos"]):
                        trazos[nombre] = trazo["puntos"]
                        enviado["puntos"][clave] = len(trazo["puntos"])

                camaras.append(
                    {
                        "id": cam["id"],
                        "hz": self.ritmo(cam["marcas"]),
                        # Segundos desde el ultimo mensaje de esta camara. Es
                        # lo que distingue "no ve al coche ahora mismo" de
                        # "esta caida", que en Hz se ven exactamente igual
                        "desde": (
                            None if not cam["marcas"]
                            else round(ahora - cam["marcas"][-1], 1)
                        ),
                        "meta": cam["meta"],
                        "trazos": trazos,
                        "total_puntos": sum(
                            len(t["puntos"]) for t in cam["trazos"].values()
                        ),
                    }
                )

            eventos = [e for e in self.eventos if e["n"] > enviado["evento"]]
            if eventos:
                enviado["evento"] = eventos[-1]["n"]

            t = ahora - self.t_arranque
            return {
                "reloj": f"{int(t // 60):02d}:{int(t % 60):02d}",
                "v_min": self.v_min,
                "v_max": self.v_max,
                # A partir de este salto entre dos puntos seguidos, el mapa
                # levanta el lapiz en vez de unirlos. Un salto asi no es pista:
                # es el trozo que esta camara NO ve (el coche se sale del
                # encuadre por un lado y vuelve a entrar por otro), y unirlo
                # dibujaria una recta que no existe. Es la misma idea que
                # _linea_con_cortes en analisis/figuras.py
                "salto_corte": self.paso_trazado * 4,
                "coches": coches,
                "camaras": camaras,
                "eventos": eventos,
            }

    @staticmethod
    def ritmo(marcas):
        """Hz a los que trabaja una camara, medidos sobre sus ultimos mensajes.

        No se pone a cero cuando la camara lleva un rato callada, y eso es
        deliberado: una camara solo publica mientras VE al coche, asi que en un
        circuito repartido entre dos camaras cada una se queda muda la mitad de
        la vuelta. Un cero ahi diria "camara caida" cuando lo unico que pasa es
        que el coche esta en la otra punta del trazado. Lo que responde este
        numero es "¿a que ritmo procesa esta Raspberry cuando le toca?", y para
        saber si sigue viva esta el campo 'desde'.

        Por eso tampoco vale dividir mensajes entre tiempo total: los huecos en
        los que el coche esta en otra camara se comerian el resultado (medido en
        pista: 10 Hz en una camara que en realidad va a 30). Se mide la mediana
        de las separaciones DENTRO de la rafaga, descartando los saltos de mas
        de medio segundo, que son justamente los huecos entre pasadas.
        """
        if len(marcas) < 2:
            return 0.0
        seguidos = sorted(
            b - a for a, b in zip(marcas, list(marcas)[1:]) if b - a < 0.5
        )
        if not seguidos:
            return 0.0
        return round(1.0 / seguidos[len(seguidos) // 2], 1)

    # -----------------------------------------------------------------
    # Servidor web
    # -----------------------------------------------------------------
    def servir(self):
        nodo = self

        class Manejador(BaseHTTPRequestHandler):
            def do_GET(self):
                try:
                    # Sin la parte de "?...": camaras.js le añade un ?t=<ahora>
                    # al reabrir un video para que el navegador no le sirva de
                    # su cache la conexion anterior, ya cerrada
                    ruta = self.path.split("?", 1)[0]
                    video = RE_RUTA_VIDEO.match(ruta)

                    if ruta in FICHEROS_WEB:
                        self.fichero(ruta)
                    elif ruta == "/estado":
                        self.estado()
                    elif ruta == "/stream":
                        self.stream()
                    elif ruta == "/estado_camaras":
                        self.estado_camaras()
                    elif video:
                        self.video(video.group(1))
                    else:
                        self.send_error(404, f"ruta desconocida: {self.path}")
                except (BrokenPipeError, ConnectionResetError):
                    pass  # el navegador cerro la pestaña: no es un error
                except Exception as e:  # noqa: BLE001 - el panel nunca tumba al nodo
                    nodo.get_logger().warn(f"panel: {type(e).__name__}: {e}")

            def fichero(self, ruta):
                nombre, tipo = FICHEROS_WEB[ruta]
                # Se lee del disco en CADA peticion, no una vez al arrancar.
                # Como los docker-compose compilan con --symlink-install, lo
                # que hay en el share es un enlace al fichero del repositorio:
                # tocar el CSS o el JS y recargar el navegador basta, sin
                # recompilar ni reiniciar el nodo
                with open(os.path.join(DIR_WEB, nombre), "rb") as f:
                    cuerpo = f.read()
                self.send_response(200)
                self.send_header("Content-Type", tipo)
                self.send_header("Content-Length", str(len(cuerpo)))
                # Sin esto el navegador se guardaria el CSS y el JS en su cache
                # y al editarlos seguiria enseñando los viejos, que es justo lo
                # que la lectura en caliente de arriba viene a evitar
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(cuerpo)

            def estado(self):
                # La misma instantanea de una sola vez, para mirarla con curl
                # sin abrir un navegador. Memoria en blanco: sale todo
                datos = nodo.instantanea({"puntos": {}, "evento": 0})
                cuerpo = json.dumps(datos).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(cuerpo)))
                self.end_headers()
                self.wfile.write(cuerpo)

            def stream(self):
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()

                # Memoria de ESTA conexion. Empieza en blanco, y por eso el
                # primer envio lleva "reinicio": el navegador tira lo que
                # tuviera pintado y vuelve a construirlo. Es lo que hace que
                # recargar la pagina (o que EventSource reconecte solo tras un
                # corte) deje el panel coherente sin que el nodo se entere
                enviado = {"puntos": {}, "evento": 0}
                primero = True
                while True:
                    datos = nodo.instantanea(enviado)
                    datos["reinicio"] = primero
                    primero = False
                    self.wfile.write(
                        f"data: {json.dumps(datos)}\n\n".encode("utf-8")
                    )
                    self.wfile.flush()
                    time.sleep(PERIODO_ENVIO)

            def estado_camaras(self):
                """Lista de camaras para el mosaico: quien hay y quien esta
                mandando imagen ahora mismo.

                Endpoint aparte y no /estado a proposito: /estado devuelve la
                instantanea entera de la carrera (trazados incluidos, que son
                miles de puntos) y esta pagina la sondea cada dos segundos solo
                para saber los nombres."""
                ahora = time.monotonic()
                with nodo.lock:
                    datos = [
                        {
                            "id": camara,
                            # Que la camara este publicando imagenes AHORA. En
                            # falso o porque tiene 'camera.debug: False', o
                            # porque se acaba de pedir y aun no ha llegado la
                            # primera
                            "imagen": ahora - d["t_imagen"] < 2.0,
                            "visores": d["visores"],
                        }
                        for camara, d in sorted(nodo.camaras.items())
                    ]
                cuerpo = json.dumps(datos).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(cuerpo)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(cuerpo)

            def video(self, camara):
                """Las imagenes de una camara, en MJPEG.

                multipart/x-mixed-replace es el formato de siempre de las
                camaras IP: una respuesta que no termina nunca y va soltando un
                JPEG detras de otro separados por un delimitador. El navegador
                lo entiende de serie en un <img>, asi que la pagina no necesita
                ni una linea de JavaScript para pintar el video."""
                with nodo.lock:
                    if camara not in nodo.camaras:
                        self.send_error(404, f"camara desconocida: {camara}")
                        return
                    # A partir de aqui esta camara cuenta como mirada, y en el
                    # medio segundo siguiente ajustar_subs_imagen la suscribe
                    nodo.camaras[camara]["visores"] += 1

                try:
                    self.send_response(200)
                    self.send_header(
                        "Content-Type",
                        "multipart/x-mixed-replace; boundary=frame",
                    )
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()

                    # Memoria de ESTA conexion, igual que en stream(): el numero
                    # de la ultima imagen que se le mando a este navegador
                    ultimo = -1
                    t_envio = 0.0
                    t_espera = time.monotonic()

                    while True:
                        ahora = time.monotonic()
                        with nodo.lock:
                            datos = nodo.camaras.get(camara)
                            jpeg = datos["jpeg"] if datos else None
                            seq = datos["seq"] if datos else -1

                        if jpeg is None:
                            # La camara existe pero no publica imagenes. Se
                            # espera un rato por si esta arrancando y, si no
                            # llega nada, se cierra: el hilo no puede quedarse
                            # aqui para siempre. La pagina lo reintenta sola
                            if ahora - t_espera > SIN_IMAGEN:
                                return
                            time.sleep(PERIODO_IMAGEN)
                            continue
                        t_espera = ahora

                        # Se reenvia la ultima imagen aunque no haya cambiado si
                        # hace mas de un segundo del ultimo envio. No es para el
                        # ojo: es la forma de ENTERARSE de que el navegador se
                        # fue, porque lo que descubre la pestaña cerrada es que
                        # este write falle. Sin esto, una camara congelada
                        # dejaria el hilo y su suscripcion vivos para siempre
                        if seq == ultimo and ahora - t_envio < 1.0:
                            time.sleep(PERIODO_IMAGEN)
                            continue
                        ultimo = seq
                        t_envio = ahora

                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii")
                        )
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                        time.sleep(PERIODO_IMAGEN)
                finally:
                    # Pase lo que pase -y lo que pasa casi siempre es un
                    # BrokenPipeError al cerrar la pestaña- esta camara deja de
                    # estar mirada. Si era el ultimo, ajustar_subs_imagen
                    # destruye la suscripcion y su trafico desaparece de la red
                    with nodo.lock:
                        if camara in nodo.camaras:
                            nodo.camaras[camara]["visores"] -= 1

            def log_message(self, formato, *args):
                pass  # sin ruido de peticiones por consola

        servidor = ThreadingHTTPServer(("0.0.0.0", PUERTO), Manejador)
        servidor.daemon_threads = True
        servidor.serve_forever()


def main(args=None):
    rclpy.init(args=args)
    nodo = VisualizacionNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
