#!/usr/bin/python3
"""
Panel de la carrera en directo

Se suscribe a los topics necesarios para seguir la carrera y sirve la
informacion por HTTP a un navegador

Sirve dos paginas en el puerto 8990:
    El panel en / (clasificacion, plano del circuito y eventos)
    Un mosaico de camaras en /camaras
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

# CMakeLists los instala en el share del paquete
DIR_WEB = os.path.join(get_package_share_directory("image_processor_pkg"), "web")

# Lista blanca de lo que el servidor esta dispuesto a servir, con su tipo. Es
# una lista y no una traduccion de URL a ruta a proposito: asi no existe la
# posibilidad de pedirle un fichero que no sea uno de estos
FICHEROS_WEB = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/index.html": ("index.html", "text/html; charset=utf-8"),
    "/estilos.css": ("estilos.css", "text/css; charset=utf-8"),
    "/panel.js": ("panel.js", "application/javascript; charset=utf-8"),
    # Segunda pagina: el mosaico de lo que ve cada camara
    # Comparte estilos.css con el panel
    "/camaras": ("camaras.html", "text/html; charset=utf-8"),
    "/camaras.html": ("camaras.html", "text/html; charset=utf-8"),
    "/camaras.js": ("camaras.js", "application/javascript; charset=utf-8"),
}

# Puerto donde se sirve el panel
PUERTO = 8990
# Cada cuanto se manda un dato al navegador
PERIODO_ENVIO = 0.1
# Cada cuanto se manda una imagen al navegador
PERIODO_IMAGEN = 0.1
# Periodo de espera para dar de baja una camara
SIN_IMAGEN = 10.0
# Tiempo sin recibir CarLocation.msg para dar por perdido un coche
TIMEOUT_SIN_SENAL = 2.0

# Hay que usar el mismo QoS que el topic para recibir la linea de meta
QOS_FINISH_LINE = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
)

# Color de cada coche en el mapa y en la tabla, por orden de aparicion
# No son los colores de sus pegatinas
COLORES_COCHES = ["#e6194b", "#4363d8", "#3cb44b", "#f58231", "#911eb4"]

# Regex para buscar topics
RE_COCHE = re.compile(r"^/(car[^/]+)/position$")
RE_CAMARA = re.compile(r"^/(camara_[^/]+)/camara_debug$")
RE_RUTA_VIDEO = re.compile(r"^/video/(camara_[^/]+)$")


class VisualizacionNode(Node):
    """
    Nodo de ROS2 que guarda el estado de la carrera y lo sirve por HTTP.
    """

    def __init__(self):
        super().__init__("visualizacion")

        # Rango de PWM del perfil, es la escala de la barra
        self.declare_parameter("controller.minimum_speed", 65)
        self.v_min = float(self.get_parameter("controller.minimum_speed").value)

        self.declare_parameter("controller.maximum_speed", 80)
        self.v_max = float(self.get_parameter("controller.maximum_speed").value)

        self.declare_parameter("controller.distancia_nodos_trayectoria", 15.0)
        self.paso_trazado = float(
            self.get_parameter("controller.distancia_nodos_trayectoria").value
        )

        self.lock = threading.Lock()

        self.coches = {}  
        self.camaras = {} 
        self.subs = {}

        # Los eventos se numeran para poder mandar solo los que falten
        # El deque es la memoria del panel, solo se ven los ultimos 200 mensajes
        self.eventos = deque(maxlen=200)
        self.n_evento = 0
            
        # Mejor vuelta de la sesion de cualquier coche
        self.mejor_absoluto = None

        self.t_arranque = time.monotonic()

        self.crear_sub(
            "/modo_calibracion", Bool, lambda m: self.cb_calibracion(m, None), 10
        )

        self.crear_sub(
            "/finish_line_position", FinishLine, self.cb_meta, QOS_FINISH_LINE
        )

        # Se mira el grafo de topics de ROS cada 2 s porque puede cambiar
        self.create_timer(2.0, self.descubrir)
        self.create_timer(0.5, self.vigilar_senal)
        self.create_timer(0.5, self.ajustar_subs_imagen)

        # Arrancamos el hilo que ejecuta el servidor web
        hilo = threading.Thread(target=self.servir, daemon=True)
        hilo.start()

        self.get_logger().info(
            f"[PANEL] Panel de carrera en http://localhost:{PUERTO}  "
            f"(esperando a que publique alguien)"
        )
        self.get_logger().info(
            f"[PANEL] Cámaras en directo en http://localhost:{PUERTO}/camaras"
        )


    def crear_sub(self, topic, tipo, callback, qos):
        """Crea la suscripcion si no la teniamos ya"""
        if topic in self.subs:
            return
        self.subs[topic] = self.create_subscription(tipo, topic, callback, qos)

    def descubrir(self):
        """
        Mira que topics hay en la red y da de alta los coches y las camaras que
        encuentra
        """
        for topic, _tipos in self.get_topic_names_and_types():
            m = RE_COCHE.match(topic)
            if m:
                self.alta_coche(m.group(1))
                continue
            m = RE_CAMARA.match(topic)
            if m:
                # Aqui solo se coge el nombre de la camara
                # La suscripcion la crea ajustar_subs_imagen, y solo si alguien mira
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

        # En modo manual nadie publica en este topic
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
                "trazos": {},  # coche -> {"puntos": [[x, y], ...], "np": (N,2)}
                "meta": None,
                # Marcas de tiempo de los ultimos mensajes, para el ritmo en Hz
                "marcas": deque(maxlen=30),
                # --- imagen en directo de /camaras ---
                "jpeg": None,
                "seq": 0,
                "t_imagen": 0.0,
                # Cuantos navegadores estan mirando esta camara ahora mismo
                # Lo suben y bajan los hilos HTTP, y ajustar_subs_imagen lo lee
                # para decidir si hace falta estar suscrito a sus imagenes
                "visores": 0,
            }

        self.evento("camara", f"📷 {camara} conectada")

    def ajustar_subs_imagen(self):
        """
        Da de alta y de baja las suscripciones a las imagenes segun quien las
        este mirando en /camaras
        Los hilos HTTP solo suben y bajan el contador de visores
        """
        with self.lock:
            miradas = {c for c, d in self.camaras.items() if d["visores"] > 0}
            camaras = list(self.camaras.items())

        for camara, datos in camaras:
            topic = f"/{camara}/camara_debug"
            if camara in miradas:
                # crear_sub no hace nada si la suscripcion ya existe
                self.crear_sub(
                    topic,
                    CompressedImage,
                    lambda m, c=camara: self.cb_imagen(m, c),
                    qos_profile_sensor_data,
                )
            elif topic in self.subs:
                self.destroy_subscription(self.subs.pop(topic))
                with self.lock:
                    datos["jpeg"] = None
                    datos["t_imagen"] = 0.0

    # -----------------------------------------------------------------
    # Callbacks de ROS
    # -----------------------------------------------------------------
    def cb_imagen(self, msg: CompressedImage, camara):
        # La imagen se guarda sin decodificar, ya viene en JPEG
        with self.lock:
            datos = self.camaras.get(camara)
            if datos is None:
                return
            datos["jpeg"] = bytes(msg.data)
            datos["seq"] += 1
            datos["t_imagen"] = time.monotonic()

    def cb_posicion(self, msg: CarLocation, coche):
        """
        Posicion del coche vista por una camara. Actualiza donde esta el coche
        en el plano y de paso va dibujando el trazado del circuito
        """
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
        """
        Va dibujando sobre el circuito las posiciones por las que pasa el coche
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
            return  # Ese trozo de circuito ya esta dibujado

        # Cada punto se coloca pegado a su vecino mas cercano
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
        """Se procesa la telemetria y se generan avisos"""
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
        """
        Se guarda el tiempo de la vuelta, se actualiza la clasificacion y se
        escribe el evento, con el delta contra la vuelta anterior y la marca de
        mejor personal o mejor de la sesion
        """

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
        """Ultimo PWM aplicado, que es lo que pinta la barra de velocidad"""
        with self.lock:
            c = self.coches.get(coche)
            if c is None:
                return
            c["pwm"] = int(msg.pwm)
            c["carril"] = msg.carril

    def cb_meta(self, msg: FinishLine):
        """Se guarda la posicion de la linea de meta"""
        self.alta_camara(msg.camara_id)
        with self.lock:
            self.camaras[msg.camara_id]["meta"] = [
                [float(msg.finish_line.start.x), float(msg.finish_line.start.y)],
                [float(msg.finish_line.end.x), float(msg.finish_line.end.y)],
            ]

    def cb_calibracion(self, msg: Bool, coche):
        """Se controla el modo de calibracion de un coche"""
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
        """Avisa cuando un coche deja de verse"""

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

    def evento(self, tipo, texto, coche=None):
        """Guarda los eventos que luego se muestran dentro del panel
        Por ejemplo, cuando se completa una vuelta"""

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

    def instantanea(self, enviado):
        """Crea un dict con la informacion que dejaron los callbacks de ROS

        Lo llama el servidor, que lo pasa a JSON y se lo manda al navegador para
        que actualice la info
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
                # Cada trazado va entero, pero solo cuando cambia
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
                        # Segundos desde el ultimo mensaje de esta camara
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
                "salto_corte": self.paso_trazado * 4,
                "coches": coches,
                "camaras": camaras,
                "eventos": eventos,
            }

    @staticmethod
    def ritmo(marcas):
        """
        Calcula los Hz a los que procesa una camara, para ponerlo en el panel
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
                    pass  # Se cerro la pestaña del navegador
                except Exception as e:  
                    nodo.get_logger().warn(f"panel: {type(e).__name__}: {e}")

            def fichero(self, ruta):
                nombre, tipo = FICHEROS_WEB[ruta]

                with open(os.path.join(DIR_WEB, nombre), "rb") as f:
                    cuerpo = f.read()

                self.send_response(200)
                self.send_header("Content-Type", tipo)
                self.send_header("Content-Length", str(len(cuerpo)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()

                self.wfile.write(cuerpo)

            def estado(self):
                """
                Devuelve en JSON el dict que genera la funcion instantanea
                Permite ver el contenido completo sin pasar por el panel, que no
                usa esta ruta
                """
                datos = nodo.instantanea({"puntos": {}, "evento": 0})
                cuerpo = json.dumps(datos).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(cuerpo)))
                self.end_headers()
                self.wfile.write(cuerpo)

            def stream(self):
                """Manda por SSE, convertido a JSON, el dict que genera la
                funcion instantanea"""

                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()

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
                mandando imagen ahora mismo
                """

                ahora = time.monotonic()
                with nodo.lock:
                    datos = [
                        {
                            "id": camara,
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
                """Envia las imagenes de una camara, en MJPEG"""
                with nodo.lock:

                    if camara not in nodo.camaras:
                        self.send_error(404, f"camara desconocida: {camara}")
                        return

                    nodo.camaras[camara]["visores"] += 1

                try:
                    self.send_response(200)
                    self.send_header(
                        "Content-Type",
                        "multipart/x-mixed-replace; boundary=frame",
                    )
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()

                    # Numero de la ultima imagen que se le mando a este navegador
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
                            # La camara existe pero no publica imagenes
                            if ahora - t_espera > SIN_IMAGEN:
                                return

                            # Esperamos por si vuelve a enviar
                            time.sleep(PERIODO_IMAGEN)
                            continue
                        t_espera = ahora

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
                    with nodo.lock:
                        if camara in nodo.camaras:
                            nodo.camaras[camara]["visores"] -= 1

            def log_message(self, formato, *args):
                pass  

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
