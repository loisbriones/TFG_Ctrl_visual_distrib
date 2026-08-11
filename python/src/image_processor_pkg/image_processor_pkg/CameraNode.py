#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from image_processor_pkg.msg import (
    CarLocation,
    FinishLine,
    BoundingRect,
    NetProbe,
    NetProbeEcho,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

# Perfil para QoS preconfigurado, tiene:
#   History: Keep last,
#   Depth: 5,
#   Reliability: Best effort,
#   Durability: Volatile,
#   Deadline: Default,
#   Lifespan: Default,
#   Liveliness: System default,
#   Liveliness lease duration: default,
#   avoid ros namespace conventions: false
# Informacion sacada de: https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1SensorDataQoS.html
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
# Para reconstruir el stamp de captura (builtin_interfaces/Time) y poder
# restarlo del instante de publicacion en publish_car_position
from rclpy.time import Time

from threading import Thread, Lock

from ProcessImage import ColorDetector
from concurrent.futures import ThreadPoolExecutor, wait
import functools
import time
import json
import os

CAMERA_MODES = {
    0: (160, 120),
    1: (320, 240),
    2: (640, 480),
    3: (800, 600),
    4: (1280, 720),
}

QOS_FINISH_LINE = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
)


class ImageProcessor(Node):
    def __init__(self):
        super().__init__("image_processor")

        # ---- CALL GROUPS ----
        # Posicion
        self.image_processor_group = MutuallyExclusiveCallbackGroup()
        # Debug
        self.debug_group = MutuallyExclusiveCallbackGroup()
        # Eco de los sondeos de red. Grupo PROPIO, con su hilo reservado en el
        # MultiThreadedExecutor de main(), y no por simetria con los de arriba:
        # el eco tiene que salir en cuanto llega el sondeo. Si compartiese
        # grupo con el procesado del fotograma, cada sondeo esperaria a que
        # terminase la deteccion en curso y ese tiempo de cola se sumaria al
        # RTT como si fuese latencia de red. La medida diria entonces cuanto
        # tarda la camara en atender, que es justo lo que NO se quiere medir.
        self.net_probe_group = MutuallyExclusiveCallbackGroup()

        # ---- ID NODO ----
        self.declare_parameter("camara_id", "camara")
        self.camara_id = self.get_parameter("camara_id").value
        # -----------------

        # ----- CAMARA -----
        # Seleccionar tamaño del frame usando diccionario de modos
        self.declare_parameter("camera.mode", 2)
        mode = self.get_parameter("camera.mode").value
        # Coger modo camara para saber w,h del frame
        self.width, self.height = CAMERA_MODES[mode]

    
        # Seleccionamos la camara
        # El ParameterDescriptor permite que el tipo que nos mandan pueda ser decidido en tiempo de ejecución cuando llega y no se comprueba 
        device_param = self.declare_parameter('camera.device','0',ParameterDescriptor(dynamic_typing=True))
        device_param = self.get_parameter('camera.device').value

        # Intentamos primero convertir a numero
        try:
            camera_name = int(device_param)
        except ValueError:
            # Asumimos que si no nos envian el numero nos dan el path /dev/deviceN 
            camera_name = device_param

        self.cam = cv.VideoCapture(camera_name, cv.CAP_V4L2)
        # Esto le pide a la cámara que envíe los datos ya comprimidos en JPEG
        self.cam.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))
        # Configuramos el ancho de la camara
        self.cam.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
        # Configuramos el alto de la camara
        self.cam.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
        # Desactivamos el autoenfoque de la camara
        self.cam.set(cv.CAP_PROP_AUTOFOCUS, 0)
        # ---------------------

        # ---- CALIBRACION ----
        # La calibracion es POR COCHE, no global. Este parametro es solo el
        # valor de ARRANQUE con el que empieza cada coche (todos calibrando);
        # el estado vivo vive en info_coches[coche]["modo_calibracion"] y la
        # senal de fin llega por coche en /<coche>/modo_calibracion (una
        # suscripcion por coche, creada en el bucle de coches de abajo y
        # guardada en self.sub_modo_calibracion[coche]). Antes era un unico
        # /modo_calibracion global: el primer coche en cerrar su vuelta cortaba
        # la calibracion de todos y las mascaras salian a medias.
        self.declare_parameter("modo_calibracion", True)
        self.calib_default = self.get_parameter("modo_calibracion").value
        self.sub_modo_calibracion = {}
        # Tamaño kernel para generar la mascara
        self.declare_parameter("camera.mascara_kernel_size", 25)
        self.mascara_kernel_size = self.get_parameter("camera.mascara_kernel_size").value
        # --------------------

        # ----- DETECTION -----
        self.declare_parameter("camera.detection.min_area", 50)
        self.min_area = self.get_parameter("camera.detection.min_area").value

        self.declare_parameter("camera.detection.kernel_size", 5)
        self.kernel_size = self.get_parameter("camera.detection.kernel_size").value
        # ---------------------

        # --- COLOR DETECTORS POR COCHE ---
        # Los colores de las pegatinas ya NO son globales de la camara: son de
        # cada coche (params.yaml -> cars.<coche>.stiker_front/back). Aqui van
        # los dos diccionarios que se rellenan en el bucle de coches de abajo,
        # uno con los colores y otro con un ColorDetector propio por coche.
        # Tener un detector independiente por coche es lo que permite seguir
        # varios a la vez con colores distintos sin que se pisen entre hilos.
        #   self.car_stikers[coche]     = {"front": color, "back": color}
        #   self.color_detectors[coche] = ColorDetector(front, back, kernel)
        self.car_stikers = {}
        self.color_detectors = {}
        # ---------------------------------

        # ---- DEBUG ----
        self.declare_parameter("camera.debug", False)
        self.debug = self.get_parameter("camera.debug").value
        # (frame copiado, stamp de captura, nº de frame) que le toca dibujar a
        # _tarea_debug
        self.next_debug_frame = None
        self.puntos_for_debug = {}
        # Para controlar el cuando hay datos de debug y cuando no. Es el
        # handshake entre los dos timers, que están en callback groups
        # distintos y por tanto corren en paralelo:
        #   True  -> "hacen falta datos nuevos": process_frame copia el frame
        #            y los hilos de tarea_por_coche rellenan puntos_for_debug
        #   False -> "los datos ya están listos": _tarea_debug los dibuja,
        #            publica la imagen y vuelve a pedir poniéndolo a True
        # Como cada lado solo toca los datos en su mitad del ciclo, no hacen
        # falta locks. Arranca valiendo lo mismo que debug porque eso es
        # justo una petición pendiente: si empezara en False con
        # next_debug_frame a None, _tarea_debug saldría sin llegar a pedir
        # nada y process_frame no capturaría nunca, los dos esperándose para
        # siempre (era lo que hacía que arrancar con debug: True no publicase
        # ni una imagen). Es lo mismo que hace parameters_callback cuando se
        # activa el debug en caliente.
        self.save_data = self.debug

        # ---- CONTADOR DE FRAMES ----
        # Cuenta los fotogramas que PROCESA este nodo (los que consume
        # process_frame, no los que el hilo de captura saca del hardware). Ese
        # número identifica el fotograma en los tres sitios donde puede acabar:
        # quemado en la imagen de debug, dentro del CarLocation publicado y, vía
        # controlador, en las líneas [FRAME n] del log del algoritmo. Con él, una
        # línea de log, un mensaje del bag y una imagen del visor se pueden casar.
        #
        # Es un contador LOCAL a esta cámara y empieza en 0 al arrancar el nodo:
        # los números de dos cámaras NO son comparables entre sí (cada Raspberry
        # arranca cuando arranca). Sincronizar un contador único entre nodos
        # exigiría un topic de reloj lógico, y no hace falta: el stamp de captura
        # que ya viaja en el mensaje es lo que permite cruzar cámaras. La regla,
        # desarrollada en NUMERACION_FRAMES.md, es: dentro de una cámara se
        # compara por número de frame; entre cámaras, por stamp.
        self.n_frame = 0

        # ---- ACTUALIZACION PARAMETROS ----
        self.add_on_set_parameters_callback(self.parameters_callback)

        # ----- PUBLISHER ------

        # - Posicion coche -
        self.declare_parameter("coches", ["car"])
        self.coches = self.get_parameter("coches").value

        # Creamos un pool de hilos para el procesamiento
        self.thread_pool = ThreadPoolExecutor(max_workers=len(self.coches))
        # Flag de control para saber si todos los hilos acabaron de ejecutarse
        self.is_processing = False

        self.publisher_coche = {}
        self.info_coches = {}

        for car_name in self.coches:
            self.get_logger().info(f"CREADO CARRIL PARA {car_name}")

            # Colores de las pegatinas de ESTE coche (params.yaml ->
            # cars.<coche>.stiker_front/back). Se guardan en car_stikers (que
            # usan publish_car_position y tarea_por_coche para etiquetar y
            # buscar) y se construye un ColorDetector propio del coche.
            param_front = f"cars.{car_name}.stiker_front"
            param_back = f"cars.{car_name}.stiker_back"
            self.declare_parameter(param_front, "rojo")
            self.declare_parameter(param_back, "verde")
            s_front = self.get_parameter(param_front).value
            s_back = self.get_parameter(param_back).value

            self.car_stikers[car_name] = {"front": s_front, "back": s_back}
            self.color_detectors[car_name] = ColorDetector(s_front, s_back, self.kernel_size)
            self.get_logger().info(
                f"🚗 {car_name} -> frontal: {s_front} | trasero: {s_back}"
            )

            self.publisher_coche[car_name] = self.create_publisher(
                CarLocation, f"/{car_name}/position", qos_profile_sensor_data
            )

            self.info_coches[car_name] = {
                "prev_cx": None,
                "prev_cy": None,
                "current_cx": None,
                "current_cy": None,
                "roi_size": 150,
                # Puntos usados para construir la mascara
                "puntos_trayectoria": [],
                # Mascara generada
                "mascara_trayectoria": None,
                # Estado de calibracion de ESTE coche (arranca con el default
                # del parametro). Cuando el coche cierra su vuelta pasa a False
                # sin tocar a los demas. La carga de cache de mas abajo lo pone
                # a False para los coches que ya tengan trayectoria guardada.
                "modo_calibracion": self.calib_default,
            }

            # Suscripcion de calibracion POR COCHE: /<coche>/modo_calibracion.
            # functools.partial fija car_name para que el callback sepa de que
            # coche viene el aviso (create_subscription entrega solo el msg).
            self.sub_modo_calibracion[car_name] = self.create_subscription(
                Bool,
                f"/{car_name}/modo_calibracion",
                functools.partial(self.callback_control, car_name),
                10,
            )

            self.puntos_for_debug[car_name] = {
                "debug_x": 0,
                "debug_y": 0,
                "debug_points": [],
            }
        # ---------------------------------------

        # -------------- Debug ------------------
        self.debug_publisher = self.create_publisher(
            CompressedImage, "camara_debug", qos_profile_sensor_data
        )
        # ---------------------------------------

        # ---- ECO DE SONDEOS DE RED ----
        # Responde a los NetProbe que manda el NetProbeNode desde el PC para
        # medir la latencia del enlace. Esta camara no mide nada: solo rebota
        # el sondeo tal cual y apunta cuanto tardo ella en hacerlo, para que el
        # emisor pueda descontarlo del RTT. Toda la explicacion de por que se
        # mide de ida y vuelta y no de ida sola esta en NetProbeNode.py.
        #
        # Topics RELATIVOS: el nodo corre en el namespace de la camara (el
        # node_id que le pasa CameraLaunch), asi que "net_probe" resuelve a
        # /camara_XX/net_probe, igual que "camara_debug" de aqui arriba.
        #
        # enabled: True por defecto — el sondeo es un mensaje pequeno cada 200
        # ms y no estorba. Se puede apagar para comprobar que el sistema se
        # comporta exactamente igual sin la instrumentacion.
        self.declare_parameter("net_probe.enabled", True)
        self.net_probe_enabled = self.get_parameter("net_probe.enabled").value

        if self.net_probe_enabled:
            self.net_probe_publisher = self.create_publisher(
                NetProbeEcho, "net_probe_echo", qos_profile_sensor_data
            )
            self.net_probe_subscription = self.create_subscription(
                NetProbe,
                "net_probe",
                self.callback_net_probe,
                qos_profile_sensor_data,
                callback_group=self.net_probe_group,
            )
            self.get_logger().info("📡 Eco de sondeos de red activo")
        # -------------------------------

        # ---- TIMER ----
        # Posicion
        self.timer = self.create_timer(0.033, self.process_frame, callback_group=self.image_processor_group)
        # Debug
        self.debug_timer = self.create_timer(0.033, self._tarea_debug, callback_group=self.debug_group)

        # --- BUSCAR LINEA DE META ---
        # El nombre del parametro tiene que ser la ruta COMPLETA que tiene en
        # params.yaml (camera -> detection -> finish_line_color): ROS2 aplana el
        # YAML anidado a "camera.detection.finish_line_color". Declararlo con el
        # nombre plano "finish_line_color" no da ningun error, simplemente crea
        # OTRO parametro que el YAML nunca toca: el color se quedaba siempre en
        # el valor por defecto de aqui y cambiarlo en el fichero no hacia nada.
        # Colaba porque los dos decian "naranja". Mismo prefijo que sus vecinos
        # camera.detection.min_area y camera.detection.kernel_size.
        self.declare_parameter("camera.detection.finish_line_color", "naranja")
        self.finish_line_color = self.get_parameter(
            "camera.detection.finish_line_color"
        ).value

        ret, frame_for_find_sectors = self.cam.read()
        # La linea de meta es del circuito, no de ningun coche: vale cualquier
        # detector (find_finish_line usa solo finish_line_color). Se coge el del
        # primer coche, o uno auxiliar si no hubiera coches configurados.
        if self.coches:
            detector_meta = self.color_detectors[self.coches[0]]
        else:
            detector_meta = ColorDetector("", "", self.kernel_size)
        self.finish_line_position = detector_meta.find_finish_line(frame_for_find_sectors, self.finish_line_color)

        if self.finish_line_position is not None and ret:
            self.finish_line_publisher = self.create_publisher(FinishLine, "/finish_line_position", QOS_FINISH_LINE)

            finish_line_msg = FinishLine()
            finish_line_msg.camara_id = self.camara_id

            finish_line_msg.finish_line.start.x = float(self.finish_line_position[0][0])
            finish_line_msg.finish_line.start.y = float(self.finish_line_position[0][1])

            finish_line_msg.finish_line.end.x = float(self.finish_line_position[1][0])
            finish_line_msg.finish_line.end.y = float(self.finish_line_position[1][1])

            self.finish_line_publisher.publish(finish_line_msg)


        # --- CARGAR RUTA DE CACHE ---
        self.cache_file = (f"/ros2_ws/src/image_processor_pkg/cache_trayectoria_{self.camara_id}.json" )

        if os.path.exists(self.cache_file):
            try:
                with open(self.cache_file, "r") as f:
                    datos_cache = json.load(f)

                # Carga POR COCHE: cada coche que ya tenga trayectoria en la
                # cache arranca en operacion (mascara lista, modo_calibracion
                # False); los que no esten en la cache se quedan calibrando con
                # su default. Asi sale gratis el caso "un coche ya cacheado +
                # otro nuevo que aun tiene que dar su vuelta de calibracion".
                for car_name in self.coches:
                    if car_name in datos_cache:
                        self.info_coches[car_name]["puntos_trayectoria"] = datos_cache[car_name]
                        # Generamos la mascara para cada coche con la info almacenada
                        self.info_coches[car_name]["mascara_trayectoria"] = (self.generar_mascara(datos_cache[car_name]))
                        self.info_coches[car_name]["modo_calibracion"] = False
                        self.get_logger().info(
                            f"🟢 {car_name}: caché cargada desde {self.cache_file}. Calibración omitida."
                        )

            except Exception as e:
                self.get_logger().error(
                    f"Error cargando caché: {e}. Se forzará calibración."
                )
                # La cache esta corrupta: todos los coches vuelven a calibrar
                for car_name in self.coches:
                    self.info_coches[car_name]["modo_calibracion"] = self.calib_default

        # --- THREAD CAPTURA ---
        self.latest_frame = None
        self.frame_lock = Lock()
        self.running = True
        # Iniciamos el hilo de captura inmediatamente
        self.capture_thread = Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        self.get_logger().info("Node ImageProcessor Ready")

    def _capture_loop(self):
        """Hilo dedicado a vaciar el hardware. Ritmo dictado por la cámara."""
        while self.running and rclpy.ok():
            ret, frame = self.cam.read()
            if ret:
                # El stamp se toma AQUÍ, justo tras leer el frame del hardware,
                # y viaja junto al frame hasta el CarLocation publicado. Así el
                # stamp representa el instante de CAPTURA (cuando el coche
                # estaba realmente en esa posición), no el de publicación:
                # el tiempo de detección + ROI no contamina la medida y el
                # controlador puede interpolar tiempos de vuelta precisos.
                stamp = self.get_clock().now().to_msg()
                with self.frame_lock:
                    self.latest_frame = (frame, stamp)

    def callback_control(self, car_name, msg):
        # Aviso de calibracion de UN coche concreto (/<car_name>/modo_calibracion).
        # car_name lo fija functools.partial al crear la suscripcion.
        info = self.info_coches[car_name]

        # Modo operacion: este coche acaba de cerrar su vuelta de calibracion
        if msg.data == False and info["modo_calibracion"]:
            # Generamos SOLO la mascara de este coche con su trayectoria
            info["mascara_trayectoria"] = self.generar_mascara(
                info["puntos_trayectoria"]
            )

            # Guardamos su trayectoria en la cache SIN pisar la de los demas:
            # leemos el JSON existente (si lo hay), fijamos la clave de este
            # coche y reescribimos. Cada coche termina su vuelta en un instante
            # distinto, asi que no se puede volcar el dict entero de golpe.
            try:
                datos_cache = {}
                if os.path.exists(self.cache_file):
                    with open(self.cache_file, "r") as f:
                        datos_cache = json.load(f)
                datos_cache[car_name] = info["puntos_trayectoria"]
                with open(self.cache_file, "w") as f:
                    json.dump(datos_cache, f)
                self.get_logger().info(
                    f"💾 {car_name}: trayectoria guardada en {self.cache_file}"
                )
            except Exception as e:
                self.get_logger().error(f"Error guardando caché de {car_name}: {e}")

            info["modo_calibracion"] = False

            self.get_logger().info(f"Calibracion Terminada ({car_name})")

        # Modo calibracion: reinicio de la calibracion de este coche
        elif msg.data == True:
            info["mascara_trayectoria"] = None
            info["modo_calibracion"] = True

            self.get_logger().warn(f"Reiniciando Calibracion ({car_name})")

    def callback_net_probe(self, msg):
        """Devuelve el sondeo tal cual, cronometrando lo que tarda en hacerlo.

        Corre en su propio grupo de callbacks con hilo reservado
        (net_probe_group), asi que no espera a que termine el procesado del
        fotograma en curso.

        proc_ns se mide con time.monotonic_ns() y NO con el reloj del nodo: es
        una DURACION dentro de esta maquina, y el monotonico no da saltos si
        NTP corrige la hora del sistema a mitad de la medida. Como es una
        duracion y no un instante, el emisor puede restarla de su RTT sin que
        el desfase entre los relojes de las dos maquinas entre en la cuenta.
        """
        # Primera linea, sin nada delante: lo que se hiciera antes quedaria
        # fuera de proc_ns y el emisor lo acabaria contando como red
        t_entrada_ns = time.monotonic_ns()

        eco = NetProbeEcho()
        # Todo esto se copia SIN TOCAR. En especial t_send: aqui no se compara
        # con nada del reloj local (seria una resta entre relojes de maquinas
        # distintas, justo lo que se evita), solo se devuelve para que el
        # sondeo quede identificado tambien dentro del bag
        eco.seq = msg.seq
        eco.origen = msg.origen
        eco.destino = msg.destino
        eco.t_send = msg.t_send
        # El mismo relleno de vuelta, para que el eco pese lo que la ida y los
        # dos sentidos del viaje se midan en igualdad de condiciones
        eco.padding = msg.padding

        # Ultimo instante antes de publicar: a partir de aqui ya es red
        eco.proc_ns = time.monotonic_ns() - t_entrada_ns
        self.net_probe_publisher.publish(eco)

    def parameters_callback(self, params):
        result = SetParametersResult(successful=True)

        for param in params:
            if param.name == "camera.detection.min_area":
                if param.value < 0:
                    result.successful = False
                    result.reason = "El área mínima no puede ser negativa"
                else:
                    self.min_area = param.value
                    self.get_logger().info(
                        f"Parámetro actualizado: min_area = {self.min_area}"
                    )

            elif param.name.startswith("cars."):
                # Cambio en caliente de un color de pegatina de un coche:
                # cars.<coche>.stiker_front / cars.<coche>.stiker_back
                parts = param.name.split(".")
                if len(parts) == 3:
                    car_name, stiker_type = parts[1], parts[2]
                    if (
                        car_name in self.car_stikers
                        and stiker_type in ("stiker_front", "stiker_back")
                    ):
                        key = "front" if stiker_type == "stiker_front" else "back"
                        self.car_stikers[car_name][key] = param.value
                        self.actualizar_detector()
                        self.get_logger().info(
                            f"Parámetro actualizado para {car_name}: {stiker_type} = {param.value}"
                        )

            elif param.name == "camera.detection.kernel_size":
                if param.value % 2 == 0:
                    result.successful = False
                    result.reason = "El kernel_size debe ser un número impar"
                else:
                    self.kernel_size = param.value
                    self.actualizar_detector()

            elif param.name == "camera.mascara_kernel_size":
                if param.value % 2 == 0:
                    result.successful = False
                    result.reason = "El kernel_size debe ser un número impar"
                else:
                    # Para que tenga efecto es necesario poner el modo calibracion por seguridad
                    self.mascara_kernel_size = param.value

            elif param.name == "camera.debug":
                self.debug = param.value
                self.save_data = param.value

                self.get_logger().info(f"Parámetro actualizado: debug = {self.debug}")

        return result

    def generar_mascara(self, puntos_trayectoria):
        # Crear lienzo negro
        mascara = np.zeros((self.height, self.width), dtype=np.uint8)

        if len(puntos_trayectoria) < 2:
            return mascara

        puntos = np.array(puntos_trayectoria, dtype=np.int32)

        # Dibujar la trayectoria uniendo los puntos con líneas blancas
        # isClosed=True para cerrar el circuito al final
        cv.polylines(mascara, [puntos], isClosed=True, color=255, thickness=15)

        kernel = np.ones((self.mascara_kernel_size, self.mascara_kernel_size), np.uint8)

        # Cierre morfologico para eliminar pequeños puntos negros que puedan quedar fruto de no detectar nada
        mascara_final = cv.morphologyEx(mascara, cv.MORPH_CLOSE, kernel)
        # Dilatiacion expandimos los bordes de la mascara hacia fuera aumenta el area de la mascara
        mascara_final = cv.dilate(mascara_final, kernel, iterations=1)

        return mascara_final

    def actualizar_detector(self):
        """Re-instancia el ColorDetector de CADA coche con sus colores actuales
        (car_stikers) y el kernel actual. Se llama al cambiar en caliente un
        color de pegatina o el kernel de deteccion."""
        for car_name in self.coches:
            s_front = self.car_stikers[car_name]["front"]
            s_back = self.car_stikers[car_name]["back"]
            self.color_detectors[car_name] = ColorDetector(s_front, s_back, self.kernel_size)

    def publish_car_position(self, detections, proc_duration, x, y, car_name, frame_stamp, n_frame):
        object_location_msg = CarLocation()
        object_bounding_rect_front = BoundingRect() 
        object_bounding_rect_back = BoundingRect()

        object_location_msg.camara_id = self.camara_id
        object_location_msg.coche = car_name

        for key in ["front", "back"]:
            p = detections[key]
            if p is not None:
                if key == "front":
                    object_location_msg.front.center.x = p["cx"] + x
                    object_location_msg.front.center.y = p["cy"] + y
                    object_location_msg.front.color = self.car_stikers[car_name]["front"]
                    # Guardar la posición del rectangulo que detectamos
                    # También hay que ajustarlo porque se sacan las coordenas de dentro del ROI
                    object_bounding_rect_front.x = p["x"] + x 
                    object_bounding_rect_front.y = p["y"] + y
                    object_bounding_rect_front.w = p["w"] + x 
                    object_bounding_rect_front.h = p["h"] + y
                    
                    # Guardamos la posición dentro del /car1/position 
                    object_location_msg.bounding_rect_stiker_front = object_bounding_rect_front

                if key == "back":
                    object_location_msg.back.center.x = p["cx"] + x
                    object_location_msg.back.center.y = p["cy"] + y
                    object_location_msg.back.color = self.car_stikers[car_name]["back"]
                    
                    # Guardar la posición del rectangulo que detectamos
                    # También hay que ajustarlo porque se sacan las coordenas de dentro del ROI
                    object_bounding_rect_back.x = p["x"] + x 
                    object_bounding_rect_back.y = p["y"] + y
                    object_bounding_rect_back.w = p["w"] + x 
                    object_bounding_rect_back.h = p["h"] + y
                    
                    # Guardamos la posición dentro del /car1/position 
                    object_location_msg.bounding_rect_stiker_back = object_bounding_rect_back
        
        object_location_msg.proc_time = proc_duration

        # El stamp es el instante de CAPTURA del frame (tomado en
        # _capture_loop), no el de publicación: es el momento en el que el
        # coche estaba de verdad en esta posición. El controlador interpola
        # con estos stamps el instante exacto del paso por meta; como el
        # tiempo de vuelta es la diferencia entre dos stamps de la MISMA
        # Raspberry, el desfase de reloj entre máquinas se cancela solo.
        object_location_msg.stamp = frame_stamp

        # Nº de frame de ESTA cámara: es lo que el controlador le pasa al
        # algoritmo para que sus líneas [FRAME n] del log apunten al mismo
        # fotograma que lleva el número quemado en la imagen de debug
        object_location_msg.n_frame = n_frame

        # EDAD DEL DATO: lo que ha envejecido esta posición desde que el
        # fotograma salió del hardware hasta que el mensaje sale por la red.
        # Es el tramo "procesado en cámara" del presupuesto de latencia, y es
        # bastante más que proc_time: ahí solo entra la llamada a
        # detector.find_object, y se quedan fuera la espera del fotograma en
        # latest_frame hasta que dispara el timer de 33 Hz, los fotogramas que
        # is_processing descarta, el recorte del ROI con su bitwise_and, el
        # despacho al ThreadPool con la espera a todos los coches y el armado
        # del mensaje. Además en calibración proc_time se publica como 0.0.
        # Comparar el proc_time con el tramo de red daba por eso una cámara
        # artificialmente rápida.
        #
        # Las dos marcas son del reloj de ESTA máquina: el stamp lo toma
        # _capture_loop justo después de cam.read() y este instante se lee con
        # el mismo get_clock(). La resta es intra-reloj, así que vale sin NTP
        # ni sincronización de ningún tipo — la misma regla que siguen el
        # lap_time y el pipeline_time.
        ahora = self.get_clock().now()
        object_location_msg.age_at_publish = (
            ahora - Time.from_msg(frame_stamp)
        ).nanoseconds / 1e9

        self.publisher_coche[car_name].publish(object_location_msg)

    def process_frame(self):
        """Timer de ROS que consume el último frame disponible."""
        if self.is_processing:
            return

        current_frame = None
        frame_stamp = None
        with self.frame_lock:
            if self.latest_frame is not None:
                # latest_frame es la tupla (frame, stamp de captura)
                current_frame, frame_stamp = self.latest_frame
                self.latest_frame = (
                    None  # Consumimos el frame para no repetir procesado
                )

        if current_frame is None:
            return

        # Este fotograma sí se procesa: le toca número. Se cuenta AQUÍ y no en
        # _capture_loop para que la numeración sea consecutiva y un hueco en el
        # log signifique una sola cosa ("esta cámara no publicó ese frame"), en
        # vez de mezclar eso con los frames que el timer de 30 Hz descarta. No
        # necesita lock: solo lo toca este timer, que está en un callback group
        # mutuamente exclusivo y además protegido por is_processing.
        self.n_frame += 1

        self.is_processing = True
        try:
            if self.debug and self.save_data:
                # El frame, su stamp y su número viajan juntos en la misma tupla
                # (igual que latest_frame en _capture_loop): así la imagen de
                # debug no puede acabar publicada con el stamp ni con el número
                # de otro fotograma
                self.next_debug_frame = (current_frame.copy(), frame_stamp, self.n_frame)

            futures = []
            for car_name in self.coches:
                futures.append(
                    self.thread_pool.submit(
                        self.tarea_por_coche,
                        current_frame,
                        frame_stamp,
                        self.n_frame,
                        car_name,
                        self.info_coches[car_name],
                    )
                )

            # Esperamos a que todos los hilos terminen
            wait(futures)

            if self.debug and self.save_data:
                self.save_data = False

        finally:
            # Marcamos que terminamos de procesar
            self.is_processing = False

    """
    Funcion para calcular la posicion del ROI 
    """

    def calcular_roi_bounds(self, info):
        # Recuperamos el estado actual del coche desde su diccionario 'info'
        prev_cx = info["prev_cx"]
        prev_cy = info["prev_cy"]
        curr_cx = info["current_cx"]
        curr_cy = info["current_cy"]
        roi_size = info["roi_size"]

        # Lógica de predicción lineal (Extrapolación)
        if prev_cx is not None and curr_cx is not None:
            pred_x = curr_cx + (curr_cx - prev_cx)
            pred_y = curr_cy + (curr_cy - prev_cy)
        elif curr_cx is not None:
            pred_x, pred_y = curr_cx, curr_cy
        else:
            # Si se perdió el rastro, buscamos en el centro y ampliamos ROI
            pred_x, pred_y = self.width // 2, self.height // 2
            info["roi_size"] = max(self.width, self.height)
            roi_size = info["roi_size"]

        half_roi = roi_size // 2

        # Ajustar a los límites de la imagen
        x1 = int(np.clip(pred_x - half_roi, 0, self.width))
        y1 = int(np.clip(pred_y - half_roi, 0, self.height))
        x2 = int(np.clip(pred_x + half_roi, 0, self.width))
        y2 = int(np.clip(pred_y + half_roi, 0, self.height))

        return x1, y1, x2, y2

    """
    Funcion que ejecutan los threads que se encargan de buscar los Stikers de los coches en el frame
    """
    def tarea_por_coche(self, frame, frame_stamp, n_frame, car_name, info):
        # Detector y colores propios de ESTE coche (cada hilo del pool trabaja
        # con la instancia de su coche, sin compartir estado con los demas)
        detector = self.color_detectors[car_name]
        s_front = self.car_stikers[car_name]["front"]
        s_back = self.car_stikers[car_name]["back"]

        # --- MODO CALIBRACIÓN ---
        # Por coche: este coche puede seguir calibrando mientras otro ya corre.
        if info["modo_calibracion"]:
            detections = detector.find_object(frame, self.min_area, s_front, s_back)
            if detections["front"] is not None:
                info["puntos_trayectoria"].append((detections["front"]["cx"], detections["front"]["cy"]))
                # Usamos 0,0 como offset porque es el frame completo
                # Solo publicamos en calibración si hemos detectado algo.
                # El 0.0 es el proc_time: en calibración no se cronometra la
                # detección, así que se publica a cero. El age_at_publish del
                # mensaje SÍ sale bien también aquí (lo calcula
                # publish_car_position a partir del frame_stamp, sin depender
                # de este parámetro), y por eso es la medida que sirve para el
                # presupuesto de latencia y proc_time no.
                self.publish_car_position(detections, 0.0, 0, 0, car_name, frame_stamp, n_frame)
            return

        # --- MODO OPERACION---
        # 1. Obtener límites según predicción
        x1, y1, x2, y2 = self.calcular_roi_bounds(info)

        # 2. Calculamos el ROI y aplicamos la mascara
        roi_frame = frame[y1:y2, x1:x2]
        if info["mascara_trayectoria"] is not None:
            mask_roi = info["mascara_trayectoria"][y1:y2, x1:x2]
            roi_frame = cv.bitwise_and(roi_frame, roi_frame, mask=mask_roi)

        # 3. Detectar
        start = time.perf_counter()
        detections = detector.find_object(roi_frame, self.min_area, s_front, s_back)
        duration = (time.perf_counter() - start) * 1000

        found_any = detections["front"] is not None and detections["back"] is not None

        if found_any:
            # Preferimos el frontal para el seguimiento
            p_ref = (
                detections["front"]
                if detections["front"] is not None
                else detections["back"]
            )

            # Guardamos coordenadas globales
            global_cx = x1 + p_ref["cx"]
            global_cy = y1 + p_ref["cy"]

            info["prev_cx"], info["prev_cy"] = info["current_cx"], info["current_cy"]
            info["current_cx"], info["current_cy"] = global_cx, global_cy
            info["roi_size"] = 150  # Resetear tamaño de búsqueda

            # 5. Publicar (Solo lo hacemos si hemos encontrado el coche)
            self.publish_car_position(detections, duration, x1, y1, car_name, frame_stamp, n_frame)

            if self.debug and self.save_data:
                # Restauramos la estructura del diccionario si venimos de perder el coche
                if self.puntos_for_debug[car_name] is None:
                    self.puntos_for_debug[car_name] = {}

                # Guardar para el dibujo de debug
                self.puntos_for_debug[car_name]["debug_x"] = x1
                self.puntos_for_debug[car_name]["debug_y"] = y1
                # Guardamos los puntos detectados (front y back) en la lista de debug
                self.puntos_for_debug[car_name]["debug_points"] = [
                    v for v in detections.values() if v is not None
                ]

        else:
            # Si no hay detección, ampliar zona de búsqueda para el próximo frame
            info["roi_size"] = min(info["roi_size"] + 50, max(self.width, self.height))
            info["current_cx"] = None
            info["prev_cx"] = None
            # Evitamos que la información de debug se quede dibujando "fantasmas"
            self.puntos_for_debug[car_name] = None

    def _dibujar_marca(self, frame, stamp, n_frame):
        """
        Quema la identidad del fotograma en la esquina superior izquierda, como
        el reloj de una cámara de vigilancia: qué cámara, qué número de frame y
        la hora de CAPTURA. Las tres cosas viajan también en los mensajes, pero
        ahí se pierden en cuanto la imagen se guarda suelta o se mira fuera de
        ROS; dentro del píxel van siempre con ella.

        El número es lo que permite decir "esta imagen es el frame 1234 del
        log": es el mismo que el CarLocation de ese fotograma lleva en n_frame
        y el que el algoritmo escribe en sus líneas [FRAME n]. Ojo: solo se
        publica imagen de debug de algunos fotogramas (el handshake save_data
        pide una imagen nueva cuando termina de dibujar la anterior), así que
        estos números avanzan a saltos. El índice de una imagen dentro del bag
        NO es el número de frame; el bueno es este.
        """
        # stamp es un builtin_interfaces/Time (sec + nanosec) tomado con el
        # reloj del nodo justo al leer el frame del hardware, en _capture_loop.
        # La hora se formatea en el huso del CONTENEDOR, que los
        # docker-compose igualan al del host con TZ + /etc/localtime (sin eso
        # ros:humble va en UTC y la marca sale con horas de desfase respecto
        # al reloj de la máquina). El huso se pinta al lado justamente para
        # que se note si alguna Raspberry se despliega mal configurada: ahí
        # pondría UTC en vez de la hora local.
        # Se saca un único struct_time y se formatea dos veces: llamar dos
        # veces a localtime podría caer a los dos lados de un cambio de hora.
        local = time.localtime(stamp.sec)
        texto = (
            f"{self.camara_id} #{n_frame} "
            f"{time.strftime('%H:%M:%S', local)}.{stamp.nanosec // 1_000_000:03d} "
            f"{time.strftime('%Z', local)}"
        )

        origen = (10, 25)  # esquina superior izquierda, ya dentro del frame
        fuente = cv.FONT_HERSHEY_SIMPLEX
        escala = 0.5
        # Recuadro negro de fondo para que el texto se lea igual sobre pista
        # clara que sobre pista oscura. El truco habitual de escribir dos
        # veces (contorno grueso negro + relleno fino blanco) NO vale aquí:
        # el grosor cambia el avance entre letras, así que las dos pasadas
        # salen desplazadas y el texto se ve doble.
        (ancho, alto), base = cv.getTextSize(texto, fuente, escala, 1)
        cv.rectangle(
            frame,
            (origen[0] - 4, origen[1] - alto - 4),
            (origen[0] + ancho + 4, origen[1] + base),
            (0, 0, 0),
            -1,
        )
        cv.putText(frame, texto, origen, fuente, escala, (255, 255, 255), 1, cv.LINE_AA)

    def _tarea_debug(self):
        if not self.debug or self.save_data:
            return

        # Referencia local: en cuanto save_data vuelva a True (lo pone
        # parameters_callback desde otro hilo) process_frame puede reasignar
        # next_debug_frame, y leerlo varias veces daría una imagen mezcla de
        # dos fotogramas. Coger el nombre una sola vez es atómico en Python.
        datos = self.next_debug_frame
        if datos is None:
            return
        frame_debug, stamp_debug, n_frame_debug = datos

        # Dibujamos los puntos de TODOS los coches que estén en el diccionario
        for car_name in self.coches:
            # Otra referencia local, y por un motivo más serio: la rama de
            # "coche no encontrado" de tarea_por_coche pone esta entrada a
            # None desde un hilo del pool. Si eso se colara entre el "is not
            # None" y el acceso, el callback petaría con un TypeError y se
            # llevaría el nodo por delante.
            puntos = self.puntos_for_debug[car_name]
            if puntos is None:
                continue

            debug_x = puntos["debug_x"]
            debug_y = puntos["debug_y"]
            for p in puntos["debug_points"]:
                # Coordenadas globales del bounding rect y del centro
                gx = int(debug_x + p["x"])
                gy = int(debug_y + p["y"])
                gw = int(gx + p["w"])
                gh = int(gy + p["h"])
                gcx = int(debug_x + p["cx"])
                gcy = int(debug_y + p["cy"])

                # Cadena de ifs para contraste según el color detectado (en BGR)
                if p["color"] in ["rojo", "naranja"]:
                    rect_color = (255, 255, 0)  # Cian
                elif p["color"] == "verde":
                    rect_color = (255, 0, 255)  # Magenta
                elif p["color"] == "azul":
                    rect_color = (0, 255, 255)  # Amarillo
                else:
                    rect_color = (255, 255, 0)  # Cian por defecto

                # 1. Dibujamos los bordes del rectángulo
                cv.rectangle(frame_debug, (gx, gy), (gw, gh), rect_color, 2)

                # 2. Dibujamos la cruceta en el centro exacto (reemplaza al cv.circle)
                c_size = 3  # Tamaño del aspa de la cruz
                cv.line(frame_debug, (gcx - c_size, gcy), (gcx + c_size, gcy), (0, 255, 255), 1)
                cv.line(frame_debug, (gcx, gcy - c_size), (gcx, gcy + c_size), (0, 255, 255), 1)

        # Lo último antes de comprimir: cámara, nº de frame y hora de captura
        # quemados en la imagen
        self._dibujar_marca(frame_debug, stamp_debug, n_frame_debug)

        # Comprimir y publicar
        success, buffer = cv.imencode(
            ".jpg", frame_debug, [cv.IMWRITE_JPEG_QUALITY, 70]
        )
        if success:
            msg = CompressedImage()
            # Instante de CAPTURA del frame, no el de publicación: es lo que
            # permite casar cada imagen con la posición que el coche tenía en
            # ese momento al analizar el bag
            msg.header.stamp = stamp_debug
            msg.format = "jpeg"
            msg.data = buffer.tobytes()
            self.debug_publisher.publish(msg)

        self.save_data = True

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()

    # Un hilo por grupo de callbacks que tiene que poder correr a la vez:
    #   1) image_processor_group  timer de posicion (el procesado del fotograma)
    #   2) debug_group            timer de la imagen de debug
    #   3) grupo por defecto      las suscripciones /<coche>/modo_calibracion
    #   4) net_probe_group        el eco de los sondeos de red
    # El cuarto es lo que garantiza que un sondeo se responde en cuanto llega y
    # no detras del procesado del fotograma en curso: si el eco tuviera que
    # esperar turno, esa espera se sumaria al RTT y la medida diria cuanto
    # tarda la camara en atender en vez de cuanto tarda la red.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(image_processor)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass  # Manejo limpio de Ctrl+C
    finally:
        image_processor.cam.release()
        image_processor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
