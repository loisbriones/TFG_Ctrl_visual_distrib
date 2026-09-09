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

# Perfil de QoS para datos de sensor (keep last, depth 5, best effort)
# Si un mensaje se pierde no se reintenta, porque enseguida llega otro mas fresco
# https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1SensorDataQoS.html
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
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, # Guarda el ultimo mensaje y lo entrega al que se suscriba despues
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
)


class ImageProcessor(Node):
    """
    Nodo de ROS2 que captura frames de su camara, busca en ellos las pegatinas de
    cada coche con ColorDetector (ProcessImage) y publica la posicion en
    /<coche>/position. Tambien busca la linea de meta y publica el frame que
    captura para depurar
    """

    def __init__(self):
        super().__init__("image_processor")

        # ---- CALL GROUPS ----
        # Posicion
        self.image_processor_group = MutuallyExclusiveCallbackGroup()
        # Debug
        self.debug_group = MutuallyExclusiveCallbackGroup()
        # Respuesta a los echos de red
        self.net_probe_group = MutuallyExclusiveCallbackGroup()

        # ---- ID NODO ----
        self.declare_parameter("camara_id", "camara")
        self.camara_id = self.get_parameter("camara_id").value
        # -----------------

        # ----- CAMARA -----
        # Tamaño del frame, sale del diccionario de modos
        self.declare_parameter("camera.mode", 2)
        mode = self.get_parameter("camera.mode").value
        # Coger modo camara para saber w,h del frame
        self.width, self.height = CAMERA_MODES[mode]
 
        # dynamic_typing porque el dispositivo puede venir como numero (0) o
        # como ruta ("/dev/video0"), y ROS no comprueba el tipo hasta que llega
        device_param = self.declare_parameter('camera.device','0',ParameterDescriptor(dynamic_typing=True))
        device_param = self.get_parameter('camera.device').value

        try:
            camera_name = int(device_param)
        except ValueError:
            # Si no es un numero, es la ruta /dev/videoN
            camera_name = device_param

        self.cam = cv.VideoCapture(camera_name, cv.CAP_V4L2)
        # Pedirle a la camara los datos ya comprimidos en JPEG
        self.cam.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))
        self.cam.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
        self.cam.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
        # Sin autoenfoque
        self.cam.set(cv.CAP_PROP_AUTOFOCUS, 0)
        # ---------------------

        # ---- CALIBRACION POR COCHE ----
        # Todos arrancan en modo calibracion por defecto
        self.declare_parameter("modo_calibracion", True)
        self.calib_default = self.get_parameter("modo_calibracion").value
        self.sub_modo_calibracion = {}

        # Tamaño kernel para generar la mascara de la trayectoria
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
        self.car_stikers = {}
        self.color_detectors = {}
        # ---------------------------------

        # ---- DEBUG ----
        self.declare_parameter("camera.debug", False)
        self.debug = self.get_parameter("camera.debug").value

        self.next_debug_frame = None
        self.puntos_for_debug = {}

        self.save_data = self.debug
        # ---------------------------------
        
        # ---- CONTADOR DE FRAMES ----
        self.n_frame = 0
        # ---------------------------------

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

            # Colores de las pegatinas para el coche
            # Salen de cars.<coche>.stiker_front y stiker_back en params.yaml
            param_front = f"cars.{car_name}.stiker_front"
            param_back = f"cars.{car_name}.stiker_back"

            self.declare_parameter(param_front, "rojo")
            self.declare_parameter(param_back, "verde")

            s_front = self.get_parameter(param_front).value
            s_back = self.get_parameter(param_back).value

            self.car_stikers[car_name] = {"front": s_front, "back": s_back}
            # Creamos el detector para este coche
            self.color_detectors[car_name] = ColorDetector(s_front, s_back, self.kernel_size)

            self.get_logger().info(f"[COCHE] {car_name} -> frontal: {s_front} | trasero: {s_back}")

            # Publisher de /<coche>/position -> CarLocation.msg
            self.publisher_coche[car_name] = self.create_publisher(
                CarLocation, f"/{car_name}/position", qos_profile_sensor_data
            )

            # Diccionario para guardar la informacion de los coches
            self.info_coches[car_name] = {
                # Para calcular el ROI
                "prev_cx": None,
                "prev_cy": None,
                "current_cx": None,
                "current_cy": None,
                "roi_size": 150,
                # Puntos usados para construir la mascara
                "puntos_trayectoria": [],
                # Mascara generada
                "mascara_trayectoria": None,
                # Estado de la calibracion de este coche
                "modo_calibracion": self.calib_default,
            }

            # Subscriber de /<coche>/modo_calibracion -> Bool
            self.sub_modo_calibracion[car_name] = self.create_subscription(
                Bool,
                f"/{car_name}/modo_calibracion",
                # partial fija car_name para que el callback sepa de que coche viene
                functools.partial(self.callback_control, car_name),
                10,
            )

            # Diccionario para guardar informacion de debug del coche
            self.puntos_for_debug[car_name] = {
                "debug_x": 0,
                "debug_y": 0,
                "debug_points": [],
            }
        # ---------------------------------------

        # -------------- Debug ------------------
        # Publisher de camara_debug -> CompressedImage
        # Manda el frame ya anotado y comprimido en JPEG
        self.debug_publisher = self.create_publisher(
            CompressedImage, "camara_debug", qos_profile_sensor_data
        )
        # ---------------------------------------

        # ---- RESPUESTA A LOS ECHOS DE RED ----
        # La camara no mide nada, solo devuelve el echo que le manda el NetProbeNode
        # y apunta cuanto tardo ella, para que el emisor lo pueda descontar
        # Los topics son relativos, asi que "net_probe" es /camara_XX/net_probe

        self.declare_parameter("net_probe.enabled", True)
        self.net_probe_enabled = self.get_parameter("net_probe.enabled").value

        if self.net_probe_enabled:

            # Publisher de net_probe_echo -> NetProbeEcho.msg
            self.net_probe_publisher = self.create_publisher(
                NetProbeEcho, "net_probe_echo", qos_profile_sensor_data
            )

            # Subscriber de net_probe -> NetProbe.msg
            self.net_probe_subscription = self.create_subscription(
                NetProbe,
                "net_probe",
                self.callback_net_probe,
                qos_profile_sensor_data,
                callback_group=self.net_probe_group,
            )

            self.get_logger().info("Eco de sondeos de red activo")
        # -------------------------------

        # ---- TIMER ----
        # Posicion
        self.timer = self.create_timer(0.033, self.process_frame, callback_group=self.image_processor_group)
        # Debug
        self.debug_timer = self.create_timer(0.033, self._tarea_debug, callback_group=self.debug_group)

        # ---LINEA DE META ---
        self.declare_parameter("camera.detection.finish_line_color", "naranja")
        self.finish_line_color = self.get_parameter("camera.detection.finish_line_color").value

        # Leemos un frame para buscar la posicion de la linea de meta
        ret, frame_for_find_sectors = self.cam.read()

        # La meta es del circuito y no de un coche, asi que vale cualquier
        # detector. Se coge el del primer coche, o uno suelto si no hay ninguno
        if self.coches:
            detector_meta = self.color_detectors[self.coches[0]]
        else:
            detector_meta = ColorDetector("", "", self.kernel_size)

        # Buscamos la linea de meta en el frame
        self.finish_line_position = detector_meta.find_finish_line(frame_for_find_sectors, self.finish_line_color)

        if self.finish_line_position is not None and ret:

            # Publisher de /finish_line_position -> FinishLine.msg
            self.finish_line_publisher = self.create_publisher(FinishLine, "/finish_line_position", QOS_FINISH_LINE)

            # Creamos el mensaje y lo rellenamos
            finish_line_msg = FinishLine()
            finish_line_msg.camara_id = self.camara_id

            finish_line_msg.finish_line.start.x = float(self.finish_line_position[0][0])
            finish_line_msg.finish_line.start.y = float(self.finish_line_position[0][1])

            finish_line_msg.finish_line.end.x = float(self.finish_line_position[1][0])
            finish_line_msg.finish_line.end.y = float(self.finish_line_position[1][1])

            # Se publica el mensaje
            self.finish_line_publisher.publish(finish_line_msg)


        # --- CARGAR RUTA DE CACHE ---
        self.cache_file = (f"/ros2_ws/src/image_processor_pkg/cache_trayectoria_{self.camara_id}.json" )

        if os.path.exists(self.cache_file):
            try:
                with open(self.cache_file, "r") as f:
                    datos_cache = json.load(f)

                # Si el coche ya tiene ruta arranca directo, si no queda en calibracion
                for car_name in self.coches:
                    if car_name in datos_cache:

                        self.info_coches[car_name]["puntos_trayectoria"] = datos_cache[car_name]
                        # Generamos la mascara con la info almacenada
                        self.info_coches[car_name]["mascara_trayectoria"] = (self.generar_mascara(datos_cache[car_name]))
                        # Como hay trayectoria arranca en carrera, no pasa por calibracion
                        self.info_coches[car_name]["modo_calibracion"] = False

                        self.get_logger().info(
                            f"[CACHE] {car_name}: caché cargada desde {self.cache_file}, calibración omitida"
                        )

            except Exception as e:
                self.get_logger().error(
                    f"Error cargando caché: {e}. Todos los coches se quedan en calibración."
                )
                # La cache no se pudo cargar, todos los coches quedan en calibracion
                for car_name in self.coches:
                    self.info_coches[car_name]["modo_calibracion"] = self.calib_default

        # --- THREAD CAPTURA ---
        self.latest_frame = None
        self.frame_lock = Lock()
        self.running = True

        # Iniciamos el hilo de captura
        self.capture_thread = Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        self.get_logger().info("Node ImageProcessor Ready")

    def _capture_loop(self):
        """Funcion que ejecuta el hilo de captura, guarda cada frame nuevo que llega"""

        while self.running and rclpy.ok():
            ret, frame = self.cam.read()
            if ret:
                # Se coge el instante de captura
                stamp = self.get_clock().now().to_msg()
                with self.frame_lock:
                    self.latest_frame = (frame, stamp)

    def callback_control(self, car_name, msg):
        """Callback que se ejecuta cuando llega un mensaje al topic de calibracion
        de uno de los coches. Termina o empieza la calibracion de ese coche"""

        info = self.info_coches[car_name]

        # Comprobamos en que modo se encuentra el coche
        if msg.data == False and info["modo_calibracion"]:

            # Si no esta en calibracion se genera la mascara
            info["mascara_trayectoria"] = self.generar_mascara(
                info["puntos_trayectoria"]
            )

            # Se abre el JSON y se modifica la trayectoria de este coche
            try:
                datos_cache = {}
                if os.path.exists(self.cache_file):
                    with open(self.cache_file, "r") as f:
                        datos_cache = json.load(f)
                datos_cache[car_name] = info["puntos_trayectoria"]
                with open(self.cache_file, "w") as f:
                    json.dump(datos_cache, f)
                self.get_logger().info(
                    f"[CACHE] {car_name}: trayectoria guardada en {self.cache_file}"
                )
            except Exception as e:
                self.get_logger().error(f"Error guardando caché de {car_name}: {e}")

            info["modo_calibracion"] = False

            self.get_logger().info(f"Calibracion Terminada ({car_name})")

        # Modo calibracion: reinicio de la calibracion de este coche
        elif msg.data == True:
            # Estamos en modo carrera y queremos pasar a calibracion
            info["mascara_trayectoria"] = None
            info["modo_calibracion"] = True

            self.get_logger().warn(f"Reiniciando Calibracion ({car_name})")

    def callback_net_probe(self, msg):
        """
        Callback que se llama cuando llega un echo del NetProbeNode
        Se devuelve la respuesta cronometrando cuanto tiempo se tarda en armarla
        """

        t_entrada_ns = time.monotonic_ns()

        eco = NetProbeEcho()

        # Se rellena la respuesta con la informacion que nos llego
        eco.seq = msg.seq
        eco.origen = msg.origen
        eco.destino = msg.destino
        eco.t_send = msg.t_send
        # Se añade el mismo relleno para que la respuesta pese lo mismo que el echo
        eco.padding = msg.padding

        # Se mide lo que costo armar la respuesta
        eco.proc_ns = time.monotonic_ns() - t_entrada_ns

        self.net_probe_publisher.publish(eco)

    def parameters_callback(self, params):
        """
        Callback que se ejecuta cuando desde ROS se modifica un parametro en
        caliente con ros2 param set
        """
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
                # Cambio de color de pegatina, cars.<coche>.stiker_front o back
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
                    # Para que tenga efecto hay que volver a calibrar, por seguridad
                    self.mascara_kernel_size = param.value

            elif param.name == "camera.debug":
                self.debug = param.value
                self.save_data = param.value

                self.get_logger().info(f"Parámetro actualizado: debug = {self.debug}")

        return result

    def generar_mascara(self, puntos_trayectoria):
        """
        Genera la mascara de la trayectoria, que permite eliminar ruido de los frames
        Es una imagen binaria que deja visible solo una franja alrededor del
        recorrido del coche
        """
        mascara = np.zeros((self.height, self.width), dtype=np.uint8)

        if len(puntos_trayectoria) < 2:
            return mascara

        puntos = np.array(puntos_trayectoria, dtype=np.int32)

        # La trayectoria como una linea blanca gruesa, cerrada por el final
        cv.polylines(mascara, [puntos], isClosed=True, color=255, thickness=15)

        kernel = np.ones((self.mascara_kernel_size, self.mascara_kernel_size), np.uint8)

        # Cierre para tapar los agujeros que dejan los tramos sin detectar, y
        # dilatacion para ensanchar la franja y dar margen al coche
        mascara_final = cv.morphologyEx(mascara, cv.MORPH_CLOSE, kernel)
        mascara_final = cv.dilate(mascara_final, kernel, iterations=1)

        return mascara_final

    def actualizar_detector(self):
        """Rehace el ColorDetector de cada coche con los colores y el kernel que
        haya ahora. Se llama al cambiar alguno de los dos en caliente"""
        for car_name in self.coches:
            s_front = self.car_stikers[car_name]["front"]
            s_back = self.car_stikers[car_name]["back"]
            self.color_detectors[car_name] = ColorDetector(s_front, s_back, self.kernel_size)

    def publish_car_position(self, detections, proc_duration, x, y, car_name, frame_stamp, n_frame):
        """
        Publica la informacion de una deteccion de un coche en un CarLocation.msg
        """

        # Creamos los mensajes
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
                    # Guardar la posicion del rectangulo que detectamos
                    # Tambien hay que ajustarlo porque se sacan las coordenas de dentro del ROI
                    object_bounding_rect_front.x = p["x"] + x 
                    object_bounding_rect_front.y = p["y"] + y
                    object_bounding_rect_front.w = p["w"] + x 
                    object_bounding_rect_front.h = p["h"] + y
                    
                    object_location_msg.bounding_rect_stiker_front = object_bounding_rect_front

                if key == "back":
                    object_location_msg.back.center.x = p["cx"] + x
                    object_location_msg.back.center.y = p["cy"] + y
                    object_location_msg.back.color = self.car_stikers[car_name]["back"]
                    
                    # Guardar la posicion del rectangulo que detectamos
                    # Tambien hay que ajustarlo porque se sacan las coordenas de dentro del ROI
                    object_bounding_rect_back.x = p["x"] + x 
                    object_bounding_rect_back.y = p["y"] + y
                    object_bounding_rect_back.w = p["w"] + x 
                    object_bounding_rect_back.h = p["h"] + y
                    
                    object_location_msg.bounding_rect_stiker_back = object_bounding_rect_back
        
        object_location_msg.proc_time = proc_duration

        # Instante de captura del frame, no el de publicacion
        object_location_msg.stamp = frame_stamp

        object_location_msg.n_frame = n_frame

        # Tiempo desde que se capturo el frame hasta que se publico el mensaje
        ahora = self.get_clock().now()
        object_location_msg.age_at_publish = (
            ahora - Time.from_msg(frame_stamp)
        ).nanoseconds / 1e9

        self.publisher_coche[car_name].publish(object_location_msg)

    def process_frame(self):
        """
        Timer de ROS a 30 Hz que coge el ultimo frame que haya capturado el
        hilo de la camara y lanza la busqueda de cada coche
        """
        if self.is_processing:
            return

        current_frame = None
        frame_stamp = None
        with self.frame_lock:
            if self.latest_frame is not None:
                # latest_frame es la tupla (frame, stamp de captura)
                current_frame, frame_stamp = self.latest_frame
                self.latest_frame = (
                    None  
                )

        if current_frame is None:
            return

        # Se cuenta cuando se procesa el frame, no cuando se captura
        self.n_frame += 1

        self.is_processing = True
        try:
            if self.debug and self.save_data:
                # Se guarda lo que hace falta para el debug
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

    def calcular_roi_bounds(self, info):
        """
        Calcula la posicion del ROI para el proximo frame

        El ROI con extrapolacion y la readquisicion son ideas heredadas de los
        TFG de Mario (https://github.com/mariolopez15/control_coche_scalextric)
        y de Adrian (https://github.com/Rego523/GEI-TFG).
        """
        prev_cx = info["prev_cx"]
        prev_cy = info["prev_cy"]
        curr_cx = info["current_cx"]
        curr_cy = info["current_cy"]
        roi_size = info["roi_size"]

        # Extrapolacion lineal
        if prev_cx is not None and curr_cx is not None:
            pred_x = curr_cx + (curr_cx - prev_cx)
            pred_y = curr_cy + (curr_cy - prev_cy)
        elif curr_cx is not None:
            pred_x, pred_y = curr_cx, curr_cy
        else:
            # Sin rastro, se busca en el centro y con el frame entero
            pred_x, pred_y = self.width // 2, self.height // 2
            info["roi_size"] = max(self.width, self.height)
            roi_size = info["roi_size"]

        half_roi = roi_size // 2

        # Recortado a los bordes del frame
        x1 = int(np.clip(pred_x - half_roi, 0, self.width))
        y1 = int(np.clip(pred_y - half_roi, 0, self.height))
        x2 = int(np.clip(pred_x + half_roi, 0, self.width))
        y2 = int(np.clip(pred_y + half_roi, 0, self.height))

        return x1, y1, x2, y2

    def tarea_por_coche(self, frame, frame_stamp, n_frame, car_name, info):
        """
        Tarea que ejecuta cada uno de los threads del pool
        Busca las pegatinas de un coche en el frame y publica su posicion
        """
        # Cada hilo trabaja con el detector de su coche
        detector = self.color_detectors[car_name]
        s_front = self.car_stikers[car_name]["front"]
        s_back = self.car_stikers[car_name]["back"]

        # --- MODO CALIBRACION ---
        if info["modo_calibracion"]:

            detections = detector.find_object(frame, self.min_area, s_front, s_back)

            # Para la calibracion solo interesa la pegatina de delante
            if detections["front"] is not None:
                info["puntos_trayectoria"].append((detections["front"]["cx"], detections["front"]["cy"]))
                # El offset es (0, 0) porque en calibracion se busca en el frame completo
                self.publish_car_position(detections, 0.0, 0, 0, car_name, frame_stamp, n_frame)

            return

        # --- MODO OPERACION---
        x1, y1, x2, y2 = self.calcular_roi_bounds(info)

        # Cogemos solo el ROI del frame
        roi_frame = frame[y1:y2, x1:x2]

        # Hacemos un and entre el ROI y la mascara generada, en caso de haber mascara
        if info["mascara_trayectoria"] is not None:
            mask_roi = info["mascara_trayectoria"][y1:y2, x1:x2]
            roi_frame = cv.bitwise_and(roi_frame, roi_frame, mask=mask_roi)

        # Buscamos las dos pegatinas
        start = time.perf_counter()
        detections = detector.find_object(roi_frame, self.min_area, s_front, s_back)
        duration = (time.perf_counter() - start) * 1000

        found_any = detections["front"] is not None and detections["back"] is not None

        if found_any:
            # Para el seguimiento se prefiere la delantera
            p_ref = (
                detections["front"]
                if detections["front"] is not None
                else detections["back"]
            )

            # Del ROI al frame entero
            global_cx = x1 + p_ref["cx"]
            global_cy = y1 + p_ref["cy"]

            info["prev_cx"], info["prev_cy"] = info["current_cx"], info["current_cy"]
            info["current_cx"], info["current_cy"] = global_cx, global_cy
            info["roi_size"] = 150

            self.publish_car_position(detections, duration, x1, y1, car_name, frame_stamp, n_frame)

            if self.debug and self.save_data:
                # Puede venir a None de haber perdido el coche antes
                if self.puntos_for_debug[car_name] is None:
                    self.puntos_for_debug[car_name] = {}

                self.puntos_for_debug[car_name]["debug_x"] = x1
                self.puntos_for_debug[car_name]["debug_y"] = y1
                self.puntos_for_debug[car_name]["debug_points"] = [
                    v for v in detections.values() if v is not None
                ]

        else:
            # Sin deteccion se pierde el rastro y calcular_roi_bounds pasa a
            # buscar sobre el frame entero en el proximo ciclo
            # info["roi_size"] = min(info["roi_size"] + 50, max(self.width, self.height))
            info["current_cx"] = None
            info["prev_cx"] = None
            # A None para que el debug no siga dibujando el coche donde ya no esta
            self.puntos_for_debug[car_name] = None

    def _dibujar_marca(self, frame, stamp, n_frame):
        """
        Añade en la esquina del frame que camara es, el numero de frame y la
        hora de captura
        """

        local = time.localtime(stamp.sec)
        texto = (
            f"{self.camara_id} #{n_frame} "
            f"{time.strftime('%H:%M:%S', local)}.{stamp.nanosec // 1_000_000:03d} "
            f"{time.strftime('%Z', local)}"
        )

        # Esquina superior izquierda
        origen = (10, 25)
        fuente = cv.FONT_HERSHEY_SIMPLEX
        escala = 0.5
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
        """
        Tarea de debug que dibuja sobre el ultimo frame lo que se ha detectado y
        lo publica comprimido en /camara_XX/camara_debug
        """

        if not self.debug or self.save_data:
            return

        datos = self.next_debug_frame
        if datos is None:
            return
        frame_debug, stamp_debug, n_frame_debug = datos

        for car_name in self.coches:
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

                # Cadena de ifs para contraste segun el color detectado
                if p["color"] in ["rojo", "naranja"]:
                    rect_color = (255, 255, 0)  # Cian
                elif p["color"] == "verde":
                    rect_color = (255, 0, 255)  # Magenta
                elif p["color"] == "azul":
                    rect_color = (0, 255, 255)  # Amarillo
                else:
                    rect_color = (255, 255, 0)  # Cian por defecto

                # 1. Dibujamos los bordes del rectangulo
                cv.rectangle(frame_debug, (gx, gy), (gw, gh), rect_color, 2)

                # 2. Dibujamos la cruceta en el centro
                c_size = 3  
                cv.line(frame_debug, (gcx - c_size, gcy), (gcx + c_size, gcy), (0, 255, 255), 1)
                cv.line(frame_debug, (gcx, gcy - c_size), (gcx, gcy + c_size), (0, 255, 255), 1)

        # Añadimos la informacion al frame
        self._dibujar_marca(frame_debug, stamp_debug, n_frame_debug)

        # Comprimir y publicar
        success, buffer = cv.imencode(
            ".jpg", frame_debug, [cv.IMWRITE_JPEG_QUALITY, 70]
        )
        if success:
            msg = CompressedImage()
            # Instante de captura del frame
            msg.header.stamp = stamp_debug
            msg.format = "jpeg"
            msg.data = buffer.tobytes()
            self.debug_publisher.publish(msg)

        self.save_data = True

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(image_processor)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass  
    finally:
        image_processor.cam.release()
        image_processor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
