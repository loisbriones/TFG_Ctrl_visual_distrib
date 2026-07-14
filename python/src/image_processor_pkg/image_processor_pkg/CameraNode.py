#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from image_processor_pkg.msg import CarLocation, FinishLine, BoundingRect
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

from threading import Thread, Lock

from ProcessImage import ColorDetector
from concurrent.futures import ThreadPoolExecutor, wait
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
        self.declare_parameter("modo_calibracion", True)
        self.modo_calibracion = self.get_parameter("modo_calibracion").value
        # Nos subcribimos al topic para que nos puedan avisar de cuando acaba la calibracion 
        self.sub_modo_calibracion = self.create_subscription(Bool, "/modo_calibracion", self.callback_control, 10)
        # Trayectoria base que sigue el coche
        self.puntos_trayectoria = []
        # Mascara para calcular la trayectoria
        self.mascara_trayectoria = None
        # Tamaño kernel para generar la mascara 
        self.declare_parameter("camera.mascara_kernel_size", 25)
        self.mascara_kernel_size = self.get_parameter("camera.mascara_kernel_size").value
        # --------------------

        # ----- DETECTION -----
        self.declare_parameter("camera.detection.min_area", 50)
        self.min_area = self.get_parameter("camera.detection.min_area").value

        self.declare_parameter("camera.detection.stiker_front", "rojo")
        self.stiker_front = self.get_parameter("camera.detection.stiker_front").value

        self.declare_parameter("camera.detection.stiker_back", "verde")
        self.stiker_back = self.get_parameter("camera.detection.stiker_back").value

        self.declare_parameter("camera.detection.kernel_size", 5)
        self.kernel_size = self.get_parameter("camera.detection.kernel_size").value
        # ---------------------

        # --- COLOR DETECTOR ---
        self.color_detector = ColorDetector(self.stiker_front, self.stiker_back, self.kernel_size)
        # ----------------------

        # ---- DEBUG ----
        self.declare_parameter("camera.debug", False)
        self.debug = self.get_parameter("camera.debug").value
        self.next_debug_frame = None
        self.puntos_for_debug = {}
        # Para controlar el cuando hay datos de debug y cuando no
        self.save_data = False

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
            }

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

        # ---- TIMER ----
        # Posicion
        self.timer = self.create_timer(0.033, self.process_frame, callback_group=self.image_processor_group)
        # Debug
        self.debug_timer = self.create_timer(0.066, self._tarea_debug, callback_group=self.debug_group)

        # --- BUSCAR LINEA DE META ---
        self.declare_parameter("finish_line_color", "naranja")
        self.finish_line_color = self.get_parameter("finish_line_color").value

        ret, frame_for_find_sectors = self.cam.read()
        self.finish_line_position = self.color_detector.find_finish_line(frame_for_find_sectors, self.finish_line_color)

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

                for car_name in self.coches:
                    if car_name in datos_cache:
                        self.info_coches[car_name]["puntos_trayectoria"] = datos_cache[car_name]
                        # Generamos la mascara para cada coche con la info almacenada
                        self.info_coches[car_name]["mascara_trayectoria"] = (self.generar_mascara(datos_cache[car_name]))

                # Si cargamos la caché, saltamos la calibración
                self.modo_calibracion = False
                self.get_logger().info(
                    f"🟢 Caché cargada desde {self.cache_file}. Modo calibración omitido."
                )

            except Exception as e:
                self.get_logger().error(
                    f"Error cargando caché: {e}. Se forzará calibración."
                )
                self.modo_calibracion = True

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
                with self.frame_lock:
                    self.latest_frame = frame

    def callback_control(self, msg):
        # Modo operacion
        if msg.data == False and self.modo_calibracion:
            datos_a_guardar = {}

            for car_name in self.coches:
                datos_a_guardar[car_name] = self.info_coches[car_name][
                    "puntos_trayectoria"
                ]
                self.info_coches[car_name]["mascara_trayectoria"] = (
                    self.generar_mascara(
                        self.info_coches[car_name]["puntos_trayectoria"]
                    )
                )

            # Guardamos en disco la trayectoria de puntos
            try:
                with open(self.cache_file, "w") as f:
                    json.dump(datos_a_guardar, f)
                self.get_logger().info(f"💾 Trayectoria guardada en {self.cache_file}")
            except Exception as e:
                self.get_logger().error(f"Error guardando caché: {e}")

            self.modo_calibracion = False

            self.get_logger().info("Calibracion Terminada")

        # Modo calibracion
        elif msg.data == True:
            for car_name in self.coches:
                self.info_coches[car_name]["mascara_trayectoria"] = None

            self.modo_calibracion = True

            self.get_logger().warn("Reiniciando Calibracion")

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

            elif param.name == "camera.detection.stiker_front":
                self.stiker_front = param.value
                self.actualizar_detector()
                self.get_logger().info(
                    f"Parámetro actualizado: color_1 = {self.stiker_front}"
                )

            elif param.name == "camera.detection.stiker_back":
                self.stiker_back = param.value
                self.actualizar_detector()
                self.get_logger().info(
                    f"Parámetro actualizado: color_2 = {self.stiker_back}"
                )

            elif param.name == "camera.detection.kernel_size":
                if param.value % 2 == 0:
                    result.successful = False
                    result.reason = "El kernel_size debe ser un número impar"
                else:
                    # Para que tenga efecto es necesario poner el modo calibracion por seguridad
                    self.mascara_kernel_size = param.value

            elif param.name == "camera.mascara_kernel_size":
                if param.value % 2 == 0:
                    result.successful = False
                    result.reason = "El kernel_size debe ser un número impar"
                else:
                    self.kernel_size = param.value
                    self.actualizar_detector()

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
        """Función auxiliar para re-instanciar el detector con los nuevos valores."""
        self.color_detector = ColorDetector(self.stiker_front, self.stiker_back, self.kernel_size)

    def publish_car_position(self, detections, proc_duration, x, y, car_name):
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
                    object_location_msg.front.color = self.stiker_front
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
                    object_location_msg.back.color = self.stiker_back
                    
                    # Guardar la posición del rectangulo que detectamos
                    # También hay que ajustarlo porque se sacan las coordenas de dentro del ROI
                    object_bounding_rect_back.x = p["x"] + x 
                    object_bounding_rect_back.y = p["y"] + y
                    object_bounding_rect_back.w = p["w"] + x 
                    object_bounding_rect_back.h = p["h"] + y
                    
                    # Guardamos la posición dentro del /car1/position 
                    object_location_msg.bounding_rect_stiker_back = object_bounding_rect_back
        
        object_location_msg.proc_time = proc_duration

        # Publicamos los puntos detectados
        object_location_msg.stamp = self.get_clock().now().to_msg()

        self.publisher_coche[car_name].publish(object_location_msg)

    def process_frame(self):
        """Timer de ROS que consume el último frame disponible."""
        if self.is_processing:
            return

        current_frame = None
        with self.frame_lock:
            if self.latest_frame is not None:
                current_frame = self.latest_frame
                self.latest_frame = (
                    None  # Consumimos el frame para no repetir procesado
                )

        if current_frame is None:
            return

        self.is_processing = True
        try:
            if self.debug and self.save_data:
                self.next_debug_frame = current_frame.copy()

            futures = []
            for car_name in self.coches:
                futures.append(
                    self.thread_pool.submit(
                        self.tarea_por_coche,
                        current_frame,
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
    def tarea_por_coche(self, frame, car_name, info):

        # --- MODO CALIBRACIÓN ---
        if self.modo_calibracion:
            detections = self.color_detector.find_object(frame, self.min_area, self.stiker_front, self.stiker_back)
            if detections["front"] is not None and detections["back"] is not None:
                info["puntos_trayectoria"].append((detections["front"]["cx"], detections["front"]["cy"]))
                # Usamos 0,0 como offset porque es el frame completo
                # Solo publicamos en calibración si hemos detectado algo
                self.publish_car_position(detections, 0.0, 0, 0, car_name)
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
        detections = self.color_detector.find_object(roi_frame, self.min_area, self.stiker_front, self.stiker_back)
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
            self.publish_car_position(detections, duration, x1, y1, car_name)

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

    def _tarea_debug(self):
        if not self.debug or self.save_data or self.next_debug_frame is None:
            return

        # Dibujamos los puntos de TODOS los coches que estén en el diccionario
        for car_name in self.coches:
            if self.puntos_for_debug[car_name] is not None:
                debug_x = self.puntos_for_debug[car_name]["debug_x"]
                debug_y = self.puntos_for_debug[car_name]["debug_y"]
                for p in self.puntos_for_debug[car_name]["debug_points"]:
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
                    cv.rectangle(self.next_debug_frame, (gx, gy), (gw, gh), rect_color, 2)

                    # 2. Dibujamos la cruceta en el centro exacto (reemplaza al cv.circle)
                    c_size = 3  # Tamaño del aspa de la cruz
                    cv.line(self.next_debug_frame, (gcx - c_size, gcy), (gcx + c_size, gcy), (0, 255, 255), 1)
                    cv.line(self.next_debug_frame, (gcx, gcy - c_size), (gcx, gcy + c_size), (0, 255, 255), 1)

        # Comprimir y publicar
        success, buffer = cv.imencode(
            ".jpg", self.next_debug_frame, [cv.IMWRITE_JPEG_QUALITY, 70]
        )
        if success:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = buffer.tobytes()
            self.debug_publisher.publish(msg)

        self.save_data = True

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()

    executor = MultiThreadedExecutor(num_threads=3)
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
