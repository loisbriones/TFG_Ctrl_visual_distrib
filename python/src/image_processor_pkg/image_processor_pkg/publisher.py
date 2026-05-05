#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from image_processor_pkg.msg import ObjectLocation, PathAndSectors, LineSegment, Point2D
from sensor_msgs.msg import CompressedImage

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
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from procesar_imagen import ColorDetector
import time

CAMERA_MODES = {
    0: (160, 120),
    1: (320, 240),
    2: (640, 480),
    3: (800, 600),
    4: (1280, 720),
}


class ImageProcessor(Node):
    def __init__(self):
        super().__init__("image_processor")
    
        # ---- CALL GROUPS ----
        #Posicion
        self.image_processor_group = MutuallyExclusiveCallbackGroup()
        #Debug
        self.debug_group = MutuallyExclusiveCallbackGroup()
        
        # ---- ID NODO ---- 
        self.declare_parameter("camara_id","rbiTemp")  
        self.camara_id=  self.get_parameter("camara_id").value

        # ----- CAMARA -----
        self.declare_parameter("camera.mode", 2)
        mode = self.get_parameter("camera.mode").value

        # Coger modo camara para saber w,h del frame
        self.width, self.height = CAMERA_MODES[mode]

        #Seleccionamos la camara
        self.cam = cv.VideoCapture(0, cv.CAP_V4L2)
        # Esto le pide a la cámara que envíe los datos ya comprimidos en JPEG
        self.cam.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))
        #Configuramos el ancho de la camara
        self.cam.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
        #Configuramos el alto de la camara
        self.cam.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
        #Desactivamos el autoenfoque de la camara
        self.cam.set(cv.CAP_PROP_AUTOFOCUS, 0)
    
        # ---- CALIBRACION ----
        self.declare_parameter("modo_calibracion", True)
        #Trayectoria base que sigue el coche
        self.puntos_trayectoria = []
        #Mascara para calcular la trayectoria
        self.mascara_trayectoria = None
        
        # ----- DETECTION -----
        self.declare_parameter("detection.min_area", 50)
        self.declare_parameter("detection.target_color_1", "rojo")
        self.declare_parameter("detection.target_color_2", "verde")
        self.declare_parameter("detection.kernel_size", 5)

        self.min_area = self.get_parameter("detection.min_area").value
        self.target_color_1 = self.get_parameter("detection.target_color_1").value
        self.target_color_2 = self.get_parameter("detection.target_color_2").value
        self.kernel_size = self.get_parameter("detection.kernel_size").value

        #Clase que tiene configurada la logica de deteccion
        self.color_detector = ColorDetector(
            self.target_color_1, self.target_color_2, self.kernel_size
        )
        
        #Posiciones para ir modificando el ROI
        self.rx = 0
        self.ry = 0 
        self.rw = self.width
        self.rh = self.height
        #Tamaño del ROI
        self.roi_size = 150
        #Posicion en un momento concreto de la esquina
        self.current_cx = None
        self.current_cy = None
        #Para que en el modo debug se dibuje bien el centro del objeto detectado
        self.debug_x = 0
        self.debug_y = 0 
        self.modo_calibracion = True  # Empezar en modo grabación de ruta
        #Puntos para hacer prediccion lineal para mover el ROI
        self.prev_cx = None
        self.prev_cy = None

        # ---- DEBUG ----
        self.declare_parameter("debug", False)
        self.debug = self.get_parameter("debug").value
        self.next_debug_frame = None
        self.next_debug_points = None
        #Para controlar el cuando hay datos de debug y cuando no
        self.new_data_available = False

        # ---- ACTUALIZACION PARAMETROS ----
        self.add_on_set_parameters_callback(self.parameters_callback)

        # ---- PUBLISHER ----
        # Posicion
        self.object_location_publisher = self.create_publisher(ObjectLocation, "/object_position", qos_profile_sensor_data)  
        #Debug
        self.debug_publisher = self.create_publisher(CompressedImage, "camara_debug", qos_profile_sensor_data)

        # Secciones y ruta base
        sector_and_path_qos_profile = QoSProfile(
            depth=1, # Guardar solo el último mensaje
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_and_sectors_msg = PathAndSectors()
        self.path_and_sectors_publisher = self.create_publisher(PathAndSectors,"path_and_sectors",sector_and_path_qos_profile)

        # ---- TIMER ----
        #Posicion
        self.timer = self.create_timer(0.033, self.process_frame, callback_group=self.image_processor_group)
        #Debug
        self.debug_timer = self.create_timer(0.066, self._tarea_debug, callback_group=self.debug_group)

        # --- SECCIONES ---
        self.declare_parameter("detection.sector_color", "naranja")
        self.sector_color = self.get_parameter("detection.sector_color").value
        # Detectar las secciones
        self.sectors = self.obtener_secciones()


        self.get_logger().info("Node ImageProcessor Ready")

    def obtener_secciones(self):
        ret, frame = self.cam.read()

        if not ret:
            return
        
        return self.color_detector.find_sector(frame,self.sector_color)

    def parameters_callback(self, params):
        result = SetParametersResult(successful=True)

        for param in params:
            if param.name == "detection.min_area":
                if param.value < 0:
                    result.successful = False
                    result.reason = "El área mínima no puede ser negativa"
                else:
                    self.min_area = param.value
                    self.get_logger().info(
                        f"Parámetro actualizado: min_area = {self.min_area}"
                    )

            elif param.name == "detection.target_color_1":
                self.target_color_1 = param.value
                self.actualizar_detector()
                self.get_logger().info(
                    f"Parámetro actualizado: color_1 = {self.target_color_1}"
                )

            elif param.name == "detection.target_color_2":
                self.target_color_2 = param.value
                self.actualizar_detector()
                self.get_logger().info(
                    f"Parámetro actualizado: color_2 = {self.target_color_2}"
                )

            elif param.name == "detection.kernel_size":
                if param.value % 2 == 0:
                    result.successful = False
                    result.reason = "El kernel_size debe ser un número impar"
                else:
                    self.kernel_size = param.value
                    self.actualizar_detector()

            elif param.name == "debug":
                self.debug = param.value
                self.get_logger().info(f"Parámetro actualizado: debug = {self.debug}")

            elif param.name == "modo_calibracion":

                # Si pasamos de True a False (Fin de calibración)
                if self.modo_calibracion == True and param.value == False:
                    self.get_logger().info("Finalizando calibración: Generando máscara y enviando ruta...")
                    self.mascara_trayectoria = self.generar_mascara()
                    self.enviar_path_and_sectors()
                
                # Si pasamos de False a True (Reiniciar calibración)
                elif param.value == True:
                    self.get_logger().info("Reiniciando calibración: Limpiando datos previos")
                    self.puntos_trayectoria = []
                    self.path_and_sectors_msg.front = []
                    self.path_and_sectors_msg.back = []

                self.modo_calibracion = param.value            
                
        return result

    def enviar_path_and_sectors(self):
        # Usamos el mensaje que ya tenemos instanciado
        self.path_and_sectors_msg.camara_id = self.camara_id
        
        # Limpiamos los sectores previos por si acaso
        self.path_and_sectors_msg.sectores = []

        for segment in self.sectors:
            line_msg = LineSegment()
            
            # Punto de inicio
            line_msg.start.x = int(segment[0][0])
            line_msg.start.y = int(segment[0][1])
            
            # Punto de fin
            line_msg.end.x = int(segment[1][0])
            line_msg.end.y = int(segment[1][1])
            
            # Añadir a la lista 'sectores' (según tu archivo .msg)
            self.path_and_sectors_msg.sectores.append(line_msg)
        
        # Publicar el mensaje correcto
        self.path_and_sectors_publisher.publish(self.path_and_sectors_msg)


    def generar_mascara(self):
        # Crear lienzo negro
        mascara = np.zeros((self.height, self.width), dtype=np.uint8)

        if len(self.puntos_trayectoria) < 2:
            return mascara

        puntos = np.array(self.puntos_trayectoria, dtype=np.int32)

        # Dibujar la trayectoria uniendo los puntos con líneas blancas
        # isClosed=True para cerrar el circuito al final
        cv.polylines(mascara, [puntos], isClosed=True, color=255, thickness=15)

        kernel = np.ones((25, 25), np.uint8)
        # Cierre morfologico para eliminar pequeños puntos negros que puedan quedar fruto de no detectar nada 
        mascara_final = cv.morphologyEx(mascara, cv.MORPH_CLOSE, kernel)
        # Dilatiacion expandimos los bordes de la mascara hacia fuera aumenta el area de la mascara
        mascara_final = cv.dilate(mascara_final, kernel, iterations=1)

        return mascara_final

    def actualizar_detector(self):
        """Función auxiliar para re-instanciar el detector con los nuevos valores."""
        self.color_detector = ColorDetector(
            self.target_color_1, self.target_color_2, self.kernel_size
        )

    def process_frame(self):
        # Capturar frame
        ret, frame = self.cam.read()

        if not ret:
            return
        
        if self.modo_calibracion: 
            # Buscamos en todo el frame para detectar el coche
            # Ahora detections es un diccionario: {"front": ..., "back": ...}
            detections = self.color_detector.find_object(
                frame, self.min_area, self.target_color_1, self.target_color_2
            ) 

            # Guardamos la trayectoria usando cualquier punto detectado
            for key in ["front", "back"]:
                p = detections[key]
                if p is not None:         
                    front_point = Point2D()
                    back_point = Point2D()

                    self.puntos_trayectoria.append((p["cx"], p["cy"]))
                    if key == "front":
                        front_point.x = p["cx"]
                        front_point.y = p["cy"]
                        
                        self.path_and_sectors_msg.front.append(front_point)

                    if key == "back":
                        back_point.x = p["cx"]
                        back_point.y = p["cy"]
                        
                        self.path_and_sectors_msg.back.append(back_point)
                
            return
        
        # Calcular punto de predicción (Extrapolación lineal)
        if self.prev_cx is not None and self.current_cx is not None:
            pred_x = self.current_cx + (self.current_cx - self.prev_cx)
            pred_y = self.current_cy + (self.current_cy - self.prev_cy)
        elif self.current_cx is not None:
            pred_x, pred_y = self.current_cx, self.current_cy
        else:
            pred_x, pred_y = self.width // 2, self.height // 2
            self.roi_size = max(self.width, self.height)
            
        half_roi = self.roi_size // 2
        
        # Calcular ROI
        x1 = int(np.clip(pred_x - half_roi, 0, self.width))
        y1 = int(np.clip(pred_y - half_roi, 0, self.height))
        x2 = int(np.clip(pred_x + half_roi, 0, self.width))
        y2 = int(np.clip(pred_y + half_roi, 0, self.height))

        # Aplicamos el ROI y la máscara de trayectoria
        roi_frame = frame[y1:y2, x1:x2]
        frame_procesar = cv.bitwise_and(roi_frame, roi_frame, mask=self.mascara_trayectoria[y1:y2, x1:x2])
        
        # Buscar coche en el frame
        start_proc = time.perf_counter()
        detections = self.color_detector.find_object(
            frame_procesar, self.min_area, self.target_color_1, self.target_color_2
        )
        end_proc = time.perf_counter()

        # Calculamos el tiempo de procesado
        proc_duration = (end_proc - start_proc) * 1000
        
        # Verificamos si se ha detectado al menos una parte del coche (frontal o trasera)
        found_any = detections["front"] is not None or detections["back"] is not None

        if found_any:
            # Usamos una de las detecciones para actualizar el seguimiento del ROI (preferiblemente el frontal)
            p_ref = detections["front"] if detections["front"] is not None else detections["back"]
            
            global_cx = x1 + p_ref["cx"]
            global_cy = y1 + p_ref["cy"]
    
            self.prev_cx, self.prev_cy = self.current_cx, self.current_cy
            self.current_cx, self.current_cy = global_cx, global_cy
            self.roi_size = 150 
             
        else:
            self.roi_size = min(self.roi_size + 50, max(self.width, self.height))
            self.current_cx = None
            self.prev_cx = None

        # Publicar los puntos detectados
        
        valid_points_for_debug = []
        object_location_msg = ObjectLocation()

        object_location_msg.camara_id = self.camara_id

        for key in ["front", "back"]:
            p = detections[key]
            if p is not None:
                if key == "front":
                    object_location_msg.front.center.x = p["cx"] + x1 
                    object_location_msg.front.center.y = p["cy"] + y1 
                    object_location_msg.front.color = self.target_color_1 
                if key == "back":
                    object_location_msg.back.center.x = p["cx"] + x1 
                    object_location_msg.back.center.y = p["cy"] + y1 
                    object_location_msg.back.color = self.target_color_1 

                valid_points_for_debug.append(p)
        
        object_location_msg.proc_time = proc_duration

        # Publicamos los puntos detectados
        object_location_msg.stamp = self.get_clock().now().to_msg()
        self.object_location_publisher.publish(object_location_msg)
    
        # Configuración de valores de debug
        if self.debug and not self.new_data_available:
            self.next_debug_frame = frame.copy()
            self.debug_x, self.debug_y = x1, y1 
            self.next_debug_points = valid_points_for_debug
            self.new_data_available = True        


    def _tarea_debug(self):
        
        if not self.new_data_available:
            return
        
        self.new_data_available = False

        for p in self.next_debug_points:
            cv.circle(self.next_debug_frame, (self.debug_x + p["cx"], self.debug_y + p["cy"]), 5, (0, 255, 255), -1) 

        success, buffer = cv.imencode('.jpg', self.next_debug_frame, [cv.IMWRITE_JPEG_QUALITY, 70])
        
        if success:
            img_msg = CompressedImage()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.format = "jpeg"
            img_msg.data = buffer.tobytes()
            self.debug_publisher.publish(img_msg) 

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    
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
