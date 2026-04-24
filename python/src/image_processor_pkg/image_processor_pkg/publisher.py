#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from image_processor_pkg.msg import ObjectLocation
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
from rclpy.qos import qos_profile_sensor_data
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
        #Pensar en si poner un valor por defecto
        self.declare_parameter("node_id","rbiTemp")  
        self.node_id =  self.get_parameter("node_id").value

        # ----- CAMARA -----
        self.declare_parameter("camera.mode", 2)
        mode = self.get_parameter("camera.mode").value

        self.width, self.height = CAMERA_MODES[mode]

        self.cam = cv.VideoCapture(0, cv.CAP_V4L2)
        self.cam.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
        self.cam.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cam.set(cv.CAP_PROP_AUTOFOCUS, 0)

        # ---- CALIBRACION ----
        self.declare_parameter("modo_calibracion", True)
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
        self.new_data_available = False

        # ---- ACTUALIZACION PARAMETROS ----
        self.add_on_set_parameters_callback(self.parameters_callback)

        # ---- PUBLISHER ----
        # Posicion
        self.msg = ObjectLocation()
        self.object_location_publisher = self.create_publisher(ObjectLocation, "/object_position", qos_profile_sensor_data)  
        #Debug
        self.debug_publisher = self.create_publisher(CompressedImage, "camara_debug", qos_profile_sensor_data)

        # ---- TIMER ----
        #Posicion
        self.timer = self.create_timer(0.033, self.process_frame, callback_group=self.image_processor_group)
        #Debug
        self.debug_timer = self.create_timer(0.066, self._tarea_debug, callback_group=self.debug_group)

        self.get_logger().info("Node ImageProcessor Ready")

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
                self.mascara_trayectoria = self.generar_mascara()
                self.modo_calibracion = param.value 
                self.get_logger().info(f"Parámetro actualizado: debug = {self.modo_calibracion}")
                
        return result

    def generar_mascara(self):
        # 1. Crear lienzo negro
        mascara = np.zeros((self.height, self.width), dtype=np.uint8)

        if len(self.puntos_trayectoria) < 2:
            return mascara

        # 2. Dibujar la trayectoria uniendo los puntos con líneas blancas
        puntos = np.array(self.puntos_trayectoria, dtype=np.int32)
        # isClosed=True para cerrar el circuito al final
        cv.polylines(mascara, [puntos], isClosed=True, color=255, thickness=15)

        # 3. Engrosar y suavizar el carril (Cierre Morfológico)
        kernel = np.ones((25, 25), np.uint8)
        mascara_final = cv.morphologyEx(mascara, cv.MORPH_CLOSE, kernel)
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
            points = self.color_detector.find_object(
                frame, self.min_area, self.target_color_1, self.target_color_2
            ) 

            for p in points:         
                self.puntos_trayectoria.append((p["cx"],p["cy"]))

            return
        
        #Calcular punto de predicción (Extrapolación lineal)
        if self.prev_cx is not None and self.current_cx is not None:
            # P_futuro = P_actual + (P_actual - P_anterior)
            pred_x = self.current_cx + (self.current_cx - self.prev_cx)
            pred_y = self.current_cy + (self.current_cy - self.prev_cy)
        elif self.current_cx is not None:
            # Si solo tenemos un punto, el ROI se centra en él
            pred_x, pred_y = self.current_cx, self.current_cy
        else:
            # Si perdimos el coche, buscamos en todo el frame 
            pred_x, pred_y = self.width // 2, self.height // 2
            self.roi_size = max(self.width, self.height)
            
        half_roi = self.roi_size // 2
        
        #Mover ROI hacia atras
        x1 = int(np.clip(pred_x - half_roi, 0, self.width))
        y1 = int(np.clip(pred_y - half_roi, 0, self.height))
        #Mover ROI hacia delante
        x2 = int(np.clip(pred_x + half_roi, 0, self.width))
        y2 = int(np.clip(pred_y + half_roi, 0, self.height))

        #Aplicamos el ROI al Frame 
        roi_frame = frame[y1:y2, x1:x2]
        
        #Aplicamos la mascara sobre la zona del frame a la que aplicamos el ROI
        frame_procesar = cv.bitwise_and(roi_frame, roi_frame, mask=self.mascara_trayectoria[y1:y2, x1:x2])
        
        start_proc = time.perf_counter()

        points = self.color_detector.find_object(
            frame_procesar, self.min_area, self.target_color_1, self.target_color_2
        )

        end_proc = time.perf_counter()

        # Convertir a ms
        proc_duration = (end_proc - start_proc) * 1000
        
        if len(points) > 0:
            # Traducir coordenadas locales del ROI a globales del Frame
            p = points[0]
            global_cx = x1 + p["cx"]
            global_cy = y1 + p["cy"]
    
            # Actualizar estado global para la siguiente iteración 
            self.prev_cx, self.prev_cy = self.current_cx, self.current_cy
            self.current_cx, self.current_cy = global_cx, global_cy
            self.roi_size = 150 # Reestablecer tamaño optimizado tras detección exitosa
             
        else:
            # Si no se detecta nada, se amplía el área de búsqueda para el próximo frame
            self.roi_size = min(self.roi_size + 50, max(self.width, self.height))
            # Resetear historial para forzar búsqueda completa si persiste el fallo
            self.current_cx = None
            self.prev_cx = None

        for p in points:
            # Publicación del mensaje ObjectLocation
            self.msg.node_id = self.node_id
            self.msg.color = p["color"]
            self.msg.x = p["cx"] + x1 
            self.msg.y = p["cy"] + y1 
            self.msg.proc_time = proc_duration
            self.msg.stamp = self.get_clock().now().to_msg()
            
            self.object_location_publisher.publish(self.msg)
    
        # Configuracion de valores de debug
        if self.debug and not self.new_data_available:
            self.next_debug_frame = frame.copy()
            self.debug_x, self.debug_y = x1,y1 
            self.next_debug_points = points
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
