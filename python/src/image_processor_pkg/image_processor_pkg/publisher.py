#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from image_processor_pkg.msg import ObjectLocation
from sensor_msgs.msg import CompressedImage
from threading import Thread

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
from rclpy.parameter import Parameter

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

        # ----- CAMARA -----
        self.declare_parameter("camera.mode", 2)
        mode = self.get_parameter("camera.mode").value

        width, height = CAMERA_MODES[mode]

        self.cam = cv.VideoCapture(0, cv.CAP_V4L2)
        self.cam.set(cv.CAP_PROP_FRAME_WIDTH, width)
        self.cam.set(cv.CAP_PROP_FRAME_HEIGHT, height)

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
        
        # ---- DEBUG ----
        self.declare_parameter("debug", False)
        self.debug = self.get_parameter("debug").value
        self.debug_counter = 0

        # ---- ACTUALIZACION PARAMETROS ----
        self.add_on_set_parameters_callback(self.parameters_callback)

        # ---- PUBLISHER -----
        # Posicion
        self.msg = ObjectLocation()
        self.object_location_publisher = self.create_publisher(ObjectLocation, "object_position", qos_profile_sensor_data)  
        # Debug
        self.debug_publisher = self.create_publisher(CompressedImage, "camara_debug", qos_profile_sensor_data)


        self.timer = self.create_timer(0.033, self.process_frame)

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

        return result

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

        start_proc = time.perf_counter()

        points = self.color_detector.find_object(
            frame, self.min_area, self.target_color_1, self.target_color_2
        )

        end_proc = time.perf_counter()

        # Convertir a ms
        proc_duration = (end_proc - start_proc) * 1000
        
        if (self.debug == True) and ((self.debug_counter % 2) == 0):
            Thread(target=self._tarea_debug,args=(frame.copy(),points),daemon=True)  

        for p in points:
            self.msg.color = p["color"]
            self.msg.x = p["cx"]
            self.msg.y = p["cy"]
            self.msg.proc_time = proc_duration

            self.msg.stamp = self.get_clock().now().to_msg()

            self.object_location_publisher.publish(self.msg)
            self.get_logger().info(
                f"[deteccion] color: {self.msg.color} | x: {self.msg.x} | y: {self.msg.y}"
            )

    def _tarea_debug(self,frame,points):
        for p in points:
            cv.circle(frame, (p["cx"], p["cy"]), 5, (0, 255, 0), -1) 

        success, buffer = cv.imencode('.jpg', frame, [cv.IMWRITE_JPEG_QUALITY, 70])
        
        if success:
            img_msg = CompressedImage()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.format = "jpeg"
            img_msg.data = buffer.tobytes()
            self.image_pub.publish(img_msg) 

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()

    try:
        rclpy.spin(image_processor)
    except KeyboardInterrupt:
        pass  # Manejo limpio de Ctrl+C
    finally:
        image_processor.cap.release()
        image_processor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
