#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from image_processor_pkg.msg import ObjectLocation

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


class ImageProcessor(Node):
    def __init__(self):
        super().__init__("image_processor")

        self.publisher_ = self.create_publisher(
            ObjectLocation, "object_position", qos_profile_sensor_data
        )

        self.cap = cv.VideoCapture(0, cv.CAP_V4L2)

        self.timer = self.create_timer(0.033, self.process_frame)

        self.get_logger().info("Node ImageProcessor Ready")

    def process_frame(self):
        # Capturar frame
        ret, frame = self.cap.read()

        if not ret:
            return

        # Pasamos la imagen a HSV
        frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # --- CONFIGURACIÓN ROJO ---
        lower_red1, upper_red1 = np.array([0, 100, 100]), np.array([10, 255, 255])
        lower_red2, upper_red2 = np.array([170, 100, 100]), np.array([180, 255, 255])

        mask_red = cv.bitwise_or(
            cv.inRange(frame_hsv, lower_red1, upper_red1),
            cv.inRange(frame_hsv, lower_red2, upper_red2),
        )

        # --- CONFIGURACIÓN VERDE ---
        lower_green = np.array([35, 100, 100])
        upper_green = np.array([85, 255, 255])
        mask_green = cv.inRange(frame_hsv, lower_green, upper_green)

        # Limpieza morfologica
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        mask_red = cv.morphologyEx(mask_red, cv.MORPH_OPEN, kernel)
        mask_green = cv.morphologyEx(mask_green, cv.MORPH_OPEN, kernel)

        # Aplicamos la detección para ambos
        self.detect_and_publish(mask_red, "ROJO")
        self.detect_and_publish(mask_green, "VERDE")

    def detect_and_publish(self, mask, color_name):
        contornos, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        for c in contornos:
            if cv.contourArea(c) > 50:
                x, y, w, h = cv.boundingRect(c)

                cx = x + (w // 2)
                cy = y + (h // 2)

                msg = ObjectLocation()
                msg.color = color_name
                msg.x = cx
                msg.y = cy

                self.publisher_.publish(msg)
                self.get_logger().info(
                    f"[deteccion] color: {color_name} | x: {msg.x} | y: {msg.y}"
                )


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
