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
from procesar_imagen import ColorDetector
import time


class ImageProcessor(Node):
    def __init__(self):
        super().__init__("image_processor")

        self.publisher_ = self.create_publisher(
            ObjectLocation, "object_position", qos_profile_sensor_data
        )

        self.cap = cv.VideoCapture(0, cv.CAP_V4L2)

        self.color_detector = ColorDetector()

        self.msg = ObjectLocation()

        self.timer = self.create_timer(0.033, self.process_frame)

        self.get_logger().info("Node ImageProcessor Ready")

    def process_frame(self):
        # Capturar frame
        ret, frame = self.cap.read()

        if not ret:
            return

        start_proc = time.perf_counter()
        points = self.color_detector.find_object(frame)
        end_proc = time.perf_counter()

        # Convertir a ms
        proc_duration = (end_proc - start_proc) * 1000
        self.get_logger().info(f"TIEMPO PROCESADO: {proc_duration:.4f} ms")

        for p in points:
            self.msg.color = p["color"]
            self.msg.x = p["cx"]
            self.msg.y = p["cy"]
            self.msg.proc_time = proc_duration

            self.msg.stamp = self.get_clock().now().to_msg()

            self.publisher_.publish(self.msg)
            self.get_logger().info(
                f"[deteccion] color: {self.msg.color} | x: {self.msg.x} | y: {self.msg.y}"
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
