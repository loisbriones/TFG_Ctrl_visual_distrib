#!/usr/bin/python3
import rclpy
from rclpy.node import Node
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
import os
import csv


class PositionReceiver(Node):
    def __init__(self):
        super().__init__("position_receiver")

        # 1. Configuración del archivo CSV
        # Guardamos en la carpeta del paquete para que persista fuera de Docker
        self.csv_path = "/ros2_ws/src/image_processor_pkg/mediciones_python.csv"
        self.preparar_csv()

        # 2. Suscripción
        self.subscription = self.create_subscription(
            ObjectLocation,
            "object_position",
            self.topic_callback,
            qos_profile_sensor_data,  #
        )

        self.get_logger().info("Node PositionReceiver Ready and logging to CSV")  #

    def preparar_csv(self):
        """Crea la cabecera si el archivo no existe o si está vacío."""
        # Verificamos si el archivo no existe O si su tamaño es 0 bytes
        necesita_cabecera = (
            not os.path.exists(self.csv_path) or os.stat(self.csv_path).st_size == 0
        )

        if necesita_cabecera:
            with open(self.csv_path, mode="w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        "timestamp_ns",
                        "color",
                        "cpu_proc_ms",
                        "network_lat_ms",
                        "total_lat_ms",
                    ]
                )
            self.get_logger().info("Cabecera del CSV creada con éxito.")

    def topic_callback(self, msg):
        # Tiempo en el que se recibe el mensaje
        tiempo_recibido = self.get_clock().now()
        # Tiempo en que el mensaje salió del emisor
        tiempo_envio = rclpy.time.Time.from_msg(msg.stamp)

        # 1. Latencia de red pura (tiempo de viaje)
        latencia_red_ns = tiempo_recibido - tiempo_envio
        latencia_red_ms = latencia_red_ns.nanoseconds / 1e6

        # 2. Tiempo de procesamiento en la otra RPi (viene en el mensaje)
        cpu_ms = msg.proc_time

        # 3. Latencia Total (desde que la cámara captó el frame hasta que llegó aquí)
        total_ms = cpu_ms + latencia_red_ms

        # --- GUARDAR DATOS ---
        with open(self.csv_path, mode="a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    tiempo_recibido.nanoseconds,
                    msg.color,
                    f"{cpu_ms:.4f}",
                    f"{latencia_red_ms:.4f}",
                    f"{total_ms:.4f}",
                ]
            )

        # --- LOGS POR CONSOLA ---
        self.get_logger().info(
            f"RECIBIDO -> Color: {msg.color} | CPU: {cpu_ms:.2f}ms | RED: {latencia_red_ms:.2f}ms"
        )


def main(args=None):
    rclpy.init(args=args)

    position_receiver = PositionReceiver()

    try:
        # Mantiene el nodo abierto procesando los mensajes entrantes
        rclpy.spin(position_receiver)
    except KeyboardInterrupt:
        pass
    finally:
        position_receiver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
