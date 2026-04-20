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

        # Ruta base donde querermos guardar los csv
        self.csv_base_path = "/ros2_ws/src/image_processor_pkg/mediciones_python_"

        #Diccionario para gestionar los escritores de csv
        self.recursos = {}

        self.preparar_csv()

        # 2. Suscripción
        self.subscription = self.create_subscription(
            ObjectLocation,
            "/object_position",
            self.topic_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info("Node PositionReceiver Ready")

    def obtener_recurso(self,nodo_id):
        if nodo_id not in self.recursos:
            file_path = f"{self.csv_base_path}{nodo_id}.csv"
            
            # Comprobación de cabecera
            necesita_cabecera = not os.path.exists(file_path) or os.stat(file_path).st_size == 0
            
            f = open(file_path, mode="a", newline="")
            writer = csv.writer(f)
            
            if necesita_cabecera:
                writer.writerow([
                    "timestamp_ns", "color", "cpu_proc_ms", 
                    "network_lat_ms", "total_lat_ms"
                ])
                # Para forzar a que haga la escritura a disco para asegurar tener cabecera
                f.flush()

            # Guardamos la pareja (archivo, escritor) en el diccionario
            self.recursos[nodo_id] = (f, writer)

        return self.recursos[nodo_id]

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
        _, writer = self.obtener_recurso(msg.node_id)

        writer.writerow([
            tiempo_recibido.nanoseconds,
            msg.color,
            f"{cpu_ms:.4f}",
            f"{latencia_red_ms:.4f}",
            f"{total_ms:.4f}",
        ])

        # --- LOGS POR CONSOLA ---
        self.get_logger().info(
            f"RECIBIDO -> Color: {msg.color} | CPU: {cpu_ms:.2f}ms | RED: {latencia_red_ms:.2f}ms"
        )

    def destroy_node(self):
        """ Cerramos los ficheros abiertos, iterando sobre el diccionario """
        for robot_id, (f_obj, _) in self.recursos.items():
            f_obj.close()
            self.get_logger().info(f"Archivo cerrado: ID {robot_id}")
        super().destroy_node()


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
