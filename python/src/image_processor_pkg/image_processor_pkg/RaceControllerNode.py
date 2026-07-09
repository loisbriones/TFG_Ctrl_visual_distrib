#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from image_processor_pkg.msg import SpeedCarril
from std_msgs.msg import Bool

from rclpy.qos import qos_profile_sensor_data

from ArduinoController import ArduinoController

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from ament_index_python.packages import get_package_share_directory

from threading import Lock
import yaml
import os
import time


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__("arduino_bridge")

        # --- Obtener la lista de coches desde el YAML ---
        self.declare_parameter("coches", ["car1"])
        self.coches = self.get_parameter("coches").value

        # TIMER
        # Cada cuanto tiempo comprobamos si el nodo Controlador está caído
        self.declare_parameter("hearthbear_timer", 0.066)
        self.heartbear_timer = self.get_parameter("hearthbear_timer").value

        # Delta t: periodo que dejamos que pase desde que recibimos un paquete
        self.declare_parameter("delta_t", 1)
        self.delta_t = self.get_parameter("delta_t").value

        # Lock para poder leer y escribir marcas de tiempo de forma segura entre hilos
        self.check_timer_lock = Lock()

        # Diccionario para mantener controlada la velocidad actual de cada carril
        self.rails = {}
        
        # [MODIFICADO] Diccionario para guardar el timestamp de cada carril de forma independiente
        self.rail_timestamps = {}

        # Lista para no perder la referencia de las suscripciones
        self.sub_pwd = {}
        # Diccionario para guardar las suscripciones de control de heartbeat
        self.sub_heartbeat_check = {}

        # Nos suscribimos al topic "pwd" de CADA coche (ej. /car1/pwd, /car2/pwd)
        for car_name in self.coches:
            topic_name = f"/{car_name}/pwd"
            self.sub_pwd[car_name] = self.create_subscription(
                SpeedCarril,
                topic_name,
                self.pwm_callback,
                qos_profile_sensor_data,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )

            group = MutuallyExclusiveCallbackGroup()
            self.sub_heartbeat_check[car_name] = {
                "group": group,
                "timer": self.create_timer(
                    self.heartbear_timer, self.check_heartbeat, callback_group=group
                ),
            }

        self.modo_calibracion = True
        self.grupo_calibracion = MutuallyExclusiveCallbackGroup()
        self.sub_modo_calibracion = self.create_subscription(
            Bool,
            "/modo_calibracion",
            self.callback_control_calibracion,
            10,
            callback_group=self.grupo_calibracion,
        )

        # --- PARAMETROS ---
        self.declare_parameter("arduino.port", "/dev/ttyACM0")
        self.declare_parameter("arduino.baudrate", 115200)
        self.declare_parameter("arduino.calibration_speed", 60)

        port = self.get_parameter("arduino.port").value
        baud = self.get_parameter("arduino.baudrate").value
        self.calibration_speed = self.get_parameter("arduino.calibration_speed").value

        # Nos conectamos al arduino
        self.arduino = ArduinoController(port=port, baudrate=baud)
        self.get_logger().info(f"Conectando a Arduino en {port}...")

        # Arrancamos con la velocidad de calibración
        self.arduino.set_both_rails(self.calibration_speed, self.calibration_speed)

    def callback_control_calibracion(self, msg):
        if msg.data == False and self.modo_calibracion:
            self.modo_calibracion = False
        elif msg.data == True and not self.modo_calibracion:
            self.modo_calibracion = True
            # Arrancamos con la velocidad de calibración
            self.arduino.set_both_rails(self.calibration_speed, self.calibration_speed)

    def check_heartbeat(self):
        """
        [MODIFICADO] Comprueba de forma independiente si cada carril sigue emiting telemetría.
        Si un carril supera delta_t sin enviar mensajes, se detiene únicamente ese raíl.
        """
        if not self.modo_calibracion:
            with self.check_timer_lock:
                ahora = self.get_clock().now()
                
                # Iteramos por cada carril del que hayamos recibido datos alguna vez
                for rail_num, timestamp in list(self.rail_timestamps.items()):
                    t_ultimo_msg = Time.from_msg(timestamp)
                    diferencia_segundos = (ahora - t_ultimo_msg).nanoseconds / 1e9

                    # Si ESTE carril concreto ha superado el tiempo límite (delta_t)
                    if diferencia_segundos > self.delta_t:
                        # Comprobamos que no esté ya parado para no saturar el cable USB
                        if self.rails.get(rail_num, -1) > 1:
                            self.get_logger().warn(
                                f"⚠️ Pérdida de telemetría en Carril {rail_num}. Deteniendo SOLO ese carril."
                            )
                            # 1. Detenemos ÚNICAMENTE el raíl del coche que se ha salido
                            self.arduino.set_rail_speed(rail_num, 1)
                            
                            # 2. Reseteamos la memoria de ESE raíl a 1 para que acepte 
                            # el nuevo PWM en cuanto el coche vuelva a ponerse en la pista
                            self.rails[rail_num] = 1

    def pwm_callback(self, msg: SpeedCarril):
        """
        Cada vez que llega un nuevo valor de PWM, lo enviamos al Arduino.
        """
        pwm_value = msg.pwm

        # Limpieza agresiva: quitamos comillas (simples y dobles), espacios y pasamos a minúscula
        rail_str = str(msg.carril).strip().lower().replace("'", "").replace('"', "")

        # Extraemos el número del carril (ej: de "r2" o "2" sacamos el entero 2)
        try:
            rail_num = int(rail_str.replace("r", ""))
        except ValueError:
            self.get_logger().error(f"Formato de carril inválido: {msg.carril}")
            return

        # [MODIFICADO] Guardamos la marca de tiempo ESPECÍFICA de este carril
        with self.check_timer_lock:
            self.rail_timestamps[rail_num] = msg.stamp

        # Inicializamos dinámicamente el carril en la memoria del puente si no existía
        if rail_num not in self.rails:
            self.rails[rail_num] = -1

        # Filtro antispam para no saturar al Arduino mandando repetidamente el mismo valor
        if pwm_value == self.rails[rail_num]:
            return

        self.rails[rail_num] = pwm_value

        # Validación de seguridad y envío físico
        if 0 <= pwm_value <= 255:
            self.get_logger().info(
                f"⚡ Arduino OK -> Carril {rail_num} a PWM {pwm_value}"
            )
            self.arduino.set_rail_speed(rail_num, pwm_value)

    def destroy_node(self):
        """
        Al cerrar el nodo, nos aseguramos de parar el coche por seguridad.
        """
        self.get_logger().info("Cerrando conexión. Deteniendo motores...")
        if hasattr(self, "arduino") and self.arduino:
            self.arduino.stop_all_rails()
            self.arduino.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()

    pkg_share = get_package_share_directory("image_processor_pkg")
    params_file = os.path.join(pkg_share, "config", "params.yaml")

    # 1. Cargamos la lista de coches desde el YAML
    with open(params_file, "r") as f:
        config = yaml.safe_load(f)
        coches = config["/**"]["ros__parameters"]["coches"]

    executor = MultiThreadedExecutor(num_threads=(2 + (2 * len(coches))))
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
