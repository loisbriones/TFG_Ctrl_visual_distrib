#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from image_processor_pkg.msg import SpeedCarril
from std_msgs.msg import Bool

from rclpy.qos import qos_profile_sensor_data

from ArduinoController import ArduinoController

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from ament_index_python.packages import get_package_share_directory

import yaml
import os


class ArduinoBridgeNode(Node):
    """
    Nodo de ROS2 que traduce las ordenes de PWM a lo que entiende el Arduino
    Escucha el /<coche>/pwd de cada coche y manda el valor por el puerto serie
    con ArduinoController
    """

    def __init__(self):
        super().__init__("arduino_bridge")

        # --- Obtener la lista de coches desde el YAML ---
        self.declare_parameter("coches", ["car1"])
        self.coches = self.get_parameter("coches").value

        # Diccionario para mantener controlada la velocidad actual de cada carril
        self.rails = {}

        # Lista para no perder la referencia de las suscripciones
        self.sub_pwd = {}

        # Subscriber de /<coche>/pwd -> SpeedCarril.msg, uno por coche
        for car_name in self.coches:
            topic_name = f"/{car_name}/pwd"
            self.sub_pwd[car_name] = self.create_subscription(
                SpeedCarril,
                topic_name,
                self.pwm_callback,
                qos_profile_sensor_data,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )

        # --- CALIBRACION POR COCHE ---
        # Numero de carril del coche
        self.carril_de = {}
        # Permite saber si tiene el modo de calibracion activado
        self.calibrando = {}
        self.sub_modo_calibracion = {}
        self.grupo_calibracion = MutuallyExclusiveCallbackGroup()

        for car_name in self.coches:

            # Numero de carril para el coche
            self.declare_parameter(f"cars.{car_name}.carril", "1")

            # Como se controla la potencia del coche
            self.declare_parameter(f"cars.{car_name}.modo", "automatico")
            modo = self.get_parameter(f"cars.{car_name}.modo").value

            if modo == "manual":
                continue

            carril_str = str(self.get_parameter(f"cars.{car_name}.carril").value)
            self.carril_de[car_name] = self._parse_carril(carril_str)
            self.calibrando[car_name] = True

            # Subscriber de /<coche>/modo_calibracion -> Bool
            self.sub_modo_calibracion[car_name] = self.create_subscription(
                Bool,
                f"/{car_name}/modo_calibracion",
                lambda msg, c=car_name: self.callback_control_calibracion(c, msg),
                10,
                callback_group=self.grupo_calibracion,
            )

        # --- PARAMETROS ---
        self.declare_parameter("arduino.port", "/dev/ttyACM0")
        port = self.get_parameter("arduino.port").value

        self.declare_parameter("arduino.baudrate", 115200)
        baud = self.get_parameter("arduino.baudrate").value

        self.declare_parameter("arduino.calibration_speed", 60)
        self.calibration_speed = self.get_parameter("arduino.calibration_speed").value

        # Nos conectamos al arduino
        self.arduino = ArduinoController(port=port, baudrate=baud)
        self.get_logger().info(f"Conectando a Arduino en {port}...")

        # Los carriles de los coches que no son manuales arrancan con calibration_speed
        for car_name, carril in self.carril_de.items():
            self.arduino.set_rail_speed(carril, self.calibration_speed)

    @staticmethod
    def _parse_carril(carril_str):
        """Convierte el carril del YAML, que puede venir como "r2" o "2", en 2"""
        limpio = carril_str.strip().lower().replace("'", "").replace('"', "")
        return int(limpio.replace("r", ""))

    def callback_control_calibracion(self, car_name, msg):
        """Callback que se llama cuando llega un mensaje del topic de calibracion
        Pasa a modo carrera o vuelve al modo calibracion de un coche
        """
        if msg.data == False and self.calibrando.get(car_name, False):
            # Cerramos la calibracion
            self.calibrando[car_name] = False
        elif msg.data == True and not self.calibrando.get(car_name, True):
            # Iniciamos la calibracion
            self.calibrando[car_name] = True
            self.arduino.set_rail_speed(self.carril_de[car_name], self.calibration_speed)

    def pwm_callback(self, msg: SpeedCarril):
        """Callback que se llama cuando llega un mensaje de /<coche>/pwd
        Se recibe el valor de PWM que hay que aplicar y se manda al Arduino"""

        pwm_value = msg.pwm

        # Quitamos comillas, espacios y mayusculas antes de leer el carril
        rail_str = str(msg.carril).strip().lower().replace("'", "").replace('"', "")

        try:
            rail_num = int(rail_str.replace("r", ""))
        except ValueError:
            self.get_logger().error(f"Formato de carril inválido: {msg.carril}")
            return

        # Carril que aparece por primera vez
        if rail_num not in self.rails:
            # Se guarda un -1 porque no es un PWM valido y asi el primero siempre se manda
            self.rails[rail_num] = -1

        # El algoritmo repite el valor de PWM, asi que solo se manda si cambia
        if pwm_value == self.rails[rail_num]:
            return

        self.rails[rail_num] = pwm_value

        # El Arduino solo acepta valores dentro de [0,255]
        if 0 <= pwm_value <= 255:
            self.get_logger().info(f"Arduino OK -> Carril {rail_num} a PWM {pwm_value}")
            self.arduino.set_rail_speed(rail_num, pwm_value)

    def destroy_node(self):
        """Para los carriles al cerrar, antes de destruir el nodo"""
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

    # Se lee el YAML solo para saber cuantos coches hay y con eso el numero de
    # hilos del Executor
    with open(params_file, "r") as f:
        config = yaml.safe_load(f)
        coches = config["/**"]["ros__parameters"]["coches"]

    executor = MultiThreadedExecutor(num_threads=(2 + len(coches)))
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
