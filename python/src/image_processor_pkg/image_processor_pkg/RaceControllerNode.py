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
    def __init__(self):
        super().__init__("arduino_bridge")

        # --- Obtener la lista de coches desde el YAML ---
        self.declare_parameter("coches", ["car1"])
        self.coches = self.get_parameter("coches").value

        # Diccionario para mantener controlada la velocidad actual de cada carril
        self.rails = {}

        # Lista para no perder la referencia de las suscripciones
        self.sub_pwd = {}

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

        # --- CALIBRACION POR COCHE (solo autonomos) ---
        # La calibracion dejo de ser global: cada coche autonomo cierra su
        # vuelta cuando quiere y solo entonces su carril pasa de calibration_speed
        # al PWM de carrera. Necesitamos por eso el carril fisico de cada coche
        # (cars.<coche>.carril) y saber cuales son autonomos (modo_manual False):
        # los manuales los conduce una persona, su carril NO lo toca el puente.
        self.carril_de = {}       # coche autonomo -> nº de carril
        self.calibrando = {}      # coche autonomo -> sigue en calibracion?
        self.sub_modo_calibracion = {}
        self.grupo_calibracion = MutuallyExclusiveCallbackGroup()

        for car_name in self.coches:
            self.declare_parameter(f"cars.{car_name}.carril", "1")
            self.declare_parameter(f"cars.{car_name}.modo_manual", False)
            es_manual = self.get_parameter(f"cars.{car_name}.modo_manual").value
            if es_manual:
                continue

            carril_str = str(self.get_parameter(f"cars.{car_name}.carril").value)
            self.carril_de[car_name] = self._parse_carril(carril_str)
            self.calibrando[car_name] = True

            # Suscripcion de calibracion POR COCHE: /<coche>/modo_calibracion
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

        # Arrancamos cada carril AUTONOMO a la velocidad de calibración (los
        # carriles de coches manuales no se tocan: los mueve la persona).
        for car_name, carril in self.carril_de.items():
            self.arduino.set_rail_speed(carril, self.calibration_speed)

    @staticmethod
    def _parse_carril(carril_str):
        # Acepta "r2" o "2" (misma limpieza que pwm_callback) -> entero 2
        limpio = carril_str.strip().lower().replace("'", "").replace('"', "")
        return int(limpio.replace("r", ""))

    def callback_control_calibracion(self, car_name, msg):
        # Aviso de calibracion de UN coche autonomo (/<car_name>/modo_calibracion).
        if msg.data == False and self.calibrando.get(car_name, False):
            # El coche cerro su vuelta: dejamos de forzar calibration_speed en su
            # carril; el PWM de carrera lo tomara en cuanto llegue por pwm_callback.
            self.calibrando[car_name] = False
        elif msg.data == True and not self.calibrando.get(car_name, True):
            # Reinicio de calibracion de este coche: su carril vuelve a calibration_speed
            self.calibrando[car_name] = True
            self.arduino.set_rail_speed(self.carril_de[car_name], self.calibration_speed)

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

        # Inicializamos dinámicamente el carril en la memoria del puente si no existía
        if rail_num not in self.rails:
            self.rails[rail_num] = -1

        # Filtro antispam para no saturar al Arduino mandando repetidamente el mismo valor
        if pwm_value == self.rails[rail_num]:
            return

        self.rails[rail_num] = pwm_value

        # Validación de seguridad y envío físico
        if 0 <= pwm_value <= 255:
            self.get_logger().info(f"Arduino OK -> Carril {rail_num} a PWM {pwm_value}")
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
