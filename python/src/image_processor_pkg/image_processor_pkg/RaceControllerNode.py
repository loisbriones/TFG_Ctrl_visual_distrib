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
        # Cada cierto tiempo se levanta el timer y se encarga de comprobar cuando hace que recibimos el ultimo mensaje, en caso de superar un limite entonces se encarga de parar el coche porque no estamos recibiendo informacion del controlador y signifca que esta caido por tanto no tiene sentido seguir controlando el coche 
        
        # Cada cuanto tiempo comprobamos si el nodo Controlador esta caido
        self.declare_parameter("hearthbear_timer",0.066)
        self.heartbear_timer = self.get_parameter("hearthbear_timer").value        
        
        # Delta t: periodo que dejamos que pase desde que recibimos un paquete
        self.declare_parameter("delta_t",1)
        self.delta_t = self.get_parameter("delta_t").value
        
        # Timestamp de cuando recibimos el mensaje
        self.msg_timestamp = None 
        self.last_msg_timestamp = None
        # Lock para poder leer la variable de cuanto hace que nos llego un mensaje
        self.check_timer_lock = Lock()
     
        # Diccionario para mantener controlada la velocidad actual de cada carril
        self.rails = {}
        # Lista para no perder la referencia de las suscripciones
        self.sub_pwd = {}
        # Diccionario para guardar los subcripciones de control de heartbeat para los diferentes CarController
        self.sub_heartbeat_check = {}

        # Nos suscribimos al topic "pwd" de CADA coche (ej. /car1/pwd, /car2/pwd)
        for car_name in self.coches:
            topic_name = f"/{car_name}/pwd"
            self.sub_pwd[car_name] = self.create_subscription(SpeedCarril, topic_name, self.pwm_callback, qos_profile_sensor_data, callback_group=MutuallyExclusiveCallbackGroup())

            group = MutuallyExclusiveCallbackGroup()
            self.sub_heartbeat_check[car_name] = {"group": group, "timer": self.create_timer(self.heartbear_timer, self.check_heartbeat,callback_group=group)} 

        
        self.modo_calibracion = True
        self.grupo_calibracion = MutuallyExclusiveCallbackGroup()
        self.sub_modo_calibracion = self.create_subscription(Bool, "/modo_calibracion", self.callback_control_calibracion, 10, callback_group=self.grupo_calibracion)

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
    
    def callback_control_calibracion(self,msg):
        
        if msg.data == False and self.modo_calibracion:
            self.modo_calibracion= False
        elif msg.data == True and not self.modo_calibracion:
            self.modo_calibracion= True      
            # Arrancamos con la velocidad de calibración
            self.arduino.set_both_rails(self.calibration_speed, self.calibration_speed)

    def check_heartbeat(self):
        if not self.modo_calibracion:
            with self.check_timer_lock:
                if self.last_msg_timestamp is not None:
                        if (self.msg_timestamp - self.last_msg_timestamp) > self.delta_t:
                            self.arduino.set_both_rails(1,1)
                        else:
                            self.last_msg_timestamp = self.msg_timestamp
                else:
                    self.last_msg_timestamp = self.msg_timestamp 

    def pwm_callback(self, msg: SpeedCarril):
        """
        Cada vez que llega un nuevo valor de PWM, lo enviamos al Arduino.
        """
        pwm_value = msg.pwm
        
        with self.check_timer_lock:
            self.msg_timestamp = msg.stamp

        # Limpieza agresiva: quitamos comillas (simples y dobles), espacios y pasamos a minúscula
        # Esto soluciona el problema de recibir "'2'", "r2" o " 2"
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

        # Validacion de seguridad y envío físico
        if 0 <= pwm_value <= 255:
            # Añadimos este log para confirmar que la señal sale hacia el cable USB
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
        # CORRECCIÓN: Cambiado 'coches_activos' a 'coches' para que coincida con params.yaml
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
