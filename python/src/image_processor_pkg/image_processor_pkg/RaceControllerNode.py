#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from image_processor_pkg.msg import SpeedCarril
from rclpy.qos import qos_profile_sensor_data

from ArduinoController import ArduinoController


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__("arduino_bridge")

        # --- Obtener la lista de coches desde el YAML ---
        self.declare_parameter("coches", ["car1"])
        coches = self.get_parameter("coches").value

        # Diccionario para mantener controlada la velocidad actual de cada carril
        self.rails = {}
        # Lista para no perder la referencia de las suscripciones
        self.subs = []

        # Nos suscribimos al topic "pwd" de CADA coche (ej. /car1/pwd, /car2/pwd)
        for i, coche in enumerate(coches):
            topic_name = f"/{coche}/pwd"
            sub = self.create_subscription(
                SpeedCarril, topic_name, self.pwm_callback, qos_profile_sensor_data
            )
            self.subs.append(sub)

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

        # TIMER
        # Cada cierto tiempo se levanta el timer y se encarga de comprobar cuando hace que recibimos el ultimo mensaje, en caso de superar un limite entonces se encarga de parar el coche porque no estamos recibiendo informacion del controlador y signifca que esta caido por tanto no tiene sentido seguir controlando el coche 
        
        # Cada cuanto tiempo comprobamos si el nodo Controlador esta caido
        self.declare_parameter("hearthbear_timer",10)
        self.hearthbear_timer = self.get_parameter("hearthbear_timer").value        
        
        # Delta t: periodo que dejamos que pase desde que recibimos un paquete
        self.declare_parameter("delta_t",1)
        self.delta_t = self.get_parameter("delta_t").value


    def pwm_callback(self, msg: SpeedCarril):
        """
        Cada vez que llega un nuevo valor de PWM, lo enviamos al Arduino.
        """
        pwm_value = msg.pwm

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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
