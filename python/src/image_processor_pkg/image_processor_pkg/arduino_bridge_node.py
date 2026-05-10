#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from image_processor_pkg.msg import SpeedCarril
from rclpy.qos import qos_profile_sensor_data

from arduino_controller import ArduinoController

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
                SpeedCarril, 
                topic_name, 
                self.pwm_callback, 
                qos_profile_sensor_data
            )
            self.subs.append(sub)
            
            # Inicializamos el diccionario con r1, r2, etc. (igual que en el Launch File)
            carril_id = f"r{i+1}"
            self.rails[carril_id] = 0
            self.get_logger().info(f"Escuchando comandos PWM en: {topic_name} (Carril {carril_id})")

        # --- PARAMETROS ---
        self.declare_parameter("arduino.port", "/dev/ttyACM0")
        self.declare_parameter("arduino.baudrate", 115200)
        self.declare_parameter("arduino.calibration_speed", 55)

        port = self.get_parameter("arduino.port").value
        baud = self.get_parameter("arduino.baudrate").value
        self.calibration_speed = self.get_parameter("arduino.calibration_speed").value

        # Nos conectamos al arduino
        self.arduino = ArduinoController(port=port, baudrate=baud)

        self.get_logger().info(f"Conectando a Arduino en {port}...") 

        # BUG FIX: variable corregida
        self.arduino.set_both_rails(self.calibration_speed, self.calibration_speed)

    def pwm_callback(self, msg: SpeedCarril):
        """
        Cada vez que llega un nuevo valor de PWM, lo enviamos al Arduino.
        """
        pwm_value = msg.pwm
        rail_str = msg.carril # Llega como "r1", "r2", etc.

        # Ignoramos si llega un carril que no tenemos registrado
        if rail_str not in self.rails:
            return

        # Si el valor no cambia del anterior recibido no lo enviamos para no saturar
        if pwm_value == self.rails[rail_str]:
            return

        self.rails[rail_str] = pwm_value

        # Validacion de seguridad
        if 0 <= pwm_value <= 255:
            try:
                # Extraemos el número del carril (de "r1" sacamos el 1)
                rail_num = int(rail_str.replace("r", ""))
                self.arduino.set_rail_speed(rail_num, pwm_value)
            except ValueError:
                self.get_logger().error(f"Formato de carril inválido: {rail_str}")

    def destroy_node(self):
        """
        Al cerrar el nodo, nos aseguramos de parar el coche por seguridad.
        """
        self.get_logger().info("Cerrando conexión. Deteniendo motores...")
        if hasattr(self, 'arduino') and self.arduino:
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