#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from arduino_controller import ArduinoController


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__("arduino_bridge")

        # --- Parámetros ---
        # Permitimos configurar el puerto y el raíl desde el lanzamiento o parámetros
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("rail_id", 2)

        port = self.get_parameter("port").value
        baud = self.get_parameter("baudrate").value
        self.rail_id = self.get_parameter("rail_id").value

        # Nos conectamos al arduino
        # self.arduino = ArduinoController(port=port, baudrate=baud)
        self.arduino = ArduinoController()

        self.get_logger().info(f"Conectando a Arduino en {port}...")

        # --- Suscriptor ---
        # Escuchamos el tópico de PWM que viene del controlador
        self.subscription = self.create_subscription(
            Int32, "/car_pwm", self.pwm_callback, 10
        )

        self.get_logger().info(f"Nodo Bridge listo. Controlando Raíl: {self.rail_id}")

    def pwm_callback(self, msg: Int32):
        """
        Cada vez que llega un nuevo valor de PWM, lo enviamos al Arduino.
        """
        pwm_value = msg.data

        self.get_logger().info(f"Recibido en Bridge: {msg.data}")

        # Validacion de seguridad
        if 0 <= pwm_value <= 255:
            self.arduino.set_rail_speed(self.rail_id, pwm_value)
        else:
            self.get_logger().warn(f"Valor de PWM fuera de rango recibido: {pwm_value}")

    def destroy_node(self):
        """
        Al cerrar el nodo, nos aseguramos de parar el coche por seguridad.
        """
        self.get_logger().info("Cerrando conexión. Deteniendo motores...")
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
