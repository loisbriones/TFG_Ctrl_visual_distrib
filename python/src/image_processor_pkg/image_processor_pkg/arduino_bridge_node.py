#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from image_processor_pkg.msg import SpeedCarril
from std_msgs.msg import Int32


from arduino_controller import ArduinoController


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__("arduino_bridge")
        
        # --- Obtener numero de coches --- 
        # Creamos tantos carriles como coches tengamos
        self.declare_parameter("num_coches",1)
        num_coches = self.get_parameter("num_coche").value

        # Diccionario para matener controlado la velocidad que ponemos a cada carril
        self.rails = {}
        
        for i in range(num_coches):
            self.subscription = self.create_subscription(SpeedCarril, f"/carril_coche{i}", self.pwm_callback, 10)
            self.rails[f"r{i}"] = 0

        # --- PARAMETROS ---
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)

        port = self.get_parameter("port").value
        baud = self.get_parameter("baudrate").value

        #  --- CALIBRACION ---
        self.declare_parameter("calibration_speed", 55)
        self.calibration_speed = self.get_parameter("calibration_speed").value

        # Nos conectamos al arduino
        self.arduino = ArduinoController(port=port, baudrate=baud)

        self.get_logger().info(f"Conectando a Arduino en {port}...") 

        self.arduino.set_both_rails(self.calib_speed, self.calib_speed)

    def pwm_callback(self, msg: Int32):
        """
        Cada vez que llega un nuevo valor de PWM, lo enviamos al Arduino.
        """

        pwm_value = msg.pwm
        rail = msg.carril

        # Si el valor no cambia del anterior recibido no lo enviamos al arduino para no saturar
        if pwm_value == self.rails[rail]:
            return

        self.rails[rail] = pwm_value

        # Validacion de seguridad
        if 0 <= pwm_value <= 255:

            self.arduino.set_rail_speed(self.rail_id, pwm_value)
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
