#!/usr/bin/python3

import rclpy
from rclpy.node import Node

# --- Suscriptores y Publicadores ---
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
from std_msgs.msg import Bool

from image_processor_pkg.msg import ObjectLocation, SpeedCarril 

import time
import math

from algoritmo_velocidad import AlgoritmoVelocidad


class CarControllerNode(Node):
    def __init__(self):
        super().__init__("car_controller")
 
        # Subscriber para recibir la posicion del coche_i
        self.sub_car_position = self.create_subscription(ObjectLocation, "position" ,self.control ,qos_profile_sensor_data)
        # Publisher para enviar el pwd del coche_i
        self.rail = self.create_publisher(SpeedCarril, "pwd" ,qos_profile_sensor_data)
                
        # --- CALIBRACION ----
        # TOPIC para controlar el modo de calibracion
        self.modo_calibracion = True
        self.sub_modo_calibracion = self.create_subscription(Bool,"/modo_calibracion", self.callback_control,10)
        
        # --- VELOCIDAD MAX Y MIN ---
        # MIN
        self.declare_parameter("minimum_speed", 50)
        self.v_min = self.get_parameter("minimum_speed").value
        # MAX
        self.declare_parameter("maximum_speed", 80)
        self.v_max = self.get_parameter("maximum_speed").value
 
        self.get_logger().info(f"Controlador de {self.car_name} iniciado y esperando mapa...")
         
         
    def callback_control(self, msg):
        # Modo operacion
        if msg.data == True and self.en_calibracion:
            self.en_calibracion = False
            self.get_logger().info("¡Calibración finalizada! Cambiando a MODO TRABAJO.")
        # Modo calibracion
        elif msg.data == False:
            self.en_calibracion = True
            self.get_logger().warn("Reiniciando calibración...")

    # Funcion donde se realiza el control de los coches
    def control(self, msg:ObjectLocation):
        return

    def _pos_cb(self, msg: ObjectLocation):
        """Extrae marcadores usando la jerarquía msg.stiker.center.x/y"""
        if not self.map_ready:
            return

        # Ajuste de acceso: msg.front/back (Stiker) -> center (Point2D) -> x/y (int32)
        self.last_positions["front"] = (
            float(msg.front.center.x), 
            float(msg.front.center.y)
        )
        self.last_positions["back"] = (
            float(msg.back.center.x), 
            float(msg.back.center.y)
        )

        self._execute_control_cycle()
        self.last_positions = {"front": None, "back": None}

    def _execute_control_cycle(self):
        """Lógica principal: detección de cruce y cálculo de PWM."""
        now = time.time()
        p_front = self.last_positions["front"]
        p_back = self.last_positions["back"]

        if self.sectores_geom:
            target_idx = self.current_sec_idx % len(self.sectores_geom)
            linea = self.sectores_geom[target_idx]

            if self._crosses_segment(p_front, p_back, linea[0], linea[1]):
                if (now - self.last_cross_time) > self.debounce_time:
                    self.last_cross_time = now

                    if target_idx == 0: 
                        if not self.first_cross_done:
                            self.first_cross_done = True
                            self.get_logger().info("¡Carrera Iniciada!")
                        else:
                            self.algo.on_lap_end()
                            self.get_logger().info("Vuelta completada.")
                    else:
                        self.algo.on_section_end(target_idx)
                        self.get_logger().info(f"Sector {target_idx} superado.")

                    self.current_sec_idx += 1

        pwm_val = self.algo.on_frame(
            frame=0,
            current_section=self.current_sec_idx % len(self.sectores_geom),
            cam_id=0,
            pos=p_back,
        )

        self.pwm_pub.publish(Int32(data=int(pwm_val)))

    def _crosses_segment(self, p1, p2, A, B):
        """Cálculo de intersección basado en orientación (determinantes)."""

        def orient(a, b, c):
            v = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
            return 0 if abs(v) < 1e-6 else (1 if v > 0 else 2)

        def on_seg(a, b, c):
            return min(a[0], c[0]) <= b[0] <= max(a[0], c[0]) and min(a[1], c[1]) <= b[
                1
            ] <= max(a[1], c[1])

        o1, o2, o3, o4 = (
            orient(p1, p2, A),
            orient(p1, p2, B),
            orient(A, B, p1),
            orient(A, B, p2),
        )
        if o1 != o2 and o3 != o4:
            return True
        if o1 == 0 and on_seg(p1, A, p2):
            return True
        if o2 == 0 and on_seg(p1, B, p2):
            return True
        if o3 == 0 and on_seg(A, p1, B):
            return True
        if o4 == 0 and on_seg(A, p2, B):
            return True
        return False

    def simplificar_path(self, path, umbral_distancia=2.0):
        """
        Reduce el número de puntos basándose en la distancia mínima.
        """
        if not path:
            return []
    
        path_simplificado = [path[0]]  # Siempre empezamos con el primer punto
        
        for i in range(1, len(path)):
            ultimo_punto = path_simplificado[-1]
            punto_actual = path[i]
            
            # Calculamos distancia euclídea
            dist = math.sqrt((punto_actual.x - ultimo_punto.x)**2 + 
                             (punto_actual.y - ultimo_punto.y)**2)
            
            # Solo lo añadimos si se ha movido lo suficiente
            if dist > umbral_distancia:
                path_simplificado.append(punto_actual)
                
        return path_simplificado


def main(args=None):
    rclpy.init(args=args)
    node = CarControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()