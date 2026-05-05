#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# --- Suscriptores y Publicadores ---
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from image_processor_pkg.msg import ObjectLocation, PathAndSectors

import time

from algoritmo_velocidad import AlgoritmoVelocidad


class CarControllerNode(Node):
    def __init__(self):
        super().__init__("car_controller")

        # --- Configuración del Algoritmo ---
        self.declare_parameter("car_name", "rbi_car_01")
        self.car_name = self.get_parameter("car_name").value
        self.algo = AlgoritmoVelocidad(self.car_name)

        # --- Estado de Referencia (Mapa) ---
        self.sectores_geom = []  
        self.map_ready = False
        self.current_sec_idx = 0
        self.first_cross_done = False

        # --- Buffer de Marcadores ---
        self.last_positions = {"front": None, "back": None}
        self.last_cross_time = 0.0
        self.debounce_time = 1.0 

        map_qos = QoSProfile(
            depth=1, 
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            PathAndSectors, "/rpi5/path_and_sectors", self._map_cb, map_qos
        )
        
        # Suscripción al ObjectLocation que ahora contiene Stiker y Point2D
        self.create_subscription(
            ObjectLocation, "/object_position", self._pos_cb, qos_profile_sensor_data
        )

        self.pwm_pub = self.create_publisher(Int32, "/car_pwm", 10)

        self.get_logger().info(
            f"Controlador de {self.car_name} iniciado y esperando mapa..."
        )

    def _map_cb(self, msg: PathAndSectors):
        """Inicializa la trayectoria y los sectores en el algoritmo."""
        self.get_logger().info("Mapa recibido. Configurando geometría...")
        trayectoria = [(0, (p.x, p.y)) for p in msg.front]
        self.algo.set_trayectoria(trayectoria)

        self.sectores_geom = [
            ((s.start.x, s.start.y), (s.end.x, s.end.y)) for s in msg.sectores
        ]
        self.map_ready = True

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