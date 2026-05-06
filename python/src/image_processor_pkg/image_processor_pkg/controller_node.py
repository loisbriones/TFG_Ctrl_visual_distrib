#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# --- Suscriptores y Publicadores ---
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from image_processor_pkg.msg import ObjectLocation, PathAndSectors

import time
import math

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
 
        # Suscripción al ObjectLocation que ahora contiene Stiker y Point2D
        self.create_subscription(
            ObjectLocation, "/object_position", self._pos_cb, qos_profile_sensor_data
        )

        self.pwm_pub = self.create_publisher(Int32, "/car_pwm", 10)

        self.get_logger().info(
            f"Controlador de {self.car_name} iniciado y esperando mapa..."
        )
        
        # Diccionario que contiene las camaras que se usan en el circuito
        self.camaras = {}

    # --- DESCUBRIR CAMARAS ---
    """ Funcion que permite buscar todos los nodoso que tiene camara y subscribirse a ellos""" 
    def discover_cameras(self):

        map_qos = QoSProfile(
            depth=1, 
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # 1. Obtener todos los tópicos y sus tipos
        topic_info = self.get_topic_names_and_types()
        
        for topic_name, topic_types in topic_info:
            # 2. Filtrar por nombre y tipo de mensaje
            if '/camara' in topic_name and 'PathAndSectors' in topic_types:
                
                # 3. Si no estamos suscritos aún, lo hacemos
                if topic_name not in self.subscribers:
                    self.get_logger().info(f'Nueva cámara detectada: {topic_name}')
                     
                    # Creamos el suscriptor dinámicamente
                    new_sub = self.create_subscription(
                        PathAndSectors,
                        topic_name,
                        # Usamos una función lambda para saber de qué cámara viene el mensaje
                        lambda msg, tn=topic_name: self.camera_callback(msg, tn),
                        map_qos
                    )
                    self.subscribers[topic_name] = new_sub

    # Funcion que se llama cuando las camaras a las que nos suscribimos publican: PathAndSectors.msg
    def camera_callback(self, msg: PathAndSectors, topic_name: str):
        self.get_logger().info(f'Procesando datos de: {topic_name} (ID: {msg.node_id})')
        
        
        # 1. Extraer el ID de la cámara del mensaje
        cam_id = msg.node_id 
    
        # 2. Convertir sectores (LineSegment -> Tuplas de coordenadas)
        # Cada LineSegment tiene un 'start' y un 'end' 
        # Cada punto tiene 'x' e 'y' 
        sectores_formateados = [
            ((s.start.x, s.start.y), (s.end.x, s.end.y)) 
            for s in msg.sectores
        ]

        # 1. Simplificamos las trayectorias (ajusta el umbral según necesites)
        # Un umbral de 2 a 5 suele ser ideal para mapas de este estilo
        front_simplificado = self.simplificar_path(msg.front, umbral_distancia=3.0)
        back_simplificado = self.simplificar_path(msg.back, umbral_distancia=3.0)
     
        # 4. Guardar en el diccionario con la estructura solicitada
        self.camera_data[cam_id] = {
            "sectores": sectores_formateados,
            "front_path": front_simplificado,
            "back_path" : back_simplificado
        }
    
        # Opcional: Si quieres mantener la lógica de "map_ready" o pasar datos al algoritmo
        # como hacías en _map_cb, puedes hacerlo aquí usando los datos procesados:
        # self.algo.set_trayectoria(path_completo) 
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