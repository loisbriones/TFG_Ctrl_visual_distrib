import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from image_processor_pkg.msg import ObjectLocation, PathAndSectors
import time

from algoritmo_velocidad import AlgoritmoVelocidad 

class CarControllerNode(Node):
    def __init__(self):
        super().__init__('car_controller')
        
        # --- Configuración del Algoritmo ---
        self.declare_parameter('car_name', 'rbi_car_01')
        self.car_name = self.get_parameter('car_name').value
        self.algo = AlgoritmoVelocidad(self.car_name)

        # --- Estado de Referencia (Mapa) ---
        self.sectores_geom = []  # Lista de segmentos ((x1,y1), (x2,y2))
        self.map_ready = False
        self.current_sec_idx = 0
        self.first_cross_done = False
        
        # --- Buffer de Marcadores ---
        # Guardamos la última posición de cada color para procesarlos juntos
        self.last_positions = {"front": None, "back": None}
        self.last_cross_time = 0.0
        self.debounce_time = 1.0 # Evita contar la misma línea varias veces

        # --- Suscriptores y Publicadores ---
        from rclpy.qos import QoSProfile, DurabilityPolicy
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Recibe el mapa una sola vez al inicio 
        self.create_subscription(PathAndSectors, '/path_and_sectors', self._map_cb, map_qos)
        # Recibe las detecciones constantes 
        self.create_subscription(ObjectLocation, '/object_position', self._pos_cb, 10)

        self.declare_parameter("detection.target_color_1", "rojo")
        self.declare_parameter("detection.target_color_2", "verde")

        self.target_color_1 = self.get_parameter("detection.target_color_1").value
        self.target_color_2 = self.get_parameter("detection.target_color_2").value 

        # Publica el PWM para los motores
        self.pwm_pub = self.create_publisher(Int32, '/car_pwm', 10)

        self.get_logger().info(f"Controlador de {self.car_name} iniciado y esperando mapa...")

    def _map_cb(self, msg: PathAndSectors):
        """Inicializa la trayectoria y los sectores en el algoritmo."""
        self.get_logger().info("Mapa recibido. Configurando geometría...")
        
        # Convertimos la trayectoria frontal para el algoritmo
        trayectoria = [(0, (p.x, p.y)) for p in msg.front]
        self.algo.set_trayectoria(trayectoria)
        
        # Guardamos los sectores como segmentos geométricos
        self.sectores_geom = [((s.start.x, s.start.y), (s.end.x, s.end.y)) for s in msg.sectores]
        self.map_ready = True

    def _pos_cb(self, msg: ObjectLocation):
        """Sincroniza los dos marcadores y ejecuta un paso de control."""
        if not self.map_ready:
            return

        # Determinamos si el color recibido es frontal o trasero basándonos en tu params.yaml
        # Por ejemplo: Verde = Frontal, Azul = Trasero
        if msg.color == self.target_color_1:
            self.last_positions["front"] = (float(msg.x), float(msg.y))
        elif msg.color == self.target_color_2:
            self.last_positions["back"] = (float(msg.x), float(msg.y))

        # Solo actuamos cuando tenemos ambos marcadores del mismo instante
        if self.last_positions["front"] and self.last_positions["back"]:
            self._execute_control_cycle()
            # Limpiamos el buffer para el siguiente frame
            self.last_positions = {"front": None, "back": None}

    def _execute_control_cycle(self):
        """Lógica principal: detección de cruce y cálculo de PWM."""
        now = time.time()
        p_front = self.last_positions["front"]
        p_back = self.last_positions["back"]

        # 1. Comprobar cruce de líneas usando la lógica geométrica de camera.py
        if self.sectores_geom:
            target_idx = self.current_sec_idx % len(self.sectores_geom)
            linea = self.sectores_geom[target_idx]
            
            if self._crosses_segment(p_front, p_back, linea[0], linea[1]):
                if (now - self.last_cross_time) > self.debounce_time:
                    self.last_cross_time = now
                    
                    if target_idx == 0: # Asumimos que el sector 0 es la Meta
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

        # 2. Calcular PWM usando el marcador trasero (back) como dicta tu race_controller
        pwm_val = self.algo.on_frame(
            frame=0, # El frame se puede trackear con un contador interno
            current_section=self.current_sec_idx % len(self.sectores_geom),
            cam_id=0,
            pos=p_back
        )

        # 3. Publicar el resultado
        self.pwm_pub.publish(Int32(data=int(pwm_val)))

    def _crosses_segment(self, p1, p2, A, B):
        """Cálculo de intersección basado en orientación (determinantes)."""
        def orient(a, b, c):
            v = (b[1]-a[1])*(c[0]-b[0]) - (b[0]-a[0])*(c[1]-b[1])
            return 0 if abs(v) < 1e-6 else (1 if v > 0 else 2)

        def on_seg(a, b, c):
            return (min(a[0],c[0]) <= b[0] <= max(a[0],c[0]) and 
                    min(a[1],c[1]) <= b[1] <= max(a[1],c[1]))

        o1, o2, o3, o4 = orient(p1,p2,A), orient(p1,p2,B), orient(A,B,p1), orient(A,B,p2)
        if o1 != o2 and o3 != o4: return True
        if o1 == 0 and on_seg(p1, A, p2): return True
        if o2 == 0 and on_seg(p1, B, p2): return True
        if o3 == 0 and on_seg(A, p1, B): return True
        if o4 == 0 and on_seg(A, p2, B): return True
        return False

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CarControllerNode())
    rclpy.shutdown()