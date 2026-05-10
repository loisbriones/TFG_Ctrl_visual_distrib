#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from image_processor_pkg.msg import CarLocation, SpeedCarril 

import math

class CarControllerNode(Node):
    def __init__(self):
        super().__init__("car_controller")
        
        # --- PARÁMETROS DEL MAPA BASE ---
        self.declare_parameter("distancia_nodos_trayectoria", 15.0)
        self.umbral_distancia = self.get_parameter("distancia_nodos_trayectoria").value
 
        # --- PARÁMETROS DE CONTROL (Velocidades y Derrape) ---
        self.declare_parameter("minimum_speed", 50)
        self.declare_parameter("maximum_speed", 80)
        self.declare_parameter("umbral_derrape", 8.0) # Píxeles de separación para considerarlo derrape
        self.declare_parameter("carril_asignado", "1") # Para el mensaje SpeedCarril
        
        self.v_min = self.get_parameter("minimum_speed").value
        self.v_max = self.get_parameter("maximum_speed").value
        self.umbral_derrape = self.get_parameter("umbral_derrape").value
        self.carril = self.get_parameter("carril_asignado").value
        
        # --- ESTADO DEL SISTEMA ---
        self.en_calibracion = True
        self.ultimo_pwm_enviado = 0
        
        # --- MAPA MENTAL DEL CIRCUITO ---
        self.puntos_crudos = {} 
        self.trayectoria_base = {} 
        
        # --- DICCIONARIO DE APRENDIZAJE (ZONAS DE PELIGRO) ---
        # Estructura: {"camara_1": [{"inicio": idx, "fin": idx, "anticipacion": num_nodos}]}
        self.zonas_peligro = {} 
        
        # --- SUSCRIPTORES Y PUBLICADORES ---
        self.sub_car_position = self.create_subscription(
            CarLocation, 
            "position", 
            self.callback_posicion, 
            qos_profile_sensor_data
        )
        self.sub_modo_calibracion = self.create_subscription(
            Bool,
            "/modo_calibracion", 
            self.callback_control_calibracion,
            10
        )
        self.pub_pwm = self.create_publisher(SpeedCarril, "pwd", qos_profile_sensor_data)
        
        self.get_logger().info("🏁 Controlador iniciado. MODO CALIBRACIÓN ACTIVO.")

    def callback_control_calibracion(self, msg):
        if msg.data == False and self.en_calibracion:
            self.en_calibracion = False
            self.get_logger().info("🛑 Calibración terminada. Procesando trayectorias base...")
            self.procesar_trayectorias()
            
        elif msg.data == True and not self.en_calibracion:
            self.en_calibracion = True
            self.puntos_crudos.clear()
            self.trayectoria_base.clear()
            self.zonas_peligro.clear()
            self.publicar_velocidad(0) # Frenamos el coche por seguridad
            self.get_logger().warn("⚠️ Reiniciando calibración. Memoria borrada.")

    def callback_posicion(self, msg: CarLocation):
        if self.en_calibracion:
            self.recolectar_datos_calibracion(msg)
        else:
            self.ejecutar_control_carrera(msg)

    # ---------------------------------------------------------
    # FASE 1: CALIBRACIÓN (Lo que ya hicimos)
    # ---------------------------------------------------------
    def recolectar_datos_calibracion(self, msg: CarLocation):
        camara = msg.camara_id
        x, y = float(msg.front.center.x), float(msg.front.center.y)

        if x == 0 and y == 0:
            return

        if camara not in self.puntos_crudos:
            self.puntos_crudos[camara] = []
            self.get_logger().info(f"👀 Nueva cámara en calibración: {camara}")

        self.puntos_crudos[camara].append((x, y))

    def procesar_trayectorias(self):
        for camara, puntos in self.puntos_crudos.items():
            if not puntos: continue

            ruta_limpia = [puntos[0]] 
            for i in range(1, len(puntos)):
                ult_p = ruta_limpia[-1]
                p_act = puntos[i]
                dist = math.hypot(p_act[0] - ult_p[0], p_act[1] - ult_p[1])
                
                if dist > self.umbral_distancia:
                    ruta_limpia.append(p_act)
            
            self.trayectoria_base[camara] = ruta_limpia
            self.zonas_peligro[camara] = [] # Inicializamos la memoria de aprendizaje para esta cámara
            self.get_logger().info(f"✅ {camara}: Ruta base creada con {len(ruta_limpia)} nodos.")
            
        self.get_logger().info("🚗 ¡Mapa mental listo! Pasando a MODO CARRERA.")

    # ---------------------------------------------------------
    # FASE 2: MODO OPERACIÓN Y APRENDIZAJE
    # ---------------------------------------------------------
    def ejecutar_control_carrera(self, msg: CarLocation):
        camara = msg.camara_id
        
        if camara not in self.trayectoria_base or len(self.trayectoria_base[camara]) < 2:
            return
            
        fx, fy = float(msg.front.center.x), float(msg.front.center.y)
        bx, by = float(msg.back.center.x), float(msg.back.center.y)
        
        if fx == 0 and fy == 0:
            return # Hemos perdido el tracker frontal, no podemos hacer nada

        # PASO 1: ENCONTRAR DÓNDE ESTAMOS (Índice más cercano al frontal)
        idx_actual = self.obtener_indice_mas_cercano((fx, fy), self.trayectoria_base[camara])
        
        # Medida de seguridad: Comprobar si nos hemos salido de la pista completamente
        p_ref = self.trayectoria_base[camara][idx_actual]
        if math.hypot(fx - p_ref[0], fy - p_ref[1]) > 50.0:
            self.publicar_velocidad(0) # Freno de emergencia
            return

        # PASO 2: DETECTAR DERRAPE (Usando el trasero)
        if bx != 0 and by != 0:
            # Cogemos el segmento de recta donde se supone que está el coche
            idx_anterior = max(0, idx_actual - 1)
            pA = self.trayectoria_base[camara][idx_anterior]
            pB = self.trayectoria_base[camara][idx_actual]
            
            dist_derrape = self.distancia_punto_segmento((bx, by), pA, pB)
            
            # PASO 3: GESTIONAR ZONAS SI DERRAPA
            if dist_derrape > self.umbral_derrape:
                self.registrar_derrape(camara, idx_actual)

        # PASO 4: CONTROL DE VELOCIDAD ANTICIPADO
        if self.estamos_en_zona_peligro(camara, idx_actual):
            self.publicar_velocidad(self.v_min)
        else:
            self.publicar_velocidad(self.v_max)

    # ---------------------------------------------------------
    # FUNCIONES MATEMÁTICAS Y DE APRENDIZAJE
    # ---------------------------------------------------------
    def registrar_derrape(self, camara, idx):
        """Si el coche derrapa, crea una zona o expande una existente"""
        zonas = self.zonas_peligro[camara]
        
        for zona in zonas:
            # Si el derrape ocurre cerca de una zona existente (margen de 5 nodos por delante o detrás)
            if zona["inicio"] - 5 <= idx <= zona["fin"] + 5:
                # Expandimos los límites del derrape
                zona["inicio"] = min(zona["inicio"], idx)
                zona["fin"] = max(zona["fin"], idx)
                
                # LA MAGIA DEL APRENDIZAJE: Si volvemos a derrapar aquí, incrementamos la anticipación (como hacía tu amigo)
                zona["anticipacion"] = min(zona["anticipacion"] + 1, 15) # Tope máximo de 15 nodos de anticipación
                return

        # Si no encaja en ninguna zona cercana, creamos una nueva
        nueva_zona = {
            "inicio": idx, 
            "fin": idx, 
            "anticipacion": 4 # Empezamos frenando 4 nodos antes por defecto
        }
        zonas.append(nueva_zona)
        self.get_logger().warn(f"💥 ¡Nuevo derrape aprendido en {camara}! Nodo: {idx}")

    def estamos_en_zona_peligro(self, camara, idx_actual):
        """Comprueba si el coche está a punto de entrar en un derrape"""
        zonas = self.zonas_peligro[camara]
        
        for zona in zonas:
            # Si mi índice actual sumado a mi "visión de futuro" cae dentro de la zona, toca frenar
            idx_futuro = idx_actual + zona["anticipacion"]
            
            # ¿Estamos cruzando físicamente la zona o a punto de hacerlo?
            if zona["inicio"] <= idx_futuro and idx_actual <= zona["fin"]:
                return True
                
        return False

    def obtener_indice_mas_cercano(self, punto, trayectoria):
        """Busca rápidamente el nodo más cercano en la lista"""
        # Forma rápida y pitagórica de encontrar el índice más cercano
        distancias = [(idx, (px-punto[0])**2 + (py-punto[1])**2) for idx, (px, py) in enumerate(trayectoria)]
        idx_mas_cercano = min(distancias, key=lambda t: t[1])[0]
        return idx_mas_cercano

    def distancia_punto_segmento(self, P, A, B):
        """Calcula la distancia ortogonal de un punto a un segmento de recta (Adaptación del código de tu amigo)"""
        vx = B[0] - A[0]
        vy = B[1] - A[1]
        wx = P[0] - A[0]
        wy = P[1] - A[1]

        # Módulo del segmento al cuadrado
        l2 = vx**2 + vy**2
        if l2 == 0: 
            return math.hypot(P[0] - A[0], P[1] - A[1])

        # Proyección del punto en el segmento (normalizada entre 0 y 1)
        t = max(0, min(1, (wx*vx + wy*vy) / l2))
        
        # Coordenadas del punto más cercano en el segmento
        px_cercano = A[0] + t * vx
        py_cercano = A[1] + t * vy
        
        return math.hypot(P[0] - px_cercano, P[1] - py_cercano)

    def publicar_velocidad(self, pwm):
        # Para evitar spam a ROS/Arduino, solo publicamos si la velocidad cambia
        if pwm != self.ultimo_pwm_enviado:
            msg_vel = SpeedCarril()
            msg_vel.pwm = int(pwm)
            msg_vel.carril = self.carril
            self.pub_pwm.publish(msg_vel)
            self.ultimo_pwm_enviado = pwm


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
