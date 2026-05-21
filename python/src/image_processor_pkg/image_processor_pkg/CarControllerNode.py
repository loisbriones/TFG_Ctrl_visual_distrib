#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool,Empty
from image_processor_pkg.msg import CarLocation, SpeedCarril,FinishLine

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import math
import numpy as np
import time

QOS_FINISH_LINE = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST
)

class CarControllerNode(Node):
    def __init__(self):
        super().__init__("car_controller")
        
        # --- CALLBACK GROUPS ---
        # Poner un hilo a procesar las posiciones de las camaras y poner otro hilo a procesar la red de heartbeat  
        # Procesar las posiciones de los coches
        self.car_position_group = MutuallyExclusiveCallbackGroup()
        # Controlar la red de heartbeat
        self.heartbeat_car_controller_group = MutuallyExclusiveCallbackGroup()

        # --- CONFIGURACION HA ---
        self.declare_parameter("is_primary",True)
        self.is_primary = self.get_parameter("is_primary").value  
        
        self.respawned_node = False 
        self.ultimo_latido_recibido = self.get_clock().now() 

        if self.is_primary:
            # Comprobar si el nodo primary no hice respawn
            self.sub_heartbeat = self.create_subscription(Empty, "heartbeat", self.callback_heartbeat, qos_profile_sensor_data,callback_group=self.heartbeat_car_controller_group)
            time.sleep(1)
            # Si hizo respawn entonces tiene que ser el subscriptor
            if self.respawned_node:
                self.get_logger().info(f"OJO -> NO DETECTA A NADIE Y AUN ASI SE PONE OPERATIVO: {self.respawned_node}")
                self.is_primary = False
            # No hay nadie publicando nada entonces no hicimos respawn somos los primary
            else:     
                self.destroy_subscription(self.sub_heartbeat)
                self.sub_heartbeat = None
                self.pub_heartbeat = self.create_publisher(Empty, "heartbeat", qos_profile_sensor_data,callback_group=self.heartbeat_car_controller_group) 
        else:
            time.sleep(1)
            # El nodo pasivo siempre que arranque por respawn o no significa que tiene que ser pasivo
            self.sub_heartbeat = self.create_subscription(Empty,"heartbeat", self.callback_heartbeat ,qos_profile_sensor_data, callback_group=self.heartbeat_car_controller_group)
            
        # TIMERS PARA EL CONTROL DEL HEARTHBEAT
        if self.is_primary:
            self.timer_publicar_latido = self.create_timer(0.1, self.publicar_heartbeat,callback_group=self.heartbeat_car_controller_group)
        else:
            self.timer_comprobar_failover = self.create_timer(0.5, self.comprobar_failover,callback_group=self.heartbeat_car_controller_group)

        # --- PARÁMETROS DEL MAPA BASE ---
        self.declare_parameter("controller.distancia_nodos_trayectoria", 15.0)
        self.umbral_distancia = self.get_parameter("controller.distancia_nodos_trayectoria").value

        # --- PARÁMETROS DE CONTROL ---
        self.declare_parameter("controller.minimum_speed", 55)
        self.v_min = float(self.get_parameter("controller.minimum_speed").value)

        self.declare_parameter("controller.maximum_speed", 85)
        self.v_max = float(self.get_parameter("controller.maximum_speed").value)

        self.declare_parameter("controller.umbral_derrape_peligro", 25.0)
        self.umbral_derrape_peligro = float(self.get_parameter("controller.umbral_derrape_peligro").value)
        
        self.declare_parameter("controller.umbral_derrape_seguro",12.0)
        self.umbral_derrape_seguro = float(self.get_parameter("controller.umbral_derrape_seguro").value)

        self.declare_parameter("controller.umbral_control_trayectoria_correcta",60.0)
        self.umbral_control_trayectoria_correcta = float(self.get_parameter("controller.umbral_control_trayectoria_correcta").value)

        self.declare_parameter("controller.latencia_min_nodos", 1)
        self.declare_parameter("controller.latencia_max_nodos", 4)

        self.latencia_min = self.get_parameter("controller.latencia_min_nodos").value
        self.latencia_max = self.get_parameter("controller.latencia_max_nodos").value
            
        self.declare_parameter("carril_asignado", "1")
        self.carril = self.get_parameter("carril_asignado").value

        # --- ESTADO DEL SISTEMA ---
        self.en_calibracion = True
        self.v_actual = self.v_min
        self.ultimo_pwm_enviado = 0.0

        # --- MAPA MENTAL Y APRENDIZAJE ---
        self.puntos_crudos = {}
        self.trayectoria_base = {}
        self.perfil_velocidad = {}
        self.limite_velocidad = {}
        self.ultima_camara_confiable = None 

        # --- FINISH LINE ---
        # Va a ser diccionario donde vas a tener de que camara viene y que posicion tiene le objeto
        self.finish_line = {} 

        # --- CONTROL DE TIEMPOS (VUELTAS) ---
        self.tiempo_ultima_vuelta = None

        self.declare_parameter("controller.debounce_meta", 2.0)
        self.debounce_meta = self.get_parameter("controller.debounce_meta").value  

        # --- SUSCRIPTORES Y PUBLICADORES ---

        # Procesar la peticion del coche
        self.sub_car_position = self.create_subscription(CarLocation, "position", self.callback_posicion, qos_profile_sensor_data, callback_group=self.car_position_group)

        # Estar pendiente de si hay que cambiar a modo operacion o a modo calibracion
        self.sub_modo_calibracion = self.create_subscription(Bool, "/modo_calibracion", self.callback_control_calibracion, 10, callback_group=self.car_position_group)
        
        # Recibir la posicion de la linea de meta
        self.sub_finish_line_position = self.create_subscription(FinishLine, "/finish_line_position", self.callback_get_finish_line_position, QOS_FINISH_LINE, callback_group=self.car_position_group)

        # Enviar el PWD al nodo RaceController 
        self.pub_pwm = self.create_publisher(SpeedCarril, "pwd", qos_profile_sensor_data, callback_group=self.car_position_group)
        
        self.get_logger().info("🏁 Controlador iniciado. MODO CALIBRACIÓN ACTIVO.")

    def callback_heartbeat(self, msg):
        """Si soy el sombra, reseteo el cronómetro al escuchar al líder."""
        if not self.is_primary:
            self.ultimo_latido_recibido = self.get_clock().now()
        else:
            self.respawned_node = True

    # Si eres el lider entonces publicas el heartbeat
    def publicar_heartbeat(self):
        self.pub_heartbeat.publish(Empty())

    # Pasivo compruba que sigue recibiendo los latidos 
    def comprobar_failover(self):
        tiempo_sin_latido = (self.get_clock().now() - self.ultimo_latido_recibido).nanoseconds / 1e9
        
        self.get_logger().info(f"Tiempo sin latido: {tiempo_sin_latido}")
        
        if tiempo_sin_latido > 5.0:
            self.get_logger().error("¡Líder caído! Asumiendo el control como PRIMARY 👑")
            self.is_primary = True

            self.destroy_timer(self.timer_comprobar_failover)
            self.destroy_subscription(self.sub_heartbeat)
            self.pub_heartbeat = self.create_publisher(Empty, "heartbeat", qos_profile_sensor_data)

            self.timer_publicar_latido = self.create_timer(0.1, self.publicar_heartbeat)
    
    def callback_get_finish_line_position(self,msg):      

        self.finish_line["camara_id"] = msg.camara_id
        f_s_x = msg.finish_line.start.x 
        f_s_y = msg.finish_line.start.y
        
        f_e_x = msg.finish_line.end.x 
        f_e_y = msg.finish_line.end.y 

        self.finish_line["coordenadas"] = ((f_s_x,f_s_y),(f_e_x,f_e_y))

    def callback_control_calibracion(self, msg):
        if msg.data == False and self.en_calibracion:
            self.en_calibracion = False
            self.procesar_trayectorias()

        elif msg.data == True and not self.en_calibracion:
            self.en_calibracion = True
            self.puntos_crudos.clear()
            self.trayectoria_base.clear()
            self.perfil_velocidad.clear()
            self.limite_velocidad.clear()
            self.v_actual = 0.0
            self.publicar_velocidad(0)
            self.get_logger().warn("⚠️ Reiniciando calibración.")

    def callback_posicion(self, msg: CarLocation):
        if self.en_calibracion:
            self.recolectar_datos_calibracion(msg)
        else:
            if self.is_primary:
                self.ejecutar_control_carrera(msg)

    # ---------------------------------------------------------
    # MODO CALIBRACION
    # ---------------------------------------------------------
    def recolectar_datos_calibracion(self, msg: CarLocation):
        camara = msg.camara_id
        x, y = float(msg.front.center.x), float(msg.front.center.y)

        if x == 0 and y == 0:
            return

        if camara not in self.puntos_crudos:
            self.puntos_crudos[camara] = []

        self.puntos_crudos[camara].append((x, y))

    def procesar_trayectorias(self):
        for camara, puntos in self.puntos_crudos.items():
            if not puntos:
                continue

            ruta_limpia = [puntos[0]]

            for i in range(1, len(puntos)):
                ult_p = ruta_limpia[-1]
                p_act = puntos[i]
                if (
                    math.hypot(p_act[0] - ult_p[0], p_act[1] - ult_p[1])
                    > self.umbral_distancia
                ):
                    ruta_limpia.append(p_act)

            # Pasamos la lista a numpy, de manera que almacenamos las rutas como matrices (N, 2)
            self.trayectoria_base[camara] = np.array(ruta_limpia, dtype=np.float32)

            # Perfil de velocidad como array unidimensional de NumPy
            self.perfil_velocidad[camara] = np.full(
                len(ruta_limpia), self.v_min, dtype=np.float32
            )

            self.limite_velocidad[camara] = np.full(
                len(ruta_limpia), self.v_max, dtype=np.float32
            )

            self.get_logger().info(
                f"✅ {camara}: Ruta base con {len(ruta_limpia)} nodos vectorizados."
            )
        self.get_logger().info("🚗 ¡Mapa mental listo! Pasando a MODO CARRERA.")

    # ---------------------------------------------------------
    # MODO OPERACION Y APRENDIZAJE
    # ---------------------------------------------------------
    def ejecutar_control_carrera(self, msg: CarLocation):
        camara = msg.camara_id

        if (
            camara not in self.trayectoria_base
            or len(self.trayectoria_base[camara]) < 2
        ):
            return

        fx, fy = float(msg.front.center.x), float(msg.front.center.y)

        bx, by = float(msg.back.center.x), float(msg.back.center.y)

        if (fx == 0 and fy == 0) or (bx == 0 and by == 0):
            return

        trayectoria = self.trayectoria_base[camara]
        num_nodos = len(trayectoria)

        punto_front = np.array([fx, fy], dtype=np.float32)
        punto_back = np.array([bx,by], dtype=np.float32)

        # Comprobamos que no hayamos cruzado la linea de meta
        self.verificar_linea_meta(camara, punto_front, punto_back)

        idx_actual, dist_a_ruta = self.obtener_nodo_mas_cercano(punto_front, trayectoria)

        if dist_a_ruta > self.umbral_control_trayectoria_correcta :
            return

        self.ultima_camara_confiable = camara
        dist_derrape = 0.0

        if bx != 0 and by != 0:
            punto_back = np.array([bx, by], dtype=np.float32)
            idx_trasero, _ = self.obtener_nodo_mas_cercano(punto_back, trayectoria)

            # Punto actual del path donde nos encontramos
            p_centro = trayectoria[idx_trasero]
            # Punto siguiente del path hacia donde vamos
            p_siguiente = trayectoria[(idx_trasero + 1) % num_nodos]
            # Punto anterior en el path de donde venimos
            p_anterior = trayectoria[(idx_trasero - 1) % num_nodos]

            # Para los 3 puntos que tenemos del path base tenemos que comprobar donde se encuentra stiker
            # con respecto a estos puntos para poder luego saber donde esta la posicion del coche

            d1 = self.distancia_punto_segmento(punto_back, p_anterior, p_centro)
            d2 = self.distancia_punto_segmento(punto_back, p_centro, p_siguiente)
            dist_derrape = min(d1, d2)

            # Referencia directa al array de NumPy para mayor velocidad
            perfil = self.perfil_velocidad[camara]
            limite = self.limite_velocidad[camara]

            # =========================================================
            # 🧠 FASE 1: APRENDIZAJE Y DIBUJO DEL MAPA (Vectorizado)
            # =========================================================
            if dist_derrape > self.umbral_derrape_peligro:
                # Derrape detectado
                velocidad_actual_mapa = perfil[idx_actual]
                # Reducimos la velocidad para el punto donde estamos
                nueva_vel_apice = max(self.v_min, velocidad_actual_mapa - 2.0)
                # Actualizamos la velocidad en el punto donde detectamos el derrape
                perfil[idx_actual] = nueva_vel_apice
                limite[idx_actual] = np.minimum(limite[idx_actual], nueva_vel_apice)

                # Modificamos las zonas cercanas para:
                # 1º Antes de llegar al punto critico reducir la velocidad
                # 2º Al salir de la curva aumentar la velocidad

                # Generamos los indices que vamos a usar para actualizar los valores de velocidad
                idx_atras_1_5 = (idx_actual - np.arange(1, 6)) % num_nodos
                idx_atras_6_10 = (idx_actual - np.arange(6, 11)) % num_nodos

                idx_adelante_1_5 = (idx_actual + np.arange(1, 6)) % num_nodos
                idx_adelante_6_10 = (idx_actual + np.arange(6, 11)) % num_nodos

                # Frenada escalonada hacia ATRÁS

                # Hacemos el minimo entre todos los elementos del array
                perfil[idx_atras_1_5] = np.minimum(
                    perfil[idx_atras_1_5], nueva_vel_apice
                )
                limite[idx_atras_1_5] = np.minimum(
                    limite[idx_atras_1_5], nueva_vel_apice
                )

                # Hacemos el minimo entre todos los elementos del array
                perfil[idx_atras_6_10] = np.minimum(
                    perfil[idx_atras_6_10], nueva_vel_apice + 1.0
                )
                limite[idx_atras_6_10] = np.minimum(
                    limite[idx_atras_6_10], nueva_vel_apice + 1.0
                )

                # Tracción escalonada hacia ADELANTE

                # Hacemos el minimo entre todos los elementos del array
                perfil[idx_adelante_1_5] = np.minimum(
                    perfil[idx_adelante_1_5], nueva_vel_apice
                )
                limite[idx_adelante_1_5] = np.minimum(
                    limite[idx_adelante_1_5], nueva_vel_apice
                )

                # Hacemos el minimo entre los elementos del array
                perfil[idx_adelante_6_10] = np.minimum(
                    perfil[idx_adelante_6_10], nueva_vel_apice + 1.0
                )
                limite[idx_adelante_6_10] = np.minimum(
                    limite[idx_adelante_6_10], nueva_vel_apice + 1.0
                )

            elif dist_derrape < self.umbral_derrape_seguro:
                # Aplicamos un incremento el las 16 siguientes posiciones
                idx_seguros = (idx_actual + np.arange(0, 16)) % num_nodos
                # Hacemos el minimo de todos los elementos del array
                perfil[idx_seguros] = np.minimum(
                    limite[idx_seguros], perfil[idx_seguros] + 1.0
                )

        # =========================================================
        # 🏎️ FASE 2: LECTURA DIRECTA DEL MAPA
        # =========================================================
        # 1. Normalizamos la velocidad actual para obtener un porcentaje (0.0 a 1.0)
        velocidad_norm = (self.v_actual - self.v_min) / (self.v_max - self.v_min)
        # Evitamos que el valor se salga de los márgenes por si hay picos
        velocidad_norm = max(0.0, min(1.0, velocidad_norm))

        # 2. Calculamos los nodos flotantes con interpolación lineal
        nodos_latencia_float = self.latencia_min + (velocidad_norm * (self.latencia_max - self.latencia_min))

        # 3. Redondeamos para obtener un índice entero válido para el array
        nodos_latencia = int(round(nodos_latencia_float))

        # 4. Obtenemos la velocidad objetivo mirando 'nodos_latencia' pasos por delante
        v_objetivo = self.perfil_velocidad[camara][(idx_actual + nodos_latencia) % num_nodos]

        self.v_actual = v_objetivo

        # =========================================================
        # 📡 FASE 3: FILTRO ANTISPAM Y PUBLICACIÓN
        # =========================================================
        #
        self.publicar_velocidad(int(self.v_actual))
        self.ultimo_pwm_enviado = self.v_actual

        # if (
        #    abs(self.v_actual - self.ultimo_pwm_enviado) >= 3.0
        #    or self.v_actual == self.v_min
        #    or self.v_actual == self.v_max
        # ):
        #    self.publicar_velocidad(int(self.v_actual))
        #    self.ultimo_pwm_enviado = self.v_actual
        ## LOGS DE DEPURACIÓN

        if not hasattr(self, "debug_counter"):
            self.debug_counter = 0
        self.debug_counter += 1

        if self.debug_counter % 15 == 0:
            estado = "ZONA MUERTA"
            if dist_derrape > self.umbral_derrape_peligro:
                estado = "PELIGRO (Restando vel mapa)"
            elif dist_derrape < self.umbral_derrape_seguro:
                estado = "SEGURO (Sumando vel mapa)"

            self.get_logger().info(
                f"Dist. Trasera: {dist_derrape:.2f}px | {estado} | Vel. Obj: {v_objetivo:.1f} | Vel. Real: {self.v_actual:.1f}"
            )

    def obtener_nodo_mas_cercano(self, punto, trayectoria):
        """Calcula la distancia de 'punto' a toda la matriz 'trayectoria' de golpe"""
        # Restamos el punto a todos los nodos a la vez, elevamos al cuadrado y sumamos.
        # Esto es equivalente a pitágoras pero miles de veces más rápido en C.
        distancias_sq = np.sum((trayectoria - punto) ** 2, axis=1)
        mejor_idx = np.argmin(
            distancias_sq
        )  # Encuentra el índice del valor más pequeño
        return int(mejor_idx), math.sqrt(distancias_sq[mejor_idx])

    def distancia_punto_segmento(self, P, A, B):
        """Cálculo vectorial de la distancia (operaciones numpy directas)"""
        AB = B - A
        AP = P - A

        l2 = np.sum(AB**2)
        if l2 == 0:
            return np.linalg.norm(AP)

        t = max(0.0, min(1.0, np.dot(AP, AB) / l2))
        proyeccion = A + t * AB

        return np.linalg.norm(P - proyeccion)

    def publicar_velocidad(self, pwm):
        msg_vel = SpeedCarril()
        msg_vel.pwm = pwm
        msg_vel.carril = "2"
        self.pub_pwm.publish(msg_vel)
        
    def crosses_segment(self,p1, p2, A, B):
        """
        Devuelve True si el segmento p1→p2 “cruza” al segmento A→B,
        usando un umbral fijo de distancia perpendicular y test de intersección.
        """

        thr = 30.0

        # calcular distancias desde ambos puntos del coche hasta la línea
        d1 = self.distancia_punto_segmento(p1, A, B)
        d2 = self.distancia_punto_segmento(p2, A, B)

        # descartamos si ambos están muy lejos
        if d1 > thr and d2 > thr:
            return False

        # 3) test clásico de intersección de segmentos
        def orientation(a, b, c):
            val = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
            if abs(val) < 1e-6:
                return 0  # colineal
            return 1 if val > 0 else 2  # 1=izquierda, 2=derecha

        def on_segment(a, b, c):
            return (min(a[0], c[0]) <= b[0] <= max(a[0], c[0]) and
                    min(a[1], c[1]) <= b[1] <= max(a[1], c[1]))

        o1 = orientation(p1, p2, A)
        o2 = orientation(p1, p2, B)
        o3 = orientation(A, B, p1)
        o4 = orientation(A, B, p2)

        # caso general
        if o1 != o2 and o3 != o4:
            return True

        # casos colineales en los extremos
        if o1 == 0 and on_segment(p1, A, p2): return True
        if o2 == 0 and on_segment(p1, B, p2): return True
        if o3 == 0 and on_segment(A, p1, B): return True
        if o4 == 0 and on_segment(A, p2, B): return True

        return False 

    def verificar_linea_meta(self, camara_id , p_front, p_back):
        # 1. Si no hay línea definida o no estamos en la cámara correcta, ignorar
        if self.finish_line["camara_id"] is None or camara_id != self.finish_line["camara_id"]:
            return

        # 2. Extraer puntos de la meta
        A, B = self.finish_line["coordenadas"]
        A_np = np.array(A, dtype=np.float32)
        B_np = np.array(B, dtype=np.float32)

        # 3. Comprobar si el coche cruza la línea
        esta_cruzando = self.crosses_segment(p_front, p_back, A_np, B_np)

        if esta_cruzando:
            ahora = self.get_clock().now()

            # 4a. Si es la primera vez que pasa por meta, solo iniciamos el reloj
            if self.tiempo_ultima_vuelta is None:
                self.tiempo_ultima_vuelta = ahora
                self.get_logger().info("🏁 Primera pasada por meta. Iniciando cronómetro...")
                return

            # 4b. Si ya estaba corriendo el tiempo, miramos cuánto ha pasado
            diferencia_segundos = (ahora - self.tiempo_ultima_vuelta).nanoseconds / 1e9

            # Solo cuenta si ha superado el tiempo de "ceguera" (debounce)
            if diferencia_segundos > self.debounce_meta:
                self.get_logger().info(f"⏱️ ¡VUELTA COMPLETADA! Tiempo: {diferencia_segundos:.3f} s")
                self.tiempo_ultima_vuelta = ahora
                
def main(args=None):
    rclpy.init(args=args)
    node = CarControllerNode()
    
    executor = MultiThreadedExecutor(num_threads=3)
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
