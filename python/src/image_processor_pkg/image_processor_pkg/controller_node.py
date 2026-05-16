#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from image_processor_pkg.msg import CarLocation, SpeedCarril

import math
import time


class CarControllerNode(Node):
    def __init__(self):
        super().__init__("car_controller")

        # --- PARÁMETROS DEL MAPA BASE ---
        self.declare_parameter("controller.distancia_nodos_trayectoria", 15.0)
        self.umbral_distancia = self.get_parameter(
            "controller.distancia_nodos_trayectoria"
        ).value

        # --- PARÁMETROS DE CONTROL ---
        self.declare_parameter("controller.minimum_speed", 55)
        self.declare_parameter("controller.maximum_speed", 85)
        self.declare_parameter("controller.umbral_derrape", 25.0)
        self.declare_parameter("carril_asignado", "1")

        self.v_min = float(self.get_parameter("controller.minimum_speed").value)
        self.v_max = float(self.get_parameter("controller.maximum_speed").value)
        self.umbral_derrape_peligro = float(
            self.get_parameter("controller.umbral_derrape").value
        )
        self.umbral_derrape_seguro = 12.0
        self.carril = self.get_parameter("carril_asignado").value

        # --- ESTADO DEL SISTEMA ---
        self.en_calibracion = True
        self.v_actual = self.v_min
        self.ultimo_pwm_enviado = 0.0  # <-- CLAVE PARA EL FILTRO ANTISPAM

        # --- MAPA MENTAL Y APRENDIZAJE ---
        self.puntos_crudos = {}
        self.trayectoria_base = {}
        self.perfil_velocidad = {}
        self.ultima_camara_confiable = None

        # --- SUSCRIPTORES Y PUBLICADORES ---
        self.sub_car_position = self.create_subscription(
            CarLocation, "position", self.callback_posicion, qos_profile_sensor_data
        )
        self.sub_modo_calibracion = self.create_subscription(
            Bool, "/modo_calibracion", self.callback_control_calibracion, 10
        )
        self.pub_pwm = self.create_publisher(
            SpeedCarril, "pwd", qos_profile_sensor_data
        )

        self.get_logger().info("🏁 Controlador iniciado. MODO CALIBRACIÓN ACTIVO.")

    def callback_control_calibracion(self, msg):
        if msg.data == False and self.en_calibracion:
            self.en_calibracion = False
            self.procesar_trayectorias()

        elif msg.data == True and not self.en_calibracion:
            self.en_calibracion = True
            self.puntos_crudos.clear()
            self.trayectoria_base.clear()
            self.perfil_velocidad.clear()
            self.v_actual = 0.0
            self.publicar_velocidad(0)
            self.get_logger().warn("⚠️ Reiniciando calibración.")

    def callback_posicion(self, msg: CarLocation):
        if self.en_calibracion:
            self.recolectar_datos_calibracion(msg)
        else:
            self.ejecutar_control_carrera(msg)

    # ---------------------------------------------------------
    # FASE 1: CALIBRACIÓN
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

            self.trayectoria_base[camara] = ruta_limpia

            # MAGIA: Cada nodo nace con la velocidad mínima (aseguramos no salirnos en la primera vuelta)
            self.perfil_velocidad[camara] = [self.v_min] * len(ruta_limpia)

            self.get_logger().info(
                f"✅ {camara}: Ruta base con {len(ruta_limpia)} nodos."
            )
        self.get_logger().info("🚗 ¡Mapa mental listo! Pasando a MODO CARRERA.")

    # ---------------------------------------------------------
    # FASE 2: MODO OPERACIÓN Y APRENDIZAJE
    # ---------------------------------------------------------
    # ---------------------------------------------------------
    # FASE 2: MODO OPERACIÓN Y APRENDIZAJE
    # ---------------------------------------------------------
    # ---------------------------------------------------------
    # FASE 2: MODO OPERACIÓN Y APRENDIZAJE
    # ---------------------------------------------------------

    # ---------------------------------------------------------
    # FASE 2: MODO OPERACIÓN Y APRENDIZAJE
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

        if fx == 0 and fy == 0:
            return

        trayectoria = self.trayectoria_base[camara]
        num_nodos = len(trayectoria)

        idx_actual, dist_a_ruta = self.obtener_nodo_mas_cercano((fx, fy), trayectoria)

        if dist_a_ruta > 60.0:
            return

        self.ultima_camara_confiable = camara
        dist_derrape = 0.0

        if bx != 0 and by != 0:
            idx_trasero, _ = self.obtener_nodo_mas_cercano((bx, by), trayectoria)

            p_centro = trayectoria[idx_trasero]
            p_siguiente = trayectoria[(idx_trasero + 1) % num_nodos]
            p_anterior = trayectoria[(idx_trasero - 1) % num_nodos]

            d1 = self.distancia_punto_segmento((bx, by), p_anterior, p_centro)
            d2 = self.distancia_punto_segmento((bx, by), p_centro, p_siguiente)
            dist_derrape = min(d1, d2)

            # --- APRENDIZAJE: REESCRITURA SUAVE ---
            if dist_derrape > self.umbral_derrape_peligro:
                # DERRAPE REAL: Marcamos el nodo actual y los 15 anteriores como peligrosos (velocidad mínima)
                for i in range(idx_actual - 15, idx_actual + 2):
                    self.perfil_velocidad[camara][i % num_nodos] = self.v_min

            elif dist_derrape < self.umbral_derrape_seguro:
                # ZONA ESTABLE: Aceleramos el bloque de 15 nodos que tenemos por delante
                for i in range(idx_actual, idx_actual + 15):
                    idx_mod = i % num_nodos
                    self.perfil_velocidad[camara][idx_mod] = min(
                        self.v_max, self.perfil_velocidad[camara][idx_mod] + 1.0
                    )

        # PASO 3: ANTICIPACIÓN INTELIGENTE
        # Escaneamos los próximos 15 nodos y cogemos la velocidad MÁS BAJA de ese tramo.
        # Así, si a 10 nodos de distancia hay una curva peligrosa, frena ya. Si todo es recta, sube.
        v_objetivo = self.v_max
        for i in range(1, 16):
            v_nodo = self.perfil_velocidad[camara][(idx_actual + i) % num_nodos]
            if v_nodo < v_objetivo:
                v_objetivo = v_nodo

        # RAMPA SUAVE
        if v_objetivo > self.v_actual:
            self.v_actual = min(v_objetivo, self.v_actual + 1.5)
        elif v_objetivo < self.v_actual:
            self.v_actual = max(v_objetivo, self.v_actual - 4.0)

        # --- PROTECCIÓN DEL ARDUINO (ANTISPAM) ---
        # Solo publicamos al topic si hay un cambio de al menos 3.0 puntos en el PWM
        if (
            abs(self.v_actual - self.ultimo_pwm_enviado) >= 3.0
            or self.v_actual == self.v_min
            or self.v_actual == self.v_max
        ):
            self.publicar_velocidad(int(self.v_actual))
            self.ultimo_pwm_enviado = self.v_actual

        if not hasattr(self, "debug_counter"):
            self.debug_counter = 0
        self.debug_counter += 1

        if self.debug_counter % 15 == 0:
            estado = "ZONA MUERTA"
            if dist_derrape > self.umbral_derrape_peligro:
                estado = "PELIGRO (Frenando)"
            elif dist_derrape < self.umbral_derrape_seguro:
                estado = "SEGURO (Acelerando)"

            self.get_logger().info(
                f"Dist. Trasera: {dist_derrape:.2f}px | {estado} | Vel. Obj (Tramo): {v_objetivo:.1f} | Vel. Real: {self.v_actual:.1f}"
            )

    # ---------------------------------------------------------
    # FUNCIONES MATEMÁTICAS
    # ---------------------------------------------------------
    def obtener_nodo_mas_cercano(self, punto, trayectoria):
        """Devuelve el índice del nodo y su distancia al punto"""
        distancias = [
            (idx, math.hypot(px - punto[0], py - punto[1]))
            for idx, (px, py) in enumerate(trayectoria)
        ]
        mejor_nodo = min(distancias, key=lambda t: t[1])
        return mejor_nodo[0], mejor_nodo[1]

    def distancia_punto_segmento(self, P, A, B):
        vx, vy = B[0] - A[0], B[1] - A[1]
        wx, wy = P[0] - A[0], P[1] - A[1]
        l2 = vx**2 + vy**2
        if l2 == 0:
            return math.hypot(P[0] - A[0], P[1] - A[1])

        t = max(0, min(1, (wx * vx + wy * vy) / l2))
        px_cercano, py_cercano = A[0] + t * vx, A[1] + t * vy

        return math.hypot(P[0] - px_cercano, P[1] - py_cercano)

    def publicar_velocidad(self, pwm):
        msg_vel = SpeedCarril()
        msg_vel.pwm = pwm
        msg_vel.carril = "2"
        self.pub_pwm.publish(msg_vel)


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
