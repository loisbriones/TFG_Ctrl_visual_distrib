#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from image_processor_pkg.msg import CarLocation, SpeedCarril

import math
import numpy as np


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
        self.ultimo_pwm_enviado = 0.0

        # --- MAPA MENTAL Y APRENDIZAJE ---
        self.puntos_crudos = {}
        self.trayectoria_base = {}
        self.perfil_velocidad = {}
        self.limite_velocidad = {}
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
            self.limite_velocidad.clear()
            self.v_actual = 0.0
            self.publicar_velocidad(0)
            self.get_logger().warn("⚠️ Reiniciando calibración.")

    def callback_posicion(self, msg: CarLocation):
        if self.en_calibracion:
            self.recolectar_datos_calibracion(msg)
        else:
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

        if fx == 0 and fy == 0:
            return

        bx, by = float(msg.back.center.x), float(msg.back.center.y)

        trayectoria = self.trayectoria_base[camara]
        num_nodos = len(trayectoria)

        # Usar arrays de numpy para cálculos matemáticos
        punto_front = np.array([fx, fy], dtype=np.float32)
        idx_actual, dist_a_ruta = self.obtener_nodo_mas_cercano(
            punto_front, trayectoria
        )

        if dist_a_ruta > 60.0:
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
        nodos_latencia = 1
        v_objetivo = self.perfil_velocidad[camara][
            (idx_actual + nodos_latencia) % num_nodos
        ]
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
