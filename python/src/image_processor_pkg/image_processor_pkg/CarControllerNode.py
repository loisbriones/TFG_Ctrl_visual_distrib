#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from std_msgs.msg import Bool, Empty
from image_processor_pkg.msg import (
    CarLocation,
    SpeedCarril,
    FinishLine,
    TimePerLap,
    CarControlTelemetry,
)

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import math
import numpy as np
import time
import json

# Importamos la clase de Mario
from AlgoritmoVelocidad import MarioAlgorithm

QOS_FINISH_LINE = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
)


class CarControllerNode(Node):
    def __init__(self):
        super().__init__("car_controller")

        self.car_position_group = MutuallyExclusiveCallbackGroup()
        self.heartbeat_car_controller_group = MutuallyExclusiveCallbackGroup()

        self.declare_parameter("car_name", "carPruebas")
        self.car_name = self.get_parameter("car_name").value

        self.declare_parameter("is_primary", True)
        self.is_primary = self.get_parameter("is_primary").value

        self.respawned_node = False
        self.ultimo_latido_recibido = self.get_clock().now()

        if self.is_primary:
            self.sub_heartbeat = self.create_subscription(
                Empty,
                "heartbeat",
                self.callback_heartbeat,
                qos_profile_sensor_data,
                callback_group=self.heartbeat_car_controller_group,
            )
            time.sleep(1)
            if self.respawned_node:
                self.is_primary = False
            else:
                self.destroy_subscription(self.sub_heartbeat)
                self.sub_heartbeat = None
                self.pub_heartbeat = self.create_publisher(
                    Empty,
                    "heartbeat",
                    qos_profile_sensor_data,
                    callback_group=self.heartbeat_car_controller_group,
                )
        else:
            time.sleep(1)
            self.sub_heartbeat = self.create_subscription(
                Empty,
                "heartbeat",
                self.callback_heartbeat,
                qos_profile_sensor_data,
                callback_group=self.heartbeat_car_controller_group,
            )

        if self.is_primary:
            self.timer_publicar_latido = self.create_timer(
                0.1,
                self.publicar_heartbeat,
                callback_group=self.heartbeat_car_controller_group,
            )
        else:
            self.timer_comprobar_failover = self.create_timer(
                0.5,
                self.comprobar_failover,
                callback_group=self.heartbeat_car_controller_group,
            )

        self.declare_parameter("controller.distancia_nodos_trayectoria", 15.0)
        self.umbral_distancia = self.get_parameter(
            "controller.distancia_nodos_trayectoria"
        ).value

        self.declare_parameter("controller.minimum_speed", 55)
        self.v_min = float(self.get_parameter("controller.minimum_speed").value)

        self.declare_parameter("controller.maximum_speed", 85)
        self.v_max = float(self.get_parameter("controller.maximum_speed").value)

        self.declare_parameter("carril_asignado", "1")
        self.carril = self.get_parameter("carril_asignado").value

        self.en_calibracion = True
        self.v_actual = self.v_max
        self.ultimo_pwm_enviado = 0.0

        self.puntos_crudos = {}
        self.trayectoria_base = {}
        self.algoritmos = {}
        self.vueltas = 0
        self.frame_count = 0

        self.finish_line = {"camara_id": None, "coordenadas": None}
        self.tiempo_ultima_vuelta = None
        self.declare_parameter("controller.debounce_meta", 1.0)
        self.debounce_meta = self.get_parameter("controller.debounce_meta").value

        self.sub_car_position = self.create_subscription(
            CarLocation,
            "position",
            self.callback_posicion,
            qos_profile_sensor_data,
            callback_group=self.car_position_group,
        )
        self.sub_modo_calibracion = self.create_subscription(
            Bool,
            "/modo_calibracion",
            self.callback_control_calibracion,
            10,
            callback_group=self.car_position_group,
        )
        self.sub_finish_line_position = self.create_subscription(
            FinishLine,
            "/finish_line_position",
            self.callback_get_finish_line_position,
            QOS_FINISH_LINE,
            callback_group=self.car_position_group,
        )
        self.pub_pwm = self.create_publisher(
            SpeedCarril,
            "pwd",
            qos_profile_sensor_data,
            callback_group=self.car_position_group,
        )

        url_publiser_time_per_lap = f"/telemetria/{self.car_name}/time_per_lap"
        self.pub_time_per_lap = self.create_publisher(
            TimePerLap,
            url_publiser_time_per_lap,
            qos_profile_sensor_data,
            callback_group=self.car_position_group,
        )

        # Avisar de que se acabo el modo calibracion
        self.pub_modo_calibracion = self.create_publisher(
            Bool,
            "/modo_calibracion",
            10,
            callback_group=self.car_position_group,
        )

        url_publiser_car_control_telemetry = f"/telemetria/{self.car_name}/car_control"
        self.pub_car_control_telemetry = self.create_publisher(
            CarControlTelemetry,
            url_publiser_car_control_telemetry,
            qos_profile_sensor_data,
            callback_group=self.car_position_group,
        )

        # --- RUTA DE GUARDADO DE TRAYECTORIAS ---
        self.cache_file = "/ros2_ws/src/image_processor_pkg/cache_trayectoria_controller.json"

        self.get_logger().info("🏁 Controlador iniciado. MODO CALIBRACIÓN ACTIVO.")

    def callback_heartbeat(self, msg):
        if not self.is_primary:
            self.ultimo_latido_recibido = self.get_clock().now()
        else:
            self.respawned_node = True

    def publicar_heartbeat(self):
        self.pub_heartbeat.publish(Empty())

    def comprobar_failover(self):
        tiempo_sin_latido = (
            self.get_clock().now() - self.ultimo_latido_recibido
        ).nanoseconds / 1e9
        if tiempo_sin_latido > 5.0:
            self.get_logger().error(
                "¡Líder caído! Asumiendo el control como PRIMARY 👑"
            )
            self.is_primary = True
            self.destroy_timer(self.timer_comprobar_failover)
            self.destroy_subscription(self.sub_heartbeat)
            self.pub_heartbeat = self.create_publisher(
                Empty, "heartbeat", qos_profile_sensor_data
            )
            self.timer_publicar_latido = self.create_timer(0.1, self.publicar_heartbeat)

    def callback_get_finish_line_position(self, msg):
        self.finish_line["camara_id"] = msg.camara_id
        f_s_x = msg.finish_line.start.x
        f_s_y = msg.finish_line.start.y
        f_e_x = msg.finish_line.end.x
        f_e_y = msg.finish_line.end.y
        self.finish_line["coordenadas"] = ((f_s_x, f_s_y), (f_e_x, f_e_y))

        self.get_logger().info("SE RECIBIO LA LINEA DE META :)")

        self.get_logger().info("COORDENADAS INICIO:")
        inicio = f"X:{self.finish_line['coordenadas'][0][0]}, Y:{self.finish_line['coordenadas'][0][1]}"
        self.get_logger().info(inicio)

        self.get_logger().info("COORDENADAS FIN:")
        fin = f"X:{self.finish_line['coordenadas'][1][0]}, Y:{self.finish_line['coordenadas'][1][1]}"
        self.get_logger().info(fin)

    def callback_control_calibracion(self, msg):
        if msg.data == False and self.en_calibracion:
            self.en_calibracion = False
            self.procesar_trayectorias()
        elif msg.data == True and not self.en_calibracion:
            self.en_calibracion = True
            self.puntos_crudos.clear()
            self.trayectoria_base.clear()
            self.algoritmos.clear()
            self.vueltas = 0
            self.v_actual = 0.0
            self.publicar_velocidad(0)
            self.get_logger().warn("⚠️ Reiniciando calibración.")

    def callback_posicion(self, msg: CarLocation):
        self.frame_count += 1
        if self.en_calibracion:
            self.recolectar_datos_calibracion(msg)
        else:
            if self.is_primary:
                self.ejecutar_control_carrera(msg)

    def recolectar_datos_calibracion(self, msg: CarLocation):
        camara = msg.camara_id

        fx, fy = float(msg.front.center.x), float(msg.front.center.y)

        if fx == 0 and fy == 0:
            return

        bx, by = float(msg.back.center.x), float(msg.back.center.y)

        punto_front = np.array([fx, fy], dtype=np.float32)

        if camara not in self.puntos_crudos:
            self.puntos_crudos[camara] = []

        self.puntos_crudos[camara].append((fx, fy))

        punto_back = np.array([bx, by], dtype=np.float32)
        # Como ya dimos una vuelta podemos terminar la calibración
        if self.verificar_linea_meta(camara, punto_front, punto_back):
            msg = Bool()
            msg.data = False
            self.pub_modo_calibracion.publish(msg)
            self.get_logger().info(
                "¡Flag activado! Se ha publicado: True en /modo_calibracion"
            )


    def procesar_trayectorias(self):
        datos_a_guardar = {}  # 1. Creamos el diccionario para el JSON

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

            self.trayectoria_base[camara] = np.array(ruta_limpia, dtype=np.float32)

            # 2. Convertimos el array de NumPy a lista de Python para poder serializarlo
            datos_a_guardar[camara] = self.trayectoria_base[camara].tolist()

            # Instanciamos a Mario para esta cámara
            self.algoritmos[camara] = MarioAlgorithm(
                self.v_max, self.v_min, self.get_name(), camara
            )
            self.algoritmos[camara].setTrayectoria(self.trayectoria_base[camara])

            self.get_logger().info(
                f"✅ {camara}: Ruta base con {len(ruta_limpia)} nodos. Algoritmo Mario inyectado."
            )

        # 3. Guardamos en disco la trayectoria de puntos por cámara
        try:
            with open(self.cache_file, "w") as f:
                # Usamos indent=4 para que el JSON quede formateado y sea fácil de leer por humanos
                json.dump(datos_a_guardar, f, indent=4)
            self.get_logger().info(f"💾 Trayectoria del controlador guardada en {self.cache_file}")
        except Exception as e:
            self.get_logger().error(f"❌ Error guardando caché del controlador: {e}")

        self.get_logger().info("🚗 ¡Mapa mental listo! Pasando a MODO CARRERA.")

    def ejecutar_control_carrera(self, msg: CarLocation):
        time_received_from_camera = self.get_clock().now()

        camara = msg.camara_id

        if camara not in self.algoritmos:
            return

        fx, fy = float(msg.front.center.x), float(msg.front.center.y)
        bx, by = float(msg.back.center.x), float(msg.back.center.y)

        if (fx == 0 and fy == 0) or (bx == 0 and by == 0):
            return

        punto_front = np.array([fx, fy], dtype=np.float32)
        punto_back = np.array([bx, by], dtype=np.float32)

        self.verificar_linea_meta(camara, punto_front, punto_back)

        # 🏎️ MAGIA: MarioAlgorithm se encarga ahora de detectar el derrape y pedir la velocidad
        nueva_vel = self.algoritmos[camara].actualizar_estado(punto_front, punto_back, self.frame_count, self.vueltas)

        # Si el algoritmo nos dice que ignoremos el frame por ruido, nueva_vel será None
        if nueva_vel is not None:
            self.v_actual = nueva_vel

        # Publicamos siempre para mantener el Heartbeat vivo
        self.publicar_velocidad(int(self.v_actual))

        time_pipeline_finish = self.get_clock().now()

        # Crear mensaje para la telemetria
        msg_car_control_telemetry = CarControlTelemetry()
        msg_car_control_telemetry.receive_msg_stamp = time_received_from_camera.to_msg()
        msg_car_control_telemetry.pipeline_time = (
            time_pipeline_finish - time_received_from_camera
        ).nanoseconds / 1e9
        msg_car_control_telemetry.dist_derrape = float(
            self.algoritmos[camara].dist_derrape
        )
        msg_car_control_telemetry.estado_derrapando = self.algoritmos[
            camara
        ].estado_derrapando

        self.pub_car_control_telemetry.publish(msg_car_control_telemetry)

        # Log solo si cambia para no saturar la terminal
        if nueva_vel is not None and nueva_vel != self.ultimo_pwm_enviado:
            estado_str = (
                "FRENANDO (Curva)" if nueva_vel == self.v_min else "ACELERANDO (Recta)"
            )
            self.get_logger().info(
                f"🚦 MARIO ACTUANDO: {estado_str} | PWM: {self.v_actual}"
            )
            self.ultimo_pwm_enviado = self.v_actual

    def distancia_punto_segmento(self, P, A, B):
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
        msg_vel.carril = self.carril
        msg_vel.stamp = self.get_clock().now().to_msg()
        self.pub_pwm.publish(msg_vel)

    def publicar_time_lap(self, lap_time, lap_number):
        msg_time_per_lap = TimePerLap()
        msg_time_per_lap.lap_time = lap_time
        msg_time_per_lap.lap_number = lap_number
        msg_time_per_lap.stamp = self.get_clock().now().to_msg()
        self.pub_time_per_lap.publish(msg_time_per_lap)

    def crosses_segment(self, p1, p2, A, B):
        thr = 30.0
        d1 = self.distancia_punto_segmento(p1, A, B)
        d2 = self.distancia_punto_segmento(p2, A, B)
        if d1 > thr and d2 > thr:
            return False

        def orientation(a, b, c):
            val = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
            if abs(val) < 1e-6:
                return 0
            return 1 if val > 0 else 2

        def on_segment(a, b, c):
            return min(a[0], c[0]) <= b[0] <= max(a[0], c[0]) and min(a[1], c[1]) <= b[
                1
            ] <= max(a[1], c[1])

        o1 = orientation(p1, p2, A)
        o2 = orientation(p1, p2, B)
        o3 = orientation(A, B, p1)
        o4 = orientation(A, B, p2)

        if o1 != o2 and o3 != o4:
            return True
        if o1 == 0 and on_segment(p1, A, p2):
            return True
        if o2 == 0 and on_segment(p1, B, p2):
            return True
        if o3 == 0 and on_segment(A, p1, B):
            return True
        if o4 == 0 and on_segment(A, p2, B):
            return True

        return False

    def verificar_linea_meta(self, camara_id, p_front, p_back):
        if (
            self.finish_line["camara_id"] is None
            or camara_id != self.finish_line["camara_id"]
        ):
            return False

        A, B = self.finish_line["coordenadas"]
        A_np = np.array(A, dtype=np.float32)
        B_np = np.array(B, dtype=np.float32)

        esta_cruzando = self.crosses_segment(p_front, p_back, A_np, B_np)

        if esta_cruzando:
            ahora = self.get_clock().now()

            # Devolvemos False porque es la primera vez que cruza
            if self.tiempo_ultima_vuelta is None:
                self.tiempo_ultima_vuelta = ahora
                self.get_logger().info(
                    "🏁 Primera pasada por meta. Iniciando cronómetro..."
                )
                return False

            diferencia_segundos = (ahora - self.tiempo_ultima_vuelta).nanoseconds / 1e9

            # Devolvemos True porque es la segunda vez que cruza ahora empezo la carrera
            if diferencia_segundos > self.debounce_meta:
                self.vueltas += 1
                self.publicar_time_lap(diferencia_segundos, self.vueltas)
                self.get_logger().info(
                    f"⏱️ ¡VUELTA {self.vueltas} COMPLETADA! Tiempo: {diferencia_segundos:.3f} s"
                )
                self.tiempo_ultima_vuelta = ahora

                return True

        return False


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
