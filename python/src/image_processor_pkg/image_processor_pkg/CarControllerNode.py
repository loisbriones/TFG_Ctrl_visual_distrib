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
from std_msgs.msg import Bool
from image_processor_pkg.msg import (
    CarLocation,
    SpeedCarril,
    FinishLine,
    TimePerLap,
    CarControlTelemetry,
)

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time

import math
import numpy as np
import json

# Importamos el algoritmo de velocidad por perfil de PWM
from AlgoritmoVelocidad import EstrategiaPerfil

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

        self.declare_parameter("car_name", "carPruebas")
        self.car_name = self.get_parameter("car_name").value

        self.declare_parameter("controller.distancia_nodos_trayectoria", 15.0)
        self.umbral_distancia = self.get_parameter("controller.distancia_nodos_trayectoria").value

        self.declare_parameter("controller.minimum_speed", 55)
        self.v_min = float(self.get_parameter("controller.minimum_speed").value)

        self.declare_parameter("controller.maximum_speed", 85)
        self.v_max = float(self.get_parameter("controller.maximum_speed").value)

        self.declare_parameter("carril_asignado", "2")
        self.carril = self.get_parameter("carril_asignado").value

        self.en_calibracion = True
        self.v_actual = self.v_min
        self.ultimo_pwm_enviado = 0.0

        self.puntos_crudos = {}
        self.trayectoria_base = {}
        self.algoritmos = {}
        self.vueltas = 0
        self.frame_count = 0
        self.derrapes_ultima_vuelta = 0

        self.finish_line = {"camara_id": None, "coordenadas": None}
        self.tiempo_ultima_vuelta = None
        self.declare_parameter("controller.debounce_meta", 1.0)
        self.debounce_meta = self.get_parameter("controller.debounce_meta").value

        # --- INTERPOLACIÓN DEL INSTANTE DE PASO POR META ---
        # A ~30 Hz el cruce real ocurre ENTRE dos frames: asignarle el stamp
        # del frame que lo detecta mete hasta ~33 ms de error por vuelta.
        # Igual que hacía Mario en su TFG, interpolamos linealmente el
        # instante exacto entre el frame anterior (delantera aún sin cruzar)
        # y el actual, proporcionalmente a la distancia perpendicular de
        # cada uno a la recta de meta. Los tiempos son stamps de CAPTURA de
        # la cámara que ve la meta: como el tiempo de vuelta es la
        # diferencia de dos stamps de la MISMA Raspberry, el desfase de
        # reloj entre máquinas se cancela (no hace falta NTP).
        #
        # (punto_front, stamp_ns) del mensaje ANTERIOR de la cámara de meta
        self.front_meta_anterior = None
        # Instante interpolado (ns, reloj de la Raspberry de meta) del
        # último cruce; la diferencia entre dos de estos es el lap_time
        self.t_meta_anterior = None

        # --- ORDEN SECUENCIAL DE CÁMARAS ---
        # El coche recorre las cámaras siempre en el mismo orden (se va por
        # delante y entra por detrás). Estas estructuras lo aprenden en
        # carrera para poder reenviar reducciones a la cámara PRECEDENTE
        # cuando una zona de derrape se sale por el inicio de una trayectoria.
        #
        # Cámara cuyo último frame válido manda ahora mismo (regla pegajosa:
        # se mantiene mientras siga entregando frames válidos, aunque otra
        # cámara solapada intercale mensajes C1,C2,C1,C2)
        self.camara_activa = None
        # Instante (reloj del controlador, no stamps de las Raspberry: los
        # relojes entre máquinas pueden estar desviados) del último frame
        # válido de la cámara activa
        self.t_ultimo_frame_valido = None
        # Segundos sin frames válidos de la activa para darla por perdida y
        # conmutar a la que sí está entregando (~10 frames a 30 Hz)
        self.timeout_camara_activa = 0.3
        # {cámara: cámara anterior en el orden de paso}: se rellena en cada
        # conmutación y tras una vuelta completa queda el ciclo entero
        self.camara_precedente = {}

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

    def callback_get_finish_line_position(self, msg):
        self.finish_line["camara_id"] = msg.camara_id
        f_s_x = msg.finish_line.start.x
        f_s_y = msg.finish_line.start.y
        f_e_x = msg.finish_line.end.x
        f_e_y = msg.finish_line.end.y
        self.finish_line["coordenadas"] = ((f_s_x, f_s_y), (f_e_x, f_e_y))

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
            self.derrapes_ultima_vuelta = 0
            self.v_actual = 0.0
            # El orden de cámaras se reaprende con la nueva calibración
            self.camara_activa = None
            self.t_ultimo_frame_valido = None
            self.camara_precedente.clear()
            # El cronómetro de meta también parte de cero
            self.front_meta_anterior = None
            self.t_meta_anterior = None
            self.tiempo_ultima_vuelta = None
            self.publicar_velocidad(0)
            self.get_logger().warn("⚠️ Reiniciando calibración.")


    def callback_posicion(self, msg: CarLocation):
        self.frame_count += 1

        if self.en_calibracion:
            self.recolectar_datos_calibracion(msg)
        else:
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
        if self.verificar_linea_meta(camara, punto_front, punto_back, msg.stamp):
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

            # Instanciamos el algoritmo de perfil para esta cámara.
            # varias_camaras desambigua dentro de setTrayectoria el caso
            # "el final conecta con el inicio y hay un salto grande": con una
            # sola cámara es el circuito completo con un hueco tapado; con
            # varias, la grabación empezó en mitad de la porción visible
            self.algoritmos[camara] = EstrategiaPerfil(
                self.v_max, self.v_min, self.get_name(), camara
            )
            self.algoritmos[camara].setTrayectoria(
                self.trayectoria_base[camara],
                varias_camaras=len(self.puntos_crudos) > 1,
            )

            self.get_logger().info(
                f"✅ {camara}: Ruta base con {len(ruta_limpia)} nodos. Algoritmo de perfil inyectado."
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

        if self.verificar_linea_meta(camara, punto_front, punto_back, msg.stamp):
            self.registrar_vuelta_algoritmos()

        # 🏎️ MAGIA: el algoritmo se encarga de detectar el derrape y pedir la velocidad
        nueva_vel = self.algoritmos[camara].actualizar_estado(punto_front, punto_back, self.frame_count, self.vueltas)

        # Si el algoritmo nos dice que ignoremos el frame por ruido, nueva_vel será None
        if nueva_vel is not None:
            self.v_actual = nueva_vel

        # --- Seguimiento de la cámara activa (orden secuencial) ---
        # Regla pegajosa: nos quedamos con la cámara que estamos escuchando
        # mientras siga entregando frames válidos; solo cuando lleva
        # timeout_camara_activa sin entregar y OTRA cámara sí entrega,
        # conmutamos. Así el intercalado C1,C2,C1,C2 de dos cámaras
        # solapadas no ensucia el orden aprendido.
        if self.algoritmos[camara].frame_valido:
            if self.camara_activa is None:
                self.camara_activa = camara
                self.t_ultimo_frame_valido = time_received_from_camera
                self.get_logger().info(f"👁️ Cámara activa inicial: {camara}")
            elif camara == self.camara_activa:
                self.t_ultimo_frame_valido = time_received_from_camera
            else:
                sin_activa = (
                    time_received_from_camera - self.t_ultimo_frame_valido
                ).nanoseconds / 1e9
                if sin_activa > self.timeout_camara_activa:
                    # La activa dejó de ver el coche: conmutamos. Se cierra
                    # cualquier derrape que tuviera abierto (no van a llegar
                    # más frames que lo extiendan) y se apunta el orden
                    self.algoritmos[self.camara_activa].notificar_perdida_vision(
                        self.vueltas, self.frame_count
                    )
                    self.camara_precedente[camara] = self.camara_activa
                    self.get_logger().info(
                        f"👁️ Cámara activa: {self.camara_activa} -> {camara} "
                        f"(precedente de {camara} = {self.camara_activa})"
                    )
                    self.camara_activa = camara
                    self.t_ultimo_frame_valido = time_received_from_camera

        # --- Derrame de reducciones hacia la cámara precedente ---
        # Si al retroceder una zona el algoritmo se salió por el inicio de
        # su trayectoria, los px sobrantes se aplican al FINAL de la
        # trayectoria de la cámara anterior en el orden de paso
        pendiente = self.algoritmos[camara].consumir_reduccion_pendiente()
        if pendiente > 0.0:
            precedente = self.camara_precedente.get(camara)
            if precedente is not None and precedente in self.algoritmos:
                self.get_logger().info(
                    f"↩️ Derrame: {camara} pide reducir {pendiente:.0f} px; "
                    f"se aplican al final de la trayectoria de {precedente}"
                )
                self.algoritmos[precedente].aplicar_reduccion_externa(
                    pendiente, self.vueltas
                )
            else:
                self.get_logger().warn(
                    f"↩️ Derrame de {pendiente:.0f} px de {camara} SIN aplicar: "
                    f"aún no se conoce su cámara precedente"
                )

        # Publicamos siempre la velocidad actual
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
            self.get_logger().info(
                f"🚦 PERFIL ACTUANDO: PWM: {self.v_actual} (perfil {self.v_min}-{self.v_max})"
            )
            self.ultimo_pwm_enviado = self.v_actual

    def registrar_vuelta_algoritmos(self):
        # Una vuelta es limpia si NINGUNA camara registro derrapes en ella:
        # solo entonces el perfil puede subir
        derrapes_totales = sum(a.derrapes_contador for a in self.algoritmos.values())
        vuelta_limpia = derrapes_totales == self.derrapes_ultima_vuelta
        self.derrapes_ultima_vuelta = derrapes_totales
        for algoritmo in self.algoritmos.values():
            algoritmo.registrar_vuelta(self.vueltas, vuelta_limpia)
        if vuelta_limpia:
            self.get_logger().info("📈 Vuelta limpia: el perfil de PWM sube.")

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
        msg_vel.carril = "2"
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

    def interpolar_instante_meta(self, p_curr, t_curr, A, B):
        # Instante exacto (ns) en el que la delantera cruzó la recta de meta,
        # interpolando linealmente entre el frame anterior (front_meta_anterior)
        # y el actual. Se llama SOLO en el frame en el que el cruce cuenta
        # (primer cruce o vuelta que pasa el debounce): la detección ya la
        # hizo crosses_segment; aquí solo se reparte el tiempo.
        if self.front_meta_anterior is None:
            self.get_logger().warn(
                "🏁 Cruce de meta sin frame anterior guardado: "
                "se usa el stamp del frame actual sin interpolar"
            )
            return t_curr
        p_prev, t_prev = self.front_meta_anterior

        # d(P) = cross(B-A, P-A) / |B-A| es la distancia perpendicular CON
        # SIGNO a la recta (el signo dice de qué lado está P). Es la fórmula
        # del TFG de Mario (getTiempoVuelta) pero con distancia perpendicular
        # real en vez de la aproximación por el eje dominante de la meta.
        # Producto cruzado 2D escrito a mano (np.cross con vectores 2D está
        # retirado en NumPy 2.x)
        AB = B - A
        norma = float(np.linalg.norm(AB))
        AP_prev = p_prev - A
        AP_curr = p_curr - A
        d_prev = float(AB[0] * AP_prev[1] - AB[1] * AP_prev[0]) / norma
        d_curr = float(AB[0] * AP_curr[1] - AB[1] * AP_curr[0]) / norma

        if d_prev * d_curr >= 0:
            # Guard de VALIDEZ, no re-detección: interpolar entre t_prev y
            # t_curr solo tiene sentido si el cruce cayó dentro de ese
            # intervalo, es decir, si la delantera cambió de lado entre esos
            # dos frames. Caso raro NO soportado (p. ej. se perdió el frame
            # justo antes del cruce y el anterior ya estaba pasado): se usa
            # el stamp de captura del frame actual tal cual, que sigue siendo
            # mejor que el reloj del controlador.
            self.get_logger().warn(
                "🏁 Interpolación de meta sin cambio de lado "
                f"(d_prev={d_prev:.1f}, d_curr={d_curr:.1f}): "
                "se usa el stamp del frame actual sin interpolar"
            )
            return t_curr

        # Fracción del intervalo entre frames recorrida hasta tocar la recta
        s = abs(d_prev) / (abs(d_prev) + abs(d_curr))
        return t_prev + s * (t_curr - t_prev)

    def verificar_linea_meta(self, camara_id, p_front, p_back, stamp):
        if (
            self.finish_line["camara_id"] is None
            or camara_id != self.finish_line["camara_id"]
        ):
            return False

        # Stamp de CAPTURA del frame (reloj de la Raspberry de meta), en ns
        t_stamp = Time.from_msg(stamp).nanoseconds

        A, B = self.finish_line["coordenadas"]
        A_np = np.array(A, dtype=np.float32)
        B_np = np.array(B, dtype=np.float32)

        esta_cruzando = self.crosses_segment(p_front, p_back, A_np, B_np)

        vuelta_completada = False

        if esta_cruzando:
            ahora = self.get_clock().now()

            # OJO: la interpolación se calcula SOLO dentro de las dos ramas
            # que consumen el instante (primer cruce y vuelta que pasa el
            # debounce). El coche está a caballo de la meta varios frames
            # seguidos y en los frames que el debounce descarta la delantera
            # ya está pasada de línea: interpolar ahí no tendría sentido.

            # Devolvemos False porque es la primera vez que cruza
            if self.tiempo_ultima_vuelta is None:
                self.tiempo_ultima_vuelta = ahora
                self.t_meta_anterior = self.interpolar_instante_meta(
                    p_front, t_stamp, A_np, B_np
                )
                self.get_logger().info(
                    "🏁 Primera pasada por meta. Iniciando cronómetro..."
                )
            else:
                # El debounce sigue con el reloj del controlador (mide "hace
                # cuánto detectamos el cruce anterior", no necesita precisión)
                diferencia_segundos = (
                    ahora - self.tiempo_ultima_vuelta
                ).nanoseconds / 1e9

                # Devolvemos True porque es la segunda vez que cruza ahora empezo la carrera
                if diferencia_segundos > self.debounce_meta:
                    self.vueltas += 1
                    # El tiempo de vuelta publicado es la diferencia entre
                    # los dos instantes INTERPOLADOS de paso por meta
                    t_meta = self.interpolar_instante_meta(
                        p_front, t_stamp, A_np, B_np
                    )
                    lap_time = (t_meta - self.t_meta_anterior) / 1e9
                    self.publicar_time_lap(lap_time, self.vueltas)
                    self.get_logger().info(
                        f"⏱️ ¡VUELTA {self.vueltas} COMPLETADA! Tiempo: {lap_time:.3f} s"
                    )
                    self.tiempo_ultima_vuelta = ahora
                    self.t_meta_anterior = t_meta

                    vuelta_completada = True

        # Guardamos SIEMPRE el último punto/stamp de la cámara de meta: en el
        # próximo cruce será el punto "pre-meta" de la interpolación (el
        # equivalente a trayectoria[-1] en el código de Mario)
        self.front_meta_anterior = (p_front.copy(), t_stamp)

        return vuelta_completada


def main(args=None):
    rclpy.init(args=args)
    node = CarControllerNode()

    executor = MultiThreadedExecutor(num_threads=2)
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
