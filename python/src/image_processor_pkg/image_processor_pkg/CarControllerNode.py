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
    TimePerLap, CarControlTelemetry,
)

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time

import math
import numpy as np
import json

from AlgoritmoVelocidad import EstrategiaPerfil

QOS_FINISH_LINE = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
)

# Separacion maxima, en frames, entre los dos mensajes que compara verificar_linea_meta
MAX_SALTO_FRAMES_META = 10


class CarControllerNode(Node):
    """
    Nodo de ROS2 que decide el PWM que hay que aplicar a un coche. Hay una
    instancia por coche
    """

    def __init__(self):
        super().__init__("car_controller")

        self.car_position_group = MutuallyExclusiveCallbackGroup()

        # --- PARAMETROS DE CONFIGURACION DEL NODO ---
        
        # Nombre del coche que vamos a controlar
        self.declare_parameter("car_name", "carPruebas")
        self.car_name = self.get_parameter("car_name").value

        # Separacion minima entre los puntos de la trayectoria base
        self.declare_parameter("controller.distancia_nodos_trayectoria", 15.0)
        self.umbral_distancia = self.get_parameter("controller.distancia_nodos_trayectoria").value

        # Valor minimo de PWM que vamos a aplicar
        self.declare_parameter("controller.minimum_speed", 55)
        self.v_min = float(self.get_parameter("controller.minimum_speed").value)

        # Valor maximo de PWM que vamos a aplicar
        self.declare_parameter("controller.maximum_speed", 85)
        self.v_max = float(self.get_parameter("controller.maximum_speed").value)

        # Carril asignado al coche
        self.declare_parameter("carril_asignado", "2")
        self.carril = self.get_parameter("carril_asignado").value
        #--------------------------------------------------

        # --- PARAMETROS DE CONFIGURACION DEL ALGORITMO---
        
        # El algoritmo no es un nodo de ROS, hay que pasarle los parametros del YAML
        self.params_algoritmo = {}
        for nombre, defecto in (
            ("umbral_derrape", 12.0),
            ("max_dist_ruta", 80.0),
            ("margen_extremo_celdas", 2),
            ("paso_celda", 15.0),
            ("umbral_celda_gigante", 150.0),
            ("umbral_cierre", 60.0),
            ("incremento_vuelta", 1.0),
            ("reduccion_derrape", 2.0),
            ("retroceso_creacion", 150.0),
            ("retroceso_fusion", 60.0),
            ("margen_fusion_celdas", 1),
            ("vueltas_proteccion", 2),
        ):
            self.declare_parameter(f"controller.algoritmo.{nombre}", defecto)
            self.params_algoritmo[nombre] = self.get_parameter(
                f"controller.algoritmo.{nombre}"
            ).value
        #--------------------------------------------------

        # --- MODO DE OPERACION (quien decide el PWM) ---
        #   manual      -> no se publica PWM, conduce una persona
        #   incremental -> se publica un PWM que no decide el algoritmo
        #   automatico  -> se publica el PWM del algoritmo, es el normal
        #   politica    -> igual, pero con el perfil congelado desde un JSON
        # En los cuatro el algoritmo corre entero

        self.declare_parameter("modo", "automatico")
        self.modo = self.get_parameter("modo").value

        # Flag para controlar el modo manual
        self.modo_manual = (self.modo == "manual")

        # Vueltas que aguanta el modo incremental antes de subir el PWM
        self.declare_parameter("controller.vueltas_incremento", 10)
        self.vueltas_incremento = max(
            1, int(self.get_parameter("controller.vueltas_incremento").value))

        # Cuanto sube el PWM el algoritmo al completar una vuelta
        self.incremento_vuelta = float(self.params_algoritmo["incremento_vuelta"])

        self.en_calibracion = True
        self.v_actual = self.v_min
        self.ultimo_pwm_enviado = 0.0

        self.puntos_crudos = {}
        self.trayectoria_base = {}
        self.algoritmos = {}
        self.vueltas = 0
        self.derrapes_ultima_vuelta = 0
        #--------------------------------------------------

        # --- INFO CONTROL MANUAL ---
        # Cuando corres en control manual se muestra informacion al conductor
        # de como fue la vuelta
        # Se usa en mostrar_panel_piloto

        # Permite saber cual fue el mejor tiempo de la carrera
        self.mejor_tiempo = None
        # Permite saber cual fue el tiempo de la vuelta anterior
        self.tiempo_vuelta_anterior = None
        #--------------------------------------------------

        
        # --- DETECCION E INSTANTE DEL PASO POR META ---
        # Como se detecta el paso por meta y como se calcula el tiempo que llevo
        # la vuelta son ideas de Mario en su TFG (getTiempoVuelta,
        # https://github.com/mariolopez15/control_coche_scalextric)

        self.finish_line = {"camara_id": None, "coordenadas": None}
        
        # Tupla: (punto_front, stamp_ns, distancia_con_signo, n_frame)
        # Guarda la informacion del ultimo mensaje de la camara que ve la meta
        self.front_meta_anterior = None

        # Instante interpolado del ultimo cruce de meta -> Tiempo por vuelta
        # None -> Todavia no se paso por meta ninguna vez
        self.t_meta_anterior = None

        # Distancia maxima a la que se puede estar de la meta para dar por bueno
        # el paso por meta
        self.declare_parameter("controller.umbral_meta", 30.0)
        self.umbral_meta = self.get_parameter("controller.umbral_meta").value
        #--------------------------------------------------


        # --- ORDEN DE CAMARAS ---
        # El coche pasa por las camaras siempre en el mismo orden. Hay que guardar
        # el orden para aplicar las reducciones

        # Camara que manda ahora mismo, es valida si sigue entregando frames
        self.camara_activa = None

        # Instante del ultimo frame valido con el reloj del controlador
        self.t_ultimo_frame_valido = None

        # Segundos sin frames validos para dar por perdida la camara activa y cambiar
        self.declare_parameter("controller.timeout_camara_activa", 0.3)
        self.timeout_camara_activa = self.get_parameter("controller.timeout_camara_activa").value

        # Diccionario para guardar el orden de las camaras
        # Se completa al dar la vuelta de calibracion
        # {camara: camara anterior}
        self.camara_precedente = {}

        # Numero del ultimo frame de la camaraN
        # {camara: ultimo n_frame suyo}
        self.ultimo_frame_camara = {}
        #--------------------------------------------------


        # --- SUBSCRIBERS ---

        # Subscriber de /<car>/position -> CarLocation.msg
        self.sub_car_position = self.create_subscription(
            CarLocation,
            "position", # Relativo al namespace del nodo
            self.callback_posicion,
            qos_profile_sensor_data,
            callback_group=self.car_position_group,
        )

        # Subscriber de /<car>/modo_calibracion -> Bool
        # Avisa de que la calibracion de este coche empieza o termina
        self.sub_modo_calibracion = self.create_subscription(
            Bool,
            "modo_calibracion", # Relativo al namespace del nodo
            self.callback_control_calibracion,
            10,
            callback_group=self.car_position_group,
        )

        # Subscriber de /finish_line_position -> FinishLine.msg
        self.sub_finish_line_position = self.create_subscription(
            FinishLine,
            "/finish_line_position",
            self.callback_get_finish_line_position,
            QOS_FINISH_LINE,
            callback_group=self.car_position_group,
        )
        #--------------------------------------------------


        # --- PUBLISHERS ---
       
        # Publisher de /<car>/pwd -> SpeedCarril.msg
        self.pub_pwm = self.create_publisher(
            SpeedCarril,
            "pwd", # Relativo al namespace del nodo
            qos_profile_sensor_data,
            callback_group=self.car_position_group,
        )

        # Publisher de /telemetria/<car>/time_per_lap -> TimePerLap.msg
        url_publiser_time_per_lap = f"/telemetria/{self.car_name}/time_per_lap"
        self.pub_time_per_lap = self.create_publisher(
            TimePerLap,
            url_publiser_time_per_lap,
            qos_profile_sensor_data,
            callback_group=self.car_position_group,
        )

        # Publisher de /<car>/modo_calibracion -> Bool
        # Avisa al resto del sistema de que se termino la calibracion de este coche
        self.pub_modo_calibracion = self.create_publisher(
            Bool,
            "modo_calibracion", # Relativo al namespace del nodo
            10,
            callback_group=self.car_position_group,
        )

        # Publisher de /telemetria/<car>/car_control -> CarControlTelemetry.msg
        url_publiser_car_control_telemetry = f"/telemetria/{self.car_name}/car_control"
        self.pub_car_control_telemetry = self.create_publisher(
            CarControlTelemetry,
            url_publiser_car_control_telemetry,
            qos_profile_sensor_data,
            callback_group=self.car_position_group,
        )

        #--------------------------------------------------

        # Ruta donde se guarda la cache
        self.cache_file = "/ros2_ws/src/image_processor_pkg/cache_trayectoria_controller.json"
        
        # Log sobre el modo de operacion del coche
        if self.modo_manual:
            self.get_logger().warn(
                "[MODO] Controlador iniciado en MODO MANUAL: conduce una persona. "
                "Da la primera vuelta despacio para calibrar el sistema"
            )
        elif self.modo == "incremental":
            self.get_logger().info(
                f"[MODO] Controlador iniciado en MODO INCREMENTAL: se arranca en "
                f"PWM {self.v_min:.0f} y se sube {self.incremento_vuelta:.0f} "
                f"cada {self.vueltas_incremento} vueltas hasta {self.v_max:.0f}"
            )
        elif self.modo == "politica":
            self.get_logger().info(
                "[MODO] Controlador iniciado en MODO POLÍTICA: el perfil se carga "
                "de un JSON y no se modifica en toda la carrera"
            )
        else:
            self.get_logger().info("[MODO] Controlador iniciado en MODO AUTOMÁTICO")



    def callback_get_finish_line_position(self, msg):
        """Callback que se llama cuando llega el mensaje de FinishLine.msg  
        Se guarda la meta y que camara la ve"""

        self.finish_line["camara_id"] = msg.camara_id

        f_s_x = msg.finish_line.start.x
        f_s_y = msg.finish_line.start.y
        f_e_x = msg.finish_line.end.x
        f_e_y = msg.finish_line.end.y

        self.finish_line["coordenadas"] = ((f_s_x, f_s_y), (f_e_x, f_e_y))

    def callback_control_calibracion(self, msg):
        """Callback que se llama cuando llega el mensaje de cambio de calibracion.
        Para al modo carrera al coche o pasa al modo de calibracion"""

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
            # El orden de camaras se reaprende con la nueva calibracion
            self.camara_activa = None
            self.t_ultimo_frame_valido = None
            self.camara_precedente.clear()
            self.ultimo_frame_camara.clear()
            self.front_meta_anterior = None
            self.t_meta_anterior = None
            self.mejor_tiempo = None
            self.tiempo_vuelta_anterior = None
            self.publicar_velocidad(0)
            self.get_logger().warn("[CALIBRACION] Reiniciando calibración")


    def callback_posicion(self, msg: CarLocation):
        """Callback que se llamada cada vez que llega un mensaje CarLocation.msg 
        Dependiendo de si estamos en calibracion o no llamamos a diferentes funciones
        porque el mensaje CarLocation se trata distinto"""

        if self.en_calibracion:
            self.recolectar_datos_calibracion(msg)

        else:
            self.ejecutar_control_carrera(msg)

    def recolectar_datos_calibracion(self, msg: CarLocation):
        """Funcion que procesa los CarLocation mientras estamos en modo calibracion
        Guarda las posiciones para construir despues su trayectoria"""
        camara = msg.camara_id

        fx, fy = float(msg.front.center.x), float(msg.front.center.y)

        if fx == 0 and fy == 0:
            return

        punto_front = np.array([fx, fy], dtype=np.float32)

        if self.t_meta_anterior is not None:
            if camara not in self.puntos_crudos:
                self.puntos_crudos[camara] = []

            self.puntos_crudos[camara].append((fx, fy))

        # Comprobamos si cruzamos la linea de meta para terminar la calibracion
        if self.verificar_linea_meta(camara, punto_front, msg.stamp, msg.n_frame):
            msg_fin_calibracion = Bool()
            msg_fin_calibracion.data = False
            self.pub_modo_calibracion.publish(msg_fin_calibracion)
            self.get_logger().info(
                f"[CALIBRACION] Vuelta completada para {self.car_name}, notificando al sistema"
            )

    def procesar_trayectorias(self):
        """
        Cierre de la calibracion y paso al modo carrera
        Crea la trayectoria base y arranca una instancia del algoritmo por camara
        """

        datos_a_guardar = {}

        for camara, puntos in self.puntos_crudos.items():
            if not puntos:
                continue

            ruta_limpia = [puntos[0]]

            for i in range(1, len(puntos)):
                ult_p = ruta_limpia[-1]
                p_act = puntos[i]
                # Solo nos quedamos con los puntos consecutivos separados mas de umbral_distancia
                if (
                    math.hypot(p_act[0] - ult_p[0], p_act[1] - ult_p[1])
                    > self.umbral_distancia
                ):
                    ruta_limpia.append(p_act)

            self.trayectoria_base[camara] = np.array(ruta_limpia, dtype=np.float32)

            # Guardamos la trayectoria base
            datos_a_guardar[camara] = self.trayectoria_base[camara].tolist()

            # Creamos una instancia del algoritmo por cada camara que tenemos
            self.algoritmos[camara] = EstrategiaPerfil(
                self.v_max, self.v_min, self.get_name(), camara,
                modo=self.modo,
                **self.params_algoritmo,
            )

            # Cargamos la trayectoria base en cada instancia del algoritmo
            self.algoritmos[camara].setTrayectoria(
                self.trayectoria_base[camara],
                varias_camaras=len(self.puntos_crudos) > 1,
            )

            self.get_logger().info(
                f"[CALIBRACION] {camara}: terminada, ruta base con {len(ruta_limpia)} nodos"
            )

        # Se guarda en disco una copia de la trayectoria
        try:
            with open(self.cache_file, "w") as f:
                json.dump(datos_a_guardar, f, indent=4)
            self.get_logger().info(f"[CACHE] Trayectoria del controlador guardada en {self.cache_file}")
        except Exception as e:
            self.get_logger().error(f"[CACHE] Error guardando la caché del controlador: {e}")

        # Cambiamos al modo carrera y empezamos a contar las vueltas
        self.vueltas = 0

        self.get_logger().info("[MODO] Trayectoria lista, pasando a MODO CARRERA")

    def ejecutar_control_carrera(self, msg: CarLocation):
        """
        Funcion que procesa los CarLocation mientras estamos en modo carrera
        Se aplica el control para cada posicion que recibe y se comprueba
        tambien si hubo cruce por meta. Se publica el PWM y el cruce de meta
        """

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

        if self.verificar_linea_meta(camara, punto_front, msg.stamp, msg.n_frame):
            self.registrar_vuelta_algoritmos()

        # El numero del frame lo marca la camara que envia el mensaje
        self.ultimo_frame_camara[camara] = msg.n_frame

        # Aplicamos el algoritmo y devuelve el PWM que toca aplicar
        nueva_vel = self.algoritmos[camara].actualizar_estado(punto_front, punto_back, msg.n_frame, self.vueltas)

        # None quiere decir que el algoritmo descarto el frame, se mantiene el PWM que habia
        if nueva_vel is not None:
            self.v_actual = nueva_vel

        # En modo incremental el PWM no lo decide el algoritmo
        if self.modo == "incremental":
            escalones = self.vueltas // self.vueltas_incremento
            self.v_actual = min(
                self.v_max, self.v_min + self.incremento_vuelta * escalones)

        # --- Seguimiento de la camara activa ---
        # Nos quedamos con la camara que estamos escuchando mientras entren frames
        # Solo se cambia cuando se llega al timeout_camara_activa

        if self.algoritmos[camara].frame_valido:
            if self.camara_activa is None:
                self.camara_activa = camara
                self.t_ultimo_frame_valido = time_received_from_camera
                self.log_algoritmo(f"[CAMARA] Cámara activa inicial: {camara}")

            elif camara == self.camara_activa:
                self.t_ultimo_frame_valido = time_received_from_camera

            else:
                # Comprobamos cuanto hace que no recibimos un frame
                sin_activa = (
                    time_received_from_camera - self.t_ultimo_frame_valido
                ).nanoseconds / 1e9

                if sin_activa > self.timeout_camara_activa:
                    # La activa dejo de ver el coche, se cierra el derrape que
                    # tuviera abierto
                    self.algoritmos[self.camara_activa].notificar_perdida_vision(
                        self.vueltas, self.ultimo_frame_camara[self.camara_activa]
                    )

                    self.camara_precedente[camara] = self.camara_activa

                    self.log_algoritmo(
                        f"[CAMARA] Cámara activa: {self.camara_activa} -> {camara} "
                        f"(precedente de {camara} = {self.camara_activa})"
                    )

                    self.camara_activa = camara
                    self.t_ultimo_frame_valido = time_received_from_camera

        # --- Aplicacion de reduccion en camara anterior ---
        # Si al retroceder una zona se salio por el inicio de su trayectoria,
        # los px sobrantes se aplican al final de la trayectoria de la camara anterior

        # Comprobamos si hay reduccion
        pendiente = self.algoritmos[camara].consumir_reduccion_pendiente()

        if pendiente > 0.0:

            precedente = self.camara_precedente.get(camara)

            if precedente is not None and precedente in self.algoritmos:
                self.log_algoritmo(
                    f"[DERRAME] {camara} pide reducir {pendiente:.0f} px; "
                    f"se aplican al final de la trayectoria de {precedente}"
                )

                self.algoritmos[precedente].aplicar_reduccion_externa(
                    pendiente, self.vueltas
                )

            else:
                # No sabemos que camara es, entonces no se puede aplicar la reduccion
                self.get_logger().warn(
                    f"[DERRAME] {pendiente:.0f} px de {camara} sin aplicar: "
                    f"aún no se conoce su cámara precedente"
                )

        # Publicamos siempre el valor de PWM
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

        # Publicamos el mensaje de telemetria
        self.pub_car_control_telemetry.publish(msg_car_control_telemetry)

        # Log solo si cambia para no saturar la terminal
        if nueva_vel is not None and nueva_vel != self.ultimo_pwm_enviado:
            self.log_algoritmo(
                f"[PWM] {self.v_actual} (perfil {self.v_min}-{self.v_max})"
            )
            self.ultimo_pwm_enviado = self.v_actual

    def registrar_vuelta_algoritmos(self):
        """Avisa a los algoritmos de que se ha cruzado meta"""

        # Calculamos si hubo derrapes en la vuelta o no
        # Sirve como log, no se usa dentro del control
        derrapes_totales = sum(a.derrapes_contador for a in self.algoritmos.values())
        vuelta_limpia = derrapes_totales == self.derrapes_ultima_vuelta
        self.derrapes_ultima_vuelta = derrapes_totales

        # Para cada instancia de los algoritmos cerramos la vuelta
        for algoritmo in self.algoritmos.values():
            algoritmo.registrar_vuelta(self.vueltas, vuelta_limpia)

        if vuelta_limpia:
            self.log_algoritmo("[VUELTA] Limpia, sin derrapes en ninguna cámara")
        else:
            self.log_algoritmo(
                "[VUELTA] Con derrapes: suben las zonas no protegidas; "
                "las castigadas esperan"
            )

    def distancia_punto_segmento(self, P, A, B):
        """Distancia del punto P al segmento AB, no a la recta que lo prolonga"""
        AB = B - A
        AP = P - A
        l2 = np.sum(AB**2)
        if l2 == 0:
            return np.linalg.norm(AP)
        t = max(0.0, min(1.0, np.dot(AP, AB) / l2))
        proyeccion = A + t * AB
        return np.linalg.norm(P - proyeccion)

    def publicar_velocidad(self, pwm):
        """Se publica el valor de PWM que queremos aplicar en el carril"""

        # Modo manual no se aplica control
        if self.modo_manual:
            return

        msg_vel = SpeedCarril()
        msg_vel.pwm = pwm
        msg_vel.carril = str(self.carril) # cars.<coche>.carril
        msg_vel.stamp = self.get_clock().now().to_msg()
        self.pub_pwm.publish(msg_vel)

    def log_algoritmo(self, texto):
        """
        Controla que los logs de control no se muestren por terminal cuando estamos
        en modo manual
        """
        if not self.modo_manual:
            self.get_logger().info(texto)

    def mostrar_panel_piloto(self, lap_time, lap_number):
        """
        Informacion que se muestra durante el modo manual, sirve de feedback a la
        persona que conduce para saber como lo esta haciendo
        """
        lineas = [f"🏁 VUELTA {lap_number} · {lap_time:.3f} s"]

        if self.tiempo_vuelta_anterior is not None:
            delta = lap_time - self.tiempo_vuelta_anterior
            flecha = "▼" if delta < 0 else "▲"
            lineas[0] += f"  {flecha} {delta:+.3f} vs anterior"

        if self.mejor_tiempo is None or lap_time < self.mejor_tiempo[0]:
            if self.mejor_tiempo is not None:
                lineas.append(
                    f"🏆 ¡MEJOR VUELTA! (antes {self.mejor_tiempo[0]:.3f} s "
                    f"en la vuelta {self.mejor_tiempo[1]})"
                )
            self.mejor_tiempo = (lap_time, lap_number)
        else:
            lineas.append(
                f"   mejor: {self.mejor_tiempo[0]:.3f} s "
                f"(vuelta {self.mejor_tiempo[1]})"
            )

        self.tiempo_vuelta_anterior = lap_time

        self.get_logger().info("\n".join(lineas))

    def publicar_time_lap(self, lap_time, lap_number):
        """Publica el tiempo de la vuelta que se acaba de cerrar"""
        msg_time_per_lap = TimePerLap()
        msg_time_per_lap.lap_time = lap_time
        msg_time_per_lap.lap_number = lap_number
        msg_time_per_lap.stamp = self.get_clock().now().to_msg()
        self.pub_time_per_lap.publish(msg_time_per_lap)

    def distancia_con_signo(self, P, A, B):
        """
        Distancia de P a la recta AB, con signo. El signo dice de que lado cae
        P, y por eso un cambio de signo entre dos frames es un cruce

        El producto cruzado 2D va a mano porque np.cross en 2D ya no existe
        """
        AB = B - A
        AP = P - A
        return float(AB[0] * AP[1] - AB[1] * AP[0]) / float(np.linalg.norm(AB))

    def verificar_linea_meta(self, camara_id, p_front, stamp, n_frame):
        """
        Devuelve True si se completo una vuelta

        El cruce es el cambio de signo de la distancia de la pegatina delantera
        a la recta de meta

        El instante exacto se interpola entre los dos frames
        """
        if (
            self.finish_line["camara_id"] is None
            or camara_id != self.finish_line["camara_id"]
        ):
            return False

        # Stamp de cuando se captura el frame en CameraNode
        t_stamp = Time.from_msg(stamp).nanoseconds

        A, B = self.finish_line["coordenadas"]
        A_np = np.array(A, dtype=np.float32)
        B_np = np.array(B, dtype=np.float32)

        d_curr = self.distancia_con_signo(p_front, A_np, B_np)

        vuelta_completada = False

        if self.front_meta_anterior is None:
            # Aun no se cruzo la meta una primera vez
            self.get_logger().info("[META] Primer frame de la cámara que ve la meta recibido")
        elif not (0 < n_frame - self.front_meta_anterior[3] <= MAX_SALTO_FRAMES_META):
            # Se comprueba si son frames consecutivos
            # Si pasa tiempo sin ver el coche puede pasar que por la posicion
            # se produzca un cambio de signo y marque un cruce de meta que es falso
            # Cuando se cruza la meta tienen que ser posiciones consecutivas
            self.front_meta_anterior = (p_front.copy(), t_stamp, d_curr, n_frame)
            return False
        else:
            p_prev, t_prev, d_prev, _ = self.front_meta_anterior

            if d_prev != 0.0 and d_prev * d_curr <= 0.0:

                # 0 -> frame anterior y 1 -> frame actual
                # Fraccion del intervalo recorrida hasta tocar la recta
                s = abs(d_prev) / (abs(d_prev) + abs(d_curr))

                # Punto donde toco la recta
                p_cruce = p_prev + s * (p_front - p_prev)

                # Sabemos si estamos pasando por el segmento AB
                dist_meta = self.distancia_punto_segmento(p_cruce, A_np, B_np)

                if dist_meta > self.umbral_meta:
                    self.get_logger().warn(
                        f"[META] Cambio de lado a {dist_meta:.0f} px del segmento de "
                        f"meta (> {self.umbral_meta:.0f}): el coche cruzó la "
                        "prolongación de la línea, no la meta"
                    )
                else:
                    # Instante exacto de cruce interpolando entre los stamps
                    t_meta = t_prev + s * (t_stamp - t_prev)

                    if self.t_meta_anterior is None:
                        # La primera pasada solo arranca el cronometro
                        self.get_logger().info("[META] Primera pasada, iniciando cronómetro")
                    else:

                        self.vueltas += 1
                        lap_time = (t_meta - self.t_meta_anterior) / 1e9

                        self.publicar_time_lap(lap_time, self.vueltas)

                        if self.modo_manual:
                            # En modo manual se da mas informacion al conductor
                            self.mostrar_panel_piloto(lap_time, self.vueltas)
                        else:
                            self.get_logger().info(f"[VUELTA] {self.vueltas} completada en {lap_time:.3f} s")

                        vuelta_completada = True

                    self.t_meta_anterior = t_meta

        self.front_meta_anterior = (p_front.copy(), t_stamp, d_curr, n_frame)

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
