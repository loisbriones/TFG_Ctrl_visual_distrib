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

# Fotogramas de separación como mucho entre los dos mensajes que compara
# verificar_linea_meta. El cruce de meta es un CAMBIO DE LADO entre dos
# fotogramas SEGUIDOS; si entre ellos la cámara procesó muchos sin ver el
# coche, el guardado no es el fotograma anterior sino el de la última vez que
# lo vio, y el coche pudo reaparecer al otro lado de la recta sin cruzar la
# meta (le dio la vuelta al circuito por fuera del encuadre).
#
# Medido en CIRCUITO_FINAL_{ROJO,AZUL}_MANUAL_002 y CIRCUITO_FINAL_AZUL_002:
# los cruces REALES van con 1 a 4 fotogramas de separación (el 93-96 % con 1),
# y los falsos con 117 a 520. Cualquier valor entre 5 y 100 da el mismo
# recuento de vueltas, así que esto no es un umbral que haya que ajustar: es
# un corte en mitad de un hueco de dos órdenes de magnitud.
#
# Se cuenta en FOTOGRAMAS y no en segundos ni en píxeles a propósito: n_frame
# dice cuántos fotogramas procesó esa cámara sin ver el coche, y eso no
# depende del tamaño del circuito, ni de la velocidad del coche, ni de lo que
# dure la vuelta. Un umbral en segundos quedaría atado a la duración de la
# vuelta y uno en píxeles a la altura de la cámara.
MAX_SALTO_FRAMES_META = 10


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

        # --- PARÁMETROS DEL ALGORITMO DE VELOCIDAD ---
        # EstrategiaPerfil no es un nodo ROS y no puede leer params.yaml por
        # su cuenta: los lee este nodo y se los pasa al construir cada
        # instancia (ver procesar_trayectorias). Se declaran con el mismo
        # valor que tiene por defecto la clase, así que un params.yaml sin
        # el bloque controller.algoritmo se comporta exactamente igual.
        # Para qué sirve cada uno está comentado en params.yaml.
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

        # --- MODO DE OPERACIÓN: quién decide el PWM ---
        # Los cuatro modos de la memoria, y lo que cambia en ESTE nodo:
        #   manual      -> NO se publica orden de PWM (publicar_velocidad sale
        #                  antes): conduce una persona con el mando físico y el
        #                  arduino_bridge ni se lanza. La terminal pasa a ser el
        #                  salpicadero del piloto (log_algoritmo y
        #                  mostrar_panel_piloto).
        #   incremental -> se publica un PWM que NO decide el algoritmo: parte
        #                  de v_min y sube un escalón cada vueltas_incremento
        #                  vueltas (ver ejecutar_control_carrera).
        #   automatico  -> se publica el PWM que decide el algoritmo. El de
        #                  siempre, y el valor por defecto.
        #   politica    -> se publica el PWM del algoritmo, pero su perfil está
        #                  congelado porque se cargó de un JSON. Lo bloquea la
        #                  propia EstrategiaPerfil, no este nodo.
        #
        # En LOS CUATRO el algoritmo corre entero, y es a propósito: la vuelta
        # de calibración, la trayectoria base, una EstrategiaPerfil por cámara,
        # la detección de derrape, el log y la telemetría son iguales en todos.
        # Su log guarda fotograma a fotograma qué PWM habría aplicado
        # ([FRAME ... pwm=N]), así que en manual se puede contrastar con lo que
        # hizo la persona y en incremental con el escalón que iba puesto: ahí se
        # ve dónde el algoritmo es demasiado conservador y dónde el humano (o el
        # escalón) se arriesga más de lo que este permitiría. El PWM realmente
        # aplicado no está en ese log, está en el bag (/carX/pwd), y el análisis
        # pinta las dos series por separado.
        self.declare_parameter("modo", "automatico")
        self.modo = self.get_parameter("modo").value
        # Flag derivado: es el que consultan publicar_velocidad, log_algoritmo y
        # el panel del piloto, y su significado no ha cambiado al añadir modos
        self.modo_manual = (self.modo == "manual")

        # Vueltas que aguanta el modo incremental antes de subir un escalón. El
        # escalón es incremento_vuelta, el mismo que usa el algoritmo al subir
        # una zona, para que los dos modos hablen en las mismas unidades
        self.declare_parameter("controller.vueltas_incremento", 10)
        self.vueltas_incremento = max(
            1, int(self.get_parameter("controller.vueltas_incremento").value))
        # El escalón sale del mismo parámetro con el que el algoritmo sube una
        # zona al cruzar meta, que ya está leído en params_algoritmo
        self.incremento_vuelta = float(self.params_algoritmo["incremento_vuelta"])

        self.en_calibracion = True
        self.v_actual = self.v_min
        self.ultimo_pwm_enviado = 0.0

        self.puntos_crudos = {}
        self.trayectoria_base = {}
        self.algoritmos = {}
        self.vueltas = 0
        self.derrapes_ultima_vuelta = 0

        # --- CRONOMETRAJE PARA EL PILOTO (solo modo manual) ---
        # Los lleva mostrar_panel_piloto, su único consumidor: en modo
        # automático se quedan a None porque nadie los mira (el dashboard saca
        # la mejor vuelta del bag). (tiempo, nº de vuelta) de la vuelta más
        # rápida hasta ahora, y el tiempo de la vuelta inmediatamente anterior
        # para poder decir "¿he mejorado?" al cruzar meta.
        self.mejor_tiempo = None
        self.tiempo_vuelta_anterior = None

        self.finish_line = {"camara_id": None, "coordenadas": None}

        # --- DETECCIÓN E INSTANTE DEL PASO POR META ---
        # El cruce se detecta con la pegatina DELANTERA sola: se mide su
        # distancia perpendicular CON SIGNO a la recta de meta (el signo
        # dice de qué lado de la recta está) y se da la meta por cruzada
        # cuando ese signo cambia entre dos fotogramas.
        #
        # Sustituye al método del TFG de Adrián, que fue el primero que se
        # implementó: cortar el segmento delantera->trasera contra el de
        # meta. Se cambió por tres razones, y las tres siguen justificando
        # el método actual:
        #   - Es un FLANCO por naturaleza: un coche parado a caballo de la
        #     línea tiene signo constante y no cuenta vueltas. Con el corte
        #     de segmentos la condición era cierta de forma continua y hacía
        #     falta un debounce temporal, que contaba una vuelta por segundo
        #     con el coche quieto encima de la meta.
        #   - Detectar e interpolar pasan a ser el MISMO evento: si se
        #     detecta porque la delantera cambió de lado, el instante exacto
        #     del cruce cae por definición entre esos dos fotogramas.
        #   - No necesita la trasera, así que la cámara puede publicar en
        #     calibración aunque solo vea la delantera (más puntos para la
        #     trayectoria base) sin provocar cruces falsos.
        #
        # Interpolar importa porque a ~30 Hz el cruce real ocurre ENTRE dos
        # fotogramas: quedarse con el stamp del que lo detecta mete hasta
        # ~33 ms de error por vuelta. Es la idea de Mario en su TFG
        # (getTiempoVuelta), con distancia perpendicular real en vez de la
        # aproximación por el eje dominante de la meta. Los tiempos son
        # stamps de CAPTURA de la cámara que ve la meta: como el tiempo de
        # vuelta es la diferencia de dos stamps de la MISMA Raspberry, el
        # desfase de reloj entre máquinas se cancela (no hace falta NTP).
        #
        # (punto_front, stamp_ns, distancia_con_signo, n_frame) del mensaje
        # ANTERIOR de la cámara de meta
        self.front_meta_anterior = None
        # Instante interpolado (ns, reloj de la Raspberry de meta) del
        # último cruce; la diferencia entre dos de estos es el lap_time, y
        # que siga a None marca que aún no hemos pasado por meta ninguna vez
        self.t_meta_anterior = None
        # Tolerancia (px) para dar un cambio de signo por bueno: el punto de
        # cruce interpolado cae sobre la RECTA de meta, pero la meta es un
        # SEGMENTO; si su prolongación corta la pista en otro sitio, el
        # cambio de signo de allí se descarta por esta distancia
        self.declare_parameter("controller.umbral_meta", 30.0)
        self.umbral_meta = self.get_parameter("controller.umbral_meta").value

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
        self.declare_parameter("controller.timeout_camara_activa", 0.3)
        self.timeout_camara_activa = self.get_parameter(
            "controller.timeout_camara_activa"
        ).value
        # {cámara: cámara anterior en el orden de paso}: se rellena en cada
        # conmutación y tras una vuelta completa queda el ciclo entero
        self.camara_precedente = {}
        # {cámara: último n_frame recibido de ella}. El controlador ya no lleva
        # ningún contador propio: el número de fotograma lo pone cada cámara
        # (msg.n_frame) y solo tiene sentido dentro de su propia escala. Esto
        # guarda el último de cada una para poder etiquetar eventos que se
        # escriben en el log de una cámara pero los dispara un mensaje de otra
        # (ver la llamada a notificar_perdida_vision más abajo).
        self.ultimo_frame_camara = {}

        self.sub_car_position = self.create_subscription(
            CarLocation,
            "position",
            self.callback_posicion,
            qos_profile_sensor_data,
            callback_group=self.car_position_group,
        )
        # Topic RELATIVO: el nodo corre en el namespace del coche (/carX), asi
        # que "modo_calibracion" resuelve a /carX/modo_calibracion. La calibracion
        # es POR COCHE: cada coche cierra su vuelta cuando cruza la meta sin sacar
        # de calibracion a los demas (antes era un /modo_calibracion global y el
        # primer coche en terminar cortaba la calibracion de todos).
        self.sub_modo_calibracion = self.create_subscription(
            Bool,
            "modo_calibracion",
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

        # Avisar de que se acabo el modo calibracion. Topic RELATIVO
        # (/carX/modo_calibracion): solo afecta a ESTE coche; la camara y el
        # puente escuchan el topic de cada coche por separado.
        self.pub_modo_calibracion = self.create_publisher(
            Bool,
            "modo_calibracion",
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

        if self.modo_manual:
            # warn y no info: en manual es importante que quien arranca el
            # sistema vea de un vistazo que el nodo NO va a mover el coche, y
            # que la vuelta de calibración hay que conducirla a mano (sin
            # arduino_bridge nadie pone los carriles a calibration_speed)
            self.get_logger().warn(
                "🕹️ Controlador iniciado en MODO MANUAL: conduce una persona y "
                "NO se publica ninguna orden de PWM. El algoritmo corre igual "
                "para dejar en su log lo que habría hecho. Conduce la vuelta de "
                "calibración despacio y sin parar: de ella sale la trayectoria "
                "base."
            )
        elif self.modo == "incremental":
            self.get_logger().info(
                f"📈 Controlador iniciado en MODO INCREMENTAL: se arranca en "
                f"PWM {self.v_min:.0f} y se sube {self.incremento_vuelta:.0f} "
                f"cada {self.vueltas_incremento} vueltas hasta {self.v_max:.0f}, "
                f"igual en todo el circuito. El algoritmo corre igual para dejar "
                f"en su log lo que habría hecho. MODO CALIBRACIÓN ACTIVO."
            )
        elif self.modo == "politica":
            self.get_logger().info(
                "📋 Controlador iniciado en MODO POLÍTICA: el perfil se carga de "
                "un JSON y NO se modifica en toda la carrera. Los derrapes se "
                "detectan y se registran, pero no castigan. MODO CALIBRACIÓN "
                "ACTIVO."
            )
        else:
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
            # Los nodos cámara NO reinician su contador al reiniciar la
            # calibración (siguen corriendo), pero los números guardados aquí
            # son de la carrera anterior: mejor vaciarlos que arrastrarlos
            self.ultimo_frame_camara.clear()
            # El cronómetro de meta también parte de cero
            self.front_meta_anterior = None
            self.t_meta_anterior = None
            # Y con él los tiempos que se le enseñan al piloto: la mejor vuelta
            # de la carrera anterior no vale para la que empieza
            self.mejor_tiempo = None
            self.tiempo_vuelta_anterior = None
            self.publicar_velocidad(0)
            self.get_logger().warn("⚠️ Reiniciando calibración.")


    def callback_posicion(self, msg: CarLocation):
        if self.en_calibracion:
            self.recolectar_datos_calibracion(msg)
        else:
            self.ejecutar_control_carrera(msg)

    def recolectar_datos_calibracion(self, msg: CarLocation):
        camara = msg.camara_id

        fx, fy = float(msg.front.center.x), float(msg.front.center.y)

        if fx == 0 and fy == 0:
            return

        punto_front = np.array([fx, fy], dtype=np.float32)

        # Solo se graba la vuelta que va del PRIMER paso por meta al
        # segundo: todo lo anterior (colocar el coche, arrancar, las vueltas
        # que dé mientras se lanza el resto del sistema) se descarta. Así la
        # trayectoria base es exactamente UNA vuelta y empieza en la meta,
        # en vez de la vuelta y pico con cola duplicada de antes.
        # La trasera puede llegar como (0, 0) si la cámara no la vio: da
        # igual, ni la trayectoria ni la detección de meta la usan.
        if self.t_meta_anterior is not None:
            if camara not in self.puntos_crudos:
                self.puntos_crudos[camara] = []

            self.puntos_crudos[camara].append((fx, fy))

        # Como ya dimos una vuelta podemos terminar la calibración
        if self.verificar_linea_meta(camara, punto_front, msg.stamp, msg.n_frame):
            msg_fin_calibracion = Bool()
            msg_fin_calibracion.data = False
            self.pub_modo_calibracion.publish(msg_fin_calibracion)
            self.get_logger().info(
                "🏁 Vuelta de calibración completada: publicado False en /carX/modo_calibracion"
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
            # varias, la meta cae en mitad de la porción que ve esta cámara y
            # el salto es el resto del circuito, que ven las demás
            # El modo se le pasa porque el de política le congela el perfil (lo
            # carga de un JSON y no deja que nada lo mueva); en los otros tres
            # el algoritmo se comporta igual, sea quien sea el que publique.
            self.algoritmos[camara] = EstrategiaPerfil(
                self.v_max, self.v_min, self.get_name(), camara,
                modo=self.modo,
                **self.params_algoritmo,
            )
            self.algoritmos[camara].setTrayectoria(
                self.trayectoria_base[camara],
                varias_camaras=len(self.puntos_crudos) > 1,
            )

            self.get_logger().info(
                f"✅ {camara}: Ruta base con {len(ruta_limpia)} nodos. Algoritmo de perfil inyectado."
            )

            # Una vuelta entera vista por una cámara da del orden de cientos
            # de nodos separados 15 px. Si salen cuatro, la vuelta de
            # calibración fue un espejismo (cruce de meta falso) o la cámara
            # apenas vio el coche: el perfil de PWM no puede funcionar y es
            # mejor enterarse aquí que tras media carrera sin acelerar.
            if len(ruta_limpia) < 20:
                self.get_logger().error(
                    f"❌ {camara}: la ruta base tiene solo {len(ruta_limpia)} nodos. "
                    "La vuelta de calibración no es válida: repítela antes de "
                    "dar valor a nada de lo que venga después."
                )

            # Un hueco grande entre dos nodos significa que la camara dejo de
            # ver el coche en ese tramo. El algoritmo lo tapa (interpolando o
            # con una celda gigante segun el tamano) y sigue funcionando, pero
            # si el tramo tapado era una curva la cadena deja de describir la
            # pista ahi y salen derrapes falsos en el mismo sitio cada vuelta.
            # Solo se avisa de los que pasan de umbral_celda_gigante, que son
            # los que ademas cambian la forma de la cadena; el detalle completo
            # de todos los huecos va al log del algoritmo ([TRAY] AVISO).
            #
            # EXCEPTO si la ruta se cierra sobre si misma: eso significa que la
            # meta cae en mitad de la porcion que ve esta camara, la lista llega
            # como [cola del tramo, salto, cabeza del tramo] y el salto es el
            # resto del circuito, no un hueco. setTrayectoria la rota y el salto
            # desaparece. Sin esta condicion, la camara que ve la meta avisaba
            # SIEMPRE de un hueco de varios cientos de px que no existia, y el
            # aviso no servia para decidir si repetir la calibracion.
            umbral_hueco = self.params_algoritmo["umbral_celda_gigante"]
            cierre = math.hypot(
                ruta_limpia[-1][0] - ruta_limpia[0][0],
                ruta_limpia[-1][1] - ruta_limpia[0][1],
            )
            huecos = [
                math.hypot(b[0] - a[0], b[1] - a[1])
                for a, b in zip(ruta_limpia, ruta_limpia[1:])
                if math.hypot(b[0] - a[0], b[1] - a[1]) > umbral_hueco
            ]
            if huecos and cierre > self.params_algoritmo["umbral_cierre"]:
                self.get_logger().warn(
                    f"⚠️ {camara}: la calibración trae {len(huecos)} hueco(s) de "
                    f"más de {umbral_hueco:.0f} px (el mayor {max(huecos):.0f} px): "
                    "la cámara perdió el coche en ese tramo. Si el coche derrapa "
                    "siempre en el mismo punto, repite la calibración."
                )

        # 3. Guardamos en disco la trayectoria de puntos por cámara
        try:
            with open(self.cache_file, "w") as f:
                # Usamos indent=4 para que el JSON quede formateado y sea fácil de leer por humanos
                json.dump(datos_a_guardar, f, indent=4)
            self.get_logger().info(f"💾 Trayectoria del controlador guardada en {self.cache_file}")
        except Exception as e:
            self.get_logger().error(f"❌ Error guardando caché del controlador: {e}")

        # La carrera empieza AQUÍ: el cruce de meta que acaba de cerrar la
        # calibración es también el inicio de la vuelta 1, y t_meta_anterior
        # ya apunta a ese instante, así que el tiempo de la primera vuelta
        # sale bien sin tocar nada más. Las vueltas de la calibración no se
        # cuentan: antes la numeración de la carrera empezaba en 2 o en 3.
        self.vueltas = 0

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

        if self.verificar_linea_meta(camara, punto_front, msg.stamp, msg.n_frame):
            self.registrar_vuelta_algoritmos()

        # El número de fotograma lo pone la cámara emisora y se guarda por
        # cámara: la instancia del algoritmo de esta cámara escribe su log en la
        # misma escala, así que sus líneas [FRAME n] casan con la imagen de
        # debug que lleva ese mismo número quemado
        self.ultimo_frame_camara[camara] = msg.n_frame

        # 🏎️ MAGIA: el algoritmo se encarga de detectar el derrape y pedir la velocidad
        nueva_vel = self.algoritmos[camara].actualizar_estado(punto_front, punto_back, msg.n_frame, self.vueltas)

        # Si el algoritmo nos dice que ignoremos el frame por ruido, nueva_vel será None
        if nueva_vel is not None:
            self.v_actual = nueva_vel

        # En modo incremental el PWM NO lo decide el algoritmo: es el mismo en
        # todo el circuito y sube un escalón cada vueltas_incremento vueltas. Se
        # calcula aquí, después de actualizar_estado, para que el algoritmo haya
        # corrido igual y su log siga guardando lo que él habría aplicado. Es
        # función pura del contador de vueltas, así que da igual que se pierdan
        # mensajes: no hay estado que se pueda descuadrar.
        if self.modo == "incremental":
            escalones = self.vueltas // self.vueltas_incremento
            self.v_actual = min(
                self.v_max, self.v_min + self.incremento_vuelta * escalones)

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
                self.log_algoritmo(f"👁️ Cámara activa inicial: {camara}")
            elif camara == self.camara_activa:
                self.t_ultimo_frame_valido = time_received_from_camera
            else:
                sin_activa = (
                    time_received_from_camera - self.t_ultimo_frame_valido
                ).nanoseconds / 1e9
                if sin_activa > self.timeout_camara_activa:
                    # La activa dejó de ver el coche: conmutamos. Se cierra
                    # cualquier derrape que tuviera abierto (no van a llegar
                    # más frames que lo extiendan) y se apunta el orden.
                    #
                    # OJO con el número de frame: este es el ÚNICO sitio donde
                    # el evento lo dispara un mensaje de una cámara (la nueva)
                    # pero se escribe en el log de OTRA (la activa, que es la
                    # dueña de esta instancia del algoritmo). Cada cámara tiene
                    # su propia numeración, empezada a contar cuando arrancó su
                    # nodo, así que meter aquí msg.n_frame dejaría en el log de
                    # la cámara antigua un número de otra escala — y no se
                    # queda en la línea del aviso: _cerrar_derrape reusa ese
                    # mismo número para la línea CERRADO, que el análisis mete
                    # en la tabla de derrapes y dibuja sobre el eje de frames.
                    # El derrape cerrado por conmutación (justo el caso
                    # interesante del sistema distribuido) acabaría marcado en
                    # una posición inventada. Se pasa el último frame que la
                    # cámara que pierde el coche vio de verdad, que además es
                    # lo que el evento describe. Desarrollado en
                    # NUMERACION_FRAMES.md.
                    self.algoritmos[self.camara_activa].notificar_perdida_vision(
                        self.vueltas, self.ultimo_frame_camara[self.camara_activa]
                    )
                    self.camara_precedente[camara] = self.camara_activa
                    self.log_algoritmo(
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
                self.log_algoritmo(
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
            self.log_algoritmo(
                f"🚦 PERFIL ACTUANDO: PWM: {self.v_actual} (perfil {self.v_min}-{self.v_max})"
            )
            self.ultimo_pwm_enviado = self.v_actual

    def registrar_vuelta_algoritmos(self):
        # Una vuelta es limpia si NINGUNA camara registro derrapes en ella.
        # Es informacion de CONTEXTO, no una condicion: cada instancia sube
        # sus zonas una a una segun su propia proteccion (ver registrar_vuelta).
        # Antes esto era una puerta global y un derrape en cualquier punto del
        # circuito bloqueaba la subida de todo el trazado, que es justo lo que
        # el aprendizaje por zonas viene a evitar.
        derrapes_totales = sum(a.derrapes_contador for a in self.algoritmos.values())
        vuelta_limpia = derrapes_totales == self.derrapes_ultima_vuelta
        self.derrapes_ultima_vuelta = derrapes_totales
        for algoritmo in self.algoritmos.values():
            algoritmo.registrar_vuelta(self.vueltas, vuelta_limpia)
        if vuelta_limpia:
            self.log_algoritmo("📈 Vuelta limpia: sin derrapes en ninguna cámara.")
        else:
            self.log_algoritmo(
                "📉 Vuelta con derrapes: suben las zonas no protegidas; "
                "las castigadas esperan."
            )

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
        # ÚNICO punto por el que sale una orden de PWM del nodo (lo llaman
        # ejecutar_control_carrera y el reinicio de calibración), así que
        # cortar aquí corta el control entero. En modo manual manda el gatillo
        # de la persona: publicar sería, en el mejor de los casos, ruido en el
        # bag, y si alguien levantase el arduino_bridge por error el coche se
        # pondría a correr solo en mitad de la carrera manual.
        if self.modo_manual:
            return

        msg_vel = SpeedCarril()
        msg_vel.pwm = pwm
        # Carril FISICO de este coche (params.yaml -> cars.<coche>.carril, que
        # el launch pasa como carril_asignado). Antes iba fijo a "2" porque el
        # Arduino de pruebas estaba en ese carril; con varios coches cada uno
        # tiene el suyo y hay que respetarlo o dos coches irian al mismo carril.
        msg_vel.carril = str(self.carril)
        msg_vel.stamp = self.get_clock().now().to_msg()
        self.pub_pwm.publish(msg_vel)

    def log_algoritmo(self, texto):
        """Mensajes de terminal que solo interesan cuando el algoritmo manda.

        En modo manual la terminal es lo único que ve la persona mientras
        conduce, y estos mensajes (el PWM en cada cambio de zona, las
        conmutaciones de cámara, los derrames) enterrarían los tiempos de
        vuelta entre decenas de líneas por minuto. No se pierde nada: todo
        sigue escribiéndose en logs_carrera/derrapesLog_*.txt, que es de donde
        lo lee el dashboard de análisis.
        """
        if not self.modo_manual:
            self.get_logger().info(texto)

    def mostrar_panel_piloto(self, lap_time, lap_number):
        """El 'salpicadero' del modo manual: lo que necesita ver de reojo quien
        está conduciendo — qué vuelta acaba de cerrar, en cuánto, si ha
        mejorado respecto a la anterior y cuál es su mejor tiempo.

        Las flechas van al revés de lo intuitivo (▼ = menos tiempo = mejor),
        así que siempre acompañan al signo del delta, que es lo que se lee de
        verdad al pasar por meta.
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
        msg_time_per_lap = TimePerLap()
        msg_time_per_lap.lap_time = lap_time
        msg_time_per_lap.lap_number = lap_number
        msg_time_per_lap.stamp = self.get_clock().now().to_msg()
        self.pub_time_per_lap.publish(msg_time_per_lap)

    def distancia_con_signo(self, P, A, B):
        # Distancia perpendicular de P a la RECTA que pasa por A y B, con
        # signo: el signo dice de qué lado de la recta cae P, y por eso un
        # cambio de signo entre dos fotogramas significa que el coche cruzó.
        # d(P) = cross(B-A, P-A) / |B-A|, con el producto cruzado 2D escrito
        # a mano (np.cross con vectores 2D está retirado en NumPy 2.x)
        AB = B - A
        AP = P - A
        return float(AB[0] * AP[1] - AB[1] * AP[0]) / float(np.linalg.norm(AB))

    def verificar_linea_meta(self, camara_id, p_front, stamp, n_frame):
        """
        Procesa un mensaje de la cámara que ve la meta y devuelve True si
        con él se ha completado una vuelta.

        El cruce es el CAMBIO DE SIGNO de la distancia de la pegatina
        delantera a la recta de meta entre el fotograma anterior y este
        (ver el bloque de comentarios del __init__). Si lo hay, el instante
        exacto se interpola dentro de ese intervalo, proporcionalmente a lo
        cerca que estaba la delantera de la recta en cada extremo.

        Solo entra aquí la cámara que ve la meta: el resto sale en la primera
        condición. La pegatina trasera no interviene: basta con la delantera.
        """
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

        d_curr = self.distancia_con_signo(p_front, A_np, B_np)

        vuelta_completada = False

        if self.front_meta_anterior is None:
            # Primer mensaje de la cámara de meta: no hay "antes" con el que
            # comparar el signo. Se avisa una sola vez y se guarda abajo.
            self.get_logger().info(
                "🏁 Primer fotograma de la cámara de meta recibido: "
                "empieza la vigilancia del cruce de la línea."
            )
        elif not (0 < n_frame - self.front_meta_anterior[3] <= MAX_SALTO_FRAMES_META):
            # Lo guardado no es el fotograma anterior: entre medias la cámara
            # procesó fotogramas en los que no vio el coche (salió del
            # encuadre y le dio la vuelta al circuito por fuera), así que
            # comparar los lados diría que "cruzó" sin haber pasado por la
            # meta. Se re-siembra con este punto y ya se compara con el
            # siguiente, que sí será el de al lado.
            #
            # La condición es una sola a propósito: el "0 <" cubre además el
            # n_frame que retrocede cuando respawn relanza el nodo cámara y su
            # contador vuelve a empezar, sin un caso aparte para eso.
            self.front_meta_anterior = (p_front.copy(), t_stamp, d_curr, n_frame)
            return False
        else:
            p_prev, t_prev, d_prev, _ = self.front_meta_anterior

            # d == 0 es la delantera JUSTO encima de la recta, y pasa de
            # verdad: el centro de la pegatina es x + w/2 del boundingRect,
            # o sea múltiplos de 0,5, los extremos de la meta son enteros
            # (find_finish_line los saca con //2) y la meta suele quedar casi
            # vertical u horizontal, así que en cuanto la delantera cae en
            # esa columna el producto da 0 exacto. Por eso el cambio de lado
            # se comprueba con <= 0 (tocar la línea ya
            # cuenta como cruce, con s=1, o sea este mismo fotograma) y se
            # exige d_prev != 0 para que el fotograma siguiente no lo cuente
            # otra vez. Con < 0 estricto, un fotograma sobre la línea hacía
            # desaparecer la vuelta entera sin dejar rastro.
            if d_prev != 0.0 and d_prev * d_curr <= 0.0:
                # La delantera cambió de lado: cruzó la recta de meta en
                # algún punto entre el fotograma anterior y este.
                # Fracción del intervalo recorrida hasta tocar la recta
                # (0 = justo en el anterior, 1 = justo en este)
                s = abs(d_prev) / (abs(d_prev) + abs(d_curr))
                # Punto donde tocó la recta. La meta es un SEGMENTO, no una
                # recta infinita: si el punto de cruce cae lejos de él, el
                # coche cruzó la prolongación de la línea por otra parte del
                # circuito y esto NO es un paso por meta. Como el punto está
                # sobre la recta, su distancia al segmento es 0 mientras
                # caiga dentro y umbral_meta actúa solo como tolerancia más
                # allá de los extremos.
                p_cruce = p_prev + s * (p_front - p_prev)
                dist_meta = self.distancia_punto_segmento(p_cruce, A_np, B_np)

                if dist_meta > self.umbral_meta:
                    self.get_logger().warn(
                        f"🏁 Cambio de lado a {dist_meta:.0f} px del segmento de "
                        f"meta (> {self.umbral_meta:.0f}): el coche cruzó la "
                        "prolongación de la línea, no la meta. No cuenta."
                    )
                else:
                    # Instante exacto del cruce, interpolado entre los stamps
                    # de CAPTURA de los dos fotogramas
                    t_meta = t_prev + s * (t_stamp - t_prev)

                    if self.t_meta_anterior is None:
                        # Primera pasada: solo arranca el cronómetro. En
                        # calibración es además el punto donde empieza a
                        # grabarse la trayectoria base.
                        self.get_logger().info(
                            "🏁 Primera pasada por meta. Iniciando cronómetro..."
                        )
                    else:
                        # El tiempo de vuelta es la diferencia entre los dos
                        # instantes INTERPOLADOS de paso por meta
                        self.vueltas += 1
                        lap_time = (t_meta - self.t_meta_anterior) / 1e9
                        self.publicar_time_lap(lap_time, self.vueltas)
                        if self.modo_manual:
                            # Conduciendo no se lee un log: se mira la pantalla
                            # de reojo al pasar por meta y hay que ver en el
                            # acto si se mejoró y cuál es el mejor tiempo
                            self.mostrar_panel_piloto(lap_time, self.vueltas)
                        else:
                            self.get_logger().info(
                                f"⏱️ ¡VUELTA {self.vueltas} COMPLETADA! Tiempo: {lap_time:.3f} s"
                            )
                        vuelta_completada = True

                    self.t_meta_anterior = t_meta

        # Guardamos SIEMPRE los datos de la delantera: en el próximo
        # fotograma serán el "antes" con el que comparar el signo (el
        # equivalente a trayectoria[-1] en el código de Mario)
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
