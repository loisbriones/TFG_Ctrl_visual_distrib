#!/usr/bin/python3
"""
Medidor de latencia de red del sistema distribuido.

Manda sondeos periodicos a cada camara y cronometra lo que tardan en volver.
Corre en el PC, junto al controlador y al puente Arduino, pero es un NODO
APARTE a proposito: metido dentro del CarControllerNode, su timer competiria
por el hilo del car_position_group con el callback de posicion, que es el lazo
de control. La instrumentacion no puede perturbar lo que mide.

POR QUE IDA Y VUELTA Y NO IDA SOLA
----------------------------------
Aqui no hay NTP en ninguna parte, y es deliberado: todo el sistema esta
disenado para que ninguna resta cruce dos relojes (los lap_time salen de dos
stamps de la MISMA Raspberry, el pipeline_time de dos lecturas del reloj del
PC...). Medir la latencia de ida sola romperia esa regla: haria falta restar
un instante tomado en el PC menos otro tomado en la Raspberry, y el desfase
entre los dos relojes se sumaria entero al resultado. Ese desfase llega a
±25 ms entre sesiones, del mismo orden de magnitud que la latencia WiFi que
queremos medir: la cifra no significaria nada, y de hecho pueden salir
latencias negativas.

Con un viaje de ida y vuelta el problema desaparece: las dos marcas de tiempo
(salida y llegada) las toma el MISMO reloj, el de este nodo, asi que el
desfase se cancela solo al restar. Es lo que recomienda el RFC 2681 en su
seccion 2.7.1. Lo que se paga es que se mide la suma de los dos sentidos: la
latencia de un sentido se estima como RTT/2, asumiendo que la red es simetrica.

POR QUE EL RELOJ MONOTONICO Y NO get_clock().now()
--------------------------------------------------
Las dos marcas se toman con time.monotonic_ns(), no con el reloj del nodo. El
reloj del sistema puede DAR UN SALTO si NTP (o un ajuste manual de la hora)
lo corrige a mitad de la medida, y ese salto se colaria entero dentro de un
RTT como si fuese latencia. Es exactamente el riesgo del que avisa el RFC
2681. El monotonico no salta nunca: solo avanza. Como aqui solo se hacen
RESTAS entre dos lecturas suyas, no importa que su origen sea arbitrario.

QUE SE MIDE Y QUE SE DESCUENTA
------------------------------
  rtt_bruto_ns = t_recv - t_send        (los dos, reloj monotonico de aqui)
  rtt_neto_ns  = rtt_bruto_ns - proc_ns

proc_ns es lo que la camara tardo dentro de su callback, medido por ella con
SU reloj monotonico. Es una duracion, no un instante, asi que restarla es
legitimo aunque venga de otra maquina. Se descuenta porque es error de medida
(tiempo de la camara, no de la red); el que interesa para el presupuesto de
latencia es el neto.

La salida es un CSV por camara, una fila por sondeo:

  seq,destino,t_send_ns,t_recv_ns,rtt_bruto_ns,proc_ns,rtt_neto_ns,perdido

Es el dato en crudo y se deja asi a proposito. El analisis (medianas,
percentiles, porcentaje de perdida) se hace fuera del repositorio, sobre esos
CSV: depende de la campana concreta que se este midiendo, no del sistema. Las
filas de sondeos perdidos llevan los tiempos VACIOS y no a cero, para que no
entren en ninguna media por descuido.
"""

import rclpy
from rclpy.node import Node

# El mismo perfil que usa el camino de datos real (/<coche>/position y la
# telemetria): best effort, keep last, depth 5. Es importante que sea el mismo
# para que el sondeo sufra lo mismo que los datos — con un perfil RELIABLE, DDS
# retransmitiria los paquetes perdidos y la medida saldria optimista y sin
# perdidas, justo lo contrario de lo que queremos caracterizar.
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from image_processor_pkg.msg import NetProbe, NetProbeEcho

import functools
import os
import time


class NetProbeNode(Node):
    def __init__(self):
        super().__init__("net_probe")

        # ---- INTERRUPTOR GENERAL ----
        # Con enabled: False el nodo arranca, avisa y no crea NADA: ni
        # publicadores, ni suscripciones, ni timers, ni ficheros. Asi se puede
        # dejar en el launch de forma permanente y encenderlo solo los dias de
        # medicion, sin desmontar nada ni tocar BrainLaunch.py.
        self.declare_parameter("net_probe.enabled", True)
        self.enabled = self.get_parameter("net_probe.enabled").value

        if not self.enabled:
            self.get_logger().warn(
                "🔇 NetProbe DESACTIVADO (net_probe.enabled = False): no se "
                "manda ningun sondeo ni se escribe ningun CSV."
            )
            return

        # ---- CAMARAS A SONDEAR ----
        # Los nombres tienen que ser los mismos que el node_id con el que se
        # levanta cada CameraNode (camara_01, camara_02...), porque de ahi sale
        # su namespace: el eco escucha en /<node_id>/net_probe.
        self.declare_parameter("net_probe.camaras", ["camara_01"])
        self.camaras = self.get_parameter("net_probe.camaras").value

        # ---- CADENCIA ----
        # Timer FIJO: se manda un sondeo por camara cada tick, pase lo que
        # pase. No se adapta al trafico ni espera al eco anterior; si un sondeo
        # se pierde, el siguiente sale igual a su hora. Una cadencia constante
        # es lo que permite que el porcentaje de perdida signifique algo.
        # 5 Hz es un compromiso: suficientes muestras para una mediana y un p95
        # decentes en una ejecucion de 30 vueltas, y muy por debajo de los
        # 30 Hz del camino de datos real, para no falsear lo que se mide
        # metiendo trafico propio en la misma red.
        self.declare_parameter("net_probe.frecuencia_hz", 5.0)
        self.frecuencia_hz = float(self.get_parameter("net_probe.frecuencia_hz").value)

        # ---- TAMANO DEL SONDEO ----
        # Relleno para que el paquete pese lo mismo que un CarLocation real y
        # atraviese la red en las mismas condiciones (fragmentacion, colas).
        self.declare_parameter("net_probe.padding_bytes", 128)
        self.padding_bytes = int(self.get_parameter("net_probe.padding_bytes").value)
        # Se construye una sola vez: es siempre el mismo relleno de ceros y no
        # tiene sentido reservarlo en cada tick
        self.padding = bytes(self.padding_bytes)

        # ---- UMBRAL DE PERDIDA ----
        # Un sondeo que lleve mas de esto sin volver se da por perdido: se
        # apunta con perdido=1 y se saca del calculo de latencia. Si se contase
        # como "muy lento" en vez de como perdido, la cola de la distribucion
        # quedaria contaminada por sondeos que en realidad no llegaron nunca y
        # el p95 mediria el umbral, no la red.
        self.declare_parameter("net_probe.umbral_perdida_ms", 500.0)
        self.umbral_perdida_ns = int(
            float(self.get_parameter("net_probe.umbral_perdida_ms").value) * 1e6
        )

        # ---- ETIQUETA DE LA EJECUCION ----
        # Va en el nombre del CSV para poder separar despues los escenarios del
        # protocolo de pruebas (A1_rep1, A2_rep3, B_100ms...). Sin ella, dos
        # ejecuciones seguidas escribirian en el mismo fichero y quedarian
        # mezcladas. Se cambia entre ejecuciones en params.yaml.
        self.declare_parameter("net_probe.etiqueta", "sin_etiqueta")
        self.etiqueta = self.get_parameter("net_probe.etiqueta").value

        # ---- DONDE SE ESCRIBEN LOS CSV ----
        # Por defecto, dentro del paquete, igual que los logs del algoritmo:
        # docker-compose-brain.yml monta src/image_processor_pkg desde el host,
        # asi que lo que se escriba ahi sobrevive a parar el contenedor y se
        # puede leer sin entrar en el.
        self.declare_parameter(
            "net_probe.directorio_csv", "/ros2_ws/src/image_processor_pkg/logs_red"
        )
        self.directorio_csv = self.get_parameter("net_probe.directorio_csv").value
        os.makedirs(self.directorio_csv, exist_ok=True)

        # ---- GRUPO DE CALLBACKS ----
        # UN SOLO grupo mutuamente exclusivo para los dos timers y para todos
        # los ecos, y el nodo gira con rclpy.spin (un hilo). Es intencionado:
        # asi el tick de envio, la llegada de un eco y el resumen periodico no
        # se solapan nunca, y las estructuras compartidas (pendientes,
        # muestras, contadores) no necesitan ningun lock. El trabajo de cada
        # callback es una resta y una linea de CSV, asi que serializarlo no
        # cuesta nada. Es lo contrario que en la camara, donde el eco SI
        # necesita su propio hilo porque compite con el procesado del fotograma.
        self.grupo_sondeo = MutuallyExclusiveCallbackGroup()

        # ---- ESTADO POR CAMARA ----
        # Un publicador, una suscripcion, un contador de secuencia y un fichero
        # por camara: cada enlace PC-camara es una medida independiente y
        # mezclarlas daria una media sin sentido si una va por cable y otra por
        # WiFi.
        self.origen = self.get_name()
        self.publicadores = {}
        self.suscripciones = {}
        self.seq = {}
        # {camara: {seq: t_send_ns}} — sondeos enviados que aun no han vuelto.
        # De aqui salen tanto el RTT (al llegar el eco) como las perdidas (al
        # barrerlo buscando los que ya pasaron del umbral)
        self.pendientes = {}
        # Ventana de RTT netos (ns) desde el ultimo resumen: se vacia en cada
        # log de 10 s para que lo que se ve por pantalla sea el estado ACTUAL
        # del enlace y no una media arrastrada desde el arranque
        self.muestras_ventana = {}
        self.enviados_ventana = {}
        self.perdidos_ventana = {}
        # Acumulados de toda la ejecucion, solo para el resumen final al cerrar
        self.enviados_total = {}
        self.perdidos_total = {}
        self.ficheros_csv = {}

        for camara in self.camaras:
            self.publicadores[camara] = self.create_publisher(
                NetProbe, f"/{camara}/net_probe", qos_profile_sensor_data
            )
            # functools.partial fija la camara: create_subscription entrega
            # solo el mensaje, y aunque el eco trae el campo 'destino', fiarse
            # del topic por el que llego es mas robusto que fiarse del contenido
            self.suscripciones[camara] = self.create_subscription(
                NetProbeEcho,
                f"/{camara}/net_probe_echo",
                functools.partial(self.callback_eco, camara),
                qos_profile_sensor_data,
                callback_group=self.grupo_sondeo,
            )

            self.seq[camara] = 0
            self.pendientes[camara] = {}
            self.muestras_ventana[camara] = []
            self.enviados_ventana[camara] = 0
            self.perdidos_ventana[camara] = 0
            self.enviados_total[camara] = 0
            self.perdidos_total[camara] = 0

            ruta = os.path.join(
                self.directorio_csv, f"netprobe_{self.etiqueta}_{camara}.csv"
            )
            # Modo "a" y no "w": si el nodo se cae y respawn lo relanza a mitad
            # de una ejecucion, las muestras de antes de la caida no se pierden.
            # Por eso la cabecera solo se escribe si el fichero esta vacio.
            nuevo = not os.path.exists(ruta) or os.path.getsize(ruta) == 0
            f = open(ruta, "a", buffering=1)  # buffering=1: linea a linea
            if nuevo:
                f.write(
                    "seq,destino,t_send_ns,t_recv_ns,rtt_bruto_ns,"
                    "proc_ns,rtt_neto_ns,perdido\n"
                )
            self.ficheros_csv[camara] = f
            self.get_logger().info(f"📈 Sondeos de {camara} -> {ruta}")

        # ---- TIMERS ----
        self.timer_sondeo = self.create_timer(
            1.0 / self.frecuencia_hz, self.enviar_sondeos, callback_group=self.grupo_sondeo
        )
        # Resumen en vivo cada 10 s: sirve para abortar una ejecucion mala en
        # cuanto se ve, en vez de descubrirlo al analizar los CSV media hora
        # despues (un WiFi con el ahorro de energia puesto se detecta aqui)
        self.timer_resumen = self.create_timer(
            10.0, self.log_resumen, callback_group=self.grupo_sondeo
        )

        self.get_logger().info(
            f"📡 NetProbe listo [{self.etiqueta}]: {len(self.camaras)} camara(s) "
            f"a {self.frecuencia_hz:.1f} Hz, {self.padding_bytes} B de relleno, "
            f"umbral de perdida {self.umbral_perdida_ns / 1e6:.0f} ms"
        )

    # -----------------------------------------------------------------
    # Envio
    # -----------------------------------------------------------------
    def enviar_sondeos(self):
        """Un sondeo a cada camara, y de paso barrido de los que no volvieron."""
        # El barrido va ANTES del envio para que un sondeo recien mandado no se
        # pueda dar por perdido en su propio tick
        self.barrer_perdidos()

        for camara in self.camaras:
            msg = NetProbe()
            msg.seq = self.seq[camara]
            msg.origen = self.origen
            msg.destino = camara
            # Marca de tiempo INFORMATIVA, con el reloj del nodo: es la que
            # viaja en el mensaje y vuelve sin tocar, util para localizar el
            # sondeo dentro del bag. NO es con la que se mide: para eso esta el
            # monotonico de aqui abajo, que nadie mas ve
            msg.t_send = self.get_clock().now().to_msg()
            msg.padding = self.padding

            # El monotonico se lee lo mas pegado posible al publish, y despues
            # de haber construido el mensaje: el tiempo de rellenar el padding
            # no es latencia de red y no tiene por que entrar en el RTT
            t_send_ns = time.monotonic_ns()
            self.publicadores[camara].publish(msg)

            self.pendientes[camara][msg.seq] = t_send_ns
            self.seq[camara] += 1
            self.enviados_ventana[camara] += 1
            self.enviados_total[camara] += 1

    # -----------------------------------------------------------------
    # Recepcion
    # -----------------------------------------------------------------
    def callback_eco(self, camara, msg: NetProbeEcho):
        # PRIMERA linea del callback, sin nada antes: cualquier cosa que se
        # hiciera aqui arriba se sumaria al RTT como si fuese red
        t_recv_ns = time.monotonic_ns()

        t_send_ns = self.pendientes[camara].pop(msg.seq, None)
        if t_send_ns is None:
            # El eco llego DESPUES de que su sondeo se diera por perdido (o es
            # un duplicado). No se puede calcular su RTT porque su marca de
            # salida ya no esta, y contarlo ahora descuadraria las cuentas: ya
            # se apunto como perdido. Solo se avisa.
            self.get_logger().warn(
                f"↩️ Eco tardio o repetido de {camara} (seq {msg.seq}): "
                f"llego despues del umbral de perdida, se ignora"
            )
            return

        rtt_bruto_ns = t_recv_ns - t_send_ns
        # Lo que la camara tardo en su callback: es error de medida, no red
        proc_ns = int(msg.proc_ns)
        rtt_neto_ns = rtt_bruto_ns - proc_ns

        self.muestras_ventana[camara].append(rtt_neto_ns)
        self.escribir_fila(
            camara, msg.seq, t_send_ns, t_recv_ns, rtt_bruto_ns, proc_ns, rtt_neto_ns, 0
        )

    # -----------------------------------------------------------------
    # Perdidas
    # -----------------------------------------------------------------
    def barrer_perdidos(self):
        """Da por perdidos los sondeos que llevan mas del umbral sin volver."""
        ahora_ns = time.monotonic_ns()
        for camara in self.camaras:
            # Se recorre una copia de las claves: el bucle borra del diccionario
            vencidos = [
                seq
                for seq, t_send_ns in self.pendientes[camara].items()
                if ahora_ns - t_send_ns > self.umbral_perdida_ns
            ]
            for seq in vencidos:
                t_send_ns = self.pendientes[camara].pop(seq)
                self.perdidos_ventana[camara] += 1
                self.perdidos_total[camara] += 1
                # Los campos de tiempo van vacios a proposito: no hay t_recv
                # porque el eco no llego, y poner un 0 invitaria a que el
                # analisis lo tomase por un RTT de cero
                self.escribir_fila(camara, seq, t_send_ns, "", "", "", "", 1)

    # -----------------------------------------------------------------
    # Salida
    # -----------------------------------------------------------------
    def escribir_fila(
        self, camara, seq, t_send_ns, t_recv_ns, rtt_bruto_ns, proc_ns, rtt_neto_ns, perdido
    ):
        """Una fila del CSV, escrita en el momento.

        Incremental a proposito (fichero abierto en modo linea a linea): si la
        ejecucion se corta a mitad —el coche se sale, se va la luz, se para el
        contenedor— lo medido hasta ese momento ya esta en disco. Acumularlo en
        memoria y volcarlo al final significaria perderlo todo justo en las
        ejecuciones que peor van, que son las interesantes."""
        self.ficheros_csv[camara].write(
            f"{seq},{camara},{t_send_ns},{t_recv_ns},{rtt_bruto_ns},"
            f"{proc_ns},{rtt_neto_ns},{perdido}\n"
        )

    def log_resumen(self):
        """Estado de cada enlace en los ultimos 10 s y reinicio de la ventana."""
        for camara in self.camaras:
            muestras = self.muestras_ventana[camara]
            enviados = self.enviados_ventana[camara]
            perdidos = self.perdidos_ventana[camara]

            if not muestras:
                if enviados:
                    self.get_logger().error(
                        f"📡 {camara}: NINGUN eco en 10 s ({enviados} sondeos "
                        f"enviados). O la camara no esta, o tiene el eco apagado, "
                        f"o el enlace esta caido"
                    )
            else:
                orden = sorted(muestras)
                mediana_ms = self.percentil(orden, 50) / 1e6
                p95_ms = self.percentil(orden, 95) / 1e6
                # El porcentaje se calcula sobre los ENVIADOS de la ventana, no
                # sobre los que volvieron: es la definicion util de perdida
                perdida_pct = 100.0 * perdidos / enviados if enviados else 0.0
                self.get_logger().info(
                    f"📡 {camara}: RTT neto mediana {mediana_ms:.2f} ms | "
                    f"p95 {p95_ms:.2f} ms | perdida {perdida_pct:.1f}% "
                    f"({perdidos}/{enviados}) | n={len(muestras)}"
                )

            self.muestras_ventana[camara] = []
            self.enviados_ventana[camara] = 0
            self.perdidos_ventana[camara] = 0

    @staticmethod
    def percentil(ordenados, p):
        """Percentil p (0-100) de una lista YA ORDENADA, por el metodo del
        vecino mas cercano.

        A mano y no con numpy o statistics.quantiles a proposito: esto solo
        alimenta el log en vivo, y aqui interesa mas no arrastrar una
        dependencia dentro del contenedor del cerebro que el matiz de la
        interpolacion. Los percentiles buenos, los de la memoria, los calcula
        pandas despues sobre el CSV."""
        if not ordenados:
            return 0
        i = int(round((p / 100.0) * (len(ordenados) - 1)))
        return ordenados[i]

    # -----------------------------------------------------------------
    def destroy_node(self):
        """Resumen de toda la ejecucion y cierre de los CSV."""
        if self.enabled:
            for camara in self.camaras:
                enviados = self.enviados_total[camara]
                perdidos = self.perdidos_total[camara]
                pct = 100.0 * perdidos / enviados if enviados else 0.0
                self.get_logger().info(
                    f"🏁 {camara} [{self.etiqueta}]: {enviados} sondeos, "
                    f"{perdidos} perdidos ({pct:.1f}%)"
                )
                self.ficheros_csv[camara].close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    net_probe = NetProbeNode()

    try:
        # Un solo hilo (rclpy.spin, no MultiThreadedExecutor): todo el nodo
        # comparte un unico grupo mutuamente exclusivo, asi que un segundo hilo
        # no ejecutaria nada en paralelo y solo aniadiria cambios de contexto.
        # Ver el comentario del grupo_sondeo en __init__.
        rclpy.spin(net_probe)
    except KeyboardInterrupt:
        pass  # Manejo limpio de Ctrl+C
    finally:
        net_probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
