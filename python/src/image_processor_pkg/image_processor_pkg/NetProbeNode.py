#!/usr/bin/python3
"""
Permite medir la latencia de red del sistema

Manda un echo periodico a cada camara, cronometra lo que tarda en volver la
respuesta y escribe un CSV por camara
"""

import rclpy
from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from image_processor_pkg.msg import NetProbe, NetProbeEcho

import functools
import os
import time


class NetProbeNode(Node):

    def __init__(self):
        super().__init__("net_probe")

        self.declare_parameter("net_probe.enabled", True)
        self.enabled = self.get_parameter("net_probe.enabled").value

        if not self.enabled:
            self.get_logger().warn(
                "[NETPROBE] Desactivado (net_probe.enabled = False): no se manda "
                "ningun echo ni se escribe ningun CSV"
            )
            return

        # ---- PARAMETROS ----
        
        # Camaras a las que se manda el echo
        # Los nombres tienen que ser los mismos node_id con los que se levantan
        self.declare_parameter("net_probe.camaras", ["camara_01"])
        self.camaras = self.get_parameter("net_probe.camaras").value

        # Cada cuanto se manda un echo
        self.declare_parameter("net_probe.frecuencia_hz", 5.0)
        self.frecuencia_hz = float(self.get_parameter("net_probe.frecuencia_hz").value)

        # Relleno para que el paquete pese lo mismo que un CarLocation
        self.declare_parameter("net_probe.padding_bytes", 128)
        self.padding_bytes = int(self.get_parameter("net_probe.padding_bytes").value)
        self.padding = bytes(self.padding_bytes)
 
        # Un echo que tarde mas de esto en volver se da por perdido
        self.declare_parameter("net_probe.umbral_perdida_ms", 500.0)
        self.umbral_perdida_ns = int(
            float(self.get_parameter("net_probe.umbral_perdida_ms").value) * 1e6
        )

        # -----------------------------------------------------------

        # ---- CSV ----
        # Nombre que se le pone a los CSV para diferenciarlos
        self.declare_parameter("net_probe.etiqueta", "sin_etiqueta")
        self.etiqueta = self.get_parameter("net_probe.etiqueta").value

        # Ruta donde se guardan los CSV
        self.declare_parameter(
            "net_probe.directorio_csv", "/ros2_ws/src/image_processor_pkg/logs_red"
        )
        self.directorio_csv = self.get_parameter("net_probe.directorio_csv").value
        os.makedirs(self.directorio_csv, exist_ok=True)
        # -----------------------------------------------------------

        self.grupo_sondeo = MutuallyExclusiveCallbackGroup()

        # -----------------------------------------------------------
        
        # ---- ESTADO POR CAMARA ----
        # Un publicador, una suscripcion, un contador y un fichero por camara
        
        self.origen = self.get_name()
        self.publicadores = {}
        self.suscripciones = {}
        self.seq = {}
        # Echos que no volvieron
        self.pendientes = {}
        # RTT desde el ultimo resumen, se vacia cada 10 s
        self.muestras_ventana = {}
        self.enviados_ventana = {}
        self.perdidos_ventana = {}
        self.enviados_total = {}
        self.perdidos_total = {}
        self.ficheros_csv = {}

        for camara in self.camaras:
            # Publisher de /<camara>/net_probe -> NetProbe.msg
            self.publicadores[camara] = self.create_publisher(
                NetProbe, f"/{camara}/net_probe", qos_profile_sensor_data
            )

            # Subscriber de /<camara>/net_probe_echo -> NetProbeEcho.msg
            # Es la respuesta que devuelve la camara
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
            # de una ejecucion, las muestras de antes de la caida no se pierden
            nuevo = not os.path.exists(ruta) or os.path.getsize(ruta) == 0

            f = open(ruta, "a", buffering=1)  # buffering=1: linea a linea

            if nuevo:
                f.write(
                    "seq,destino,t_send_ns,t_recv_ns,rtt_bruto_ns,"
                    "proc_ns,rtt_neto_ns,perdido\n"
                )

            self.ficheros_csv[camara] = f
            self.get_logger().info(f"[NETPROBE] Echos de {camara} -> {ruta}")
        # -----------------------------------------------------------

        # ---- TIMERS ----
        
        # Timer que manda los echos
        self.timer_sondeo = self.create_timer(
            1.0 / self.frecuencia_hz, self.enviar_sondeos, callback_group=self.grupo_sondeo
        )

        # Muestra un resumen cada 10 s
        self.timer_resumen = self.create_timer(
            10.0, self.log_resumen, callback_group=self.grupo_sondeo
        )
        # -----------------------------------------------------------

        self.get_logger().info(
            f"[NETPROBE] Listo [{self.etiqueta}]: {len(self.camaras)} camara(s) "
            f"a {self.frecuencia_hz:.1f} Hz, {self.padding_bytes} B de relleno, "
            f"umbral de perdida {self.umbral_perdida_ns / 1e6:.0f} ms"
        )

    # -----------------------------------------------------------------
    # Envio
    # -----------------------------------------------------------------
    def enviar_sondeos(self):
        """Manda un echo a cada camara y comprueba los que no volvieron"""

        # Primero se comprueban los que no volvieron
        self.barrer_perdidos()

        # Segundo se manda un echo nuevo a cada camara
        for camara in self.camaras:
            msg = NetProbe()
            msg.seq = self.seq[camara]
            msg.origen = self.origen
            msg.destino = camara
            # Solo informativo, la medida se hace con el reloj monotonico
            msg.t_send = self.get_clock().now().to_msg()
            msg.padding = self.padding

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
        """Callback que se llama cada vez que llega la respuesta de una camara"""
        t_recv_ns = time.monotonic_ns()

        t_send_ns = self.pendientes[camara].pop(msg.seq, None)
        if t_send_ns is None:
            # La respuesta llega despues de dar el echo por perdido, o esta
            # duplicada, y se descarta
            self.get_logger().warn(
                f"[NETPROBE] Respuesta tardia o repetida de {camara} (seq {msg.seq}): "
                f"llego despues del umbral de perdida, se ignora"
            )

            return

        rtt_bruto_ns = t_recv_ns - t_send_ns
        # Lo que la camara tardo dentro de su callback
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
        """Da por perdidos los echos que llevan mas del umbral sin volver"""

        ahora_ns = time.monotonic_ns()
        for camara in self.camaras:
            
            # Se comprueban los echos que superan el umbral
            vencidos = [
                seq
                for seq, t_send_ns in self.pendientes[camara].items()
                if ahora_ns - t_send_ns > self.umbral_perdida_ns
            ]

            # Se eliminan los echos que superaron el umbral
            for seq in vencidos:
                t_send_ns = self.pendientes[camara].pop(seq)
                self.perdidos_ventana[camara] += 1
                self.perdidos_total[camara] += 1
                # Los campos de tiempo van vacios a proposito: no hay t_recv
                # porque la respuesta no llego, y un 0 invitaria a que el
                # analisis lo tomase por un RTT de cero
                self.escribir_fila(camara, seq, t_send_ns, "", "", "", "", 1)

    # -----------------------------------------------------------------
    # Salida
    # -----------------------------------------------------------------
    def escribir_fila(
        self, camara, seq, t_send_ns, t_recv_ns, rtt_bruto_ns, proc_ns, rtt_neto_ns, perdido
    ):
        """
        Escribe una fila en el fichero CSV
        """

        self.ficheros_csv[camara].write(
            f"{seq},{camara},{t_send_ns},{t_recv_ns},{rtt_bruto_ns},"
            f"{proc_ns},{rtt_neto_ns},{perdido}\n"
        )

    def log_resumen(self):
        """Muestra por terminal un resumen de los ultimos 10 s"""

        for camara in self.camaras:
            muestras = self.muestras_ventana[camara]
            enviados = self.enviados_ventana[camara]
            perdidos = self.perdidos_ventana[camara]

            if not muestras:
                if enviados:
                    self.get_logger().error(
                        f"[NETPROBE] {camara}: ninguna respuesta en 10 s ({enviados} "
                        f"echos enviados). O la camara no esta, o tiene el eco apagado, "
                        f"o el enlace esta caido"
                    )
            else:
                orden = sorted(muestras)
                mediana_ms = self.percentil(orden, 50) / 1e6
                p95_ms = self.percentil(orden, 95) / 1e6
                # El porcentaje se calcula sobre los enviados de la ventana, no
                # sobre los que volvieron
                perdida_pct = 100.0 * perdidos / enviados if enviados else 0.0

                self.get_logger().info(
                    f"[NETPROBE] {camara}: RTT neto mediana {mediana_ms:.2f} ms | "
                    f"p95 {p95_ms:.2f} ms | perdida {perdida_pct:.1f}% "
                    f"({perdidos}/{enviados}) | n={len(muestras)}"
                )

            self.muestras_ventana[camara] = []
            self.enviados_ventana[camara] = 0
            self.perdidos_ventana[camara] = 0

    @staticmethod
    def percentil(ordenados, p):
        if not ordenados:
            return 0
        i = int(round((p / 100.0) * (len(ordenados) - 1)))
        return ordenados[i]

    # -----------------------------------------------------------------
    def destroy_node(self):
        """Resumen de toda la ejecucion y cierre de los CSV"""
        if self.enabled:
            for camara in self.camaras:
                enviados = self.enviados_total[camara]
                perdidos = self.perdidos_total[camara]
                pct = 100.0 * perdidos / enviados if enviados else 0.0
                self.get_logger().info(
                    f"[NETPROBE] {camara} [{self.etiqueta}]: {enviados} echos, "
                    f"{perdidos} perdidos ({pct:.1f}%)"
                )
                self.ficheros_csv[camara].close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    net_probe = NetProbeNode()

    try:
        rclpy.spin(net_probe)
    except KeyboardInterrupt:
        pass  
    finally:
        net_probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
