import json
import math
import os
import numpy as np
from datetime import datetime


class EstrategiaPerfil:
    """
    El algoritmo de velocidad. No es un nodo de ROS, el controlador crea una
    instancia por camara y le pasa los parametros de params.yaml.

    Parte la trayectoria que ve esa camara en celdas de paso_celda px, y las
    agrupa en zonas que comparten un mismo PWM. Cada derrape baja el PWM de su
    zona y cada vuelta sube el de las que no esten protegidas, asi que el coche
    aprende a ir mas rapido donde puede y mas despacio donde se sale.

    Todo lo que hace queda escrito en logs_carrera/derrapesLog_<nodo>_<camara>.txt

    El controlador lo usa en este orden: setTrayectoria una vez al calibrar,
    actualizar_estado en cada frame y registrar_vuelta al cruzar la meta
    """

    def __init__(
        self,
        v_max,
        v_min,
        node_name="node",
        camara_id="cam",
        # --- modo de operacion (ver params.yaml -> cars.<coche>.modo) ---
        modo="automatico",
        # --- deteccion de derrape y localizacion ---
        umbral_derrape=12.0,
        max_dist_ruta=80.0,
        margen_extremo_celdas=2,
        # --- construccion de la cadena de celdas ---
        paso_celda=15.0,
        umbral_celda_gigante=150.0,
        umbral_cierre=60.0,
        # --- perfil de zonas y aprendizaje ---
        incremento_vuelta=1.0,
        reduccion_derrape=2.0,
        retroceso_creacion=150.0,
        retroceso_fusion=60.0,
        margen_fusion_celdas=1,
        vueltas_proteccion=2,
    ):
        # Rango de PWM con el que trabaja el algoritmo
        self.v_max = float(v_max)
        self.v_min = float(v_min)

        # De los cuatro modos, solo "politica" cambia lo que hace esta clase:
        # congela el perfil
        self.modo = modo
        self.camara_id = camara_id

        # Deteccion de derrape y localizacion
        self.umbral_derrape = float(umbral_derrape)
        self.max_dist_ruta = float(max_dist_ruta)
        self.margen_extremo_celdas = int(margen_extremo_celdas)

        # Construccion de la cadena de celdas
        self.paso_celda = float(paso_celda)
        self.umbral_celda_gigante = float(umbral_celda_gigante)
        self.umbral_cierre = float(umbral_cierre)

        # Perfil de zonas y aprendizaje
        self.incremento_vuelta = float(incremento_vuelta)
        self.reduccion_derrape = float(reduccion_derrape)
        self.retroceso_creacion = float(retroceso_creacion)
        self.retroceso_fusion = float(retroceso_fusion)
        self.margen_fusion_celdas = int(margen_fusion_celdas)
        self.vueltas_proteccion = int(vueltas_proteccion)

        # ------------------------------------------------------------------
        # Trayectoria base
        # ------------------------------------------------------------------
        # Donde se guarda la trayectoria base que usa el algoritmo
        self.trayectoriaUsada = None
        # Matriz (P, 2) con la posicion de cada celda normal
        self._puntos = None
        # Vector (P,) indice de celda de cada fila de _puntos
        self._celda_de_punto = None
        # Numero total de celdas de la cadena (normales + gigantes)
        self.n_celdas = 0
        # Vector (n_celdas,) bool: True en las celdas gigantes
        self._es_gigante = None
        # Vector (n_celdas,): px que cubre cada celda
        self._long_celdas = None
        # Vector (n_celdas,): px acumulados al inicio de cada celda
        self._long_acum = None
        # Longitud total de la cadena en px
        self._long_total = 0.0
        # True si la cadena es cerrada (la camara ve el circuito completo)
        self._cerrada = False

        # ------------------------------------------------------------------
        # Zonas: lista ordenada de dicts que particionan la cadena
        #   {"ini", "fin" (ambos inclusive), "pwm", "tipo", "vueltas"}
        # ------------------------------------------------------------------
        self.zonas = []
        # Numero de derrapes que se produjeron
        self.derrapes_contador = 0
        # Numero de derrapes de la ultima vuelta
        self._derrapes_vuelta_anterior = 0

        # ------------------------------------------------------------------
        # Estado del derrape en curso
        # ------------------------------------------------------------------
        self._estado_derrapando = False
        # Celda donde empezo el derrape
        self._derrape_idx_ini = 0
        # Celda donde termino el derrape
        self._derrape_idx_fin = 0
        # Distancia de la pegatina trasera a la trayectoria base
        self._dist_derrape = 0.0

        # Px que hay que aplicar en otra camara (derrame)
        self._reduccion_pendiente = 0.0
        # Px de trayectoria que le quedan al coche por delante en esta camara
        self._margen_restante = 0.0
        # True si en el ultimo frame el coche se localizo sobre la ruta
        self._frame_valido = False

        # ------------------------------------------------------------------
        # Ficheros de log
        # ------------------------------------------------------------------
        directorio_logs = "/ros2_ws/src/image_processor_pkg/logs_carrera"
        os.makedirs(directorio_logs, exist_ok=True)

        # Uno por camara
        self.log_file = f"{directorio_logs}/derrapesLog_{node_name}_{camara_id}.txt"

        # ------------------------------------------------------------------
        # Ficheros de politica
        # ------------------------------------------------------------------
        
        base_politica = f"/ros2_ws/src/image_processor_pkg/politica_{node_name}_{camara_id}"
        # Fichero que contiene la politica que se quiere cargar
        self.fichero_politica = f"{base_politica}.json"
        # La plantilla se reescribe en cada arranque con la particion recien hecha
        self.fichero_plantilla = f"{base_politica}_PLANTILLA.json"

        try:
            # Se abre en modo "w" para vaciar el log de la sesion anterior
            with open(self.log_file, "w") as f:
                f.write(f"=== LOG INICIADO {datetime.now().isoformat()} ===\n")
        except IOError as e:
            print(f"Error inicializando log: {e}")

        self.saveLogFile(
            f"[INIT] nodo={node_name} camara={camara_id} modo={self.modo}")
        self.saveLogFile(
            f"[INIT] v_min={self.v_min:.0f} v_max={self.v_max:.0f} | "
            f"umbral_derrape={self.umbral_derrape} max_dist_ruta={self.max_dist_ruta} "
            f"margen_extremo_celdas={self.margen_extremo_celdas}"
        )
        self.saveLogFile(
            f"[INIT] paso_celda={self.paso_celda} "
            f"umbral_celda_gigante={self.umbral_celda_gigante} "
            f"umbral_cierre={self.umbral_cierre}"
        )
        self.saveLogFile(
            f"[INIT] incremento_vuelta={self.incremento_vuelta} "
            f"reduccion_derrape={self.reduccion_derrape} "
            f"retroceso_creacion={self.retroceso_creacion} "
            f"retroceso_fusion={self.retroceso_fusion} "
            f"margen_fusion_celdas={self.margen_fusion_celdas} "
            f"vueltas_proteccion={self.vueltas_proteccion}"
        )

    # ----------------------------------------------------------------------
    # Propiedades de solo lectura: exponen estado interno a CarControllerNode
    # ----------------------------------------------------------------------
    @property
    def dist_derrape(self):
        """Ultima distancia perpendicular (px) de la pegatina trasera a la
        trayectoria. Se publica en CarControlTelemetry"""
        return self._dist_derrape

    @property
    def estado_derrapando(self):
        """True si ahora mismo hay un derrape en curso (sin cerrar)"""
        return self._estado_derrapando

    @property
    def frame_valido(self):
        """True si el ultimo frame procesado localizo el coche sobre la
        trayectoria (no fue descartado por ruido)"""
        return self._frame_valido

    def consumir_reduccion_pendiente(self):
        """Devuelve los px de reduccion pendientes y los pone a cero. El
        controlador la llama tras cada actualizar_estado"""
        pendiente = self._reduccion_pendiente
        self._reduccion_pendiente = 0.0
        return pendiente

    # ======================================================================
    # CONSTRUCCION DE LA CADENA DE CELDAS
    # ======================================================================
    def setTrayectoria(self, trayectoria, varias_camaras=False):
        """
        Convierte la trayectoria de calibracion en la cadena de celdas
        Primero orienta la lista de puntos y despues la remuestrea en celdas
        A partir de aqui ya solo se trabaja con celdas
        """

        # Idempotente, la calibracion solo debe procesarse una vez
        if self.trayectoriaUsada is not None:
            self.saveLogFile(
                "[TRAY] setTrayectoria ignorado: ya hay una trayectoria cargada"
            )
            return

        # Comprobamos que tenga el tamaño suficiente
        puntos = np.asarray(trayectoria, dtype=np.float32)
        if puntos.ndim != 2 or len(puntos) < 2:
            self.saveLogFile(
                f"[TRAY] Trayectoria RECHAZADA: hacen falta >=2 puntos (x, y) "
                f"(recibido con forma {puntos.shape})"
            )
            return

        # --- Paso 1: orientacion (cierre / rotacion) ---
        filtrados, cerrada = self._filtrar_trayectoria(puntos, varias_camaras)
        self.trayectoriaUsada = filtrados
        self._cerrada = cerrada

        # --- Paso 2: celdas ---
        self._construir_celdas(filtrados)

        # --- Zonas iniciales: una zona libre cubriendolo todo a v_min, con
        # cada celda gigante separada en su propia zona permanente ---
        self.zonas = []
        ini_libre = None
        
        """
        Ejemplo de como quedan las zonas cuando hay celdas gigantes

        es_gigante = [F, F, F, F, T, F, F, F]         
        self.zonas = [
            {"ini": 0, "fin": 3, "tipo": "libre",   "pwm": v_min, "vueltas": []},
            {"ini": 4, "fin": 4, "tipo": "gigante", "pwm": v_min, "vueltas": []},
            {"ini": 5, "fin": 7, "tipo": "libre",   "pwm": v_min, "vueltas": []},
        ] 
        """
 
        for c in range(self.n_celdas):

            if self._es_gigante[c]:

                if ini_libre is not None:
                    self.zonas.append(
                        {"ini": ini_libre, "fin": c - 1, "pwm": self.v_min,
                         "tipo": "libre", "vueltas": []}
                    )

                    ini_libre = None

                self.zonas.append(
                    {"ini": c, "fin": c, "pwm": self.v_min,
                     "tipo": "gigante", "vueltas": []}
                )

            elif ini_libre is None:
                ini_libre = c

        if ini_libre is not None:
            self.zonas.append(
                {"ini": ini_libre, "fin": self.n_celdas - 1, "pwm": self.v_min,
                 "tipo": "libre", "vueltas": []}
            )

        self.saveLogFile(
            f"[TRAY] Cadena construida: {self.n_celdas} celdas "
            f"({int(np.count_nonzero(self._es_gigante))} gigantes), "
            f"longitud total {self._long_total:.1f} px, cerrada={self._cerrada}"
        )

        self._log_perfil("inicial: una zona libre a v_min (+ zonas gigantes)")

        # Guardamos la plantilla de politica en su fichero
        self._guardar_plantilla_politica()

        if self.modo == "politica":
            self._cargar_politica()

    def _filtrar_trayectoria(self, puntos, varias_camaras):
        """
        Orienta la cadena y devuelve (puntos, cerrada)
        La lista que llega ya es una vuelta exacta, pero empieza en la meta, que
        puede caer en mitad del cacho de circuito que ve una camara. En ese caso
        se rota para empezar por el principio
        """

        filtrados = puntos

        # Resta el punto actual con el siguiente y devuelve la diferencia
        dif = np.diff(filtrados, axis=0)
        dist = np.hypot(dif[:, 0], dif[:, 1])

        # Indices donde hay un salto que sera una celda gigante
        saltos = [int(i) for i in np.nonzero(dist > self.umbral_celda_gigante)[0]]

        # Se guarda la trayectoria de puntos que llego desde el controlador
        self.saveLogFile(
            f"[TRAY] Trayectoria de calibración: {len(filtrados)} puntos, "
            f"separación entre puntos mín={float(dist.min()):.1f} "
            f"mediana={float(np.median(dist)):.1f} máx={float(dist.max()):.1f} px"
        )

        cierre = math.hypot(
            float(filtrados[-1][0] - filtrados[0][0]),
            float(filtrados[-1][1] - filtrados[0][1]),
        )

        cerrada = False

        if cierre <= self.umbral_cierre:
            if saltos and varias_camaras:
                # La camara ve un cacho de circuito con la meta dentro, se rota
                # [meta empieza, mitad A, resto del circuito, mitad B, meta termina]
                
                k = saltos[0]

                if len(saltos) > 1:
                    self.saveLogFile(
                        f"[TRAY] AVISO: cierre pequeño ({cierre:.1f} px) con "
                        f"{len(saltos)} saltos grandes; caso ambiguo, se rota "
                        f"en el PRIMER salto (los demás quedan como celdas "
                        f"gigantes)"
                    )

                filtrados = np.vstack([filtrados[k + 1:], filtrados[: k + 1]])

                self.saveLogFile(
                    f"[TRAY] Cierre: el final conecta con el inicio "
                    f"({cierre:.1f} px <= {self.umbral_cierre:.0f}) y hay un "
                    f"salto de {dist[k]:.1f} px en el punto {k}: la meta cae "
                    f"en mitad de la porción -> lista ROTADA para empezar "
                    f"tras el salto; cadena ABIERTA"
                )
            else:
                # Solo hay una camara y ve el circuito entero
                cerrada = True
                motivo = (
                    "sin saltos grandes"
                    if not saltos
                    else f"{len(saltos)} salto(s) tapado(s) que serán celdas "
                         f"gigantes (única cámara)"
                )
                self.saveLogFile(
                    f"[TRAY] Cierre: el final conecta con el inicio "
                    f"({cierre:.1f} px <= {self.umbral_cierre:.0f}), {motivo} "
                    f"-> cadena CERRADA (esta cámara ve el circuito completo)"
                )

        else:
            # Cadena abierta, solo vemos un cacho del circuito sin linea de meta
            self.saveLogFile(
                f"[TRAY] Cierre: el final NO conecta con el inicio "
                f"({cierre:.1f} px > {self.umbral_cierre:.0f}) -> cadena "
                f"ABIERTA (porción del circuito); saltos interiores "
                f"tapados: {len(saltos)}"
            )


        # Huecos que no llegan a celda gigante pero pueden venir de un fallo de
        # medida, porque donde deberia haber una celda no la hay
        # Solo se deja en el log, no se hace nada con ellos
        # Hay que repetir el calculo porque no se sabe de que rama del if venimos
        dif_final = np.diff(filtrados, axis=0)
        dist_final = np.hypot(dif_final[:, 0], dif_final[:, 1])
        huecos = [
            (int(i), float(dist_final[i]))
            for i in np.nonzero(dist_final > 3.0 * self.paso_celda)[0]
        ]

        if huecos:
            detalle = ", ".join(
                f"{L:.0f} px tras el punto {i}" for i, L in huecos
            )
            self.saveLogFile(
                f"[TRAY] AVISO: {len(huecos)} hueco(s) de más de "
                f"{3.0 * self.paso_celda:.0f} px en la calibración ({detalle}). "
                f"Si el coche derrapa siempre en el mismo sitio, repetir la "
                f"calibración"
            )

        return filtrados, cerrada

    def _construir_celdas(self, p):
        """
        Coloca una celda cada paso_celda px sobre la trayectoria, interpolando
        entre los puntos de calibracion. Los saltos mayores que
        umbral_celda_gigante no se interpolan, se vuelven una celda gigante
        """

        celdas_xy = []       # Posicion de cada celda normal
        celda_de_punto = []  # Indice de celda de cada posicion
        es_gigante = []      # True en las celdas gigantes
        long_celdas = []     # Px que cubre cada celda

        def emitir_normal(pt):
            es_gigante.append(False)
            long_celdas.append(self.paso_celda)
            celda_de_punto.append(len(es_gigante) - 1)
            celdas_xy.append((float(pt[0]), float(pt[1])))

        # La primera celda es el propio primer punto
        emitir_normal(p[0])
        # Px que faltan, dentro del segmento actual, hasta la proxima celda
        pendiente = self.paso_celda

        n_seg = len(p) if self._cerrada else len(p) - 1

        for i in range(n_seg):

            a = p[i]
            b = p[(i + 1) % len(p)]

            d = math.hypot(float(b[0] - a[0]), float(b[1] - a[1]))

            if d > self.umbral_celda_gigante:
                # Marcamos la celda c como gigante
                es_gigante.append(True)
                long_celdas.append(d)

                self.saveLogFile(
                    f"[TRAY] Celda GIGANTE {len(es_gigante) - 1}: salto de "
                    f"{d:.1f} px entre ({a[0]:.1f}, {a[1]:.1f}) y "
                    f"({b[0]:.1f}, {b[1]:.1f})"
                )

                # Guardamos el punto que esta al final de la celda gigante
                emitir_normal(b)
                pendiente = self.paso_celda

                continue

            if d == 0.0:
                continue

            # Donde cae la proxima celda, medido desde a
            recorrido = pendiente 

            # Se van poniendo celdas en el segmento ab
            while recorrido <= d: 
                t = recorrido / d
                emitir_normal(
                    (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
                )
                recorrido += self.paso_celda

            # Lo que sobro queda para el siguiente segmento, para que la
            # separacion siga siendo paso_celda
            pendiente = recorrido - d

        # En una cadena cerrada el muestreo puede dejar la ultima celda pegada
        # a la primera, y hay que eliminarla
        if self._cerrada and len(celdas_xy) > 1:
            d_ultima = math.hypot(
                celdas_xy[-1][0] - celdas_xy[0][0],
                celdas_xy[-1][1] - celdas_xy[0][1],
            )
            if d_ultima < self.paso_celda / 2.0 and not es_gigante[-1]:
                celdas_xy.pop()
                celda_de_punto.pop()
                es_gigante.pop()
                long_celdas.pop()

        self._puntos = np.asarray(celdas_xy, dtype=np.float32)
        self._celda_de_punto = np.asarray(celda_de_punto, dtype=np.int32)
        self.n_celdas = len(es_gigante)
        self._es_gigante = np.asarray(es_gigante, dtype=bool)
        self._long_celdas = np.asarray(long_celdas, dtype=np.float64)
        # Px acumulados al inicio de cada celda, para margen_restante
        self._long_acum = np.concatenate([[0.0], np.cumsum(self._long_celdas)[:-1]])
        self._long_total = float(np.sum(self._long_celdas))

        # Volcado celda a celda sobre el log
        self.saveLogFile("[TRAY] --- Cadena de celdas: celda: (x, y) | px acumulados ---")
        fila = 0

        for c in range(self.n_celdas):
            if self._es_gigante[c]:
                self.saveLogFile(
                    f"[TRAY] celda {c:3d}: GIGANTE ({self._long_celdas[c]:.1f} px) "
                    f"| acum={self._long_acum[c]:.1f}"
                )
            else:
                x, y = self._puntos[fila]
                self.saveLogFile(
                    f"[TRAY] celda {c:3d}: ({x:7.1f}, {y:7.1f}) "
                    f"| acum={self._long_acum[c]:.1f}"
                )
                fila += 1

    # ======================================================================
    # LOCALIZACION
    # ======================================================================
    def _dist_a_segmento(self, p, A, B):
        """
        Distancia del punto p al segmento AB
        """
        AB = B - A
        l2 = float(np.dot(AB, AB))
        if l2 == 0.0:
            return float(np.linalg.norm(p - A))
        t = max(0.0, min(1.0, float(np.dot(p - A, AB)) / l2))
        proyeccion = A + t * AB
        return float(np.linalg.norm(p - proyeccion))

    def localizar(self, punto):

        """
        Situa sobre la trayectoria base un punto, que puede ser la posicion de
        la pegatina delantera o la de la trasera

        Devuelve (celda, distancia perpendicular, fila), o None si aun no hay
        trayectoria

        Primero busca la celda mas cercana y luego proyecta sobre los dos
        segmentos que la unen con sus vecinas para sacar la distancia
        """

        if self._puntos is None or len(self._puntos) == 0:
            return None

        # Convertimos el punto a un array de numpy
        p = np.asarray(punto, dtype=np.float32)

        # Restamos el punto con todos los de la trayectoria base y nos quedamos
        # con las distancias al cuadrado
        d2 = np.sum((self._puntos - p) ** 2, axis=1)

        # argmin da la posicion del punto que queda a menor distancia
        fila = int(np.argmin(d2))
        
        # La fila va desplazada respecto a la cadena porque _puntos no guarda
        # las celdas gigantes
        idx = int(self._celda_de_punto[fila])

        mejor_dist = math.sqrt(float(d2[fila]))

        n_filas = len(self._puntos)

        # Indices de las filas vecinas, la anterior y la posterior
        f_prev = fila - 1
        f_next = fila + 1

        # Gestion de la envoltura en una cadena cerrada
        if self._cerrada:

            if fila == 0 and (int(self._celda_de_punto[-1]) + 1) % self.n_celdas == int(
                self._celda_de_punto[0]
            ):
                f_prev = n_filas - 1

            if fila == n_filas - 1 and (
                int(self._celda_de_punto[-1]) + 1
            ) % self.n_celdas == int(self._celda_de_punto[0]):

                f_next = 0

        # Hay que comprobarlo porque la cadena puede ser abierta o puede haber
        # una celda gigante al lado

        # Comprobamos si tiene vecino detras
        tengo_prev = (
            f_prev >= 0
            and self._celda_de_punto[f_prev] == (idx - 1) % self.n_celdas
        )

        # Comprobamos si tiene vecino delante
        tengo_next = (
            f_next < n_filas
            and self._celda_de_punto[f_next] == (idx + 1) % self.n_celdas
        )

        segmentos = []
        
        if tengo_prev:
            segmentos.append((f_prev, fila))

        if tengo_next:
            segmentos.append((fila, f_next))

        for fa, fb in segmentos:
            dist = self._dist_a_segmento(p, self._puntos[fa], self._puntos[fb])
            if dist < mejor_dist:
                mejor_dist = dist

        return idx, mejor_dist, fila

    # ======================================================================
    # LOGICA PRINCIPAL POR FOTOGRAMA
    # ======================================================================
    def actualizar_estado(self, p_front, p_back, frame_count, vuelta):
        """
        Entrada por frame. Recibe las dos pegatinas y devuelve el PWM que toca,
        o None si descarta el frame

        Situa la pegatina delantera para saber donde esta el coche, mira la
        trasera para ver si derrapa y devuelve el PWM de la zona en la que este
        """

        self._frame_valido = False

        loc_front = self.localizar(p_front)

        if loc_front is None:
            self.saveLogFile(
                f"[FRAME {frame_count} v={vuelta}] DESCARTADO: aún no hay "
                f"trayectoria cargada"
            )
            return None

        idx_f, dist_f, fila_f = loc_front

        if dist_f > self.max_dist_ruta:

            self._margen_restante = 0.0
             
            if self._estado_derrapando:
                # El coche se perdio con un derrape abierto y hay que cerrarlo
                self.saveLogFile(
                    f"[DERRAPE] Frame {frame_count} v={vuelta}: el coche salió "
                    f"de la trayectoria con un derrape abierto -> se CIERRA "
                    f"con el recorrido que llevaba"
                )
                self._cerrar_derrape(vuelta, frame_count)

            self.saveLogFile(
                f"[FRAME {frame_count} v={vuelta}] DESCARTADO por ruido: "
                f"front=({p_front[0]:.1f}, {p_front[1]:.1f}) está a {dist_f:.1f} px "
                f"de la ruta (> max_dist_ruta={self.max_dist_ruta:.0f}); el "
                f"controlador mantiene la velocidad anterior"
            )

            return None

        # Margen restante: px de trayectoria que le quedan por delante al coche
        if self._cerrada:
            self._margen_restante = float("inf")
        else:
            self._margen_restante = self._long_total - float(
                self._long_acum[idx_f]
            )

        # Deteccion del derrape con la pegatina trasera
        loc_back = self.localizar(p_back)

        if loc_back is not None:

            idx_b, dist_b, fila_b = loc_back
            self._dist_derrape = dist_b

            # Las dos celdas no pueden estar mas separadas que las dos pegatinas
            sep_celdas = float(
                np.linalg.norm(self._puntos[fila_b] - self._puntos[fila_f])
            )

            sep_pegatinas = float(
                np.linalg.norm(
                    np.asarray(p_back, dtype=np.float32)
                    - np.asarray(p_front, dtype=np.float32)
                )
            )

            coherente = sep_celdas <= sep_pegatinas + self.paso_celda

            if coherente:
                # Actualizamos el derrape porque la deteccion fue correcta
                self._actualizar_derrape(idx_b, dist_b, vuelta, frame_count)

            elif dist_b > self.umbral_derrape and not self._estado_derrapando:
                # La deteccion no cuadra y daria un derrape falso
                self.saveLogFile(
                    f"[DERRAPE] Frame {frame_count} v={vuelta}: dist={dist_b:.1f} "
                    f"> umbral pero IGNORADO por localización incoherente: la "
                    f"trasera cae en la celda {idx_b} y la delantera en la "
                    f"{idx_f}, que están a {sep_celdas:.1f} px, pero las "
                    f"pegatinas están a {sep_pegatinas:.1f} px"
                )
            texto_back = (
                f"back=({p_back[0]:.1f}, {p_back[1]:.1f})->c={idx_b} "
                f"d={dist_b:.1f} (umbral {self.umbral_derrape:.0f})"
            )

        else:
            texto_back = "back=NO localizado (máquina de derrape sin actualizar)"

        # Se comprueba si se llego al final de la trayectoria con un derrape abierto
        if (
            self._estado_derrapando
            and not self._cerrada
            and idx_f >= self.n_celdas - self.margen_extremo_celdas
        ):
            self.saveLogFile(
                f"[DERRAPE] Frame {frame_count} v={vuelta}: el coche llegó al "
                f"final de la cadena abierta (celda {idx_f} de {self.n_celdas}) "
                f"con un derrape abierto -> se CIERRA"
            )
            self._cerrar_derrape(vuelta, frame_count)

        self._frame_valido = True
        velocidad = self._velocidad_en(idx_f)

        zona = self._zona_de(idx_f)

        texto_zona = (
            f"zona=[{zona['ini']}-{zona['fin']}]({zona['tipo']})"
            if zona is not None
            else "zona=?"
        )

        texto_margen = "inf" if self._cerrada else f"{self._margen_restante:.1f}"

        self.saveLogFile(
            f"[FRAME {frame_count} v={vuelta}] "
            f"front=({p_front[0]:.1f}, {p_front[1]:.1f})->c={idx_f} "
            f"d={dist_f:.1f} | {texto_back} | "
            f"derrapando={self._estado_derrapando} | {texto_zona} "
            f"pwm={velocidad:.0f} | margen={texto_margen}"
        )

        return velocidad

    def notificar_perdida_vision(self, vuelta, frame_count):
        """
        El coche dejo de verse por esta camara. Si habia un derrape abierto se
        cierra
        """

        if self._estado_derrapando:
            self.saveLogFile(
                f"[DERRAPE] Frame {frame_count} v={vuelta}: el controlador "
                f"avisa de que el coche salió del campo de visión con un "
                f"derrape abierto -> se CIERRA"
            )

            self._cerrar_derrape(vuelta, frame_count)

    # ======================================================================
    # MAQUINA DE ESTADOS DEL DERRAPE
    # ======================================================================
    def _en_borde(self, idx):
        """True si idx cae a menos de margen_extremo_celdas de un extremo
        El extremo puede ser el borde de la trayectoria o una celda gigante
        """

        m = self.margen_extremo_celdas
        n = self.n_celdas

        if not self._cerrada and (idx < m or idx >= n - m):
            return True

        for k in range(-m, m + 1):
            c = idx + k

            if self._cerrada:
                c %= n

            elif c < 0 or c >= n:
                continue

            if self._es_gigante[c]:
                return True

        return False

    def _actualizar_derrape(self, idx, dist, vuelta, frame):
        """
        Con la distancia de la pegatina se comprueba si el coche esta derrapando
        Si lo esta se abre o se cierra el intervalo de derrape y se registra
        """

        if dist > self.umbral_derrape:
            # Se esta produciendo un derrape
            if not self._estado_derrapando:
                # Si no estabamos derrapando hay que marcar el inicio
                if self._en_borde(idx):
                    # Si estamos en el borde no se abre derrape
                    self.saveLogFile(
                        f"[DERRAPE] Frame {frame} v={vuelta}: dist={dist:.1f} > "
                        f"umbral pero IGNORADO por zona muerta: la celda {idx} "
                        f"está a menos de {self.margen_extremo_celdas} celdas de "
                        f"un extremo o de una celda gigante"
                    )
                    return

                # Abrimos el derrape
                self._estado_derrapando = True
                self._derrape_idx_ini = idx
                self._derrape_idx_fin = idx
                self.saveLogFile(
                    f"[DERRAPE] Frame {frame} v={vuelta}: ABIERTO en celda "
                    f"{idx} (dist={dist:.1f} > umbral {self.umbral_derrape:.0f})"
                )
            else:
                # Seguimos derrapando, se extiende el intervalo
                self._derrape_idx_fin = idx
        else:
            # No estamos derrapando, se comprueba si habia uno abierto para cerrarlo
            if self._estado_derrapando:
                self._cerrar_derrape(vuelta, frame)

    def _cerrar_derrape(self, vuelta, frame):
        """
        Cierra el derrape en curso y lo registra como zona. Antes ordena el
        intervalo [ini, fin] en el sentido de la marcha, porque en una cadena
        cerrada puede dar la vuelta por el principio
        """

        self._estado_derrapando = False
        ini = self._derrape_idx_ini
        fin = self._derrape_idx_fin

        if self._cerrada:
            adelante = (fin - ini) % self.n_celdas
            if adelante > self.n_celdas // 2:
                ini, fin = fin, ini
        else:
            ini, fin = min(ini, fin), max(ini, fin)

        n_celdas_derrape = (fin - ini) % self.n_celdas + 1 if self._cerrada else fin - ini + 1

        self.saveLogFile(
            f"[DERRAPE] Frame {frame} v={vuelta}: CERRADO, celdas "
            f"[{ini}, {fin}] ({n_celdas_derrape} celdas)"
        )

        self._registrar_zona(ini, fin, vuelta, frame)

    # ======================================================================
    # ZONAS
    # ======================================================================
    def _zona_de(self, idx):
        """Zona de la particion que contiene a idx, o None"""
        for z in self.zonas:
            if z["ini"] <= idx <= z["fin"]:
                return z
        return None

    # ----------------------------------------------------------------------
    # MODO POLITICA: perfil fijo cargado de un JSON
    # ----------------------------------------------------------------------
    def _guardar_plantilla_politica(self):
        """
        Deja en disco la particion actual como plantilla de politica, con la
        misma estructura que usa el algoritmo
        """

        datos = {
            "_ayuda": (
                "Plantilla de politica. Copiar a politica_<nodo>_<camara>.json "
                "(sin _PLANTILLA), editar los pwm de cada zona y arrancar el "
                "coche con cars.<coche>.modo: politica. Las zonas se aplican "
                "por INDICE DE CELDA, asi que valen para la cadena de "
                "n_celdas que se indica aqui."
            ),
            "camara": self.camara_id,
            "n_celdas": self.n_celdas,
            "v_min": self.v_min,
            "v_max": self.v_max,
            "zonas": self.zonas,
        }

        try:
            with open(self.fichero_plantilla, "w") as f:
                json.dump(datos, f, indent=2, ensure_ascii=False)

            self.saveLogFile(
                f"[INIT] Plantilla de politica guardada en "
                f"{self.fichero_plantilla} ({len(self.zonas)} zonas, "
                f"{self.n_celdas} celdas)"
            )

        except (IOError, OSError) as e:
            self.saveLogFile(
                f"[INIT] AVISO: no se pudo guardar la plantilla de politica en "
                f"{self.fichero_plantilla}: {e}"
            )

    def _cargar_politica(self):
        """
        Carga el perfil de politica del JSON, solo se usa en modo politica
        Se aplica por indice de celda, y como dos calibraciones nunca dan la
        misma trayectoria, se ajusta lo que se puede y se avisa
        Si algo falla se deja el perfil plano a v_min y el coche rueda despacio
        """
        try:
            with open(self.fichero_politica) as f:
                datos = json.load(f)

        except FileNotFoundError:
            self.saveLogFile(
                f"[INIT] AVISO: modo politica pero no existe "
                f"{self.fichero_politica}. Se sigue con el perfil plano a "
                f"v_min={self.v_min:.0f}. Copia el _PLANTILLA.json, editalo y "
                f"vuelve a arrancar"
            )
            return

        except (ValueError, IOError, OSError) as e:
            self.saveLogFile(
                f"[INIT] AVISO: no se pudo leer la politica de "
                f"{self.fichero_politica} ({e}). Se sigue con el perfil plano "
                f"a v_min={self.v_min:.0f}"
            )
            return

        n_esperado = datos.get("n_celdas")
        if n_esperado is not None and int(n_esperado) != self.n_celdas:
            self.saveLogFile(
                f"[INIT] AVISO: la politica se escribio para una cadena de "
                f"{int(n_esperado)} celdas y esta tiene {self.n_celdas}. La "
                f"calibracion no da la misma trayectoria dos veces: se aplica "
                f"por indice de celda y puede quedar desplazada. Revisa el "
                f"perfil resultante antes de dar valor a la prueba"
            )

        # Se recorta cada zona a la cadena de verdad y se descartan las que
        # queden fuera del todo
        zonas = []
        for z in datos.get("zonas", []):

            try:
                ini = max(0, int(z["ini"]))
                fin = min(self.n_celdas - 1, int(z["fin"]))
                pwm = float(z["pwm"])

            except (KeyError, TypeError, ValueError):
                self.saveLogFile(f"[INIT] AVISO: zona mal formada, se ignora: {z}")
                continue

            if ini > fin:
                self.saveLogFile(
                    f"[INIT] AVISO: zona [{z['ini']}-{z['fin']}] cae fuera de la "
                    f"cadena (0-{self.n_celdas - 1}), se ignora"
                )
                continue

            zonas.append({
                "ini": ini, "fin": fin,
                "pwm": max(self.v_min, min(self.v_max, pwm)),
                "tipo": "gigante" if self._es_gigante[ini] and ini == fin
                        else z.get("tipo", "libre"),
                "vueltas": [],
            })

        zonas.sort(key=lambda z: z["ini"])

        if not zonas:
            self.saveLogFile(
                f"[INIT] AVISO: la politica de {self.fichero_politica} no tiene "
                f"ninguna zona utilizable. Se sigue con el perfil plano a "
                f"v_min={self.v_min:.0f}"
            )
            return

        completas = []
        siguiente = 0
        for z in zonas:
            if z["ini"] > siguiente:

                self.saveLogFile(
                    f"[INIT] AVISO: las celdas {siguiente}-{z['ini'] - 1} no las "
                    f"cubre ninguna zona de la politica: se rellenan a "
                    f"v_min={self.v_min:.0f}"
                )

                completas.append({
                    "ini": siguiente, "fin": z["ini"] - 1, "pwm": self.v_min,
                    "tipo": "libre", "vueltas": [],
                })

            elif z["ini"] < siguiente:

                if z["fin"] < siguiente:

                    self.saveLogFile(
                        f"[INIT] AVISO: la zona [{z['ini']}-{z['fin']}] queda "
                        f"tapada por la anterior, se ignora"
                    )

                    continue

                self.saveLogFile(
                    f"[INIT] AVISO: la zona [{z['ini']}-{z['fin']}] solapa con la "
                    f"anterior, se recorta a [{siguiente}-{z['fin']}]"
                )

                z["ini"] = siguiente

            completas.append(z)
            siguiente = z["fin"] + 1

        if siguiente <= self.n_celdas - 1:
            self.saveLogFile(
                f"[INIT] AVISO: las celdas {siguiente}-{self.n_celdas - 1} no las "
                f"cubre ninguna zona de la politica: se rellenan a "
                f"v_min={self.v_min:.0f}"
            )

            completas.append({
                "ini": siguiente, "fin": self.n_celdas - 1, "pwm": self.v_min,
                "tipo": "libre", "vueltas": [],
            })

        self.zonas = completas
        self.saveLogFile(
            f"[INIT] Politica cargada de {self.fichero_politica}: "
            f"{len(self.zonas)} zonas sobre {self.n_celdas} celdas. El perfil "
            f"queda FIJO: los derrapes se registran pero no lo mueven"
        )

        self._log_perfil("politica cargada, perfil fijo")

    def _intervalo_a_celdas(self, ini, fin):
        """Lista de celdas del intervalo [ini, fin] en orden de recorrido
        En cadena cerrada el intervalo puede envolver el origen (ini > fin)"""
        if fin >= ini:
            return list(range(ini, fin + 1))

        # Envuelve el origen, solo posible en cadena cerrada
        return list(range(ini, self.n_celdas)) + list(range(0, fin + 1))

    def _extender_atras(self, idx_desde, px):
        """
        Aplica la reduccion hacia atras cuando se crea una zona o se unen dos
        Empieza en idx_desde - 1 y gasta px de retroceso por cada celda que pasa

        Devuelve (celdas normales, celdas gigantes, px sobrantes)
        """

        normales = []
        gigantes = []
        restante = float(px)

        j = idx_desde - 1
        pasos = 0

        while restante > 0.0 and pasos < self.n_celdas:

            if j < 0:
                if not self._cerrada:
                    # Se salio por el inicio, lo que queda se derrama
                    return normales, gigantes, restante

                j = self.n_celdas - 1

            if self._es_gigante[j]:
                gigantes.append(j)

            else:
                normales.append(j)

            restante -= float(self._long_celdas[j])
            j -= 1
            pasos += 1

        return normales, gigantes, 0.0

    def _castigar_gigante(self, c, vuelta):
        """Baja el PWM de una celda gigante alcanzada por un retroceso"""

        zona = self._zona_de(c)

        if zona is None or zona["tipo"] != "gigante":
            return

        antes = zona["pwm"]
        zona["pwm"] = max(self.v_min, zona["pwm"] - self.reduccion_derrape)
        zona["vueltas"].append(vuelta)

        self.saveLogFile(
            f"[ZONA] Celda gigante {c} castigada por retroceso: "
            f"pwm {antes:.0f}->{zona['pwm']:.0f} (queda protegida)"
        )

    def _carvear_particion(self, a, b, nueva):
        """
        Particiona las zonas metiendo la nueva y recortando las que ya estaban
        La nueva zona ocupa el intervalo [a, b]
        """

        resultado = []
        for z in self.zonas:

            if z["fin"] < a or z["ini"] > b:
                resultado.append(z)
                continue

            if z["ini"] < a:
                resultado.append(
                    {"ini": z["ini"], "fin": a - 1, "pwm": z["pwm"],
                     "tipo": z["tipo"], "vueltas": list(z["vueltas"])}
                )

            if z["fin"] > b:
                resultado.append(
                    {"ini": b + 1, "fin": z["fin"], "pwm": z["pwm"],
                     "tipo": z["tipo"], "vueltas": list(z["vueltas"])}
                )

        resultado.append(nueva)
        self.zonas = sorted(resultado, key=lambda z: z["ini"])

    def _carvear_derrape(self, a, b, vuelta):
        """
        Convierte el intervalo de celdas [a, b] en una zona de derrape y se
        traga las zonas de derrape que ya hubiera pegadas o solapadas. El PWM
        nuevo es el minimo de lo que cubria menos reduccion_derrape
        """

        a2, b2 = a, b
        absorbidas = []
        cambio = True

        while cambio:
            cambio = False

            for z in self.zonas:
                if z["tipo"] != "derrape" or z in absorbidas:
                    continue

                if (
                    z["ini"] > b2 + self.margen_fusion_celdas
                    or z["fin"] < a2 - self.margen_fusion_celdas
                ):
                    continue

                if z["fin"] < a2:
                    hueco = range(z["fin"] + 1, a2)
                else:
                    hueco = range(b2 + 1, z["ini"])

                if any(self._es_gigante[c] for c in hueco):
                    continue

                absorbidas.append(z)
                a2 = min(a2, z["ini"])
                b2 = max(b2, z["fin"])
                cambio = True

        # El PWM nuevo es el minimo de las zonas que cubren [a2, b2] menos la reduccion
        pwms = [
            z["pwm"] for z in self.zonas
            if not (z["fin"] < a2 or z["ini"] > b2)
        ]

        nuevo_pwm = max(self.v_min, min(pwms) - self.reduccion_derrape)

        vueltas_hist = []
        for z in absorbidas:
            vueltas_hist.extend(z["vueltas"])

        vueltas_hist.append(vuelta)

        if absorbidas:
            self.saveLogFile(
                f"[ZONA] Fusión: el castigo [{a}, {b}] absorbe "
                f"{len(absorbidas)} zona(s) de derrape ("
                + ", ".join(f"[{z['ini']}-{z['fin']}]" for z in absorbidas)
                + f") -> zona [{a2}, {b2}] pwm={nuevo_pwm:.0f}"
            )
        else:
            self.saveLogFile(
                f"[ZONA] Nueva zona de derrape [{a2}, {b2}] pwm={nuevo_pwm:.0f}"
            )

        for z in absorbidas:
            self.zonas.remove(z)

        nueva = {
            "ini": a2, "fin": b2, "pwm": nuevo_pwm,
            "tipo": "derrape", "vueltas": vueltas_hist,
        }

        self._carvear_particion(a2, b2, nueva)

    def _registrar_zona(self, ini, fin, vuelta, frame):
        """
        Cuando se cierra un derrape crea la zona y aplica el castigo
        """
        self.derrapes_contador += 1

        # En modo politica el derrape se cuenta y queda en el log, pero no se aplica
        if self.modo == "politica":
            return

        celdas_int = self._intervalo_a_celdas(ini, fin)
        normales_int = [c for c in celdas_int if not self._es_gigante[c]]
        gigantes_int = [c for c in celdas_int if self._es_gigante[c]]

        # Se comprueba si hay alguna zona de derrape vecina
        vecinas = set()

        for c in normales_int:

            for k in range(-self.margen_fusion_celdas, self.margen_fusion_celdas + 1):
                cc = c + k

                if self._cerrada:
                    cc %= self.n_celdas
                elif cc < 0 or cc >= self.n_celdas:
                    continue
                vecinas.add(cc)

        tocadas = [
            z for z in self.zonas
            if z["tipo"] == "derrape"
            and any(z["ini"] <= c <= z["fin"] for c in vecinas)
        ]

        if tocadas:
            # Hay zonas de derrape solapadas
            retro = self.retroceso_fusion
            arranque = min([ini] + [z["ini"] for z in tocadas])
            motivo = (
                f"fusión con {len(tocadas)} zona(s) existente(s): retroceso "
                f"corto de {retro:.0f} px desde la celda {arranque}"
            )

        else:
            # Nueva zona de derrape
            retro = self.retroceso_creacion
            arranque = ini
            motivo = (
                f"zona nueva: retroceso de {retro:.0f} px desde la celda "
                f"{arranque}"
            )

        self.saveLogFile(
            f"[ZONA] Frame {frame} vuelta {vuelta}: derrape en [{ini}, {fin}] "
            f"-> {motivo} (derrapes_contador={self.derrapes_contador})"
        )

        normales_atras, gigantes_atras, sobrante = self._extender_atras(
            arranque, retro
        )

        # Se obtienen las celdas que hay que castigar
        objetivo = set(normales_int) | set(normales_atras)
        for z in tocadas:
            objetivo.update(range(z["ini"], z["fin"] + 1))
        objetivo = sorted(objetivo)
        for g in sorted(set(gigantes_int) | set(gigantes_atras)):
            self._castigar_gigante(g, vuelta)

        tramo_ini = None
        anterior = None

        # Se construyen las zonas nuevas
        for c in objetivo + [None]:
            if tramo_ini is None:
                tramo_ini = c
            elif c is None or c != anterior + 1:
                self._carvear_derrape(tramo_ini, anterior, vuelta)
                tramo_ini = c
            anterior = c

        if sobrante > 0.0:
            self._reduccion_pendiente += sobrante
            self.saveLogFile(
                f"[ZONA] Derrame: el retroceso se salió por el inicio de la "
                f"cadena con {sobrante:.1f} px por reducir -> "
                f"reduccion_pendiente={self._reduccion_pendiente:.1f} px "
                f"(el controlador debe avisar a la cámara precedente)"
            )

        self._log_zonas(f"estado tras el derrape de la vuelta {vuelta}")
        self._log_perfil("tras castigo por derrape")

    def aplicar_reduccion_externa(self, px, vuelta):
        """
        Aplica la reduccion sobrante que no cupo en la camara siguiente
        Las celdas que hay que castigar estan al final de nuestra trayectoria
        """

        if self._puntos is None or self.n_celdas == 0:
            self.saveLogFile(
                "[ZONA] aplicar_reduccion_externa ignorada: sin trayectoria"
            )
            return

        if self._cerrada:
            self.saveLogFile(
                "[ZONA] aplicar_reduccion_externa ignorada: la cadena es "
                "cerrada (esta cámara ve el circuito completo, no tiene "
                "'final' donde aplicar la reducción)"
            )
            return

        self.saveLogFile(
            f"[ZONA] Reducción externa: la cámara siguiente pide reducir "
            f"{px:.1f} px al final de nuestra trayectoria (vuelta {vuelta})"
        )

        normales, gigantes, sobrante = self._extender_atras(self.n_celdas, px)

        for g in sorted(set(gigantes)):
            self._castigar_gigante(g, vuelta)
        objetivo = sorted(set(normales))

        tramo_ini = None
        anterior = None

        for c in objetivo + [None]:
            if tramo_ini is None:
                tramo_ini = c
            elif c is None or c != anterior + 1:
                self._carvear_derrape(tramo_ini, anterior, vuelta)
                tramo_ini = c
            anterior = c

        if sobrante > 0.0:
            self._reduccion_pendiente += sobrante
            self.saveLogFile(
                f"[ZONA] Derrame en cascada: tampoco caben {sobrante:.1f} px "
                f"aquí -> reduccion_pendiente={self._reduccion_pendiente:.1f}"
            )
        self._log_zonas("estado tras la reducción externa")
        self._log_perfil("tras la reducción externa")

    # ======================================================================
    # PERFIL (lectura y subida por vuelta)
    # ======================================================================
    def _velocidad_en(self, idx):
        """
        El PWM de la celda idx es el PWM de su zona
        """

        zona = self._zona_de(idx)
        if zona is None:
            return self.v_min

        return float(min(zona["pwm"], self.v_max))

    def registrar_vuelta(self, vuelta, vuelta_limpia):
        """
        Aviso de vuelta completada: cada zona sube incremento_vuelta salvo que
        este protegida
        """

        hubo_derrape_local = self.derrapes_contador != self._derrapes_vuelta_anterior

        self.saveLogFile(
            f"[VUELTA {vuelta}] limpia_global={vuelta_limpia} | derrapes de esta "
            f"cámara: {self._derrapes_vuelta_anterior}->{self.derrapes_contador} "
            f"(hubo_derrape_local={hubo_derrape_local})"
        )

        self._derrapes_vuelta_anterior = self.derrapes_contador

        # En modo politica no hay nada que tocar
        if self.modo == "politica":
            self.saveLogFile(
                f"[VUELTA {vuelta}] El perfil NO sube: modo politica, el perfil "
                f"es fijo y se cargó del JSON de la política"
            )
            return

        subidas = 0
        protegidas = 0

        for z in self.zonas:

            ultima = z["vueltas"][-1] if z["vueltas"] else None

            if ultima is not None and vuelta - ultima <= self.vueltas_proteccion:

                protegidas += 1
                self.saveLogFile(
                    f"[VUELTA {vuelta}] zona [{z['ini']}-{z['fin']}]({z['tipo']}) "
                    f"PROTEGIDA: último derrape en vuelta {ultima}, le quedan "
                    f"{self.vueltas_proteccion - (vuelta - ultima)} vuelta(s) "
                    f"de protección"
                )
                continue

            z["pwm"] = min(self.v_max, z["pwm"] + self.incremento_vuelta)
            subidas += 1

        if subidas == 0:
            self.saveLogFile(
                f"[VUELTA {vuelta}] El perfil NO sube: todas las zonas "
                f"({protegidas}) están protegidas"
            )
            return

        self.saveLogFile(
            f"[VUELTA {vuelta}] suben {subidas} de {len(self.zonas)} zonas "
            f"+{self.incremento_vuelta:.0f} (tope v_max={self.v_max:.0f}); "
            f"{protegidas} protegidas"
        )
        self._log_perfil(f"tras la vuelta {vuelta}")

    # ======================================================================
    # VOLCADOS DE ESTADO (solo escriben en el log, no modifican nada)
    # ======================================================================
    def _log_perfil(self, motivo):
        """
        Vuelca al log la particion completa de zonas con su PWM. Se llama tras
        cada modificacion
        """
        self.saveLogFile(f"[PERFIL] ({motivo})")
        for z in self.zonas:
            n_z = z["fin"] - z["ini"] + 1
            self.saveLogFile(
                f"[PERFIL] zona [{z['ini']:3d}-{z['fin']:3d}] {z['tipo']:8s} "
                f"pwm={z['pwm']:.0f} ({n_z} celdas)"
            )
        compacto = ",".join(
            "[" + ",".join(f"{z['pwm']:.0f}" for _ in range(z["fin"] - z["ini"] + 1)) + "]"
            for z in self.zonas
        )
        self.saveLogFile(f"[PERFIL] perfil=[{compacto}]")

    def _log_zonas(self, motivo):
        """Vuelca todas las zonas con su historial de vueltas, para ver como
        evoluciona a lo largo de la carrera"""
        self.saveLogFile(f"[ZONA] ({motivo})")
        for z in self.zonas:
            self.saveLogFile(
                f"[ZONA] [{z['ini']:3d}-{z['fin']:3d}] {z['tipo']:8s} "
                f"pwm={z['pwm']:.0f} vueltas={z['vueltas']}"
            )

    # ======================================================================
    # PERSISTENCIA
    # ======================================================================
    def saveLogFile(self, data):
        try:
            with open(self.log_file, "a") as log_file:
                marca = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                log_file.write(f"{marca} {data}\n")
        except IOError as e:
            print(f"Error escribiendo log: {e}")
