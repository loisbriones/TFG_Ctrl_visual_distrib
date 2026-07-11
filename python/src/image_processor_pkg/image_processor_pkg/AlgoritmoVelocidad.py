import math
import csv
import os
import numpy as np


class EstrategiaPerfil:
    """
    Algoritmo de velocidad por perfil de PWM continuo sobre la trayectoria.

    Combina dos ideas de los TFG anteriores, adaptadas a la arquitectura
    distribuida (una instancia por cámara, con cobertura parcial del circuito
    y mensajes que pueden perderse o llegar desordenados):

      - Detección de derrape del TFG de Mario López Cea (sección 5.4.2): la
        pegatina trasera se separa perpendicularmente de la trayectoria más de
        `umbral_derrape` px.
      - Perfil de velocidad por secciones del TFG de Adrián Rego, con dos
        correcciones importantes:
          * El perfil se indexa por longitud de arco sobre la trayectoria
            (celdas de `paso_perfil` px). El original lo indexaba por número
            de fotograma dentro de la sección, con lo que dependía de los FPS
            de la cámara y de la propia velocidad del coche.
          * El derrape se detecta con la distancia perpendicular fina, no con
            la distancia a puntos sueltos.

    La trayectoria de cada cámara puede ser una parte abierta del circuito (o
    varios tramos separados, si la cámara ve dos zonas de la pista). La
    posición del coche y las zonas de derrape se expresan como distancia
    recorrida sobre la trayectoria (longitud de arco), de forma que la
    decisión de velocidad es una función pura de la posición actual y no
    depende de recibir todos los fotogramas seguidos y en orden.

    Funcionamiento del perfil:
      - Todas las celdas empiezan en v_min (el coche arranca lento).
      - Vuelta limpia (sin derrapes en ninguna cámara): las celdas suben
        +incremento_vuelta, salvo las de zonas con derrapes recientes.
      - Derrape: bajan -reduccion_derrape las celdas desde `ventana_reduccion`
        px antes del inicio del derrape hasta su fin, que es el tramo donde se
        gestó la pérdida de adherencia.
      - Las zonas castigadas vuelven a subir tras `vueltas_proteccion` vueltas
        sin derrapar en ellas, de forma que el perfil oscila justo por debajo
        del límite de adherencia de cada punto del circuito.

    El controlador debe llamar a `registrar_vuelta(vuelta, vuelta_limpia)` en
    todas las instancias cada vez que el coche completa una vuelta; sin ese
    aviso el perfil nunca sube.
    """

    def __init__(self, v_max, v_min, node_name="node", camara_id="cam"):
        self.v_max = float(v_max)
        self.v_min = float(v_min)

        # Umbral de distancia perpendicular (px) para considerar derrape
        self.umbral_derrape = 8.0
        # Distancia de anticipación inicial de cada zona (px sobre la trayectoria)
        self.anticipacion_inicial = 280.0
        # Incremento de la anticipación cuando se repite un derrape en una zona
        self.incremento_anticipacion = 20.0
        # Margen para fusionar derrapes cercanos en una única zona (px de arco)
        self.margen_fusion = 20.0
        # Ruido: si la etiqueta frontal está más lejos que esto de la ruta se ignora el frame
        self.max_dist_ruta = 80.0
        # Zona muerta en los extremos de un tramo abierto: evita registrar falsos
        # derrapes cuando el coche entra o sale del campo de visión de la cámara
        self.margen_extremo = 30.0

        # --- Parámetros del perfil ---
        # Tamaño de celda del perfil (px de trayectoria)
        self.paso_perfil = 30.0
        # Subida por vuelta limpia y bajada por derrape (unidades de PWM)
        self.incremento_vuelta = 1.0
        self.reduccion_derrape = 2.0
        # Cuánto antes del inicio del derrape se reduce el perfil (px)
        self.ventana_reduccion = 300.0
        # Vueltas sin derrapar en una zona antes de dejarla subir de nuevo
        self.vueltas_proteccion = 2

        # --- Trayectoria ---
        self.trayectoriaUsada = None  # np.array (N, 2)
        self._tramo_de_nodo = None  # np.array (N,) id del tramo de cada nodo
        self._s_nodos = None  # np.array (N,) longitud de arco dentro del tramo
        self._long_tramos = {}  # id de tramo -> longitud total
        self._cerrada = False  # True si la trayectoria es el circuito completo

        # --- Perfil de PWM: {tramo: np.ndarray con el PWM de cada celda} ---
        self.perfil = {}
        self._derrapes_vuelta_anterior = 0

        # --- Zonas de derrape: {tramo: [{s_ini, s_fin, anticipacion, vueltas}]} ---
        self.zonas = {}
        # Nº total de eventos de derrape detectados (lo usa el controlador para
        # saber si en la última vuelta hubo derrapes)
        self.derrapes_contador = 0

        # --- Estado del derrape en curso ---
        self._estado_derrapando = False
        self._derrape_tramo = None
        self._derrape_s_ini = 0.0
        self._derrape_s_fin = 0.0
        self._dist_derrape = 0.0

        # Distancia de trayectoria que le queda al coche por delante dentro del
        # campo de visión de esta cámara (por si se arbitra entre cámaras)
        self._margen_restante = 0.0
        # True si en el último fotograma el coche se localizó sobre la trayectoria
        self._frame_valido = False

        directorio_logs = "/ros2_ws/src/image_processor_pkg/logs_carrera"
        os.makedirs(directorio_logs, exist_ok=True)

        self.log_file = f"{directorio_logs}/derrapesLog_{node_name}_{camara_id}.txt"
        self.csv_file = f"{directorio_logs}/datosDerrapes_{node_name}_{camara_id}.csv"

        try:
            with open(self.log_file, "w") as f:
                f.write("=== LOG INICIADO ===\n")
        except IOError as e:
            print(f"Error inicializando log: {e}")

    @property
    def dist_derrape(self):
        return self._dist_derrape

    @property
    def estado_derrapando(self):
        return self._estado_derrapando

    @property
    def margen_restante(self):
        return self._margen_restante

    @property
    def frame_valido(self):
        return self._frame_valido

    def set_velocidades(self, v_max, v_min):
        self.v_max = float(v_max)
        self.v_min = float(v_min)

    # --- TRAYECTORIA ---
    def setTrayectoria(self, trayectoria):
        if self.trayectoriaUsada is not None:
            return

        puntos = np.asarray(trayectoria, dtype=np.float32)
        if puntos.ndim != 2 or len(puntos) < 2:
            return
        self.trayectoriaUsada = puntos

        dif = np.diff(puntos, axis=0)
        dist = np.hypot(dif[:, 0], dif[:, 1])

        # Un salto mucho mayor que la separación típica entre nodos indica que
        # el coche salió del campo de visión: ahí termina un tramo y empieza otro
        umbral_salto = max(4.0 * float(np.median(dist)), 60.0)

        tramos = np.zeros(len(puntos), dtype=np.int32)
        s = np.zeros(len(puntos), dtype=np.float64)
        for i in range(1, len(puntos)):
            if dist[i - 1] > umbral_salto:
                tramos[i] = tramos[i - 1] + 1
                s[i] = 0.0
            else:
                tramos[i] = tramos[i - 1]
                s[i] = s[i - 1] + dist[i - 1]

        self._tramo_de_nodo = tramos
        self._s_nodos = s
        for t in np.unique(tramos):
            seleccion = tramos == t
            self._long_tramos[int(t)] = float(s[seleccion].max())
            self.zonas[int(t)] = []

        # Si es un único tramo cuyo final conecta con el inicio, la cámara ve el
        # circuito completo y la trayectoria se trata como cerrada
        if tramos[-1] == 0:
            cierre = math.hypot(
                float(puntos[-1][0] - puntos[0][0]),
                float(puntos[-1][1] - puntos[0][1]),
            )
            if cierre <= umbral_salto:
                self._cerrada = True
                self._long_tramos[0] += cierre

        # El perfil arranca con todas las celdas a v_min
        for tramo, longitud in self._long_tramos.items():
            n_celdas = max(1, int(math.ceil(longitud / self.paso_perfil)))
            self.perfil[tramo] = np.full(n_celdas, self.v_min, dtype=float)

        self.saveLogFile(
            f"Trayectoria cargada: {len(puntos)} nodos, "
            f"{len(self._long_tramos)} tramo(s), cerrada={self._cerrada}"
        )
        self.saveLogFile(
            "Perfil inicializado: "
            + ", ".join(f"tramo {t}: {len(c)} celdas" for t, c in self.perfil.items())
        )

    def _proyectar_segmento(self, p, a_idx, b_idx, s_base):
        A = self.trayectoriaUsada[a_idx]
        B = self.trayectoriaUsada[b_idx]
        AB = B - A
        l2 = float(np.dot(AB, AB))
        if l2 == 0.0:
            return float(np.linalg.norm(p - A)), s_base
        t = max(0.0, min(1.0, float(np.dot(p - A, AB)) / l2))
        proyeccion = A + t * AB
        return float(np.linalg.norm(p - proyeccion)), s_base + t * math.sqrt(l2)

    def localizar(self, punto):
        """Devuelve (tramo, s, distancia perpendicular) del punto sobre la
        trayectoria, o None si aún no hay trayectoria."""
        if self.trayectoriaUsada is None:
            return None

        p = np.asarray(punto, dtype=np.float32)
        d2 = np.sum((self.trayectoriaUsada - p) ** 2, axis=1)
        idx = int(np.argmin(d2))
        tramo = int(self._tramo_de_nodo[idx])
        n = len(self.trayectoriaUsada)

        mejor_dist = math.sqrt(float(d2[idx]))
        mejor_s = float(self._s_nodos[idx])

        # Refinamos proyectando sobre los segmentos adyacentes del mismo tramo
        segmentos = []
        if idx > 0 and self._tramo_de_nodo[idx - 1] == tramo:
            segmentos.append((idx - 1, idx, float(self._s_nodos[idx - 1])))
        if idx < n - 1 and self._tramo_de_nodo[idx + 1] == tramo:
            segmentos.append((idx, idx + 1, float(self._s_nodos[idx])))
        if self._cerrada and (idx == 0 or idx == n - 1):
            segmentos.append((n - 1, 0, float(self._s_nodos[n - 1])))

        for a, b, s_base in segmentos:
            distancia, s = self._proyectar_segmento(p, a, b, s_base)
            if distancia < mejor_dist:
                mejor_dist = distancia
                mejor_s = s

        long_tramo = self._long_tramos[tramo]
        if self._cerrada and long_tramo > 0:
            mejor_s = mejor_s % long_tramo

        return tramo, mejor_s, mejor_dist

    # --- LÓGICA PRINCIPAL ---
    def actualizar_estado(self, p_front, p_back, frame_count, vuelta):
        """Procesa una detección y devuelve la velocidad a aplicar, o None si
        el fotograma se descarta por ruido."""
        self._frame_valido = False

        loc_front = self.localizar(p_front)
        if loc_front is None:
            return None

        tramo_f, s_f, dist_f = loc_front

        # La etiqueta frontal marca la posición del coche y siempre va sobre el
        # carril: si está lejos de la ruta es una detección falsa
        if dist_f > self.max_dist_ruta:
            self._margen_restante = 0.0
            return None

        long_tramo = self._long_tramos[tramo_f]
        if self._cerrada:
            self._margen_restante = float("inf")
        else:
            self._margen_restante = long_tramo - s_f

        # Detección del derrape con la etiqueta trasera
        loc_back = self.localizar(p_back)
        if loc_back is not None:
            tramo_b, s_b, dist_b = loc_back
            self._dist_derrape = dist_b
            self._actualizar_derrape(tramo_b, s_b, dist_b, vuelta, frame_count)

        self._frame_valido = True
        return self._velocidad_en(tramo_f, s_f)

    def _actualizar_derrape(self, tramo, s, dist, vuelta, frame):
        if dist > self.umbral_derrape:
            if not self._estado_derrapando:
                # No abrimos derrapes pegados al borde del campo de visión: al
                # entrar/salir de la imagen la detección no es fiable
                en_borde = (not self._cerrada) and (
                    s < self.margen_extremo
                    or s > self._long_tramos[tramo] - self.margen_extremo
                )
                if en_borde:
                    return
                self._estado_derrapando = True
                self._derrape_tramo = tramo
                self._derrape_s_ini = s
                self._derrape_s_fin = s
            elif tramo == self._derrape_tramo:
                long_tramo = self._long_tramos[tramo]
                if self._cerrada and abs(s - self._derrape_s_fin) > long_tramo / 2.0:
                    # El derrape cruzó el origen de la trayectoria cerrada: se
                    # cierra la zona actual y se abre otra al otro lado
                    self._cerrar_derrape(vuelta, frame)
                    self._estado_derrapando = True
                    self._derrape_s_ini = s
                self._derrape_s_fin = s
        else:
            if self._estado_derrapando:
                self._cerrar_derrape(vuelta, frame)

    def _cerrar_derrape(self, vuelta, frame):
        self._estado_derrapando = False
        self._registrar_zona(
            self._derrape_tramo,
            min(self._derrape_s_ini, self._derrape_s_fin),
            max(self._derrape_s_ini, self._derrape_s_fin),
            vuelta,
            frame,
        )

    def _registrar_zona(self, tramo, s_ini, s_fin, vuelta, frame):
        self.derrapes_contador += 1
        zonas = self.zonas.setdefault(tramo, [])

        solapadas = [
            z
            for z in zonas
            if s_ini - self.margen_fusion <= z["s_fin"]
            and s_fin + self.margen_fusion >= z["s_ini"]
        ]

        if solapadas:
            zona = solapadas[0]
            # Si el derrape toca varias zonas, se unen en una sola
            for extra in solapadas[1:]:
                zonas.remove(extra)
                zona["s_ini"] = min(zona["s_ini"], extra["s_ini"])
                zona["s_fin"] = max(zona["s_fin"], extra["s_fin"])
                zona["anticipacion"] = max(zona["anticipacion"], extra["anticipacion"])
                zona["vueltas"].extend(extra["vueltas"])

            zona["s_ini"] = min(zona["s_ini"], s_ini)
            zona["s_fin"] = max(zona["s_fin"], s_fin)
            # Derrape repetido en la zona: hay que empezar a frenar antes
            zona["anticipacion"] += self.incremento_anticipacion
            zona["vueltas"].append(vuelta)
            self.saveLogFile(
                f"Frame {frame} vuelta {vuelta}: derrape repetido en zona "
                f"[{zona['s_ini']:.0f}, {zona['s_fin']:.0f}] del tramo {tramo}. "
                f"Anticipación aumentada a {zona['anticipacion']:.0f}"
            )
        else:
            zonas.append(
                {
                    "s_ini": s_ini,
                    "s_fin": s_fin,
                    "anticipacion": self.anticipacion_inicial,
                    "vueltas": [vuelta],
                }
            )
            zonas.sort(key=lambda z: z["s_ini"])
            self.saveLogFile(
                f"Frame {frame} vuelta {vuelta}: nueva zona de derrape "
                f"[{s_ini:.0f}, {s_fin:.0f}] en tramo {tramo}"
            )

        # Además castigamos el perfil en la zona previa al derrape, que es
        # donde se gestó la pérdida de adherencia
        indices = self._celdas_en(tramo, s_ini - self.ventana_reduccion, s_fin)
        if not indices:
            return
        celdas = self.perfil[tramo]
        for i in indices:
            celdas[i] = max(self.v_min, celdas[i] - self.reduccion_derrape)
        self.saveLogFile(
            f"Perfil reducido -{self.reduccion_derrape} en tramo {tramo}, "
            f"{len(indices)} celdas antes del derrape [{s_ini:.0f}, {s_fin:.0f}]"
        )

    # --- PERFIL DE PWM ---
    def _celdas_en(self, tramo, s_desde, s_hasta):
        """Índices de las celdas del tramo que cubren [s_desde, s_hasta], con
        vuelta al origen si la trayectoria es cerrada."""
        celdas = self.perfil.get(tramo)
        if celdas is None or len(celdas) == 0:
            return []
        n = len(celdas)
        i0 = int(math.floor(s_desde / self.paso_perfil))
        i1 = int(math.floor(s_hasta / self.paso_perfil))
        if self._cerrada:
            return [i % n for i in range(i0, i1 + 1)]
        return list(range(max(0, i0), min(n - 1, i1) + 1))

    def _velocidad_en(self, tramo, s):
        """Velocidad en la posición s del tramo: el PWM aprendido de su celda."""
        celdas = self.perfil.get(tramo)
        if celdas is None or len(celdas) == 0:
            return self.v_min
        idx = min(int(s / self.paso_perfil), len(celdas) - 1)
        return float(min(celdas[idx], self.v_max))

    def registrar_vuelta(self, vuelta, vuelta_limpia):
        """Aviso del controlador al completarse una vuelta. `vuelta_limpia` es
        True si ninguna cámara registró derrapes en ella."""
        hubo_derrape_local = self.derrapes_contador != self._derrapes_vuelta_anterior
        self._derrapes_vuelta_anterior = self.derrapes_contador

        if not vuelta_limpia or hubo_derrape_local:
            return

        # Vuelta limpia: sube todo el perfil salvo las celdas de zonas con
        # derrapes recientes (siguen protegidas unas vueltas)
        for tramo, celdas in self.perfil.items():
            protegidas = set()
            for zona in self.zonas.get(tramo, []):
                ultima = zona["vueltas"][-1] if zona["vueltas"] else -10**6
                if vuelta - ultima <= self.vueltas_proteccion:
                    protegidas.update(
                        self._celdas_en(
                            tramo, zona["s_ini"] - self.ventana_reduccion, zona["s_fin"]
                        )
                    )
            for i in range(len(celdas)):
                if i not in protegidas:
                    celdas[i] = min(self.v_max, celdas[i] + self.incremento_vuelta)

        self.saveLogFile(
            f"Vuelta {vuelta} limpia: perfil incrementado +{self.incremento_vuelta}"
        )

    # --- PERSISTENCIA ---
    def saveLogFile(self, data):
        try:
            with open(self.log_file, "a") as log_file:
                log_file.write(data + "\n")
        except IOError as e:
            print(f"Error escribiendo log: {e}")

    def saveData(self, vueltas):
        try:
            with open(self.csv_file, "w", newline="") as archivoCSV:
                todas = [(t, z) for t, zonas in self.zonas.items() for z in zonas]

                fichero = csv.writer(archivoCSV)
                fichero.writerow(
                    ["vueltas"]
                    + [
                        f"tramo{t}[{z['s_ini']:.0f}-{z['s_fin']:.0f}]"
                        for t, z in todas
                    ]
                )
                for v in range(0, vueltas + 1):
                    fila = [v]
                    for _, z in todas:
                        fila.append(v if v in z["vueltas"] else None)
                    fichero.writerow(fila)
        except IOError as e:
            print(f"Error guardando CSV: {e}")
