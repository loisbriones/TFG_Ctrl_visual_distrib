import math
import csv
import os
import numpy as np
from datetime import datetime


class EstrategiaPerfil:
    """
    Algoritmo de velocidad por ZONAS de PWM uniforme sobre una cadena de
    celdas equiespaciadas.

    Sustituye al modelo anterior de tramos + longitud de arco, que en pista
    generaba tramos espurios (la trayectoria de calibración llega con ~1,25
    vueltas y puntos duplicados) y zonas de derrape gigantes e imprecisas.

    ============================================================
    CONCEPTOS CLAVE PARA SEGUIR EL CÓDIGO
    ============================================================

    TRAYECTORIA: lista ordenada de puntos (x, y) en píxeles de imagen que el
    coche recorrió durante la vuelta de calibración, vista por ESTA cámara.
    Una cámara ve UNA única porción contigua del circuito (decisión de
    diseño: no se contemplan porciones sueltas). Esa porción puede tener
    huecos tapados por dentro (el circuito no se ve pero continúa).

    FILTRADO: como la calibración graba más de una vuelta, la lista trae una
    cola que repite puntos ya grabados. `setTrayectoria` recorta esa cola
    (cuando varios puntos seguidos vuelven a pasar por donde ya se pasó, la
    vuelta se cerró). Si la grabación empezó en mitad de la porción visible,
    la lista queda "rotada" ([mitad B, salto, mitad A]) y se reordena para
    que empiece por el principio real de la porción.

    CELDA: la trayectoria filtrada se remuestrea colocando un punto cada
    `paso_celda` px SOBRE la polilínea (interpolando en los huecos pequeños
    entre puntos de calibración). 1 punto remuestreado = 1 celda. Así las
    celdas quedan repartidas de forma uniforme sobre la línea real, en vez
    de depender de cómo cayeran los puntos crudos.

    CELDA GIGANTE: un salto entre puntos consecutivos mayor que
    `umbral_celda_gigante` es un hueco tapado (no vemos el circuito, pero
    sigue). No se interpola: se crea UNA celda especial sin punto dentro,
    con su longitud real en px y su propio PWM. No podemos ser precisos
    dentro del hueco, pero sí ajustar su valor (curva tapada -> baja;
    recta tapada -> sube).

    CADENA: la secuencia completa de celdas (normales + gigantes) en orden
    de recorrido. Si el final conecta con el inicio la cadena es CERRADA
    (la cámara ve el circuito completo; índices módulo N); si no, ABIERTA.

    ZONA: intervalo contiguo de celdas [ini, fin] con UN único valor de PWM
    para todas ellas. Las zonas PARTICIONAN la cadena: toda celda pertenece
    exactamente a una zona. Tipos:
      - "libre":   sin derrapes conocidos; sube en cada vuelta limpia.
      - "derrape": hubo derrape; baja al derrapar y queda protegida.
      - "gigante": la celda gigante; permanente, sube y baja como las demás.
    Estado inicial: UNA zona libre cubriendo toda la cadena a v_min (más una
    zona por cada celda gigante). Los derrapes van "rompiendo" esa zona.

    CICLO DE APRENDIZAJE:
      - Derrape cerrado en [ini, fin]: se crea una zona de derrape desde
        `retroceso_creacion` px ANTES del inicio (el exceso de velocidad se
        gesta antes de manifestarse) hasta el fin, con pwm = mínimo de lo
        que cubría - reduccion_derrape.
      - Derrape que cae dentro/al lado de una zona de derrape existente:
        se FUSIONAN (unión de intervalos, mínimo pwm - reducción) y solo se
        retrocede `retroceso_fusion` px más. Si cada reincidencia retro-
        cediera los 300 px completos, el circuito entero acabaría cubierto
        de zonas protegidas y el perfil no subiría nunca.
      - Vuelta limpia: suben +incremento_vuelta TODAS las zonas salvo las
        que tuvieron derrape hace <= vueltas_proteccion vueltas. Al subir
        POR ZONAS, una curva lenta ya no limita la velocidad del resto.

    DERRAME ENTRE CÁMARAS: el orden de las cámaras es secuencial (el coche
    se va por delante y entra por detrás). Si el retroceso de una zona se
    sale por el INICIO de la cadena abierta con R px aún por reducir, se
    apunta en `reduccion_pendiente`; el controlador lo lee y llama a
    `aplicar_reduccion_externa(R)` de la cámara PRECEDENTE, que crea la
    zona al FINAL de su propia trayectoria.

    FLUJO DE LLAMADAS desde CarControllerNode:
      1. setTrayectoria(puntos, varias_camaras)  -> una vez, tras calibrar
      2. actualizar_estado(front, back, frame, vuelta) -> PWM o None
      3. registrar_vuelta(v, limpia)   -> al cruzar meta; sin esto no sube
      4. consumir_reduccion_pendiente() / aplicar_reduccion_externa(px, v)
      5. notificar_perdida_vision(v, frame) -> al conmutar de cámara activa

    LOG DE EJECUCIÓN (derrapesLog_<nodo>_<camara>.txt), prefijos grep-ables:
      [INIT]    parámetros con los que corre la instancia
      [TRAY]    filtrado (recorte de cola, rotación), remuestreo, celdas
                gigantes y cadena final celda a celda
      [FRAME]   una línea por llamada a actualizar_estado
      [DERRAPE] transiciones de la máquina de estados
      [ZONA]    altas, fusiones, castigos y estado completo de las zonas
      [PERFIL]  partición completa de zonas con su PWM tras cada cambio
      [VUELTA]  decisión al cruzar meta: qué zonas suben y cuáles no
    """

    def __init__(self, v_max, v_min, node_name="node", camara_id="cam"):
        """
        Crea una instancia del algoritmo para UNA cámara concreta.

        Solo guarda parámetros y deja el estado vacío: la trayectoria llega
        después por `setTrayectoria`. También abre el fichero de log.
        """
        self.v_max = float(v_max)
        self.v_min = float(v_min)

        # ------------------------------------------------------------------
        # Parámetros de detección de derrape y localización
        # ------------------------------------------------------------------
        # Umbral de distancia perpendicular (px) para considerar derrape.
        # Ajustado empíricamente en pista: con 15 px (valor original) el
        # derrape se confirmaba tan tarde que el coche ya se había salido;
        # con 8 px se reacciona a tiempo
        self.umbral_derrape = 12.0
        # Ruido: si la etiqueta frontal está más lejos que esto de la ruta
        # se ignora el frame entero (detección falsa)
        self.max_dist_ruta = 80.0
        # Zona muerta (en CELDAS) junto a los extremos de la cadena abierta
        # y junto a las celdas gigantes: al entrar/salir del encuadre la
        # detección de la pose no es fiable y daría falsos derrapes
        self.margen_extremo_celdas = 2 

        # ------------------------------------------------------------------
        # Parámetros de construcción de la cadena de celdas
        # ------------------------------------------------------------------
        # Separación entre celdas (px sobre la polilínea). Las celdas se
        # generan interpolando, así que este valor manda sobre la densidad
        # real de puntos de calibración
        self.paso_celda = 15.0
        # Salto entre puntos consecutivos a partir del cual se considera un
        # hueco tapado (celda gigante). En los logs de pista reales los
        # huecos por salida de encuadre miden 189-489 px y los cortes falsos
        # del algoritmo antiguo 60-133 px: 150 separa bien ambos mundos
        self.umbral_celda_gigante = 150.0
        # Distancia (px) por debajo de la cual un punto "repite" uno antiguo:
        # se usa para detectar la cola duplicada de la calibración. Del orden
        # de la separación entre nodos de la ruta limpia (15 px) con holgura
        self.umbral_duplicado = 20.0
        # Nº de puntos SEGUIDOS repitiendo puntos antiguos para dar la vuelta
        # por cerrada y recortar. Exigir una racha evita recortar en el cruce
        # del circuito en ocho, donde la trayectoria se toca solo un instante
        self.racha_duplicado = 5
        # Al buscar duplicados se ignoran los últimos N puntos (los vecinos
        # inmediatos siempre están cerca y no son duplicados de nada)
        self.exclusion_duplicado = 8
        # Distancia máxima (px) entre el último y el primer punto filtrados
        # para considerar que el final conecta con el inicio (la grabación
        # volvió al punto de partida). Tras el recorte de la cola duplicada
        # el hueco de cierre real queda por debajo de ~40 px
        self.umbral_cierre = 60.0

        # ------------------------------------------------------------------
        # Parámetros del perfil de zonas
        # ------------------------------------------------------------------
        # Subida por vuelta limpia y bajada por derrape (unidades de PWM)
        self.incremento_vuelta = 1.0
        self.reduccion_derrape = 2.0
        # Retroceso (px) al CREAR una zona de derrape nueva: el coche llegó
        # demasiado rápido, hay que frenar bastante antes del punto donde
        # se manifestó la pérdida de adherencia
        self.retroceso_creacion = 150.0
        # Retroceso (px) al FUSIONAR un derrape con una zona existente
        # (~2 celdas). Si cada reincidencia retrocediera los 300 px completos
        # el circuito entero acabaría cubierto y el perfil no subiría nunca
        self.retroceso_fusion = 60.0
        # Holgura (en celdas) para considerar que un derrape "toca" una zona
        # existente y hay que fusionar en vez de crear
        self.margen_fusion_celdas = 1
        # Vueltas sin derrapar en una zona antes de dejarla subir de nuevo
        self.vueltas_proteccion = 2

        # ------------------------------------------------------------------
        # Estado de la cadena de celdas (lo rellena setTrayectoria)
        # ------------------------------------------------------------------
        # Trayectoria filtrada que se usó para construir la cadena (para
        # idempotencia y depuración)
        self.trayectoriaUsada = None
        # Matriz (P, 2) con la posición de cada celda NORMAL (las gigantes
        # no tienen punto: no se puede localizar nada dentro de ellas)
        self._puntos = None
        # Vector (P,): índice de celda de cada fila de _puntos
        self._celda_de_punto = None
        # Nº total de celdas de la cadena (normales + gigantes)
        self.n_celdas = 0
        # Vector (n_celdas,) bool: True en las celdas gigantes
        self._es_gigante = None
        # Vector (n_celdas,): px reales que cubre cada celda (paso_celda en
        # las normales; la longitud del salto en las gigantes)
        self._long_celdas = None
        # Vector (n_celdas,): px acumulados al INICIO de cada celda
        self._long_acum = None
        # Longitud total de la cadena en px
        self._long_total = 0.0
        # True si la cadena es cerrada (la cámara ve el circuito completo)
        self._cerrada = False

        # ------------------------------------------------------------------
        # Zonas: lista ORDENADA de dicts que particionan la cadena
        #   {"ini", "fin" (ambos inclusive), "pwm", "tipo", "vueltas"}
        # ------------------------------------------------------------------
        self.zonas = []
        # Nº total de eventos de derrape detectados (lo usa el controlador
        # para saber si en la última vuelta hubo derrapes)
        self.derrapes_contador = 0
        # Valor del contador la última vez que se llamó a registrar_vuelta
        self._derrapes_vuelta_anterior = 0

        # ------------------------------------------------------------------
        # Estado del derrape EN CURSO (máquina de estados)
        # ------------------------------------------------------------------
        self._estado_derrapando = False
        # Celda donde empezó y celda del último frame del derrape en curso
        self._derrape_idx_ini = 0
        self._derrape_idx_fin = 0
        # Última distancia perpendicular medida de la pegatina trasera
        self._dist_derrape = 0.0

        # ------------------------------------------------------------------
        # Derrame entre cámaras: px de reducción que no cupieron por el
        # inicio de la cadena y deben aplicarse en la cámara precedente
        # ------------------------------------------------------------------
        self._reduccion_pendiente = 0.0

        # Px de trayectoria que le quedan al coche por delante en esta cámara
        self._margen_restante = 0.0
        # True si en el último fotograma el coche se localizó sobre la ruta
        self._frame_valido = False

        # ------------------------------------------------------------------
        # Ficheros de log (uno por nodo y cámara para que no se pisen)
        # ------------------------------------------------------------------
        directorio_logs = "/ros2_ws/src/image_processor_pkg/logs_carrera"
        os.makedirs(directorio_logs, exist_ok=True)

        self.log_file = f"{directorio_logs}/derrapesLog_{node_name}_{camara_id}.txt"
        self.csv_file = f"{directorio_logs}/datosDerrapes_{node_name}_{camara_id}.csv"

        try:
            # Se abre en modo "w" para vaciar el log de la sesión anterior
            with open(self.log_file, "w") as f:
                f.write(f"=== LOG INICIADO {datetime.now().isoformat()} ===\n")
        except IOError as e:
            print(f"Error inicializando log: {e}")

        # Volcado de TODOS los parámetros vigentes para que el log sea
        # autocontenido (se ajustan a mano en el código)
        self.saveLogFile(f"[INIT] nodo={node_name} camara={camara_id}")
        self.saveLogFile(
            f"[INIT] v_min={self.v_min:.0f} v_max={self.v_max:.0f} | "
            f"umbral_derrape={self.umbral_derrape} max_dist_ruta={self.max_dist_ruta} "
            f"margen_extremo_celdas={self.margen_extremo_celdas}"
        )
        self.saveLogFile(
            f"[INIT] paso_celda={self.paso_celda} "
            f"umbral_celda_gigante={self.umbral_celda_gigante} "
            f"umbral_duplicado={self.umbral_duplicado} "
            f"racha_duplicado={self.racha_duplicado} "
            f"exclusion_duplicado={self.exclusion_duplicado} "
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
        """Última distancia perpendicular (px) de la pegatina trasera a la
        trayectoria. Se publica en CarControlTelemetry."""
        return self._dist_derrape

    @property
    def estado_derrapando(self):
        """True si ahora mismo hay un derrape en curso (sin cerrar)."""
        return self._estado_derrapando

    @property
    def margen_restante(self):
        """Px de trayectoria que le quedan al coche dentro del campo de
        visión de esta cámara (infinito si la cadena es cerrada)."""
        return self._margen_restante

    @property
    def frame_valido(self):
        """True si el último fotograma procesado localizó el coche sobre la
        trayectoria (no fue descartado por ruido)."""
        return self._frame_valido

    @property
    def reduccion_pendiente(self):
        """Px de reducción que se salieron por el inicio de la cadena y que
        el controlador debe reenviar a la cámara precedente."""
        return self._reduccion_pendiente

    def consumir_reduccion_pendiente(self):
        """Devuelve los px de reducción pendientes y los pone a cero. El
        controlador la llama tras cada actualizar_estado: si devuelve > 0
        debe llamar a aplicar_reduccion_externa de la cámara precedente."""
        pendiente = self._reduccion_pendiente
        self._reduccion_pendiente = 0.0
        return pendiente

    def set_velocidades(self, v_max, v_min):
        """Actualiza los límites de PWM en caliente. No toca las zonas ya
        aprendidas: solo cambia el tope aplicado en _velocidad_en."""
        self.saveLogFile(
            f"[INIT] Límites PWM cambiados en caliente: "
            f"v_min {self.v_min:.0f}->{float(v_min):.0f}, "
            f"v_max {self.v_max:.0f}->{float(v_max):.0f} (las zonas no se tocan)"
        )
        self.v_max = float(v_max)
        self.v_min = float(v_min)

    # ======================================================================
    # CONSTRUCCIÓN DE LA CADENA DE CELDAS
    # ======================================================================
    def setTrayectoria(self, trayectoria, varias_camaras=False):
        """
        Procesa la trayectoria de calibración de la cámara. Se llama UNA vez.

        Dos pasos, y a partir de ahí solo se trabaja con las celdas:
          1. FILTRAR la lista de puntos: recortar la cola duplicada de la
             calibración y, si procede, rotar la lista para que empiece por
             el principio real de la porción visible.
          2. GENERAR LAS CELDAS: remuestrear la polilínea a `paso_celda` px,
             convirtiendo los saltos grandes en celdas gigantes.

        `varias_camaras` desambigua el caso "el final conecta con el inicio
        y hay un salto grande por medio": con una sola cámara significa que
        ve el circuito completo con un hueco tapado (cadena CERRADA con
        celda gigante); con varias cámaras significa que la grabación empezó
        en mitad de la porción visible y el salto es el resto del circuito
        que ven las otras (se ROTA y la cadena queda ABIERTA).
        """
        # Idempotente: la calibración solo debe procesarse una vez
        if self.trayectoriaUsada is not None:
            self.saveLogFile(
                "[TRAY] setTrayectoria ignorado: ya hay una trayectoria cargada"
            )
            return

        puntos = np.asarray(trayectoria, dtype=np.float32)
        if puntos.ndim != 2 or len(puntos) < 2:
            self.saveLogFile(
                f"[TRAY] Trayectoria RECHAZADA: hacen falta >=2 puntos (x, y) "
                f"(recibido con forma {puntos.shape})"
            )
            return

        # --- Paso 1: filtrado ---
        filtrados, cerrada = self._filtrar_trayectoria(puntos, varias_camaras)
        if len(filtrados) < 2:
            self.saveLogFile(
                "[TRAY] Trayectoria RECHAZADA: tras el filtrado quedan <2 puntos"
            )
            return
        self.trayectoriaUsada = filtrados
        self._cerrada = cerrada

        # --- Paso 2: celdas ---
        self._construir_celdas(filtrados)

        # --- Zonas iniciales: una zona libre cubriéndolo todo a v_min, con
        # cada celda gigante separada en su propia zona permanente ---
        self.zonas = []
        ini_libre = None
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

    def _filtrar_trayectoria(self, puntos, varias_camaras):
        """
        Paso 1 de setTrayectoria: limpia la lista de puntos de calibración.
        Devuelve (puntos_filtrados, cerrada).

        a) RECORTE DE LA COLA DUPLICADA: la calibración graba desde que
           arranca el sistema hasta el 2º cruce de meta, o sea, MÁS de una
           vuelta. Se recorre la lista midiendo la distancia de cada punto a
           los puntos antiguos (excluyendo los `exclusion_duplicado` vecinos
           recientes): cuando `racha_duplicado` puntos seguidos caen a menos
           de `umbral_duplicado` px de puntos antiguos, el coche está
           repitiendo recorrido -> la vuelta se cerró y se recorta ahí.

        b) CIERRE / ROTACIÓN: tras el recorte, si el final queda cerca del
           inicio (< umbral_cierre) la grabación volvió al punto de partida:
             - sin saltos grandes -> la cámara ve el circuito completo:
               cadena CERRADA;
             - con saltos grandes y VARIAS cámaras -> la grabación empezó en
               mitad de la porción visible y el salto es el resto del
               circuito: se ROTA la lista para empezar tras el salto y la
               cadena queda ABIERTA;
             - con saltos grandes y UNA cámara -> ve el circuito completo
               con huecos tapados: cadena CERRADA (los saltos serán celdas
               gigantes).
        """
        n = len(puntos)

        # a) recorte de la cola duplicada
        corte = n
        racha = 0
        inicio_racha = n
        for j in range(1, n):
            limite = j - self.exclusion_duplicado
            if limite <= 0:
                continue
            d2 = np.sum((puntos[:limite] - puntos[j]) ** 2, axis=1)
            if math.sqrt(float(d2.min())) < self.umbral_duplicado:
                if racha == 0:
                    inicio_racha = j
                racha += 1
                if racha >= self.racha_duplicado:
                    corte = inicio_racha
                    break
            else:
                racha = 0

        if corte < n:
            self.saveLogFile(
                f"[TRAY] Filtrado: {n - corte} puntos de cola duplicada "
                f"recortados (racha de {self.racha_duplicado} puntos a "
                f"<{self.umbral_duplicado:.0f} px de puntos antiguos a partir "
                f"del punto {corte}); quedan {corte} de {n}"
            )
        else:
            self.saveLogFile(
                f"[TRAY] Filtrado: sin cola duplicada detectada "
                f"({n} puntos se mantienen)"
            )
        filtrados = puntos[:corte]
        if len(filtrados) < 2:
            return filtrados, False

        # b) cierre / rotación
        dif = np.diff(filtrados, axis=0)
        dist = np.hypot(dif[:, 0], dif[:, 1])
        saltos = [int(i) for i in np.nonzero(dist > self.umbral_celda_gigante)[0]]
        cierre = math.hypot(
            float(filtrados[-1][0] - filtrados[0][0]),
            float(filtrados[-1][1] - filtrados[0][1]),
        )
        cerrada = False
        if cierre <= self.umbral_cierre:
            if saltos and varias_camaras:
                # La grabación empezó en mitad de la porción: [mitad B,
                # salto, mitad A]. Se rota para empezar justo tras el salto
                # (el salto es el resto del circuito, que ven otras cámaras,
                # y desaparece de esta cadena)
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
                    f"[TRAY] Filtrado: el final conecta con el inicio "
                    f"({cierre:.1f} px <= {self.umbral_cierre:.0f}) y hay un "
                    f"salto de {dist[k]:.1f} px en el punto {k}: la grabación "
                    f"empezó en mitad de la porción -> lista ROTADA para "
                    f"empezar tras el salto; cadena ABIERTA"
                )
            else:
                cerrada = True
                motivo = (
                    "sin saltos grandes"
                    if not saltos
                    else f"{len(saltos)} salto(s) tapado(s) que serán celdas "
                         f"gigantes (única cámara)"
                )
                self.saveLogFile(
                    f"[TRAY] Filtrado: el final conecta con el inicio "
                    f"({cierre:.1f} px <= {self.umbral_cierre:.0f}), {motivo} "
                    f"-> cadena CERRADA (esta cámara ve el circuito completo)"
                )
        else:
            self.saveLogFile(
                f"[TRAY] Filtrado: el final NO conecta con el inicio "
                f"({cierre:.1f} px > {self.umbral_cierre:.0f}) -> cadena "
                f"ABIERTA (porción del circuito); saltos interiores "
                f"tapados: {len(saltos)}"
            )
        return filtrados, cerrada

    def _construir_celdas(self, p):
        """
        Paso 2 de setTrayectoria: remuestrea la polilínea filtrada colocando
        una celda cada `paso_celda` px, interpolando linealmente entre los
        puntos de calibración (así los huecos pequeños quedan rellenos con
        puntos nuestros y las celdas uniformemente repartidas). Los saltos
        mayores que `umbral_celda_gigante` no se interpolan: generan una
        celda gigante con su longitud real.

        Deja rellenos: _puntos, _celda_de_punto, n_celdas, _es_gigante,
        _long_celdas, _long_acum y _long_total.
        """
        celdas_xy = []       # posición de cada celda normal
        celda_de_punto = []  # índice de celda de cada posición
        es_gigante = []      # por celda (normales y gigantes)
        long_celdas = []     # px que cubre cada celda

        def emitir_normal(pt):
            es_gigante.append(False)
            long_celdas.append(self.paso_celda)
            celda_de_punto.append(len(es_gigante) - 1)
            celdas_xy.append((float(pt[0]), float(pt[1])))

        # La primera celda es el propio primer punto
        emitir_normal(p[0])
        # px que faltan (dentro del segmento actual) hasta la próxima celda
        pendiente = self.paso_celda

        # Segmentos de la polilínea; en cadena cerrada se añade el segmento
        # de cierre último->primero para que las celdas cubran todo el anillo
        n_seg = len(p) if self._cerrada else len(p) - 1
        for i in range(n_seg):
            a = p[i]
            b = p[(i + 1) % len(p)]
            d = math.hypot(float(b[0] - a[0]), float(b[1] - a[1]))
            if d > self.umbral_celda_gigante:
                # Hueco tapado: una celda gigante con la longitud del salto,
                # y el muestreo se reinicia en el borde lejano del hueco
                es_gigante.append(True)
                long_celdas.append(d)
                self.saveLogFile(
                    f"[TRAY] Celda GIGANTE {len(es_gigante) - 1}: salto de "
                    f"{d:.1f} px entre ({a[0]:.1f}, {a[1]:.1f}) y "
                    f"({b[0]:.1f}, {b[1]:.1f})"
                )
                emitir_normal(b)
                pendiente = self.paso_celda
                continue
            if d == 0.0:
                continue
            recorrido = pendiente
            while recorrido <= d:
                t = recorrido / d
                emitir_normal(
                    (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
                )
                recorrido += self.paso_celda
            pendiente = recorrido - d

        # En cadena cerrada el muestreo del segmento de cierre puede dejar la
        # última celda pegada a la primera: se elimina para no duplicar
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
        # px acumulados al inicio de cada celda (para margen_restante)
        self._long_acum = np.concatenate(
            [[0.0], np.cumsum(self._long_celdas)[:-1]]
        )
        self._long_total = float(np.sum(self._long_celdas))

        # Volcado celda a celda: es el "mapa" con el que se interpretan las
        # líneas [FRAME] posteriores (solo ocurre una vez)
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
    # LOCALIZACIÓN
    # ======================================================================
    
    def _dist_a_curva_local(self, p, P_prev, P_curr, P_next, n_muescas=16):
        """
        Calcula la distancia ortogonal desde el punto `p` a una curva parabólica suave
        (polinomio de Lagrange C^1) que pasa exactamente por P_prev, P_curr y P_next.
        Evita la congelación en vértices y genera una curva de off-tracking limpia.
        """
        # Vector de parámetros u desde -1 (P_prev) hasta 1 (P_next) pasando por 0 (P_curr)
        u = np.linspace(-1.0, 1.0, n_muescas, dtype=np.float32).reshape(-1, 1)

        # Pesos vectorizados del polinomio de Lagrange
        L_prev = 0.5 * u * (u - 1.0)
        L_curr = 1.0 - (u**2)
        L_next = 0.5 * u * (u + 1.0)

        # Generación de la micro-curva suave (matriz de n_muescas x 2)
        curva = L_prev * P_prev + L_curr * P_curr + L_next * P_next

        # Proyección ortogonal sobre los micro-segmentos de la curva continua
        mejor_dist = float("inf")
        for i in range(len(curva) - 1):
            dist = self._dist_a_segmento(p, curva[i], curva[i + 1])
            if dist < mejor_dist:
                mejor_dist = dist

        return mejor_dist

    def _dist_a_segmento(self, p, A, B):
        """
        Distancia del punto p al segmento AB (proyección con t acotado a
        [0, 1]: si p "cae" más allá de un extremo, la distancia es al propio
        extremo). Es el paso fino de `localizar`: el punto de celda más
        cercano da la resolución "de celda"; proyectar sobre los segmentos
        adyacentes da la distancia perpendicular exacta, clave para que el
        umbral de derrape de 8 px tenga sentido.
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
        Convierte un punto (x, y) de la imagen en coordenadas de cadena:
        devuelve (índice de celda, distancia perpendicular), o None si aún
        no hay trayectoria cargada.

        Paso GRUESO: celda cuyo punto está más cerca (argmin vectorizado).
        Paso FINO: intenta formar un trío de celdas consecutivas (anterior,
                   actual, siguiente) para construir una curva local suave (C^1).
                   Si hay cortes por celdas gigantes o extremos abiertos, recae
                   con elegancia en la proyección por segmentos.
        """
        if self._puntos is None or len(self._puntos) == 0:
            return None

        p = np.asarray(punto, dtype=np.float32)
        d2 = np.sum((self._puntos - p) ** 2, axis=1)
        fila = int(np.argmin(d2))
        idx = int(self._celda_de_punto[fila])
        mejor_dist = math.sqrt(float(d2[fila]))

        n_filas = len(self._puntos)

        # Identificar índices de las filas vecina anterior y posterior
        f_prev = fila - 1
        f_next = fila + 1

        # Gestión de envoltura en cadena cerrada
        if self._cerrada:
            if fila == 0 and (int(self._celda_de_punto[-1]) + 1) % self.n_celdas == int(
                self._celda_de_punto[0]
            ):
                f_prev = n_filas - 1
            if fila == n_filas - 1 and (
                int(self._celda_de_punto[-1]) + 1
            ) % self.n_celdas == int(self._celda_de_punto[0]):
                f_next = 0

        # Comprobar si existe continuidad lógica en la cuadrícula (sin celdas gigantes por medio)
        tengo_prev = (
            f_prev >= 0
            and self._celda_de_punto[f_prev] == (idx - 1) % self.n_celdas
        )
        tengo_next = (
            f_next < n_filas
            and self._celda_de_punto[f_next] == (idx + 1) % self.n_celdas
        )

        # CASO IDEAL: Trío continuo -> Proyectamos sobre curva suave C^1
        if tengo_prev and tengo_next:
            mejor_dist = self._dist_a_curva_local(
                p,
                self._puntos[f_prev],
                self._puntos[fila],
                self._puntos[f_next],
            )
        else:
            # CASO DE DEGRADACIÓN (extremos abiertos o junto a celda gigante):
            # Recaemos en tu lógica original de segmentos adyacentes
            segmentos = []
            if tengo_prev:
                segmentos.append((f_prev, fila))
            if tengo_next:
                segmentos.append((fila, f_next))

            for fa, fb in segmentos:
                dist = self._dist_a_segmento(
                    p, self._puntos[fa], self._puntos[fb]
                )
                if dist < mejor_dist:
                    mejor_dist = dist

        return idx, mejor_dist

    # ======================================================================
    # LÓGICA PRINCIPAL POR FOTOGRAMA
    # ======================================================================
    def actualizar_estado(self, p_front, p_back, frame_count, vuelta):
        """
        Punto de entrada por fotograma. CarControllerNode la llama con las
        posiciones de las dos pegatinas y devuelve el PWM a aplicar, o None
        si el fotograma se descarta (el controlador mantiene entonces la
        velocidad anterior).

        `frame_count` es el contador de la CÁMARA dueña de esta instancia (el
        campo n_frame del CarLocation), no un contador global del sistema: solo
        se usa para etiquetar el log, y sus números casan con los que van
        quemados en las imágenes de debug de esa cámara. Los números de dos
        cámaras no son comparables entre sí (ver NUMERACION_FRAMES.md).

        Pasos:
          1. Localiza la pegatina FRONTAL (define la posición del coche).
             Si está a más de `max_dist_ruta` px es una detección falsa y el
             frame se descarta entero; además, si había un derrape abierto
             se CIERRA (el coche salió de la trayectoria: no hay más puntos
             que lo puedan extender).
          2. Actualiza `_margen_restante`.
          3. Localiza la pegatina TRASERA y alimenta la máquina de estados
             del derrape con su distancia perpendicular.
          4. Si hay un derrape abierto y el coche llega al final de la
             cadena abierta, se cierra ahí (misma razón que en 1).
          5. Devuelve el PWM de la ZONA a la que pertenece la celda actual.

        La decisión de velocidad SOLO depende de la posición actual: no
        importa si se perdieron fotogramas o llegaron fuera de orden.
        """
        self._frame_valido = False

        loc_front = self.localizar(p_front)
        if loc_front is None:
            self.saveLogFile(
                f"[FRAME {frame_count} v={vuelta}] DESCARTADO: aún no hay "
                f"trayectoria cargada"
            )
            return None

        idx_f, dist_f = loc_front

        if dist_f > self.max_dist_ruta:
            self._margen_restante = 0.0
            # El coche desapareció de la ruta con un derrape abierto: se
            # cierra con lo que llevaba (no habrá más puntos que lo extiendan)
            if self._estado_derrapando:
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

        # Margen restante = px de cadena por delante del coche
        if self._cerrada:
            self._margen_restante = float("inf")
        else:
            self._margen_restante = self._long_total - float(
                self._long_acum[idx_f]
            )

        # Detección del derrape con la etiqueta trasera
        loc_back = self.localizar(p_back)
        if loc_back is not None:
            idx_b, dist_b = loc_back
            self._dist_derrape = dist_b
            self._actualizar_derrape(idx_b, dist_b, vuelta, frame_count)
            texto_back = (
                f"back=({p_back[0]:.1f}, {p_back[1]:.1f})->c={idx_b} "
                f"d={dist_b:.1f} (umbral {self.umbral_derrape:.0f})"
            )
        else:
            texto_back = "back=NO localizado (máquina de derrape sin actualizar)"

        # Final de cadena abierta con derrape abierto: no hay más trayectoria
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
        Aviso del controlador: el coche dejó de verse por esta cámara (la
        cámara activa conmutó a otra). Si había un derrape abierto se cierra
        con lo que llevaba: no van a llegar más frames que lo extiendan.

        `frame_count` tiene que ser el último frame que vio ESTA cámara, no el
        del mensaje que disparó el aviso (que viene de la cámara nueva y está en
        otra escala de numeración): el controlador lo saca de
        ultimo_frame_camara justo por eso.
        """
        if self._estado_derrapando:
            self.saveLogFile(
                f"[DERRAPE] Frame {frame_count} v={vuelta}: el controlador "
                f"avisa de que el coche salió del campo de visión con un "
                f"derrape abierto -> se CIERRA"
            )
            self._cerrar_derrape(vuelta, frame_count)

    # ======================================================================
    # MÁQUINA DE ESTADOS DEL DERRAPE
    # ======================================================================
    def _en_borde(self, idx):
        """True si la celda idx está en la zona muerta: a menos de
        `margen_extremo_celdas` de un extremo de la cadena abierta o de una
        celda gigante. Ahí la detección de la pose no es fiable (el coche
        entra/sale del encuadre) y no deben abrirse derrapes."""
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
        Máquina de estados del derrape. Se llama en cada frame con la celda
        y la distancia perpendicular de la pegatina TRASERA.

          NO derrapando --(dist > umbral y no está en zona muerta)--> derrapando
              Se anota dónde empieza (_derrape_idx_ini = _derrape_idx_fin).
          derrapando --(dist > umbral)--> sigue derrapando
              Solo se actualiza el final: el derrape se va "alargando".
          derrapando --(dist <= umbral)--> NO derrapando
              El coche se realineó: se cierra y se registra la zona entera.

        (El cierre por salir del campo de visión está en actualizar_estado y
        en notificar_perdida_vision.) Un derrape NO se registra frame a
        frame sino como un único evento [ini, fin] al terminar: así el
        castigo y la fusión de zonas trabajan con el derrape entero.
        """
        if dist > self.umbral_derrape:
            if not self._estado_derrapando:
                if self._en_borde(idx):
                    self.saveLogFile(
                        f"[DERRAPE] Frame {frame} v={vuelta}: dist={dist:.1f} > "
                        f"umbral pero IGNORADO por zona muerta: la celda {idx} "
                        f"está a menos de {self.margen_extremo_celdas} celdas de "
                        f"un extremo o de una celda gigante"
                    )
                    return
                self._estado_derrapando = True
                self._derrape_idx_ini = idx
                self._derrape_idx_fin = idx
                self.saveLogFile(
                    f"[DERRAPE] Frame {frame} v={vuelta}: ABIERTO en celda "
                    f"{idx} (dist={dist:.1f} > umbral {self.umbral_derrape:.0f})"
                )
            else:
                self._derrape_idx_fin = idx
        else:
            if self._estado_derrapando:
                self._cerrar_derrape(vuelta, frame)

    def _cerrar_derrape(self, vuelta, frame):
        """
        Da por terminado el derrape en curso y lo registra como zona.

        Normaliza el intervalo [ini, fin] al sentido de la marcha:
          - Cadena abierta: min/max (por si hubo jitter hacia atrás).
          - Cadena cerrada: el coche avanza con índices crecientes módulo N;
            si el arco "hacia delante" de ini a fin es más corto que media
            cadena, el intervalo es ini->fin (puede envolver el origen); si
            no, fue jitter numérico y se intercambian.
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
        """Zona de la partición que contiene la celda idx (o None)."""
        for z in self.zonas:
            if z["ini"] <= idx <= z["fin"]:
                return z
        return None

    def _intervalo_a_celdas(self, ini, fin):
        """Lista de celdas del intervalo [ini, fin] en orden de recorrido.
        En cadena cerrada el intervalo puede envolver el origen (ini > fin)."""
        if fin >= ini:
            return list(range(ini, fin + 1))
        # Envuelve el origen (solo posible en cadena cerrada)
        return list(range(ini, self.n_celdas)) + list(range(0, fin + 1))

    def _extender_atras(self, idx_desde, px):
        """
        Camina hacia ATRÁS desde la celda idx_desde-1 consumiendo `px` de
        retroceso. Cada celda normal consume `paso_celda` px; cada celda
        gigante consume su longitud real (el hueco absorbe retroceso).

        Devuelve (celdas_normales, celdas_gigantes, px_sobrantes):
          - px_sobrantes > 0 solo si la cadena es abierta y el retroceso se
            salió por el inicio: es lo que hay que derramar a la cámara
            precedente (reduccion_pendiente).
        """
        normales = []
        gigantes = []
        restante = float(px)
        j = idx_desde - 1
        pasos = 0
        while restante > 0.0 and pasos < self.n_celdas:
            if j < 0:
                if not self._cerrada:
                    # Se salió por el inicio: lo que queda se derrama
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
        """Baja el PWM de la zona de una celda gigante alcanzada por un
        retroceso: dentro del hueco no hay precisión posible, pero el valor
        sí se puede ajustar (y la protección funciona igual)."""
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
        Sustituye el intervalo [a, b] de la partición por la zona `nueva`.
        Las zonas de derrape solapadas deben haberse absorbido ANTES (en
        _carvear_derrape) y las gigantes nunca caen dentro por construcción:
        aquí solo se recortan/parten zonas libres.
        """
        resultado = []
        for z in self.zonas:
            if z["fin"] < a or z["ini"] > b:
                resultado.append(z)
                continue
            # Zona libre solapada: se conservan los trozos que sobresalen
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
        Convierte el intervalo de celdas [a, b] (sin celdas gigantes) en una
        zona de derrape, absorbiendo las zonas de derrape existentes que
        solapen o queden a <= margen_fusion_celdas (fusión). El PWM nuevo es
        el mínimo de todo lo que cubría el intervalo menos reduccion_derrape
        (con suelo v_min), y el historial de vueltas se hereda de las zonas
        absorbidas.
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
                # No se fusiona a través de una celda gigante: el hueco entre
                # ambos intervalos tiene que ser terreno normal
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

        # PWM nuevo: mínimo de todas las zonas que cubren [a2, b2], menos la
        # reducción (así reincidir en una zona la baja otro escalón)
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
        Registra un derrape terminado en las celdas [ini, fin]. Hace:

          1. CONTADOR: incrementa `derrapes_contador` (lo que consulta el
             controlador para decidir si la vuelta fue limpia).
          2. RETROCESO: decide cuánto extender hacia atrás. Si el derrape
             toca una zona de derrape existente (fusión) solo
             `retroceso_fusion` px desde el inicio de la unión; si es nueva,
             `retroceso_creacion` px. El coche llegó demasiado rápido: hay
             que frenar ANTES del punto donde se manifestó el derrape.
          3. CASTIGO: convierte las celdas del derrape + retroceso en
             zona(s) de derrape (partiendo en trozos si hay celdas gigantes
             por medio, que se castigan aparte) y baja su PWM.
          4. DERRAME: si el retroceso se salió por el inicio de la cadena,
             apunta los px sobrantes en reduccion_pendiente para que el
             controlador los reenvíe a la cámara precedente.
        """
        self.derrapes_contador += 1

        celdas_int = self._intervalo_a_celdas(ini, fin)
        normales_int = [c for c in celdas_int if not self._es_gigante[c]]
        gigantes_int = [c for c in celdas_int if self._es_gigante[c]]

        # ¿El derrape toca alguna zona de derrape existente? (con holgura de
        # margen_fusion_celdas para juntar derrapes casi contiguos)
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
            retro = self.retroceso_fusion
            # El retroceso arranca en el borde trasero de la UNIÓN (la zona
            # existente ya tenía su retroceso: solo se añade un poco más)
            arranque = min([ini] + [z["ini"] for z in tocadas])
            motivo = (
                f"fusión con {len(tocadas)} zona(s) existente(s): retroceso "
                f"corto de {retro:.0f} px desde la celda {arranque}"
            )
        else:
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

        # Conjunto final de celdas a castigar, partido en tramos contiguos
        # (las celdas gigantes rompen los tramos y se castigan aparte).
        # Las celdas de las zonas tocadas entran también en el conjunto: así
        # la fusión produce UN único tramo contiguo y una única bajada de
        # PWM (si no, el derrape y el retroceso quedarían como dos tramos a
        # ambos lados de la zona vieja y la absorción en cadena la bajaría
        # dos veces por un solo derrape)
        objetivo = set(normales_int) | set(normales_atras)
        for z in tocadas:
            objetivo.update(range(z["ini"], z["fin"] + 1))
        objetivo = sorted(objetivo)
        for g in sorted(set(gigantes_int) | set(gigantes_atras)):
            self._castigar_gigante(g, vuelta)

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
                f"[ZONA] Derrame: el retroceso se salió por el inicio de la "
                f"cadena con {sobrante:.1f} px por reducir -> "
                f"reduccion_pendiente={self._reduccion_pendiente:.1f} px "
                f"(el controlador debe avisar a la cámara precedente)"
            )

        self._log_zonas(f"estado tras el derrape de la vuelta {vuelta}")
        self._log_perfil("tras castigo por derrape")

    def aplicar_reduccion_externa(self, px, vuelta):
        """
        Aviso del controlador: la cámara SIGUIENTE se quedó sin trayectoria
        al retroceder una zona y faltan `px` por reducir. Como el orden es
        secuencial (el coche se nos va por delante), esos px corresponden al
        FINAL de nuestra trayectoria: se crea/amplía ahí una zona de derrape.

        No incrementa derrapes_contador (el derrape ya lo contó la cámara
        que lo vio), pero la zona sí queda protegida por su historial.
        Si tampoco cabe aquí, el sobrante vuelve a quedar pendiente (cascada
        hacia la cámara anterior).
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
        Velocidad en la celda idx: el PWM de su zona, con tope v_max.
        A propósito es trivial: toda la inteligencia está en cómo se parten
        y ajustan las zonas, no en cómo se leen. Si aún no hay zonas
        (trayectoria sin cargar) devuelve v_min por prudencia.
        """
        zona = self._zona_de(idx)
        if zona is None:
            return self.v_min
        return float(min(zona["pwm"], self.v_max))

    def registrar_vuelta(self, vuelta, vuelta_limpia):
        """
        Aviso del controlador al completarse una vuelta. Es el mecanismo por
        el que el perfil SUBE (la bajada ocurre al derrapar).

        `vuelta_limpia` la calcula CarControllerNode con visión global (True
        si NINGUNA cámara registró derrapes). Cada instancia hace además su
        comprobación local comparando su contador con el de la vuelta
        anterior, por si el controlador se equivocara.

        Si la vuelta fue limpia, TODAS las zonas suben +incremento_vuelta
        (tope v_max)... excepto las PROTEGIDAS: zonas cuyo último derrape
        (o castigo) fue hace `vueltas_proteccion` vueltas o menos. Subir POR
        ZONAS es el punto clave: las zonas sin problemas ganan velocidad
        aunque haya una curva conflictiva contenida en otra zona.
        """
        hubo_derrape_local = self.derrapes_contador != self._derrapes_vuelta_anterior
        self.saveLogFile(
            f"[VUELTA {vuelta}] limpia_global={vuelta_limpia} | derrapes de esta "
            f"cámara: {self._derrapes_vuelta_anterior}->{self.derrapes_contador} "
            f"(hubo_derrape_local={hubo_derrape_local})"
        )
        self._derrapes_vuelta_anterior = self.derrapes_contador

        if not vuelta_limpia or hubo_derrape_local:
            motivo = (
                "esta cámara registró derrapes"
                if hubo_derrape_local
                else "otra cámara registró derrapes (aviso global del controlador)"
            )
            self.saveLogFile(f"[VUELTA {vuelta}] El perfil NO sube: {motivo}")
            return

        subidas = 0
        for z in self.zonas:
            ultima = z["vueltas"][-1] if z["vueltas"] else None
            if ultima is not None and vuelta - ultima <= self.vueltas_proteccion:
                self.saveLogFile(
                    f"[VUELTA {vuelta}] zona [{z['ini']}-{z['fin']}]({z['tipo']}) "
                    f"PROTEGIDA: último derrape en vuelta {ultima}, le quedan "
                    f"{self.vueltas_proteccion - (vuelta - ultima)} vuelta(s) "
                    f"de protección"
                )
                continue
            z["pwm"] = min(self.v_max, z["pwm"] + self.incremento_vuelta)
            subidas += 1

        self.saveLogFile(
            f"[VUELTA {vuelta}] limpia: {subidas} de {len(self.zonas)} zonas "
            f"suben +{self.incremento_vuelta:.0f} (tope v_max={self.v_max:.0f})"
        )
        self._log_perfil(f"tras la vuelta {vuelta} limpia")

    # ======================================================================
    # VOLCADOS DE ESTADO (solo escriben en el log, no modifican nada)
    # ======================================================================
    def _log_perfil(self, motivo):
        """
        Vuelca al log la partición COMPLETA de zonas con su PWM. Se llama
        tras cada modificación, así comparando dos volcados consecutivos se
        ve exactamente qué cambió. La línea "perfil=" pinta el PWM celda a
        celda agrupado por zonas (estilo [[55,55,55],[58,58]]).
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
        """Vuelca todas las zonas con su historial de vueltas, para ver cómo
        evoluciona la partición a lo largo de la carrera."""
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
        """Añade una línea al log de texto de esta cámara, con marca de
        tiempo HH:MM:SS.mmm para poder correlacionarla con la telemetría.
        Los errores de E/S solo se imprimen: un fallo de disco nunca debe
        tumbar el algoritmo."""
        try:
            with open(self.log_file, "a") as log_file:
                marca = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                log_file.write(f"{marca} {data}\n")
        except IOError as e:
            print(f"Error escribiendo log: {e}")

    def saveData(self, vueltas):
        """
        Vuelca a CSV el historial de zonas castigadas para analizarlo
        después. Una columna por zona con historial (cabecera
        "tipo[ini-fin]") y una fila por vuelta; la celda lleva el número de
        vuelta si esa zona registró un castigo en esa vuelta.
        """
        try:
            with open(self.csv_file, "w", newline="") as archivoCSV:
                castigadas = [z for z in self.zonas if z["vueltas"]]
                fichero = csv.writer(archivoCSV)
                fichero.writerow(
                    ["vueltas"]
                    + [f"{z['tipo']}[{z['ini']}-{z['fin']}]" for z in castigadas]
                )
                for v in range(0, vueltas + 1):
                    fila = [v]
                    for z in castigadas:
                        fila.append(v if v in z["vueltas"] else None)
                    fichero.writerow(fila)
        except IOError as e:
            print(f"Error guardando CSV: {e}")
