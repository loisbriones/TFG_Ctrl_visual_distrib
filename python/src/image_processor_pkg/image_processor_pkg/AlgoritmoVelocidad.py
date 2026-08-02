import json
import math
import os
import numpy as np
from datetime import datetime


class EstrategiaPerfil:
    """
    Algoritmo de velocidad por ZONAS de PWM uniforme sobre una cadena de
    celdas equiespaciadas.

    Sustituye al modelo anterior de tramos + longitud de arco, que en pista
    generaba tramos espurios y zonas de derrape gigantes e imprecisas.

    ============================================================
    CONCEPTOS CLAVE PARA SEGUIR EL CÓDIGO
    ============================================================

    TRAYECTORIA: lista ordenada de puntos (x, y) en píxeles de imagen que el
    coche recorrió durante la vuelta de calibración, vista por ESTA cámara.
    Una cámara ve UNA única porción contigua del circuito (decisión de
    diseño: no se contemplan porciones sueltas). Esa porción puede tener
    huecos tapados por dentro (el circuito no se ve pero continúa).

    ORIENTACIÓN: la lista de entrada ya es UNA vuelta limpia, porque el
    controlador solo acumula puntos entre el primer y el segundo paso por
    meta. Lo único que queda por resolver es que esa vuelta empieza EN LA
    META, que no tiene por qué caer en un extremo de la porción visible: si
    cae en mitad de ella, la lista llega "rotada" ([mitad B, salto, mitad A])
    y `setTrayectoria` la reordena para que empiece por el principio real de
    la porción.

    CELDA: la trayectoria se remuestrea colocando un punto cada
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
        cediera el `retroceso_creacion` completo, el circuito entero acabaría
        cubierto de zonas protegidas y el perfil no subiría nunca.
      - Al cruzar meta suben +incremento_vuelta todas las zonas salvo las
        que tuvieron derrape hace <= vueltas_proteccion vueltas. La decisión
        es POR ZONA y no depende de si la vuelta fue limpia en el resto del
        circuito: una curva conflictiva se frena a sí misma mientras las
        rectas siguen ganando velocidad vuelta tras vuelta.

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
      [TRAY]    orientación (cierre / rotación), remuestreo, celdas
                gigantes y cadena final celda a celda
      [FRAME]   una línea por llamada a actualizar_estado
      [DERRAPE] transiciones de la máquina de estados
      [ZONA]    altas, fusiones, castigos y estado completo de las zonas
      [PERFIL]  partición completa de zonas con su PWM tras cada cambio
      [VUELTA]  decisión al cruzar meta: qué zonas suben y cuáles no
    """

    def __init__(
        self,
        v_max,
        v_min,
        node_name="node",
        camara_id="cam",
        # --- modo de operación (ver params.yaml -> cars.<coche>.modo) ---
        modo="automatico",
        # --- detección de derrape y localización ---
        umbral_derrape=12.0,
        max_dist_ruta=80.0,
        margen_extremo_celdas=2,
        # --- construcción de la cadena de celdas ---
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
        """
        Crea una instancia del algoritmo para UNA cámara concreta.

        Solo guarda parámetros y deja el estado vacío: la trayectoria llega
        después por `setTrayectoria`. También abre el fichero de log.

        Todos los umbrales llegan como argumentos con valor por defecto. En el
        sistema es CarControllerNode quien los lee de params.yaml
        (controller.algoritmo.*) y los pasa aquí; los defaults son esos mismos
        valores, para que la clase se pueda instanciar suelta y probar fuera
        de ROS sin arrastrar el fichero de configuración. La explicación de
        para qué sirve cada uno está en params.yaml, junto a su valor.
        """
        self.v_max = float(v_max)
        self.v_min = float(v_min)

        # Modo de operación. De los cuatro, solo "politica" cambia lo que hace
        # esta clase: le congela el perfil (lo carga de un JSON y ni los
        # derrapes ni el paso por meta pueden moverlo). En "manual",
        # "incremental" y "automatico" el algoritmo se comporta EXACTAMENTE
        # igual; quién publica el PWM y con qué valor es cosa del controlador.
        self.modo = modo
        # Se guarda porque el JSON de la política lo lleva dentro, para poder
        # saber de qué cámara es un fichero suelto
        self.camara_id = camara_id

        # Detección de derrape y localización
        self.umbral_derrape = float(umbral_derrape)
        self.max_dist_ruta = float(max_dist_ruta)
        self.margen_extremo_celdas = int(margen_extremo_celdas)

        # Construcción de la cadena de celdas
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
        # Estado de la cadena de celdas (lo rellena setTrayectoria)
        # ------------------------------------------------------------------
        # Trayectoria (ya orientada) que se usó para construir la cadena, para
        # idempotencia y depuración
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

        # --- Ficheros del modo política ---
        # Van en la raíz del paquete, junto a las cachés de trayectoria, porque
        # es lo que el docker-compose monta desde el host: así la plantilla se
        # puede editar desde fuera del contenedor.
        #
        # Son DOS ficheros distintos a propósito. La PLANTILLA se reescribe en
        # cada arranque con la partición recién construida; la POLÍTICA es la
        # que se lee, y se crea copiando la plantilla y editando los pwm. Si
        # fuese el mismo fichero, arrancar en modo política machacaría la
        # política editada con la partición inicial antes de poder leerla.
        base_politica = f"/ros2_ws/src/image_processor_pkg/politica_{node_name}_{camara_id}"
        self.fichero_politica = f"{base_politica}.json"
        self.fichero_plantilla = f"{base_politica}_PLANTILLA.json"

        try:
            # Se abre en modo "w" para vaciar el log de la sesión anterior
            with open(self.log_file, "w") as f:
                f.write(f"=== LOG INICIADO {datetime.now().isoformat()} ===\n")
        except IOError as e:
            print(f"Error inicializando log: {e}")

        # Volcado de TODOS los parámetros vigentes para que el log sea
        # autocontenido: como ahora llegan de params.yaml, esta es la única
        # forma de saber después con qué valores corrió la sesión. La
        # herramienta de análisis los lee de aquí (umbral_derrape,
        # max_dist_ruta y paso_celda los usa para dibujar), así que el formato
        # "clave=valor" de estas líneas es un contrato: no cambiarlo
        # `modo` no es un número, así que el extractor de parámetros del
        # análisis (que solo recoge pares clave=valor numéricos) lo ignora sin
        # más: la línea sigue casando con el patrón de [INIT] de siempre
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
        """Última distancia perpendicular (px) de la pegatina trasera a la
        trayectoria. Se publica en CarControlTelemetry."""
        return self._dist_derrape

    @property
    def estado_derrapando(self):
        """True si ahora mismo hay un derrape en curso (sin cerrar)."""
        return self._estado_derrapando

    @property
    def frame_valido(self):
        """True si el último fotograma procesado localizó el coche sobre la
        trayectoria (no fue descartado por ruido)."""
        return self._frame_valido

    def consumir_reduccion_pendiente(self):
        """Devuelve los px de reducción pendientes y los pone a cero. El
        controlador la llama tras cada actualizar_estado: si devuelve > 0
        debe llamar a aplicar_reduccion_externa de la cámara precedente."""
        pendiente = self._reduccion_pendiente
        self._reduccion_pendiente = 0.0
        return pendiente

    # ======================================================================
    # CONSTRUCCIÓN DE LA CADENA DE CELDAS
    # ======================================================================
    def setTrayectoria(self, trayectoria, varias_camaras=False):
        """
        Procesa la trayectoria de calibración de la cámara. Se llama UNA vez.

        Dos pasos, y a partir de ahí solo se trabaja con las celdas:
          1. ORIENTAR la lista de puntos: decidir si la cadena es cerrada o
             abierta y, si procede, rotarla para que empiece por el principio
             real de la porción visible.
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

        # --- Paso 1: orientación (cierre / rotación) ---
        filtrados, cerrada = self._filtrar_trayectoria(puntos, varias_camaras)
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

        # La plantilla se vuelca SIEMPRE, en cualquier modo: es la forma de
        # tener a mano la partición recién construida para escribir una política
        # a partir de ella
        self._guardar_plantilla_politica()
        if self.modo == "politica":
            self._cargar_politica()

    def _filtrar_trayectoria(self, puntos, varias_camaras):
        """
        Paso 1 de setTrayectoria: decide la ORIENTACIÓN de la cadena.
        Devuelve (puntos, cerrada), rotando la lista si hace falta.

        No hay nada que recortar: el controlador acumula los puntos de la
        calibración entre el PRIMER y el SEGUNDO paso por meta
        (`recolectar_datos_calibracion`), así que la lista que llega es
        exactamente una vuelta. Antes la calibración grababa desde que
        arrancaba el sistema y había que detectar y recortar la cola que
        repetía recorrido ya grabado; eso desapareció con la vuelta delimitada
        por meta.

        Lo que sí queda por resolver es que la vuelta empieza EN LA META, y la
        meta no tiene por qué caer en un extremo de la porción que ve esta
        cámara. Si el final queda cerca del inicio (< umbral_cierre), la
        grabación volvió al punto de partida y hay tres lecturas:
          - sin saltos grandes -> la cámara ve el circuito completo:
            cadena CERRADA;
          - con saltos grandes y VARIAS cámaras -> la meta cae en mitad de la
            porción visible: la lista llega como [mitad B, salto, mitad A],
            donde el salto es el resto del circuito que ven las otras cámaras.
            Se ROTA para empezar tras el salto y la cadena queda ABIERTA. Es
            el caso de la cámara que ve la meta, y con esta calibración ocurre
            siempre;
          - con saltos grandes y UNA cámara -> ve el circuito completo con
            huecos tapados: cadena CERRADA (los saltos serán celdas gigantes).
        """
        filtrados = puntos

        dif = np.diff(filtrados, axis=0)
        dist = np.hypot(dif[:, 0], dif[:, 1])
        saltos = [int(i) for i in np.nonzero(dist > self.umbral_celda_gigante)[0]]

        # Volcado de los huecos de la calibración. Un hueco es un tramo en el
        # que la cámara dejó de ver el coche: la cadena lo tapa interpolando
        # una recta (o, si pasa de umbral_celda_gigante, con una celda
        # gigante). Si el tramo tapado era en realidad una curva, la cadena
        # deja de describir la pista ahí y el coche pasa a varios píxeles de
        # una trayectoria que no existe, con derrapes falsos vuelta tras
        # vuelta. Desde la lista de puntos NO se puede saber si el hueco cae
        # sobre una recta (inofensivo) o sobre una curva, así que no se juzga:
        # se listan todos y se decide leyendo el log
        huecos = [
            (int(i), float(dist[i]))
            for i in np.nonzero(dist > 3.0 * self.paso_celda)[0]
        ]
        self.saveLogFile(
            f"[TRAY] Trayectoria de calibración: {len(filtrados)} puntos, "
            f"separación entre puntos mín={float(dist.min()):.1f} "
            f"mediana={float(np.median(dist)):.1f} máx={float(dist.max()):.1f} px"
        )
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

        cierre = math.hypot(
            float(filtrados[-1][0] - filtrados[0][0]),
            float(filtrados[-1][1] - filtrados[0][1]),
        )
        cerrada = False
        if cierre <= self.umbral_cierre:
            if saltos and varias_camaras:
                # La meta cae en mitad de la porción visible, así que la
                # vuelta grabada llega como [mitad B, salto, mitad A]. Se rota
                # para empezar justo tras el salto (el salto es el resto del
                # circuito, que ven otras cámaras, y desaparece de esta cadena)
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
            self.saveLogFile(
                f"[TRAY] Cierre: el final NO conecta con el inicio "
                f"({cierre:.1f} px > {self.umbral_cierre:.0f}) -> cadena "
                f"ABIERTA (porción del circuito); saltos interiores "
                f"tapados: {len(saltos)}"
            )
        return filtrados, cerrada

    def _construir_celdas(self, p):
        """
        Paso 2 de setTrayectoria: remuestrea la polilínea colocando
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
    def _dist_a_segmento(self, p, A, B):
        """
        Distancia del punto p al segmento AB (proyección con t acotado a
        [0, 1]: si p "cae" más allá de un extremo, la distancia es al propio
        extremo). Es el paso fino de `localizar`: el punto de celda más
        cercano da la resolución "de celda"; proyectar sobre los segmentos
        adyacentes da la distancia perpendicular a la trayectoria, que es lo
        que da sentido a un umbral de derrape de unos pocos píxeles.
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
        devuelve (índice de celda, distancia perpendicular, fila), o None si
        aún no hay trayectoria cargada. `fila` es el índice con el que se
        indexa `_puntos`, que NO coincide con el de celda cuando la cadena
        tiene celdas gigantes (esas no tienen punto). Se devuelve porque
        `actualizar_estado` necesita la POSICIÓN de la celda localizada para
        comprobar que las dos pegatinas se han colocado de forma coherente.

        Paso GRUESO: celda cuyo punto está más cerca (argmin vectorizado).
        Paso FINO: se proyecta el punto sobre los DOS segmentos que unen esa
                   celda con su anterior y su siguiente, y se conserva la
                   menor de las dos distancias. Medir contra el punto de la
                   celda daría una distancia escalonada (depende de dónde
                   cayeran las celdas); contra los segmentos se obtiene la
                   distancia perpendicular a la trayectoria.

        Un segmento solo existe si la celda vecina es realmente contigua sobre
        la trayectoria: en los extremos de la cadena abierta y a ambos lados
        de una celda gigante no hay recorrido conocido sobre el que proyectar,
        y ahí se usa la distancia al punto de la celda sin más.
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

        # Un vecino solo sirve si su celda es la contigua sobre la cadena: si
        # no lo es, entre ambos hay una celda gigante (o el final de la cadena
        # abierta) y no hay trayectoria sobre la que proyectar
        tengo_prev = (
            f_prev >= 0
            and self._celda_de_punto[f_prev] == (idx - 1) % self.n_celdas
        )
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
          3. Localiza la pegatina TRASERA y, si la localización es coherente
             con la de la delantera, alimenta la máquina de estados del
             derrape con su distancia perpendicular.
          4. Si hay un derrape abierto y el coche llega al final de la
             cadena abierta, se cierra ahí (misma razón que en 1).
          5. Devuelve el PWM de la ZONA a la que pertenece la celda actual.

        COHERENCIA ENTRE LAS DOS PEGATINAS (paso 3): las celdas localizadas
        son la aproximación de dónde está cada pegatina, así que la distancia
        entre las DOS CELDAS no puede ser mayor que la distancia entre las DOS
        PEGATINAS, con una celda de holgura por el redondeo del remuestreo. Si
        lo es, el coche estaría en dos sitios a la vez: la trasera no está
        sobre esta cadena y su distancia no significa nada.
        Ocurre cuando el coche circula por un tramo de pista que no tiene
        celdas (el hueco de una celda gigante, o un tramo que la calibración
        no llegó a grabar). Ahí la trasera no tiene dónde localizarse y el
        argmin le da la celda que le pilla más cerca, que puede estar en
        cualquier otra parte del trazado: en los logs de pista se ha visto la
        delantera en la celda 0 y la trasera en la 49, a 42 px, y eso se
        registraba como un derrape que castigaba 50 de las 58 celdas.
        La comprobación NO sustituye a la zona muerta de `_en_borde`: esa
        sigue protegiendo los extremos y las celdas gigantes. Es una condición
        distinta, se aplica en todos los fotogramas, y a propósito no descarta
        los derrapes que empiezan al entrar en la cadena, que son habituales
        cuando dos cámaras reparten una curva.

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

        idx_f, dist_f, fila_f = loc_front

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
            idx_b, dist_b, fila_b = loc_back
            self._dist_derrape = dist_b
            # Las dos celdas no pueden estar más separadas que las dos
            # pegatinas (ver COHERENCIA en el docstring)
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
                self._actualizar_derrape(idx_b, dist_b, vuelta, frame_count)
            elif dist_b > self.umbral_derrape and not self._estado_derrapando:
                # Solo se avisa cuando la medida descartada habría abierto un
                # derrape: si no, serían miles de líneas sin interés
                self.saveLogFile(
                    f"[DERRAPE] Frame {frame_count} v={vuelta}: dist={dist_b:.1f} "
                    f"> umbral pero IGNORADO por localización incoherente: la "
                    f"trasera cae en la celda {idx_b} y la delantera en la "
                    f"{idx_f}, que están a {sep_celdas:.1f} px, pero las "
                    f"pegatinas están a {sep_pegatinas:.1f} px"
                )
            # El formato de esta parte de la línea [FRAME] es un contrato con
            # RE_FRAME del análisis: no se le añade nada. Que un fotograma se
            # haya descartado por incoherencia se ve en la línea [DERRAPE]
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

    # ----------------------------------------------------------------------
    # MODO POLÍTICA: perfil fijo cargado de un JSON
    # ----------------------------------------------------------------------
    def _guardar_plantilla_politica(self):
        """Vuelca la partición actual como PLANTILLA de política.

        Se llama al final de setTrayectoria, en cualquier modo. El fichero
        lleva la misma estructura que usa el algoritmo (la lista `zonas` tal
        cual), para no inventar un formato ni tener que convertir nada al
        leerlo: se copia a `politica_<nodo>_<camara>.json`, se editan los `pwm`
        (y los cortes `ini`/`fin` si se quiere más detalle) y ya está listo.

        Es un fichero de trabajo, no un dato de la carrera: si no se puede
        escribir se avisa y se sigue, como con el log.
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
        """Sustituye la partición inicial por la del JSON de la política.

        Se llama solo en modo política, justo después de construir la cadena.
        A partir de ahí el perfil ya no se mueve: `_registrar_zona` y
        `registrar_vuelta` salen antes de tocarlo.

        La política se aplica POR ÍNDICE DE CELDA. La trayectoria no es la
        misma en dos calibraciones (lo dice la memoria, §Modos de operación), y
        emparejarlas exigiría traslación y rotación, que no se hace: si la
        cadena de ahora no tiene las mismas celdas que la de cuando se escribió
        la política, se ajusta lo que se pueda y se avisa de cada arreglo.
        Todos los avisos van en líneas [INIT], que el análisis ya ignora.

        Ante cualquier problema (fichero que no está, JSON roto, sin zonas
        utilizables) se avisa y se deja el perfil plano a v_min: es el valor de
        arranque de siempre, así que el coche rueda despacio en vez de quedarse
        sin perfil o tumbar el nodo.
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

        # Se recorta cada zona a la cadena de verdad y se descartan las que se
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
                # El suelo y el techo mandan sobre lo que ponga el fichero: son
                # los limites del carril, no una preferencia del algoritmo
                "pwm": max(self.v_min, min(self.v_max, pwm)),
                # El tipo se recalcula: si la celda es gigante tiene que seguir
                # marcada como tal, porque _en_borde y _castigar_gigante miran
                # el tipo de la zona, no lo que diga el JSON
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

        # Las zonas tienen que PARTICIONAR la cadena: _zona_de recorre la lista
        # y devuelve None si una celda no cae en ninguna, y ahi _velocidad_en
        # daria v_min sin decir por que. Se rellenan los huecos y se recortan
        # los solapes, avisando de cada arreglo
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
        # Formato ya conocido por el análisis (el motivo es texto libre)
        self._log_perfil("politica cargada, perfil fijo")

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

        # Modo política: el perfil es fijo, así que el derrape se CUENTA y queda
        # en el log (la línea [DERRAPE] ... CERRADO ya está escrita), pero no
        # castiga a nadie. Salir aquí es lo que impide que un derrape carvee la
        # política cargada, y de paso deja sin efecto el retroceso y el derrame,
        # que solo tienen sentido si el perfil se puede mover.
        #
        # No se escribe ninguna línea nueva: la de decisión ([ZONA] ... derrape
        # en [a, b] -> ...) simplemente no aparece, que es la verdad de lo que
        # pasó. Una línea que falta no rompe el análisis; una con formato nuevo
        # sí obligaría a tocar los dos parsers.
        if self.modo == "politica":
            return

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

        La mejora se decide ZONA A ZONA, nunca sobre el circuito completo:
        cada zona sube +incremento_vuelta (tope v_max) salvo que esté
        PROTEGIDA, es decir, que su último derrape o castigo fuese hace
        `vueltas_proteccion` vueltas o menos. Un derrape en una curva no
        impide que las rectas sigan ganando velocidad, que es justo lo que
        hace que el aprendizaje sea por tramos y no del trazado entero.

        Que la protección por zona baste, sin ninguna condición global, se
        apoya en tres cosas que ya garantizan las otras piezas:
          - La zona que acaba de derrapar NO puede subir: _registrar_zona
            mete la vuelta en curso en su historial, así que en esta misma
            llamada `vuelta - ultima` vale 1 y queda protegida.
          - Si el exceso de velocidad se gestó ANTES de la curva, el castigo
            ya se extendió hacia atrás `retroceso_creacion` px, de modo que
            la zona de entrada también bajó y también está protegida.
          - Lo que una cámara necesita saber de otra viaja por el derrame
            (aplicar_reduccion_externa), que crea la zona en la cámara
            precedente CON su historial de vueltas, o sea protegida igual.

        `vuelta_limpia` la calcula CarControllerNode con visión global (True
        si NINGUNA cámara registró derrapes) y `hubo_derrape_local` compara el
        contador propio con el de la vuelta anterior. Ninguna de las dos
        decide ya nada: se registran en el log porque dan el contexto de la
        vuelta al analizarla después. Antes actuaban como una puerta que
        bloqueaba la subida de TODAS las zonas si había un derrape en
        cualquier punto del circuito — herencia del perfil global del TFG de
        Adrián, que dejaba sin efecto el aprendizaje por zonas.
        """
        hubo_derrape_local = self.derrapes_contador != self._derrapes_vuelta_anterior
        self.saveLogFile(
            f"[VUELTA {vuelta}] limpia_global={vuelta_limpia} | derrapes de esta "
            f"cámara: {self._derrapes_vuelta_anterior}->{self.derrapes_contador} "
            f"(hubo_derrape_local={hubo_derrape_local})"
        )
        self._derrapes_vuelta_anterior = self.derrapes_contador

        # Modo política: el perfil es el que se cargó del JSON y no se toca en
        # toda la carrera, así que aquí no sube nada. Se reutiliza a propósito
        # el formato de la línea de "no sube" que ya existe (su motivo es texto
        # libre), para no obligar a tocar los regex del análisis.
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
