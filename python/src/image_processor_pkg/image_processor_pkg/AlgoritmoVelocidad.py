import math
import csv
import os
import numpy as np
from datetime import datetime


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

    ============================================================
    CONCEPTOS CLAVE PARA SEGUIR EL CÓDIGO
    ============================================================

    TRAYECTORIA: lista ordenada de puntos (x, y) en píxeles de imagen que el
    coche recorrió durante la vuelta de calibración. Es lo que "ve" ESTA
    cámara, así que puede ser una porción abierta del circuito, varias
    porciones separadas, o el circuito completo si una sola cámara lo cubre.

    TRAMO: cada porción continua de la trayectoria. Si la cámara ve dos zonas
    de la pista (el coche sale del encuadre y vuelve a entrar), la trayectoria
    tiene un "salto" grande entre dos puntos consecutivos, y ahí se corta en
    dos tramos. Cada tramo se numera 0, 1, 2...

    LONGITUD DE ARCO (s): distancia en píxeles recorrida SOBRE la trayectoria
    desde el inicio del tramo hasta un punto dado. Es la coordenada con la que
    trabaja todo el algoritmo: en lugar de decir "el coche está en el píxel
    (412, 87)" decimos "el coche está en s=350 del tramo 0". La ventaja es que
    s es unidimensional y monótona: comparar posiciones, medir zonas y definir
    ventanas ("300 px antes del derrape") se vuelve trivial.

    CELDA: el tramo se divide en trozos de `paso_perfil` px (30 px). Cada
    celda guarda un valor de PWM. El conjunto de celdas es el PERFIL: la
    "memoria" de qué velocidad es segura en cada punto del circuito. La celda
    de la posición s es simplemente int(s / paso_perfil).

    ZONA DE DERRAPE: intervalo [s_ini, s_fin] de un tramo donde se ha
    detectado derrape alguna vez. Se guarda con la lista de vueltas en las que
    derrapó y una distancia de "anticipación" que crece si reincide. Las zonas
    sirven para proteger sus celdas (no dejarlas subir justo después de un
    derrape) y para el registro/telemetría.

    CICLO DE APRENDIZAJE del perfil:
      - Todas las celdas empiezan en v_min (el coche arranca lento).
      - Vuelta limpia (sin derrapes en ninguna cámara): las celdas suben
        +incremento_vuelta, salvo las de zonas con derrapes recientes.
      - Derrape: bajan -reduccion_derrape las celdas desde `ventana_reduccion`
        px antes del inicio del derrape hasta su fin, que es el tramo donde se
        gestó la pérdida de adherencia.
      - Las zonas castigadas vuelven a subir tras `vueltas_proteccion` vueltas
        sin derrapar en ellas, de forma que el perfil oscila justo por debajo
        del límite de adherencia de cada punto del circuito.

    FLUJO DE LLAMADAS desde CarControllerNode:
      1. setTrayectoria(puntos)          -> una vez, al recibir la calibración
      2. actualizar_estado(front, back,  -> en cada fotograma con detección;
                           frame, vuelta)   devuelve el PWM a aplicar (o None)
      3. registrar_vuelta(v, limpia)     -> al cruzar la línea de meta; sin
                                            este aviso el perfil NUNCA sube

    LOG DE EJECUCIÓN (derrapesLog_<nodo>_<camara>.txt): cada línea lleva
    marca de tiempo y un prefijo que permite filtrar con grep:
      [INIT]    parámetros con los que corre la instancia
      [TRAY]    trayectoria base completa (nodo a nodo: x, y, tramo, s),
                cortes en tramos y detección de cierre
      [FRAME]   una línea por llamada a actualizar_estado: localización de
                ambas pegatinas, estado del derrape, celda leída y PWM devuelto
      [DERRAPE] transiciones de la máquina de estados (abierto/ignorado/cerrado)
      [ZONA]    altas, fusiones y estado completo de las zonas de derrape
      [PERFIL]  volcado completo del perfil tras cada modificación
      [VUELTA]  decisión al cruzar meta: sube o no, y qué celdas se protegen
    Leyendo el log de arriba abajo se reconstruye la ejecución completa.
    """

    def __init__(self, v_max, v_min, node_name="node", camara_id="cam"):
        """
        Crea una instancia del algoritmo para UNA cámara concreta.

        Solo guarda parámetros y deja el estado vacío: la trayectoria llega
        después por `setTrayectoria` (cuando CarControllerNode recibe la
        calibración de esa cámara). También abre el fichero de log en disco.

        Parámetros:
          v_max, v_min : límites de PWM entre los que se moverá el perfil.
          node_name    : nombre del nodo ROS (para diferenciar los logs).
          camara_id    : id de la cámara (ídem).
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

        # ------------------------------------------------------------------
        # Parámetros del perfil de PWM
        # ------------------------------------------------------------------
        # Tamaño de celda del perfil (px de trayectoria)
        self.paso_perfil = 30.0
        # Subida por vuelta limpia y bajada por derrape (unidades de PWM)
        self.incremento_vuelta = 1.0
        self.reduccion_derrape = 2.0
        # Cuánto antes del inicio del derrape se reduce el perfil (px). El
        # coche derrapa DESPUÉS de venir demasiado rápido: hay que frenar antes
        self.ventana_reduccion = 300.0
        # Vueltas sin derrapar en una zona antes de dejarla subir de nuevo
        self.vueltas_proteccion = 2

        # ------------------------------------------------------------------
        # Estado de la trayectoria (lo rellena setTrayectoria)
        # ------------------------------------------------------------------
        # Matriz (N, 2) con los N puntos (x, y) de la trayectoria de calibración
        self.trayectoriaUsada = None
        # Vector (N,): a qué tramo pertenece cada punto (0, 0, 0, 1, 1, ...)
        self._tramo_de_nodo = None
        # Vector (N,): longitud de arco s de cada punto DENTRO de su tramo
        self._s_nodos = None
        # {id de tramo: longitud total del tramo en px de arco}
        self._long_tramos = {}
        # True si la trayectoria es el circuito completo (el final conecta con
        # el inicio); cambia cómo se tratan bordes, márgenes y wraps de índice
        self._cerrada = False

        # ------------------------------------------------------------------
        # Perfil de PWM: {tramo: np.ndarray con el PWM de cada celda}
        # ------------------------------------------------------------------
        self.perfil = {}
        # Valor de derrapes_contador la última vez que se llamó a
        # registrar_vuelta: permite saber si ESTA cámara vio derrapes en la
        # vuelta que acaba de terminar (comparando con el contador actual)
        self._derrapes_vuelta_anterior = 0

        # ------------------------------------------------------------------
        # Zonas de derrape: {tramo: [{s_ini, s_fin, anticipacion, vueltas}]}
        # ------------------------------------------------------------------
        self.zonas = {}
        # Nº total de eventos de derrape detectados (lo usa el controlador para
        # saber si en la última vuelta hubo derrapes)
        self.derrapes_contador = 0

        # ------------------------------------------------------------------
        # Estado del derrape EN CURSO (máquina de estados de _actualizar_derrape)
        # ------------------------------------------------------------------
        # True mientras la pegatina trasera lleva varios frames fuera de umbral
        self._estado_derrapando = False
        # Tramo en el que empezó el derrape en curso
        self._derrape_tramo = None
        # s donde empezó y s del último frame del derrape en curso
        self._derrape_s_ini = 0.0
        self._derrape_s_fin = 0.0
        # Última distancia perpendicular medida de la pegatina trasera (para
        # publicarla en la telemetría aunque no haya derrape)
        self._dist_derrape = 0.0

        # Distancia de trayectoria que le queda al coche por delante dentro del
        # campo de visión de esta cámara (por si se arbitra entre cámaras)
        self._margen_restante = 0.0
        # True si en el último fotograma el coche se localizó sobre la trayectoria
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
        # autocontenido: al analizar una sesión a posteriori se sabe con qué
        # constantes exactas corrió el algoritmo (se ajustan a mano en el código)
        self.saveLogFile(f"[INIT] nodo={node_name} camara={camara_id}")
        self.saveLogFile(
            f"[INIT] v_min={self.v_min:.0f} v_max={self.v_max:.0f} | "
            f"umbral_derrape={self.umbral_derrape} max_dist_ruta={self.max_dist_ruta} "
            f"margen_extremo={self.margen_extremo}"
        )
        self.saveLogFile(
            f"[INIT] paso_perfil={self.paso_perfil} incremento_vuelta={self.incremento_vuelta} "
            f"reduccion_derrape={self.reduccion_derrape} ventana_reduccion={self.ventana_reduccion} "
            f"vueltas_proteccion={self.vueltas_proteccion}"
        )
        self.saveLogFile(
            f"[INIT] anticipacion_inicial={self.anticipacion_inicial} "
            f"incremento_anticipacion={self.incremento_anticipacion} "
            f"margen_fusion={self.margen_fusion}"
        )

    # ----------------------------------------------------------------------
    # Propiedades de solo lectura: exponen estado interno a CarControllerNode
    # (telemetría y arbitraje) sin dejar que lo modifique desde fuera.
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
        """Px de trayectoria que le quedan al coche dentro del campo de visión
        de esta cámara (infinito si la trayectoria es cerrada)."""
        return self._margen_restante

    @property
    def frame_valido(self):
        """True si el último fotograma procesado localizó el coche sobre la
        trayectoria (no fue descartado por ruido)."""
        return self._frame_valido

    def set_velocidades(self, v_max, v_min):
        """Actualiza los límites de PWM en caliente (p. ej. si el usuario los
        cambia por parámetro ROS). No toca el perfil ya aprendido: las celdas
        conservan su valor, solo cambia el tope aplicado en _velocidad_en."""
        self.saveLogFile(
            f"[INIT] Límites PWM cambiados en caliente: "
            f"v_min {self.v_min:.0f}->{float(v_min):.0f}, "
            f"v_max {self.v_max:.0f}->{float(v_max):.0f} (el perfil aprendido no se toca)"
        )
        self.v_max = float(v_max)
        self.v_min = float(v_min)

    # --- TRAYECTORIA ---
    def setTrayectoria(self, trayectoria):
        """
        Procesa la trayectoria de calibración de la cámara. Se llama UNA vez.

        Hace cuatro cosas, en este orden:
          1. Parte la lista de puntos en TRAMOS: si entre dos puntos
             consecutivos hay un salto mucho mayor que la separación típica,
             es que el coche salió del encuadre y volvió a entrar por otro
             sitio, así que ahí se corta.
          2. Calcula la longitud de arco s de cada punto dentro de su tramo
             (suma acumulada de las distancias entre puntos consecutivos).
          3. Detecta si la trayectoria es CERRADA (un único tramo cuyo último
             punto queda cerca del primero): significa que esta cámara ve el
             circuito completo.
          4. Inicializa el PERFIL: para cada tramo, un array de celdas de
             `paso_perfil` px todas a v_min.

        Estructuras que deja rellenas: trayectoriaUsada, _tramo_de_nodo,
        _s_nodos, _long_tramos, _cerrada, perfil y zonas (vacías).
        """
        # Idempotente: si ya hay trayectoria cargada, se ignoran las siguientes
        # (la calibración solo debe procesarse una vez)
        if self.trayectoriaUsada is not None:
            self.saveLogFile(
                "[TRAY] setTrayectoria ignorado: ya hay una trayectoria cargada"
            )
            return

        puntos = np.asarray(trayectoria, dtype=np.float32)
        # Hace falta una matriz (N, 2) con al menos 2 puntos para poder medir
        if puntos.ndim != 2 or len(puntos) < 2:
            self.saveLogFile(
                f"[TRAY] Trayectoria RECHAZADA: hacen falta >=2 puntos (x, y) "
                f"(recibido con forma {puntos.shape})"
            )
            return
        self.trayectoriaUsada = puntos

        # dif[i] = puntos[i+1] - puntos[i]; dist[i] = distancia euclídea entre
        # el punto i y el i+1 (hay N-1 distancias para N puntos)
        dif = np.diff(puntos, axis=0)
        dist = np.hypot(dif[:, 0], dif[:, 1])

        # Un salto mucho mayor que la separación típica entre nodos indica que
        # el coche salió del campo de visión: ahí termina un tramo y empieza otro.
        # Se usa la MEDIANA (robusta frente a los propios saltos) multiplicada
        # por 4, con un mínimo absoluto de 60 px por si la mediana fuese diminuta
        umbral_salto = max(4.0 * float(np.median(dist)), 60.0)
        # Queda registrado el criterio de corte, para poder justificar a
        # posteriori por qué la trayectoria se partió (o no) en tramos
        self.saveLogFile(
            f"[TRAY] Separación entre nodos: mediana={float(np.median(dist)):.1f} px, "
            f"mín={float(dist.min()):.1f}, máx={float(dist.max()):.1f} -> "
            f"umbral_salto={umbral_salto:.1f} px (max(4*mediana, 60))"
        )

        # Recorrido secuencial asignando tramo y arco a cada punto:
        #  - salto grande  -> nuevo tramo, s se reinicia a 0
        #  - salto normal  -> mismo tramo, s acumula la distancia recorrida
        tramos = np.zeros(len(puntos), dtype=np.int32)
        s = np.zeros(len(puntos), dtype=np.float64)
        for i in range(1, len(puntos)):
            if dist[i - 1] > umbral_salto:
                tramos[i] = tramos[i - 1] + 1
                s[i] = 0.0
                # Cada corte queda registrado: entre qué nodos ocurrió y de
                # cuántos px fue el salto que lo provocó
                self.saveLogFile(
                    f"[TRAY] Salto de {dist[i - 1]:.1f} px entre los nodos "
                    f"{i - 1} y {i}: comienza el tramo {int(tramos[i])}"
                )
            else:
                tramos[i] = tramos[i - 1]
                s[i] = s[i - 1] + dist[i - 1]

        self._tramo_de_nodo = tramos
        self._s_nodos = s

        # ------------------------------------------------------------------
        # TRAYECTORIA BASE COMPLETA: una línea por nodo con su posición en
        # píxeles de imagen, el tramo asignado y su longitud de arco s. Este
        # volcado es el "mapa" con el que se interpretan todas las líneas
        # [FRAME] posteriores (solo ocurre una vez, al cargar la calibración)
        # ------------------------------------------------------------------
        self.saveLogFile("[TRAY] --- Trayectoria base: nodo: (x, y) | tramo | s ---")
        for i in range(len(puntos)):
            self.saveLogFile(
                f"[TRAY] nodo {i:3d}: ({puntos[i][0]:7.1f}, {puntos[i][1]:7.1f}) | "
                f"tramo {int(tramos[i])} | s={s[i]:.1f}"
            )
        # Longitud de cada tramo = s máximo alcanzado dentro de él; de paso se
        # crea la lista (vacía) de zonas de derrape del tramo
        for t in np.unique(tramos):
            seleccion = tramos == t
            self._long_tramos[int(t)] = float(s[seleccion].max())
            self.zonas[int(t)] = []
            self.saveLogFile(
                f"[TRAY] tramo {int(t)}: {int(np.count_nonzero(seleccion))} nodos, "
                f"longitud de arco {self._long_tramos[int(t)]:.1f} px"
            )

        # Si es un único tramo cuyo final conecta con el inicio, la cámara ve el
        # circuito completo y la trayectoria se trata como cerrada.
        # (tramos[-1] == 0 significa que nunca hubo salto: un solo tramo)
        if tramos[-1] == 0:
            cierre = math.hypot(
                float(puntos[-1][0] - puntos[0][0]),
                float(puntos[-1][1] - puntos[0][1]),
            )
            if cierre <= umbral_salto:
                self._cerrada = True
                # La longitud total incluye el "segmento fantasma" que une el
                # último punto con el primero
                self._long_tramos[0] += cierre
                self.saveLogFile(
                    f"[TRAY] Cierre: distancia último->primero = {cierre:.1f} px "
                    f"<= umbral {umbral_salto:.1f} -> trayectoria CERRADA (esta cámara "
                    f"ve el circuito completo; longitud total con el segmento "
                    f"fantasma: {self._long_tramos[0]:.1f} px)"
                )
            else:
                self.saveLogFile(
                    f"[TRAY] Cierre: distancia último->primero = {cierre:.1f} px "
                    f"> umbral {umbral_salto:.1f} -> trayectoria ABIERTA "
                    f"(un solo tramo que no conecta consigo mismo)"
                )
        else:
            self.saveLogFile(
                f"[TRAY] {int(tramos[-1]) + 1} tramos separados -> trayectoria "
                f"ABIERTA (esta cámara ve porciones sueltas del circuito)"
            )

        # El perfil arranca con todas las celdas a v_min. ceil() garantiza que
        # el final del tramo también cae dentro de alguna celda; max(1, ...)
        # cubre el caso degenerado de un tramo más corto que una celda
        for tramo, longitud in self._long_tramos.items():
            n_celdas = max(1, int(math.ceil(longitud / self.paso_perfil)))
            self.perfil[tramo] = np.full(n_celdas, self.v_min, dtype=float)

        self.saveLogFile(
            f"[TRAY] Trayectoria cargada: {len(puntos)} nodos, "
            f"{len(self._long_tramos)} tramo(s), cerrada={self._cerrada}"
        )
        self._log_perfil("inicial: todas las celdas a v_min")

    def _proyectar_segmento(self, p, a_idx, b_idx, s_base):
        """
        Proyecta el punto p sobre el segmento que une los nodos a_idx y b_idx
        de la trayectoria, y devuelve (distancia perpendicular, s del punto
        proyectado).

        Geometría: la proyección de p sobre la recta AB es A + t·AB, con
        t = dot(AP, AB) / |AB|². Se recorta t a [0, 1] para quedarnos DENTRO
        del segmento (si p "cae" más allá de un extremo, la proyección es el
        propio extremo). El s resultante es el s del nodo A (`s_base`) más la
        fracción t recorrida del segmento.

        Es el paso fino de `localizar`: el nodo más cercano da una posición
        con resolución "de nodo"; proyectar sobre los segmentos adyacentes da
        la posición y la distancia perpendicular exactas.
        """
        A = self.trayectoriaUsada[a_idx]
        B = self.trayectoriaUsada[b_idx]
        AB = B - A
        l2 = float(np.dot(AB, AB))
        # Segmento degenerado (dos nodos idénticos): la "proyección" es A
        if l2 == 0.0:
            return float(np.linalg.norm(p - A)), s_base
        # t ∈ [0, 1]: fracción del segmento donde cae la proyección
        t = max(0.0, min(1.0, float(np.dot(p - A, AB)) / l2))
        proyeccion = A + t * AB
        # distancia p-proyección = distancia perpendicular; s = s_A + t·|AB|
        return float(np.linalg.norm(p - proyeccion)), s_base + t * math.sqrt(l2)

    def localizar(self, punto):
        """
        Convierte un punto (x, y) de la imagen en coordenadas de trayectoria:
        devuelve (tramo, s, distancia perpendicular), o None si aún no hay
        trayectoria cargada.

        Estrategia en dos pasos:
          1. GRUESO: nodo de la trayectoria más cercano al punto (argmin de
             distancias al cuadrado, vectorizado con numpy). Eso fija el tramo
             y da una primera estimación de s y de distancia.
          2. FINO: se proyecta el punto sobre los segmentos que tocan ese nodo
             (el anterior y el siguiente, si pertenecen al mismo tramo) y se
             queda con la proyección más cercana. Así s es continuo en lugar
             de saltar de nodo en nodo, y la distancia es la perpendicular
             real (clave para que el umbral de derrape de 8 px tenga sentido).

        Si la trayectoria es cerrada, también se prueba el segmento "fantasma"
        último→primero y el s final se toma módulo la longitud, para que justo
        pasado el origen s vuelva a empezar en 0.
        """
        if self.trayectoriaUsada is None:
            return None

        p = np.asarray(punto, dtype=np.float32)
        # Distancia al cuadrado del punto a TODOS los nodos de golpe (evita la
        # raíz cuadrada, que no cambia cuál es el mínimo)
        d2 = np.sum((self.trayectoriaUsada - p) ** 2, axis=1)
        idx = int(np.argmin(d2))
        tramo = int(self._tramo_de_nodo[idx])
        n = len(self.trayectoriaUsada)

        # Estimación inicial: el propio nodo más cercano
        mejor_dist = math.sqrt(float(d2[idx]))
        mejor_s = float(self._s_nodos[idx])

        # Refinamos proyectando sobre los segmentos adyacentes del mismo tramo.
        # Cada candidato es (nodo A, nodo B, s del nodo A)
        segmentos = []
        # Se comprueba si el nodo anterior pertenece al tramo en el que estamos
        # Como dentro del array puede haber nodos que no sean consecutivos hace falta
        if idx > 0 and self._tramo_de_nodo[idx - 1] == tramo:
            segmentos.append((idx - 1, idx, float(self._s_nodos[idx - 1])))
        # Se comprueba que el nodo de delante pertenezca al tramo en el que estamos
        if idx < n - 1 and self._tramo_de_nodo[idx + 1] == tramo:
            segmentos.append((idx, idx + 1, float(self._s_nodos[idx])))
        # Se usa para comprobar cuando el circuito esta cerrado
        if self._cerrada and (idx == 0 or idx == n - 1):
            segmentos.append((n - 1, 0, float(self._s_nodos[n - 1])))

        # Se comprueba cual es el nodo más cercano al punto en el que estamos
        for a, b, s_base in segmentos:
            distancia, s = self._proyectar_segmento(p, a, b, s_base)
            if distancia < mejor_dist:
                mejor_dist = distancia
                mejor_s = s

        # En cerrada, el segmento fantasma puede dar s > longitud: el módulo
        # lo devuelve al rango [0, long_tramo)
        long_tramo = self._long_tramos[tramo]
        if self._cerrada and long_tramo > 0:
            mejor_s = mejor_s % long_tramo

        return tramo, mejor_s, mejor_dist

    # --- LÓGICA PRINCIPAL ---
    def actualizar_estado(self, p_front, p_back, frame_count, vuelta):
        """
        Punto de entrada por fotograma. CarControllerNode la llama con las
        posiciones de las dos pegatinas y devuelve el PWM a aplicar, o None
        si el fotograma se descarta (el controlador mantiene entonces la
        velocidad anterior).

        Pasos:
          1. Localiza la pegatina FRONTAL (es la que define la posición del
             coche). Si está a más de `max_dist_ruta` px de la trayectoria,
             es una detección falsa (ruido, reflejo, otro objeto) y el frame
             se descarta entero.
          2. Actualiza `_margen_restante`: cuánta trayectoria le queda al
             coche por delante en esta cámara (infinito si es cerrada).
          3. Localiza la pegatina TRASERA y alimenta con su distancia
             perpendicular la máquina de estados del derrape
             (`_actualizar_derrape`). La trasera es la que "se abre" en un
             derrape porque el coche pivota sobre la guía delantera.
          4. Devuelve el PWM que el perfil tiene aprendido para la celda en la
             que está el coche (`_velocidad_en`).

        Nótese que la decisión de velocidad SOLO depende de la posición actual
        (paso 4): no importa si se perdieron fotogramas o llegaron fuera de
        orden, lo que hace al algoritmo robusto en la arquitectura distribuida.
        """
        # Por defecto el frame es inválido; solo se marca válido al final,
        # cuando ha pasado todos los filtros
        self._frame_valido = False

        loc_front = self.localizar(p_front)
        if loc_front is None:
            # Aún no hay trayectoria cargada: no se puede decidir nada
            self.saveLogFile(
                f"[FRAME {frame_count} v={vuelta}] DESCARTADO: aún no hay "
                f"trayectoria cargada"
            )
            return None

        tramo_f, s_f, dist_f = loc_front

        # La etiqueta frontal marca la posición del coche y siempre va sobre el
        # carril: si está lejos de la ruta es una detección falsa
        if dist_f > self.max_dist_ruta:
            self._margen_restante = 0.0
            self.saveLogFile(
                f"[FRAME {frame_count} v={vuelta}] DESCARTADO por ruido: "
                f"front=({p_front[0]:.1f}, {p_front[1]:.1f}) está a {dist_f:.1f} px "
                f"de la ruta (> max_dist_ruta={self.max_dist_ruta:.0f}); el "
                f"controlador mantiene la velocidad anterior"
            )
            return None

        # Margen restante = lo que queda de tramo por delante del coche
        long_tramo = self._long_tramos[tramo_f]
        if self._cerrada:
            self._margen_restante = float("inf")
        else:
            self._margen_restante = long_tramo - s_f

        # Detección del derrape con la etiqueta trasera. Puede fallar su
        # localización de forma independiente a la frontal; en ese caso
        # simplemente no se actualiza la máquina de estados este frame
        loc_back = self.localizar(p_back)
        if loc_back is not None:
            tramo_b, s_b, dist_b = loc_back
            self._dist_derrape = dist_b
            self._actualizar_derrape(tramo_b, s_b, dist_b, vuelta, frame_count)
            texto_back = (
                f"back=({p_back[0]:.1f}, {p_back[1]:.1f})->t={tramo_b} "
                f"s={s_b:.1f} d={dist_b:.1f} (umbral {self.umbral_derrape:.0f})"
            )
        else:
            texto_back = "back=NO localizado (máquina de derrape sin actualizar)"

        self._frame_valido = True
        # La velocidad es función pura de la posición: se lee del perfil
        velocidad = self._velocidad_en(tramo_f, s_f)

        # Traza principal del algoritmo: una línea por fotograma con TODA la
        # decisión tomada — dónde se localizó cada pegatina (tramo, s y
        # distancia perpendicular), el estado del derrape, la celda del perfil
        # leída y el PWM devuelto. Con el volcado [TRAY] como mapa, estas
        # líneas permiten reconstruir la carrera completa
        celdas_tramo = self.perfil.get(tramo_f)
        if celdas_tramo is not None and len(celdas_tramo) > 0:
            # Mismo índice (con recorte) que usa _velocidad_en para leer el PWM
            celda = min(int(s_f / self.paso_perfil), len(celdas_tramo) - 1)
        else:
            celda = -1
        texto_margen = "inf" if self._cerrada else f"{self._margen_restante:.1f}"
        self.saveLogFile(
            f"[FRAME {frame_count} v={vuelta}] "
            f"front=({p_front[0]:.1f}, {p_front[1]:.1f})->t={tramo_f} "
            f"s={s_f:.1f} d={dist_f:.1f} | {texto_back} | "
            f"derrapando={self._estado_derrapando} | celda={celda} "
            f"pwm={velocidad:.0f} | margen={texto_margen}"
        )
        return velocidad

    def _actualizar_derrape(self, tramo, s, dist, vuelta, frame):
        """
        Máquina de estados del derrape. Se llama en cada frame con la posición
        (tramo, s) y la distancia perpendicular `dist` de la pegatina TRASERA.

        Estados y transiciones:

          NO derrapando ──(dist > umbral y no está en un borde)──> derrapando
              Se anota dónde empieza (_derrape_s_ini = _derrape_s_fin = s).
              Excepción "en_borde": en tramos ABIERTOS, si el derrape
              empezaría a menos de `margen_extremo` px de un extremo, se
              ignora: al entrar/salir del encuadre la pose del coche se
              detecta mal y daría falsos positivos.

          derrapando ──(dist > umbral, mismo tramo)──> sigue derrapando
              Solo se actualiza el final (_derrape_s_fin = s), es decir, el
              derrape se va "alargando" frame a frame.
              Caso especial en trayectoria CERRADA: si s pega un salto de más
              de media vuelta respecto al frame anterior, es que el derrape
              cruzó el origen (s pasó de ~long a ~0). Como una zona es un
              intervalo [s_ini, s_fin] que no puede envolver el origen, se
              cierra la zona actual y se abre otra al otro lado.

          derrapando ──(dist <= umbral)──> NO derrapando
              El coche se realineó: se cierra el derrape y se registra la
              zona completa (_cerrar_derrape -> _registrar_zona).

        Fíjate en que un derrape NO se registra frame a frame sino como un
        único evento con su intervalo [s_ini, s_fin] al terminar: así el
        castigo del perfil y la fusión de zonas trabajan con el derrape entero.
        """
        if dist > self.umbral_derrape:
            if not self._estado_derrapando:
                # No abrimos derrapes pegados al borde del campo de visión: al
                # entrar/salir de la imagen la detección no es fiable
                en_borde = (not self._cerrada) and (
                    s < self.margen_extremo
                    or s > self._long_tramos[tramo] - self.margen_extremo
                )
                if en_borde:
                    self.saveLogFile(
                        f"[DERRAPE] Frame {frame} v={vuelta}: dist={dist:.1f} > umbral "
                        f"pero IGNORADO por borde: s={s:.1f} está a menos de "
                        f"{self.margen_extremo:.0f} px de un extremo del tramo {tramo} "
                        f"(longitud {self._long_tramos[tramo]:.1f}); al entrar/salir "
                        f"del encuadre la detección no es fiable"
                    )
                    return
                # Arranca un derrape nuevo en este punto
                self._estado_derrapando = True
                self._derrape_tramo = tramo
                self._derrape_s_ini = s
                self._derrape_s_fin = s
                self.saveLogFile(
                    f"[DERRAPE] Frame {frame} v={vuelta}: ABIERTO en tramo {tramo}, "
                    f"s={s:.1f} (dist={dist:.1f} > umbral {self.umbral_derrape:.0f})"
                )
            elif tramo == self._derrape_tramo:
                long_tramo = self._long_tramos[tramo]
                if self._cerrada and abs(s - self._derrape_s_fin) > long_tramo / 2.0:
                    # El derrape cruzó el origen de la trayectoria cerrada: se
                    # cierra la zona actual y se abre otra al otro lado
                    self.saveLogFile(
                        f"[DERRAPE] Frame {frame} v={vuelta}: salto de s "
                        f"{self._derrape_s_fin:.1f}->{s:.1f} (> media vuelta de "
                        f"{long_tramo:.1f} px): el derrape cruzó el origen; se "
                        f"cierra la zona actual y se abre otra al otro lado"
                    )
                    self._cerrar_derrape(vuelta, frame)
                    self._estado_derrapando = True
                    self._derrape_s_ini = s
                # El derrape continúa: se extiende su final hasta la posición actual
                self._derrape_s_fin = s
        else:
            # La trasera volvió a alinearse con la trayectoria: si había un
            # derrape abierto, se da por terminado y se registra
            if self._estado_derrapando:
                self._cerrar_derrape(vuelta, frame)

    def _cerrar_derrape(self, vuelta, frame):
        """
        Da por terminado el derrape en curso y lo registra como zona.

        El min/max ordena el intervalo por si el coche se movía "hacia atrás"
        en s (puede pasar si la trayectoria de calibración se recorrió en
        sentido contrario al de carrera): garantiza s_ini <= s_fin.
        """
        self._estado_derrapando = False
        s_ini = min(self._derrape_s_ini, self._derrape_s_fin)
        s_fin = max(self._derrape_s_ini, self._derrape_s_fin)
        self.saveLogFile(
            f"[DERRAPE] Frame {frame} v={vuelta}: CERRADO en tramo "
            f"{self._derrape_tramo}, intervalo [{s_ini:.1f}, {s_fin:.1f}] "
            f"({s_fin - s_ini:.1f} px de arco)"
        )
        self._registrar_zona(self._derrape_tramo, s_ini, s_fin, vuelta, frame)

    def _registrar_zona(self, tramo, s_ini, s_fin, vuelta, frame):
        """
        Registra un derrape terminado [s_ini, s_fin]. Hace tres cosas:

          1. CONTADOR: incrementa `derrapes_contador`, que es lo que el
             controlador consulta para decidir si la vuelta fue limpia.

          2. ZONAS: busca zonas existentes que se solapen con el nuevo derrape
             (con `margen_fusion` px de holgura para juntar derrapes casi
             contiguos):
               - Si toca varias, primero las fusiona todas en una (unión de
                 intervalos, máxima anticipación, unión de listas de vueltas).
               - Si toca alguna, la extiende con el nuevo intervalo, aumenta
                 su anticipación (+incremento_anticipacion: reincidir en una
                 zona significa que hay que empezar a frenar antes) y apunta
                 la vuelta actual en su historial.
               - Si no toca ninguna, crea una zona nueva con la anticipación
                 inicial.
             El historial `vueltas` de cada zona es lo que luego usa
             `registrar_vuelta` para mantener la protección de la zona.

          3. CASTIGO DEL PERFIL: baja -reduccion_derrape (con suelo v_min) las
             celdas del intervalo [s_ini - ventana_reduccion, s_fin]. La
             ventana se extiende hacia ATRÁS del derrape porque la pérdida de
             adherencia se gesta antes de manifestarse: el exceso de velocidad
             ocurrió en los ~300 px previos, y es ahí donde hay que ir más
             despacio la próxima vuelta.
        """
        self.derrapes_contador += 1
        zonas = self.zonas.setdefault(tramo, [])

        # Zonas cuyo intervalo (ampliado con margen_fusion) toca al nuevo:
        # dos intervalos [a,b] y [c,d] se solapan si a <= d y b >= c
        solapadas = [
            z
            for z in zonas
            if s_ini - self.margen_fusion <= z["s_fin"]
            and s_fin + self.margen_fusion >= z["s_ini"]
        ]

        if solapadas:
            zona = solapadas[0]
            # Si el derrape toca varias zonas, se unen en una sola: queda
            # registrado qué intervalos se fusionan y en qué resultado
            if len(solapadas) > 1:
                self.saveLogFile(
                    f"[ZONA] Frame {frame} vuelta {vuelta}: el derrape "
                    f"[{s_ini:.1f}, {s_fin:.1f}] toca {len(solapadas)} zonas del "
                    f"tramo {tramo}, se fusionan: "
                    + ", ".join(
                        f"[{z['s_ini']:.1f}, {z['s_fin']:.1f}]" for z in solapadas
                    )
                )
            for extra in solapadas[1:]:
                zonas.remove(extra)
                zona["s_ini"] = min(zona["s_ini"], extra["s_ini"])
                zona["s_fin"] = max(zona["s_fin"], extra["s_fin"])
                zona["anticipacion"] = max(zona["anticipacion"], extra["anticipacion"])
                zona["vueltas"].extend(extra["vueltas"])

            # Se amplía la zona para cubrir también el derrape nuevo
            zona["s_ini"] = min(zona["s_ini"], s_ini)
            zona["s_fin"] = max(zona["s_fin"], s_fin)
            # Derrape repetido en la zona: hay que empezar a frenar antes
            zona["anticipacion"] += self.incremento_anticipacion
            zona["vueltas"].append(vuelta)
            self.saveLogFile(
                f"[ZONA] Frame {frame} vuelta {vuelta}: derrape repetido en zona "
                f"[{zona['s_ini']:.0f}, {zona['s_fin']:.0f}] del tramo {tramo}. "
                f"Anticipación aumentada a {zona['anticipacion']:.0f} "
                f"(derrapes_contador={self.derrapes_contador})"
            )
        else:
            # Primera vez que se derrapa aquí: zona nueva
            zonas.append(
                {
                    "s_ini": s_ini,
                    "s_fin": s_fin,
                    "anticipacion": self.anticipacion_inicial,
                    "vueltas": [vuelta],
                }
            )
            # Mantener las zonas ordenadas por posición facilita leer los logs
            zonas.sort(key=lambda z: z["s_ini"])
            self.saveLogFile(
                f"[ZONA] Frame {frame} vuelta {vuelta}: nueva zona de derrape "
                f"[{s_ini:.0f}, {s_fin:.0f}] en tramo {tramo} con anticipación "
                f"{self.anticipacion_inicial:.0f} (derrapes_contador="
                f"{self.derrapes_contador})"
            )

        # Estado completo de las zonas tras el alta/fusión/ampliación
        self._log_zonas(f"estado tras el derrape de la vuelta {vuelta}")

        # Además castigamos el perfil en la zona previa al derrape, que es
        # donde se gestó la pérdida de adherencia
        indices = self._celdas_en(tramo, s_ini - self.ventana_reduccion, s_fin)
        if not indices:
            self.saveLogFile(
                f"[PERFIL] Castigo sin efecto: el intervalo "
                f"[{s_ini - self.ventana_reduccion:.1f}, {s_fin:.1f}] no cubre "
                f"ninguna celda del tramo {tramo}"
            )
            return
        celdas = self.perfil[tramo]
        # Se anota el valor previo de cada celda para poder loguear el cambio
        # exacto antes->después que produce este castigo
        antes = {i: celdas[i] for i in indices}
        for i in indices:
            # Bajada con suelo: nunca por debajo de v_min
            celdas[i] = max(self.v_min, celdas[i] - self.reduccion_derrape)
        self.saveLogFile(
            f"[PERFIL] Castigo -{self.reduccion_derrape:.0f} en tramo {tramo}: "
            f"intervalo [{s_ini - self.ventana_reduccion:.1f}, {s_fin:.1f}] "
            f"({self.ventana_reduccion:.0f} px antes del derrape), "
            f"{len(indices)} celdas: "
            + ", ".join(f"c{i}:{antes[i]:.0f}->{celdas[i]:.0f}" for i in indices)
        )
        self._log_perfil("tras castigo por derrape")

    # --- PERFIL DE PWM ---
    def _celdas_en(self, tramo, s_desde, s_hasta):
        """
        Traduce un intervalo de arco [s_desde, s_hasta] a la lista de índices
        de celda del perfil que lo cubren.

        La celda de una posición s es floor(s / paso_perfil). Se calculan los
        índices de la primera (i0) y la última (i1) celda del intervalo y se
        devuelven todos los índices intermedios.

        El intervalo puede salirse del tramo (p. ej. `s_desde` negativo cuando
        la ventana de castigo de 300 px empieza antes que el propio tramo):
          - Trayectoria CERRADA: el índice se toma módulo n (i % n), de forma
            que la ventana "da la vuelta" por el origen. Ejemplo: con 20
            celdas, el intervalo de celdas [-2, 1] devuelve [18, 19, 0, 1].
          - Trayectoria ABIERTA: los índices se recortan a [0, n-1]; lo que
            cae fuera del tramo simplemente no existe y se descarta.
        """
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
        """
        Velocidad en la posición s del tramo: el PWM aprendido de su celda.

        Es la lectura del perfil, y a propósito es trivial: toda la
        inteligencia está en cómo se actualizan las celdas, no en cómo se
        leen. El min(idx, n-1) protege el caso s == longitud exacta del tramo
        (caería en una celda inexistente); el min(..., v_max) aplica el tope
        por si v_max bajó en caliente con set_velocidades.

        Si el perfil aún no existe (trayectoria sin cargar), devuelve v_min
        por prudencia.
        """
        celdas = self.perfil.get(tramo)
        if celdas is None or len(celdas) == 0:
            return self.v_min
        idx = min(int(s / self.paso_perfil), len(celdas) - 1)
        return float(min(celdas[idx], self.v_max))

    def registrar_vuelta(self, vuelta, vuelta_limpia):
        """
        Aviso del controlador al completarse una vuelta. Es el mecanismo por
        el que el perfil SUBE (la bajada ocurre en _registrar_zona al derrapar).

        `vuelta_limpia` la calcula CarControllerNode con visión global: True
        si NINGUNA cámara registró derrapes en la vuelta. Además, cada
        instancia hace su propia comprobación local comparando su
        `derrapes_contador` actual con el de la vuelta anterior
        (`hubo_derrape_local`); es redundante con la comprobación global pero
        deja la clase protegida por sí misma si el controlador se equivocara.

        Si la vuelta fue limpia, TODAS las celdas de TODOS los tramos suben
        +incremento_vuelta (con tope v_max)... excepto las PROTEGIDAS: las
        celdas de zonas cuyo último derrape fue hace `vueltas_proteccion` (2)
        vueltas o menos. La protección cubre el mismo intervalo que se castigó
        ([s_ini - ventana_reduccion, s_fin]) y evita el ciclo absurdo de
        castigar una zona y devolverle el PWM a la vuelta siguiente: la zona
        debe demostrar 2 vueltas seguidas sin derrape antes de volver a subir.

        Resultado neto del ciclo completo: las celdas sin problemas suben
        hasta saturar en v_max, y las celdas conflictivas oscilan justo por
        debajo del límite de adherencia de su curva.
        """
        # ¿Vio ESTA cámara algún derrape desde el último aviso de vuelta?
        hubo_derrape_local = self.derrapes_contador != self._derrapes_vuelta_anterior
        # Queda registrada la decisión completa: la visión global del
        # controlador, el contador local antes/después y qué se decide
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

        # Vuelta limpia: sube todo el perfil salvo las celdas de zonas con
        # derrapes recientes (siguen protegidas unas vueltas)
        for tramo, celdas in self.perfil.items():
            # 1º se reúnen los índices protegidos de todas las zonas del tramo
            protegidas = set()
            for zona in self.zonas.get(tramo, []):
                # Vuelta del último derrape de la zona (-10^6 si no hay
                # ninguna apuntada: equivale a "hace muchísimo", sin protección)
                ultima = zona["vueltas"][-1] if zona["vueltas"] else -10**6
                if vuelta - ultima <= self.vueltas_proteccion:
                    celdas_zona = self._celdas_en(
                        tramo, zona["s_ini"] - self.ventana_reduccion, zona["s_fin"]
                    )
                    protegidas.update(celdas_zona)
                    self.saveLogFile(
                        f"[VUELTA {vuelta}] tramo {tramo}: la zona "
                        f"[{zona['s_ini']:.1f}, {zona['s_fin']:.1f}] protege las "
                        f"celdas {sorted(celdas_zona)} (último derrape en vuelta "
                        f"{ultima}, le quedan "
                        f"{self.vueltas_proteccion - (vuelta - ultima)} vuelta(s) "
                        f"de protección)"
                    )
            # 2º suben todas las celdas no protegidas, con tope v_max
            for i in range(len(celdas)):
                if i not in protegidas:
                    celdas[i] = min(self.v_max, celdas[i] + self.incremento_vuelta)

        self.saveLogFile(
            f"[VUELTA {vuelta}] limpia: perfil incrementado "
            f"+{self.incremento_vuelta:.0f} (tope v_max={self.v_max:.0f}) en las "
            f"celdas no protegidas"
        )
        self._log_perfil(f"tras la vuelta {vuelta} limpia")

    # --- VOLCADOS DE ESTADO (solo escriben en el log, no modifican nada) ---
    def _log_perfil(self, motivo):
        """
        Vuelca al log el perfil COMPLETO de PWM de todos los tramos.

        Cada celda i cubre el intervalo de arco [i·paso_perfil, (i+1)·paso_perfil):
        con paso_perfil=30, la celda 0 es s=[0,30), la 1 es s=[30,60), etc.
        Se llama después de CADA modificación del perfil (inicialización,
        castigo por derrape, subida por vuelta limpia), de forma que comparando
        dos volcados consecutivos se ve exactamente qué celdas cambiaron.
        """
        self.saveLogFile(f"[PERFIL] ({motivo})")
        for tramo, celdas in self.perfil.items():
            self.saveLogFile(
                f"[PERFIL] tramo {tramo} ({len(celdas)} celdas de "
                f"{self.paso_perfil:.0f} px): "
                + " ".join(f"{v:.0f}" for v in celdas)
            )

    def _log_zonas(self, motivo):
        """
        Vuelca al log todas las zonas de derrape acumuladas, por tramo: su
        intervalo [s_ini, s_fin], la anticipación actual y en qué vueltas
        derrapó. Se llama tras cada alta/ampliación/fusión de zona para ver
        cómo evoluciona la estructura `zonas` a lo largo de la carrera.
        """
        self.saveLogFile(f"[ZONA] ({motivo})")
        total = 0
        for tramo, zonas in self.zonas.items():
            for z in zonas:
                total += 1
                self.saveLogFile(
                    f"[ZONA] tramo {tramo}: [{z['s_ini']:.1f}, {z['s_fin']:.1f}] "
                    f"anticipación={z['anticipacion']:.0f} vueltas={z['vueltas']}"
                )
        if total == 0:
            self.saveLogFile("[ZONA] (sin zonas registradas)")

    # --- PERSISTENCIA ---
    def saveLogFile(self, data):
        """Añade una línea al log de texto de esta cámara, con marca de tiempo
        HH:MM:SS.mmm para poder correlacionarla con la telemetría y los bags.
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
        Vuelca a CSV el historial de zonas de derrape para analizarlo después.

        Formato: una columna por zona (cabecera "tramoT[s_ini-s_fin]") y una
        fila por vuelta; la celda lleva el número de vuelta si esa zona
        registró un derrape en esa vuelta, y queda vacía si no. Permite ver de
        un vistazo en qué vueltas reincidió cada curva.
        """
        try:
            with open(self.csv_file, "w", newline="") as archivoCSV:
                # Aplana {tramo: [zonas]} a una lista [(tramo, zona), ...]
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
