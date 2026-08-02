#!/usr/bin/env python3
"""
Analiza un log de EstrategiaPerfil (derrapesLog_<nodo>_<camara>.txt, el formato
de CADENA DE CELDAS + ZONAS con etiquetas [INIT]/[TRAY]/[FRAME]/[DERRAPE]/
[ZONA]/[PERFIL]/[VUELTA]) y genera un ÚNICO HTML interactivo (plotly) con la
carrera completa vista por vueltas. No necesita ROS ni Docker: es puro
procesado de texto.

Uso:

  ANALISIS/env/bin/python analisis/analizar_log_algoritmo.py <fichero_log>

El HTML se escribe al lado del log como <nombre_log>_analisis.html y se abre
con doble clic en el navegador (plotly.js va embebido: funciona sin conexión).

Qué contiene el HTML (en este orden):

  0. Resumen de la carrera + ANOMALÍAS detectadas (derrapes que abarcan
     demasiadas celdas, saltos de celda de la pegatina trasera dentro de un
     derrape abierto, zonas que cubren gran parte de la cadena...).
  1. Evolución de las zonas: eje X = celda, eje Y = vuelta. Por cada vuelta
     se dibuja la partición de zonas al cerrarla (barras por tipo), los
     derrapes individuales de esa vuelta (rojo), las aperturas ignoradas por
     zona muerta, las fusiones (estrella) y los derrames entre cámaras.
  2. Heatmap del perfil de PWM: eje X = celda, eje Y = vuelta, color = PWM de
     la zona de cada celda CON EL QUE SE CORRIÓ esa vuelta. Encima, las
     celdas castigadas durante la vuelta y las zonas protegidas al empezarla.
  3. Mapa del circuito (posiciones x,y de las celdas de [TRAY]) coloreado por
     el PWM de cada celda, con slider de vuelta; las celdas gigantes se
     dibujan como segmento discontinuo (ahí no se ve el circuito).
  4. Distancia perpendicular de la trasera frente a su celda (una vuelta cada
     vez, con slider) + resumen por vuelta (derrapes, tiempo, PWM, velocidad).

============================================================
CÓMO ESTÁ ORGANIZADO EL SCRIPT
============================================================

  1. Constantes ajustables (colores y umbrales de anomalías) bajo los imports.
  2. Expresiones regulares: una por tipo de línea del log, copiadas de los
     f-string de AlgoritmoVelocidad.py (si se cambia un mensaje allí, hay que
     retocar aquí la regex correspondiente).
  3. parsear_log(): una pasada por el fichero -> objeto Carrera.
  4. Derivados por vuelta: perfil vigente y partición de zonas por vuelta.
  5. detectar_anomalias(): las comprobaciones automáticas.
  6. Las funciones grafica_N_*: una por figura del HTML.
  7. main(): parsea, imprime el resumen por consola y ensambla el HTML.
"""

import html
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# ---------------------------------------------------------------------------
# Umbrales del detector de anomalías. Ahora todo se mide en CELDAS de la
# cadena (paso_celda px cada una, 30 por defecto); se ajustan aquí.
# ---------------------------------------------------------------------------
# Un derrape individual (evento [DERRAPE] CERRADO) que abarque más de esta
# fracción de la cadena se marca como anómalo: un derrape real dura unas
# pocas celdas, no medio circuito.
FRAC_CELDAS_DERRAPE_ANOMALO = 0.15

# Una zona de derrape que cubra más de esta fracción de la cadena bloquea el
# aprendizaje de casi todo el perfil (queda protegida tras cada reincidencia).
FRAC_ZONA_GIGANTE = 0.40

# Salto de celda de la pegatina TRASERA entre dos frames consecutivos con un
# derrape abierto. El derrape se extiende frame a frame con la celda de la
# trasera: una localización falsa lejos estira el intervalo de golpe.
SALTO_CELDAS_TRASERA_ANOMALO = 3

# ---------------------------------------------------------------------------
# Colores (paleta validada del skill dataviz, modo claro). Los roles de estado
# se reservan para "cosas malas" (derrapes/zonas) y los secuenciales para
# magnitud (PWM); el texto siempre va en tintas, nunca en color de serie.
# ---------------------------------------------------------------------------
COL_SUPERFICIE = "#fcfcfb"   # fondo de las gráficas y de la página
COL_TINTA = "#0b0b0b"        # texto principal
COL_TINTA_2 = "#52514e"      # texto secundario (explicaciones)
COL_MUTED = "#898781"        # ejes, etiquetas apagadas, marcadores neutros
COL_GRID = "#e1e0d9"         # rejilla fina
COL_SERIE_1 = "#2a78d6"      # azul, serie categórica 1 (dist, tiempos...)
COL_SERIE_2 = "#1baf7a"      # aqua, serie categórica 2 (segunda serie PWM)
COL_CRITICO = "#d03b3b"      # rojo estado "critical": eventos de derrape
COL_SERIO = "#ec835a"        # naranja estado "serious": zonas de derrape
COL_AVISO = "#b58324"        # ámbar: celdas gigantes y derrames entre cámaras
COL_BUENO = "#0ca30c"        # verde estado "good": vuelta con subida de perfil
COL_CONTEXTO = "#c3c2b7"     # gris de las series de fondo/contexto

# Rampa secuencial azul claro->oscuro para el PWM: claro = lento (v_min),
# oscuro = rápido (v_max), en estructura colorscale de plotly.
_RAMPA_AZUL = [
    "#cde2fb", "#b7d3f6", "#9ec5f4", "#86b6ef", "#6da7ec", "#5598e7",
    "#3987e5", "#2a78d6", "#256abf", "#1c5cab", "#184f95", "#104281", "#0d366b",
]
ESCALA_PWM = [[i / (len(_RAMPA_AZUL) - 1), c] for i, c in enumerate(_RAMPA_AZUL)]

# Tipografía de todo el HTML y las figuras (sans del sistema, sin serifas)
FUENTE = 'system-ui, -apple-system, "Segoe UI", sans-serif'


# ---------------------------------------------------------------------------
# Expresiones regulares: UNA por mensaje de AlgoritmoVelocidad.py. El prefijo
# común "HH:MM:SS.mmm " se separa antes (RE_LINEA) y aquí se casa solo el
# cuerpo. Mantenerlas sincronizadas con los saveLogFile() del algoritmo.
# ---------------------------------------------------------------------------
RE_LINEA = re.compile(r"^(\d{2}):(\d{2}):(\d{2})\.(\d{3}) (.*)$")
RE_CABECERA = re.compile(r"^=== LOG INICIADO ?(\S*) ===$")

RE_FRAME = re.compile(
    r"\[FRAME (\d+) v=(\d+)\] "
    r"front=\((-?[\d.]+), (-?[\d.]+)\)->c=(\d+) d=([\d.]+) \| "
    r"(?:back=\((-?[\d.]+), (-?[\d.]+)\)->c=(\d+) d=([\d.]+) "
    r"\(umbral [\d.]+\)|back=NO localizado \([^)]*\)) \| "
    r"derrapando=(True|False) \| "
    r"(?:zona=\[(\d+)-(\d+)\]\((\w+)\)|zona=\?) "
    r"pwm=(\d+) \| margen=([\d.]+|inf)"
)
RE_FRAME_DESCARTADO = re.compile(r"\[FRAME (\d+) v=(\d+)\] DESCARTADO(.*)$")

RE_DERRAPE_ABIERTO = re.compile(
    r"\[DERRAPE\] Frame (\d+) v=(\d+): ABIERTO en celda (\d+) \(dist=([\d.]+)"
)
RE_DERRAPE_CERRADO = re.compile(
    r"\[DERRAPE\] Frame (\d+) v=(\d+): CERRADO, celdas "
    r"\[(\d+), (\d+)\] \((\d+) celdas\)"
)
RE_DERRAPE_ZONA_MUERTA = re.compile(
    r"\[DERRAPE\] Frame (\d+) v=(\d+): dist=([\d.]+) > umbral pero IGNORADO "
    r"por zona muerta: la celda (\d+)"
)
# Segundo motivo de rechazo, más reciente: la trasera se localizó en una celda
# incompatible con la de la delantera (el coche estaría en dos sitios a la vez),
# lo que pasa cuando circula por un tramo que no tiene celdas. Es un regex
# aparte y no una ampliación del anterior a propósito: así los logs grabados
# antes de esta comprobación se siguen analizando exactamente igual.
RE_DERRAPE_INCOHERENTE = re.compile(
    r"\[DERRAPE\] Frame (\d+) v=(\d+): dist=([\d.]+) > umbral pero IGNORADO "
    r"por localización incoherente: la trasera cae en la celda (\d+) y la "
    r"delantera en la (\d+), que están a ([\d.]+) px, pero las pegatinas "
    r"están a ([\d.]+) px"
)
# Cierres "especiales": el coche salió de la ruta / llegó al final de la
# cadena / el controlador avisó de la pérdida de visión. Solo se cuentan.
RE_DERRAPE_CIERRE_VISION = re.compile(
    r"\[DERRAPE\] Frame (\d+) v=(\d+): el (coche salió de la trayectoria|"
    r"coche llegó al final de la cadena abierta|controlador avisa)"
)

# Decisión al registrar un derrape: zona nueva (retroceso largo) o fusión
# (retroceso corto), con el arranque del retroceso
RE_ZONA_DECISION = re.compile(
    r"\[ZONA\] Frame (\d+) vuelta (\d+): derrape en \[(\d+), (\d+)\] -> "
    r"(zona nueva|fusión con (\d+) zona\(s\) existente\(s\)): retroceso "
    r"(?:corto )?de ([\d.]+) px desde la celda (\d+) "
    r"\(derrapes_contador=(\d+)\)"
)
# Carveo resultante en la partición (una línea por tramo contiguo castigado)
RE_ZONA_CARVE_NUEVA = re.compile(
    r"\[ZONA\] Nueva zona de derrape \[(\d+), (\d+)\] pwm=(\d+)"
)
RE_ZONA_CARVE_FUSION = re.compile(
    r"\[ZONA\] Fusión: el castigo \[(\d+), (\d+)\] absorbe (\d+) zona\(s\) "
    r"de derrape \((.+)\) -> zona \[(\d+), (\d+)\] pwm=(\d+)"
)
RE_ZONA_GIGANTE_CASTIGO = re.compile(
    r"\[ZONA\] Celda gigante (\d+) castigada por retroceso: "
    r"pwm (\d+)->(\d+)"
)
RE_ZONA_DERRAME = re.compile(
    r"\[ZONA\] Derrame(?: en cascada)?: (?:el retroceso se salió por el "
    r"inicio de la cadena con|tampoco caben) ([\d.]+) px"
)
RE_ZONA_EXTERNA = re.compile(
    r"\[ZONA\] Reducción externa: la cámara siguiente pide reducir "
    r"([\d.]+) px al final de nuestra trayectoria \(vuelta (\d+)\)"
)
RE_ZONA_ESTADO_CAB = re.compile(r"\[ZONA\] \((estado tras .+)\)$")
RE_ZONA_ESTADO_ITEM = re.compile(
    r"\[ZONA\] \[\s*(\d+)-\s*(\d+)\] (\w+)\s+pwm=(\d+) vueltas=\[([\d, ]*)\]"
)

RE_PERFIL_MOTIVO = re.compile(r"\[PERFIL\] \((.+)\)$")
RE_PERFIL_ITEM = re.compile(
    r"\[PERFIL\] zona \[\s*(\d+)-\s*(\d+)\] (\w+)\s+pwm=(\d+) \((\d+) celdas\)"
)
RE_PERFIL_COMPACTO = re.compile(r"\[PERFIL\] perfil=\[")

RE_VUELTA = re.compile(
    r"\[VUELTA (\d+)\] limpia_global=(True|False) \| derrapes de esta "
    r"cámara: (\d+)->(\d+) \(hubo_derrape_local=(True|False)\)"
)
RE_VUELTA_PROTEGE = re.compile(
    r"\[VUELTA (\d+)\] zona \[(\d+)-(\d+)\]\((\w+)\) PROTEGIDA: último "
    r"derrape en vuelta (\d+)"
)
# Dos formas, y las dos tienen que casar: la política de mejora pasó a
# decidirse zona a zona (antes solo subía el perfil si la vuelta era limpia en
# TODO el circuito, y por eso la línea empezaba por "limpia:"). Los logs
# grabados antes de ese cambio llevan la forma antigua y se siguen analizando.
RE_VUELTA_SUBE = re.compile(
    r"\[VUELTA (\d+)\] (?:limpia: \d+ de \d+ zonas suben|suben \d+ de \d+ zonas)"
)
RE_VUELTA_NO_SUBE = re.compile(r"\[VUELTA (\d+)\] El perfil NO sube: (.+)$")

RE_TRAY_CELDA = re.compile(
    r"\[TRAY\] celda\s+(\d+): \(\s*(-?[\d.]+),\s*(-?[\d.]+)\) \| acum=([\d.]+)"
)
RE_TRAY_CELDA_GIGANTE = re.compile(
    r"\[TRAY\] celda\s+(\d+): GIGANTE \(([\d.]+) px\) \| acum=([\d.]+)"
)
RE_TRAY_CADENA = re.compile(
    r"\[TRAY\] Cadena construida: (\d+) celdas \((\d+) gigantes\), "
    r"longitud total ([\d.]+) px, cerrada=(True|False)"
)

# Líneas conocidas que no aportan datos al análisis: se reconocen para que no
# cuenten como "no reconocidas" (el aviso de líneas desconocidas queda para
# detectar cambios de formato reales en el algoritmo)
RE_IGNORABLES = [
    re.compile(r"\[INIT\] "),  # los pares clave=valor se extraen aparte
    # "Filtrado" es el nombre que tenía la línea de cierre/rotación antes de
    # que el filtrado desapareciera: se mantiene para los logs ya grabados
    re.compile(r"\[TRAY\] Filtrado"),
    re.compile(r"\[TRAY\] Cierre"),
    re.compile(r"\[TRAY\] Trayectoria de calibración"),
    re.compile(r"\[TRAY\] AVISO"),
    re.compile(r"\[TRAY\] Celda GIGANTE"),
    re.compile(r"\[TRAY\] --- Cadena de celdas"),
    re.compile(r"\[TRAY\] setTrayectoria ignorado"),
    re.compile(r"\[TRAY\] Trayectoria RECHAZADA"),
    re.compile(r"\[ZONA\] aplicar_reduccion_externa ignorada"),
]


class Carrera:
    """Contenedor simple de todo lo extraído del log (sin lógica).

    Atributos rellenados por parsear_log():
      fecha            texto ISO de la cabecera del log (o "")
      params           dict {nombre: float} con los [INIT] (v_min, v_max...)
      celdas           DataFrame [celda, x, y, gigante, long_px, acum]
                       (las gigantes llevan x=y=NaN: no hay punto dentro)
      n_celdas         nº total de celdas de la cadena
      long_total       longitud de la cadena en px
      cerrada          bool (cadena cerrada o abierta)
      frames           DataFrame con una fila por [FRAME] válido
      descartados      lista de (frame, vuelta, motivo)
      derrapes         DataFrame de eventos CERRADO [t, linea, frame, vuelta,
                       ini, fin, n_celdas]
      aperturas        lista de dicts de eventos ABIERTO
      zona_muerta      lista de dicts (aperturas rechazadas por zona muerta)
      incoherentes     lista de dicts (aperturas rechazadas porque las dos
                       pegatinas se localizaron en celdas incompatibles)
      cierres_vision   lista de (frame, vuelta) de derrapes cerrados por
                       salir del campo de visión / fin de cadena
      decisiones       lista de dicts [ZONA] decisión (nueva/fusión+retroceso)
      carveos          lista de dicts de carveos en la partición (con pwm)
      gigante_castigos lista de {linea, vuelta, celda, antes, despues}
      derrames         lista de {linea, vuelta, px, t_abs} (reduccion_pendiente)
      externas         lista de {linea, vuelta, px, t_abs} (reducción de otra
                       cámara). t_abs = segundos desde medianoche
      estados_zona     lista de {linea, vuelta, zonas: [dict]} (volcados [ZONA])
      snapshots_perfil lista de {linea, vuelta, motivo, zonas: [dict]}
      vueltas_reg      lista de dicts de los bloques [VUELTA]
      protecciones     lista de {vuelta, ini, fin, tipo, ultimo_derrape}
      no_reconocidas   lista de (nº línea, texto) que no casó con ningún patrón
    """

    def __init__(self):
        self.fecha = ""
        self.params = {}
        self.celdas = None
        self.n_celdas = 0
        self.long_total = 0.0
        self.cerrada = False
        self.frames = None
        self.descartados = []
        self.derrapes = None
        self.aperturas = []
        self.zona_muerta = []
        self.incoherentes = []
        self.cierres_vision = []
        self.decisiones = []
        self.carveos = []
        self.gigante_castigos = []
        self.derrames = []
        self.externas = []
        self.estados_zona = []
        self.snapshots_perfil = []
        self.vueltas_reg = []
        self.protecciones = []
        self.no_reconocidas = []


def parsear_log(ruta: Path) -> Carrera:
    """Una pasada por el log casando cada línea contra su regex.

    El log es secuencial y los mensajes multilínea ([ZONA] estado y [PERFIL]
    volcado) van SIEMPRE seguidos: se abren con su cabecera y las líneas
    siguientes que casen con el patrón "item" se añaden al último registro
    abierto. Los timestamps HH:MM:SS.mmm se convierten a segundos desde la
    primera línea (con corrección de +24 h si cruzara medianoche)."""
    c = Carrera()
    filas_frame = []
    filas_derrape = []
    filas_celda = []

    t0 = None          # segundos absolutos de la primera línea con hora
    t_prev = 0.0       # último t relativo visto (para detectar medianoche)
    ajuste_dia = 0.0   # se suma 86400 cada vez que la hora "retrocede"
    vuelta_actual = 0  # contexto de vuelta para snapshots/castigos

    # Registros multilínea abiertos (None cuando no hay ninguno en curso)
    estado_abierto = None
    snapshot_abierto = None

    with open(ruta, encoding="utf-8") as f:
        lineas = f.readlines()

    for n_linea, cruda in enumerate(lineas):
        cruda = cruda.rstrip("\n")
        if not cruda.strip():
            continue

        m = RE_CABECERA.match(cruda)
        if m:
            c.fecha = m.group(1)
            continue

        m = RE_LINEA.match(cruda)
        if not m:
            c.no_reconocidas.append((n_linea + 1, cruda))
            continue
        hh, mm, ss, ms, cuerpo = m.groups()
        t_abs = int(hh) * 3600 + int(mm) * 60 + int(ss) + int(ms) / 1000.0
        if t0 is None:
            t0 = t_abs
        t = t_abs - t0 + ajuste_dia
        if t < t_prev - 43200:  # la hora retrocedió >12 h: cruzó medianoche
            ajuste_dia += 86400.0
            t += 86400.0
        t_prev = t

        # Los [INIT] llevan pares clave=valor sueltos: se acumulan todos en el
        # dict de parámetros (v_min, v_max, umbral_derrape, paso_celda...)
        if cuerpo.startswith("[INIT]"):
            for clave, valor in re.findall(r"(\w+)=(-?[\d.]+)", cuerpo):
                c.params[clave] = float(valor)
            continue

        # --- continuaciones de mensajes multilínea (van antes que el resto
        # porque sus patrones son subconjuntos de otros mensajes [ZONA]/[PERFIL])
        if estado_abierto is not None:
            m = RE_ZONA_ESTADO_ITEM.search(cuerpo)
            if m:
                estado_abierto["zonas"].append(
                    {
                        "ini": int(m.group(1)),
                        "fin": int(m.group(2)),
                        "tipo": m.group(3),
                        "pwm": int(m.group(4)),
                        "vueltas": [int(v) for v in re.findall(r"\d+", m.group(5))],
                    }
                )
                continue
            estado_abierto = None  # primera línea que no es item: se cierra
        if snapshot_abierto is not None:
            m = RE_PERFIL_ITEM.search(cuerpo)
            if m:
                snapshot_abierto["zonas"].append(
                    {
                        "ini": int(m.group(1)),
                        "fin": int(m.group(2)),
                        "tipo": m.group(3),
                        "pwm": int(m.group(4)),
                    }
                )
                continue
            if RE_PERFIL_COMPACTO.search(cuerpo):
                continue  # la línea perfil=[[...]] es redundante con los items
            snapshot_abierto = None

        # --- [FRAME]: el tipo más frecuente, se prueba primero
        m = RE_FRAME.search(cuerpo)
        if m:
            g = m.groups()
            # Índices de grupo (0-based): 0 frame, 1 vuelta, 2-3 front x/y,
            # 4 c_f, 5 d_f, 6-9 back x/y/c/d (None si no se localizó),
            # 10 derrapando, 11-13 zona ini/fin/tipo (None si zona=?),
            # 14 pwm, 15 margen
            vuelta_actual = int(g[1])
            back_ok = g[6] is not None
            zona_ok = g[11] is not None
            filas_frame.append(
                {
                    "t": t,
                    "linea": n_linea,
                    "frame": int(g[0]),
                    "vuelta": int(g[1]),
                    "fx": float(g[2]),
                    "fy": float(g[3]),
                    "c_f": int(g[4]),
                    "d_f": float(g[5]),
                    "bx": float(g[6]) if back_ok else np.nan,
                    "by": float(g[7]) if back_ok else np.nan,
                    "c_b": int(g[8]) if back_ok else -1,
                    "d_b": float(g[9]) if back_ok else np.nan,
                    "derrapando": g[10] == "True",
                    "z_ini": int(g[11]) if zona_ok else -1,
                    "z_fin": int(g[12]) if zona_ok else -1,
                    "z_tipo": g[13] if zona_ok else "?",
                    "pwm": int(g[14]),
                    "margen": np.inf if g[15] == "inf" else float(g[15]),
                }
            )
            continue
        m = RE_FRAME_DESCARTADO.search(cuerpo)
        if m:
            c.descartados.append((int(m.group(1)), int(m.group(2)), m.group(3).strip()))
            continue

        # --- [DERRAPE]
        m = RE_DERRAPE_CERRADO.search(cuerpo)
        if m:
            filas_derrape.append(
                {
                    "t": t,
                    "linea": n_linea,
                    "frame": int(m.group(1)),
                    "vuelta": int(m.group(2)),
                    "ini": int(m.group(3)),
                    "fin": int(m.group(4)),
                    "n_celdas": int(m.group(5)),
                }
            )
            continue
        m = RE_DERRAPE_ABIERTO.search(cuerpo)
        if m:
            c.aperturas.append(
                {
                    "t": t,
                    "frame": int(m.group(1)),
                    "vuelta": int(m.group(2)),
                    "celda": int(m.group(3)),
                    "dist": float(m.group(4)),
                }
            )
            continue
        m = RE_DERRAPE_ZONA_MUERTA.search(cuerpo)
        if m:
            c.zona_muerta.append(
                {
                    "frame": int(m.group(1)),
                    "vuelta": int(m.group(2)),
                    "dist": float(m.group(3)),
                    "celda": int(m.group(4)),
                }
            )
            continue
        m = RE_DERRAPE_INCOHERENTE.search(cuerpo)
        if m:
            c.incoherentes.append(
                {
                    "frame": int(m.group(1)),
                    "vuelta": int(m.group(2)),
                    "dist": float(m.group(3)),
                    "celda": int(m.group(4)),
                    "celda_front": int(m.group(5)),
                    "sep_celdas": float(m.group(6)),
                    "sep_pegatinas": float(m.group(7)),
                }
            )
            continue
        m = RE_DERRAPE_CIERRE_VISION.search(cuerpo)
        if m:
            c.cierres_vision.append((int(m.group(1)), int(m.group(2))))
            continue

        # --- [ZONA]
        m = RE_ZONA_DECISION.search(cuerpo)
        if m:
            c.decisiones.append(
                {
                    "frame": int(m.group(1)),
                    "vuelta": int(m.group(2)),
                    "ini": int(m.group(3)),
                    "fin": int(m.group(4)),
                    "fusion": m.group(5) != "zona nueva",
                    "n_tocadas": int(m.group(6)) if m.group(6) else 0,
                    "retroceso": float(m.group(7)),
                    "arranque": int(m.group(8)),
                    "contador": int(m.group(9)),
                }
            )
            continue
        m = RE_ZONA_CARVE_FUSION.search(cuerpo)
        if m:
            absorbidas = [
                (int(a), int(b))
                for a, b in re.findall(r"\[(\d+)-(\d+)\]", m.group(4))
            ]
            c.carveos.append(
                {
                    "linea": n_linea,
                    "vuelta": vuelta_actual,
                    "tipo": "fusion",
                    "ini": int(m.group(5)),
                    "fin": int(m.group(6)),
                    "pwm": int(m.group(7)),
                    "absorbidas": absorbidas,
                }
            )
            continue
        m = RE_ZONA_CARVE_NUEVA.search(cuerpo)
        if m:
            c.carveos.append(
                {
                    "linea": n_linea,
                    "vuelta": vuelta_actual,
                    "tipo": "nueva",
                    "ini": int(m.group(1)),
                    "fin": int(m.group(2)),
                    "pwm": int(m.group(3)),
                    "absorbidas": [],
                }
            )
            continue
        m = RE_ZONA_GIGANTE_CASTIGO.search(cuerpo)
        if m:
            c.gigante_castigos.append(
                {
                    "linea": n_linea,
                    "vuelta": vuelta_actual,
                    "celda": int(m.group(1)),
                    "antes": int(m.group(2)),
                    "despues": int(m.group(3)),
                }
            )
            continue
        # `t_abs` (segundos desde medianoche) y no el `t` relativo: es el reloj
        # común que permite casar el "Derrame" de una cámara con la "Reducción
        # externa" de la otra, que son las dos caras del mismo evento. Mismo
        # campo que en parseo_log.py, que lleva el parser gemelo
        m = RE_ZONA_DERRAME.search(cuerpo)
        if m:
            c.derrames.append(
                {"linea": n_linea, "vuelta": vuelta_actual,
                 "px": float(m.group(1)), "t_abs": t_abs}
            )
            continue
        m = RE_ZONA_EXTERNA.search(cuerpo)
        if m:
            c.externas.append(
                {"linea": n_linea, "vuelta": int(m.group(2)),
                 "px": float(m.group(1)), "t_abs": t_abs}
            )
            continue
        m = RE_ZONA_ESTADO_CAB.search(cuerpo)
        if m:
            estado_abierto = {"linea": n_linea, "vuelta": vuelta_actual, "zonas": []}
            c.estados_zona.append(estado_abierto)
            continue

        # --- [PERFIL]
        m = RE_PERFIL_MOTIVO.search(cuerpo)
        if m:
            snapshot_abierto = {
                "linea": n_linea,
                "vuelta": vuelta_actual,
                "motivo": m.group(1),
                "zonas": [],
            }
            c.snapshots_perfil.append(snapshot_abierto)
            continue

        # --- [VUELTA]
        m = RE_VUELTA.search(cuerpo)
        if m:
            vuelta_actual = int(m.group(1))
            c.vueltas_reg.append(
                {
                    "t": t,
                    "linea": n_linea,
                    "vuelta": int(m.group(1)),
                    "limpia_global": m.group(2) == "True",
                    "derr_antes": int(m.group(3)),
                    "derr_despues": int(m.group(4)),
                    "hubo_local": m.group(5) == "True",
                    "sube": None,  # se rellena con la línea siguiente
                    "motivo_no": "",
                }
            )
            continue
        m = RE_VUELTA_PROTEGE.search(cuerpo)
        if m:
            c.protecciones.append(
                {
                    "vuelta": int(m.group(1)),
                    "ini": int(m.group(2)),
                    "fin": int(m.group(3)),
                    "tipo": m.group(4),
                    "ultimo_derrape": int(m.group(5)),
                }
            )
            continue
        m = RE_VUELTA_SUBE.search(cuerpo)
        if m:
            if c.vueltas_reg:
                c.vueltas_reg[-1]["sube"] = True
            continue
        m = RE_VUELTA_NO_SUBE.search(cuerpo)
        if m:
            if c.vueltas_reg:
                c.vueltas_reg[-1]["sube"] = False
                c.vueltas_reg[-1]["motivo_no"] = m.group(2)
            continue

        # --- [TRAY]
        m = RE_TRAY_CELDA.search(cuerpo)
        if m:
            filas_celda.append(
                {
                    "celda": int(m.group(1)),
                    "x": float(m.group(2)),
                    "y": float(m.group(3)),
                    "gigante": False,
                    "long_px": np.nan,
                    "acum": float(m.group(4)),
                }
            )
            continue
        m = RE_TRAY_CELDA_GIGANTE.search(cuerpo)
        if m:
            filas_celda.append(
                {
                    "celda": int(m.group(1)),
                    "x": np.nan,
                    "y": np.nan,
                    "gigante": True,
                    "long_px": float(m.group(2)),
                    "acum": float(m.group(3)),
                }
            )
            continue
        m = RE_TRAY_CADENA.search(cuerpo)
        if m:
            c.n_celdas = int(m.group(1))
            c.long_total = float(m.group(3))
            c.cerrada = m.group(4) == "True"
            continue

        if any(r.search(cuerpo) for r in RE_IGNORABLES):
            continue
        c.no_reconocidas.append((n_linea + 1, cruda))

    c.frames = pd.DataFrame(filas_frame)
    c.derrapes = pd.DataFrame(
        filas_derrape,
        columns=["t", "linea", "frame", "vuelta", "ini", "fin", "n_celdas"],
    )
    c.celdas = pd.DataFrame(
        filas_celda, columns=["celda", "x", "y", "gigante", "long_px", "acum"]
    )
    return c


# ---------------------------------------------------------------------------
# Derivados por vuelta
# ---------------------------------------------------------------------------
def zonas_a_celdas(zonas, n_celdas):
    """Expande una partición de zonas a un array de PWM por celda (NaN donde
    ninguna zona cubra, que no debería pasar)."""
    arr = np.full(n_celdas, np.nan)
    for z in zonas:
        arr[z["ini"]: z["fin"] + 1] = z["pwm"]
    return arr


def derivar_por_vuelta(c: Carrera):
    """Reconstruye el estado del algoritmo vuelta a vuelta.

    Devuelve (vueltas, perfil_vuelta, zonas_ini_vuelta, zonas_fin_vuelta):
      vueltas          lista ordenada de números de vuelta con frames
      perfil_vuelta    {v: array de PWM por celda} = perfil CON EL QUE SE
                       CORRIÓ la vuelta v (último volcado [PERFIL] anterior a
                       su primer frame)
      zonas_ini_vuelta {v: [zona]} = partición al EMPEZAR la vuelta v
      zonas_fin_vuelta {v: [zona]} = partición al TERMINAR la vuelta v
                       (último volcado anterior al primer frame de v+1)
    """
    vueltas = sorted(c.frames["vuelta"].unique().tolist())
    primera_linea = c.frames.groupby("vuelta")["linea"].min().to_dict()

    perfil_vuelta = {}
    zonas_ini_vuelta = {}
    zonas_fin_vuelta = {}
    for v in vueltas:
        frontera_ini = primera_linea[v]
        candidatos = [s for s in c.snapshots_perfil if s["linea"] < frontera_ini]
        if candidatos:
            perfil_vuelta[v] = zonas_a_celdas(candidatos[-1]["zonas"], c.n_celdas)
            zonas_ini_vuelta[v] = candidatos[-1]["zonas"]
        else:
            perfil_vuelta[v] = np.full(c.n_celdas, np.nan)
            zonas_ini_vuelta[v] = []
        idx = vueltas.index(v)
        frontera_fin = (
            primera_linea[vueltas[idx + 1]] if idx + 1 < len(vueltas) else float("inf")
        )
        candidatos = [s for s in c.snapshots_perfil if s["linea"] < frontera_fin]
        zonas_fin_vuelta[v] = candidatos[-1]["zonas"] if candidatos else []
    return vueltas, perfil_vuelta, zonas_ini_vuelta, zonas_fin_vuelta


def tabla_vueltas(c: Carrera, vueltas, perfil_vuelta) -> pd.DataFrame:
    """Tabla resumen con una fila por vuelta: derrapes, si el perfil subió al
    terminarla, duración, PWM medio (del perfil y el realmente aplicado) y
    velocidad media medida (celdas/s convertidas a px/s)."""
    paso = c.params.get("paso_celda", 30.0)
    # El bloque [VUELTA v] se emite al CRUZAR meta: cierra la vuelta v-1 y abre
    # la v. Por eso los datos "de cierre" de la vuelta v salen de [VUELTA v+1].
    reg_por_vuelta = {r["vuelta"]: r for r in c.vueltas_reg}
    filas = []
    for v in vueltas:
        fr = c.frames[c.frames["vuelta"] == v]
        cierre = reg_por_vuelta.get(v + 1)

        apertura = reg_por_vuelta.get(v)
        t_ini = apertura["t"] if apertura else fr["t"].min()
        duracion = (cierre["t"] - t_ini) if cierre else np.nan

        # Velocidad media: avances de celda entre frames consecutivos por el
        # paso de celda, filtrando saltos enormes (localización falsa o cruce
        # del origen en cadena cerrada) y huecos temporales (frames perdidos)
        dc = fr["c_f"].diff().abs()
        dt = fr["t"].diff()
        validos = (dc <= 3) & (dt > 0) & (dt < 0.25)
        vel_media = (
            dc[validos].sum() * paso / dt[validos].sum()
            if dt[validos].sum() > 0 else np.nan
        )

        perfil = perfil_vuelta.get(v)
        filas.append(
            {
                "vuelta": v,
                "n_derrapes": int((c.derrapes["vuelta"] == v).sum()),
                "sube": cierre["sube"] if cierre else None,
                "duracion": duracion,
                "pwm_perfil_medio": np.nanmean(perfil) if perfil is not None else np.nan,
                "pwm_aplicado_medio": fr["pwm"].mean(),
                "vel_media": vel_media,
                "n_frames": len(fr),
            }
        )
    return pd.DataFrame(filas)


# ---------------------------------------------------------------------------
# Detector de anomalías: las comprobaciones que responden a "esta zona se ha
# creado de forma rara". Devuelve una lista de textos (vacía si todo normal).
# ---------------------------------------------------------------------------
def detectar_anomalias(c: Carrera, zonas_fin_vuelta, vueltas):
    avisos = []
    n = max(c.n_celdas, 1)

    # 1) Derrapes individuales que abarcan demasiadas celdas: un derrape
    # físico dura unas pocas; media cadena indica que la trasera se localizó
    # mal en algún frame intermedio y el intervalo se estiró
    for _, d in c.derrapes.iterrows():
        if d["n_celdas"] >= FRAC_CELDAS_DERRAPE_ANOMALO * n:
            avisos.append(
                f"Derrape SOSPECHOSO de {int(d['n_celdas'])} celdas "
                f"({100 * d['n_celdas'] / n:.0f} % de la cadena) en la vuelta "
                f"{int(d['vuelta'])}, frame {int(d['frame'])}: celdas "
                f"[{int(d['ini'])}, {int(d['fin'])}]. Un derrape real dura "
                f"unas pocas celdas: revisar los saltos de celda de la "
                f"trasera en esos frames (siguiente comprobación)."
            )

    # 2) Saltos de celda de la pegatina trasera ENTRE frames consecutivos con
    # el derrape abierto: el mecanismo exacto por el que se estira el intervalo
    fr = c.frames
    dc_b = fr["c_b"].diff().abs()
    consecutivos = fr["frame"].diff() <= 2  # tolera 1 frame descartado en medio
    con_derrape = fr["derrapando"] & fr["derrapando"].shift(fill_value=False)
    saltos = fr[(dc_b > SALTO_CELDAS_TRASERA_ANOMALO) & con_derrape & consecutivos]
    for _, s in saltos.iterrows():
        avisos.append(
            f"Salto de celda de la TRASERA con derrape abierto en la vuelta "
            f"{int(s['vuelta'])}, frame {int(s['frame'])}: pasó a la celda "
            f"{int(s['c_b'])} ({int(dc_b.loc[s.name])} celdas de golpe) — el "
            f"derrape en curso se extiende hasta ahí."
        )

    # 3) Zonas de derrape gigantes al final de la carrera: cubren tanta cadena
    # que su protección bloquea la subida del perfil en casi todo el circuito
    if vueltas:
        for z in zonas_fin_vuelta[vueltas[-1]]:
            if z["tipo"] != "derrape":
                continue
            cobertura = (z["fin"] - z["ini"] + 1) / n
            if cobertura >= FRAC_ZONA_GIGANTE:
                avisos.append(
                    f"Zona GIGANTE al final: [{z['ini']}, {z['fin']}] cubre el "
                    f"{100 * cobertura:.0f} % de la cadena (pwm {z['pwm']}). "
                    f"Una zona así congela el perfil de casi todo el circuito."
                )

    # 4) Datos informativos que ayudan a interpretar lo anterior
    unos = int((c.derrapes["n_celdas"] == 1).sum())
    if unos:
        avisos.append(
            f"INFO: {unos} de {len(c.derrapes)} derrapes duran 1 sola celda. "
            f"Son picos de un frame por encima del umbral: puede interesar "
            f"exigir una duración mínima antes de registrar zona."
        )
    if c.descartados:
        avisos.append(
            f"INFO: {len(c.descartados)} frames DESCARTADOS por ruido "
            f"(front a más de max_dist_ruta de la trayectoria)."
        )
    if c.zona_muerta:
        avisos.append(
            f"INFO: {len(c.zona_muerta)} aperturas de derrape ignoradas por "
            f"zona muerta (junto a un extremo o a una celda gigante)."
        )
    if c.incoherentes:
        celdas = sorted({e["celda"] for e in c.incoherentes})
        avisos.append(
            f"INFO: {len(c.incoherentes)} aperturas de derrape ignoradas porque "
            f"las dos pegatinas se localizaron en celdas incompatibles "
            f"(trasera en {celdas}). El coche circulaba por un tramo sin "
            f"celdas: mirar los huecos que lista [TRAY] AVISO."
        )
    if c.cierres_vision:
        avisos.append(
            f"INFO: {len(c.cierres_vision)} derrapes cerrados por salir del "
            f"campo de visión (fin de cadena o pérdida de la ruta)."
        )
    if c.derrames:
        avisos.append(
            f"INFO: {len(c.derrames)} derrames de reducción hacia la cámara "
            f"precedente (px: "
            + ", ".join(f"{d['px']:.0f}" for d in c.derrames) + ")."
        )
    if c.externas:
        avisos.append(
            f"INFO: {len(c.externas)} reducciones externas recibidas de la "
            f"cámara siguiente (px: "
            + ", ".join(f"{d['px']:.0f}" for d in c.externas) + ")."
        )
    return avisos


# ---------------------------------------------------------------------------
# Aspecto común de las figuras
# ---------------------------------------------------------------------------
def _layout_base(fig, titulo, alto):
    """Chrome común: superficie clara, tinta oscura, rejilla fina, sin logo."""
    fig.update_layout(
        title=dict(text=titulo, font=dict(size=15, color=COL_TINTA)),
        height=alto,
        paper_bgcolor=COL_SUPERFICIE,
        plot_bgcolor=COL_SUPERFICIE,
        font=dict(family=FUENTE, size=12, color=COL_TINTA),
        margin=dict(l=70, r=30, t=95, b=60),
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
        hoverlabel=dict(font=dict(family=FUENTE, size=12)),
    )
    fig.update_xaxes(gridcolor=COL_GRID, zeroline=False, linecolor=COL_CONTEXTO)
    fig.update_yaxes(gridcolor=COL_GRID, zeroline=False, linecolor=COL_CONTEXTO)
    return fig


def _celda_fisica(c: Carrera, celda):
    """Posición (x, y) de una celda para el hover (None si es gigante)."""
    if c.celdas is None or c.celdas.empty:
        return None
    sel = c.celdas[c.celdas["celda"] == celda]
    if sel.empty or bool(sel.iloc[0]["gigante"]):
        return None
    return sel.iloc[0]


# ===========================================================================
# FIGURA 1: evolución de las zonas (la central del análisis)
# ===========================================================================
# Eje X = celda de la cadena, eje Y = vuelta (la primera arriba: se lee hacia
# abajo como el propio log). En cada fila (vuelta) se dibuja:
#   - barra naranja gruesa  = cada zona de DERRAPE tal como queda al terminar
#     esa vuelta, con su PWM en el hover
#   - banda ámbar           = las celdas gigantes (fijas todas las vueltas)
#   - segmento/rombo rojo   = cada derrape individual de esa vuelta
#   - estrella roja         = una fusión (el hover lista las zonas absorbidas)
#   - círculo gris          = apertura ignorada por la zona muerta
#   - triángulo ámbar       = derrame de reducción hacia la cámara precedente
# ===========================================================================
def grafica_1_zonas(c: Carrera, vueltas, zonas_fin_vuelta):
    fig = go.Figure()

    # Bandas verticales fijas en las celdas gigantes
    gigantes = c.celdas[c.celdas["gigante"]] if c.celdas is not None else []
    if len(gigantes):
        for _, g in gigantes.iterrows():
            fig.add_vrect(
                x0=g["celda"] - 0.5, x1=g["celda"] + 0.5,
                fillcolor=COL_AVISO, opacity=0.15, line_width=0,
            )

    # Zonas de derrape al cierre de cada vuelta: una única traza con
    # segmentos separados por None (ligero); el hover lleva pwm e historial
    xs, ys, textos = [], [], []
    for v in vueltas:
        for z in zonas_fin_vuelta[v]:
            if z["tipo"] != "derrape":
                continue
            # El historial de vueltas solo viene en los volcados [ZONA] (los
            # [PERFIL] no lo llevan): puede no estar disponible
            hist = z.get("vueltas")
            texto = (
                f"zona [{z['ini']}, {z['fin']}] ({z['fin'] - z['ini'] + 1} celdas)"
                f"<br>pwm={z['pwm']}"
                + (f"<br>derrapes en vueltas: {hist}" if hist is not None else "")
            )
            xs += [z["ini"], z["fin"], None]
            ys += [v, v, None]
            textos += [texto, texto, ""]
    fig.add_trace(
        go.Scatter(
            # lines+markers: los marcadores hacen visibles las zonas de 1 celda
            x=xs, y=ys, mode="lines+markers", name="zona de derrape",
            line=dict(color=COL_SERIO, width=7),
            marker=dict(size=5, color=COL_SERIO),
            text=textos, hovertemplate="%{text}<extra></extra>",
        )
    )

    # Derrapes individuales de cada vuelta (eventos CERRADO), medio carril
    # por debajo de la fila para no pisar la barra de la zona
    xs, ys, textos = [], [], []
    px_uno, py_uno, t_uno = [], [], []
    for _, d in c.derrapes.iterrows():
        texto = (
            f"derrape v{int(d['vuelta'])} frame {int(d['frame'])}"
            f"<br>celdas [{int(d['ini'])}, {int(d['fin'])}] "
            f"({int(d['n_celdas'])} celdas)"
        )
        if d["n_celdas"] <= 1:
            px_uno.append(d["ini"])
            py_uno.append(d["vuelta"] - 0.30)
            t_uno.append(texto + "<br>(1 sola celda)")
        else:
            xs += [d["ini"], d["fin"], None]
            ys += [d["vuelta"] - 0.30, d["vuelta"] - 0.30, None]
            textos += [texto, texto, ""]
    fig.add_trace(
        go.Scatter(
            x=xs, y=ys, mode="lines", name="derrape (evento)",
            line=dict(color=COL_CRITICO, width=3),
            text=textos, hovertemplate="%{text}<extra></extra>",
        )
    )
    fig.add_trace(
        go.Scatter(
            x=px_uno, y=py_uno, mode="markers", name="derrape de 1 celda",
            marker=dict(symbol="diamond", size=7, color=COL_CRITICO),
            text=t_uno, hovertemplate="%{text}<extra></extra>",
        )
    )

    # Fusiones: estrella en el centro del carveo que absorbió zonas
    fus = [k for k in c.carveos if k["tipo"] == "fusion"]
    fig.add_trace(
        go.Scatter(
            x=[(k["ini"] + k["fin"]) / 2 for k in fus],
            y=[k["vuelta"] - 0.30 for k in fus],
            mode="markers", name="fusión de zonas",
            marker=dict(symbol="star", size=13, color=COL_CRITICO,
                        line=dict(color=COL_SUPERFICIE, width=1)),
            text=[
                f"FUSIÓN en v{k['vuelta']}: la zona pasa a "
                f"[{k['ini']}, {k['fin']}] pwm={k['pwm']}<br>absorbe: "
                + ", ".join(f"[{a}-{b}]" for a, b in k["absorbidas"])
                for k in fus
            ],
            hovertemplate="%{text}<extra></extra>",
        )
    )

    # Aperturas ignoradas por la zona muerta (contexto en gris)
    fig.add_trace(
        go.Scatter(
            x=[e["celda"] for e in c.zona_muerta],
            y=[e["vuelta"] - 0.30 for e in c.zona_muerta],
            mode="markers", name="ignorado (zona muerta)",
            marker=dict(symbol="circle-open", size=6, color=COL_MUTED),
            text=[
                f"v{e['vuelta']} frame {e['frame']}: dist={e['dist']:.1f} > "
                f"umbral pero la celda {e['celda']} está en zona muerta"
                for e in c.zona_muerta
            ],
            hovertemplate="%{text}<extra></extra>",
        )
    )

    # Derrames hacia la cámara precedente (salen por la celda 0) y
    # reducciones externas recibidas (entran por la última celda)
    fig.add_trace(
        go.Scatter(
            x=[0] * len(c.derrames),
            y=[d["vuelta"] - 0.30 for d in c.derrames],
            mode="markers", name="derrame a la precedente",
            marker=dict(symbol="triangle-left", size=11, color=COL_AVISO),
            text=[f"v{d['vuelta']}: se derraman {d['px']:.0f} px hacia la "
                  f"cámara precedente" for d in c.derrames],
            hovertemplate="%{text}<extra></extra>",
        )
    )
    fig.add_trace(
        go.Scatter(
            x=[max(c.n_celdas - 1, 0)] * len(c.externas),
            y=[d["vuelta"] - 0.30 for d in c.externas],
            mode="markers", name="reducción externa recibida",
            marker=dict(symbol="triangle-right", size=11, color=COL_AVISO),
            text=[f"v{d['vuelta']}: la cámara siguiente pide reducir "
                  f"{d['px']:.0f} px al final" for d in c.externas],
            hovertemplate="%{text}<extra></extra>",
        )
    )

    fig.update_xaxes(
        title_text="celda de la cadena",
        range=[-1.5, c.n_celdas + 0.5],
    )
    fig.update_yaxes(
        title_text="vuelta", range=[vueltas[-1] + 0.8, vueltas[0] - 0.8],
        dtick=1,
    )
    alto = max(500, 130 + 22 * len(vueltas))
    return _layout_base(
        fig,
        "1 · Evolución de las zonas de derrape "
        "(estado al cerrar cada vuelta + eventos de esa vuelta; "
        "banda ámbar = celda gigante)",
        alto,
    )


# ===========================================================================
# FIGURA 2: heatmap del perfil de PWM (vuelta x celda)
# ===========================================================================
# Cada fila es el perfil CON EL QUE EL COCHE CORRIÓ esa vuelta (el último
# volcado [PERFIL] anterior a su primer frame), expandiendo las zonas a
# celdas. Claro = lento (v_min), oscuro = rápido (v_max). Encima:
#   - aspa roja     = celdas castigadas DURANTE esa vuelta (carveos y
#                     castigos de gigantes; efecto en la fila siguiente)
#   - círculo gris  = celdas protegidas al EMPEZAR esa vuelta (no subieron
#                     aunque la vuelta anterior fuera limpia)
# ===========================================================================
def grafica_2_heatmap(c: Carrera, vueltas, perfil_vuelta):
    v_min = c.params.get("v_min", np.nan)
    v_max = c.params.get("v_max", np.nan)

    z = np.full((len(vueltas), c.n_celdas), np.nan)
    for i, v in enumerate(vueltas):
        z[i, :] = perfil_vuelta[v]
    fig = go.Figure(
        go.Heatmap(
            z=z, x=list(range(c.n_celdas)), y=vueltas,
            colorscale=ESCALA_PWM, zmin=v_min, zmax=v_max,
            xgap=1, ygap=1,
            colorbar=dict(title="PWM"),
            hovertemplate=("vuelta %{y} · celda %{x}"
                           "<br>PWM del perfil: %{z}<extra></extra>"),
        )
    )
    # Castigos aplicados durante cada vuelta: los carveos (todas sus celdas)
    # y los castigos de celdas gigantes
    cast_x, cast_y, cast_t = [], [], []
    for k in c.carveos:
        for i in range(k["ini"], k["fin"] + 1):
            cast_x.append(i)
            cast_y.append(k["vuelta"])
            cast_t.append(f"castigo en v{k['vuelta']}: celda {i} -> pwm {k['pwm']}")
    for g in c.gigante_castigos:
        cast_x.append(g["celda"])
        cast_y.append(g["vuelta"])
        cast_t.append(
            f"gigante castigada en v{g['vuelta']}: "
            f"{g['antes']}→{g['despues']}"
        )
    fig.add_trace(
        go.Scatter(
            x=cast_x, y=cast_y, mode="markers", name="castigo en esa vuelta",
            marker=dict(symbol="x-thin", size=6,
                        line=dict(color=COL_CRITICO, width=1.5)),
            text=cast_t, hovertemplate="%{text}<extra></extra>",
        )
    )
    # Zonas protegidas al empezar cada vuelta (las líneas PROTEGIDA del
    # bloque [VUELTA v], que es el que ABRE la vuelta v)
    prot_x, prot_y, prot_t = [], [], []
    for p in c.protecciones:
        for i in range(p["ini"], p["fin"] + 1):
            prot_x.append(i)
            prot_y.append(p["vuelta"])
            prot_t.append(
                f"v{p['vuelta']}: celda {i} protegida "
                f"(zona [{p['ini']}-{p['fin']}], último derrape en "
                f"v{p['ultimo_derrape']})"
            )
    fig.add_trace(
        go.Scatter(
            x=prot_x, y=prot_y, mode="markers", name="celda protegida (no sube)",
            marker=dict(symbol="circle-open", size=5, color=COL_TINTA_2),
            text=prot_t, hovertemplate="%{text}<extra></extra>",
        )
    )
    fig.update_xaxes(title_text="celda de la cadena")
    fig.update_yaxes(title_text="vuelta",
                     range=[vueltas[-1] + 0.5, vueltas[0] - 0.5], dtick=2)
    alto = max(500, 150 + 20 * len(vueltas))
    return _layout_base(
        fig,
        f"2 · Perfil de PWM con el que se corrió cada vuelta "
        f"(claro={v_min:.0f}, oscuro={v_max:.0f})",
        alto,
    )


# ===========================================================================
# FIGURA 3: mapa del circuito coloreado por PWM, con slider de vuelta
# ===========================================================================
# Las celdas [TRAY] en sus coordenadas de imagen (Y invertida, escala 1:1),
# coloreadas por el PWM de su zona en la vuelta del slider. Las celdas dentro
# de una zona de derrape vigente al EMPEZAR esa vuelta llevan un anillo rojo.
# Las celdas gigantes se dibujan como una línea ámbar discontinua entre sus
# celdas vecinas (ahí el circuito no se ve).
# ===========================================================================
def grafica_3_mapa(c: Carrera, vueltas, perfil_vuelta, zonas_ini_vuelta):
    v_min = c.params.get("v_min", None)
    v_max = c.params.get("v_max", None)
    celdas = c.celdas.sort_values("celda")
    normales = celdas[~celdas["gigante"]]

    def datos_vuelta(v):
        """(colores PWM, anchos de anillo, textos) de las celdas normales en
        la vuelta v, según su zona al EMPEZAR la vuelta."""
        zonas = zonas_ini_vuelta.get(v, [])
        perfil = perfil_vuelta[v]
        colores, anchos, textos = [], [], []
        for _, cd in normales.iterrows():
            i = int(cd["celda"])
            pwm = perfil[i] if i < len(perfil) and not np.isnan(perfil[i]) else None
            en_zona = any(
                z["tipo"] == "derrape" and z["ini"] <= i <= z["fin"] for z in zonas
            )
            colores.append(pwm)
            anchos.append(2.5 if en_zona else 0)
            textos.append(
                f"celda {i}<br>PWM {pwm}"
                + ("<br><b>dentro de zona de derrape</b>" if en_zona else "")
            )
        return colores, anchos, textos

    # Trayectoria de fondo (gris) cortada en las celdas gigantes, y los
    # huecos tapados como línea ámbar discontinua entre las celdas vecinas
    tx, ty = [], []
    gx, gy, gt = [], [], []
    filas = celdas.reset_index(drop=True)
    for i, fila in filas.iterrows():
        if fila["gigante"]:
            tx.append(None)
            ty.append(None)
            ant = filas.iloc[i - 1] if i > 0 else None
            sig = filas.iloc[i + 1] if i + 1 < len(filas) else None
            if ant is not None and sig is not None and not ant["gigante"] \
                    and not sig["gigante"]:
                gx += [ant["x"], sig["x"], None]
                gy += [ant["y"], sig["y"], None]
                texto = (f"celda gigante {int(fila['celda'])}: hueco tapado de "
                         f"{fila['long_px']:.0f} px")
                gt += [texto, texto, ""]
        else:
            tx.append(fila["x"])
            ty.append(fila["y"])
    # En cadena cerrada, cerrar el dibujo uniendo la última con la primera
    if c.cerrada and len(normales) > 1:
        tx.append(normales.iloc[0]["x"])
        ty.append(normales.iloc[0]["y"])

    colores, anchos, textos = datos_vuelta(vueltas[0])
    fig = go.Figure(
        data=[
            go.Scatter(x=tx, y=ty, mode="lines", name="trayectoria",
                       line=dict(color=COL_GRID, width=2), hoverinfo="skip"),
            go.Scatter(x=gx, y=gy, mode="lines", name="hueco tapado (gigante)",
                       line=dict(color=COL_AVISO, width=2, dash="dash"),
                       text=gt, hovertemplate="%{text}<extra></extra>"),
            go.Scatter(
                x=normales["x"], y=normales["y"], mode="markers", name="celdas",
                marker=dict(
                    size=11, color=colores, colorscale=ESCALA_PWM,
                    cmin=v_min, cmax=v_max, colorbar=dict(title="PWM"),
                    line=dict(color=COL_CRITICO, width=anchos),
                ),
                text=textos, hovertemplate="%{text}<extra></extra>",
            ),
        ],
        frames=[
            go.Frame(
                name=str(v),
                traces=[2],  # cada frame solo redefine la traza de las celdas
                data=[go.Scatter(
                    marker=dict(
                        size=11, color=dv[0], colorscale=ESCALA_PWM,
                        cmin=v_min, cmax=v_max, colorbar=dict(title="PWM"),
                        line=dict(color=COL_CRITICO, width=dv[1]),
                    ),
                    text=dv[2],
                )],
            )
            for v in vueltas
            for dv in [datos_vuelta(v)]
        ],
    )
    fig.update_layout(
        sliders=[dict(
            active=0, currentvalue=dict(prefix="Vuelta "), pad=dict(t=30),
            steps=[dict(
                label=str(v), method="animate",
                args=[[str(v)], dict(mode="immediate",
                                     frame=dict(duration=0, redraw=True),
                                     transition=dict(duration=0))],
            ) for v in vueltas],
        )],
    )
    # Coordenadas de imagen: origen arriba a la izquierda (Y invertida) y
    # misma escala en ambos ejes para que el circuito no salga deformado
    fig.update_yaxes(autorange="reversed", scaleanchor="x", scaleratio=1,
                     title_text="y (px de imagen)")
    fig.update_xaxes(title_text="x (px de imagen)")
    return _layout_base(
        fig,
        "3 · El circuito visto por la cámara: PWM de cada celda "
        "(anillo rojo = celda dentro de una zona de derrape)",
        720,
    )


# ===========================================================================
# FIGURA 4a: distancia perpendicular de la trasera frente a su celda
# ===========================================================================
# La serie con la que trabaja la máquina de derrapes: d(celda) de la pegatina
# trasera. Slider para aislar cada vuelta; de fondo (gris) todas las vueltas.
# Los puntos con derrape abierto van en rojo. La línea negra es el umbral.
# ===========================================================================
def grafica_4a_dist(c: Carrera, vueltas):
    umbral = c.params.get("umbral_derrape", 8.0)
    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            x=c.frames["c_b"], y=c.frames["d_b"], mode="markers",
            name="todas las vueltas",
            marker=dict(size=3, color=COL_CONTEXTO), opacity=0.5,
            hoverinfo="skip",
        )
    )
    for v in vueltas:
        fr = c.frames[c.frames["vuelta"] == v].sort_values("frame")
        # None donde la trasera salta mucho: que la línea no cruce la figura
        xs, ys, textos = [], [], []
        c_prev = None
        for _, r in fr.iterrows():
            if r["c_b"] < 0 or pd.isna(r["d_b"]):
                continue
            if c_prev is not None and abs(r["c_b"] - c_prev) > 5:
                xs.append(None); ys.append(None); textos.append("")
            xs.append(r["c_b"]); ys.append(r["d_b"])
            textos.append(f"frame {int(r['frame'])} · celda {int(r['c_b'])} · "
                          f"d={r['d_b']:.1f} px · pwm={int(r['pwm'])}")
            c_prev = r["c_b"]
        derr = fr[fr["derrapando"]]
        visible = v == vueltas[0]
        fig.add_trace(
            go.Scatter(
                x=xs, y=ys, mode="lines+markers", name=f"vuelta {v}",
                line=dict(color=COL_SERIE_1, width=1.5), marker=dict(size=4),
                text=textos, hovertemplate="%{text}<extra></extra>",
                visible=visible,
            )
        )
        fig.add_trace(
            go.Scatter(
                x=derr["c_b"], y=derr["d_b"], mode="markers",
                name="derrape abierto",
                marker=dict(symbol="x", size=7, color=COL_CRITICO),
                text=[f"frame {int(f)}" for f in derr["frame"]],
                hovertemplate="derrapando · %{text}<br>celda=%{x} "
                              "d=%{y:.1f}<extra></extra>",
                visible=visible,
            )
        )
    fig.add_hline(y=umbral, line=dict(color=COL_TINTA, width=1),
                  annotation_text=f"umbral_derrape = {umbral:.0f} px",
                  annotation_font_color=COL_TINTA_2)

    # Slider: la traza 0 (fondo gris) siempre visible; por cada vuelta se
    # activan sus dos trazas (línea azul + aspas rojas)
    pasos = []
    for i, v in enumerate(vueltas):
        visibles = [True] + [False] * (2 * len(vueltas))
        visibles[1 + 2 * i] = visibles[2 + 2 * i] = True
        pasos.append(dict(label=str(v), method="update",
                          args=[{"visible": visibles}]))
    fig.update_layout(
        sliders=[dict(active=0, currentvalue=dict(prefix="Vuelta "),
                      pad=dict(t=30), steps=pasos)],
    )
    fig.update_xaxes(title_text="celda de la pegatina trasera")
    fig.update_yaxes(title_text="distancia perpendicular (px)",
                     rangemode="tozero")
    return _layout_base(
        fig,
        "4a · Distancia perpendicular de la pegatina trasera a la trayectoria, "
        "vuelta a vuelta",
        520,
    )


# ===========================================================================
# FIGURA 4b: resumen por vuelta (4 paneles)
# ===========================================================================
#   1. nº de derrapes por vuelta (barras rojas: son el evento "malo")
#   2. tiempo por vuelta (marcador verde = al cerrarla el perfil subió,
#      gris = no subió; la última vuelta no tiene cierre y no aparece)
#   3. PWM medio: del perfil (toda la cadena) y el realmente aplicado
#      en los frames de la vuelta (donde pasó el coche)
#   4. velocidad media medida (px/s, filtrando saltos y huecos)
# ===========================================================================
def grafica_4b_resumen(tabla: pd.DataFrame):
    fig = make_subplots(
        rows=2, cols=2, vertical_spacing=0.16, horizontal_spacing=0.10,
        subplot_titles=[
            "derrapes por vuelta", "tiempo por vuelta (s)",
            "PWM medio", "velocidad media (px/s)",
        ],
    )
    fig.add_trace(
        # Los paneles con una sola serie no necesitan entrada de leyenda (el
        # título del panel ya la nombra): solo se listan las dos series de PWM
        go.Bar(x=tabla["vuelta"], y=tabla["n_derrapes"], name="derrapes",
               marker_color=COL_CRITICO, showlegend=False,
               hovertemplate="v%{x}: %{y} derrapes<extra></extra>"),
        row=1, col=1,
    )
    colores = [COL_BUENO if s else COL_MUTED for s in tabla["sube"].fillna(False)]
    fig.add_trace(
        go.Scatter(
            x=tabla["vuelta"], y=tabla["duracion"], mode="lines+markers",
            name="tiempo de vuelta", showlegend=False,
            line=dict(color=COL_SERIE_1, width=2),
            marker=dict(size=8, color=colores),
            text=["perfil subió al cerrarla" if s else "el perfil no subió"
                  for s in tabla["sube"].fillna(False)],
            hovertemplate="v%{x}: %{y:.2f} s<br>%{text}<extra></extra>",
        ),
        row=1, col=2,
    )
    fig.add_trace(
        go.Scatter(x=tabla["vuelta"], y=tabla["pwm_perfil_medio"],
                   mode="lines+markers", name="perfil (media de celdas)",
                   line=dict(color=COL_SERIE_2, width=2), marker=dict(size=6),
                   hovertemplate="v%{x}: %{y:.1f}<extra>perfil</extra>"),
        row=2, col=1,
    )
    fig.add_trace(
        go.Scatter(x=tabla["vuelta"], y=tabla["pwm_aplicado_medio"],
                   mode="lines+markers", name="aplicado en carrera",
                   line=dict(color=COL_SERIE_1, width=2), marker=dict(size=6),
                   hovertemplate="v%{x}: %{y:.1f}<extra>aplicado</extra>"),
        row=2, col=1,
    )
    fig.add_trace(
        go.Scatter(x=tabla["vuelta"], y=tabla["vel_media"],
                   mode="lines+markers", name="velocidad media",
                   showlegend=False,
                   line=dict(color=COL_SERIE_1, width=2), marker=dict(size=6),
                   hovertemplate="v%{x}: %{y:.0f} px/s<extra></extra>"),
        row=2, col=2,
    )
    for fila, col in [(1, 1), (1, 2), (2, 1), (2, 2)]:
        fig.update_xaxes(title_text="vuelta", dtick=5, row=fila, col=col)
    fig.update_yaxes(rangemode="tozero", row=1, col=1)
    fig = _layout_base(fig, "4b · Resumen por vuelta", 640)
    # La leyenda (solo las dos series de PWM) centrada entre los títulos de
    # los dos paneles de arriba, que están a x~0.22 y x~0.78
    fig.update_layout(legend=dict(x=0.5, xanchor="center"))
    return fig


# ---------------------------------------------------------------------------
# Ensamblado del HTML y resumen por consola
# ---------------------------------------------------------------------------
def seccion(titulo, parrafo, fig, primera=False):
    """Una sección del HTML: título, párrafo explicativo y la figura plotly.
    Solo la primera figura embebe plotly.js (≈3 MB); el resto lo reutiliza."""
    cuerpo = fig.to_html(
        full_html=False, include_plotlyjs=primera,
        config={"displaylogo": False, "responsive": True},
    )
    return (
        f'<section><h2>{html.escape(titulo)}</h2>'
        f'<p>{parrafo}</p>{cuerpo}</section>'
    )


def main():
    if len(sys.argv) != 2:
        sys.exit(f"Uso: {sys.argv[0]} <fichero_log>")
    ruta = Path(sys.argv[1])
    if not ruta.is_file():
        sys.exit(f"ERROR: no existe {ruta}")

    print(f"Leyendo {ruta} ...")
    c = parsear_log(ruta)
    if c.frames.empty:
        sys.exit("ERROR: el log no contiene líneas [FRAME] válidas. ¿Es el "
                 "formato de cadena de celdas (front=(x, y)->c=N ...)? Para "
                 "logs del formato antiguo por tramos/arco usar la versión "
                 "anterior del script (git).")

    vueltas, perfil_vuelta, zonas_ini_vuelta, zonas_fin_vuelta = derivar_por_vuelta(c)
    tabla = tabla_vueltas(c, vueltas, perfil_vuelta)
    avisos = detectar_anomalias(c, zonas_fin_vuelta, vueltas)

    n_gigantes = int(c.celdas["gigante"].sum()) if c.celdas is not None else 0

    # --- Resumen por consola -------------------------------------------------
    print(f"  fecha del log: {c.fecha or '(sin fecha)'}")
    print(f"  parámetros [INIT]: {c.params}")
    print(f"  cadena: {c.n_celdas} celdas ({n_gigantes} gigantes), "
          f"{c.long_total:.0f} px, cerrada={c.cerrada}")
    print(f"  frames válidos: {len(c.frames)}  (descartados: {len(c.descartados)})")
    print(f"  vueltas con datos: {len(vueltas)} "
          f"({vueltas[0]}..{vueltas[-1]})")
    print(f"  derrapes cerrados: {len(c.derrapes)}  "
          f"(ignorados por zona muerta: {len(c.zona_muerta)}, "
          f"por localización incoherente: {len(c.incoherentes)})")
    print(f"  derrames a la precedente: {len(c.derrames)}  "
          f"reducciones externas: {len(c.externas)}")
    print(f"  zonas al final: {len(zonas_fin_vuelta[vueltas[-1]])}")
    if c.no_reconocidas:
        print(f"  AVISO: {len(c.no_reconocidas)} líneas no reconocidas "
              f"(¿cambió el formato del log?); primeras:")
        for n, texto in c.no_reconocidas[:5]:
            print(f"    línea {n}: {texto[:100]}")
    print()
    if avisos:
        print(f"ANOMALÍAS ({len(avisos)}):")
        for a in avisos:
            print(f"  - {a}")
    else:
        print("Sin anomalías detectadas.")

    # --- Figuras -------------------------------------------------------------
    print("\nGenerando figuras ...")
    fig1 = grafica_1_zonas(c, vueltas, zonas_fin_vuelta)
    fig2 = grafica_2_heatmap(c, vueltas, perfil_vuelta)
    fig3 = grafica_3_mapa(c, vueltas, perfil_vuelta, zonas_ini_vuelta)
    fig4a = grafica_4a_dist(c, vueltas)
    fig4b = grafica_4b_resumen(tabla)

    # --- HTML ----------------------------------------------------------------
    lista_avisos = "".join(f"<li>{html.escape(a)}</li>" for a in avisos) or \
        "<li>Sin anomalías detectadas.</li>"
    zonas_finales = "".join(
        f"<li>[{z['ini']}, {z['fin']}] {z['tipo']} · pwm {z['pwm']} · "
        f"{z['fin'] - z['ini'] + 1} celdas "
        f"({100 * (z['fin'] - z['ini'] + 1) / max(c.n_celdas, 1):.0f} % de la "
        f"cadena)</li>"
        for z in zonas_fin_vuelta[vueltas[-1]]
    ) or "<li>ninguna</li>"

    resumen_html = f"""
<section>
<h2>Resumen de la carrera</h2>
<p>Log: <code>{html.escape(ruta.name)}</code> · {html.escape(c.fecha)} ·
{len(vueltas)} vueltas · {len(c.frames)} frames válidos
({len(c.descartados)} descartados) · {len(c.derrapes)} derrapes ·
v_min={c.params.get('v_min', float('nan')):.0f}
v_max={c.params.get('v_max', float('nan')):.0f}
umbral_derrape={c.params.get('umbral_derrape', float('nan')):.0f} px ·
cadena de {c.n_celdas} celdas ({n_gigantes} gigantes,
{'cerrada' if c.cerrada else 'abierta'}, {c.long_total:.0f} px).</p>
<h3>Zonas al final de la carrera</h3>
<ul>{zonas_finales}</ul>
<h3>Anomalías detectadas</h3>
<ul class="avisos">{lista_avisos}</ul>
</section>
"""

    partes = [
        resumen_html,
        seccion(
            "1 · Evolución de las zonas de derrape",
            "Cada fila es una vuelta (la primera arriba, se lee hacia abajo). "
            "Las barras naranjas son las zonas de derrape <em>tal como quedan "
            "al terminar esa vuelta</em>, con su PWM en el hover. Debajo de "
            "cada fila, en rojo, los derrapes individuales de esa vuelta "
            "(rombo = 1 sola celda) y las estrellas marcan fusiones. Los "
            "círculos grises son aperturas descartadas por la zona muerta; "
            "los triángulos ámbar, derrames de reducción entre cámaras; la "
            "banda ámbar vertical, una celda gigante (hueco tapado).",
            fig1, primera=True,
        ),
        seccion(
            "2 · Perfil de PWM vuelta a vuelta",
            "Cada fila es el perfil con el que el coche corrió esa vuelta, "
            "expandiendo cada zona a sus celdas (claro = lento, oscuro = "
            "rápido). Las aspas rojas marcan celdas castigadas durante la "
            "vuelta (su efecto se ve en la fila siguiente; si la zona ya "
            "estaba en v_min el castigo no cambia nada) y los círculos "
            "grises las celdas protegidas al empezarla (no subieron aunque "
            "la vuelta anterior fuese limpia).",
            fig2,
        ),
        seccion(
            "3 · El circuito físico",
            "Las celdas de la cadena en píxeles de la cámara (mismo encuadre "
            "que la imagen real). El color es el PWM de la zona de cada celda "
            "en la vuelta del slider; el anillo rojo marca las celdas dentro "
            "de una zona de derrape vigente. La línea ámbar discontinua es "
            "una celda gigante: un hueco tapado donde el circuito no se ve.",
            fig3,
        ),
        seccion(
            "4a · Detección de derrape sobre la trayectoria",
            "La distancia perpendicular de la pegatina trasera en función de "
            "su celda, la serie que dispara la máquina de derrapes. En gris "
            "todas las vueltas (contexto), en azul la vuelta del slider y en "
            "rojo sus frames con derrape abierto. La línea negra es el umbral.",
            fig4a,
        ),
        seccion(
            "4b · Resumen por vuelta",
            "Los derrapes, el tiempo de vuelta (verde = al cerrarla el perfil "
            "subió), el PWM medio (perfil completo frente al aplicado donde "
            "pasó el coche) y la velocidad media medida sobre la trayectoria.",
            fig4b,
        ),
    ]

    pagina = f"""<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="utf-8">
<title>Análisis {html.escape(ruta.stem)}</title>
<style>
  :root {{ color-scheme: light; }}
  body {{
    font-family: {FUENTE};
    background: {COL_SUPERFICIE}; color: {COL_TINTA};
    max-width: 1250px; margin: 0 auto; padding: 24px;
  }}
  h1 {{ font-size: 22px; }}
  h2 {{ font-size: 17px; margin-top: 40px; border-bottom: 1px solid {COL_GRID};
       padding-bottom: 6px; }}
  h3 {{ font-size: 14px; color: {COL_TINTA_2}; }}
  p  {{ color: {COL_TINTA_2}; font-size: 13px; line-height: 1.5; }}
  ul {{ color: {COL_TINTA_2}; font-size: 13px; }}
  ul.avisos li {{ margin-bottom: 6px; }}
  code {{ background: {COL_GRID}; padding: 1px 5px; border-radius: 3px; }}
</style>
</head>
<body>
<h1>Análisis del algoritmo · {html.escape(ruta.name)}</h1>
{''.join(partes)}
</body>
</html>
"""
    salida = ruta.with_name(ruta.stem + "_analisis.html")
    salida.write_text(pagina, encoding="utf-8")
    print(f"  -> {salida}  ({salida.stat().st_size / 1e6:.1f} MB)")


if __name__ == "__main__":
    main()
