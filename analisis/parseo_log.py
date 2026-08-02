#!/usr/bin/env python3
"""
Parser del log de EstrategiaPerfil (derrapesLog_<nodo>_<camara>.txt, formato
de CADENA DE CELDAS + ZONAS con etiquetas [INIT]/[TRAY]/[FRAME]/[DERRAPE]/
[ZONA]/[PERFIL]/[VUELTA]).

Código movido TAL CUAL de analizar_log_algoritmo.py al unificar el análisis
en el dashboard (analisis.py): las regex son copias de los f-string de
AlgoritmoVelocidad.py; si se cambia un mensaje allí hay que retocar aquí la
regex correspondiente.

Contenido:
  - Umbrales del detector de anomalías (constantes bajo esta cabecera).
  - Una regex por tipo de línea del log.
  - Carrera: contenedor de todo lo extraído de UN log (una cámara).
  - parsear_log(): una pasada por el fichero -> Carrera.
  - derivar_por_vuelta() / tabla_vueltas(): estado del algoritmo por vuelta.
  - detectar_anomalias(): comprobaciones automáticas.
  - camara_del_log(): nombre de la cámara desde la línea [INIT] del fichero.
"""

import re
from pathlib import Path

import numpy as np
import pandas as pd

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
      camara           cámara dueña del log (de su [INIT]); los números de
                       frame son de su contador y no se comparan con los de
                       otra cámara (ver NUMERACION_FRAMES.md)
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
                       cámara). t_abs = segundos desde medianoche, el único
                       reloj común entre los logs de dos cámaras
      estados_zona     lista de {linea, vuelta, zonas: [dict]} (volcados [ZONA])
      snapshots_perfil lista de {linea, vuelta, motivo, zonas: [dict]}
      vueltas_reg      lista de dicts de los bloques [VUELTA]
      protecciones     lista de {vuelta, ini, fin, tipo, ultimo_derrape}
      no_reconocidas   lista de (nº línea, texto) que no casó con ningún patrón
    """

    def __init__(self):
        self.camara = ""
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
        # Las dos caras del MISMO evento: la cámara que se queda sin
        # trayectoria al retroceder apunta un "Derrame" y la precedente, en el
        # mismo instante, una "Reducción externa". Se guarda `t_abs` (segundos
        # desde medianoche) y no el `t` relativo que usa el resto del parser,
        # porque el `t` va referido al t0 de SU log y cada cámara tiene el suyo:
        # para casar las dos caras hace falta un reloj común, y lo es porque los
        # dos logs los escribe el mismo proceso (ver enlazar_zonas_entre_camaras)
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
    # Los números de frame del log son de ESTA cámara y solo tienen sentido
    # dentro de su escala, así que la cámara acompaña al número en todos los
    # textos que este módulo genera (avisos de anomalías, etiquetas)
    c.camara = camara_del_log(ruta)
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


def celdas_de_zona(zona, n_celdas):
    """Celdas de una zona en orden de recorrido.

    Si `fin` < `ini` la zona ENVUELVE la meta (solo pasa en cadena cerrada, ver
    unir_por_la_meta) y se recorre del `ini` al final de la cadena y luego del 0
    al `fin`. Es el mismo criterio que `_intervalo_a_celdas()` del algoritmo."""
    if zona["fin"] >= zona["ini"]:
        return list(range(zona["ini"], zona["fin"] + 1))
    return list(range(zona["ini"], n_celdas)) + list(range(0, zona["fin"] + 1))


def unir_por_la_meta(zonas, n_celdas, cerrada):
    """Funde en UNA las dos zonas de los extremos de la partición cuando en la
    pista son la misma, separadas solo por la línea de meta.

    En cadena cerrada la celda n-1 es vecina de la 0, pero la partición del
    algoritmo es una lista plana de intervalos [ini, fin] sin envoltura: cuando
    un castigo cae en medio de la zona libre, `_carvear_particion()` deja los
    dos trozos que sobresalen como zonas distintas aunque sean el mismo tramo de
    pista. En el log del óvalo se ve tal cual: [0-46] libre pwm=91, [47-68]
    derrape pwm=89 y [69-88] libre pwm=91, tres zonas para lo que en la pista
    son dos. Aquí se deshace ese corte para DIBUJARLO como es (el algoritmo se
    queda como está: los dos trozos tienen el mismo valor y suben juntos, así
    que el resultado es el mismo).

    Se unen solo si la primera empieza en la celda 0, la última acaba en la
    n-1 y las dos comparten `tipo` y `pwm`: con el mismo valor ya se pintan como
    un bloque continuo del mismo color, así que unirlas es contar lo que se ve.
    Si el PWM difiere son dos zonas de verdad y no se tocan (pasa en el mismo
    log: [0-80] derrape pwm=85 junto a [85-88] derrape pwm=90).

    La zona unida va con `fin` < `ini` (la marca de que envuelve) y con
    `cruza_meta=True`, que es lo que la gráfica 4 usa para decirlo en la
    leyenda. No modifica los dicts de entrada."""
    if not cerrada or len(zonas) < 2:
        return list(zonas)
    primera, ultima = zonas[0], zonas[-1]
    if (primera["ini"] != 0 or ultima["fin"] != n_celdas - 1
            or primera["tipo"] != ultima["tipo"]
            or primera["pwm"] != ultima["pwm"]):
        return list(zonas)
    unida = dict(ultima, fin=primera["fin"], cruza_meta=True)
    # El historial de vueltas con derrape solo lo traen los vuelcos [ZONA]
    if "vueltas" in primera and "vueltas" in ultima:
        unida["vueltas"] = sorted(set(ultima["vueltas"]) | set(primera["vueltas"]))
    # La unida se queda al final: la lista sigue ordenada por `ini`
    return list(zonas[1:-1]) + [unida]


def seguir_zonas(vueltas, zonas_ini_vuelta, n_celdas, cerrada):
    """Sigue la PISTA de cada zona a lo largo de las vueltas: le pone un `id`
    que no cambia mientras la zona viva y el `delta` de PWM respecto a la
    vuelta anterior. Devuelve {v: [zona]} con COPIAS enriquecidas.

    Hace falta porque el log NO numera las zonas: cada volcado [PERFIL] es una
    partición nueva y completa de la cadena de celdas, y la única identidad que
    trae una zona es su par (ini, fin)... que cambia justo cuando la zona crece,
    la carvean o absorbe a la vecina. Sin este seguimiento la gráfica 4 no puede
    dar a cada tramo un color estable y no se ve nacer ni morir a los tramos.

    Antes de emparejar nada se deshace el corte de la meta (unir_por_la_meta),
    para que la identidad se siga sobre la zona ENTERA: si no, el trozo de
    después de la meta nace como zona nueva en cuanto un castigo parte la libre.

    La identidad se deriva EMPAREJANDO las dos particiones por SOLAPE de celdas,
    de forma voraz: se listan todos los pares (zona vieja, zona nueva) que
    comparten alguna celda, se ordenan de más a menos celdas compartidas y se
    casan uno a uno (cada zona solo puede casarse una vez); la nueva hereda el
    id de la vieja con la que más comparte. Las nuevas que se quedan sin pareja
    NACEN (id nuevo, nunca reutilizado) y las viejas sin pareja MUEREN. Esa
    única regla cubre sin casos especiales las tres cosas que hace el algoritmo:
      - castigo/crecimiento (una vieja <-> una nueva): mismo id, delta != 0
      - fusión (dos viejas -> una nueva): gana la que aportaba más celdas, la
        otra muere (y en la gráfica 4 libera su color)
      - carveo/división (una vieja -> dos nuevas): la de mayor solape hereda,
        la otra nace
    Se trabaja con COPIAS (dict(z, ...)) y no tocando los dicts originales
    porque el mismo volcado [PERFIL] lo comparten varias vueltas cuando entre
    ellas no hubo volcado nuevo: escribir dentro mezclaría datos entre vueltas.

    delta: pwm de esta vuelta menos el de la anterior para ESE MISMO id, o None
    si la zona nace aquí (no hay con qué compararla).
    """
    seguidas = {}
    previas = []       # zonas ya enriquecidas de la vuelta anterior
    siguiente_id = 0
    for v in vueltas:
        actuales = unir_por_la_meta(
            [dict(z) for z in zonas_ini_vuelta.get(v, [])], n_celdas, cerrada)
        # El solape se cuenta con CONJUNTOS de celdas y no restando ini/fin: así
        # una zona que envuelve la meta se compara como cualquier otra
        celdas_previas = [set(celdas_de_zona(z, n_celdas)) for z in previas]
        celdas_actuales = [set(celdas_de_zona(z, n_celdas)) for z in actuales]
        # Pares con solape > 0, del que más comparte al que menos. El desempate
        # va por (ini de la vieja, ini de la nueva) para que el resultado no
        # dependa del orden en que python recorra la lista.
        pares = []
        for i, vieja in enumerate(previas):
            for j, nueva in enumerate(actuales):
                solape = len(celdas_previas[i] & celdas_actuales[j])
                if solape > 0:
                    pares.append((-solape, vieja["ini"], nueva["ini"], i, j))
        casadas_viejas, casadas_nuevas = set(), set()
        for _, _, _, i, j in sorted(pares):
            if i in casadas_viejas or j in casadas_nuevas:
                continue
            casadas_viejas.add(i)
            casadas_nuevas.add(j)
            actuales[j]["id"] = previas[i]["id"]
            actuales[j]["delta"] = actuales[j]["pwm"] - previas[i]["pwm"]
        for j, nueva in enumerate(actuales):
            if j not in casadas_nuevas:
                nueva["id"] = siguiente_id
                nueva["delta"] = None
                siguiente_id += 1
        seguidas[v] = actuales
        previas = actuales
    return seguidas


# ---------------------------------------------------------------------------
# Enlace de zonas ENTRE cámaras (derrame)
# ---------------------------------------------------------------------------
# Segundos de margen para dar por simultáneos un "Derrame" y la "Reducción
# externa" que provoca. En los logs de pista van a 1 ms: los escribe el MISMO
# proceso (el controlador), uno justo detrás del otro dentro del mismo callback.
# Medio segundo es holgadísimo y sigue siendo mil veces menor que la separación
# entre dos derrames consecutivos (varios segundos), así que no puede cruzarlos.
TOLERANCIA_ENLACE_S = 0.5
# Margen en px para dar por iguales las dos cifras. El algoritmo escribe el
# mismo float por los dos lados, así que en la práctica coinciden exactas; el
# margen solo cubre el redondeo del texto del log (un decimal).
TOLERANCIA_ENLACE_PX = 0.5


def enlazar_zonas_entre_camaras(datos_camaras,
                                tol_px=TOLERANCIA_ENLACE_PX,
                                tol_s=TOLERANCIA_ENLACE_S):
    """Zonas que son la MISMA zona de derrape partida entre dos cámaras.

    Devuelve {(camara, id_zona): clave_de_grupo}; las zonas sin enlazar NO
    aparecen en el mapa. La gráfica 4 usa esa clave para repartir el estilo, de
    modo que las dos mitades salen del mismo color y la misma forma y se leen
    como lo que son: un único tramo de pista en el que el coche derrapa.

    Hace falta porque `seguir_zonas()` trabaja sobre UNA cámara: sus `id` son
    locales y dos cámaras no comparten ni coordenadas ni numeración de celdas.
    Nadie mira entre cámaras, así que una curva repartida entre dos encuadres
    salía como dos zonas independientes y parecían dos problemas distintos.

    El log NO nombra a la otra cámara, pero sí deja las dos caras del evento:
    quien se queda sin trayectoria al retroceder escribe "[ZONA] Derrame: ... N
    px" y la precedente "[ZONA] Reducción externa: ... N px ...". Se casan por
    los px y por la marca de tiempo (ver TOLERANCIA_ENLACE_S), de forma voraz y
    uno a uno, del par más próximo en el tiempo al más lejano: mismo criterio
    que el emparejamiento por solape de `seguir_zonas()`.

    Sabido el par de cámaras, QUÉ zona es cada extremo lo fija la construcción
    del algoritmo, sin heurística:
      - en la EMISORA, la que toca la celda 0 (el retroceso se salió justo por
        el inicio de la cadena, que es lo que genera el derrame);
      - en la RECEPTORA, la que toca la última celda (`aplicar_reduccion_externa`
        extiende hacia atrás desde el final de su trayectoria).
    La zona que nace en la vuelta V aparece en la partición del inicio de V+1,
    así que se busca en la primera vuelta posterior de la que haya datos.

    Los grupos se cierran con union-find para que un derrame en cascada
    (A -> B -> C, cuando tampoco cabe en B) deje las tres zonas en el mismo
    grupo. La clave del grupo es el menor de sus (camara, id): es estable entre
    ejecuciones y no hace falta un contador nuevo.
    """
    # --- union-find sobre claves (camara, id) ---
    padre = {}

    def raiz(x):
        padre.setdefault(x, x)
        while padre[x] != x:
            padre[x] = padre[padre[x]]
            x = padre[x]
        return x

    def unir(a, b):
        ra, rb = raiz(a), raiz(b)
        if ra != rb:
            # El menor manda, así la clave del grupo no depende del orden en
            # que se hayan ido uniendo los pares
            mayor, menor = max(ra, rb), min(ra, rb)
            padre[mayor] = menor

    def zona_en_celda(cam, vuelta_evento, celda):
        """(cam, id) de la zona que cubre `celda` en la primera partición
        posterior al evento, o None si no hay ninguna."""
        d = datos_camaras[cam]
        posteriores = [v for v in d["vueltas"] if v > vuelta_evento]
        if not posteriores:
            return None
        n = d["c"].n_celdas
        for z in d["zonas_ini_vuelta"].get(min(posteriores), []):
            if celda in celdas_de_zona(z, n):
                return (cam, z["id"])
        return None

    # --- candidatos (derrame de A, externa de B) ordenados por cercanía ---
    candidatos = []
    for cam_a, d_a in datos_camaras.items():
        for cam_b, d_b in datos_camaras.items():
            if cam_a == cam_b:
                continue
            for i, der in enumerate(d_a["c"].derrames):
                for j, ext in enumerate(d_b["c"].externas):
                    if abs(der["px"] - ext["px"]) > tol_px:
                        continue
                    dt = abs(der["t_abs"] - ext["t_abs"])
                    # Por si la sesión cruza medianoche: t_abs se reinicia a 0
                    dt = min(dt, 86400.0 - dt)
                    if dt <= tol_s:
                        candidatos.append((dt, cam_a, i, cam_b, j))

    enlaces = 0
    usados_der, usados_ext = set(), set()
    for _, cam_a, i, cam_b, j in sorted(candidatos):
        if (cam_a, i) in usados_der or (cam_b, j) in usados_ext:
            continue
        der = datos_camaras[cam_a]["c"].derrames[i]
        ext = datos_camaras[cam_b]["c"].externas[j]
        # Emisora: la zona pegada al INICIO. Receptora: la pegada al FINAL
        za = zona_en_celda(cam_a, der["vuelta"], 0)
        zb = zona_en_celda(
            cam_b, ext["vuelta"], max(datos_camaras[cam_b]["c"].n_celdas - 1, 0))
        usados_der.add((cam_a, i))
        usados_ext.add((cam_b, j))
        if za is None or zb is None:
            # El evento cayó en la última vuelta del log y no llegó a haber una
            # partición posterior donde mirar: no se puede enlazar
            continue
        unir(za, zb)
        enlaces += 1

    grupos = {clave: raiz(clave) for clave in padre}
    if grupos:
        print(f"  zonas enlazadas entre cámaras: {enlaces} enlace(s), "
              f"{len(set(grupos.values()))} grupo(s)")
    return grupos


def derivar_por_vuelta(c: Carrera):
    """Reconstruye el estado del algoritmo vuelta a vuelta.

    Devuelve (vueltas, perfil_vuelta, zonas_ini_vuelta, zonas_fin_vuelta):
      vueltas          lista ordenada de números de vuelta con frames
      perfil_vuelta    {v: array de PWM por celda} = perfil CON EL QUE SE
                       CORRIÓ la vuelta v (último volcado [PERFIL] anterior a
                       su primer frame)
      zonas_ini_vuelta {v: [zona]} = partición al EMPEZAR la vuelta v, con el
                       `id` y el `delta` que les pone seguir_zonas()
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
    # Solo la partición de INICIO se sigue entre vueltas: es la que dibuja la
    # gráfica 4 (el perfil con el que se corrió la vuelta). La de fin la usa
    # detectar_anomalias, a la que el id no le aporta nada.
    return (vueltas, perfil_vuelta,
            seguir_zonas(vueltas, zonas_ini_vuelta, c.n_celdas, c.cerrada),
            zonas_fin_vuelta)


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
                f"{int(d['vuelta'])}, {c.camara} frame {int(d['frame'])}: celdas "
                f"[{int(d['ini'])}, {int(d['fin'])}]. Un derrape real dura "
                f"unas pocas celdas: revisar los saltos de celda de la "
                f"trasera en esos frames (siguiente comprobación)."
            )

    # 2) Saltos de celda de la pegatina trasera ENTRE frames consecutivos con
    # el derrape abierto: el mecanismo exacto por el que se estira el intervalo
    fr = c.frames
    dc_b = fr["c_b"].diff().abs()
    # Los números de frame son del contador de ESTA cámara y avanzan de uno en
    # uno mientras publique, así que la diferencia mide de verdad "cuántos
    # fotogramas han pasado". Un hueco significa que la cámara procesó ese
    # fotograma pero no publicó (no vio el coche). Se toleran 2 para no perder
    # el caso de un frame descartado en medio.
    # (Antes el número era un contador global del controlador, con los mensajes
    # de todas las cámaras mezclados: con dos cámaras la diferencia entre
    # frames consecutivos de la misma ya era >= 2 y esta comprobación se
    # quedaba sin casos. Ver NUMERACION_FRAMES.md.)
    consecutivos = fr["frame"].diff() <= 2
    con_derrape = fr["derrapando"] & fr["derrapando"].shift(fill_value=False)
    saltos = fr[(dc_b > SALTO_CELDAS_TRASERA_ANOMALO) & con_derrape & consecutivos]
    for _, s in saltos.iterrows():
        avisos.append(
            f"Salto de celda de la TRASERA con derrape abierto en la vuelta "
            f"{int(s['vuelta'])}, {c.camara} frame {int(s['frame'])}: pasó a la celda "
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


def camara_del_log(ruta: Path) -> str:
    """Nombre de la cámara de un log, leído de su línea "[INIT] nodo=...
    camara=camara_01" (los pares no numéricos no entran en Carrera.params).
    Si no aparece se usa el nombre del fichero, que siempre lo lleva detrás
    del último guion bajo (derrapesLog_<nodo>_<camara>.txt)."""
    with open(ruta, encoding="utf-8") as f:
        for linea in f:
            m = re.search(r"\[INIT\] nodo=\S+ camara=(\S+)", linea)
            if m:
                return m.group(1)
    return ruta.stem.split("_")[-1]
