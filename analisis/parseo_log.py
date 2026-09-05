#!/usr/bin/env python3
"""
Parser del log de EstrategiaPerfil (derrapesLog_<nodo>_<camara>.txt).

Las regex son copias de los f-string de AlgoritmoVelocidad.py: si se cambia
un mensaje alli, hay que retocar aqui la regex correspondiente. 

Contenido:
  - Una regex por tipo de linea del log.
  - Carrera: contenedor de todo lo extraido de un log (una camara).
  - parsear_log(): una pasada por el fichero -> Carrera.
  - derivar_por_vuelta() / tabla_vueltas(): estado del algoritmo por vuelta.
  - camara_del_log(): nombre de la camara desde la linea [INIT] del fichero
"""

import re
from pathlib import Path

import numpy as np
import pandas as pd

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
# pegatina trasera se localizo en una celda incompatible con la de la delantera 
# el coche estaria en dos sitios a la vez
# regex aparte y no una ampliacion del anterior a proposito: asi los logs grabados
# antes de esta comprobacion se siguen analizando exactamente igual
RE_DERRAPE_INCOHERENTE = re.compile(
    r"\[DERRAPE\] Frame (\d+) v=(\d+): dist=([\d.]+) > umbral pero IGNORADO "
    r"por localización incoherente: la trasera cae en la celda (\d+) y la "
    r"delantera en la (\d+), que están a ([\d.]+) px, pero las pegatinas "
    r"están a ([\d.]+) px"
)
# Cierres "especiales": el coche salio de la ruta / llego al final de la
# cadena / el controlador aviso de la perdida de vision
RE_DERRAPE_CIERRE_VISION = re.compile(
    r"\[DERRAPE\] Frame (\d+) v=(\d+): el (coche salió de la trayectoria|"
    r"coche llegó al final de la cadena abierta|controlador avisa)"
)

# Decision al registrar un derrape: zona nueva (retroceso largo) o fusion
# (retroceso corto), con el arranque del retroceso
RE_ZONA_DECISION = re.compile(
    r"\[ZONA\] Frame (\d+) vuelta (\d+): derrape en \[(\d+), (\d+)\] -> "
    r"(zona nueva|fusión con (\d+) zona\(s\) existente\(s\)): retroceso "
    r"(?:corto )?de ([\d.]+) px desde la celda (\d+) "
    r"\(derrapes_contador=(\d+)\)"
)
# Carveo resultante en la particion (una linea por tramo contiguo castigado)
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

# Lineas conocidas que no aportan datos al analisis 
RE_IGNORABLES = [
    re.compile(r"\[INIT\] "),  
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
    """Contenedor con todo lo extraido del log

    Atributos rellenados por parsear_log():
      camara           camara dueña del log
      fecha            texto ISO de la cabecera del log (o "")
      params           dict {nombre: float} con los [INIT] (v_min, v_max...)
      celdas           DataFrame [celda, x, y, gigante, long_px, acum]
                       (las gigantes llevan x=y=NaN: no hay punto dentro)
      n_celdas         nº total de celdas de la cadena
      long_total       longitud de la cadena en px
      cerrada          bool (cadena cerrada o abierta)
      frames           DataFrame con una fila por [FRAME] valido
      descartados      lista de (frame, vuelta, motivo)
      derrapes         DataFrame [t, linea, frame, vuelta, ini, fin, n_celdas]
      aperturas        lista de dicts de eventos ABIERTO
      zona_muerta      lista de dicts (aperturas rechazadas por zona muerta)
      incoherentes     lista de dicts (aperturas rechazadas porque las dos
                       pegatinas se localizaron en celdas incompatibles)
      cierres_vision   lista de (frame, vuelta) de derrapes cerrados por
                       salir del campo de vision / fin de cadena
      decisiones       lista de dicts [ZONA] decision (nueva/fusion+retroceso)
      carveos          lista de dicts de carveos en la particion (con pwm)
      gigante_castigos lista de {linea, vuelta, celda, antes, despues}
      derrames         lista de {linea, vuelta, px, t_abs} (reduccion_pendiente)
      externas         lista de {linea, vuelta, px, t_abs} (reduccion de otra
                       camara). t_abs = segundos desde medianoche, el unico
                       reloj comun entre los logs de dos camaras
      estados_zona     lista de {linea, vuelta, zonas: [dict]} (volcados [ZONA])
      snapshots_perfil lista de {linea, vuelta, motivo, zonas: [dict]}
      vueltas_reg      lista de dicts de los bloques [VUELTA]
      protecciones     lista de {vuelta, ini, fin, tipo, ultimo_derrape}
      no_reconocidas   lista de (nº linea, texto) que no caso con ningun patron
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
    """Una pasada por el log casando cada linea contra su regex.

    El log es secuencial y los mensajes multilinea ([ZONA] estado y [PERFIL]
    volcado) van siempre seguidos: se abren con su cabecera y las lineas
    siguientes que casen con el patron "item" se añaden al ultimo registro
    abierto. Los timestamps HH:MM:SS.mmm se convierten a segundos desde la
    primera linea"""

    c = Carrera()
    filas_frame = []
    filas_derrape = []
    filas_celda = []

    t0 = None          # segundos absolutos de la primera linea con hora
    t_prev = 0.0       # ultimo t relativo visto
    ajuste_dia = 0.0   # se suma 86400 cada vez que la hora "retrocede"
    vuelta_actual = 0  # contexto de vuelta para snapshots/castigos

    # Registros multilinea abiertos (None cuando no hay ninguno en curso)
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
        if t < t_prev - 43200:  # la hora retrocedio >12 h: cruzo medianoche
            ajuste_dia += 86400.0
            t += 86400.0
        t_prev = t

        # Los [INIT] llevan pares clave=valor sueltos: se acumulan todos en el
        # dict de parametros (v_min, v_max, umbral_derrape, paso_celda...)
        if cuerpo.startswith("[INIT]"):
            for clave, valor in re.findall(r"(\w+)=(-?[\d.]+)", cuerpo):
                c.params[clave] = float(valor)
            continue

        # --- continuaciones de mensajes multilinea (van antes que el resto
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
            estado_abierto = None  # primera linea que no es item: se cierra
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
                continue  # la linea perfil=[[...]] es redundante con los items
            snapshot_abierto = None

        # --- [FRAME]: el tipo mas frecuente, se prueba primero
        m = RE_FRAME.search(cuerpo)
        if m:
            g = m.groups()
            # Indices de grupo (0-based): 0 frame, 1 vuelta, 2-3 front x/y,
            # 4 c_f, 5 d_f, 6-9 back x/y/c/d (None si no se localizo),
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
                    "sube": None,  # se rellena con la linea siguiente
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

    c.camara = camara_del_log(ruta)
    return c



def zonas_a_celdas(zonas, n_celdas):
    """Expande una particion de zonas a un array de PWM por celda (NaN donde
    ninguna zona cubra, que no deberia pasar)"""
    arr = np.full(n_celdas, np.nan)
    for z in zonas:
        arr[z["ini"]: z["fin"] + 1] = z["pwm"]
    return arr



def celdas_de_zona(zona, n_celdas):
    """Celdas de una zona en orden de recorrido.

    Si fin < ini la zona envuelve la meta, solo en cadena cerrada
    Se recorre del ini al final de la cadena y luego del 0
    al fin"""

    if zona["fin"] >= zona["ini"]:
        return list(range(zona["ini"], zona["fin"] + 1))
    return list(range(zona["ini"], n_celdas)) + list(range(0, zona["fin"] + 1))



def unir_por_la_meta(zonas, n_celdas, cerrada):
    """Funde en una las dos zonas de los extremos de la particion cuando en la
    pista son la misma, separadas solo por la linea de meta.

    [0-46] libre pwm=91, [47-68] derrape pwm=89 y [69-88] libre pwm=91

    Tres zonas para lo que en la pista son dos. 
    Aqui se deshace ese corte para dibujarlo como es.

    Se unen solo si la primera empieza en la celda 0, la ultima acaba en la
    n-1 y las dos comparten tipo y pwm

    La zona unida va con fin < ini y con cruza_meta=True"""
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
    # La unida se queda al final: la lista sigue ordenada por ini
    return list(zonas[1:-1]) + [unida]


def seguir_zonas(vueltas, zonas_ini_vuelta, n_celdas, cerrada):
    """Para cada vuelta añade informacion extra a las zonas. Un id que no
    cambia mientras la zona viva, y que por tanto permite seguirla de una
    vuelta a otra, y un delta con lo que vario su PWM respecto a la vuelta
    anterior. Devuelve un diccionario {vuelta: [zona]}, siendo zona el
    diccionario del log (4 campos) +2 nuevos.

    Hace falta porque el log no numera las zonas, cada escritura de [PERFIL] 
    es una particion nueva y completa de la cadena de celdas. Sin este seguimiento 
    la grafica 4 no puede dar a cada tramo un color estable y no se ve nacer ni morir 
    a los tramos.

    Se usan copias y no los originales porque el mismo [PERFIL] lo comparten varias vueltas
    cuando no se produce un cambio entre ellas. Envitando mezclar datos entre vueltas 
    """
    seguidas = {}
    previas = []       
    siguiente_id = 0
    for v in vueltas:
        # Hace falta para deshacer el corte que provoca la meta
        actuales = unir_por_la_meta([dict(z) for z in zonas_ini_vuelta.get(v, [])], n_celdas, cerrada)

        celdas_previas = [set(celdas_de_zona(z, n_celdas)) for z in previas]
        celdas_actuales = [set(celdas_de_zona(z, n_celdas)) for z in actuales]
        pares = []

        # Se comparan las zonas viejas con las nuevas y se guardan las parejas que comparten celdas
        for i, vieja in enumerate(previas):
            for j, nueva in enumerate(actuales):
                solape = len(celdas_previas[i] & celdas_actuales[j])
                if solape > 0:
                    # solape va en negativo para que sorted() ponga primero el que mas comparte 
                    pares.append((-solape, vieja["ini"], nueva["ini"], i, j))

        casadas_viejas, casadas_nuevas = set(), set()

        # Se asocian los ids de las zonas viejas a las zonas nuevas
        for _, _, _, i, j in sorted(pares):
            if i in casadas_viejas or j in casadas_nuevas:
                continue
            casadas_viejas.add(i)
            casadas_nuevas.add(j)
            actuales[j]["id"] = previas[i]["id"]
            actuales[j]["delta"] = actuales[j]["pwm"] - previas[i]["pwm"]

        # Se crean ids nuevos para las zonas que se quedaron sin pareja
        for j, nueva in enumerate(actuales):
            if j not in casadas_nuevas:
                nueva["id"] = siguiente_id
                nueva["delta"] = None
                siguiente_id += 1

        seguidas[v] = actuales
        previas = actuales

    return seguidas


# Segundos de margen para dar por simultaneos un "Derrame" y la "Reduccion externa" que provoca
TOLERANCIA_ENLACE_S = 0.5
# Margen en px para dar por iguales las dos cifras. 
# El algoritmo escribe el mismo float por los dos lados
TOLERANCIA_ENLACE_PX = 0.5
def enlazar_zonas_entre_camaras(datos_camaras,
                                tol_px=TOLERANCIA_ENLACE_PX,
                                tol_s=TOLERANCIA_ENLACE_S):
    """Zonas que son la misma zona de derrape partida entre dos camaras.

    Devuelve {(camara, id_zona): clave_de_grupo}. La grafica 4 usa esa clave 
    para repartir el estilo, de modo que las dos mitades salen del mismo color 
    y la misma forma

    El log deja las partes del evento:
    Quien se queda sin trayectoria al retroceder escribe "[ZONA] Derrame: ... N px" 
    Quien va despues escribe "[ZONA] Reduccion externa: ... N px ...". 
    Se unen por los px y por la marca de tiempo 

    La zona que nace en la vuelta V aparece en la particion del inicio de V+1,
    asi que se busca en la primera vuelta posterior de la que haya datos.

    Los grupos se cierran con union-find para que un derrame en cascada
    (A -> B -> C, cuando tampoco cabe en B) deje las tres zonas en el mismo
    grupo. La clave del grupo es el menor de sus (camara, id) 
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
            mayor, menor = max(ra, rb), min(ra, rb)
            padre[mayor] = menor

    def zona_en_celda(cam, vuelta_evento, celda):
        """(cam, id) de la zona que cubre celda en la primera particion
        posterior al evento, o None si no hay ninguna"""
        d = datos_camaras[cam]
        posteriores = [v for v in d["vueltas"] if v > vuelta_evento]
        if not posteriores:
            return None
        n = d["c"].n_celdas
        for z in d["zonas_ini_vuelta"].get(min(posteriores), []):
            if celda in celdas_de_zona(z, n):
                return (cam, z["id"])
        return None

    # --- candidatos (derrame de A, externa de B) ordenados por cercania ---
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
                    # Por si la sesion cruza medianoche: t_abs se reinicia a 0
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
        # Emisora: la zona pegada al inicio. Receptora: la pegada al final
        za = zona_en_celda(cam_a, der["vuelta"], 0)
        zb = zona_en_celda(
            cam_b, ext["vuelta"], max(datos_camaras[cam_b]["c"].n_celdas - 1, 0))
        usados_der.add((cam_a, i))
        usados_ext.add((cam_b, j))
        if za is None or zb is None:
            # El evento cayo en la ultima vuelta del log y no llego a haber una
            # particion posterior donde mirar no se puede enlazar
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

    Devuelve (vueltas, perfil_vuelta, zonas_ini_vuelta):
      vueltas          lista ordenada de numeros de vuelta con frames
      perfil_vuelta    {v: array de PWM por celda} = perfil con el que se
                       corrio la vuelta v (ultimo volcado [PERFIL] anterior a
                       su primer frame)
      zonas_ini_vuelta {v: [zona]} diccionario que devuelve seguir_zonas()
    """
    vueltas = sorted(c.frames["vuelta"].unique().tolist())
    primera_linea = c.frames.groupby("vuelta")["linea"].min().to_dict()

    perfil_vuelta = {}
    zonas_ini_vuelta = {}
    for v in vueltas:
        # El perfil de la vuelta v es el ultimo volcado [PERFIL] anterior a su
        # primer frame: con ese es con el que el coche corrio la vuelta
        candidatos = [s for s in c.snapshots_perfil if s["linea"] < primera_linea[v]]
        if candidatos:
            perfil_vuelta[v] = zonas_a_celdas(candidatos[-1]["zonas"], c.n_celdas)
            zonas_ini_vuelta[v] = candidatos[-1]["zonas"]
        else:
            perfil_vuelta[v] = np.full(c.n_celdas, np.nan)
            zonas_ini_vuelta[v] = []
    return (vueltas, perfil_vuelta,
            seguir_zonas(vueltas, zonas_ini_vuelta, c.n_celdas, c.cerrada))


def tabla_vueltas(c: Carrera, vueltas, perfil_vuelta) -> pd.DataFrame:
    """Tabla resumen con una fila por vuelta: derrapes, si el perfil subio al
    terminarla, duracion, PWM medio (del perfil y el realmente aplicado) y
    velocidad media medida (celdas/s convertidas a px/s)"""
    paso = c.params.get("paso_celda", 30.0)
    # El bloque [VUELTA v] se emite al cruzar meta: cierra la vuelta v-1 y abre
    # la v. Por eso los datos "de cierre" de la vuelta v salen de [VUELTA v+1]
    reg_por_vuelta = {r["vuelta"]: r for r in c.vueltas_reg}
    filas = []
    for v in vueltas:
        fr = c.frames[c.frames["vuelta"] == v]
        cierre = reg_por_vuelta.get(v + 1)

        apertura = reg_por_vuelta.get(v)
        t_ini = apertura["t"] if apertura else fr["t"].min()
        duracion = (cierre["t"] - t_ini) if cierre else np.nan

        # Velocidad media: avances de celda entre frames consecutivos por el
        # paso de celda, filtrando saltos enormes y huecos temporales
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


def camara_del_log(ruta: Path) -> str:
    """Nombre de la camara de un log, leido de su linea "[INIT] nodo=...
    camara=camara_01" (los pares no numericos no entran en Carrera.params).
    Si no aparece se usa el nombre del fichero, que siempre lo lleva detras
    del ultimo guion bajo (derrapesLog_<nodo>_<camara>.txt)"""
    with open(ruta, encoding="utf-8") as f:
        for linea in f:
            m = re.search(r"\[INIT\] nodo=\S+ camara=(\S+)", linea)
            if m:
                return m.group(1)
    return ruta.stem.split("_")[-1]
