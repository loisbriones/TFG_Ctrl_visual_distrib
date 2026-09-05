#!/usr/bin/env python3
"""
Se localizan y se leen las dos fuentes: el bag y los logs del algoritmo y se cruzan entre si. 

Salen dos tablas, que son las que alimentan todas las figuras:
  df_pos  una fila por posicion valida del bag, con su vuelta y el
          dist_derrape de la telemetria que origino   -> figuras 1, 2 y 4
  df_tel  una fila por telemetria caida dentro de una vuelta, con el instante
          relativo al cruce de meta, la camara que la vio y el PWM que estaba
          en vigor                                     -> figuras 3 y 5
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd

from lectura_bag import (
    emparejar_telemetria, leer_bag, pwm_en_instantes, repartir_por_vueltas,
)
from parseo_log import (
    camara_del_log, derivar_por_vuelta, parsear_log, tabla_vueltas,
)

# Valores por defecto de EstrategiaPerfil, para cuando no hay ningun log del que leerlos
UMBRAL_DERRAPE_DEF = 8.0
MAX_DIST_RUTA_DEF = 80.0



# ---------------------------------------------------------------------------
# Los logs del algoritmo (uno por camara)
# ---------------------------------------------------------------------------
def buscar_logs(argumentos, carpeta_bag):
    """Convierte los argumentos de logs en una lista de ficheros: cada
    argumento puede ser un .txt o una carpeta donde buscarlos; sin argumentos
    se buscan dentro de la carpeta del bag"""
    if not argumentos:
        return sorted(carpeta_bag.rglob("derrapesLog_*.txt"))
    rutas = []
    for arg in argumentos:
        p = Path(arg)
        if p.is_dir():
            rutas += sorted(p.rglob("derrapesLog_*.txt"))
        elif p.is_file():
            rutas.append(p)
        else:
            sys.exit(f"ERROR: no existe {p}")
    return rutas



def cargar_logs(rutas):
    """Parsea cada log 

    Devuelve {camara: {"c": Carrera, "vueltas": [...], "perfil_vuelta": {...},
    "zonas_ini_vuelta": {...}, "tabla": DataFrame, "ruta": Path}} ordenado por
    nombre de camara. La camara sale de la linea [INIT] del propio fichero, no
    de su nombre"""

    datos = {}
    for ruta in rutas:
        c = parsear_log(ruta)
        if c.frames is None or c.frames.empty:
            print(f"AVISO: {ruta} no contiene líneas [FRAME] válidas se ignora este log")
            continue
        vueltas, perfil_vuelta, zonas_ini = derivar_por_vuelta(c)
        datos[camara_del_log(ruta)] = {
            "c": c,
            "vueltas": vueltas,
            "perfil_vuelta": perfil_vuelta,
            "zonas_ini_vuelta": zonas_ini,
            "tabla": tabla_vueltas(c, vueltas, perfil_vuelta),
            "ruta": ruta,
        }
    return dict(sorted(datos.items()))



def umbrales_del_algoritmo(logs):
    """(umbral_derrape, max_dist_ruta) con los que corrio el algoritmo, leidos
    de la linea [INIT] del primer log que los traiga. Sin logs se usan los
    defaults, para que las figuras del bag sigan pudiendo pintar su umbral"""
    def primero(nombre, defecto):
        return next((d["c"].params[nombre] for d in logs.values()
                     if nombre in d["c"].params), defecto)
    return (primero("umbral_derrape", UMBRAL_DERRAPE_DEF),
            primero("max_dist_ruta", MAX_DIST_RUTA_DEF))



def preparar_bag(bag_dir, coche):
    """Lee el bag y devuelve (bag, df_pos, df_tel)"""
    bag = leer_bag(bag_dir, coche)
    posiciones, telemetria, vueltas = (
        bag["posiciones"], bag["telemetria"], bag["vueltas"])
    print(f"Mensajes: {len(posiciones)} posiciones, {len(telemetria)} "
          f"telemetrías, {len(vueltas)} vueltas cronometradas")
    if vueltas:
        t = [v["tiempo"] for v in vueltas]
        print(f"Tiempos: mejor {min(t):.3f} s | media {sum(t) / len(t):.3f} s "
              f"| peor {max(t):.3f} s")

    # Una posicion con alguna pegatina en (0, 0) no se coge porque no es valida para el analisis
    validas = [p for p in posiciones
               if (p["fx"], p["fy"]) != (0, 0) and (p["bx"], p["by"]) != (0, 0)]
    # La telemetria no dice que posicion la origino se empareja por tiempo de grabacion
    indices_tel = emparejar_telemetria(telemetria, validas)

    df_pos = _tabla_posiciones(validas, telemetria, indices_tel, vueltas)
    df_tel = _tabla_telemetria(telemetria, validas, indices_tel, vueltas,
                               bag["pwm"])
    return bag, df_pos, df_tel



def _tabla_posiciones(validas, telemetria, indices_tel, vueltas):
    """Una fila por posicion valida, con la vuelta a la que pertenece y el
    dist_derrape de su telemetria"""
    vuelta_de_pos = repartir_por_vueltas([p["t"] for p in validas], vueltas)
    dist_de_pos = [np.nan] * len(validas)
    for i_tel, i_pos in enumerate(indices_tel):
        if i_pos is not None:
            dist_de_pos[i_pos] = telemetria[i_tel]["dist"]
    df = pd.DataFrame(validas)
    if len(df):
        df["vuelta"] = vuelta_de_pos
        df["dist"] = dist_de_pos
    n_en_vuelta = sum(1 for v in vuelta_de_pos if v is not None)
    print(f"Posiciones válidas: {len(validas)} ({n_en_vuelta} dentro de "
          f"vueltas); telemetrías emparejadas: "
          f"{sum(1 for i in indices_tel if i is not None)}/{len(telemetria)}")
    return df



def _tabla_telemetria(telemetria, validas, indices_tel, vueltas, pwm):
    """Una fila por telemetria caida dentro de una vuelta. Las que caen en la
    calibracion o entre vueltas se descartan"""
    vuelta_de_tel = repartir_por_vueltas([m["t"] for m in telemetria], vueltas)
    pwm_de_tel = pwm_en_instantes([m["t"] for m in telemetria], pwm)
    # El mensaje time_per_lap se publica al cerrar la vuelta, asi que la vuelta
    # empezo lap_time segundos antes de ese instante
    inicio = {v["numero"]: v["t"] - int(v["tiempo"] * 1e9) for v in vueltas}
    filas = []
    for m, i_pos, v, p in zip(telemetria, indices_tel, vuelta_de_tel, pwm_de_tel):
        if v is None:
            continue
        pos = validas[i_pos] if i_pos is not None else None
        filas.append({
            "vuelta": v,
            "t_vuelta": (m["t"] - inicio[v]) / 1e9,
            "dist": m["dist"],
            "derrapando": m["derrapando"],
            "camara": pos["camara"] if pos else None,
            "bx": pos["bx"] if pos else np.nan,
            "by": pos["by"] if pos else np.nan,
            "pwm": p,
        })
    return pd.DataFrame(filas)



def tabla_derrape_por_vuelta(df_tel, umbral):
    """Resumen por vuelta respecto a dist_derrape: maximo, percentil
    95, mediana, cuantas muestras pasan el umbral, cuantas tenian derrape
    abierto y el PWM medio"""
    filas = []
    for v, g in df_tel.groupby("vuelta"):
        filas.append({
            "vuelta": int(v),
            "n_muestras": len(g),
            "dist_max": g["dist"].max(),
            "dist_p95": g["dist"].quantile(0.95),
            "dist_mediana": g["dist"].median(),
            "n_sobre_umbral": int((g["dist"] > umbral).sum()),
            "n_derrapando": int(g["derrapando"].sum()),
            "pwm_medio": g["pwm"].mean(),
        })
    return pd.DataFrame(filas)
