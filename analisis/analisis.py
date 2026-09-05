#!/usr/bin/env python3
"""
Dashboard de analisis de una carrera: un comando que junta el bag (posiciones,
tiempos por vuelta, telemetria) con el/los logs del algoritmo (derrapesLog_*.txt,
uno por camara) y genera un HTML interactivo que ademas se sirve por web desde
el contenedor.

Uso (dentro del contenedor; ver run.sh, que lo lanza con Docker):

  python3 analisis.py <carpeta_bag> [log1.txt log2.txt ...] [--coche car1]

El HTML se escribe junto a los datos como <nombre_bag>_analisis.html y se sirve
en http://localhost:8988, salvo con --sin-servidor.

Este fichero es solo el guion: que se lee, en que orden salen las secciones y
que se hace con el resultado. El trabajo esta repartido en
  lectura_bag.py + parseo_log.py   leer las dos fuentes
  datos.py                         cruzarlas en los DataFrames de las figuras
  figuras.py                       una funcion por figura
  pagina.py + web/                 el HTML, el estilo y la interactividad
  servidor.py                      servirlo y exportar a PDF
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

import datos
import figuras
import pagina
import servidor
from figuras import PANELES_6
from lectura_bag import encontrar_bag

# Una vuelta se marca como anomala en la tabla de tiempos si supera este
# multiplo de la media (paradas, salidas de pista, relocalizaciones largas)
FACTOR_VUELTA_ANOMALA = 2.0

# {clave: go.Figure} de todo lo que se dibuja: la clave es la que llevan los
# botones de la pagina y la que atiende el endpoint /pdf. Con varias camaras
# se le añade ":<camara>", porque hay una figura de cada por camara
FIGURAS = {}


def estadisticos_tiempos(vueltas):
    """mejor / media / mediana / peor de los tiempos de vuelta del bag"""
    tiempos = sorted(v["tiempo"] for v in vueltas)
    mitad = len(tiempos) // 2
    return {
        "mejor": tiempos[0],
        "peor": tiempos[-1],
        "media": sum(tiempos) / len(tiempos),
        "mediana": (tiempos[mitad] if len(tiempos) % 2
                    else (tiempos[mitad - 1] + tiempos[mitad]) / 2),
        "factor_anomala": FACTOR_VUELTA_ANOMALA,
    }


def marco_de_la_pista(df_pos, logs):
    """El encuadre comun de las graficas 2 y 4: la caja que envuelve todo lo
    que dibujan las dos. Se calcula aqui porque cada figura sola solo conoce la
    mitad de los datos, y asi el circuito sale a la misma escala en ambas"""
    camaras = sorted(
        (set(df_pos["camara"].dropna().unique()) if len(df_pos) else set())
        | set(logs))
    celdas_globales = []
    for i, cam in enumerate(camaras):
        if cam not in logs:
            continue
        celdas = logs[cam]["c"].celdas
        celdas = celdas[~celdas["gigante"]]
        dx, dy = figuras.offset_camara(cam, i)
        celdas_globales.append(np.column_stack([
            celdas["x"].to_numpy(dtype=float) + dx,
            celdas["y"].to_numpy(dtype=float) + dy,
        ]))
    vacio = pd.DataFrame(columns=["camara", "fx", "fy", "bx", "by"])
    return figuras.marco_comun(df_pos if len(df_pos) else vacio,
                               camaras, celdas_globales)


def registrar(clave, fig, camara=None, varias=False):
    """Guarda una figura en FIGURAS y devuelve su clave definitiva"""
    if varias and camara:
        clave = f"{clave}:{camara}"
        fig = pagina.etiquetar_camara(fig, camara, varias)
    FIGURAS[clave] = fig
    return clave


def main():

    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("carpeta_bag", type=Path,
                        help="Carpeta del bag (o que contiene el bag)")
    parser.add_argument("logs", nargs="*",
                        help="Logs del algoritmo (ficheros .txt o carpetas donde "
                             "buscarlos); por defecto se buscan en la carpeta del bag")
    parser.add_argument("--coche", default="car1", help="Nombre del coche (def: car1)")
    parser.add_argument("--sin-servidor", action="store_true",
                        help="Solo generar el HTML, sin servirlo (y sin botones "
                             "de PDF, que necesitan el servidor)")
    args = parser.parse_args()

    carpeta = args.carpeta_bag.resolve()
    if not carpeta.is_dir():
        sys.exit(f"ERROR: {carpeta} no es una carpeta")
    bag_dir = encontrar_bag(carpeta)
    print(f"Bag: {bag_dir}")
    pagina.CON_SERVIDOR = not args.sin_servidor

    # --- Las dos fuentes ---------------------------------------------------
    logs = datos.cargar_logs(datos.buscar_logs(args.logs, carpeta))
    if not logs:
        print("AVISO: sin logs del algoritmo; no saldrán las secciones 4 y 6 ni "
              "la trayectoria base de la 2")
    varias = len(logs) > 1
    for camara, d in logs.items():
        print(f"Log {camara}: {d['ruta'].name} · {len(d['vueltas'])} vueltas · "
              f"{d['c'].n_celdas} celdas · {len(d['c'].derrapes)} derrapes")

    bag, df_pos, df_tel = datos.preparar_bag(bag_dir, args.coche)
    vueltas = bag["vueltas"]
    umbral_derrape, max_dist_ruta = datos.umbrales_del_algoritmo(logs)
    if len(df_tel):
        print(f"Telemetrías dentro de vueltas: {len(df_tel)} · dist máx "
              f"{df_tel['dist'].max():.1f} px · "
              f"{int((df_tel['dist'] > umbral_derrape).sum())} muestras sobre "
              f"el umbral ({umbral_derrape:.0f} px)")

    # --- Las graficas, en el orden en el que salen ------------------------
    print("\nGenerando figuras ...")

    hay_posiciones = len(df_pos) and df_pos["vuelta"].notna().any()
    marco = marco_de_la_pista(df_pos, logs)
    partes = [pagina.seccion_resumen(
        bag_dir.name, len(bag["posiciones"]), len(df_pos), len(vueltas),
        len(bag["telemetria"]), logs)]

    # Solo la primera figura del documento embebe plotly.js; las demas lo
    # reutilizan, y por eso hay que ir contando cual es la primera
    primera = True

    if hay_posiciones:
        registrar("1", figuras.grafica_1_derrape3d(df_pos))
        partes.append(pagina.seccion("1", [FIGURAS["1"]], ["1"], primera=primera))
        primera = False

        # La trayectoria base y las perpendiculares salen de las celdas del
        # log. Sin logs la figura se queda solo con las dos pegatinas
        registrar("2", figuras.grafica_2_trayectorias(
            df_pos,
            celdas_por_camara={cam: d["c"].celdas for cam, d in logs.items()},
            cerradas={cam: d["c"].cerrada for cam, d in logs.items()},
            umbral=umbral_derrape, max_dist_ruta=max_dist_ruta, marco=marco))
        partes.append(pagina.seccion_trayectorias(FIGURAS["2"], bool(logs), primera))
    else:
        partes += [pagina.seccion_vacia("1"), pagina.seccion_vacia("2")]

    if len(df_tel):
        tabla_3 = datos.tabla_derrape_por_vuelta(df_tel, umbral_derrape)
        registrar("3", figuras.grafica_3_derrape_bag(df_tel, umbral_derrape))
        registrar("3-resumen",
                  figuras.grafica_3_resumen_derrape(tabla_3, umbral_derrape))
        partes.append(pagina.seccion_derrape_bag(
            FIGURAS["3"], FIGURAS["3-resumen"], primera))
        primera = False
    else:
        partes.append(pagina.seccion_vacia("3"))

    if logs:
        registrar("4", figuras.grafica_4_circuito(
            logs, df_pos if hay_posiciones else None, marco=marco))
        partes.append(pagina.seccion("4", [FIGURAS["4"]], ["4"], primera=primera))
        primera = False

    if vueltas:
        stats = estadisticos_tiempos(vueltas)
        registrar("5", figuras.grafica_5_tiempos(vueltas, stats["mediana"]))
        cajas = None
        if len(df_tel):
            tiempos_por_vuelta = {v["numero"]: v["tiempo"] for v in vueltas}
            registrar("5-cajas", figuras.grafica_5_cajas_derrape(
                df_tel, tiempos_por_vuelta, umbral_derrape))
            cajas = FIGURAS["5-cajas"]
        partes.append(pagina.seccion_tiempos(
            vueltas, stats, FIGURAS["5"], cajas, primera))
        primera = False
    else:
        partes.append(pagina.seccion_vacia("5"))

    if logs:
        figs, claves, botones = [], [], []
        for cam, d in logs.items():
            clave = registrar("6", figuras.grafica_6_resumen(d["tabla"]),
                              cam, varias)
            figs.append(FIGURAS[clave])
            claves.append(clave)
            paneles = [
                (registrar(f"6-{panel}",
                           figuras.grafica_6_subplot(d["tabla"], panel),
                           cam, varias), panel)
                for panel in PANELES_6
            ]
            botones.append(pagina.botones_paneles(paneles, cam if varias else ""))
        partes.append(pagina.seccion("6", figs, claves,
                                     extra="".join(botones)))

    # --- Escribir y servir --------------------------------------------------
    html = pagina.ensamblar(bag_dir.name, partes)
    salida = bag_dir.parent / f"{bag_dir.name}_analisis.html"
    salida.write_text(html, encoding="utf-8")
    print(f"  -> {salida}  ({salida.stat().st_size / 1e6:.1f} MB)")

    if not args.sin_servidor:
        servidor.servir(html.encode("utf-8"), FIGURAS)


if __name__ == "__main__":
    main()
