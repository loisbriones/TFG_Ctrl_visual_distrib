#!/usr/bin/env python3
"""
Comparativa de TIEMPOS POR VUELTA entre varias carreras, en una sola página.

El dashboard de analisis.py genera un HTML por carrera. Este script coge N de
esos HTML ya generados y los pone juntos para responder a tres preguntas que
con una pestaña por carrera no se pueden contestar:

  - ¿quién dio mejor las 50 VUELTAS?  -> la contrarreloj de la tabla, que es la
                                         suma de las 50 (ver más abajo)
  - ¿quién hizo la MEJOR VUELTA?      -> el ★ de la gráfica de líneas
  - ¿quién CONTROLÓ MEJOR el coche?   -> las cajas (dispersión) y la σ

De dónde salen los datos: NO se lee el bag (en GRAFICAS_CARLOS/GUARDAR solo
están el HTML y el mp4, los bags se quedaron fuera). Se lee la tabla de la
sección 5 del propio dashboard, que va en HTML plano dentro del fichero:

    <table class="tiempos">...<tr class=""><td>7</td><td>1.819</td></tr>...

Cinco reglas de lectura (la página explica las cuatro de los tiempos al
abrirla; la quinta se ve sola en los paneles):

  1. Se tira la PRIMERA vuelta 1. Todas las carreras traen dos vueltas
     numeradas 1: el controlador publica el time_per_lap de la vuelta de
     calibración y solo DESPUÉS pone el contador a 0 (CarControllerNode.py,
     "self.vueltas = 0" al terminar la calibración). Esa primera es la vuelta
     lenta de calibración (PWM 65, o conducida despacio a mano en manual) y no
     compara nada.
  2. Las vueltas LENTAS cuentan. Antes se excluían las que el dashboard marca
     como anómalas (> 2x la media de su carrera, FACTOR_VUELTA_ANOMALA en
     analisis.py) porque una de 9,9 s con una mediana de 1,59 s aplasta el eje.
     Pero esto es una CONTRARRELOJ: si el coche se salió y hubo que volver a
     ponerlo en la pista, ese tiempo se perdió y tiene que pesar. Excluirlas
     medía "cómo iba el coche cuando iba bien", que es otra pregunta.
  3. Se descartan las vueltas MAL MEDIDAS: por debajo de la mitad de la mediana
     no hay vuelta que valga, es la meta disparando dos veces y partiendo una
     en dos trozos, así que se van los dos trozos (FRAC_VUELTA_MAL_MEDIDA). Es
     el único descarte que queda, y va avisado debajo de la tabla.
  4. Se comparan las VUELTAS_CONTRARRELOJ primeras vueltas de cada carrera: la
     que dé más se corta ahí (lo de después suele ser el coche rodando mientras
     se paraba la grabación) y la que no llegue sale sin tiempo de contrarreloj.
  5. Un PANEL POR COCHE. El coche pesa más que el piloto (en el óvalo de las
     pruebas el rojo va ~0,3 s por vuelta más rápido que el azul), así que
     mezclar coches en un mismo eje compararía coches, no pilotos. El coche se
     detecta por el nombre del fichero.

Uso (no necesita ROS ni el contenedor de run.sh, solo plotly):

    ANALISIS/env/bin/python analisis/comparativa.py GRAFICAS_CARLOS/GUARDAR
    ANALISIS/env/bin/python analisis/comparativa.py carpeta_A carpeta_B \\
        "Mi etiqueta=carpeta_C/algo_analisis.html" --salida /tmp/comparativa.html

Cada argumento posicional es un _analisis.html o una CARPETA (se buscan dentro
los *_analisis.html, recursivamente y ordenados por nombre). Con el prefijo
"Etiqueta=" se renombra a mano una serie; sin él la etiqueta sale del nombre.
"""

import argparse
import html
import re
import statistics
import sys
from pathlib import Path

import plotly.graph_objects as go
from plotly.subplots import make_subplots

# figuras.py vive en esta misma carpeta y arrastra solo numpy/pandas/plotly
# (nada de ROS), así que se puede importar aunque el script se lance desde la
# raíz del repositorio y fuera del contenedor.
sys.path.insert(0, str(Path(__file__).parent))

from figuras import (  # noqa: E402
    COL_CRITICO,
    COL_GRID,
    COL_SUPERFICIE,
    COL_TINTA,
    COL_TINTA_2,
    FUENTE,
    _layout_base,
)

# Paleta categórica validada del skill dataviz (modo claro), la misma familia
# de colores que usa figuras.py. El ORDEN es lo que garantiza que dos series
# vecinas se distingan también con daltonismo, así que no se toca: se van
# gastando slots de izquierda a derecha. Los colores de ESTADO (el rojo
# crítico de los atípicos) quedan fuera: no se reciclan como serie.
PALETA = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100",
          "#e87ba4", "#008300", "#4a3aa7", "#e34948"]

# Con más carreras del mismo coche que slots habría que reciclar colores
# dentro de un panel y dos series quedarían iguales. Caso raro: no se soporta,
# solo se avisa y se recortan (comparar 8 carreras de un tirón ya es mucho).
MAX_CARRERAS_POR_COCHE = len(PALETA)

# La fila de la tabla de tiempos del dashboard: la clase (que trae "anomala" o
# no, aquí ya no se mira), el número de vuelta y el tiempo. Lo que va después
# del tiempo en esa celda son las notas del dashboard ("★ mejor", "⚠ >2× media")
# y se ignora.
FILA_TIEMPO = re.compile(
    r'<tr class="([^"]*)"><td>(\d+)</td><td>(\d+\.\d+)')

# Cuántas vueltas dura la contrarreloj. Todas las carreras se comparan sobre las
# MISMAS 50 primeras vueltas: la que dé más se corta ahí (lo que viene después
# suele ser el coche rodando mientras se paraba la grabación) y la que no llegue
# se enseña igual, pero sin tiempo de contrarreloj, porque sumar 20 vueltas
# contra 50 no compara nada.
VUELTAS_CONTRARRELOJ = 50

# Por debajo de esta fracción de la mediana de su carrera, un tiempo no es una
# vuelta rápida: es la línea de meta disparando dos veces y partiendo una vuelta
# en dos trozos (en el óvalo hay un 0,186 s seguido de un 1,181 s que juntos son
# la vuelta normal de 1,367 s de ese coche). No hay forma de saber cuál de los
# dos trozos es cuál, así que se van los dos y se avisa.
FRAC_VUELTA_MAL_MEDIDA = 0.5


# ---------------------------------------------------------------------------
# Lectura de los HTML del dashboard
# ---------------------------------------------------------------------------
def leer_tiempos(ruta):
    """Las vueltas de la CONTRARRELOJ de un _analisis.html: [(numero, tiempo),
    ...], sin la vuelta de calibración, sin las mal medidas y cortadas a
    VUELTAS_CONTRARRELOJ.

    OJO con lo que NO se quita: las vueltas lentas (las que el dashboard marca
    como anómalas, > 2x la media de su carrera) cuentan enteras. Antes se
    tiraban, y eso medía "cómo iba el coche cuando iba bien"; aquí se compara
    una contrarreloj, y si el coche se sale y hay que volver a ponerlo en la
    pista ese tiempo se perdió y tiene que pesar en el resultado igual que en
    una carrera de verdad.

    Devuelve (vueltas, n_mal_medidas) o None si el fichero no es un dashboard."""
    texto = ruta.read_text(encoding="utf-8", errors="replace")
    tabla = re.search(r'<table class="tiempos">.*?</tbody>', texto, re.S)
    if not tabla:
        print(f"AVISO: {ruta.name} no tiene la tabla de tiempos de la "
              "sección 5 (¿no es un HTML del dashboard?). Se salta.")
        return None

    filas = [(cls, int(num), float(t))
             for cls, num, t in FILA_TIEMPO.findall(tabla.group(0))]
    if not filas:
        print(f"AVISO: {ruta.name} tiene la tabla vacía. Se salta.")
        return None

    # Regla 1: la carrera empieza en la ÚLTIMA vuelta 1 de la tabla. Todo lo
    # anterior es la calibración (y, si alguien reinició la calibración a
    # media grabación, también lo que corrió antes de reiniciarla).
    inicios = [i for i, (_, num, _) in enumerate(filas) if num == 1]
    if inicios:
        descartadas = inicios[-1]
        filas = filas[inicios[-1]:]
    else:
        descartadas = 0
        print(f"AVISO: {ruta.name} no tiene ninguna vuelta 1; se usan todas "
              "las filas tal cual.")

    # Regla 2: fuera las vueltas MAL MEDIDAS. Una fila muy por debajo de la
    # mediana es media vuelta (la meta disparó dos veces), así que se va con la
    # SIGUIENTE, que es el otro trozo de esa misma vuelta. La mediana se saca de
    # todas las filas: es robusta y las pocas raras no la mueven.
    tiempos = sorted(t for _, _, t in filas)
    mediana = tiempos[len(tiempos) // 2]
    vueltas = []
    n_mal_medidas = 0
    saltar_siguiente = False
    for _, num, t in filas:
        if saltar_siguiente:
            saltar_siguiente = False
            continue
        if t < FRAC_VUELTA_MAL_MEDIDA * mediana:
            n_mal_medidas += 1
            saltar_siguiente = True   # el otro trozo de la vuelta partida
            continue
        vueltas.append((num, t))

    # Regla 3: la contrarreloj son las N primeras vueltas de lo que quede
    cortadas = max(0, len(vueltas) - VUELTAS_CONTRARRELOJ)
    vueltas = vueltas[:VUELTAS_CONTRARRELOJ]
    print(f"  {ruta.parent.name}: {len(vueltas)} vueltas "
          f"(- {descartadas} de calibración, - {n_mal_medidas} mal medidas, "
          f"- {cortadas} sobrantes)")
    return vueltas, n_mal_medidas


def etiqueta_y_coche(ruta):
    """Nombre corto para la leyenda y coche (rojo/azul) de una carrera.

    La etiqueta sale del NOMBRE DEL FICHERO (su stem sin el sufijo "_analisis"),
    que es el nombre con el que se grabó la carrera. Antes salía de la CARPETA,
    pero eso solo funciona cuando cada carrera está en su propia subcarpeta (como
    en GRAFICAS_CARLOS/GUARDAR); con los HTML sueltos en una misma carpeta (como
    GRAFICAS_CIRCUITO_OCHO) todos compartían el nombre de la carpeta contenedora
    y salían idénticos. La carpeta queda como respaldo por si el HTML de dentro
    se llama distinto (por ejemplo MANUAL_BRAIS_COCHE_AZUL_analisis.html dentro
    de MANUAL_BRAIS_COCHE_AZUL_CIRCUITO_OVALO_1). El coche se busca primero en el
    nombre del fichero, que es el que manda (en alguna carpeta se coló un mp4 del
    coche contrario)."""
    base = re.sub(r"_analisis$", "", ruta.stem)
    carpeta = ruta.parent.name
    coche = next((c for nombre in (base.lower(), carpeta.lower())
                  for c in ("rojo", "azul") if c in nombre), None)
    if coche is None:
        coche = "sin identificar"
        print(f"AVISO: no se reconoce el coche de {ruta.name} (ni rojo ni "
              "azul); va a su propio panel.")

    # El nombre descriptivo suele estar en el fichero; se prueba primero ahí y
    # se cae a la carpeta por si el HTML se llamara distinto que su subcarpeta.
    for nombre in (base, carpeta):
        # Carreras manuales: MANUAL_<PILOTO>_COCHE_<COLOR>_..._<nnn>. Se añade el
        # id de ejecución (3 cifras, como en las del algoritmo) si aparece al
        # final, para distinguir dos vueltas del mismo piloto (JAICKOB 003 vs
        # 004). Restringirlo a 3 cifras deja intactas las de Carlos: el "_1" de
        # "..._OVALO_1" es el circuito (1 cifra), no un id de ejecución.
        manual = re.match(r"MANUAL_([A-ZÁÉÍÓÚÑ]+)_", nombre, re.I)
        if manual:
            piloto = manual.group(1).upper()
            run = re.search(r"_(\d{3})$", nombre)
            return (f"MANUAL {piloto} {run.group(1)}" if run
                    else f"MANUAL {piloto}"), coche

        # Carreras del algoritmo: ..._<nnn>_vmin_<v>_vmax_<V>_d_<d>
        algo = re.search(r"_(\d{3})_vmin_(\d+)_vmax_(\d+)_d_(\d+)", nombre)
        if algo:
            n, vmin, vmax, d = algo.groups()
            return f"ALGO {n} · {vmin}-{vmax} d{d}", coche

    # Cualquier otra cosa: el nombre del fichero, legible y recortado
    nombre = base.replace("_", " ")
    return (nombre[:37] + "…") if len(nombre) > 38 else nombre, coche


def recopilar(rutas):
    """Lista de carreras {etiqueta, coche, vueltas, n_mal_medidas} en el orden
    en que se pidieron. Cada ruta puede ser un HTML o una carpeta que los
    contenga."""
    carreras = []
    for entrada in rutas:
        etiqueta_manual = None
        if "=" in entrada and not Path(entrada).exists():
            etiqueta_manual, entrada = entrada.split("=", 1)
        ruta = Path(entrada)
        if not ruta.exists():
            print(f"AVISO: no existe {entrada}. Se salta.")
            continue

        ficheros = ([ruta] if ruta.is_file()
                    else sorted(ruta.rglob("*_analisis.html")))
        if not ficheros:
            print(f"AVISO: en {entrada} no hay ningún *_analisis.html.")
            continue

        for fichero in ficheros:
            leido = leer_tiempos(fichero)
            if leido is None:
                continue
            vueltas, n_mal_medidas = leido
            etiqueta, coche = etiqueta_y_coche(fichero)
            carreras.append({
                "etiqueta": etiqueta_manual or etiqueta,
                "coche": coche,
                "vueltas": vueltas,
                "n_mal_medidas": n_mal_medidas,
            })
    return carreras


def agrupar_por_coche(carreras):
    """{coche: [carreras]} en el orden en que aparecieron, recortando los
    grupos que no caben en la paleta."""
    grupos = {}
    for c in carreras:
        grupos.setdefault(c["coche"], []).append(c)
    for coche, grupo in grupos.items():
        if len(grupo) > MAX_CARRERAS_POR_COCHE:
            print(f"AVISO: {len(grupo)} carreras del coche {coche} y solo "
                  f"{MAX_CARRERAS_POR_COCHE} colores distinguibles; se "
                  "comparan las primeras.")
            grupos[coche] = grupo[:MAX_CARRERAS_POR_COCHE]
    # Cada carrera se queda con su color, el mismo en las dos figuras: el
    # color identifica la CARRERA, no su puesto en la clasificación.
    for grupo in grupos.values():
        for color, carrera in zip(PALETA, grupo):
            carrera["color"] = color
    return grupos


# ---------------------------------------------------------------------------
# Figura 1: tiempo por vuelta, un panel por coche
# ---------------------------------------------------------------------------
def grafica_lineas(grupos):
    """Líneas de tiempo por vuelta superpuestas, un subplot por coche.

    Ejes Y independientes por panel (cada coche tiene su rango) y SIN empezar
    en 0: con vueltas de ~1,5 s, arrancar el eje en cero deja todas las líneas
    pegadas y no se ve la décima que separa a un piloto de otro."""
    coches = list(grupos)
    fig = make_subplots(
        rows=len(coches), cols=1, shared_xaxes=True, vertical_spacing=0.12,
        subplot_titles=[f"coche {c}" for c in coches])

    for i, coche in enumerate(coches, start=1):
        # Una leyenda por panel: los colores se reparten dentro de cada coche,
        # así que una leyenda común mezclaría dos series del mismo color.
        clave_leyenda = "legend" if i == 1 else f"legend{i}"
        for carrera in grupos[coche]:
            x = [v for v, _ in carrera["vueltas"]]
            y = [t for _, t in carrera["vueltas"]]
            fig.add_trace(go.Scatter(
                x=x, y=y, mode="lines", name=carrera["etiqueta"],
                legend=clave_leyenda,
                line=dict(color=carrera["color"], width=2),
                hovertemplate="%{y:.3f} s<extra>"
                              + carrera["etiqueta"] + "</extra>",
            ), row=i, col=1)

        # La mejor vuelta del panel: UNA sola marca, sobre el punto que ya
        # está dibujado. Es la respuesta a "quién hizo la mejor vuelta".
        tiempos = [t for c in grupos[coche] for _, t in c["vueltas"]]
        t, v, quien = min((t, v, c["etiqueta"])
                          for c in grupos[coche] for v, t in c["vueltas"])
        fig.add_annotation(
            x=v, y=t, row=i, col=1, text=f"★ mejor · {quien} {t:.3f} s",
            showarrow=True, arrowhead=0, arrowwidth=1,
            arrowcolor=COL_TINTA_2, ax=0, ay=26, yanchor="top",
            font=dict(size=11, color=COL_TINTA))

        # Rango del eje a mano: el automático deja la mejor vuelta pegada al
        # borde de abajo y la etiqueta del ★ se saldría del panel. Un 18% de
        # hueco debajo y un 5% encima. Sin empezar en 0: con vueltas de ~1,5 s
        # arrancar el eje en cero deja todas las líneas pegadas y no se ve la
        # décima que separa a un piloto de otro.
        alto = max(tiempos) - min(tiempos)
        fig.update_yaxes(title_text="tiempo (s)", row=i, col=1,
                         range=[min(tiempos) - 0.18 * alto,
                                max(tiempos) + 0.05 * alto])

    fig.update_xaxes(title_text="vuelta", dtick=5, row=len(coches), col=1)
    _layout_base(fig, "Tiempo por vuelta · todas las carreras",
                 160 + 300 * len(coches))
    fig.update_layout(hovermode="x unified",
                      margin=dict(l=70, r=250, t=90, b=60))
    # Cada leyenda pegada a la altura de su panel (el dominio del eje Y lo
    # calcula make_subplots, así que se lee después de crear los subplots)
    for i, coche in enumerate(coches, start=1):
        dominio = fig.layout["yaxis" if i == 1 else f"yaxis{i}"].domain
        fig.update_layout({
            "legend" if i == 1 else f"legend{i}": dict(
                x=1.01, xanchor="left", y=dominio[1], yanchor="top",
                orientation="v",
                title=dict(text=f"coche {coche}", font=dict(size=11)),
                font=dict(size=11), bgcolor=COL_SUPERFICIE,
                bordercolor=COL_GRID, borderwidth=1)})
    for anotacion in fig.layout.annotations[:len(coches)]:
        anotacion.font = dict(size=13, color=COL_TINTA)
    return fig


# ---------------------------------------------------------------------------
# Figura 2: cajas, la regularidad de cada carrera
# ---------------------------------------------------------------------------
def grafica_cajas(grupos):
    """Una caja por carrera (horizontal, que las etiquetas son largas), un
    panel por coche. La caja son los cuartiles de sus vueltas de la
    contrarreloj: cuanto más estrecha, más regular fue la conducción. Los
    atípicos van sueltos en rojo, igual que en las cajas de la sección 5 del
    dashboard, y
    ahí es donde asoman las salidas de pista (que ya no se excluyen)."""
    coches = list(grupos)
    fig = make_subplots(
        rows=len(coches), cols=1, shared_xaxes=False, vertical_spacing=0.10,
        subplot_titles=[f"coche {c}" for c in coches],
        row_heights=[len(grupos[c]) for c in coches])

    for i, coche in enumerate(coches, start=1):
        # De abajo arriba por mediana: la carrera más rápida arriba del todo
        ordenadas = sorted(grupos[coche],
                           key=lambda c: statistics.median(
                               t for _, t in c["vueltas"]), reverse=True)
        for carrera in ordenadas:
            y = [t for _, t in carrera["vueltas"]]
            fig.add_trace(go.Box(
                x=y, name=carrera["etiqueta"], orientation="h",
                boxpoints="outliers", showlegend=False,
                marker=dict(color=carrera["color"],
                            outliercolor=COL_CRITICO, size=5),
                line=dict(color=carrera["color"], width=2),
                fillcolor=COL_SUPERFICIE,
                hovertemplate="%{x:.3f} s<extra>"
                              + carrera["etiqueta"] + "</extra>",
            ), row=i, col=1)
        fig.update_xaxes(title_text="tiempo de vuelta (s)", row=i, col=1)
        fig.update_yaxes(tickfont=dict(size=11), row=i, col=1)

    _layout_base(fig, "Regularidad · cuartiles del tiempo de vuelta",
                 140 + 90 * len(carreras_totales(grupos)) + 60 * len(coches))
    fig.update_layout(margin=dict(l=180, r=40, t=90, b=60), showlegend=False)
    for anotacion in fig.layout.annotations[:len(coches)]:
        anotacion.font = dict(size=13, color=COL_TINTA)
    return fig


def carreras_totales(grupos):
    """Todas las carreras de todos los grupos, en orden."""
    return [c for grupo in grupos.values() for c in grupo]


# ---------------------------------------------------------------------------
# Tabla resumen
# ---------------------------------------------------------------------------
def tabla_resumen(grupos):
    """Una tabla por coche, ordenada por el tiempo de la CONTRARRELOJ (la más
    rápida arriba).

    La contrarreloj —la suma de las VUELTAS_CONTRARRELOJ vueltas, salidas de
    pista incluidas— es la respuesta a "¿cuál dio mejor las 50 vueltas?", que es
    lo que se viene a mirar: premia igual ir rápido que no irse fuera. Las
    carreras que no llegan a esas vueltas no tienen con qué compararse, así que
    van al final ordenadas por mediana y con un aviso debajo.

    La mediana y la σ siguen contando la otra mitad: quién controló mejor no es
    quién bajó más una vuelta suelta, sino quién repitió el tiempo vuelta tras
    vuelta. Todos los números salen de las MISMAS vueltas que la contrarreloj,
    para que la fila sea coherente consigo misma.

    La tabla es además la lectura alternativa al color que exige la revisión
    de accesibilidad: tres de los colores de la paleta no llegan a 3:1 de
    contraste sobre el fondo claro."""
    partes = []
    for coche, grupo in grupos.items():
        # Primero las que compiten (por tiempo total) y luego las que no llegan
        # a las vueltas de la contrarreloj (por mediana)
        completas = [c for c in grupo
                     if len(c["vueltas"]) == VUELTAS_CONTRARRELOJ]
        cortas = [c for c in grupo if len(c["vueltas"]) < VUELTAS_CONTRARRELOJ]
        ordenadas = (
            sorted(completas, key=lambda c: sum(t for _, t in c["vueltas"]))
            + sorted(cortas, key=lambda c: statistics.median(
                t for _, t in c["vueltas"])))
        filas = []
        for carrera in ordenadas:
            t = [x for _, x in carrera["vueltas"]]
            sigma = statistics.stdev(t) if len(t) > 1 else 0.0
            total = (f"{sum(t):.2f}" if len(t) == VUELTAS_CONTRARRELOJ else "—")
            filas.append(
                f"<tr><td style='text-align:left'>"
                f"<span style='color:{carrera['color']}'>■</span> "
                f"{html.escape(carrera['etiqueta'])}</td>"
                f"<td>{len(t)}</td><td>{total}</td><td>{min(t):.3f}</td>"
                f"<td>{statistics.median(t):.3f}</td>"
                f"<td>{statistics.mean(t):.3f}</td><td>{sigma:.3f}</td></tr>")
        # Avisos: por qué una carrera no tiene tiempo de contrarreloj y qué se
        # descartó por venir mal medido
        notas = [f"<strong>{html.escape(c['etiqueta'])}</strong>: "
                 f"{len(c['vueltas'])} vueltas, no llega a "
                 f"{VUELTAS_CONTRARRELOJ}: fuera de la contrarreloj."
                 for c in cortas]
        notas += [f"<strong>{html.escape(c['etiqueta'])}</strong>: "
                  f"{c['n_mal_medidas']} vuelta(s) mal medida(s) descartada(s) "
                  "(la meta disparó dos veces y partió la vuelta en dos)."
                  for c in ordenadas if c["n_mal_medidas"]]
        pie = ("<p>" + "<br>".join(notas) + "</p>") if notas else ""
        partes.append(
            f"<h3>coche {html.escape(coche)}</h3>"
            '<table class="tiempos"><thead><tr>'
            "<th style='text-align:left'>carrera</th><th>vueltas</th>"
            f"<th>contrarreloj {VUELTAS_CONTRARRELOJ} (s)</th>"
            "<th>mejor (s)</th><th>mediana (s)</th><th>media (s)</th>"
            "<th>σ (s)</th></tr></thead><tbody>"
            + "".join(filas) + "</tbody></table>" + pie)
    return "".join(partes)


# ---------------------------------------------------------------------------
# Página
# ---------------------------------------------------------------------------
def ensamblar_html(grupos, fig_lineas, fig_cajas):
    """La página completa, con el mismo estilo que el dashboard (superficie
    clara, tinta oscura). plotly.js va incrustado UNA vez, en la primera
    figura, para que el fichero funcione con doble clic sin red."""
    div_lineas = fig_lineas.to_html(
        full_html=False, include_plotlyjs=True,
        config={"displaylogo": False, "responsive": True})
    div_cajas = fig_cajas.to_html(
        full_html=False, include_plotlyjs=False,
        config={"displaylogo": False, "responsive": True})
    n = len(carreras_totales(grupos))
    cuantas = "1 carrera" if n == 1 else f"{n} carreras"
    return f"""<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="utf-8">
<title>Comparativa de tiempos</title>
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
  h3 {{ font-size: 14px; color: {COL_TINTA_2}; margin-bottom: 6px; }}
  p  {{ color: {COL_TINTA_2}; font-size: 13px; line-height: 1.5; }}
  ul {{ color: {COL_TINTA_2}; font-size: 13px; line-height: 1.5;
        max-width: 900px; }}
  table.tiempos {{ border-collapse: collapse; font-size: 13px;
                   margin-bottom: 22px; }}
  table.tiempos th, table.tiempos td {{
    border: 1px solid {COL_GRID}; padding: 3px 12px; text-align: right; }}
  table.tiempos tbody tr:first-child td {{ font-weight: 600; }}
</style>
</head>
<body>
<h1>Comparativa de tiempos por vuelta · {cuantas}</h1>

<p>Esto es una <strong>contrarreloj a {VUELTAS_CONTRARRELOJ} vueltas</strong>:
gana quien menos tarda en darlas, no quien firma la vuelta rápida. Cómo se leen
los tiempos de cada carrera:</p>
<ul>
<li>Se tira la <strong>vuelta de calibración</strong> (todas las carreras traen
dos vueltas numeradas 1: la primera es la lenta de calibración).</li>
<li>Las <strong>vueltas lentas cuentan</strong>. Si el coche se salió y hubo que
volver a ponerlo en la pista, ese tiempo se perdió y suma en el total, en la
media y en la σ, igual que en una carrera de verdad.</li>
<li>Se descartan las <strong>vueltas mal medidas</strong> (un tiempo por debajo
de la mitad de la mediana no es una vuelta: es la meta disparando dos veces), y
con ellas el otro trozo de la vuelta que partieron. Se avisa debajo de la
tabla.</li>
<li>Se comparan las <strong>{VUELTAS_CONTRARRELOJ} primeras</strong> vueltas de
cada carrera. La que dé más se corta ahí; la que no llegue sale en la tabla pero
sin tiempo de contrarreloj.</li>
</ul>

<h2>1 · Tiempo por vuelta</h2>
{div_lineas}

<h2>2 · Regularidad</h2>
{div_cajas}

<h2>3 · Resumen</h2>
{tabla_resumen(grupos)}
</body>
</html>
"""


def main():
    parser = argparse.ArgumentParser(
        description="Compara los tiempos por vuelta de varias carreras a "
                    "partir de sus HTML de análisis.")
    parser.add_argument("rutas", nargs="+",
                        help="HTML de análisis o carpetas que los contengan; "
                             "admite el prefijo 'Etiqueta=ruta'")
    parser.add_argument("--salida", default="comparativa_vueltas.html",
                        help="fichero HTML de salida "
                             "(por defecto comparativa_vueltas.html)")
    args = parser.parse_args()

    print("Leyendo carreras:")
    carreras = recopilar(args.rutas)
    if not carreras:
        print("ERROR: no se pudo leer ninguna carrera.")
        return 1

    grupos = agrupar_por_coche(carreras)
    salida = Path(args.salida)
    salida.write_text(ensamblar_html(grupos, grafica_lineas(grupos),
                                     grafica_cajas(grupos)), encoding="utf-8")
    print(f"\n{len(carreras_totales(grupos))} carreras en "
          f"{len(grupos)} paneles -> {salida.resolve()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
