#!/usr/bin/env python3
"""
Todas las figuras plotly del dashboard unificado (analisis.py).

Las figuras 6b/6c (heatmap de PWM y resumen por vuelta) vienen de
analizar_log_algoritmo.py; solo cambió el número de sección en el título. El
resto (circuito con PWM, derrape 3D, trayectorias, distancia del bag) sigue
el mismo estilo: paleta validada del skill dataviz, chrome común en
_layout_base y sliders por vuelta.

TODOS los sliders por vuelta son INSTANTÁNEOS: cada paso muestra SOLO su
vuelta (figuras 1, 2, 3b y 4). Se implementan con arrays de visibilidad: las
trazas "estáticas" (fondos, trayectorias, fantasmas de leyenda) siempre
visibles y un bloque de trazas por vuelta. Además las figuras con plano
(1, 2) llevan los EJES FIJOS, calculados sobre toda la carrera: si cada
vuelta se autoescalara, el circuito se estiraría o achataría según por dónde
pasó el coche y dos vueltas dejarían de ser comparables a ojo.
"""

import math

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from parseo_log import Carrera, celdas_de_zona

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
# PLANO GLOBAL multicámara: cada cámara tiene su propio sistema de píxeles
# (su imagen), así que para dibujar TODO el circuito en una sola figura cada
# cámara entra con una traslación (dx, dy). Los valores se ajustan A MANO
# aquí hasta que el circuito quede continuo (no hay calibración automática:
# depende de cómo estén colocadas las cámaras en cada montaje). Una cámara
# que no aparezca en el dict se coloca automáticamente a la derecha de la
# anterior (una "plaza" de ANCHO_PLAZA_CAMARA px por cámara).
# ---------------------------------------------------------------------------
OFFSETS_CAMARAS = {
    "camara_01": (0.0, 0.0),
    "camara_02": (660.0, 0.0),
}
# Ancho de la plaza por defecto: 640 px de imagen + 20 de margen visual
ANCHO_PLAZA_CAMARA = 660.0


def offset_camara(camara, indice):
    """(dx, dy) de una cámara en el plano global; si no está en el dict se
    coloca en la plaza `indice` (orden alfabético de cámaras)."""
    if camara in OFFSETS_CAMARAS:
        return OFFSETS_CAMARAS[camara]
    return (indice * ANCHO_PLAZA_CAMARA, 0.0)


# ---------------------------------------------------------------------------
# SENTIDO DE LA MARCHA: una sola flecha, FUERA del trazado y a la altura de meta
# ---------------------------------------------------------------------------
# Quien mire una figura del plano (1, 2 y 4) no tiene por qué saber hacia dónde
# iba el coche. Se indica con UNA flecha a la altura del inicio de meta, no con
# una hilera de flechas por todo el trazado: la figura 2 ya lleva tres
# trayectorias superpuestas y otra capa más la haría ilegible.
#
# La flecha se dibuja SEPARADA hacia fuera del circuito (por el lado contrario
# al centro), no encima de la trayectoria: puesta sobre el trazado se pierde
# entre los marcadores de las pegatinas y no se distingue. Fuera, en el hueco
# vacío del margen, se ve de un vistazo. Va del ámbar de la estrella de inicio
# (COL_INICIO_VUELTA) para que las dos marcas se lean como una sola cosa.
# ---------------------------------------------------------------------------
# Cuánto se avanza desde la meta para sacar la dirección de salida. Se mide en
# PÍXELES y no en muestras porque el número de posiciones por vuelta depende de
# la velocidad y del circuito (en el óvalo de las pruebas una vuelta son ~56
# muestras, así que "8 muestras" ya es un séptimo de vuelta y la flecha se iría
# lejísimos de la meta). 40 px es un tramo corto —la flecha queda a la altura de
# la estrella— pero suficiente para que el ruido de detección (unos pocos
# píxeles) no tuerza la punta.
DIST_TANGENTE_SENTIDO = 40.0
# Tope de muestras que se miran buscando esos 40 px: si el coche iba muy lento
# (o parado en meta) no se sigue avanzando media vuelta, se usa lo que haya.
MAX_MUESTRAS_TANGENTE = 12
# Cuánto se aparta la flecha del trazado, en fracción del lado mayor del
# circuito. Tiene que sacarla del ancho de la pista (las dos pegatinas y la
# trayectoria base van a unos pocos píxeles unas de otras) sin mandarla al otro
# extremo de la figura: 0,07 la deja claramente fuera pero al lado.
SEPARACION_FLECHA_SENTIDO = 0.07


def _tangente_meta(fr, camaras, centro=None, separacion=0.0):
    """(p0, p1) en coordenadas del plano global: el primer punto de la vuelta
    (pegatina DELANTERA, el mismo que marca la estrella de inicio) y el primero
    que se aleja de él DIST_TANGENTE_SENTIDO píxeles. Es lo que orienta la
    flecha del sentido de la marcha, que se dibuja en p1.

    Con `centro` (el del circuito) y `separacion` (en píxeles) el par se
    desplaza esa distancia en PERPENDICULAR a la marcha, hacia el lado
    contrario al centro: la flecha sale así fuera del trazado, donde se ve. Los
    dos puntos se desplazan igual, así que la dirección no cambia.

    Se corta en cuanto cambia la cámara: dos cámaras entran en el plano global
    con offsets distintos, así que un cambio dentro de la ventana metería un
    salto y la flecha apuntaría a cualquier sitio. Devuelve None si la vuelta
    no da dos puntos utilizables (caso raro: entonces no se dibuja flecha).

    fr: filas de UNA vuelta, en orden de grabación."""
    if len(fr) < 2:
        return None
    indice = {c: i for i, c in enumerate(camaras)}
    filas = fr.iloc[:MAX_MUESTRAS_TANGENTE + 1]
    cam = filas.iloc[0]["camara"]
    dx, dy = offset_camara(cam, indice.get(cam, 0))
    a = filas.iloc[0]
    p0 = np.array([a["fx"] + dx, a["fy"] + dy], dtype=float)
    p1 = None
    for i in range(1, len(filas)):
        fila = filas.iloc[i]
        if fila["camara"] != cam:
            break
        p = np.array([fila["fx"] + dx, fila["fy"] + dy], dtype=float)
        p1 = p
        if np.linalg.norm(p - p0) >= DIST_TANGENTE_SENTIDO:
            break
    if p1 is None or np.allclose(p0, p1):
        return None  # el coche estaba parado en meta: no hay dirección
    if centro is not None and separacion:
        u = (p1 - p0) / np.linalg.norm(p1 - p0)
        n = np.array([-u[1], u[0]])           # perpendicular a la marcha
        medio = (p0 + p1) / 2
        if np.dot(n, medio - np.asarray(centro, dtype=float)) < 0:
            n = -n                            # el lado que se aleja del centro
        p0, p1 = p0 + separacion * n, p1 + separacion * n
    return p0, p1


def _posicion_etiqueta_flecha(p, centro):
    """Dónde colgar el texto "sentido de la marcha" de la flecha: SIEMPRE hacia
    el lado contrario al centro del circuito, para que la etiqueta no se meta
    dentro del trazado. Ojo con el eje Y: en estas figuras va invertido (coor-
    denadas de imagen), así que una Y de dato mayor que la del centro cae MÁS
    ABAJO en pantalla."""
    if centro is None:
        return "middle right"
    d = np.asarray(p, dtype=float) - np.asarray(centro, dtype=float)
    if abs(d[0]) >= abs(d[1]):
        return "middle right" if d[0] > 0 else "middle left"
    return "bottom center" if d[1] > 0 else "top center"


def _puntos_con_holgura(traza, centro, holgura=1.15):
    """Los puntos (Nx2) de una traza de flecha MÁS una copia un poco más lejos
    del centro del circuito. Se usan para acotar los ejes de las figuras de
    rango fijo (1 y 2): si se acotan solo con la flecha, esta queda pegada al
    borde y su ETIQUETA se sale del área de dibujo y se corta contra la
    leyenda. Con la copia se reserva ese hueco."""
    pts = np.column_stack([traza.x, traza.y]).astype(float)
    c = np.asarray(centro, dtype=float)
    return np.vstack([pts, c + (pts - c) * holgura])


def _flecha_sentido_2d(tangente, visible=True, centro=None):
    """Traza de la flecha del sentido de la marcha para las figuras 2D.

    Son los DOS puntos de la tangente con marcador de punta de flecha y
    `angleref="previous"`: plotly orienta cada punta según el vector que la
    trae del punto anterior, y lo hace en coordenadas de PANTALLA, así que el
    eje Y invertido de estas figuras no lo despista y no hace falta calcular
    ángulos a mano. El primer punto solo es la referencia del ángulo: va con
    opacidad 0.

    La flecha va GRANDE y con la etiqueta al lado: como está fuera del trazado
    no molesta a nadie, y así se entiende sin tener que buscar la leyenda."""
    p0, p1 = tangente
    return go.Scatter(
        x=[p0[0], p1[0]], y=[p0[1], p1[1]], mode="markers+text",
        name="Sentido de la marcha", showlegend=False,
        marker=dict(symbol="arrow", angleref="previous", size=28,
                    color=COL_INICIO_VUELTA, opacity=[0.0, 1.0],
                    line=dict(color=COL_SUPERFICIE, width=1)),
        text=["", "sentido de la marcha"],
        textposition=_posicion_etiqueta_flecha(p1, centro),
        textfont=dict(color=COL_INICIO_VUELTA, size=11),
        # Sin recorte: la etiqueta cae fuera del trazado y, si la flecha está
        # cerca del borde, se saldría del área de dibujo y quedaría cortada.
        # Con cliponaxis=False se pinta encima del margen de la figura.
        cliponaxis=False,
        hovertemplate="sentido de la marcha<extra></extra>",
        visible=visible,
    )


def _flecha_sentido_3d(tangente, largo, visible=True, centro=None):
    """La misma flecha para la figura 1, dibujada como una línea en el plano
    z=0: el astil (p0 -> p1) y las dos alas de la punta. No se usa go.Cone a
    propósito: el cono se dimensiona en unidades de datos y el eje Z de la
    figura 1 va exagerado (ALTURA_RELATIVA_Z), así que un cono de 30 px de
    radio taparía media caja. `largo` es lo que miden las alas, en píxeles del
    plano.

    Se dibuja de un trazo (astil, un ala, vuelta a la punta y la otra ala): en
    3D repasar un segmento no se nota y así todo cabe en una sola traza."""
    p0, p1 = tangente
    u = (p1 - p0) / np.linalg.norm(p1 - p0)
    alas = []
    for grados in (150.0, -150.0):
        t = np.radians(grados)
        r = np.array([u[0] * np.cos(t) - u[1] * np.sin(t),
                      u[0] * np.sin(t) + u[1] * np.cos(t)])
        alas.append(p1 + largo * r)
    puntos = [p0, p1, alas[0], p1, alas[1]]
    return go.Scatter3d(
        x=[p[0] for p in puntos], y=[p[1] for p in puntos],
        z=[0] * len(puntos), mode="lines+text",
        name="Sentido de la marcha", showlegend=False,
        line=dict(color=COL_INICIO_VUELTA, width=8),
        # La etiqueta va en el arranque del astil, para que no se monte con la
        # punta ni con el trazado
        text=["sentido de la marcha", "", "", "", ""],
        textposition=_posicion_etiqueta_flecha(p0, centro),
        textfont=dict(color=COL_INICIO_VUELTA, size=11),
        hovertemplate="sentido de la marcha<extra></extra>",
        visible=visible,
    )


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


# ===========================================================================
# FIGURA 6b: heatmap del perfil de PWM (vuelta x celda)
# ===========================================================================
# Cada fila es el perfil CON EL QUE EL COCHE CORRIÓ esa vuelta (el último
# volcado [PERFIL] anterior a su primer frame), expandiendo las zonas a
# celdas. Claro = lento (v_min), oscuro = rápido (v_max). Encima:
#   - aspa roja     = celdas castigadas DURANTE esa vuelta (carveos y
#                     castigos de gigantes; efecto en la fila siguiente)
#   - círculo gris  = celdas protegidas al EMPEZAR esa vuelta (no subieron
#                     aunque la vuelta anterior fuera limpia)
# ===========================================================================
def grafica_6b_heatmap(c: Carrera, vueltas, perfil_vuelta):
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
        f"6b · Perfil de PWM con el que se corrió cada vuelta "
        f"(claro={v_min:.0f}, oscuro={v_max:.0f})",
        alto,
    )
# ===========================================================================
# FIGURA 3b: la distancia de derrape DEL BAG, vuelta a vuelta
# ===========================================================================
# Lo que la 3 cuenta desde el log (por cámara y contra la celda), esta lo
# cuenta desde el BAG y contra el TIEMPO: el dist_derrape que publicó el
# controlador en /telemetria/<coche>/car_control, muestra a muestra, dentro
# de una vuelta. Sirve para ver de un vistazo si la distancia sube y baja
# como debe (plana en las primeras vueltas, con picos según sube el PWM) o
# si hay picos que no se corresponden con ningún derrape real.
#
# Cada punto va coloreado según la CÁMARA que envió la posición que originó
# esa telemetría, y una línea vertical discontinua marca cada cambio de
# cámara: los picos que aparecen justo en un cambio son sospechosos (la
# pegatina trasera todavía está fuera de la cadena de la cámara que entra,
# así que su "distancia perpendicular" es en realidad longitudinal).
#
# El eje derecho lleva el PWM que se estaba aplicando (topic /<coche>/pwd),
# escalonado: la orden vale hasta que llega la siguiente.
#
# df: DataFrame con una fila por telemetría dentro de una vuelta y columnas
#     [vuelta, t_vuelta, dist, derrapando, camara, bx, by, pwm]
# ===========================================================================
# Color de cada cámara en esta figura (la primera repite el azul de serie 1;
# con más de cuatro cámaras se reciclan, cosa que no va a pasar)
COLORES_CAMARA = [COL_SERIE_1, COL_SERIE_2, "#7b5ea7", COL_AVISO]


def _color_por_camara(camaras):
    return {cam: COLORES_CAMARA[i % len(COLORES_CAMARA)]
            for i, cam in enumerate(sorted(camaras))}


def grafica_3b_derrape_bag(df, umbral):
    vueltas = sorted(df["vuelta"].unique())
    # isinstance(str): una telemetría que no se pudo emparejar con ninguna
    # posición se queda sin cámara (None), y esa no es una cámara más
    camaras = sorted(c for c in df["camara"].unique() if isinstance(c, str))
    color_cam = _color_por_camara(camaras)
    # Rango del eje Y FIJO para todas las vueltas: si cada vuelta se
    # autoescalara, una vuelta plana de 3 px se vería igual de "picuda" que
    # una con un pico de 60 y no se podría comparar pasando de una a otra
    y_max = max(float(df["dist"].max()) * 1.05, umbral * 1.5)

    fig = go.Figure()

    # Contexto: todas las muestras de todas las vueltas (siempre visible)
    fig.add_trace(go.Scatter(
        x=df["t_vuelta"], y=df["dist"], mode="markers",
        name="todas las vueltas", marker=dict(size=3, color=COL_CONTEXTO),
        opacity=0.45, hoverinfo="skip",
    ))
    # Trazas fantasma que sostienen la leyenda de cámaras: si el color de
    # cámara lo explicaran las trazas de una vuelta, la leyenda cambiaría al
    # mover el slider
    for cam in camaras:
        fig.add_trace(go.Scatter(
            x=[None], y=[None], mode="markers", name=cam,
            marker=dict(size=7, color=color_cam[cam]), hoverinfo="skip",
        ))
    n_estaticas = 1 + len(camaras)

    for v in vueltas:
        fr = df[df["vuelta"] == v].sort_values("t_vuelta")
        visible = v == vueltas[0]
        pwms = [p for p in fr["pwm"] if p is not None and not pd.isna(p)]
        n_sobre = int((fr["dist"] > umbral).sum())
        # El nombre de la traza es el resumen de la vuelta: como la leyenda
        # solo enseña las trazas visibles, al cambiar de vuelta con el slider
        # (o con las flechas) el resumen se actualiza solo
        etiqueta = f"vuelta {int(v)} · máx {fr['dist'].max():.0f} px"
        if pwms:
            etiqueta += (f" · pwm {int(min(pwms))}" if min(pwms) == max(pwms)
                         else f" · pwm {int(min(pwms))}-{int(max(pwms))}")
        if n_sobre:
            etiqueta += f" · {n_sobre} sobre el umbral"
        fig.add_trace(go.Scatter(
            x=fr["t_vuelta"], y=fr["dist"], mode="lines+markers", name=etiqueta,
            # La línea solo une los puntos (gris): el color lo llevan los
            # marcadores, que es donde se lee de qué cámara viene cada muestra
            line=dict(color=COL_MUTED, width=1),
            marker=dict(size=6, color=[color_cam.get(c, COL_MUTED)
                                       for c in fr["camara"]]),
            # customdata como lista de listas, no np.stack: al apilar textos
            # (la cámara) con números, numpy lo convertiría todo a texto y el
            # formato numérico del hover (%{...:.0f}) dejaría de aplicarse
            customdata=[[f.camara or "?", f.bx, f.by, f.pwm]
                        for f in fr.itertuples()],
            hovertemplate=(
                "t=%{x:.2f} s · dist=%{y:.1f} px<br>%{customdata[0]}"
                "<br>trasera=(%{customdata[1]:.0f}, %{customdata[2]:.0f})"
                "<br>pwm=%{customdata[3]:.0f}<extra></extra>"
            ),
            visible=visible,
        ))
        # Muestras en las que el controlador tenía un derrape ABIERTO
        derr = fr[fr["derrapando"]]
        fig.add_trace(go.Scatter(
            x=derr["t_vuelta"], y=derr["dist"], mode="markers",
            name="derrape abierto", showlegend=False,
            marker=dict(symbol="x", size=9, color=COL_CRITICO),
            hovertemplate="derrapando · t=%{x:.2f} s · %{y:.1f} px<extra></extra>",
            visible=visible,
        ))
        # PWM aplicado, escalonado, en el eje de la derecha
        fig.add_trace(go.Scatter(
            x=fr["t_vuelta"], y=fr["pwm"], mode="lines", name="PWM aplicado",
            showlegend=False, yaxis="y2",
            line=dict(color=COL_BUENO, width=1.5, shape="hv"),
            hovertemplate="t=%{x:.2f} s · pwm=%{y:.0f}<extra></extra>",
            visible=visible,
        ))
        # Cambios de cámara: línea vertical en el punto medio entre la última
        # muestra de una cámara y la primera de la siguiente
        xs, textos = [], []
        filas = list(fr.itertuples())
        for anterior, actual in zip(filas, filas[1:]):
            if actual.camara and anterior.camara and actual.camara != anterior.camara:
                xc = (anterior.t_vuelta + actual.t_vuelta) / 2
                xs += [xc, xc, None]
                textos += [f"{anterior.camara} → {actual.camara}"] * 2 + [""]
        fig.add_trace(go.Scatter(
            x=xs, y=[0, y_max, None] * (len(xs) // 3), mode="lines",
            name="cambio de cámara", showlegend=False,
            line=dict(color=COL_MUTED, width=1, dash="dot"),
            text=textos, hovertemplate="%{text}<extra></extra>",
            visible=visible,
        ))

    fig.add_hline(y=umbral, line=dict(color=COL_TINTA, width=1),
                  annotation_text=f"umbral_derrape = {umbral:.0f} px",
                  annotation_font_color=COL_TINTA_2)

    # Slider instantáneo: las estáticas siempre visibles + las 4 trazas de
    # la vuelta (distancia, derrapes, PWM y cambios de cámara)
    pasos = []
    for k, v in enumerate(vueltas):
        visibles = [True] * n_estaticas + [False] * (4 * len(vueltas))
        for j in range(4):
            visibles[n_estaticas + 4 * k + j] = True
        pasos.append(dict(label=str(int(v)), method="update",
                          args=[{"visible": visibles}]))
    # Rango del eje de PWM: el de toda la carrera, también fijo, para que la
    # escalera se pueda comparar entre vueltas (y con margen, si no la línea
    # se pega al borde de la figura)
    pwms = [p for p in df["pwm"] if p is not None and not pd.isna(p)]
    rango_pwm = ([min(pwms) - 2, max(pwms) + 2] if pwms else None)

    # Los ejes se fijan por update_layout y NO con update_xaxes/update_yaxes:
    # esas dos aplican a TODOS los ejes de la figura, así que le pondrían al
    # eje del PWM el título y el rango de la distancia (y la escalera de PWM
    # desaparecería fuera de rango)
    fig.update_layout(
        sliders=[dict(active=0, currentvalue=dict(prefix="Vuelta "),
                      pad=dict(t=30), steps=pasos)],
        xaxis=dict(title_text="tiempo dentro de la vuelta (s)",
                   rangemode="tozero"),
        yaxis=dict(title_text="dist_derrape del bag (px)", range=[0, y_max]),
        # Eje derecho para el PWM que se estaba aplicando
        yaxis2=dict(
            title=dict(text="PWM aplicado", font=dict(color=COL_BUENO)),
            tickfont=dict(color=COL_BUENO), overlaying="y", side="right",
            showgrid=False, range=rango_pwm,
        ),
    )
    return _layout_base(
        fig,
        "3b · Distancia de derrape grabada en el bag, vuelta a vuelta "
        "(color = cámara; línea de puntos = cambio de cámara)",
        560,
    )


# ===========================================================================
# FIGURA 3c: resumen por vuelta de la distancia del bag
# ===========================================================================
# La misma serie de la 3b resumida a un número por vuelta, para ver la
# tendencia de toda la carrera de golpe: si las primeras vueltas son planas
# y los picos aparecen según sube el PWM, o si el máximo está disparado
# desde la primera vuelta (señal de que la distancia se calcula mal).
#
# tabla: DataFrame con una fila por vuelta y columnas
#        [vuelta, dist_max, dist_p95, dist_mediana, n_sobre_umbral, pwm_medio]
# ===========================================================================
def grafica_3c_resumen_derrape(tabla, umbral):
    fig = go.Figure()
    series = [
        ("dist_max", "máximo", COL_CRITICO),
        ("dist_p95", "percentil 95", COL_SERIO),
        ("dist_mediana", "mediana", COL_SERIE_1),
    ]
    for columna, nombre, color in series:
        fig.add_trace(go.Scatter(
            x=tabla["vuelta"], y=tabla[columna], mode="lines+markers",
            name=nombre, line=dict(color=color, width=2), marker=dict(size=5),
            hovertemplate=f"v%{{x}}: %{{y:.1f}} px<extra>{nombre}</extra>",
        ))
    fig.add_trace(go.Scatter(
        x=tabla["vuelta"], y=tabla["pwm_medio"], mode="lines",
        name="PWM medio", yaxis="y2",
        line=dict(color=COL_BUENO, width=2, shape="hv"),
        hovertemplate="v%{x}: pwm %{y:.1f}<extra></extra>",
    ))
    fig.add_hline(y=umbral, line=dict(color=COL_TINTA, width=1),
                  annotation_text=f"umbral_derrape = {umbral:.0f} px",
                  annotation_font_color=COL_TINTA_2)
    # Por ejes con nombre, no con update_yaxes: si no, el eje del PWM también
    # se llevaría el título y el rango de la distancia (ver la 3b)
    fig.update_layout(
        xaxis=dict(title_text="vuelta", dtick=5),
        yaxis=dict(title_text="dist_derrape (px)", rangemode="tozero"),
        yaxis2=dict(
            title=dict(text="PWM medio", font=dict(color=COL_BUENO)),
            tickfont=dict(color=COL_BUENO), overlaying="y", side="right",
            showgrid=False,
        ),
    )
    return _layout_base(
        fig, "3c · Resumen por vuelta de la distancia de derrape del bag", 440)


# ===========================================================================
# FIGURA 5c: la comparativa derrape <-> tiempo por vuelta
# ===========================================================================
# Es la que ilustra la tesis del algoritmo: subir el PWM aparta al coche de la
# trayectoria (más derrape) y baja el tiempo, hasta que el derrape pasa del
# umbral y el tiempo EMPEORA (el coche patina, se sale o hay que castigar la
# zona). Va en orden cronológico (X = vuelta), la distribución del derrape en
# cajas y el tiempo por encima: es hermana de la 3c y se lee igual.
#
# Hubo también una 5b (una vuelta = un punto, derrape en X y tiempo en Y, para
# ver la forma de U): se retiró porque con 40 vueltas la nube de puntos y sus
# bigotes se solapaban y no había quien la leyera.
#
# Las dos fuentes comparten numeración de vuelta: la tabla de derrape y los
# tiempos salen del MISMO bag (ventanas de time_per_lap), así que basta cruzar
# por el número de vuelta y quedarse con las que estén en las dos.
# ===========================================================================
def grafica_5c_cajas_derrape(df_tel, tiempos_por_vuelta, umbral):
    """df_tel: una fila por telemetría dentro de una vuelta (la de la 3b).
    Una caja por vuelta con TODAS sus muestras de dist_derrape y, en el eje
    derecho, el tiempo que se tardó en esa vuelta."""
    datos = df_tel[df_tel["vuelta"].isin(tiempos_por_vuelta)]
    fig = go.Figure()
    fig.add_trace(go.Box(
        x=datos["vuelta"], y=datos["dist"], name="dist_derrape",
        # La caja son los cuartiles (el derrape "normal" de la vuelta) y los
        # puntos sueltos los valores atípicos: los picos que abren un derrape
        boxpoints="outliers", showlegend=False,
        line=dict(color=COL_SERIE_1, width=1.5),
        fillcolor="rgba(42, 120, 214, 0.15)",
        # Con boxpoints="outliers" los ÚNICOS puntos que se dibujan son los
        # atípicos, así que marker.color es el color de los atípicos (rojo, que
        # es lo que son: los picos que abren un derrape). marker.outliercolor
        # NO vale aquí: solo lo mira boxpoints="suspectedoutliers".
        marker=dict(color=COL_CRITICO, size=4),
    ))
    vs = sorted(tiempos_por_vuelta)
    fig.add_trace(go.Scatter(
        x=vs, y=[tiempos_por_vuelta[v] for v in vs], mode="lines+markers",
        name="tiempo de vuelta", yaxis="y2",
        line=dict(color=COL_SERIO, width=2), marker=dict(size=5),
        hovertemplate="v%{x}: %{y:.3f} s<extra>tiempo</extra>",
    ))
    fig.add_hline(y=umbral, line=dict(color=COL_TINTA, width=1),
                  annotation_text=f"umbral_derrape = {umbral:.0f} px",
                  annotation_font_color=COL_TINTA_2)
    # Ejes con nombre, no update_yaxes: si no, el eje del tiempo heredaría el
    # título y el rango del derrape (mismo motivo que en la 3b y la 3c)
    fig.update_layout(
        xaxis=dict(title_text="vuelta", dtick=5),
        yaxis=dict(title_text="dist_derrape (px)", rangemode="tozero"),
        yaxis2=dict(
            title=dict(text="tiempo de vuelta (s)", font=dict(color=COL_SERIO)),
            tickfont=dict(color=COL_SERIO), overlaying="y", side="right",
            showgrid=False,
        ),
    )
    return _layout_base(
        fig, "5c · Distribución del derrape y tiempo, vuelta a vuelta "
             "(caja = cuartiles; puntos rojos = valores atípicos)", 460)


# ===========================================================================
# FIGURA 6c: resumen por vuelta (4 paneles)
# ===========================================================================
#   1. nº de derrapes por vuelta (barras rojas: son el evento "malo")
#   2. tiempo por vuelta (marcador verde = al cerrarla el perfil subió,
#      gris = no subió; la última vuelta no tiene cierre y no aparece)
#   3. PWM medio: del perfil (toda la cadena) y el realmente aplicado
#      en los frames de la vuelta (donde pasó el coche)
#   4. velocidad media medida (px/s, filtrando saltos y huecos)
# ===========================================================================
def grafica_6c_resumen(tabla: pd.DataFrame):
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
    fig = _layout_base(fig, "6c · Resumen por vuelta", 640)
    # La leyenda (solo las dos series de PWM) centrada entre los títulos de
    # los dos paneles de arriba, que están a x~0.22 y x~0.78
    fig.update_layout(legend=dict(x=0.5, xanchor="center"))
    return fig


# Un panel de 6c aislado, para poder exportarlo a PDF por separado (pedido:
# en la memoria interesa poder incluir cada subplot suelto). Mismas trazas
# que su panel en grafica_6c_resumen, pero como figura independiente.
PANELES_6C = ("derrapes", "tiempo", "pwm", "velocidad")


def grafica_6c_subplot(tabla: pd.DataFrame, panel: str):
    if panel not in PANELES_6C:
        raise ValueError(f"panel debe ser uno de {PANELES_6C}, no {panel!r}")
    fig = go.Figure()
    if panel == "derrapes":
        fig.add_trace(go.Bar(
            x=tabla["vuelta"], y=tabla["n_derrapes"], marker_color=COL_CRITICO,
            hovertemplate="v%{x}: %{y} derrapes<extra></extra>",
        ))
        fig.update_yaxes(rangemode="tozero")
        titulo = "derrapes por vuelta"
    elif panel == "tiempo":
        colores = [COL_BUENO if s else COL_MUTED for s in tabla["sube"].fillna(False)]
        fig.add_trace(go.Scatter(
            x=tabla["vuelta"], y=tabla["duracion"], mode="lines+markers",
            line=dict(color=COL_SERIE_1, width=2), marker=dict(size=8, color=colores),
            text=["perfil subió al cerrarla" if s else "el perfil no subió"
                  for s in tabla["sube"].fillna(False)],
            hovertemplate="v%{x}: %{y:.2f} s<br>%{text}<extra></extra>",
        ))
        titulo = "tiempo por vuelta (s)"
    elif panel == "pwm":
        fig.add_trace(go.Scatter(x=tabla["vuelta"], y=tabla["pwm_perfil_medio"],
                                 mode="lines+markers", name="perfil (media de celdas)",
                                 line=dict(color=COL_SERIE_2, width=2), marker=dict(size=6)))
        fig.add_trace(go.Scatter(x=tabla["vuelta"], y=tabla["pwm_aplicado_medio"],
                                 mode="lines+markers", name="aplicado en carrera",
                                 line=dict(color=COL_SERIE_1, width=2), marker=dict(size=6)))
        titulo = "PWM medio"
    else:  # velocidad
        fig.add_trace(go.Scatter(
            x=tabla["vuelta"], y=tabla["vel_media"], mode="lines+markers",
            line=dict(color=COL_SERIE_1, width=2), marker=dict(size=6),
            hovertemplate="v%{x}: %{y:.0f} px/s<extra></extra>",
        ))
        titulo = "velocidad media (px/s)"
    fig.update_xaxes(title_text="vuelta", dtick=5)
    return _layout_base(fig, f"6c · {titulo}", 420)


# ===========================================================================
# FIGURA 4 (la más importante): el circuito con el PWM de cada celda
# ===========================================================================
# Todas las cámaras en el MISMO plano (plano global, ver OFFSETS_CAMARAS):
# la trayectoria de fondo en gris tenue y encima cada celda pintada del COLOR
# DE SU ZONA, con la leyenda al lado diciendo qué zona es cada color y con qué
# PWM corrió.
#
# El color es la IDENTIDAD de la zona, no su valor: una zona conserva su color
# desde que nace hasta que muere o la absorbe otra, aunque por el camino la
# castiguen y le bajen el PWM. Ese es el objetivo de la figura: con el slider
# instantáneo (cada paso pinta el perfil con el que se corrió esa vuelta) se ve
# NACER, CRECER, FUSIONARSE y MORIR a los tramos. Colorear por valor —como se
# hacía antes— no dejaba verlo: un castigo cambiaba el color de la zona, y dos
# zonas distintas con el mismo PWM salían del mismo color y parecían una sola.
# La identidad (`id`) la deriva seguir_zonas() en parseo_log.py, porque el log
# no numera las zonas.
#
# La FORMA del marcador acompaña al color y dice lo mismo que él: qué zona es.
# El VALOR de PWM (y si subió o bajó en esta vuelta) va en el texto de la
# leyenda, que es donde se mira cuando hace falta; el dibujo se queda solo con
# lo que interesa de un vistazo: qué tramos hay y dónde empiezan y acaban.
#
# datos_camaras: {camara: {"c": Carrera, "vueltas": [v...],
#                          "perfil_vuelta": {v: array PWM por celda},
#                          "zonas_ini_vuelta": {v: [zona con id y delta]}}}
# ===========================================================================
# Reserva de colores de zona: MUY distintos entre sí, porque dos zonas vecinas
# de colores parecidos se leerían como una sola. No se indexa por PWM ni por
# número de zona: los colores se REPARTEN (el primero libre al nacer una zona,
# devuelto a la reserva cuando muere), así que hacen falta tantos como zonas
# vivas a la vez pueda haber, no tantas como zonas salgan en toda la carrera.
PALETA_ZONAS = [
    "#2a78d6",  # azul
    "#d03b3b",  # rojo
    "#1baf7a",  # aqua
    "#b58324",  # ámbar
    "#7b5ea7",  # violeta
    "#0ca30c",  # verde
    "#ec835a",  # naranja
    "#00838f",  # teal oscuro
    "#c2185b",  # magenta oscuro
    "#8d6e63",  # marrón
    "#5c6bc0",  # índigo
    "#9e9d24",  # oliva
    "#e91e63",  # rosa fuerte
    "#009688",  # verde azulado
    "#795548",  # marrón oscuro
    "#3f51b5",  # azul oscuro
    "#ff7043",  # coral
    "#607d8b",  # gris azulado
]

# Formas del marcador, que acompañan al color: la forma también es IDENTIDAD de
# la zona, no un segundo dato encima. Con las dos cosas a la vez se reconoce un
# tramo de un vistazo (y se distingue igual de bien en un PDF en blanco y negro
# o para quien no separe dos colores). Van rellenas y son de silueta muy
# distinta entre sí, que a tamaño 11 px es lo único que se lee.
SIMBOLOS_ZONA = [
    "circle",
    "square",
    "diamond",
    "triangle-up",
    "star",
    "hexagon",
    "triangle-down",
    "pentagon",
]


class _ReservaEstilos:
    """Reparte estilos (color + forma) entre las zonas vivas y recoge el de las
    que mueren para volver a darlo más adelante.

    Cada zona ocupa un HUECO mientras vive, y del número de hueco salen su color
    (PALETA_ZONAS) y su forma (SIMBOLOS_ZONA): así conserva las dos cosas desde
    que nace hasta que muere o la absorbe otra, y las zonas que conviven en una
    vuelta ocupan huecos distintos y no se confunden.

    La clave de una zona es (cámara, id): los ids son por cámara y no se
    reutilizan nunca, así que una zona que muere y otra que nace después jamás
    comparten clave aunque acaben compartiendo hueco."""

    def __init__(self):
        self.de_zona = {}     # (cam, id) -> número de hueco
        self.aviso_dado = False

    def liberar_muertas(self, vivas):
        for clave in [k for k in self.de_zona if k not in vivas]:
            del self.de_zona[clave]

    def estilo(self, clave):
        """(color, forma) de la zona; se le asigna el primer hueco libre la
        primera vez que aparece."""
        if clave not in self.de_zona:
            usados = set(self.de_zona.values())
            libres = [i for i in range(len(PALETA_ZONAS)) if i not in usados]
            if libres:
                self.de_zona[clave] = libres[0]
            else:
                # Caso raro (más zonas vivas a la vez que huecos): se reparte
                # por módulo y dos zonas repetirán estilo. No se soporta con más
                # maquinaria; basta con avisar y ampliar PALETA_ZONAS si pasa.
                if not self.aviso_dado:
                    print(f"AVISO: más de {len(PALETA_ZONAS)} zonas vivas a la "
                          "vez en la gráfica 4: hay colores repetidos "
                          "(ampliar PALETA_ZONAS en figuras.py)")
                    self.aviso_dado = True
                self.de_zona[clave] = len(self.de_zona) % len(PALETA_ZONAS)
        hueco = self.de_zona[clave]
        return (PALETA_ZONAS[hueco % len(PALETA_ZONAS)],
                SIMBOLOS_ZONA[hueco % len(SIMBOLOS_ZONA)])


def grafica_4_circuito(datos_camaras, df_pos=None, marco=None):
    camaras = sorted(datos_camaras)
    indice = {c: i for i, c in enumerate(camaras)}
    # Eje de vueltas del slider: la unión de las vueltas de todas las
    # cámaras (con varias cámaras alguna puede perderse una vuelta)
    vueltas_global = sorted({v for d in datos_camaras.values() for v in d["vueltas"]})

    fig = go.Figure()
    n_estaticas = 0  # trazas siempre visibles (fondos), van primero
    # Todos los puntos del trazado (de todas las cámaras): con ellos se sitúa
    # el centro del circuito y su tamaño, que es lo que decide hacia dónde y
    # cuánto se aparta la flecha del sentido de la marcha
    trazado_x, trazado_y = [], []

    # --- Fondos por cámara: trayectoria gris + huecos tapados (gigantes) ---
    for i, cam in enumerate(camaras):
        dx, dy = offset_camara(cam, i)
        celdas = datos_camaras[cam]["c"].celdas.sort_values("celda").reset_index(drop=True)
        tx, ty = [], []
        gx, gy, gt = [], [], []
        for j, fila in celdas.iterrows():
            if fila["gigante"]:
                # La trayectoria se corta en la gigante y el hueco se dibuja
                # aparte como línea ámbar discontinua entre sus vecinas
                tx.append(None)
                ty.append(None)
                ant = celdas.iloc[j - 1] if j > 0 else None
                sig = celdas.iloc[j + 1] if j + 1 < len(celdas) else None
                if ant is not None and sig is not None and not ant["gigante"] \
                        and not sig["gigante"]:
                    gx += [ant["x"] + dx, sig["x"] + dx, None]
                    gy += [ant["y"] + dy, sig["y"] + dy, None]
                    texto = (f"{cam} · celda gigante {int(fila['celda'])}: "
                             f"hueco tapado de {fila['long_px']:.0f} px")
                    gt += [texto, texto, ""]
            else:
                tx.append(fila["x"] + dx)
                ty.append(fila["y"] + dy)
        # En cadena cerrada se une la última celda con la primera
        normales = celdas[~celdas["gigante"]]
        if datos_camaras[cam]["c"].cerrada and len(normales) > 1:
            tx.append(normales.iloc[0]["x"] + dx)
            ty.append(normales.iloc[0]["y"] + dy)
        fig.add_trace(go.Scatter(
            x=tx, y=ty, mode="lines", name=f"trayectoria {cam}",
            line=dict(color=COL_GRID, width=2), hoverinfo="skip",
            showlegend=(i == 0), legendgroup="tray",
        ))
        fig.add_trace(go.Scatter(
            x=gx, y=gy, mode="lines", name="hueco tapado (gigante)",
            line=dict(color=COL_AVISO, width=2, dash="dash"),
            text=gt, hovertemplate="%{text}<extra></extra>",
            showlegend=(i == 0 and len(gx) > 0), legendgroup="gigante",
        ))
        n_estaticas += 2
        trazado_x += [x for x in tx if x is not None]
        trazado_y += [y for y in ty if y is not None]

    # Centro y tamaño del circuito, para apartar la flecha de sentido hacia
    # fuera del trazado. Si viene `marco` (el encuadre común con la gráfica 2)
    # se usa ese, para que la flecha caiga dentro del cuadro; si no, la caja
    # del propio trazado, que es lo que los ejes van a autoescalar.
    if marco is not None:
        mx0, mx1, my0, my1 = marco
        centro_trazado = ((mx0 + mx1) / 2, (my0 + my1) / 2)
        separacion_flecha = max(mx1 - mx0, my1 - my0) * SEPARACION_FLECHA_SENTIDO
    else:
        centro_trazado = ((min(trazado_x) + max(trazado_x)) / 2,
                          (min(trazado_y) + max(trazado_y)) / 2) \
            if trazado_x else (0.0, 0.0)
        separacion_flecha = (max(max(trazado_x) - min(trazado_x),
                                 max(trazado_y) - min(trazado_y))
                             * SEPARACION_FLECHA_SENTIDO) if trazado_x else 0.0

    # Posición de cada celda normal en el plano global, por cámara: se calcula
    # una vez y la reutilizan todas las vueltas ({celda: (x, y)})
    puntos_celda = {}
    for i, cam in enumerate(camaras):
        dx, dy = offset_camara(cam, i)
        normales = datos_camaras[cam]["c"].celdas
        normales = normales[~normales["gigante"]]
        puntos_celda[cam] = {int(cd["celda"]): (cd["x"] + dx, cd["y"] + dy)
                             for _, cd in normales.iterrows()}

    # --- Un bloque de trazas por vuelta: UNA TRAZA POR ZONA ----------------
    # Una traza por zona (y no por valor) es lo que da el estilo estable: la
    # traza de la zona 3 lleva el color y la forma de la zona 3 en todas las
    # vueltas en las que exista. La leyenda solo lista las trazas visibles, así
    # que se actualiza sola con el slider y siempre dice las zonas de la vuelta
    # que se está mirando, con su PWM. El número de trazas cambia de una vuelta
    # a otra (cada vuelta tiene sus zonas), así que se apuntan los índices para
    # armar luego el slider.
    reserva = _ReservaEstilos()
    varias_camaras = len(camaras) > 1
    indices_por_vuelta = {}
    for v in vueltas_global:
        # Zonas vigentes en esta vuelta. Si una cámara no tiene datos de la
        # vuelta v se usa la última vuelta anterior que sí tenga (su perfil
        # sigue vigente).
        zonas_vuelta = {}   # cam -> [zona]
        for cam in camaras:
            d = datos_camaras[cam]
            v_datos = max((u for u in d["vueltas"] if u <= v), default=None)
            if v_datos is not None:
                zonas_vuelta[cam] = d["zonas_ini_vuelta"].get(v_datos, [])
        # Primero se devuelven a la reserva los huecos de las zonas que ya no
        # están (así una zona que nace en esta misma vuelta puede quedarse el
        # estilo de la que acaba de morir), y luego se reparten los que faltan
        vivas = {(cam, z["id"]) for cam, zs in zonas_vuelta.items() for z in zs}
        reserva.liberar_muertas(vivas)

        indices = []
        # Por cámara y por celda de inicio: la leyenda se lee siguiendo el
        # recorrido del circuito
        ordenadas = sorted(
            ((cam, z) for cam, zs in zonas_vuelta.items() for z in zs),
            key=lambda par: (indice[par[0]], par[1]["ini"]))
        con_zona = {cam: set() for cam in camaras}
        for cam, z in ordenadas:
            xs, ys, hover = [], [], []
            # celdas_de_zona, y no range(ini, fin+1), porque una zona puede
            # ENVOLVER la meta en cadena cerrada (fin < ini): ver
            # unir_por_la_meta en parseo_log.py
            for idx in celdas_de_zona(z, datos_camaras[cam]["c"].n_celdas):
                punto = puntos_celda[cam].get(idx)
                if punto is None:   # celda gigante (sin punto) o fuera de rango
                    continue
                con_zona[cam].add(idx)
                xs.append(punto[0])
                ys.append(punto[1])
                hover.append(f"{cam} · celda {idx}")
            if not xs:
                continue
            delta = z.get("delta")
            cambio = ("nace en esta vuelta" if delta is None else
                      "el PWM no cambió en esta vuelta" if delta == 0 else
                      f"PWM {delta:+d} respecto a la vuelta anterior")
            # Si el valor se movió, la leyenda lo dice en texto plano al lado
            # del número: es el único sitio donde hace falta mirarlo, y así el
            # dibujo se queda solo con lo que interesa de un vistazo (qué zonas
            # hay y dónde empiezan y acaban)
            marca = ("" if delta == 0 else
                     " (nueva)" if delta is None else f" ({delta:+d})")
            # En la leyenda la cámara solo se nombra si hay más de una (los
            # ids de zona son POR CÁMARA: la Z3 de la 01 no es la de la 02)
            prefijo = f"cam{cam.split('_')[-1]} · " if varias_camaras else ""
            # La que envuelve la meta se nombra igual (de donde empieza a donde
            # acaba siguiendo la marcha) pero avisando, que si no el 69-46
            # parece un error
            meta = " (cruza meta)" if z.get("cruza_meta") else ""
            zona_txt = (f"Z{z['id']} · {z['ini']}-{z['fin']}{meta} "
                        f"· PWM {z['pwm']}")
            color, simbolo = reserva.estilo((cam, z["id"]))
            fig.add_trace(go.Scatter(
                x=xs, y=ys, mode="markers", name=prefijo + zona_txt + marca,
                marker=dict(size=11, symbol=simbolo, color=color,
                            line=dict(color=COL_SUPERFICIE, width=1)),
                customdata=[f"{h}<br>{zona_txt} ({z['tipo']})<br>{cambio}"
                            for h in hover],
                hovertemplate="%{customdata}<extra></extra>",
                visible=(v == vueltas_global[0]),
            ))
            indices.append(len(fig.data) - 1)

        # Celdas que ninguna zona cubre (sin volcado [PERFIL] todavía, o hueco
        # entre zonas): en gris y todas en una traza al final de la leyenda
        sx, sy, shover = [], [], []
        for cam in camaras:
            for idx, (x, y) in puntos_celda[cam].items():
                if idx not in con_zona[cam]:
                    sx.append(x)
                    sy.append(y)
                    shover.append(f"{cam} · celda {idx}<br>sin zona (sin PWM)")
        if sx:
            fig.add_trace(go.Scatter(
                x=sx, y=sy, mode="markers", name="sin zona",
                marker=dict(size=11, color=COL_MUTED,
                            line=dict(color=COL_SUPERFICIE, width=1)),
                customdata=shover,
                hovertemplate="%{customdata}<extra></extra>",
                visible=(v == vueltas_global[0]),
            ))
            indices.append(len(fig.data) - 1)

        # Inicio de meta: el primer punto capturado de la vuelta sobre el
        # circuito (mismo criterio que la estrella de la g2), para orientar
        # por dónde arranca. Se busca en las posiciones del bag. La numeración
        # de vueltas del log y la del bag (time_per_lap) pueden no coincidir
        # (p.ej. el log arranca en la vuelta 0 y el bag no la cronometró); si
        # la vuelta exacta no está, se cae a la PRIMERA posición del bag: la
        # meta es un punto ~fijo del circuito, así que el marcador sirve igual
        # de referencia y la vista por defecto nunca se queda sin él.
        if df_pos is not None and len(df_pos):
            fr = df_pos[df_pos["vuelta"] == v]
            primera = (fr if len(fr) else df_pos).iloc[0]
            dx, dy = offset_camara(
                primera["camara"], indice.get(primera["camara"], 0))
            fig.add_trace(go.Scatter(
                x=[primera["fx"] + dx], y=[primera["fy"] + dy],
                mode="markers", name="inicio de vuelta", showlegend=False,
                marker=dict(symbol="star", size=15, color=COL_INICIO_VUELTA,
                            line=dict(color=COL_SUPERFICIE, width=1)),
                text=[f"v{int(v)} · inicio de meta"],
                hovertemplate="%{text}<extra></extra>",
                visible=(v == vueltas_global[0]),
            ))
            indices.append(len(fig.data) - 1)

            # Y a su altura, apartada hacia fuera del circuito, la flecha del
            # sentido de la marcha (misma reserva de vuelta que la estrella).
            # Aquí el bloque por vuelta no es de tamaño fijo —se apuntan los
            # índices uno a uno—, así que si no hay tangente basta con no
            # añadirla.
            tangente = _tangente_meta(
                fr if len(fr) else df_pos, camaras, centro_trazado,
                separacion_flecha)
            if tangente is not None:
                fig.add_trace(_flecha_sentido_2d(
                    tangente, visible=(v == vueltas_global[0]),
                    centro=centro_trazado))
                indices.append(len(fig.data) - 1)
        indices_por_vuelta[v] = indices

    # Slider instantáneo: fondos siempre visibles + las trazas de su vuelta
    n_total = len(fig.data)
    pasos = []
    for v in vueltas_global:
        visibles = [False] * n_total
        for i in range(n_estaticas):
            visibles[i] = True
        for idx in indices_por_vuelta[v]:
            visibles[idx] = True
        # int(v): la etiqueta es la que viaja al servidor para sacar el PDF de
        # esta vuelta (ver script_pdf_vuelta en analisis.py), y con un float de
        # numpy saldría "3.0" y no casaría con la vuelta 3
        pasos.append(dict(label=str(int(v)), method="update",
                          args=[{"visible": visibles}]))
    fig.update_layout(
        sliders=[dict(active=0, currentvalue=dict(prefix="Vuelta "),
                      pad=dict(t=30), steps=pasos)],
    )
    # Coordenadas de imagen: origen arriba a la izquierda (Y invertida) y misma
    # escala en ambos ejes para que el circuito no salga deformado. El `dtick`
    # común a los dos ejes es lo que hace la cuadrícula de celdas cuadradas
    # (ver paso_rejilla) y, con `marco`, el encuadre y la rejilla salen
    # idénticos a los de la gráfica 2.
    if marco is not None:
        mx0, mx1, my0, my1 = marco
        paso_grid = paso_rejilla(mx0, mx1, my0, my1)
        fig.update_yaxes(range=[my1, my0], scaleanchor="x", scaleratio=1,
                         dtick=paso_grid, tick0=0,
                         title_text="y (px)")
        fig.update_xaxes(range=[mx0, mx1], dtick=paso_grid, tick0=0,
                         title_text="x (px)")
    else:
        # Sin marco los ejes se autoescalan; el paso sale de la caja del trazado
        paso_grid = paso_rejilla(min(trazado_x), max(trazado_x),
                                 min(trazado_y), max(trazado_y)) \
            if trazado_x else None
        fig.update_yaxes(autorange="reversed", scaleanchor="x", scaleratio=1,
                         dtick=paso_grid, tick0=0,
                         title_text="y (px)")
        fig.update_xaxes(dtick=paso_grid, tick0=0,
                         title_text="x (px)")
    fig = _layout_base(
        fig,
        "4 · El circuito con las zonas de PWM "
        "(cada zona con su color y su forma mientras vive)",
        720,
    )
    # Leyenda VERTICAL a la derecha, DESPUÉS de _layout_base (que pone la
    # horizontal común a todas las figuras): aquí hay una entrada por zona
    # viva, con nombre largo, y en horizontal se comerían media figura. El
    # margen derecho crece para dejarle sitio.
    fig.update_layout(
        legend=dict(orientation="v", yanchor="top", y=1, xanchor="left",
                    x=1.02, font=dict(size=11)),
        margin=dict(l=70, r=210, t=95, b=60),
    )
    return fig


# ---------------------------------------------------------------------------
# Colores de las trayectorias de las pegatinas (los de las figuras gnuplot
# de la memoria de Mario, para que las de esta memoria se parezcan)
# ---------------------------------------------------------------------------
COL_ETIQ_DELANTERA = "#009E73"  # verde etiqueta delantera (fig. 5.17)
COL_ETIQ_TRASERA = "#CC00CC"    # magenta etiqueta trasera (fig. 5.17)
COL_TRAY_BASE_3D = "#9400D3"    # violeta de la línea "Posicion" (fig. 5.12)
# Azul de serie para la trayectoria BASE aprendida (las celdas del log): es
# la referencia del algoritmo, y en azul no se confunde con el magenta de la
# trasera ni con el verde de la delantera
COL_TRAY_BASE = COL_SERIE_1
# Ámbar para el punto de arranque de la vuelta (el que sigue al cruce de meta)
COL_INICIO_VUELTA = COL_AVISO

# Salto en píxeles entre dos posiciones consecutivas a partir del cual la
# línea de trayectoria se corta (None) en vez de unirlas: o es un cambio de
# cámara, o una detección falsa, o el coche reapareció por otro lado
SALTO_CORTE_TRAZO = 100.0

# Altura de la caja 3D de la figura 1 respecto al lado mayor del circuito.
# El derrape mide decenas de píxeles y el circuito más de mil: a escala real
# el relieve sería invisible, así que el eje z se exagera hasta esta
# fracción (0.35 = un tercio del ancho). Subirlo hace los picos más
# aparatosos; bajarlo, la vista más plana.
ALTURA_RELATIVA_Z = 0.35


def _con_cortes(df, col_x, col_y, camaras):
    """Listas x, y para una traza de línea con None donde no hay que unir:
    cambio de cámara o salto > SALTO_CORTE_TRAZO px. Aplica el offset del
    plano global de cada cámara. df debe venir ordenado por tiempo."""
    xs, ys = [], []
    cam_prev = x_prev = y_prev = None
    indice = {c: i for i, c in enumerate(camaras)}
    for fila in df.itertuples():
        cam = fila.camara
        dx, dy = offset_camara(cam, indice.get(cam, 0))
        x, y = getattr(fila, col_x) + dx, getattr(fila, col_y) + dy
        if cam_prev is not None and (
                cam != cam_prev
                or abs(x - x_prev) + abs(y - y_prev) > SALTO_CORTE_TRAZO):
            xs.append(None)
            ys.append(None)
        xs.append(x)
        ys.append(y)
        cam_prev, x_prev, y_prev = cam, x, y
    return xs, ys


def _rangos_globales(df, camaras, extra=()):
    """(x_min, x_max, y_min, y_max) del plano global, con un 2 % de margen.

    Recorre las DOS pegatinas de todas las cámaras aplicando su offset, más
    los arrays `extra` (Nx2, ya en coordenadas globales) que haya que acotar
    (la trayectoria base). Fijar los ejes con estos rangos es lo que hace que
    el encuadre NO cambie al pasar de vuelta: si cada vuelta se autoescalara,
    el circuito se estiraría o achataría según por dónde pasó el coche y dos
    vueltas dejarían de ser comparables a ojo."""
    xs, ys = [], []
    for i, cam in enumerate(camaras):
        dx, dy = offset_camara(cam, i)
        sub = df[df["camara"] == cam]
        xs += list(pd.concat([sub["fx"], sub["bx"]]).dropna() + dx)
        ys += list(pd.concat([sub["fy"], sub["by"]]).dropna() + dy)
    for pts in extra:
        xs += list(pts[:, 0])
        ys += list(pts[:, 1])
    if not xs:
        return 0.0, 1.0, 0.0, 1.0
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    margen_x = (x_max - x_min) * 0.02 or 1.0
    margen_y = (y_max - y_min) * 0.02 or 1.0
    return x_min - margen_x, x_max + margen_x, y_min - margen_y, y_max + margen_y


# Cuánto se infla el marco por cada lado, en fracción del LADO MAYOR de la caja
# de los datos. Tiene que dar sitio a lo que se dibuja FUERA del trazado: la
# flecha de sentido, que se aparta SEPARACION_FLECHA_SENTIDO (7 %), y su
# etiqueta, que _puntos_con_holgura cuelga un 15 % más lejos del centro. Se
# infla lo MISMO en píxeles en los dos ejes (por eso sale del lado mayor y no
# del lado de cada eje): así el marco no estira ni achata el circuito.
HOLGURA_MARCO = 0.12


def marco_comun(df, camaras, extras=()):
    """(x_min, x_max, y_min, y_max) del plano global, con holgura: el encuadre
    COMÚN de las gráficas 2 y 4.

    Común porque las dos enseñan el mismo circuito: con el mismo marco sale en
    la misma posición y a la misma escala en ambas, y se pueden mirar (o
    imprimir) una al lado de la otra. La holgura evita tener que recalcular el
    rango al final de cada figura para que quepan la flecha y su etiqueta.

    El marco NO se cuadra a propósito: cuadrarlo obligaba a encoger el área de
    dibujo hasta el alto de la figura y las gráficas salían pequeñas. Lo que
    hace que la cuadrícula tenga celdas cuadradas es la escala 1:1 (scaleanchor)
    más el mismo paso de rejilla en los dos ejes; de eso se encarga
    paso_rejilla()."""
    x0, x1, y0, y1 = _rangos_globales(df, camaras, extra=extras)
    holgura = max(x1 - x0, y1 - y0) * HOLGURA_MARCO
    return x0 - holgura, x1 + holgura, y0 - holgura, y1 + holgura


def paso_rejilla(x0, x1, y0, y1):
    """Separación entre líneas de la rejilla (px del plano global), la MISMA
    para el eje X y el eje Y.

    Es la pieza que hace que la cuadrícula sea de celdas CUADRADAS: los ejes ya
    van a la misma escala (scaleanchor), así que en cuanto los dos parten con el
    mismo paso, un salto de 200 px mide lo mismo a lo ancho que a lo alto. Sin
    esto plotly elige el paso de cada eje por su cuenta (X cada 500, Y cada 200)
    y la rejilla sale de rectángulos.

    Se calcula en vez de fijarlo porque cada circuito ocupa un número de píxeles
    distinto: se apunta a una decena de divisiones a lo largo del lado mayor y
    se redondea al número redondo más cercano (1, 2 o 5 por potencia de diez),
    que son los que dan etiquetas de eje legibles (100, 200, 500...)."""
    objetivo = max(x1 - x0, y1 - y0) / 10 or 1.0
    potencia = 10 ** math.floor(math.log10(objetivo))
    for redondo in (1, 2, 5, 10):
        if objetivo <= redondo * potencia:
            return redondo * potencia
    return 10 * potencia


def _pie_perpendicular(p, pts, cerrada=False):
    """Pie de la perpendicular de `p` sobre la polilínea `pts` (Nx2) y su
    distancia, replicando el PASO FINO de `localizar()` del algoritmo:
    punto más cercano (argmin) y luego proyección acotada sobre los dos
    segmentos adyacentes (`_dist_a_segmento` en AlgoritmoVelocidad.py).

    `cerrada` replica la envoltura de la cadena cerrada: sin ella, un punto
    que caiga justo en la COSTURA (entre la última celda y la primera) se
    quedaría sin uno de sus dos segmentos y la distancia saldría de más —
    medida a un extremo en vez de al tramo que de verdad tiene al lado.

    OJO: cuando `localizar` puede formar un trío de celdas consecutivas usa
    una curva local C¹ en vez de los segmentos rectos, así que su valor puede
    diferir en décimas del que se dibuja aquí. Justo por eso interesa ver los
    dos números juntos (el calculado aquí y el dist_derrape que el
    controlador publicó en el bag): si se separan mucho, el cálculo del
    algoritmo es lo que hay que mirar."""
    n = len(pts)
    d2 = np.sum((pts - p) ** 2, axis=1)
    i = int(np.argmin(d2))
    mejor = float(np.sqrt(d2[i]))
    pie = pts[i]
    for a, b in ((i - 1, i), (i, i + 1)):
        if cerrada:
            a, b = a % n, b % n
        elif a < 0 or b >= n:
            continue
        A, B = pts[a], pts[b]
        AB = B - A
        l2 = float(np.dot(AB, AB))
        if l2 == 0.0:
            continue
        t = max(0.0, min(1.0, float(np.dot(p - A, AB)) / l2))
        proyeccion = A + t * AB
        d = float(np.linalg.norm(p - proyeccion))
        if d < mejor:
            mejor, pie = d, proyeccion
    return pie, mejor


# ===========================================================================
# FIGURA 1: evolución 3D del derrape sobre la trayectoria (fig. 5.12 de
# la memoria de Mario)
# ===========================================================================
# Plano XY = el plano global en píxeles, con la trayectoria de la pegatina
# DELANTERA como línea base en z=0 ("Posicion" en la figura de Mario).
# Eje Z y color = distancia de derrape de la pegatina TRASERA en cada punto
# (el dist_derrape de la telemetría emparejado con su posición).
#
# Slider INSTANTÁNEO: cada paso enseña SOLO los puntos de esa vuelta (la
# línea base delantera queda siempre visible). Los ejes tienen rango FIJO
# (calculado de toda la carrera): la caja no cambia de tamaño ni de posición
# al pasar de vuelta, así se comparan. La vista se rota con el ratón.
#
# df: DataFrame ordenado por t con columnas
#     [t, vuelta, camara, fx, fy, bx, by, dist]  (dist NaN si la telemetría
#     no se emparejó con esa posición; esos puntos no se dibujan en 3D)
# ===========================================================================
def grafica_1_derrape3d(df):
    camaras = sorted(df["camara"].unique())
    vueltas = sorted(df.loc[df["vuelta"].notna(), "vuelta"].unique())
    indice = {c: i for i, c in enumerate(camaras)}

    fig = go.Figure()

    # Línea base: la trayectoria de la delantera durante UNA vuelta completa
    # (la primera con datos), que dibuja el circuito limpio en z=0. Si no
    # hay vueltas cronometradas se usa todo el recorrido.
    base = df[df["vuelta"] == vueltas[0]] if vueltas else df
    bx, by = _con_cortes(base, "fx", "fy", camaras)
    fig.add_trace(go.Scatter3d(
        x=bx, y=by, z=[0] * len(bx), mode="lines", name="Posición",
        line=dict(color=COL_TRAY_BASE_3D, width=3), hoverinfo="skip",
    ))

    # Rangos del plano: aquí arriba se usan para dimensionar y colocar la
    # flecha de sentido (tamaño y separación salen del lado mayor del
    # circuito, y el desplazamiento hacia fuera, del centro). Los rangos
    # DEFINITIVOS de la caja se recalculan al final incluyendo las flechas,
    # que caen fuera del trazado y si no quedarían recortadas.
    gx_min, gx_max, gy_min, gy_max = _rangos_globales(df, camaras)
    mayor_plano = max(gx_max - gx_min, gy_max - gy_min)
    largo_flecha = mayor_plano * 0.03
    centro_plano = ((gx_min + gx_max) / 2, (gy_min + gy_max) / 2)
    puntos_flecha = []

    # Un bloque de TRAZAS por vuelta: los puntos de derrape y, encima, la
    # ESTRELLA de inicio de meta (mismo criterio que la g2: el primer punto
    # capturado de la vuelta, sobre el plano base en z=0) con su FLECHA de
    # sentido de la marcha. Son 3 trazas por vuelta; el slider las enciende
    # juntas.
    TRAZAS_POR_VUELTA_1 = 3
    con_dist = df[df["dist"].notna() & df["vuelta"].notna()]
    z_max = max(float(con_dist["dist"].max()) if len(con_dist) else 1.0, 1.0)
    for v in vueltas:
        fr = con_dist[con_dist["vuelta"] == v]
        xs = [f.bx + offset_camara(f.camara, indice[f.camara])[0]
              for f in fr.itertuples()]
        ys = [f.by + offset_camara(f.camara, indice[f.camara])[1]
              for f in fr.itertuples()]
        fig.add_trace(go.Scatter3d(
            x=xs, y=ys, z=fr["dist"], mode="markers",
            name=f"vuelta {int(v)}", showlegend=False,
            marker=dict(
                size=2.5, color=fr["dist"],
                # Rampa perceptual negro->morado->naranja->amarillo, como la
                # paleta de la figura gnuplot de Mario (Dist_derrape 0..14)
                colorscale="Inferno", cmin=0, cmax=z_max,
                colorbar=dict(title="Dist<br>derrape"),
            ),
            text=[f"v{int(v)} · {f.camara}<br>dist={f.dist:.1f} px"
                  for f in fr.itertuples()],
            hovertemplate="%{text}<extra></extra>",
            visible=(v == vueltas[0]),
        ))

        # Inicio de meta: el primer punto capturado de la vuelta (todo el df,
        # no solo los que tienen dist), dibujado a z=0 sobre el plano base
        # para orientar por dónde arranca el trazado.
        fr_vuelta = df[df["vuelta"] == v]
        primera = fr_vuelta.iloc[0]
        dx, dy = offset_camara(primera["camara"], indice[primera["camara"]])
        fig.add_trace(go.Scatter3d(
            x=[primera["fx"] + dx], y=[primera["fy"] + dy], z=[0],
            mode="markers", name="inicio de vuelta", showlegend=False,
            marker=dict(symbol="diamond", size=6, color=COL_INICIO_VUELTA,
                        line=dict(color=COL_SUPERFICIE, width=1)),
            text=[f"v{int(v)} · inicio de meta"],
            hovertemplate="%{text}<extra></extra>",
            visible=(v == vueltas[0]),
        ))

        # Y la flecha del sentido de la marcha, a la altura de la estrella pero
        # apartada hacia fuera del circuito. Si la vuelta no da tangente se
        # mete una traza vacía: el bloque por vuelta tiene que medir siempre
        # TRAZAS_POR_VUELTA_1 porque el slider direcciona las trazas por índice.
        tangente = _tangente_meta(
            fr_vuelta, camaras, centro_plano,
            mayor_plano * SEPARACION_FLECHA_SENTIDO)
        if tangente is None:
            fig.add_trace(go.Scatter3d(x=[], y=[], z=[], mode="lines",
                                       showlegend=False, hoverinfo="skip"))
        else:
            traza = _flecha_sentido_3d(
                tangente, largo_flecha, visible=(v == vueltas[0]),
                centro=centro_plano)
            fig.add_trace(traza)
            puntos_flecha.append(_puntos_con_holgura(traza, centro_plano))

    # Slider instantáneo: cada paso enseña SOLO su vuelta (la traza 0, la
    # línea base delantera, siempre visible; detrás, los trazos de la vuelta)
    pasos = []
    for k, v in enumerate(vueltas):
        visibles = [True] + [False] * (TRAZAS_POR_VUELTA_1 * len(vueltas))
        for j in range(TRAZAS_POR_VUELTA_1):
            visibles[1 + TRAZAS_POR_VUELTA_1 * k + j] = True
        pasos.append(dict(label=str(int(v)), method="update",
                          args=[{"visible": visibles}]))

    # Rangos FIJOS de la caja, ya con las flechas dentro: así el encuadre no
    # cambia al pasar de vuelta y las vueltas se pueden comparar
    gx_min, gx_max, gy_min, gy_max = _rangos_globales(
        df, camaras, extra=puntos_flecha)

    # Proporción de la caja 3D: x e y guardan la proporción real del plano
    # (el circuito no sale deformado) y z se lleva ALTURA_RELATIVA_Z del lado
    # mayor. Con aspectmode="data" el eje z (0..~20 px de derrape) quedaría
    # aplastado contra un plano de más de mil píxeles de ancho y no se vería
    # el relieve, que es justo lo que cuenta esta figura.
    rango_x = (gx_max - gx_min) or 1.0
    rango_y = (gy_max - gy_min) or 1.0
    mayor = max(rango_x, rango_y)

    fig.update_layout(
        sliders=[dict(active=0, currentvalue=dict(prefix="Vuelta "),
                      pad=dict(t=10), steps=pasos)],
        scene=dict(
            xaxis_title="x (px)",
            yaxis_title="y (px)",
            zaxis_title="dist. derrape (px)",
            # Rangos FIJOS. La Y va invertida (de mayor a menor) para que el
            # circuito se vea como en la imagen de la cámara. nticks bajo en
            # Y: es el lado corto de la caja y sus marcas se amontonarían
            xaxis=dict(range=[gx_min, gx_max], nticks=8),
            yaxis=dict(range=[gy_max, gy_min], nticks=5),
            zaxis=dict(range=[0, z_max * 1.05]),
            aspectmode="manual",
            aspectratio=dict(x=rango_x / mayor, y=rango_y / mayor,
                             z=ALTURA_RELATIVA_Z),
            # Punto de vista inicial parecido al de la figura 5.12 de Mario:
            # en diagonal y algo elevado, para ver a la vez el trazado y el
            # relieve del derrape. Cuanto más pequeño el vector eye, más
            # cerca queda la cámara (una caja alargada como esta se ve
            # diminuta con el valor por defecto, 1.25 en los tres ejes). En
            # pantalla se puede rotar a gusto: esto es solo el encuadre de
            # partida, que además es el que sale en el PDF.
            camera=dict(eye=dict(x=1.0, y=1.0, z=0.6)),
        ),
    )
    fig = _layout_base(
        fig,
        "1 · Valor de derrape en cada punto del recorrido "
        "(una vuelta cada vez, ejes fijos; arrastrar para rotar)",
        720,
    )
    # Los márgenes de _layout_base son para figuras 2D (dejan sitio al
    # título del eje Y); en 3D la escena los desaprovecha y sale pequeña
    fig.update_layout(margin=dict(l=10, r=10, t=70, b=10))
    return fig


# ===========================================================================
# FIGURA 2: trayectorias de la etiqueta delantera y trasera (fig. 5.17 de
# la memoria de Mario)
# ===========================================================================
# Vista 2D del plano global. Slider INSTANTÁNEO: cada paso enseña SOLO la
# vuelta seleccionada, y con los EJES FIJOS (misma escala en todas), así se
# ve cómo cambia el trazado de una vuelta a la siguiente sin que el
# autoescalado engañe estirando o achatando el circuito.
#
# Qué lleva la vuelta activa (todo visible por defecto; los CHECKBOXES del
# HTML quitan/ponen cada capa, el JS de la sección 2 en analisis.py usa los
# índices de traza que van en layout.meta):
#   - TRAYECTORIA BASE (azul): las celdas aprendidas, la referencia contra la
#     que el algoritmo mide de verdad
#   - pegatina DELANTERA (verde) y pegatina TRASERA (magenta): las dos a la vez
#   - INICIO de la vuelta (estrella ámbar, SIN checkbox): el primer punto tras
#     el cruce de meta, para saber por dónde empieza a leerse el trazado, con
#     la FLECHA del sentido de la marcha al lado (también sin checkbox)
#   - DISTANCIA DE DERRAPE (rojo): un segmento de la trasera al pie de su
#     perpendicular sobre la trayectoria base, calculado COMO LO CALCULA el
#     algoritmo (ver _pie_perpendicular). El valor (MAGNITUD, sin signo) va
#     SIEMPRE en el hover de la recta y sus extremos, junto al dist_derrape que
#     el controlador publicó en el bag, que es con quien se contrasta.
#
# Todas las series van con MARCADOR además de la línea: la polilínea sola no
# deja ver los puntos reales, que son con los que trabaja el controlador.
#
# df: el mismo DataFrame de grafica_1_derrape3d.
# celdas_por_camara: {camara: DataFrame de celdas del log}, o None si no hay
#     logs (entonces no hay trayectoria base: ni su checkbox ni las
#     perpendiculares).
# cerradas: {camara: bool} con si la cadena de esa cámara es cerrada; hace
#     falta para medir bien en la costura (ver _pie_perpendicular).
# umbral: umbral_derrape (px). Ya no decide nada visual (el valor va siempre
#     en el hover); se conserva por compatibilidad de la llamada.
# max_dist_ruta: px; los frames cuya DELANTERA esté más lejos de la ruta se
#     saltan, porque el algoritmo los descarta enteros (paso 1 de
#     actualizar_estado) y nunca llegan a producir un dist_derrape. Sin este
#     filtro la figura se llena de segmentos larguísimos hacia detecciones
#     falsas que el controlador ni miró.
# ===========================================================================
# Trazas que lleva CADA vuelta, en este orden: trasera, delantera, inicio,
# perpendiculares y flecha de sentido. El bloque es de tamaño fijo (aunque
# alguna vaya vacía) porque el slider y el JS de los checkboxes direccionan las
# trazas por índice; por eso la flecha se añadió AL FINAL, para no mover los
# índices que ya usaban las capas con checkbox.
TRAZAS_POR_VUELTA_2 = 5


def grafica_2_trayectorias(df, celdas_por_camara=None, cerradas=None,
                           umbral=None, max_dist_ruta=None, marco=None):
    camaras = sorted(df["camara"].unique())
    vueltas = sorted(df.loc[df["vuelta"].notna(), "vuelta"].unique())
    indice = {c: i for i, c in enumerate(camaras)}

    # --- Trayectoria base aprendida (celdas del log) en el plano global ----
    # Es la referencia CONTRA LA QUE el algoritmo mide la distancia de la
    # trasera, así que sirve para las dos cosas: dibujarla y calcular las
    # perpendiculares. Las celdas gigantes no tienen punto (x/y NaN).
    base_por_camara = {}
    for cam in camaras:
        celdas = (celdas_por_camara or {}).get(cam)
        if celdas is None or celdas.empty:
            continue
        normales = celdas[~celdas["gigante"]].sort_values("celda")
        if len(normales) < 2:
            continue
        dx, dy = offset_camara(cam, indice[cam])
        base_por_camara[cam] = np.column_stack([
            normales["x"].to_numpy(dtype=float) + dx,
            normales["y"].to_numpy(dtype=float) + dy,
        ])
    cerrada_de = cerradas or {}
    hay_base = bool(base_por_camara)

    fig = go.Figure()

    # --- Estáticas: fantasmas de leyenda + la trayectoria base -------------
    # Las trazas de cada vuelta van con showlegend=False y la leyenda la
    # sostienen estos fantasmas: si la llevaran ellas, cambiaría al mover el
    # slider a otra vuelta.
    fantasmas = [
        ("Etiq. trasera", COL_ETIQ_TRASERA, "lines+markers"),
        ("Etiq. delantera", COL_ETIQ_DELANTERA, "lines+markers"),
        ("Inicio de vuelta y sentido", COL_INICIO_VUELTA, "markers"),
    ]
    if hay_base:
        fantasmas.append(("Dist. derrape (perpendicular)", COL_CRITICO, "lines"))
        fantasmas.append(("Trayectoria base (celdas)", COL_TRAY_BASE,
                          "lines+markers"))
    for nombre, color, modo in fantasmas:
        fig.add_trace(go.Scatter(
            x=[None], y=[None], mode=modo, name=nombre,
            line=dict(color=color, width=1.5),
            marker=dict(size=8, color=color), hoverinfo="skip",
        ))
    # Índices de los fantasmas de leyenda, para que cada checkbox del HTML
    # apague/encienda también su entrada de leyenda (orden de la lista de
    # arriba). Los de la base solo existen si hay_base.
    idx_fantasma_trasera = 0
    idx_fantasma_delantera = 1
    idx_fantasma_perp = 3 if hay_base else -1
    idx_fantasma_base = len(fantasmas) - 1 if hay_base else -1

    if hay_base:
        # La línea se CORTA en las celdas gigantes (huecos tapados: ahí no hay
        # puntos aprendidos) y entre cámaras. Es importante que el hueco se
        # vea: dentro de él la celda más cercana está lejísimos, así que la
        # perpendicular sale enorme, y sin el corte parecería un error de
        # cálculo en vez de "aquí la trayectoria base no existe".
        bxs, bys = [], []
        for cam in camaras:
            if cam not in base_por_camara:
                continue
            if bxs:  # None entre cámaras: no se unen dos porciones distintas
                bxs.append(None)
                bys.append(None)
            celdas = celdas_por_camara[cam].sort_values("celda")
            dx, dy = offset_camara(cam, indice[cam])
            for _, cd in celdas.iterrows():
                if cd["gigante"]:
                    bxs.append(None)
                    bys.append(None)
                else:
                    bxs.append(cd["x"] + dx)
                    bys.append(cd["y"] + dy)
        fig.add_trace(go.Scatter(
            x=bxs, y=bys, mode="lines+markers",
            name="Trayectoria base (celdas)",
            line=dict(color=COL_TRAY_BASE, width=1.5),
            marker=dict(size=6, color=COL_TRAY_BASE), showlegend=False,
            hovertemplate="celda base · (%{x:.0f}, %{y:.0f})<extra></extra>",
            # Arranca VISIBLE: por defecto se enseñan las tres capas a la vez
            # (base + delantera + trasera) y los checkboxes del HTML las quitan.
            visible=True,
        ))
    idx_base = len(fantasmas) if hay_base else -1
    n_estaticas = len(fantasmas) + (1 if hay_base else 0)

    # El marco (encuadre cuadrado, común con la gráfica 4) sitúa el centro del
    # circuito y su tamaño, que es lo que aparta la flecha de sentido fuera del
    # trazado. Sin marco se cae a la caja de los datos y, como antes, los
    # rangos definitivos se calculan al final incluyendo las propias flechas
    # (que si no quedarían recortadas).
    if marco is None:
        x0, x1, y0, y1 = _rangos_globales(
            df, camaras, extra=list(base_por_camara.values()))
    else:
        x0, x1, y0, y1 = marco
    centro_plano = ((x0 + x1) / 2, (y0 + y1) / 2)
    separacion = max(x1 - x0, y1 - y0) * SEPARACION_FLECHA_SENTIDO
    puntos_flecha = []

    # --- Un bloque de TRAZAS_POR_VUELTA_2 trazas por vuelta ----------------
    for v in vueltas:
        fr = df[df["vuelta"] == v]
        visible = v == vueltas[0]

        for col_x, col_y, nombre, color in (
            ("bx", "by", "Etiq. trasera", COL_ETIQ_TRASERA),
            ("fx", "fy", "Etiq. delantera", COL_ETIQ_DELANTERA),
        ):
            xs, ys = _con_cortes(fr, col_x, col_y, camaras)
            fig.add_trace(go.Scatter(
                x=xs, y=ys, mode="lines+markers", name=nombre,
                line=dict(color=color, width=1.5),
                # Marcadores grandes con borde: los nodos reales (con los que
                # trabaja el controlador) tienen que verse sobre la polilínea
                marker=dict(size=8, color=color,
                            line=dict(color=COL_SUPERFICIE, width=1)),
                showlegend=False,
                hovertemplate=f"{nombre} · v{int(v)}<br>" +
                              "(%{x:.0f}, %{y:.0f})<extra></extra>",
                visible=visible,
            ))

        # Inicio de la vuelta: el primer punto que llegó tras cruzar meta
        # (df viene en orden de grabación), en las dos pegatinas
        primera = fr.iloc[0]
        dx, dy = offset_camara(primera["camara"], indice[primera["camara"]])
        fig.add_trace(go.Scatter(
            x=[primera["fx"] + dx, primera["bx"] + dx],
            y=[primera["fy"] + dy, primera["by"] + dy],
            mode="markers", name="Inicio de vuelta", showlegend=False,
            marker=dict(symbol="star", size=14, color=COL_INICIO_VUELTA,
                        line=dict(color=COL_SUPERFICIE, width=1)),
            text=["inicio (delantera)", "inicio (trasera)"],
            hovertemplate=f"v{int(v)} · %{{text}}<extra></extra>",
            visible=visible,
        ))

        # Perpendiculares de la trasera a la trayectoria base: un segmento
        # por punto, todos en una traza separados por None. El VALOR ya no se
        # escribe al lado (saturaba la figura): va SIEMPRE en el hover, tanto
        # en la recta como en sus dos extremos (marcadores), que son diana
        # fácil para el ratón. Es una MAGNITUD, sin signo: el algoritmo mide
        # cuánto se aparta la trasera, no de qué lado (ver _dist_a_segmento).
        px, py, hovers = [], [], []
        for fila in fr.itertuples():
            pts = base_por_camara.get(fila.camara)
            if pts is None:
                continue
            dx, dy = offset_camara(fila.camara, indice[fila.camara])
            # Mismo filtro que el algoritmo: si la DELANTERA está a más de
            # max_dist_ruta de la ruta, el frame entero se descarta y nunca
            # llega a medirse el derrape. Dibujar esos puntos llenaría la
            # figura de segmentos larguísimos hacia detecciones falsas.
            if max_dist_ruta is not None:
                _, d_front = _pie_perpendicular(
                    np.array([fila.fx + dx, fila.fy + dy], dtype=float), pts,
                    cerrada_de.get(fila.camara, False))
                if d_front > max_dist_ruta:
                    continue
            p = np.array([fila.bx + dx, fila.by + dy], dtype=float)
            pie, d = _pie_perpendicular(
                p, pts, cerrada_de.get(fila.camara, False))
            texto = f"v{int(v)} · d calculada = {d:.1f} px (sin signo)"
            if not pd.isna(fila.dist):
                texto += f"<br>dist_derrape del bag = {fila.dist:.1f} px"
            px += [p[0], pie[0], None]
            py += [p[1], pie[1], None]
            hovers += [texto, texto, ""]
        fig.add_trace(go.Scatter(
            x=px, y=py, mode="lines+markers", name="Dist. derrape",
            line=dict(color=COL_CRITICO, width=1.5),
            marker=dict(size=4, color=COL_CRITICO), showlegend=False,
            customdata=hovers, hovertemplate="%{customdata}<extra></extra>",
            visible=visible,
        ))

        # Flecha del sentido de la marcha, a la altura de la estrella de inicio
        # pero apartada hacia fuera del circuito (encima del trazado se pierde
        # entre los marcadores de las pegatinas). Va sin checkbox (como la
        # estrella) y siempre ocupa su hueco del bloque: si la vuelta no da
        # tangente, entra una traza vacía.
        tangente = _tangente_meta(fr, camaras, centro_plano, separacion)
        if tangente is None:
            fig.add_trace(go.Scatter(x=[], y=[], mode="markers",
                                     showlegend=False, hoverinfo="skip"))
        else:
            traza = _flecha_sentido_2d(
                tangente, visible=visible, centro=centro_plano)
            fig.add_trace(traza)
            puntos_flecha.append(_puntos_con_holgura(traza, centro_plano))

    # --- Slider instantáneo ------------------------------------------------
    # El paso enciende TODAS las estáticas y las trazas de la vuelta activa:
    # el estado por defecto es "todo visible". Justo después, el JS de los
    # checkboxes re-aplica lo que el usuario tenga marcado (por eso hace falta
    # re-aplicar tras cada cambio de vuelta: el paso reactiva capas ocultas).
    pasos = []
    for k, v in enumerate(vueltas):
        visibles = ([True] * n_estaticas
                    + [False] * (TRAZAS_POR_VUELTA_2 * len(vueltas)))
        for j in range(TRAZAS_POR_VUELTA_2):
            visibles[n_estaticas + TRAZAS_POR_VUELTA_2 * k + j] = True
        pasos.append(dict(label=str(int(v)), method="update",
                          args=[{"visible": visibles}]))

    # Índices de las trazas por vuelta, por categoría (orden del bloque:
    # trasera j=0, delantera j=1, inicio j=2, perpendiculares j=3, flecha de
    # sentido j=4). El inicio de vuelta y la flecha no llevan checkbox (van
    # siempre visibles).
    def idx_por_vuelta(j):
        return [n_estaticas + TRAZAS_POR_VUELTA_2 * k + j
                for k in range(len(vueltas))]

    if marco is None:
        x_min, x_max, y_min, y_max = _rangos_globales(
            df, camaras, extra=list(base_por_camara.values()) + puntos_flecha)
    else:
        x_min, x_max, y_min, y_max = marco
    # El paso sale del propio marco, así que la 4 (que usa el mismo marco)
    # calcula exactamente el mismo y las dos rejillas coinciden
    paso_grid = paso_rejilla(x_min, x_max, y_min, y_max)
    fig.update_layout(
        sliders=[dict(active=0, currentvalue=dict(prefix="Vuelta "),
                      pad=dict(t=30), steps=pasos)],
        # Los índices de traza que necesita el JS de los checkboxes. Van en
        # layout.meta, el sitio de plotly para datos propios que viajan con la
        # figura hasta el HTML. Cada categoría lleva su lista por vuelta (o el
        # índice de la estática, en el caso de la base) y su fantasma de
        # leyenda, para poder apagar/encender capa + leyenda a la vez.
        meta=dict(
            idx_base=idx_base,                        # estática única
            idx_fantasma_base=idx_fantasma_base,
            idx_fantasma_delantera=idx_fantasma_delantera,
            idx_fantasma_trasera=idx_fantasma_trasera,
            idx_fantasma_perp=idx_fantasma_perp,
            idx_trasera=idx_por_vuelta(0),
            idx_delantera=idx_por_vuelta(1),
            idx_perp=idx_por_vuelta(3),
        ),
    )
    # Coordenadas de imagen: Y invertida (rango de mayor a menor), escala 1:1
    # y rangos FIJOS para que todas las vueltas se dibujen a la misma escala.
    #
    # `dtick` igual en los dos ejes (ver paso_rejilla) es lo que hace que la
    # CUADRÍCULA sea de celdas cuadradas: con la escala 1:1 ya atada por el
    # scaleanchor, en cuanto los dos ejes se parten cada N píxeles el salto mide
    # lo mismo a lo ancho que a lo alto. `tick0=0` además cuadra la rejilla de
    # esta figura con la de la 4.
    #
    # Sin `constrain`, plotly resuelve la escala 1:1 ENSANCHANDO el rango del
    # eje que sobre hasta llenar el ancho que le dé el navegador: el encuadre
    # pedido nunca se recorta (solo se ve algo más de circuito a los lados) y la
    # figura ocupa todo el ancho disponible, que es lo que interesa para
    # mirarla. Las vueltas siguen siendo comparables porque la escala no cambia.
    fig.update_yaxes(range=[y_max, y_min], scaleanchor="x", scaleratio=1,
                     dtick=paso_grid, tick0=0,
                     title_text="y (px)")
    fig.update_xaxes(range=[x_min, x_max], dtick=paso_grid, tick0=0,
                     title_text="x (px)")
    return _layout_base(
        fig,
        "2 · Trayectorias de la etiqueta delantera y trasera "
        "(una vuelta cada vez, ejes fijos y rejilla cuadrada)",
        680,
    )
