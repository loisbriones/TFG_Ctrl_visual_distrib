#!/usr/bin/env python3
"""
Todas las figuras plotly del dashboard. Una funcion por figura,
grafica_<n>_<nombre>, donde <n> es el numero de su seccion. Todas comparten
paleta y aspecto a traves de _layout_base
"""

import math

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from parseo_log import celdas_de_zona, enlazar_zonas_entre_camaras

# ---------------------------------------------------------------------------
# Colores
# ---------------------------------------------------------------------------
COL_SUPERFICIE = "#fcfcfb"   # fondo de las graficas y de la pagina
COL_TINTA = "#0b0b0b"        # texto principal
COL_TINTA_2 = "#52514e"      # texto secundario (explicaciones)
COL_MUTED = "#898781"        # ejes, etiquetas apagadas, marcadores neutros
COL_GRID = "#e1e0d9"         # rejilla fina
COL_SERIE_1 = "#2a78d6"      # azul, serie categorica 1 (dist, tiempos...)
COL_SERIE_2 = "#1baf7a"      # aqua, serie categorica 2 (segunda serie PWM)
COL_CRITICO = "#d03b3b"      # rojo estado "critical": eventos de derrape
COL_SERIO = "#ec835a"        # naranja estado "serious": zonas de derrape
COL_AVISO = "#b58324"        # ambar: celdas gigantes y derrames entre camaras
COL_BUENO = "#0ca30c"        # verde estado "good": vuelta con subida de perfil
COL_CONTEXTO = "#c3c2b7"     # gris de las series de fondo/contexto

# Tipografia de todo el HTML y las figuras
FUENTE = 'system-ui, -apple-system, "Segoe UI", sans-serif'


# ---------------------------------------------------------------------------
# Plano global multicamara: cada camara tiene su propio sistema de pixeles
# ---------------------------------------------------------------------------
OFFSETS_CAMARAS = {
    "camara_01": (0.0, 0.0),
    "camara_02": (660.0, 0.0),
}
# Distancia entre camaras cuando no pertenecen al OFFSETS_CAMARAS: 640 px de imagen + 20 de margen visual
ANCHO_PLAZA_CAMARA = 660.0


def offset_camara(camara, indice):
    """(dx, dy) de una camara en el plano global. Si no esta en el dict se desplaza hacia un lado"""
    if camara in OFFSETS_CAMARAS:
        return OFFSETS_CAMARAS[camara]
    return (indice * ANCHO_PLAZA_CAMARA, 0.0)


# Cuanto se avanza desde la meta para sacar la direccion de salida, en pixeles
DIST_TANGENTE_SENTIDO = 40.0
# Tope de muestras que se miran buscando esos 40 px
MAX_MUESTRAS_TANGENTE = 12
# Cuanto se aparta la flecha del trazado
SEPARACION_FLECHA_SENTIDO = 0.07

def _tangente_meta(fr, camaras, centro=None, separacion=0.0):
    """(p0, p1) del plano global: el primer punto de la vuelta y el primero que
    se aleja de el DIST_TANGENTE_SENTIDO pixeles, que es donde va la flecha. Con
    centro y separacion el par se aparta en perpendicular a la marcha, hacia
    el lado contrario al centro. Devuelve None si no hay dos puntos utilizables"""
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
        return None  # el coche estaba parado en meta: no hay direccion
    if centro is not None and separacion:
        u = (p1 - p0) / np.linalg.norm(p1 - p0)
        n = np.array([-u[1], u[0]])           # perpendicular a la marcha
        medio = (p0 + p1) / 2
        if np.dot(n, medio - np.asarray(centro, dtype=float)) < 0:
            n = -n                            # el lado que se aleja del centro
        p0, p1 = p0 + separacion * n, p1 + separacion * n
    return p0, p1


def _posicion_etiqueta_flecha(p, centro):
    """Donde colocar el texto de la flecha, siempre hacia el lado contrario al
    centro del circuito"""
    if centro is None:
        return "middle right"
    d = np.asarray(p, dtype=float) - np.asarray(centro, dtype=float)
    if abs(d[0]) >= abs(d[1]):
        return "middle right" if d[0] > 0 else "middle left"
    return "bottom center" if d[1] > 0 else "top center"


def _puntos_con_holgura(traza, centro, holgura=1.15):
    """Los puntos (Nx2) de una traza de flecha mas una copia un poco mas lejos
    del centro, con la que se acotan los ejes de las figuras de rango fijo"""
    pts = np.column_stack([traza.x, traza.y]).astype(float)
    c = np.asarray(centro, dtype=float)
    return np.vstack([pts, c + (pts - c) * holgura])


def _flecha_sentido_2d(tangente, visible=True, centro=None):
    """Traza de la flecha del sentido de la marcha para las figuras 2D: los dos
    puntos de la tangente con angleref="previous", que orienta la punta en
    coordenadas de pantalla. El primero solo da el angulo y va con opacidad 0"""
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
        # La etiqueta cae fuera del trazado y sin esto se recortaria
        cliponaxis=False,
        hovertemplate="sentido de la marcha<extra></extra>",
        visible=visible,
    )


def _flecha_sentido_3d(tangente, largo, visible=True, centro=None):
    """La misma flecha para la figura 1, como una linea en el plano z=0"""
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
        text=["sentido de la marcha", "", "", "", ""],
        textposition=_posicion_etiqueta_flecha(p0, centro),
        textfont=dict(color=COL_INICIO_VUELTA, size=11),
        hovertemplate="sentido de la marcha<extra></extra>",
        visible=visible,
    )


# ---------------------------------------------------------------------------
# Aspecto comun de las figuras
# ---------------------------------------------------------------------------
def _layout_base(fig, titulo, alto):
    """Aspecto comun"""
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


# ---------------------------------------------------------------------------
# Figura 3: la distancia de derrape del bag, vuelta a vuelta
# ---------------------------------------------------------------------------
COLORES_CAMARA = [COL_SERIE_1, COL_SERIE_2, "#7b5ea7", COL_AVISO]


def _color_por_camara(camaras):
    return {cam: COLORES_CAMARA[i % len(COLORES_CAMARA)]
            for i, cam in enumerate(sorted(camaras))}


def grafica_3_derrape_bag(df, umbral):
    """El dist_derrape muestra a muestra dentro de una vuelta, con el PWM
    aplicado en el eje derecho y una vertical en cada cambio de camara"""
    vueltas = sorted(df["vuelta"].unique())
    # Una telemetria que no se pudo emparejar se queda sin camara (None)
    camaras = sorted(c for c in df["camara"].unique() if isinstance(c, str))
    color_cam = _color_por_camara(camaras)
    # Rango del eje Y fijo para todas las vueltas, para poder compararlas
    y_max = max(float(df["dist"].max()) * 1.05, umbral * 1.5)

    fig = go.Figure()

    fig.add_trace(go.Scatter(
        x=df["t_vuelta"], y=df["dist"], mode="markers",
        name="todas las vueltas", marker=dict(size=3, color=COL_CONTEXTO),
        opacity=0.45, hoverinfo="skip",
    ))
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
        etiqueta = f"vuelta {int(v)} · máx {fr['dist'].max():.0f} px"
        if pwms:
            etiqueta += (f" · pwm {int(min(pwms))}" if min(pwms) == max(pwms)
                         else f" · pwm {int(min(pwms))}-{int(max(pwms))}")
        if n_sobre:
            etiqueta += f" · {n_sobre} sobre el umbral"
        fig.add_trace(go.Scatter(
            x=fr["t_vuelta"], y=fr["dist"], mode="lines+markers", name=etiqueta,
            line=dict(color=COL_MUTED, width=1),
            marker=dict(size=6, color=[color_cam.get(c, COL_MUTED) for c in fr["camara"]]),
            customdata=[[f.camara or "?", f.bx, f.by, f.pwm] for f in fr.itertuples()],
            hovertemplate=(
                "t=%{x:.2f} s · dist=%{y:.1f} px<br>%{customdata[0]}"
                "<br>trasera=(%{customdata[1]:.0f}, %{customdata[2]:.0f})"
                "<br>pwm=%{customdata[3]:.0f}<extra></extra>"
            ),
            visible=visible,
        ))
        # Muestras en las que el controlador tenia un derrape abierto
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
        # Cambios de camara: vertical en el punto medio entre las dos muestras
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

    # Slider: las estaticas siempre visibles + las 4 trazas de la vuelta
    # (distancia, derrapes, PWM y cambios de camara)
    pasos = []
    for k, v in enumerate(vueltas):
        visibles = [True] * n_estaticas + [False] * (4 * len(vueltas))
        for j in range(4):
            visibles[n_estaticas + 4 * k + j] = True
        pasos.append(dict(label=str(int(v)), method="update",
                          args=[{"visible": visibles}]))
    # Rango del eje de PWM: el de toda la carrera, tambien fijo
    pwms = [p for p in df["pwm"] if p is not None and not pd.isna(p)]
    rango_pwm = ([min(pwms) - 2, max(pwms) + 2] if pwms else None)

    # Los ejes se fijan por update_layout: update_xaxes/update_yaxes aplican a
    # todos los ejes y le pondrian al del PWM el rango de la distancia
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
        "3 · Distancia de derrape grabada en el bag, vuelta a vuelta "
        "(color = cámara; línea de puntos = cambio de cámara)",
        560,
    )


# ---------------------------------------------------------------------------
# Figura 3 resumen por vuelta la distancia de derrape
# ---------------------------------------------------------------------------
def grafica_3_resumen_derrape(tabla, umbral):
    """La misma serie resumida a un valor por vuelta (maximo, percentil 95 y
    mediana), con el PWM medio en el eje derecho"""
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
        fig, "3 · Resumen por vuelta de la distancia de derrape del bag", 440)


# ---------------------------------------------------------------------------
# Figura 5: tiempo por vuelta
# ---------------------------------------------------------------------------
def grafica_5_tiempos(vueltas, mediana):
    """El tiempo de cada vuelta y la mediana de la carrera"""
    fig = go.Figure(go.Scatter(
        x=[v["numero"] for v in vueltas], y=[v["tiempo"] for v in vueltas],
        mode="lines+markers", line=dict(color=COL_SERIE_1, width=2),
        marker=dict(size=6), hovertemplate="v%{x}: %{y:.3f} s<extra></extra>",
    ))
    fig.add_hline(y=mediana, line=dict(color=COL_GRID, width=1, dash="dash"),
                  annotation_text=f"mediana {mediana:.2f} s",
                  annotation_font_color=COL_TINTA_2)
    fig.update_xaxes(title_text="vuelta", dtick=5)
    # Sin rangemode="tozero": desde 0 la linea sale plana
    fig.update_yaxes(title_text="tiempo (s)")
    _layout_base(fig, "5 · Tendencia del tiempo por vuelta", 420)
    fig.update_layout(margin=dict(l=60, r=20, t=60, b=50))
    return fig


# ---------------------------------------------------------------------------
# Figura 5 (cajas): la comparativa derrape <-> tiempo por vuelta
# ---------------------------------------------------------------------------
def grafica_5_cajas_derrape(df_tel, tiempos_por_vuelta, umbral):
    """Una caja por vuelta con todas sus muestras de dist_derrape y, en el eje
    derecho, el tiempo que se tardo en esa vuelta"""
    datos = df_tel[df_tel["vuelta"].isin(tiempos_por_vuelta)]
    fig = go.Figure()
    fig.add_trace(go.Box(
        x=datos["vuelta"], y=datos["dist"], name="dist_derrape",
        boxpoints="outliers", showlegend=False,
        line=dict(color=COL_SERIE_1, width=1.5),
        fillcolor="rgba(42, 120, 214, 0.15)",
        # Con boxpoints="outliers" los unicos puntos dibujados son los atipicos,
        # asi que su color es marker.color: marker.outliercolor solo lo mira
        # boxpoints="suspectedoutliers"
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
    # Ejes con nombre, mismo motivo que en la grafica 3
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
        fig, "5 · Distribución del derrape y tiempo, vuelta a vuelta "
             "(caja = cuartiles; puntos rojos = valores atípicos)", 460)


# ---------------------------------------------------------------------------
# Figura 6: resumen por vuelta (4 paneles)
# ---------------------------------------------------------------------------
def grafica_6_resumen(tabla: pd.DataFrame):
    """Cuatro paneles por vuelta: derrapes, tiempo (marcador verde si al
    cerrarla subio el perfil), PWM medio y velocidad media"""
    fig = make_subplots(
        rows=2, cols=2, vertical_spacing=0.16, horizontal_spacing=0.10,
        subplot_titles=[
            "derrapes por vuelta", "tiempo por vuelta (s)",
            "PWM medio", "velocidad media (px/s)",
        ],
    )
    fig.add_trace(
        # Solo se listan en la leyenda las dos series de PWM
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
    fig = _layout_base(fig, "6 · Resumen por vuelta", 640)
    fig.update_layout(legend=dict(x=0.5, xanchor="center"))
    return fig


# Los paneles de la 6 que se pueden sacar sueltos en PDF
PANELES_6 = ("derrapes", "tiempo", "pwm", "velocidad")


def grafica_6_subplot(tabla: pd.DataFrame, panel: str):
    """Uno de los paneles de la 6 como figura suelta, con las mismas trazas"""
    if panel not in PANELES_6:
        raise ValueError(f"panel debe ser uno de {PANELES_6}, no {panel!r}")
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
    return _layout_base(fig, f"6 · {titulo}", 420)


# ---------------------------------------------------------------------------
# Figura 4: el circuito con el PWM de cada celda
# ---------------------------------------------------------------------------
# Colores de zona. Se reparten: el primero libre al nacer una zona, y vuelve a
# la reserva cuando muere
PALETA_ZONAS = [
    "#2a78d6",  # azul
    "#d03b3b",  # rojo
    "#1baf7a",  # aqua
    "#7b5ea7",  # violeta
    "#0ca30c",  # verde
    "#ec835a",  # naranja
    "#00838f",  # teal oscuro
    "#c2185b",  # magenta oscuro
    "#8d6e63",  # marron
    "#5c6bc0",  # indigo
    "#9e9d24",  # oliva
    "#e91e63",  # rosa fuerte
    "#009688",  # verde azulado
    "#795548",  # marron oscuro
    "#3f51b5",  # azul oscuro
    "#ff7043",  # coral
    "#607d8b",  # gris azulado
]

# Formas del marcador, que acompañan al color y dicen lo mismo que el
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
    """Reparte estilos entre las zonas vivas y recoge el de las que mueren.
    Cada zona ocupa un hueco mientras vive, y del numero de hueco salen su color
    (PALETA_ZONAS) y su forma (SIMBOLOS_ZONA). La clave es normalmente
    (camara, id), pero dos mitades de la misma zona comparten clave y estilo"""

    def __init__(self):
        self.de_zona = {}     # clave de estilo -> numero de hueco
        self.aviso_dado = False

    def liberar_muertas(self, vivas):
        for clave in [k for k in self.de_zona if k not in vivas]:
            del self.de_zona[clave]

    def estilo(self, clave):
        """(color, forma) de la zona. Se le asigna el primer hueco libre la
        primera vez que aparece"""
        if clave not in self.de_zona:
            usados = set(self.de_zona.values())
            libres = [i for i in range(len(PALETA_ZONAS)) if i not in usados]
            if libres:
                self.de_zona[clave] = libres[0]
            else:
                # Mas zonas vivas que huecos: se reparte por modulo y dos zonas
                # repiten estilo
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
    """Cada punto del color y la forma de su zona. El slider pinta el perfil con el que se corrio cada vuelta"""
    camaras = sorted(datos_camaras)
    indice = {c: i for i, c in enumerate(camaras)}
    # Eje de vueltas del slider la union de las vueltas de todas las camaras
    vueltas_global = sorted({v for d in datos_camaras.values() for v in d["vueltas"]})

    # Zonas que son la misma zona de derrape partida entre dos camaras, que comparten hueco de estilo
    grupo_de = enlazar_zonas_entre_camaras(datos_camaras)

    def clave_estilo(cam, id_zona):
        """La clave del grupo si la zona esta enlazada con otra camara, y si no
        la suya propia"""
        return grupo_de.get((cam, id_zona), (cam, id_zona))

    fig = go.Figure()
    n_estaticas = 0  
    # Todos los puntos del trazado, con los que salen el centro del circuito y su tamaño
    trazado_x, trazado_y = [], []

    for i, cam in enumerate(camaras):
        dx, dy = offset_camara(cam, i)
        celdas = datos_camaras[cam]["c"].celdas.sort_values("celda").reset_index(drop=True)
        tx, ty = [], []
        gx, gy, gt = [], [], []
        for j, fila in celdas.iterrows():
            if fila["gigante"]:
                # La trayectoria se corta en la gigante y el hueco va aparte,
                # como linea discontinua entre sus vecinas
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
        # En cadena cerrada se une la ultima celda con la primera
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

    # {celda: (x, y)} por camara, que lo reutilizan todas las vueltas
    puntos_celda = {}
    for i, cam in enumerate(camaras):
        dx, dy = offset_camara(cam, i)
        normales = datos_camaras[cam]["c"].celdas
        normales = normales[~normales["gigante"]]
        puntos_celda[cam] = {int(cd["celda"]): (cd["x"] + dx, cd["y"] + dy)
                             for _, cd in normales.iterrows()}

    # --- Un bloque de trazas por vuelta: una traza por zona ----------------
    # El numero de trazas cambia de una vuelta a otra, por eso se apuntan los
    # indices para armar luego el slider
    reserva = _ReservaEstilos()
    varias_camaras = len(camaras) > 1
    indices_por_vuelta = {}
    for v in vueltas_global:
        # Zonas vigentes en esta vuelta. Si una camara no tiene datos de la
        # vuelta v se usa la ultima anterior que si tenga
        zonas_vuelta = {}   # cam -> [zona]
        for cam in camaras:
            d = datos_camaras[cam]
            v_datos = max((u for u in d["vueltas"] if u <= v), default=None)
            if v_datos is not None:
                zonas_vuelta[cam] = d["zonas_ini_vuelta"].get(v_datos, [])
        # Primero se devuelven a la reserva los huecos de las zonas que ya no
        # estan, y luego se reparten los que faltan. Se libera por clave de
        # estilo, la misma con la que luego se pide
        vivas = {clave_estilo(cam, z["id"])
                 for cam, zs in zonas_vuelta.items() for z in zs}
        reserva.liberar_muertas(vivas)

        indices = []
        # Por camara y por celda de inicio, con las mitades de una zona partida
        # juntas en la posicion de la primera
        orden_natural = sorted(
            ((cam, z) for cam, zs in zonas_vuelta.items() for z in zs),
            key=lambda par: (indice[par[0]], par[1]["ini"]))
        primer_puesto = {}
        for puesto, (cam, z) in enumerate(orden_natural):
            primer_puesto.setdefault(clave_estilo(cam, z["id"]), puesto)
        ordenadas = sorted(
            orden_natural,
            key=lambda par: (primer_puesto[clave_estilo(par[0], par[1]["id"])],
                             indice[par[0]], par[1]["ini"]))
        # Con quien esta enlazada cada zona, para decirlo en la leyenda
        companeras = {}
        for cam, z in orden_natural:
            companeras.setdefault(clave_estilo(cam, z["id"]), []).append(
                (cam, z["id"]))
        con_zona = {cam: set() for cam in camaras}
        for cam, z in ordenadas:
            xs, ys, hover = [], [], []
            # celdas_de_zona, y no range(ini, fin+1), porque una zona puede
            # envolver la meta en cadena cerrada (fin < ini): ver
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
            marca = ("" if delta == 0 else
                     " (nueva)" if delta is None else f" ({delta:+d})")
            # La camara solo se nombra si hay mas de una, que los ids de zona
            # son por camara: la Z3 de la 01 no es la de la 02
            prefijo = f"cam{cam.split('_')[-1]} · " if varias_camaras else ""
            # La que envuelve la meta va avisada: si no, un 69-46 parece un
            # error
            meta = " (cruza meta)" if z.get("cruza_meta") else ""
            zona_txt = (f"Z{z['id']} · {z['ini']}-{z['fin']}{meta} "
                        f"· PWM {z['pwm']}")
            clave = clave_estilo(cam, z["id"])
            # Con quien comparte estilo, si la zona esta partida entre dos
            # camaras. Cada mitad conserva su rango y su PWM
            otras = [f"cam{c.split('_')[-1]} Z{i}"
                     for c, i in companeras.get(clave, [])
                     if (c, i) != (cam, z["id"])]
            enlace = f" ↔ {', '.join(otras)}" if otras else ""
            color, simbolo = reserva.estilo(clave)
            fig.add_trace(go.Scatter(
                x=xs, y=ys, mode="markers",
                name=prefijo + zona_txt + marca + enlace,
                # Las mitades de una zona partida se apagan juntas
                legendgroup=str(clave),
                marker=dict(size=11, symbol=simbolo, color=color,
                            line=dict(color=COL_SUPERFICIE, width=1)),
                customdata=[f"{h}<br>{zona_txt} ({z['tipo']})<br>{cambio}"
                            + (f"<br>misma zona que {', '.join(otras)}"
                               if otras else "")
                            for h in hover],
                hovertemplate="%{customdata}<extra></extra>",
                visible=(v == vueltas_global[0]),
            ))
            indices.append(len(fig.data) - 1)

        # Celdas que ninguna zona cubre, en gris y todas en una traza
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

        # Inicio de meta: el primer punto capturado de la vuelta en el bag. La
        # numeracion del log y la del bag pueden no coincidir, asi que si esa
        # vuelta no esta se cae a la primera posicion
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

            # Y a su altura la flecha de sentido. Aqui el bloque por vuelta no
            # es de tamaño fijo (los indices se apuntan uno a uno), asi que si
            # no hay tangente basta con no añadirla
            tangente = _tangente_meta(
                fr if len(fr) else df_pos, camaras, centro_trazado,
                separacion_flecha)
            if tangente is not None:
                fig.add_trace(_flecha_sentido_2d(
                    tangente, visible=(v == vueltas_global[0]),
                    centro=centro_trazado))
                indices.append(len(fig.data) - 1)
        indices_por_vuelta[v] = indices

    # Slider instantaneo: fondos siempre visibles + las trazas de su vuelta
    n_total = len(fig.data)
    pasos = []
    for v in vueltas_global:
        visibles = [False] * n_total
        for i in range(n_estaticas):
            visibles[i] = True
        for idx in indices_por_vuelta[v]:
            visibles[idx] = True
        # int(v): la etiqueta viaja al servidor para sacar el PDF de esa vuelta
        # y con un float de numpy saldria "3.0"
        pasos.append(dict(label=str(int(v)), method="update",
                          args=[{"visible": visibles}]))
    fig.update_layout(
        sliders=[dict(active=0, currentvalue=dict(prefix="Vuelta "),
                      pad=dict(t=30), steps=pasos)],
    )
    # Coordenadas de imagen, Y invertida y misma escala en los dos ejes
    if marco is not None:
        mx0, mx1, my0, my1 = marco
        paso_grid = paso_rejilla(mx0, mx1, my0, my1)
        fig.update_yaxes(range=[my1, my0], scaleanchor="x", scaleratio=1,
                         dtick=paso_grid, tick0=0,
                         title_text="y (px)")
        fig.update_xaxes(range=[mx0, mx1], dtick=paso_grid, tick0=0,
                         title_text="x (px)")
    else:
        # Sin marco los ejes se autoescalan, el paso sale de la caja del trazado
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
    # Leyenda vertical a la derecha, despues de _layout_base, que pone la
    # horizontal comun. El margen derecho crece para dejarle sitio
    fig.update_layout(
        legend=dict(orientation="v", yanchor="top", y=1, xanchor="left",
                    x=1.02, font=dict(size=11)),
        margin=dict(l=70, r=210, t=95, b=60),
    )
    return fig


# ---------------------------------------------------------------------------
# Colores de las trayectorias de las pegatinas
# ---------------------------------------------------------------------------
COL_ETIQ_DELANTERA = "#009E73"  # verde etiqueta delantera
COL_ETIQ_TRASERA = "#CC00CC"    # magenta etiqueta trasera
COL_TRAY_BASE_3D = "#9400D3"    # violeta de la linea de posicion en 3D
COL_TRAY_BASE = COL_SERIE_1     # azul de la trayectoria base aprendida
COL_INICIO_VUELTA = COL_AVISO   # ambar del arranque de la vuelta

# Salto en pixeles entre dos posiciones consecutivas a partir del cual la linea
# de trayectoria se corta (None) en vez de unirlas
SALTO_CORTE_TRAZO = 100.0

# Altura de la caja 3D de la figura 1 respecto al lado mayor del circuito. El eje z se exagera hasta esta fraccion
ALTURA_RELATIVA_Z = 0.35


def _con_cortes(df, col_x, col_y, camaras):
    """Listas x, y para una traza de linea con None donde no hay que unir. 
    Cambio de camara o salto > SALTO_CORTE_TRAZO px. Aplica el offset del plano
    global de cada camara y df tiene que venir ordenado por tiempo"""
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
    Recorre las dos pegatinas de todas las camaras aplicando su offset, mas los
    arrays extra (Nx2, ya en coordenadas globales) que haya que acotar"""
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


# Cuanto se infla el marco por cada lado, en fraccion del lado mayor de la caja
# de los datos, para dejar sitio a la flecha de sentido y a su etiqueta
HOLGURA_MARCO = 0.12


def marco_comun(df, camaras, extras=()):
    """(x_min, x_max, y_min, y_max) del plano global con holgura, el encuadre
    comun de las graficas 2 y 4. Las dos enseñan el mismo circuito y con el
    mismo marco sale en la misma posicion y a la misma escala"""
    x0, x1, y0, y1 = _rangos_globales(df, camaras, extra=extras)
    holgura = max(x1 - x0, y1 - y0) * HOLGURA_MARCO
    return x0 - holgura, x1 + holgura, y0 - holgura, y1 + holgura


def paso_rejilla(x0, x1, y0, y1):
    """Separacion entre lineas de la rejilla (px del plano global), la misma
    para el eje X y el eje Y.
    Se apunta a una decena de divisiones en el lado mayor y se redondea a 1, 2 o 5
    por potencia de diez"""
    objetivo = max(x1 - x0, y1 - y0) / 10 or 1.0
    potencia = 10 ** math.floor(math.log10(objetivo))
    for redondo in (1, 2, 5, 10):
        if objetivo <= redondo * potencia:
            return redondo * potencia
    return 10 * potencia


def _pie_perpendicular(p, pts, cerrada=False):
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


# ---------------------------------------------------------------------------
# Figura 1: evolucion 3D del derrape sobre la trayectoria
# ---------------------------------------------------------------------------
def grafica_1_derrape3d(df):
    """El plano global en XY, con la trayectoria de la delantera de linea base
    en z=0, y en el eje Z y en el color el derrape de la trasera. Los puntos sin
    dist emparejada no se dibujan"""
    camaras = sorted(df["camara"].unique())
    vueltas = sorted(df.loc[df["vuelta"].notna(), "vuelta"].unique())
    indice = {c: i for i, c in enumerate(camaras)}

    fig = go.Figure()

    # Linea base: la delantera durante una vuelta completa, la primera con
    # datos. Sin vueltas cronometradas se usa todo el recorrido
    base = df[df["vuelta"] == vueltas[0]] if vueltas else df
    bx, by = _con_cortes(base, "fx", "fy", camaras)
    fig.add_trace(go.Scatter3d(
        x=bx, y=by, z=[0] * len(bx), mode="lines", name="Posición",
        line=dict(color=COL_TRAY_BASE_3D, width=3), hoverinfo="skip",
    ))

    # Rangos del plano, aqui solo para dimensionar y colocar la flecha de
    # sentido, los definitivos se recalculan al final con las flechas dentro
    gx_min, gx_max, gy_min, gy_max = _rangos_globales(df, camaras)
    mayor_plano = max(gx_max - gx_min, gy_max - gy_min)
    largo_flecha = mayor_plano * 0.03
    centro_plano = ((gx_min + gx_max) / 2, (gy_min + gy_max) / 2)
    puntos_flecha = []

    # Un bloque de trazas por vuelta: los puntos de derrape, la estrella de
    # inicio de meta (el primer punto capturado de la vuelta, en z=0) y la
    # flecha de sentido. Son 3 trazas y el slider las enciende juntas
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
                colorscale="Inferno", cmin=0, cmax=z_max,
                colorbar=dict(title="Dist<br>derrape"),
            ),
            text=[f"v{int(v)} · {f.camara}<br>dist={f.dist:.1f} px"
                  for f in fr.itertuples()],
            hovertemplate="%{text}<extra></extra>",
            visible=(v == vueltas[0]),
        ))

        # Inicio de meta: el primer punto de la vuelta en todo el df, no solo
        # de los que tienen dist, dibujado a z=0
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

        # Y la flecha de sentido. Si la vuelta no da tangente se mete una traza
        # vacia: el bloque tiene que medir siempre TRAZAS_POR_VUELTA_1, que el
        # slider direcciona las trazas por indice
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

    # Slider: cada paso enseña solo su vuelta, con la traza 0 (la linea base)
    # siempre visible
    pasos = []
    for k, v in enumerate(vueltas):
        visibles = [True] + [False] * (TRAZAS_POR_VUELTA_1 * len(vueltas))
        for j in range(TRAZAS_POR_VUELTA_1):
            visibles[1 + TRAZAS_POR_VUELTA_1 * k + j] = True
        pasos.append(dict(label=str(int(v)), method="update",
                          args=[{"visible": visibles}]))

    # Rangos fijos de la caja, ya con las flechas dentro, para que el encuadre
    # no cambie al pasar de vuelta
    gx_min, gx_max, gy_min, gy_max = _rangos_globales(
        df, camaras, extra=puntos_flecha)

    # Proporcion de la caja 3D: x e y guardan la proporcion real del plano y z
    # se lleva ALTURA_RELATIVA_Z del lado mayor
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
            # Rangos fijos, con la Y invertida (de mayor a menor) para ver el
            # circuito como en la imagen de la camara
            xaxis=dict(range=[gx_min, gx_max], nticks=8),
            yaxis=dict(range=[gy_max, gy_min], nticks=5),
            zaxis=dict(range=[0, z_max * 1.05]),
            aspectmode="manual",
            aspectratio=dict(x=rango_x / mayor, y=rango_y / mayor,
                             z=ALTURA_RELATIVA_Z),
            # Encuadre de partida, el que sale tambien en el PDF. Cuanto mas
            # pequeño el vector eye, mas cerca queda la camara
            camera=dict(eye=dict(x=1.0, y=1.0, z=0.6)),
        ),
    )
    fig = _layout_base(
        fig,
        "1 · Valor de derrape en cada punto del recorrido "
        "(una vuelta cada vez, ejes fijos; arrastrar para rotar)",
        720,
    )
    # Los margenes de _layout_base son para 2D y en 3D la escena sale pequeña
    fig.update_layout(margin=dict(l=10, r=10, t=70, b=10))
    return fig


# ---------------------------------------------------------------------------
# Figura 2: trayectorias de la etiqueta delantera y trasera
# ---------------------------------------------------------------------------
# Trazas que lleva cada vuelta, en este orden: trasera, delantera, inicio,
# perpendiculares y flecha de sentido
TRAZAS_POR_VUELTA_2 = 5


def grafica_2_trayectorias(df, celdas_por_camara=None, cerradas=None,
                           umbral=None, max_dist_ruta=None, marco=None):
    """El recorrido de las dos pegatinas sobre la trayectoria base aprendida,
    con las perpendiculares del derrape y una vuelta cada vez. Sin
    celdas_por_camara no hay base ni perpendiculares, los frames con la
    delantera a mas de max_dist_ruta de la ruta se saltan, y umbral ya no
    se usa"""
    camaras = sorted(df["camara"].unique())
    vueltas = sorted(df.loc[df["vuelta"].notna(), "vuelta"].unique())
    indice = {c: i for i, c in enumerate(camaras)}

    # --- Trayectoria base aprendida (celdas del log) en el plano global ----
    # Sirve para dibujarla y para calcular las perpendiculares. Las celdas
    # gigantes no tienen punto (x/y NaN)
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

    # --- Estaticas: fantasmas de leyenda + la trayectoria base -------------
    # Las trazas de cada vuelta van con showlegend=False y la leyenda la
    # sostienen estos fantasmas
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
    # Indices de los fantasmas, en el orden de la lista de arriba, para que
    # cada checkbox apague tambien su entrada de leyenda
    idx_fantasma_trasera = 0
    idx_fantasma_delantera = 1
    idx_fantasma_perp = 3 if hay_base else -1
    idx_fantasma_base = len(fantasmas) - 1 if hay_base else -1

    if hay_base:
        # La linea se corta en las celdas gigantes, que ahi no hay puntos
        # aprendidos, y entre camaras
        bxs, bys = [], []
        for cam in camaras:
            if cam not in base_por_camara:
                continue
            if bxs:  # None entre camaras: no se unen dos porciones distintas
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
            visible=True,
        ))
    idx_base = len(fantasmas) if hay_base else -1
    n_estaticas = len(fantasmas) + (1 if hay_base else 0)

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
                marker=dict(size=8, color=color,
                            line=dict(color=COL_SUPERFICIE, width=1)),
                showlegend=False,
                hovertemplate=f"{nombre} · v{int(v)}<br>" +
                              "(%{x:.0f}, %{y:.0f})<extra></extra>",
                visible=visible,
            ))

        # Inicio de la vuelta el primer punto que llego tras cruzar meta
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

        # Perpendiculares de la trasera a la trayectoria base un segmento por
        # punto, todos en una traza separados por None
        px, py, hovers = [], [], []
        for fila in fr.itertuples():
            pts = base_por_camara.get(fila.camara)
            if pts is None:
                continue
            dx, dy = offset_camara(fila.camara, indice[fila.camara])
            # Mismo filtro que el algoritmo, si la delantera esta a mas de
            # max_dist_ruta de la ruta, el frame entero se descarta y nunca
            # llega a medirse el derrape
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

        tangente = _tangente_meta(fr, camaras, centro_plano, separacion)
        if tangente is None:
            fig.add_trace(go.Scatter(x=[], y=[], mode="markers",
                                     showlegend=False, hoverinfo="skip"))
        else:
            traza = _flecha_sentido_2d(
                tangente, visible=visible, centro=centro_plano)
            fig.add_trace(traza)
            puntos_flecha.append(_puntos_con_holgura(traza, centro_plano))

    # --- Slider instantaneo ------------------------------------------------
    pasos = []
    for k, v in enumerate(vueltas):
        visibles = ([True] * n_estaticas
                    + [False] * (TRAZAS_POR_VUELTA_2 * len(vueltas)))
        for j in range(TRAZAS_POR_VUELTA_2):
            visibles[n_estaticas + TRAZAS_POR_VUELTA_2 * k + j] = True
        pasos.append(dict(label=str(int(v)), method="update",
                          args=[{"visible": visibles}]))

    # Indices de las trazas por vuelta, por categoria y en el orden del bloque:
    # trasera j=0, delantera j=1, inicio j=2, perpendiculares j=3 y flecha j=4.
    # El inicio y la flecha no llevan checkbox, van siempre visibles
    def idx_por_vuelta(j):
        return [n_estaticas + TRAZAS_POR_VUELTA_2 * k + j
                for k in range(len(vueltas))]

    if marco is None:
        x_min, x_max, y_min, y_max = _rangos_globales(
            df, camaras, extra=list(base_por_camara.values()) + puntos_flecha)
    else:
        x_min, x_max, y_min, y_max = marco
    # El paso sale del propio marco, asi que la 4 calcula el mismo y las dos
    # rejillas coinciden
    paso_grid = paso_rejilla(x_min, x_max, y_min, y_max)
    fig.update_layout(
        sliders=[dict(active=0, currentvalue=dict(prefix="Vuelta "),
                      pad=dict(t=30), steps=pasos)],
        # Los indices de traza que necesita el JS de los checkboxes
        meta=dict(
            idx_base=idx_base,                        # estatica unica
            idx_fantasma_base=idx_fantasma_base,
            idx_fantasma_delantera=idx_fantasma_delantera,
            idx_fantasma_trasera=idx_fantasma_trasera,
            idx_fantasma_perp=idx_fantasma_perp,
            idx_trasera=idx_por_vuelta(0),
            idx_delantera=idx_por_vuelta(1),
            idx_perp=idx_por_vuelta(3),
        ),
    )
    # Coordenadas de imagen: Y invertida, escala 1:1 y rangos fijos para que
    # todas las vueltas salgan igual. El dtick comun hace la cuadricula
    # cuadrada (ver paso_rejilla) y tick0=0 la cuadra con la de la 4
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
